#!/usr/bin/env python3
"""
Betaflight SITL <-> PyBullet Bridge

This bridge connects:
- Betaflight SITL (firmware simulation)
- PyBullet (physics simulation)
- pi_drone_nav MSP driver (control)

Communication:
- UDP 9002: SITL → Bridge (motor values)
- UDP 9003: Bridge → SITL (sensor data)
- TCP 5761: MSP communication (UART1)

The bridge:
1. Receives motor outputs from SITL
2. Applies them in PyBullet
3. Reads IMU/position from PyBullet
4. Sends sensor feedback to SITL
"""

import socket
import struct
import threading
import time
import math
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, List
import pybullet as p

from simulation.pybullet.drone_sim import PyBulletDrone


@dataclass
class FDMPacket:
    """Flight Dynamics Model packet sent TO SITL (port 9003)"""
    timestamp: float = 0.0
    # Angular velocity (rad/s) - body frame
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    # Linear acceleration (m/s²) - body frame
    acc_x: float = 0.0
    acc_y: float = 0.0
    acc_z: float = 0.0
    # Orientation quaternion (w, x, y, z)
    quat_w: float = 1.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    # Velocity (m/s) - earth frame (ENU)
    vel_e: float = 0.0
    vel_n: float = 0.0
    vel_up: float = 0.0
    # Position - for GPS (lon, lat, alt)
    longitude: float = 1.389879  # Default: Toulouse area
    latitude: float = 43.483061
    altitude: float = 0.0
    # Barometric pressure (Pa)
    pressure: float = 101325.0

    def pack(self) -> bytes:
        """Pack into bytes for UDP transmission (fdm_packet: 18 doubles = 144 bytes)"""
        return struct.pack(
            '<18d',
            self.timestamp,
            self.gyro_x, self.gyro_y, self.gyro_z,      # imu_angular_velocity_rpy[3]
            self.acc_x, self.acc_y, self.acc_z,         # imu_linear_acceleration_xyz[3]
            self.quat_w, self.quat_x, self.quat_y, self.quat_z,  # imu_orientation_quat[4]
            self.vel_e, self.vel_n, self.vel_up,        # velocity_xyz[3]
            self.longitude, self.latitude, self.altitude,  # position_xyz[3]
            self.pressure
        )


@dataclass
class ServoPacket:
    """Motor values received FROM SITL (port 9002)"""
    motor_0: float = 0.0
    motor_1: float = 0.0
    motor_2: float = 0.0
    motor_3: float = 0.0

    @classmethod
    def unpack(cls, data: bytes) -> 'ServoPacket':
        """Unpack from bytes"""
        if len(data) >= 16:
            m0, m1, m2, m3 = struct.unpack('<4f', data[:16])
            return cls(motor_0=m0, motor_1=m1, motor_2=m2, motor_3=m3)
        return cls()


@dataclass
class RCPacket:
    """RC channels sent TO SITL (port 9004)"""
    timestamp: float = 0.0
    channels: Tuple[int, ...] = (1500,) * 16  # 16 channels

    def pack(self) -> bytes:
        """Pack into bytes for UDP transmission"""
        return struct.pack(
            '<d16H',  # 1 double + 16 uint16
            self.timestamp,
            *self.channels
        )


class SITLBridge:
    """
    Bridge between Betaflight SITL and PyBullet physics.

    Handles:
    - UDP communication with SITL
    - PyBullet physics simulation
    - Sensor feedback generation
    """

    def __init__(
        self,
        sitl_ip: str = "127.0.0.1",
        motor_port: int = 9002,
        sensor_port: int = 9003,
        rc_port: int = 9004,
        gui: bool = True
    ):
        self.sitl_ip = sitl_ip
        self.motor_port = motor_port
        self.sensor_port = sensor_port
        self.rc_port = rc_port

        # UDP sockets
        self.motor_socket: Optional[socket.socket] = None
        self.sensor_socket: Optional[socket.socket] = None
        self.rc_socket: Optional[socket.socket] = None

        # PyBullet drone
        self.drone = PyBulletDrone(gui=gui, real_time=False)
        self.drone.arm()  # Always armed in simulation

        # State
        self.running = False
        self.motor_values = [0.0, 0.0, 0.0, 0.0]
        self.start_time = time.time()

        # Threading
        self.motor_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        # Statistics
        self.packets_received = 0
        self.packets_sent = 0

        # RC state - default channels (disarmed, ANGLE mode on AUX2)
        # Channels: [Roll, Pitch, Throttle, Yaw, AUX1=ARM, AUX2=ANGLE, AUX3, AUX4, ...]
        self._rc_channels = [1500, 1500, 1000, 1500, 1000, 1800, 1500, 1500] + [1500] * 8
        self._armed = False

    def start(self):
        """Start the bridge"""
        print(f"[Bridge] Starting SITL bridge...")
        print(f"[Bridge] Motor input:  UDP {self.motor_port}")
        print(f"[Bridge] Sensor output: UDP {self.sensor_port} -> {self.sitl_ip}")
        print(f"[Bridge] RC output: UDP {self.rc_port} -> {self.sitl_ip}")

        # Create motor receiver socket (server)
        self.motor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.motor_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.motor_socket.bind(('0.0.0.0', self.motor_port))
        self.motor_socket.settimeout(0.1)

        # Create sensor sender socket (client)
        self.sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Create RC sender socket (client)
        self.rc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.running = True

        # Start motor receiver thread
        self.motor_thread = threading.Thread(target=self._motor_receiver, daemon=True)
        self.motor_thread.start()

        print(f"[Bridge] Bridge started, waiting for SITL...")

    def stop(self):
        """Stop the bridge"""
        self.running = False
        if self.motor_thread:
            self.motor_thread.join(timeout=1.0)
        if self.motor_socket:
            self.motor_socket.close()
        if self.sensor_socket:
            self.sensor_socket.close()
        if self.rc_socket:
            self.rc_socket.close()
        self.drone.close()
        print(f"[Bridge] Bridge stopped")

    def arm(self):
        """Arm the drone (set AUX1 high)"""
        self._armed = True
        self._rc_channels[4] = 1800  # AUX1 = arm
        print("[Bridge] Armed")

    def disarm(self):
        """Disarm the drone (set AUX1 low)"""
        self._armed = False
        self._rc_channels[4] = 1000  # AUX1 = disarm
        self._rc_channels[2] = 1000  # Throttle low
        print("[Bridge] Disarmed")

    def set_throttle(self, throttle: int):
        """Set throttle value (1000-2000)"""
        self._rc_channels[2] = max(1000, min(2000, throttle))

    def set_rc_channel(self, channel: int, value: int):
        """Set individual RC channel (0-15, value 1000-2000)"""
        if 0 <= channel < 16:
            self._rc_channels[channel] = max(1000, min(2000, value))

    def send_rc(self, channels: Optional[List[int]] = None):
        """
        Send RC channels to SITL via UDP.

        Args:
            channels: Optional list of 8-16 RC channel values (1000-2000)
                     If None, uses internal RC state
        """
        if channels is not None:
            # Pad to 16 channels
            padded = list(channels) + [1500] * (16 - len(channels))
            self._rc_channels = padded[:16]

        packet = RCPacket(
            timestamp=time.time() - self.start_time,
            channels=tuple(self._rc_channels)
        )
        try:
            self.rc_socket.sendto(
                packet.pack(),
                (self.sitl_ip, self.rc_port)
            )
        except Exception as e:
            print(f"[Bridge] RC send error: {e}")

    def _motor_receiver(self):
        """Thread to receive motor values from SITL"""
        while self.running:
            try:
                data, addr = self.motor_socket.recvfrom(64)
                if len(data) >= 16:
                    packet = ServoPacket.unpack(data)
                    with self.lock:
                        # SITL motor remapping (sitl.c lines 601-604):
                        # pwmPkt[0] = motorsPwm[1] (FRONT_R)
                        # pwmPkt[1] = motorsPwm[2] (REAR_L)
                        # pwmPkt[2] = motorsPwm[3] (FRONT_L)
                        # pwmPkt[3] = motorsPwm[0] (REAR_R)
                        #
                        # PyBullet motors: [FR, FL, BR, BL]
                        # Betaflight QUAD_X: [REAR_R, FRONT_R, REAR_L, FRONT_L]
                        self.motor_values = [
                            packet.motor_0,  # FRONT_R -> our FR (0)
                            packet.motor_2,  # FRONT_L -> our FL (1)
                            packet.motor_3,  # REAR_R -> our BR (2)
                            packet.motor_1,  # REAR_L -> our BL (3)
                        ]
                    self.packets_received += 1

                    if self.packets_received == 1:
                        print(f"[Bridge] First motor packet from {addr}")

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[Bridge] Motor receive error: {e}")

    def _generate_fdm_packet(self) -> FDMPacket:
        """Generate FDM packet from PyBullet state"""
        state = self.drone.get_state()

        # Get orientation quaternion from PyBullet
        pos, orn = p.getBasePositionAndOrientation(self.drone.drone_id)
        vel, ang_vel = p.getBaseVelocity(self.drone.drone_id)

        # Convert world angular velocity to body frame
        rot_matrix = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        ang_vel_body = rot_matrix.T @ np.array(ang_vel)

        # Calculate body-frame acceleration
        # In hover, acc_z should be ~9.81 (gravity compensation)
        # For simplicity, we estimate from velocity change or use gravity + thrust
        # Here we use a simplified model
        gravity_body = rot_matrix.T @ np.array([0, 0, -9.81])
        acc_body = -gravity_body  # Approximate: in stable flight, acc ≈ -gravity

        # Calculate barometric pressure from altitude
        # Standard atmosphere: P = P0 * (1 - 0.0065*h/288.15)^5.255
        pressure = 101325.0 * (1 - 0.0065 * state.z / 288.15) ** 5.255

        # GPS position (simple: use local coordinates as offset from reference)
        # Reference point: Toulouse
        ref_lat = 43.483061
        ref_lon = 1.389879
        meters_per_deg_lat = 111320
        meters_per_deg_lon = 111320 * math.cos(math.radians(ref_lat))

        # Frame conversion: PyBullet (X fwd, Y left, Z up) -> Betaflight (X fwd, Y right, Z down)
        # PyBullet: X forward, Y left, Z up
        # Betaflight: X forward, Y right, Z down
        # Transform: BF_y = -PB_y, BF_z = -PB_z

        packet = FDMPacket(
            timestamp=time.time() - self.start_time,
            # Gyro (rad/s) - test: roll not inverted, pitch inverted
            gyro_x=ang_vel_body[0],   # Roll raw
            gyro_y=-ang_vel_body[1],  # Pitch inverted
            gyro_z=ang_vel_body[2],   # Yaw raw
            # Acc (m/s²) - SITL applies negation on all axes internally
            # For level drone: PB acc_z = +9.81 (up), BF wants acc_z = +9.81 (down = negative in BF)
            # SITL does -raw, so we need to send -PB values to get correct BF values
            acc_x=acc_body[0],
            acc_y=-acc_body[1],  # Flip Y: left->right
            acc_z=-acc_body[2],  # Flip Z: up->down
            # Quaternion (w, x, y, z) - PyBullet returns (x,y,z,w)
            # For frame conversion: negate y and z components
            quat_w=orn[3],
            quat_x=orn[0],
            quat_y=-orn[1],
            quat_z=-orn[2],
            # Velocity ENU (east, north, up)
            vel_e=state.vx,
            vel_n=state.vy,
            vel_up=state.vz,
            # GPS position
            longitude=ref_lon + state.x / meters_per_deg_lon,
            latitude=ref_lat + state.y / meters_per_deg_lat,
            altitude=state.z,
            # Pressure
            pressure=pressure
        )

        return packet

    def step(self) -> Tuple[float, float, float, float]:
        """
        Run one simulation step.

        Returns:
            Tuple of motor values (0-1 range)
        """
        # Get current motor values
        with self.lock:
            motors = list(self.motor_values)

        # Apply to PyBullet
        self.drone.set_motors(motors)
        self.drone.step()

        # Generate and send sensor feedback (FDM)
        fdm = self._generate_fdm_packet()
        try:
            self.sensor_socket.sendto(
                fdm.pack(),
                (self.sitl_ip, self.sensor_port)
            )
            self.packets_sent += 1
        except Exception as e:
            print(f"[Bridge] Sensor send error: {e}")

        # Send RC packet (must be continuous to avoid RXLOSS)
        self.send_rc()

        return tuple(motors)

    def get_state(self):
        """Get current drone state"""
        return self.drone.get_state()

    def follow_camera(self):
        """Update camera to follow drone"""
        self.drone.follow_camera()


def demo():
    """Demo: Run bridge with automatic arm/hover test"""
    print("=" * 60)
    print("  Betaflight SITL <-> PyBullet Bridge Demo")
    print("=" * 60)
    print()
    print("This demo will:")
    print("  1. Wait for SITL connection")
    print("  2. Arm the drone via UDP RC")
    print("  3. Ramp up throttle for hover")
    print("  4. Hold position")
    print()
    print("Make sure SITL is running:")
    print("  ~/projects/betaflight/obj/main/betaflight_SITL.elf 127.0.0.1")
    print()

    bridge = SITLBridge(gui=True)
    bridge.start()

    try:
        frame = 0
        last_print = time.time()
        phase = "disarmed"
        phase_start = time.time()

        while True:
            motors = bridge.step()
            bridge.follow_camera()

            # Phase state machine
            elapsed = time.time() - phase_start

            if phase == "disarmed" and elapsed > 2.0:
                # Wait for communication to stabilize then arm
                bridge.arm()
                phase = "armed"
                phase_start = time.time()

            elif phase == "armed" and elapsed > 1.0:
                # Ramp up throttle
                bridge.set_throttle(1100)  # Just above idle
                phase = "spinup"
                phase_start = time.time()

            elif phase == "spinup" and elapsed > 2.0:
                # Increase to hover throttle
                bridge.set_throttle(1350)  # ~35% for hover
                phase = "hover"
                phase_start = time.time()
                print("\n[Bridge] Hovering at throttle 1350")

            # Limit to ~240 Hz
            time.sleep(1/240)

            frame += 1

            # Print status every second
            if time.time() - last_print > 1.0:
                state = bridge.get_state()
                armed_str = "ARMED" if bridge._armed else "DISARM"
                print(f"\r[{frame:6d}] {armed_str} Thr:{bridge._rc_channels[2]:4d} "
                      f"Motors: [{motors[0]:.2f} {motors[1]:.2f} {motors[2]:.2f} {motors[3]:.2f}] "
                      f"Alt: {state.z:.2f}m   ",
                      end="", flush=True)
                last_print = time.time()

    except KeyboardInterrupt:
        print("\n\n[Bridge] Disarming...")
        bridge.disarm()
        # Give time for disarm command to be sent
        for _ in range(50):
            bridge.step()
            time.sleep(1/240)
        print("[Bridge] Stopped")
    finally:
        bridge.stop()


if __name__ == "__main__":
    demo()
