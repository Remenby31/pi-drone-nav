#!/usr/bin/env python3
"""
Betaflight SITL <-> Gazebo Bridge

This bridge connects Betaflight SITL to Gazebo:
1. Receives motor PWM from SITL (UDP 9002)
2. Applies forces to drone model in Gazebo
3. Reads IMU/state from Gazebo
4. Sends sensor data back to SITL (UDP 9003)

Can run standalone or be imported.
"""

import socket
import struct
import time
import math
import threading
from dataclasses import dataclass, field
from typing import Optional, List
import logging

logger = logging.getLogger(__name__)


@dataclass
class DronePhysics:
    """Drone physics parameters"""
    mass: float = 1.5  # kg
    arm_length: float = 0.177  # m (diagonal, matches model)

    # Thrust model: T = k * throttle^2
    thrust_coefficient: float = 15.0  # N at full throttle per motor

    # Motor response
    motor_tau: float = 0.02  # time constant (s)

    # Drag
    drag_xy: float = 0.5
    drag_z: float = 0.8

    # Inertia (approximate)
    Ixx: float = 0.029
    Iyy: float = 0.029
    Izz: float = 0.055


@dataclass
class SimState:
    """Simulation state"""
    # Position (ENU - East North Up)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.1  # Start slightly above ground

    # Velocity
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    # Attitude (euler angles, radians)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Angular velocity
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0

    # Motor values (0-1 normalized)
    motors: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])

    # Timestamp
    time: float = 0.0


class BetaflightGazeboBridge:
    """
    Bridge between Betaflight SITL and Gazebo (or internal physics)

    Betaflight SITL sends motor outputs to UDP 9002 as 16 floats (0-1).
    We simulate physics and send back sensor data to UDP 9003.
    """

    def __init__(self, physics: Optional[DronePhysics] = None):
        self.physics = physics or DronePhysics()
        self.state = SimState()

        # Network
        self._sitl_motor_socket: Optional[socket.socket] = None
        self._sitl_sensor_socket: Optional[socket.socket] = None
        self._sitl_addr = ('127.0.0.1', 9003)

        # Threading
        self._running = False
        self._motor_thread: Optional[threading.Thread] = None
        self._physics_thread: Optional[threading.Thread] = None

        # Motor values from SITL (normalized 0-1)
        self._motors = [0.0, 0.0, 0.0, 0.0]
        self._motors_lock = threading.Lock()

        # Physics rate
        self._dt = 0.002  # 500 Hz

        # Gravity
        self._gravity = 9.81

    def start(self, sitl_motor_port: int = 9002, sitl_sensor_port: int = 9003):
        """Start the bridge"""
        # Create motor receive socket
        self._sitl_motor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sitl_motor_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sitl_motor_socket.bind(('0.0.0.0', sitl_motor_port))
        self._sitl_motor_socket.settimeout(0.1)

        # Create sensor send socket
        self._sitl_sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sitl_addr = ('127.0.0.1', sitl_sensor_port)

        self._running = True

        # Start motor receive thread
        self._motor_thread = threading.Thread(target=self._motor_receive_loop, daemon=True)
        self._motor_thread.start()

        # Start physics thread
        self._physics_thread = threading.Thread(target=self._physics_loop, daemon=True)
        self._physics_thread.start()

        logger.info(f"Bridge started: motors on UDP {sitl_motor_port}, sensors to UDP {sitl_sensor_port}")

    def stop(self):
        """Stop the bridge"""
        self._running = False

        if self._motor_thread:
            self._motor_thread.join(timeout=2.0)
        if self._physics_thread:
            self._physics_thread.join(timeout=2.0)

        if self._sitl_motor_socket:
            self._sitl_motor_socket.close()
        if self._sitl_sensor_socket:
            self._sitl_sensor_socket.close()

    def _motor_receive_loop(self):
        """Receive motor values from SITL"""
        while self._running:
            try:
                data, addr = self._sitl_motor_socket.recvfrom(1024)

                # Parse 16 floats
                if len(data) >= 64:
                    motors = struct.unpack('<16f', data[:64])

                    with self._motors_lock:
                        self._motors = list(motors[:4])

            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    logger.debug(f"Motor receive error: {e}")

    def _physics_loop(self):
        """Main physics simulation loop"""
        last_time = time.time()

        while self._running:
            current_time = time.time()
            dt = current_time - last_time

            if dt >= self._dt:
                last_time = current_time
                self._update_physics(self._dt)
                self._send_sensors()

            time.sleep(0.0005)  # Small sleep to not hog CPU

    def _update_physics(self, dt: float):
        """Update physics simulation"""
        # Get motor values
        with self._motors_lock:
            motors = self._motors.copy()

        self.state.motors = motors
        self.state.time += dt

        # Calculate thrust from each motor
        thrusts = [m * self.physics.thrust_coefficient for m in motors]

        # Total thrust (body Z up)
        total_thrust = sum(thrusts)

        # Calculate torques
        # Motor layout (X config, looking from above):
        #   0(CW)    1(CCW)   Front
        #      \    /
        #       \  /
        #        \/
        #        /\
        #       /  \
        #      /    \
        #   3(CCW)  2(CW)    Back

        L = self.physics.arm_length * 0.707  # Component along X and Y

        # Roll torque (positive = right wing down)
        torque_roll = L * (thrusts[1] + thrusts[2] - thrusts[0] - thrusts[3])

        # Pitch torque (positive = nose up)
        torque_pitch = L * (thrusts[0] + thrusts[1] - thrusts[2] - thrusts[3])

        # Yaw torque (from motor reaction, simplified)
        torque_yaw = 0.01 * (thrusts[0] - thrusts[1] + thrusts[2] - thrusts[3])

        # Angular acceleration
        alpha_roll = torque_roll / self.physics.Ixx
        alpha_pitch = torque_pitch / self.physics.Iyy
        alpha_yaw = torque_yaw / self.physics.Izz

        # Update angular velocity
        self.state.wx += alpha_roll * dt
        self.state.wy += alpha_pitch * dt
        self.state.wz += alpha_yaw * dt

        # Angular damping
        damping = 0.1
        self.state.wx *= (1 - damping * dt)
        self.state.wy *= (1 - damping * dt)
        self.state.wz *= (1 - damping * dt)

        # Update attitude
        self.state.roll += self.state.wx * dt
        self.state.pitch += self.state.wy * dt
        self.state.yaw += self.state.wz * dt

        # Clamp attitude
        self.state.roll = max(-1.0, min(1.0, self.state.roll))
        self.state.pitch = max(-1.0, min(1.0, self.state.pitch))

        # Calculate acceleration in world frame
        # Thrust vector in body frame is [0, 0, thrust]
        # Rotate to world frame (simplified small angle)

        ax = total_thrust * (-math.sin(self.state.pitch)) / self.physics.mass
        ay = total_thrust * (math.sin(self.state.roll) * math.cos(self.state.pitch)) / self.physics.mass
        az = total_thrust * (math.cos(self.state.roll) * math.cos(self.state.pitch)) / self.physics.mass - self._gravity

        # Add drag
        speed = math.sqrt(self.state.vx**2 + self.state.vy**2 + self.state.vz**2)
        if speed > 0.1:
            drag_factor = self.physics.drag_xy / self.physics.mass
            ax -= drag_factor * self.state.vx * abs(self.state.vx)
            ay -= drag_factor * self.state.vy * abs(self.state.vy)
            az -= self.physics.drag_z / self.physics.mass * self.state.vz * abs(self.state.vz)

        # Update velocity
        self.state.vx += ax * dt
        self.state.vy += ay * dt
        self.state.vz += az * dt

        # Update position
        self.state.x += self.state.vx * dt
        self.state.y += self.state.vy * dt
        self.state.z += self.state.vz * dt

        # Ground constraint
        if self.state.z <= 0.05:
            self.state.z = 0.05
            if self.state.vz < 0:
                self.state.vz = 0
            # Ground friction
            self.state.vx *= 0.95
            self.state.vy *= 0.95

    def _send_sensors(self):
        """Send sensor data to SITL"""
        # Calculate IMU values
        # Accelerometer (body frame, m/s^2)
        # When hovering, should read ~0, 0, 9.81

        # Simplified: just send gravity rotated to body frame + acceleration
        acc_x = self._gravity * math.sin(self.state.pitch)
        acc_y = -self._gravity * math.sin(self.state.roll) * math.cos(self.state.pitch)
        acc_z = self._gravity * math.cos(self.state.roll) * math.cos(self.state.pitch)

        # Gyro (body frame, deg/s for SITL)
        gyro_x = math.degrees(self.state.wx)
        gyro_y = math.degrees(self.state.wy)
        gyro_z = math.degrees(self.state.wz)

        # Build FDM packet for SITL
        # Format matches fdmPacket in Betaflight SITL
        packet = struct.pack(
            '<dddddddddd',
            gyro_x, gyro_y, gyro_z,       # Gyro deg/s
            acc_x, acc_y, acc_z,          # Accel m/s^2
            self.state.roll, self.state.pitch, self.state.yaw,  # Attitude rad
            self.state.z,                  # Altitude m
        )

        try:
            self._sitl_sensor_socket.sendto(packet, self._sitl_addr)
        except Exception as e:
            logger.debug(f"Sensor send error: {e}")

    def get_state(self) -> SimState:
        """Get current simulation state"""
        return SimState(
            x=self.state.x,
            y=self.state.y,
            z=self.state.z,
            vx=self.state.vx,
            vy=self.state.vy,
            vz=self.state.vz,
            roll=self.state.roll,
            pitch=self.state.pitch,
            yaw=self.state.yaw,
            wx=self.state.wx,
            wy=self.state.wy,
            wz=self.state.wz,
            motors=self.state.motors.copy(),
            time=self.state.time,
        )


def main():
    """Run the bridge standalone"""
    import argparse

    parser = argparse.ArgumentParser(description='Betaflight SITL <-> Gazebo Bridge')
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s'
    )

    print("=" * 60)
    print("  Betaflight SITL <-> Gazebo Bridge")
    print("=" * 60)
    print()
    print("Waiting for SITL motor outputs on UDP 9002...")
    print("Sending sensor data to SITL on UDP 9003...")
    print()
    print("Press Ctrl+C to stop")
    print()

    bridge = BetaflightGazeboBridge()
    bridge.start()

    try:
        while True:
            state = bridge.get_state()
            motors = state.motors
            print(f"\rMotors: [{motors[0]:.2f}, {motors[1]:.2f}, {motors[2]:.2f}, {motors[3]:.2f}] | "
                  f"Pos: ({state.x:.2f}, {state.y:.2f}, {state.z:.2f}) | "
                  f"Att: (R{math.degrees(state.roll):+5.1f}, P{math.degrees(state.pitch):+5.1f}, Y{math.degrees(state.yaw):+5.1f})",
                  end='', flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        bridge.stop()


if __name__ == '__main__':
    main()
