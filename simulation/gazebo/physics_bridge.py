"""
Physics Bridge: Betaflight SITL <-> Gazebo

This bridge:
1. Receives motor PWM values from Betaflight SITL (UDP port 9002)
2. Converts to forces and sends to Gazebo
3. Receives physics state from Gazebo
4. Sends IMU/sensor data back to Betaflight SITL
"""

import socket
import struct
import threading
import time
import math
from dataclasses import dataclass, field
from typing import Optional, Callable
import logging

logger = logging.getLogger(__name__)


@dataclass
class QuadcopterParams:
    """Physical parameters for quadcopter simulation"""
    # Mass and inertia
    mass: float = 1.5  # kg
    arm_length: float = 0.25  # m (motor to center)

    # Motor parameters
    motor_kv: float = 2300  # RPM/V
    motor_tau: float = 0.02  # s (motor time constant)
    max_rpm: float = 25000
    min_rpm: float = 1000

    # Propeller parameters
    prop_diameter: float = 0.127  # m (5 inch)
    thrust_coef: float = 1.0e-7  # Thrust = Ct * rpm^2
    torque_coef: float = 1.0e-9  # Torque = Cq * rpm^2

    # PWM to RPM mapping
    pwm_min: int = 1000
    pwm_max: int = 2000

    # Drag coefficients
    drag_xy: float = 0.5
    drag_z: float = 0.8


@dataclass
class DroneState:
    """Current drone state"""
    # Position (NED, meters)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0  # Down is positive in NED

    # Velocity (NED, m/s)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    # Attitude (radians)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Angular velocity (rad/s)
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0

    # Accelerometer (body frame, m/s^2)
    acc_x: float = 0.0
    acc_y: float = 0.0
    acc_z: float = 9.81  # Gravity

    # Motor RPMs
    motor_rpm: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])


class PhysicsBridge:
    """
    Bridge between Betaflight SITL and Gazebo

    Betaflight SITL sends PWM values via UDP to port 9002.
    We convert these to forces and apply them in Gazebo.
    Then we read the resulting state from Gazebo and send
    sensor data back to SITL.
    """

    def __init__(self, params: Optional[QuadcopterParams] = None):
        self.params = params or QuadcopterParams()
        self.state = DroneState()

        # Network sockets
        self._sitl_socket: Optional[socket.socket] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Motor values from SITL (PWM 1000-2000)
        self._motor_pwm = [1000, 1000, 1000, 1000]
        self._motor_pwm_lock = threading.Lock()

        # Callbacks
        self._on_motor_update: Optional[Callable[[list], None]] = None

    def start(self, sitl_port: int = 9002):
        """
        Start the physics bridge

        Args:
            sitl_port: UDP port to receive PWM from SITL
        """
        if self._running:
            return

        # Create UDP socket to receive from SITL
        self._sitl_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sitl_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sitl_socket.bind(('0.0.0.0', sitl_port))
        self._sitl_socket.settimeout(0.1)

        logger.info(f"Physics bridge listening on UDP port {sitl_port}")

        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the physics bridge"""
        self._running = False

        if self._thread:
            self._thread.join(timeout=2.0)

        if self._sitl_socket:
            self._sitl_socket.close()
            self._sitl_socket = None

    def _receive_loop(self):
        """Receive motor PWM from SITL"""
        while self._running:
            try:
                data, addr = self._sitl_socket.recvfrom(1024)

                # Parse PWM values from SITL
                # Format: 16 x float (motor values, only first 4 used for quad)
                if len(data) >= 64:  # 16 floats * 4 bytes
                    motors = struct.unpack('<16f', data[:64])

                    with self._motor_pwm_lock:
                        # Convert from 0-1 range to PWM
                        for i in range(4):
                            self._motor_pwm[i] = int(1000 + motors[i] * 1000)

                    # Callback
                    if self._on_motor_update:
                        self._on_motor_update(self._motor_pwm[:4])

            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    logger.error(f"Error receiving from SITL: {e}")

    def get_motor_pwm(self) -> list:
        """Get current motor PWM values"""
        with self._motor_pwm_lock:
            return self._motor_pwm.copy()

    def pwm_to_rpm(self, pwm: int) -> float:
        """Convert PWM to motor RPM"""
        # Linear mapping
        normalized = (pwm - self.params.pwm_min) / (self.params.pwm_max - self.params.pwm_min)
        normalized = max(0, min(1, normalized))
        return self.params.min_rpm + normalized * (self.params.max_rpm - self.params.min_rpm)

    def rpm_to_thrust(self, rpm: float) -> float:
        """Convert RPM to thrust (N)"""
        return self.params.thrust_coef * rpm * rpm

    def rpm_to_torque(self, rpm: float) -> float:
        """Convert RPM to torque (N.m)"""
        return self.params.torque_coef * rpm * rpm

    def calculate_forces(self) -> tuple:
        """
        Calculate total force and torque from motors

        Returns:
            (force_z, torque_x, torque_y, torque_z) in body frame
        """
        motors = self.get_motor_pwm()

        # Convert to RPM and thrust
        thrusts = []
        torques = []
        for pwm in motors:
            rpm = self.pwm_to_rpm(pwm)
            thrusts.append(self.rpm_to_thrust(rpm))
            torques.append(self.rpm_to_torque(rpm))

        # Total thrust (all motors point up in body frame)
        force_z = sum(thrusts)

        # Torques from thrust differential
        # Motor layout (X configuration):
        #   0   1     Front
        #    \ /
        #     X
        #    / \
        #   3   2     Back

        L = self.params.arm_length

        # Roll torque (positive = right side down)
        torque_x = L * (thrusts[0] + thrusts[3] - thrusts[1] - thrusts[2]) * 0.707

        # Pitch torque (positive = nose up)
        torque_y = L * (thrusts[2] + thrusts[3] - thrusts[0] - thrusts[1]) * 0.707

        # Yaw torque (from motor reaction torques)
        # Motors 0,2 spin CW (positive torque), 1,3 spin CCW
        torque_z = torques[0] - torques[1] + torques[2] - torques[3]

        return force_z, torque_x, torque_y, torque_z

    def send_sensor_data_to_sitl(self, state: DroneState, sitl_addr: tuple = ('127.0.0.1', 9003)):
        """
        Send sensor data back to SITL

        The SITL expects sensor data in a specific format on UDP port 9003.
        """
        # Pack state data
        # Format matches what SITL expects (see sitl_state_in_t in Betaflight)
        data = struct.pack(
            '<3f3f3f3f',  # 12 floats
            state.acc_x, state.acc_y, state.acc_z,  # Accelerometer
            math.degrees(state.roll_rate), math.degrees(state.pitch_rate), math.degrees(state.yaw_rate),  # Gyro (deg/s)
            state.roll, state.pitch, state.yaw,  # Attitude (unused by SITL)
            state.vx, state.vy, state.vz,  # Velocity (unused)
        )

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(data, sitl_addr)
            sock.close()
        except Exception as e:
            logger.error(f"Failed to send sensor data: {e}")

    def on_motor_update(self, callback: Callable[[list], None]):
        """Set callback for motor updates"""
        self._on_motor_update = callback


class SimplePhysicsSimulator:
    """
    Simple physics simulator (no Gazebo dependency)

    For testing the bridge without Gazebo.
    """

    def __init__(self, params: Optional[QuadcopterParams] = None):
        self.params = params or QuadcopterParams()
        self.state = DroneState()
        self.bridge = PhysicsBridge(params)

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._dt = 0.002  # 500 Hz

    def start(self):
        """Start simulation"""
        self.bridge.start()

        self._running = True
        self._thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop simulation"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self.bridge.stop()

    def _sim_loop(self):
        """Main simulation loop"""
        last_time = time.time()

        while self._running:
            current_time = time.time()
            dt = current_time - last_time

            if dt >= self._dt:
                last_time = current_time
                self._update_physics(dt)

            time.sleep(0.001)

    def _update_physics(self, dt: float):
        """Update physics state"""
        # Get forces from motors
        thrust, tx, ty, tz = self.bridge.calculate_forces()

        # Simple integration (body frame to world frame)
        g = 9.81
        mass = self.params.mass

        # Acceleration in body frame
        acc_body_z = thrust / mass - g * math.cos(self.state.roll) * math.cos(self.state.pitch)

        # Transform to NED (simplified, small angle assumption)
        self.state.acc_z = acc_body_z

        # Integrate velocity
        self.state.vz += self.state.acc_z * dt

        # Integrate position
        self.state.z += self.state.vz * dt

        # Ground constraint
        if self.state.z > 0:  # NED: positive z is down, but we start at z=0
            self.state.z = 0
            self.state.vz = 0

        # Angular dynamics (simplified)
        Ixx = 0.01  # Moment of inertia
        self.state.roll_rate += (tx / Ixx) * dt
        self.state.pitch_rate += (ty / Ixx) * dt
        self.state.yaw_rate += (tz / Ixx) * dt

        # Integrate attitude
        self.state.roll += self.state.roll_rate * dt
        self.state.pitch += self.state.pitch_rate * dt
        self.state.yaw += self.state.yaw_rate * dt


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    print("=== Physics Bridge Test ===\n")
    print("This will listen for PWM from Betaflight SITL on port 9002")
    print("Start SITL first, then run this test.\n")

    def on_motors(pwm):
        print(f"\rMotors: {pwm}", end='', flush=True)

    bridge = PhysicsBridge()
    bridge.on_motor_update(on_motors)
    bridge.start()

    try:
        print("Listening... (Ctrl+C to stop)")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        bridge.stop()
