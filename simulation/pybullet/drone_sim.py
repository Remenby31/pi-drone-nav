#!/usr/bin/env python3
"""
PyBullet Drone Simulator

Realistic physics simulation for quadcopter using PyBullet.
Applies motor forces and lets PyBullet handle all physics.
"""

import pybullet as p
import pybullet_data
import numpy as np
import math
import time
from pathlib import Path
from dataclasses import dataclass
from typing import Tuple, List, Optional


@dataclass
class DroneState:
    """Current drone state from simulation"""
    # Position (world frame)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    # Velocity (world frame)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    # Attitude (degrees)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Angular velocity (deg/s)
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0

    # Motor values (0-1)
    motors: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)


class PyBulletDrone:
    """
    Quadcopter simulation using PyBullet physics engine.

    Motor layout (X configuration, looking from above):
        Front
      1     0      (0=FR, 1=FL, 2=BR, 3=BL)
        X
      3     2
        Back

    Motor 0, 3: CW (clockwise)
    Motor 1, 2: CCW (counter-clockwise)
    """

    def __init__(
        self,
        gui: bool = True,
        real_time: bool = True,
        time_step: float = 1/240,
    ):
        self.gui = gui
        self.real_time = real_time
        self.time_step = time_step

        # Physical parameters - D7S Combo (Mark4 7") with Pi5
        # Research: 2807 1300KV motors, 7040 props, 6S battery
        self.mass = 0.90  # kg (frame 125g + motors 216g + 6S 1500mAh 240g + Pi5 46g + electronics 150g)
        self.arm_length = 0.1475  # m (295mm wheelbase / 2)
        self.max_thrust_per_motor = 27.64  # N (2818g per motor from bench tests)
        self.hover_throttle = 0.30  # Realistic hover throttle (T/W ratio 12:1)

        # Torque coefficients
        self.thrust_to_torque = 0.01  # Nm per N of thrust (yaw torque)

        # Motor positions relative to center (x, y)
        self.motor_positions = [
            (self.arm_length, -self.arm_length),   # Motor 0: Front-Right
            (self.arm_length, self.arm_length),    # Motor 1: Front-Left
            (-self.arm_length, -self.arm_length),  # Motor 2: Back-Right
            (-self.arm_length, self.arm_length),   # Motor 3: Back-Left
        ]

        # Motor rotation directions (1=CW, -1=CCW for yaw torque)
        self.motor_directions = [1, -1, -1, 1]  # 0:CW, 1:CCW, 2:CCW, 3:CW

        # State
        self.drone_id = None
        self.physics_client = None
        self.motor_values = [0.0, 0.0, 0.0, 0.0]
        self.armed = False

        self._setup_simulation()

    def _setup_simulation(self):
        """Initialize PyBullet simulation"""
        # Connect to physics server
        if self.gui:
            self.physics_client = p.connect(p.GUI)
            # Camera setup
            p.resetDebugVisualizerCamera(
                cameraDistance=3,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=[0, 0, 1]
            )
            # Disable extra GUI panels
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        else:
            self.physics_client = p.connect(p.DIRECT)

        # Set physics parameters
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.time_step)
        p.setRealTimeSimulation(0)  # We control stepping

        # Load ground plane
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")

        # Add landing pad visual (checkerboard pattern)
        self._add_landing_pad()

        # Load drone
        urdf_path = Path(__file__).parent / "models" / "quadcopter.urdf"
        self.drone_id = p.loadURDF(
            str(urdf_path),
            basePosition=[0, 0, 0.1],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=False
        )

        # Get base link index
        self.base_link_index = -1  # -1 means base in PyBullet

        # Enable velocity/force control
        p.setPhysicsEngineParameter(enableConeFriction=True)

        # Add reference markers
        self._add_reference_markers()

    def _add_landing_pad(self):
        """Add visual landing pad"""
        # Create a visual shape for landing pad
        pad_size = 0.5
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[pad_size, pad_size, 0.001],
            rgbaColor=[0.8, 0.8, 0.8, 1]
        )
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=visual_shape,
            basePosition=[0, 0, 0.001]
        )

    def _add_reference_markers(self):
        """Add altitude reference markers"""
        for height in [1, 2, 3, 4, 5]:
            # Vertical pole
            visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=0.02,
                length=height,
                rgbaColor=[0.2, 0.6, 0.2, 0.5]
            )
            p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=visual,
                basePosition=[2, 0, height/2]
            )
            # Height label (debug text)
            p.addUserDebugText(
                f"{height}m",
                [2.2, 0, height],
                textColorRGB=[0, 0, 0],
                textSize=1.0
            )

    def reset(self, position: Tuple[float, float, float] = (0, 0, 0.1)):
        """Reset drone to initial position"""
        p.resetBasePositionAndOrientation(
            self.drone_id,
            position,
            p.getQuaternionFromEuler([0, 0, 0])
        )
        p.resetBaseVelocity(self.drone_id, [0, 0, 0], [0, 0, 0])
        self.motor_values = [0.0, 0.0, 0.0, 0.0]
        self.armed = False

    def set_motors(self, motor_values: List[float]):
        """
        Set motor thrust values (0-1 range).

        Args:
            motor_values: List of 4 motor values [FR, FL, BR, BL]
        """
        if len(motor_values) != 4:
            raise ValueError("Must provide 4 motor values")

        self.motor_values = [max(0, min(1, v)) for v in motor_values]

    def set_rc(self, roll: int, pitch: int, throttle: int, yaw: int):
        """
        Set RC-style inputs (1000-2000 range) and compute motor mixing.

        This simulates Betaflight's mixer with X-frame configuration.
        """
        # Normalize inputs (-1 to 1 for stick, 0-1 for throttle)
        roll_cmd = (roll - 1500) / 500      # -1 to 1
        pitch_cmd = (pitch - 1500) / 500    # -1 to 1
        yaw_cmd = (yaw - 1500) / 500        # -1 to 1
        throttle_cmd = (throttle - 1000) / 1000  # 0 to 1

        # Betaflight X-frame mixer
        # Motor 0 (FR): throttle - roll - pitch + yaw
        # Motor 1 (FL): throttle + roll - pitch - yaw
        # Motor 2 (BR): throttle - roll + pitch - yaw
        # Motor 3 (BL): throttle + roll + pitch + yaw
        mix_scale = 0.3  # How much roll/pitch/yaw affects motors

        motors = [
            throttle_cmd - roll_cmd * mix_scale - pitch_cmd * mix_scale + yaw_cmd * mix_scale,
            throttle_cmd + roll_cmd * mix_scale - pitch_cmd * mix_scale - yaw_cmd * mix_scale,
            throttle_cmd - roll_cmd * mix_scale + pitch_cmd * mix_scale - yaw_cmd * mix_scale,
            throttle_cmd + roll_cmd * mix_scale + pitch_cmd * mix_scale + yaw_cmd * mix_scale,
        ]

        self.set_motors(motors)

    def step(self):
        """Advance simulation by one time step"""
        if self.armed:
            self._apply_motor_forces()

        p.stepSimulation()

        if self.real_time and self.gui:
            time.sleep(self.time_step)

    def _apply_motor_forces(self):
        """Apply thrust forces from motors"""
        # Get current orientation
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        rot_matrix = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)

        # Local Z axis (up direction in body frame)
        local_up = rot_matrix @ np.array([0, 0, 1])

        total_torque = np.array([0.0, 0.0, 0.0])

        for i, motor_val in enumerate(self.motor_values):
            # Thrust force
            thrust = motor_val * self.max_thrust_per_motor
            force = local_up * thrust

            # Motor position in world frame
            motor_x, motor_y = self.motor_positions[i]
            motor_pos_local = np.array([motor_x, motor_y, 0.02])
            motor_pos_world = rot_matrix @ motor_pos_local + np.array(pos)

            # Apply force at motor position
            p.applyExternalForce(
                self.drone_id,
                self.base_link_index,
                forceObj=force.tolist(),
                posObj=motor_pos_world.tolist(),
                flags=p.WORLD_FRAME
            )

            # Yaw torque from motor rotation
            yaw_torque = thrust * self.thrust_to_torque * self.motor_directions[i]
            total_torque[2] += yaw_torque

        # Apply yaw torque
        torque_world = rot_matrix @ total_torque
        p.applyExternalTorque(
            self.drone_id,
            self.base_link_index,
            torqueObj=torque_world.tolist(),
            flags=p.WORLD_FRAME
        )

    def get_state(self) -> DroneState:
        """Get current drone state"""
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        vel, ang_vel = p.getBaseVelocity(self.drone_id)

        # Convert quaternion to Euler angles
        euler = p.getEulerFromQuaternion(orn)

        return DroneState(
            x=pos[0],
            y=pos[1],
            z=pos[2],
            vx=vel[0],
            vy=vel[1],
            vz=vel[2],
            roll=math.degrees(euler[0]),
            pitch=math.degrees(euler[1]),
            yaw=math.degrees(euler[2]),
            roll_rate=math.degrees(ang_vel[0]),
            pitch_rate=math.degrees(ang_vel[1]),
            yaw_rate=math.degrees(ang_vel[2]),
            motors=tuple(self.motor_values)
        )

    def arm(self):
        """Arm the drone"""
        self.armed = True

    def disarm(self):
        """Disarm the drone"""
        self.armed = False
        self.motor_values = [0.0, 0.0, 0.0, 0.0]

    def close(self):
        """Close simulation"""
        p.disconnect(self.physics_client)

    def follow_camera(self, distance: float = 3.0):
        """Update camera to follow drone"""
        if not self.gui:
            return

        state = self.get_state()
        p.resetDebugVisualizerCamera(
            cameraDistance=distance,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[state.x, state.y, state.z]
        )


def demo():
    """Quick demo of the simulation"""
    print("=" * 60)
    print("  PyBullet Drone Simulation Demo")
    print("=" * 60)

    drone = PyBulletDrone(gui=True, real_time=True)
    drone.reset()

    print("\nPress Ctrl+C to exit\n")

    try:
        # Arm
        print("Arming...")
        drone.arm()
        time.sleep(0.5)

        # Takeoff
        print("Taking off...")
        for i in range(300):  # 300 steps
            # Simple altitude controller
            state = drone.get_state()
            target_alt = 2.0
            error = target_alt - state.z

            throttle = 1000 + int(drone.hover_throttle * 1000 + error * 150)
            throttle = max(1100, min(1800, throttle))

            drone.set_rc(1500, 1500, throttle, 1500)
            drone.step()
            drone.follow_camera()

            if i % 50 == 0:
                print(f"  Alt: {state.z:.2f}m  Vel: {state.vz:+.2f}m/s")

        # Hover
        print("\nHovering for 3 seconds...")
        for i in range(720):  # ~3 seconds at 240Hz
            state = drone.get_state()
            error = 2.0 - state.z
            throttle = 1000 + int(drone.hover_throttle * 1000 + error * 150 - state.vz * 50)
            throttle = max(1100, min(1800, throttle))

            drone.set_rc(1500, 1500, throttle, 1500)
            drone.step()
            drone.follow_camera()

        # Land
        print("\nLanding...")
        while True:
            state = drone.get_state()
            if state.z < 0.15:
                break

            target_vz = -0.5
            error_vz = target_vz - state.vz
            throttle = 1000 + int(drone.hover_throttle * 1000 + error_vz * 100)
            throttle = max(1000, min(1600, throttle))

            drone.set_rc(1500, 1500, throttle, 1500)
            drone.step()
            drone.follow_camera()

        # Disarm
        print("Disarming...")
        drone.disarm()
        for _ in range(100):
            drone.step()

        print("\nDemo complete!")

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        drone.close()


if __name__ == "__main__":
    demo()
