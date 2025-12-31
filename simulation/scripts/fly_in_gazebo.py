#!/usr/bin/env python3
"""
Fly drone in Gazebo - syncs physics simulation with Gazebo visualization
"""

import subprocess
import time
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))


class GazeboDroneController:
    """Control drone in Gazebo by setting its pose"""

    def __init__(self):
        # Physics state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.1  # Start on ground

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Physics params
        self.mass = 1.5
        self.gravity = 9.81
        self.max_thrust = 25.0
        self.hover_throttle = 0.59

        self.dt = 0.02
        self.armed = False
        self.throttle = 0.0

    def set_pose_in_gazebo(self):
        """Update drone position in Gazebo"""
        # Use gz model to set pose
        cmd = [
            "docker", "exec", "pidrone_gazebo",
            "gz", "model", "-m", "drone",
            "-x", str(self.x),
            "-y", str(self.y),
            "-z", str(self.z),
            "-R", str(self.roll),
            "-P", str(self.pitch),
            "-Y", str(self.yaw)
        ]
        subprocess.run(cmd, capture_output=True)

    def update_physics(self, throttle_cmd, roll_cmd=0, pitch_cmd=0):
        """Update physics simulation"""
        self.throttle = throttle_cmd

        if throttle_cmd < 0.05:
            return

        # Attitude response
        tau = 0.1
        self.roll += (roll_cmd - self.roll) * self.dt / tau
        self.pitch += (pitch_cmd - self.pitch) * self.dt / tau

        # Thrust
        thrust = throttle_cmd * self.max_thrust

        # Acceleration
        roll_rad = math.radians(self.roll)
        pitch_rad = math.radians(self.pitch)

        ax = thrust * (-math.sin(pitch_rad)) / self.mass
        ay = thrust * (math.sin(roll_rad)) / self.mass
        az = thrust * math.cos(roll_rad) * math.cos(pitch_rad) / self.mass - self.gravity

        # Drag
        self.vx *= 0.98
        self.vy *= 0.98
        self.vz *= 0.98

        # Integrate
        self.vx += ax * self.dt
        self.vy += ay * self.dt
        self.vz += az * self.dt

        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        self.z += self.vz * self.dt

        # Ground constraint
        if self.z < 0.1:
            self.z = 0.1
            self.vz = max(0, self.vz)


def run_mission():
    """Execute hover mission with Gazebo visualization"""

    print("=" * 60)
    print("  🚁 MISSION: Hover 2m dans Gazebo")
    print("=" * 60)

    drone = GazeboDroneController()

    # Initial position
    drone.set_pose_in_gazebo()
    time.sleep(0.5)

    print("\n🔓 ARMING...")
    time.sleep(1)
    print("   ✓ Armed")

    # === TAKEOFF ===
    print("\n🛫 TAKEOFF to 2m...")
    target_alt = 2.0

    for i in range(200):  # 4 seconds
        # Altitude controller
        error = target_alt - drone.z
        throttle = drone.hover_throttle + error * 0.15
        throttle = max(0.3, min(0.9, throttle))

        drone.update_physics(throttle)
        drone.set_pose_in_gazebo()

        # Display
        bar = "█" * int(drone.z / 3 * 20) + "░" * (20 - int(drone.z / 3 * 20))
        print(f"\r   Alt: {drone.z:5.2f}m [{bar}] Vel: {drone.vz:+5.2f}m/s", end="", flush=True)

        time.sleep(drone.dt)

        if abs(error) < 0.1 and abs(drone.vz) < 0.1:
            break

    print(f"\n   ✓ Altitude atteinte: {drone.z:.2f}m")

    # === HOVER ===
    print("\n🚁 HOVER 3 seconds...")

    for i in range(150):  # 3 seconds
        error = target_alt - drone.z
        throttle = drone.hover_throttle + error * 0.2 - drone.vz * 0.1
        throttle = max(0.3, min(0.9, throttle))

        drone.update_physics(throttle)
        drone.set_pose_in_gazebo()

        remaining = 3.0 - i * drone.dt
        print(f"\r   Hovering... {remaining:.1f}s | Alt: {drone.z:.2f}m", end="", flush=True)

        time.sleep(drone.dt)

    print("\n   ✓ Hover complete")

    # === LAND ===
    print("\n🛬 LANDING...")

    while drone.z > 0.15:
        # Descend slowly
        target_vz = -0.5
        error_vz = target_vz - drone.vz
        throttle = drone.hover_throttle + error_vz * 0.15
        throttle = max(0.1, min(0.8, throttle))

        drone.update_physics(throttle)
        drone.set_pose_in_gazebo()

        print(f"\r   Descending... Alt: {drone.z:.2f}m Vel: {drone.vz:+.2f}m/s", end="", flush=True)

        time.sleep(drone.dt)

    # Touch down
    drone.z = 0.1
    drone.vz = 0
    drone.set_pose_in_gazebo()

    print("\n   ✓ Landed!")

    # === DISARM ===
    print("\n🔒 DISARMING...")
    time.sleep(0.5)
    print("   ✓ Disarmed")

    print("\n" + "=" * 60)
    print("  ✅ MISSION COMPLETE!")
    print("=" * 60)


if __name__ == "__main__":
    run_mission()
