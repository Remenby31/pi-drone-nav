#!/usr/bin/env python3
"""
Full Mission Simulator with integrated physics

This runs a complete physics simulation without requiring SITL motor feedback.
The physics engine computes drone state based on RC commands directly.
"""

import sys
import time
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))


class DronePhysics:
    """Self-contained drone physics simulation"""

    def __init__(self):
        # State
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0  # Altitude (up positive)

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.roll = 0.0   # degrees
        self.pitch = 0.0
        self.yaw = 0.0

        # Physical params
        self.mass = 1.5
        self.gravity = 9.81
        self.max_thrust = 25.0  # N total at full throttle
        self.hover_throttle = 0.59  # Throttle to hover (mg/max_thrust)

        # Control
        self.armed = False
        self.throttle = 0.0  # 0-1
        self.roll_cmd = 0.0  # degrees
        self.pitch_cmd = 0.0
        self.yaw_rate_cmd = 0.0

        # Time
        self.dt = 0.02  # 50Hz

    def set_rc(self, roll: int, pitch: int, throttle: int, yaw: int, arm: bool):
        """Set RC inputs (1000-2000 range)"""
        self.armed = arm
        if not arm:
            self.throttle = 0
            return

        # Convert RC to normalized
        self.roll_cmd = (roll - 1500) / 500 * 30  # -30 to +30 deg
        self.pitch_cmd = (pitch - 1500) / 500 * 30
        self.throttle = (throttle - 1000) / 1000  # 0 to 1
        self.yaw_rate_cmd = (yaw - 1500) / 500 * 180  # deg/s

    def update(self):
        """Update physics"""
        if not self.armed:
            return

        # Attitude response (first order)
        tau = 0.1
        self.roll += (self.roll_cmd - self.roll) * self.dt / tau
        self.pitch += (self.pitch_cmd - self.pitch) * self.dt / tau
        self.yaw += self.yaw_rate_cmd * self.dt

        # Thrust
        thrust = self.throttle * self.max_thrust

        # Acceleration (body to world, simplified)
        roll_rad = math.radians(self.roll)
        pitch_rad = math.radians(self.pitch)

        ax = thrust * (-math.sin(pitch_rad)) / self.mass
        ay = thrust * (math.sin(roll_rad) * math.cos(pitch_rad)) / self.mass
        az = thrust * (math.cos(roll_rad) * math.cos(pitch_rad)) / self.mass - self.gravity

        # Drag
        drag = 0.3
        self.vx -= drag * self.vx * self.dt
        self.vy -= drag * self.vy * self.dt
        self.vz -= drag * self.vz * self.dt

        # Integrate
        self.vx += ax * self.dt
        self.vy += ay * self.dt
        self.vz += az * self.dt

        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        self.z += self.vz * self.dt

        # Ground constraint
        if self.z < 0:
            self.z = 0
            self.vz = 0
            self.vx *= 0.8
            self.vy *= 0.8


class MissionSimulator:
    """Mission simulator with real physics"""

    def __init__(self):
        self.physics = DronePhysics()
        self.armed = False

    def arm(self):
        print("\n🔓 ARMING...")
        self.armed = True
        self.physics.set_rc(1500, 1500, 1000, 1500, True)
        time.sleep(0.5)
        print("   ✓ Armed")

    def disarm(self):
        print("\n🔒 DISARMING...")
        self.armed = False
        self.physics.set_rc(1500, 1500, 1000, 1500, False)
        time.sleep(0.3)
        print("   ✓ Disarmed")

    def display(self, action: str = ""):
        """Display current state with visual bar"""
        p = self.physics

        # Altitude bar (0-5m scale)
        bar_len = 30
        alt_pct = min(p.z / 5.0, 1.0)
        filled = int(alt_pct * bar_len)
        alt_bar = "█" * filled + "░" * (bar_len - filled)

        # Velocity indicator
        vel_ind = "↑" if p.vz > 0.1 else ("↓" if p.vz < -0.1 else "―")

        print(f"\r  Alt: {p.z:5.2f}m {vel_ind} [{alt_bar}] "
              f"Vel: {p.vz:+5.2f}m/s | "
              f"Pos: ({p.x:+5.1f}, {p.y:+5.1f}) | "
              f"Att: R{p.roll:+5.1f}° P{p.pitch:+5.1f}° | "
              f"{action}     ", end='', flush=True)

    def takeoff(self, target_alt: float):
        """Execute takeoff to target altitude"""
        print(f"\n🛫 TAKEOFF to {target_alt}m...")

        p = self.physics
        start_time = time.time()

        # Phase 1: Ramp up throttle
        print("   Throttle ramp up...")
        for i in range(30):
            throttle = 1000 + int(i * 20)  # 1000 -> 1600
            p.set_rc(1500, 1500, throttle, 1500, True)
            p.update()
            self.display("Takeoff - Ramp")
            time.sleep(0.05)

        # Phase 2: Climb with altitude hold
        print("\n   Climbing...")
        while time.time() - start_time < 8.0:
            # Simple altitude controller
            error = target_alt - p.z
            throttle_adj = error * 150  # P controller
            throttle = int(1000 + p.hover_throttle * 1000 + throttle_adj)
            throttle = max(1100, min(1800, throttle))

            p.set_rc(1500, 1500, throttle, 1500, True)
            p.update()
            self.display(f"Takeoff - Climb ({error:+.1f}m)")
            time.sleep(0.02)

            # Check if reached altitude
            if abs(error) < 0.1 and abs(p.vz) < 0.2:
                break

        print(f"\n   ✓ Reached {p.z:.2f}m (target: {target_alt}m)")

    def hover(self, duration: float):
        """Hover at current position"""
        print(f"\n🚁 HOVER for {duration}s...")

        p = self.physics
        target_alt = p.z
        start_time = time.time()

        while time.time() - start_time < duration:
            # Altitude hold
            error = target_alt - p.z
            throttle_adj = error * 150 + p.vz * (-50)  # PD controller
            throttle = int(1000 + p.hover_throttle * 1000 + throttle_adj)
            throttle = max(1100, min(1800, throttle))

            p.set_rc(1500, 1500, throttle, 1500, True)
            p.update()

            remaining = duration - (time.time() - start_time)
            self.display(f"Hover - {remaining:.1f}s remaining")
            time.sleep(0.02)

        print(f"\n   ✓ Hover complete at {p.z:.2f}m")

    def land(self):
        """Execute landing"""
        print("\n🛬 LANDING...")

        p = self.physics

        # Phase 1: Reduce altitude
        print("   Descending...")
        descent_rate = -0.5  # m/s target

        while p.z > 0.1:
            # Altitude controller targeting descent
            target_vz = descent_rate
            error_vz = target_vz - p.vz
            throttle_adj = error_vz * 100
            throttle = int(1000 + p.hover_throttle * 1000 + throttle_adj)
            throttle = max(1000, min(1600, throttle))

            p.set_rc(1500, 1500, throttle, 1500, True)
            p.update()
            self.display(f"Landing - {p.z:.1f}m")
            time.sleep(0.02)

        # Phase 2: Cut throttle
        print("\n   Touchdown...")
        for i in range(20):
            throttle = max(1000, 1200 - i * 10)
            p.set_rc(1500, 1500, throttle, 1500, True)
            p.update()
            time.sleep(0.05)

        p.set_rc(1500, 1500, 1000, 1500, True)
        p.update()

        print(f"   ✓ Landed at {p.z:.2f}m")


def main():
    print("=" * 70)
    print("  🚁 Pi Drone Nav - Full Physics Mission Simulator")
    print("=" * 70)
    print()
    print("  Mission: Test Hover 2m")
    print("  Actions: Takeoff → Hover 3s → Land")
    print()
    print("-" * 70)

    sim = MissionSimulator()

    try:
        # Execute mission
        sim.arm()

        print("\n>>> Action 1/3: TAKEOFF")
        sim.takeoff(target_alt=2.0)

        print("\n>>> Action 2/3: HOVER")
        sim.hover(duration=3.0)

        print("\n>>> Action 3/3: LAND")
        sim.land()

        sim.disarm()

        print("\n" + "=" * 70)
        print("  ✅ MISSION COMPLETE!")
        print("=" * 70)

        # Final stats
        p = sim.physics
        print(f"\n  Final Position: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})m")
        print(f"  Final Attitude: Roll={p.roll:.1f}° Pitch={p.pitch:.1f}° Yaw={p.yaw:.1f}°")
        print()

    except KeyboardInterrupt:
        print("\n\n⚠ Mission aborted!")
        sim.disarm()


if __name__ == '__main__':
    main()
