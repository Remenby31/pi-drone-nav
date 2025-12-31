#!/usr/bin/env python3
"""
Run missions in PyBullet simulation

Usage:
    python simulation/pybullet/run_mission.py [mission_file]
    python simulation/pybullet/run_mission.py  # Uses default test mission
"""

import sys
import time
import json
import signal
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from simulation.pybullet.drone_sim import PyBulletDrone, DroneState


class MissionRunner:
    """Run missions in PyBullet simulation"""

    def __init__(self, gui: bool = True):
        self.drone = PyBulletDrone(gui=gui, real_time=True)
        self.current_action = ""

        # Control parameters
        self.hover_throttle = 0.59

    def display(self, state: DroneState):
        """Display current state"""
        bar_len = 30
        alt_pct = min(state.z / 5.0, 1.0)
        filled = int(alt_pct * bar_len)
        alt_bar = "█" * filled + "░" * (bar_len - filled)

        vel_ind = "↑" if state.vz > 0.1 else ("↓" if state.vz < -0.1 else "―")

        print(f"\r  Alt: {state.z:5.2f}m {vel_ind} [{alt_bar}] "
              f"Vel: {state.vz:+5.2f}m/s | "
              f"Att: R{state.roll:+5.1f}° P{state.pitch:+5.1f}° | "
              f"{self.current_action}     ", end="", flush=True)

    def arm(self):
        """Arm the drone"""
        print("\n🔓 ARMING...")
        self.drone.arm()
        # Spin up motors slowly
        for i in range(50):
            throttle = 1000 + i * 2
            self.drone.set_rc(1500, 1500, throttle, 1500)
            self.drone.step()
        print("   ✓ Armed")

    def disarm(self):
        """Disarm the drone"""
        print("\n🔒 DISARMING...")
        self.drone.disarm()
        for _ in range(100):
            self.drone.step()
        print("   ✓ Disarmed")

    def takeoff(self, target_alt: float):
        """Takeoff to target altitude"""
        print(f"\n🛫 TAKEOFF to {target_alt}m...")
        self.current_action = f"Takeoff → {target_alt}m"

        start_time = time.time()
        stable_count = 0

        while time.time() - start_time < 10.0:  # 10s timeout
            state = self.drone.get_state()

            # Altitude P controller
            error = target_alt - state.z
            throttle = 1000 + int(self.hover_throttle * 1000 + error * 150)
            throttle = max(1100, min(1800, throttle))

            self.drone.set_rc(1500, 1500, throttle, 1500)

            # Run multiple physics steps per display
            for _ in range(8):  # 8 steps at 240Hz ≈ 30Hz display
                self.drone.step()

            self.drone.follow_camera()
            self.display(state)

            # Check if stable at target
            if abs(error) < 0.1 and abs(state.vz) < 0.2:
                stable_count += 1
                if stable_count > 30:
                    break
            else:
                stable_count = 0

        state = self.drone.get_state()
        print(f"\n   ✓ Reached {state.z:.2f}m")

    def hover(self, duration: float):
        """Hover at current position"""
        print(f"\n🚁 HOVER for {duration}s...")

        state = self.drone.get_state()
        target_alt = state.z
        start_time = time.time()

        while time.time() - start_time < duration:
            state = self.drone.get_state()
            remaining = duration - (time.time() - start_time)
            self.current_action = f"Hover - {remaining:.1f}s"

            # PD altitude controller
            error = target_alt - state.z
            throttle = 1000 + int(self.hover_throttle * 1000 + error * 150 - state.vz * 50)
            throttle = max(1100, min(1800, throttle))

            self.drone.set_rc(1500, 1500, throttle, 1500)

            for _ in range(8):
                self.drone.step()

            self.drone.follow_camera()
            self.display(state)

        print(f"\n   ✓ Hover complete at {state.z:.2f}m")

    def land(self):
        """Execute landing"""
        print("\n🛬 LANDING...")
        self.current_action = "Landing"

        # Phase 1: Controlled descent
        while True:
            state = self.drone.get_state()

            if state.z < 0.15:
                break

            # Descent rate controller
            target_vz = -0.5  # m/s
            error_vz = target_vz - state.vz
            throttle = 1000 + int(self.hover_throttle * 1000 + error_vz * 100)
            throttle = max(1000, min(1600, throttle))

            self.drone.set_rc(1500, 1500, throttle, 1500)

            for _ in range(8):
                self.drone.step()

            self.drone.follow_camera()
            self.display(state)

        # Phase 2: Cut throttle
        print("\n   Touchdown...")
        for i in range(50):
            throttle = max(1000, 1200 - i * 4)
            self.drone.set_rc(1500, 1500, throttle, 1500)
            self.drone.step()

        state = self.drone.get_state()
        print(f"   ✓ Landed at {state.z:.2f}m")

    def goto(self, target_x: float, target_y: float, target_alt: float, speed: float = 2.0):
        """Fly to position (simplified)"""
        print(f"\n✈️  GOTO ({target_x:.1f}, {target_y:.1f}, {target_alt:.1f}m)...")
        self.current_action = f"Goto ({target_x:.1f}, {target_y:.1f})"

        start_time = time.time()

        while time.time() - start_time < 20.0:  # 20s timeout
            state = self.drone.get_state()

            # Position errors
            error_x = target_x - state.x
            error_y = target_y - state.y
            error_z = target_alt - state.z

            # Distance to target
            dist = (error_x**2 + error_y**2)**0.5

            if dist < 0.2 and abs(error_z) < 0.2:
                break

            # Simple P controller for position -> attitude
            # Pitch controls X (forward), Roll controls Y (sideways)
            pitch = 1500 - int(error_x * 30)  # Negative pitch = forward
            roll = 1500 + int(error_y * 30)   # Positive roll = right
            pitch = max(1400, min(1600, pitch))
            roll = max(1400, min(1600, roll))

            # Altitude
            throttle = 1000 + int(self.hover_throttle * 1000 + error_z * 150 - state.vz * 50)
            throttle = max(1100, min(1800, throttle))

            self.drone.set_rc(roll, pitch, throttle, 1500)

            for _ in range(8):
                self.drone.step()

            self.drone.follow_camera()
            self.display(state)

        state = self.drone.get_state()
        print(f"\n   ✓ Reached ({state.x:.2f}, {state.y:.2f}, {state.z:.2f})")

    def run_mission(self, mission: dict):
        """Run a complete mission"""
        print("\n" + "=" * 60)
        print(f"  📋 Mission: {mission.get('name', 'Unknown')}")
        print("=" * 60)

        if 'description' in mission:
            print(f"  {mission['description']}")

        actions = mission.get('actions', [])
        print(f"\n  Actions: {len(actions)}")
        for i, action in enumerate(actions):
            print(f"    {i+1}. {action['type']}")

        print("\n" + "-" * 60)

        # Reset drone
        self.drone.reset()

        # ARM
        self.arm()
        time.sleep(0.3)

        # Execute actions
        for i, action in enumerate(actions):
            action_type = action['type']
            print(f"\n>>> Action {i+1}/{len(actions)}: {action_type.upper()}")

            if action_type == 'takeoff':
                self.takeoff(action.get('alt', 2.0))

            elif action_type == 'hover':
                self.hover(action.get('duration', 3.0))

            elif action_type == 'land':
                self.land()

            elif action_type == 'goto':
                self.goto(
                    action.get('x', 0),
                    action.get('y', 0),
                    action.get('alt', 2.0),
                    action.get('speed', 2.0)
                )

            elif action_type == 'delay':
                duration = action.get('duration', 1.0)
                print(f"\n⏳ DELAY {duration}s...")
                time.sleep(duration)

            else:
                print(f"  ⚠ Unknown action: {action_type}")

        # DISARM
        self.disarm()

        print("\n" + "=" * 60)
        print("  ✅ Mission Complete!")
        print("=" * 60)

        # Final stats
        state = self.drone.get_state()
        print(f"\n  Final Position: ({state.x:.2f}, {state.y:.2f}, {state.z:.2f})m")
        print(f"  Final Attitude: R={state.roll:.1f}° P={state.pitch:.1f}° Y={state.yaw:.1f}°")
        print()

    def close(self):
        """Close simulation"""
        self.drone.close()


def main():
    # Load mission
    if len(sys.argv) > 1:
        mission_file = sys.argv[1]
    else:
        mission_file = "config/missions/test_hover_2m.json"

    mission_path = Path(mission_file)

    if not mission_path.exists():
        # Use default mission
        mission = {
            "name": "Test Hover 2m",
            "description": "Simple takeoff, hover, and land test",
            "actions": [
                {"type": "takeoff", "alt": 2.0},
                {"type": "hover", "duration": 3.0},
                {"type": "land"}
            ]
        }
    else:
        with open(mission_path) as f:
            mission = json.load(f)

    # Create runner
    runner = MissionRunner(gui=True)

    # Handle Ctrl+C
    def signal_handler(sig, frame):
        print("\n\n⚠ Interrupted!")
        runner.disarm()
        runner.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        runner.run_mission(mission)
        time.sleep(2)  # Keep window open briefly
    finally:
        runner.close()


if __name__ == '__main__':
    main()
