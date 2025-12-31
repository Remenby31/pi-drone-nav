#!/usr/bin/env python3
"""
Run a mission in simulation with real-time display

Usage:
    python simulation/scripts/run_mission.py [mission_file]
"""

import sys
import time
import math
import json
import signal
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from simulation.sitl.betaflight_process import BetaflightSITL
from simulation.gazebo.betaflight_gazebo_bridge import BetaflightGazeboBridge


class MissionRunner:
    """Run missions in simulation"""

    def __init__(self):
        self.sitl = BetaflightSITL()
        self.bridge = BetaflightGazeboBridge()
        self.running = False

        # Flight state
        self.armed = False
        self.throttle = 1000
        self.roll = 1500
        self.pitch = 1500
        self.yaw = 1500

        # Mission state
        self.mission_active = False
        self.current_action = None

    def start(self) -> bool:
        """Start simulation"""
        print("\n" + "=" * 60)
        print("  🚁 Pi Drone Nav - Mission Simulator")
        print("=" * 60)

        print("\n[1/2] Starting Betaflight SITL...")
        if not self.sitl.start(timeout=10):
            print("ERROR: Failed to start SITL")
            return False
        print("      ✓ SITL running")

        print("\n[2/2] Starting physics bridge...")
        self.bridge.start()
        print("      ✓ Physics ready")

        self.running = True
        return True

    def stop(self):
        """Stop simulation"""
        self.running = False
        self.bridge.stop()
        self.sitl.stop()

    def send_rc(self):
        """Send current RC values"""
        channels = [
            self.roll,      # Roll
            self.pitch,     # Pitch
            self.throttle,  # Throttle
            self.yaw,       # Yaw
            1800 if self.armed else 1000,  # AUX1 (ARM)
            1500,  # AUX2
            1500,  # AUX3
            1500,  # AUX4
        ]
        self.sitl.send_rc_channels(channels)

    def arm(self):
        """Arm the drone"""
        print("\n🔓 ARMING...")
        self.throttle = 1000
        self.armed = True
        for _ in range(20):
            self.send_rc()
            time.sleep(0.05)
        print("   ✓ Armed")

    def disarm(self):
        """Disarm the drone"""
        print("\n🔒 DISARMING...")
        self.throttle = 1000
        self.armed = False
        for _ in range(20):
            self.send_rc()
            time.sleep(0.05)
        print("   ✓ Disarmed")

    def takeoff(self, target_alt: float):
        """Execute takeoff"""
        print(f"\n🛫 TAKEOFF to {target_alt}m...")
        self.current_action = f"Takeoff → {target_alt}m"

        # Ramp up throttle
        for t in range(1000, 1450, 10):
            self.throttle = t
            self.send_rc()
            self._display_state()
            time.sleep(0.05)

        # Hold and climb
        start = time.time()
        while time.time() - start < 3.0:
            state = self.bridge.get_state()

            # Simple altitude controller
            error = target_alt - state.z
            self.throttle = int(1350 + error * 100)
            self.throttle = max(1100, min(1600, self.throttle))

            self.send_rc()
            self._display_state()
            time.sleep(0.05)

        print(f"\n   ✓ Reached {state.z:.1f}m")

    def hover(self, duration: float):
        """Execute hover"""
        print(f"\n🚁 HOVER for {duration}s...")
        self.current_action = f"Hover {duration}s"

        start = time.time()
        while time.time() - start < duration:
            state = self.bridge.get_state()

            # Hold altitude
            target_alt = 2.0
            error = target_alt - state.z
            self.throttle = int(1350 + error * 100)
            self.throttle = max(1100, min(1600, self.throttle))

            # Hold position (attitude = centered)
            self.roll = 1500
            self.pitch = 1500

            self.send_rc()
            self._display_state()
            time.sleep(0.05)

        print(f"\n   ✓ Hover complete")

    def land(self):
        """Execute landing"""
        print("\n🛬 LANDING...")
        self.current_action = "Landing"

        # Gradual descent
        for t in range(50):
            state = self.bridge.get_state()

            # Reduce throttle gradually
            self.throttle = max(1000, 1350 - t * 7)

            self.send_rc()
            self._display_state()
            time.sleep(0.1)

            # Check if landed
            if state.z < 0.1 and abs(state.vz) < 0.1:
                break

        # Cut throttle
        self.throttle = 1000
        self.send_rc()
        print(f"\n   ✓ Landed")

    def _display_state(self):
        """Display current state"""
        state = self.bridge.get_state()

        # Build display
        bar_width = 20
        alt_bar = int(min(state.z / 5.0, 1.0) * bar_width)
        alt_display = "█" * alt_bar + "░" * (bar_width - alt_bar)

        throttle_pct = (self.throttle - 1000) / 10
        thr_bar = int(throttle_pct / 100 * bar_width)
        thr_display = "█" * thr_bar + "░" * (bar_width - thr_bar)

        motors = state.motors
        motor_str = f"[{motors[0]:.1f} {motors[1]:.1f} {motors[2]:.1f} {motors[3]:.1f}]"

        print(f"\r  Alt: {state.z:5.2f}m [{alt_display}] | "
              f"Thr: {throttle_pct:3.0f}% [{thr_display}] | "
              f"Motors: {motor_str} | "
              f"{self.current_action or ''}     ",
              end='', flush=True)

    def run_mission(self, mission: dict):
        """Run a mission"""
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
        input("Press ENTER to start mission...")
        print("-" * 60)

        self.mission_active = True

        # ARM
        self.arm()
        time.sleep(0.5)

        # Execute actions
        for i, action in enumerate(actions):
            action_type = action['type']
            print(f"\n>>> Action {i+1}/{len(actions)}: {action_type.upper()}")

            if action_type == 'takeoff':
                alt = action.get('alt', 2.0)
                self.takeoff(alt)

            elif action_type == 'hover':
                duration = action.get('duration', 3.0)
                self.hover(duration)

            elif action_type == 'land':
                self.land()

            elif action_type == 'goto':
                # Simplified goto (just hover for now)
                print(f"  → Going to ({action.get('lat')}, {action.get('lon')})")
                self.hover(2.0)

            else:
                print(f"  ⚠ Unknown action: {action_type}")

        # DISARM
        self.disarm()

        self.mission_active = False

        print("\n" + "=" * 60)
        print("  ✅ Mission Complete!")
        print("=" * 60)


def main():
    # Load mission
    mission_file = sys.argv[1] if len(sys.argv) > 1 else "config/missions/test_hover_2m.json"
    mission_path = Path(mission_file)

    if not mission_path.exists():
        print(f"ERROR: Mission file not found: {mission_file}")
        sys.exit(1)

    with open(mission_path) as f:
        mission = json.load(f)

    # Create runner
    runner = MissionRunner()

    # Handle Ctrl+C
    def signal_handler(sig, frame):
        print("\n\n⚠ Interrupted!")
        runner.disarm()
        runner.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Start simulation
    if not runner.start():
        sys.exit(1)

    try:
        # Run mission
        runner.run_mission(mission)
    finally:
        runner.stop()


if __name__ == '__main__':
    main()
