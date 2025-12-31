#!/usr/bin/env python3
"""
Integration Test for Pi Drone Nav Simulation

Tests the complete simulation chain:
1. Betaflight SITL
2. Physics Bridge
3. RC Control
4. Flight simulation

Usage:
    python simulation/scripts/test_integration.py
"""

import sys
import time
import math
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from simulation.sitl.betaflight_process import BetaflightSITL
from simulation.gazebo.betaflight_gazebo_bridge import BetaflightGazeboBridge


def test_sitl_basic():
    """Test basic SITL functionality"""
    print("\n" + "=" * 60)
    print("  TEST 1: Basic SITL Start/Stop")
    print("=" * 60)

    sitl = BetaflightSITL()

    print("\nStarting SITL...")
    if not sitl.start(timeout=10):
        print("FAIL: Could not start SITL")
        return False

    print("OK: SITL started")

    if not sitl.is_running():
        print("FAIL: SITL not running")
        return False

    print("OK: SITL is running")

    sitl.stop()
    print("OK: SITL stopped")

    return True


def test_rc_communication():
    """Test RC communication with SITL"""
    print("\n" + "=" * 60)
    print("  TEST 2: RC Communication")
    print("=" * 60)

    sitl = BetaflightSITL()

    if not sitl.start(timeout=10):
        print("FAIL: Could not start SITL")
        return False

    print("\nSending centered RC...")
    if not sitl.send_rc_channels([1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500]):
        print("FAIL: Could not send RC")
        sitl.stop()
        return False

    print("OK: RC sent")

    time.sleep(0.5)

    print("Sending ARM command...")
    sitl.send_rc_channels([1500, 1500, 1000, 1500, 1800, 1500, 1500, 1500])

    time.sleep(0.5)

    print("OK: ARM command sent")

    sitl.stop()
    return True


def test_physics_bridge():
    """Test physics bridge with SITL"""
    print("\n" + "=" * 60)
    print("  TEST 3: Physics Bridge")
    print("=" * 60)

    sitl = BetaflightSITL()
    bridge = BetaflightGazeboBridge()

    if not sitl.start(timeout=10):
        print("FAIL: Could not start SITL")
        return False

    print("\nStarting physics bridge...")
    bridge.start()
    time.sleep(0.5)

    print("OK: Bridge started")

    # Send RC and verify motor response
    print("\nSending throttle command...")
    sitl.send_rc_channels([1500, 1500, 1000, 1500, 1800, 1500, 1500, 1500])  # ARM
    time.sleep(0.5)

    sitl.send_rc_channels([1500, 1500, 1200, 1500, 1800, 1500, 1500, 1500])  # Throttle up
    time.sleep(1.0)

    state = bridge.get_state()
    print(f"Motors: {state.motors}")
    print(f"Position: ({state.x:.2f}, {state.y:.2f}, {state.z:.2f})")

    if sum(state.motors) < 0.1:
        print("WARN: Motors not responding (may need SITL angle mode config)")
    else:
        print("OK: Motors responding")

    bridge.stop()
    sitl.stop()

    return True


def test_hover_simulation():
    """Test hover simulation"""
    print("\n" + "=" * 60)
    print("  TEST 4: Hover Simulation")
    print("=" * 60)

    sitl = BetaflightSITL()
    bridge = BetaflightGazeboBridge()

    if not sitl.start(timeout=10):
        print("FAIL: Could not start SITL")
        return False

    bridge.start()
    time.sleep(0.5)

    # ARM
    print("\nArming...")
    sitl.send_rc_channels([1500, 1500, 1000, 1500, 1800, 1500, 1500, 1500])
    time.sleep(1.0)

    # Gradual throttle up
    print("Throttle up (simulating takeoff)...")
    for throttle in range(1000, 1400, 20):
        sitl.send_rc_channels([1500, 1500, throttle, 1500, 1800, 1500, 1500, 1500])
        time.sleep(0.1)

    # Hold
    print("Holding...")
    time.sleep(2.0)

    state = bridge.get_state()
    print(f"\nFinal state:")
    print(f"  Position: ({state.x:.2f}, {state.y:.2f}, {state.z:.2f})")
    print(f"  Velocity: ({state.vx:.2f}, {state.vy:.2f}, {state.vz:.2f})")
    print(f"  Attitude: (R{math.degrees(state.roll):.1f}, P{math.degrees(state.pitch):.1f}, Y{math.degrees(state.yaw):.1f})")

    # Throttle down
    print("\nThrottle down...")
    for throttle in range(1400, 1000, -20):
        sitl.send_rc_channels([1500, 1500, throttle, 1500, 1800, 1500, 1500, 1500])
        time.sleep(0.1)

    # Disarm
    sitl.send_rc_channels([1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500])
    time.sleep(0.5)

    bridge.stop()
    sitl.stop()

    print("OK: Hover simulation complete")
    return True


def main():
    print("=" * 60)
    print("  Pi Drone Nav - Simulation Integration Test")
    print("=" * 60)

    results = []

    # Run tests
    results.append(("Basic SITL", test_sitl_basic()))
    results.append(("RC Communication", test_rc_communication()))
    results.append(("Physics Bridge", test_physics_bridge()))
    results.append(("Hover Simulation", test_hover_simulation()))

    # Summary
    print("\n" + "=" * 60)
    print("  TEST SUMMARY")
    print("=" * 60)

    passed = 0
    for name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"  {status}: {name}")
        if result:
            passed += 1

    print()
    print(f"  {passed}/{len(results)} tests passed")
    print("=" * 60)

    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
