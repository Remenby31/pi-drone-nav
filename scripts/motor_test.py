#!/usr/bin/env python3
"""
Motor Test Script for Pi Drone Navigation

Tests MSP communication and motor control with DAKEFPVH743 or compatible FC.

WARNING: REMOVE ALL PROPELLERS BEFORE RUNNING!

Usage:
    python scripts/motor_test.py [--port /dev/ttyACM0] [--test-motors]
"""

import argparse
import sys
import time
import signal

# Add src to path
sys.path.insert(0, '.')

import serial
from src.drivers.msp import MSPClient, MSPError


def signal_handler(signum, frame, msp=None):
    """Handle Ctrl+C - stop motors immediately"""
    print("\n\nInterrupted! Stopping motors...")
    if msp:
        try:
            msp.stop_all_motors()
        except:
            pass
    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='Test MSP motor control')
    parser.add_argument('--port', default='/dev/ttyACM0',
                        help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baudrate', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--test-motors', action='store_true',
                        help='Run sequential motor test with ramp')
    parser.add_argument('--max-value', type=int, default=1200,
                        help='Max motor value for test (default: 1200)')
    args = parser.parse_args()

    print("=" * 50)
    print("PI DRONE NAV - MSP Motor Test")
    print("=" * 50)
    print(f"\nPort: {args.port}")
    print("\nWARNING: Ensure propellers are REMOVED!")
    print()

    # Open serial connection
    try:
        ser = serial.Serial(args.port, args.baudrate, timeout=1)
        time.sleep(0.1)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"Error opening port: {e}")
        print("Try: sudo chmod 666 /dev/ttyACM0")
        sys.exit(1)

    # Create MSP client
    msp = MSPClient(ser, timeout=1.0)

    # Setup signal handler for Ctrl+C
    signal.signal(signal.SIGINT, lambda s, f: signal_handler(s, f, msp))

    try:
        # Test connection
        print("1. Testing MSP connection...")
        if not msp.connect():
            print("   FAILED - No response from FC")
            sys.exit(1)

        api_version = msp.get_api_version()
        fc_variant = msp.get_fc_variant()
        print(f"   Connected: {fc_variant}, API {api_version}")

        # Read current state
        print("\n2. Reading current state...")
        try:
            rc = msp.get_rc_channels()
            print(f"   RC Channels: {rc[:4]} (AETR)")
        except MSPError as e:
            print(f"   RC read failed: {e}")

        try:
            motors = msp.get_motor_values()
            print(f"   Motors: {motors[:4]}")
        except MSPError as e:
            print(f"   Motor read failed: {e}")

        # Motor test
        if args.test_motors:
            print("\n3. Running sequential motor test...")
            print(f"   Max value: {args.max_value}")
            print("   Press Ctrl+C to stop at any time\n")

            input("   Press ENTER to start (or Ctrl+C to cancel)...")

            msp.test_motors_sequential(max_value=args.max_value)

            print("\n   Motor test complete!")
        else:
            print("\n3. Quick motor pulse test...")
            for i in range(4):
                motor_values = [1000] * 8
                motor_values[i] = 1100
                print(f"   Motor {i+1}: pulse to 1100")
                msp.set_motor_test(motor_values)
                time.sleep(0.3)
                msp.stop_all_motors()
                time.sleep(0.2)

        # Final readings
        print("\n4. Final motor state:")
        motors = msp.get_motor_values()
        for i, m in enumerate(motors[:4]):
            print(f"   Motor {i+1}: {m}")

    except KeyboardInterrupt:
        print("\n\nInterrupted!")

    except Exception as e:
        print(f"\nError: {e}")

    finally:
        # Always stop motors
        print("\n--- Stopping all motors ---")
        try:
            msp.stop_all_motors()
            msp.stop_all_motors()  # Double send for safety
        except:
            pass
        msp.close()
        print("Done.")


if __name__ == "__main__":
    main()
