#!/usr/bin/env python3
"""
Test UDP communication with Betaflight SITL

This script sends test packets to SITL and verifies communication.
Run SITL first: ~/projects/betaflight/obj/betaflight_SITL 127.0.0.1
"""

import socket
import struct
import time
import sys

# SITL ports
PORT_PWM_RAW = 9001   # Out (SITL -> Bridge)
PORT_PWM = 9002       # Out (SITL -> Bridge) - motor values
PORT_STATE = 9003     # In (Bridge -> SITL) - FDM sensor data
PORT_RC = 9004        # In (Bridge -> SITL) - RC channels

def create_fdm_packet(timestamp: float) -> bytes:
    """Create a valid fdm_packet (144 bytes = 18 doubles)"""
    return struct.pack(
        '<18d',
        timestamp,          # timestamp
        0.0, 0.0, 0.0,      # gyro x, y, z (rad/s)
        0.0, 0.0, -9.81,    # acc x, y, z (m/s²) - gravity pointing down
        1.0, 0.0, 0.0, 0.0, # quaternion w, x, y, z (identity = level)
        0.0, 0.0, 0.0,      # velocity east, north, up (m/s)
        1.389879, 43.483061, 0.0,  # longitude, latitude, altitude
        101325.0            # pressure (Pa)
    )

def create_rc_packet(timestamp: float, throttle: int = 1000) -> bytes:
    """Create a valid rc_packet (40 bytes = 1 double + 16 uint16)"""
    channels = [1500, 1500, throttle, 1500,  # AETR
                1800, 1800, 1500, 1500,      # AUX1-4 (arm on)
                1500, 1500, 1500, 1500,      # AUX5-8
                1500, 1500, 1500, 1500]      # AUX9-12
    return struct.pack('<d16H', timestamp, *channels)

def main():
    print("=" * 60)
    print("  Betaflight SITL UDP Communication Test")
    print("=" * 60)
    print()
    print("Make sure SITL is running:")
    print("  ~/projects/betaflight/obj/betaflight_SITL 127.0.0.1")
    print()

    # Create sockets
    # Motor receiver (server - we listen)
    motor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    motor_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    motor_sock.bind(('0.0.0.0', PORT_PWM))
    motor_sock.settimeout(0.5)
    print(f"[✓] Motor receiver listening on UDP port {PORT_PWM}")

    # Sensor sender (client - we send to SITL)
    sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[✓] Sensor sender ready (target: 127.0.0.1:{PORT_STATE})")

    # RC sender (client - we send to SITL)
    rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[✓] RC sender ready (target: 127.0.0.1:{PORT_RC})")

    print()
    print("-" * 60)
    print("Starting communication test...")
    print("Watch SITL output for: '[SITL] new fdm' and '[SITL] new rc'")
    print("-" * 60)
    print()

    start_time = time.time()
    fdm_sent = 0
    rc_sent = 0
    motor_received = 0

    try:
        for i in range(200):  # 10 seconds at 20Hz
            timestamp = time.time() - start_time

            # Send FDM packet
            fdm_data = create_fdm_packet(timestamp)
            sensor_sock.sendto(fdm_data, ('127.0.0.1', PORT_STATE))
            fdm_sent += 1

            # Send RC packet
            rc_data = create_rc_packet(timestamp, throttle=1200)
            rc_sock.sendto(rc_data, ('127.0.0.1', PORT_RC))
            rc_sent += 1

            # Try to receive motor packet
            try:
                data, addr = motor_sock.recvfrom(64)
                if len(data) >= 16:
                    m0, m1, m2, m3 = struct.unpack('<4f', data[:16])
                    motor_received += 1

                    if motor_received == 1:
                        print(f"[!] First motor packet from {addr}")

                    if i % 10 == 0:
                        print(f"  [{i:3d}] Motors: {m0:.3f} {m1:.3f} {m2:.3f} {m3:.3f}")
            except socket.timeout:
                pass

            # Status update
            if i % 20 == 0 and i > 0:
                print(f"  Status: FDM sent={fdm_sent}, RC sent={rc_sent}, "
                      f"Motors rx={motor_received}")

            time.sleep(0.05)  # 20Hz

    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    finally:
        motor_sock.close()
        sensor_sock.close()
        rc_sock.close()

    print()
    print("=" * 60)
    print(f"  Results:")
    print(f"    FDM packets sent: {fdm_sent}")
    print(f"    RC packets sent: {rc_sent}")
    print(f"    Motor packets received: {motor_received}")
    print("=" * 60)

    if motor_received > 0:
        print("\n[✓] SITL communication is working!")
        print("    If motors are 0, check SITL arming configuration.")
    else:
        print("\n[✗] No motor packets received from SITL")
        print("    Make sure SITL is running with argument '127.0.0.1'")
        print("    Check SITL console for '[SITL] new fdm' messages")

if __name__ == "__main__":
    main()
