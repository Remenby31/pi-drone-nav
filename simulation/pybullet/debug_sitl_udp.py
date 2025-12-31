#!/usr/bin/env python3
"""
Debug SITL UDP communication - verify packet sizes and connectivity.

Run SITL first: ~/projects/betaflight/obj/betaflight_SITL 127.0.0.1
"""

import socket
import struct
import time
import sys
import ctypes

# SITL ports
PORT_PWM = 9002       # Out (SITL -> Bridge) - motor values
PORT_STATE = 9003     # In (Bridge -> SITL) - FDM sensor data
PORT_RC = 9004        # In (Bridge -> SITL) - RC channels

# Expected packet sizes (from Betaflight target.h)
EXPECTED_FDM_SIZE = 144  # 18 doubles
EXPECTED_RC_SIZE = 40    # 1 double + 16 uint16

def create_fdm_packet(timestamp: float) -> bytes:
    """Create fdm_packet matching C struct layout"""
    # fdm_packet structure:
    # double timestamp                      8 bytes
    # double imu_angular_velocity_rpy[3]   24 bytes
    # double imu_linear_acceleration_xyz[3] 24 bytes
    # double imu_orientation_quat[4]       32 bytes
    # double velocity_xyz[3]               24 bytes
    # double position_xyz[3]               24 bytes
    # double pressure                       8 bytes
    # Total: 144 bytes
    return struct.pack(
        '<18d',
        timestamp,          # timestamp
        0.0, 0.0, 0.0,      # gyro x, y, z (rad/s)
        0.0, 0.0, -9.81,    # acc x, y, z (m/s² NED body frame)
        1.0, 0.0, 0.0, 0.0, # quaternion w, x, y, z (identity = level)
        0.0, 0.0, 0.0,      # velocity ENU (Ve, Vn, Vup)
        1.389879, 43.483061, 10.0,  # position: lon, lat, alt (ENU for GPS)
        101325.0            # pressure (Pa)
    )

def create_rc_packet(timestamp: float, throttle: int = 1000, arm: bool = False) -> bytes:
    """Create rc_packet matching C struct layout"""
    # rc_packet structure:
    # double timestamp                       8 bytes
    # uint16_t channels[16]                 32 bytes
    # Total: 40 bytes
    aux1 = 1800 if arm else 1000  # ARM switch
    channels = [
        1500,      # 0: Aileron
        1500,      # 1: Elevator
        throttle,  # 2: Throttle
        1500,      # 3: Rudder
        aux1,      # 4: AUX1 (ARM)
        1500,      # 5: AUX2
        1500,      # 6: AUX3
        1500,      # 7: AUX4
        1500, 1500, 1500, 1500,  # 8-11
        1500, 1500, 1500, 1500   # 12-15
    ]
    return struct.pack('<d16H', timestamp, *channels)

def main():
    print("=" * 60)
    print("  SITL UDP Debug Test")
    print("=" * 60)

    # Verify packet sizes
    fdm_test = create_fdm_packet(0.0)
    rc_test = create_rc_packet(0.0)

    print(f"\nPacket size verification:")
    print(f"  FDM packet: {len(fdm_test)} bytes (expected {EXPECTED_FDM_SIZE})", end="")
    print(" ✓" if len(fdm_test) == EXPECTED_FDM_SIZE else " ✗ SIZE MISMATCH!")
    print(f"  RC packet:  {len(rc_test)} bytes (expected {EXPECTED_RC_SIZE})", end="")
    print(" ✓" if len(rc_test) == EXPECTED_RC_SIZE else " ✗ SIZE MISMATCH!")

    if len(fdm_test) != EXPECTED_FDM_SIZE or len(rc_test) != EXPECTED_RC_SIZE:
        print("\n[!] Packet size mismatch - SITL will ignore these packets!")
        return

    print("\n" + "-" * 60)
    print("Testing UDP connectivity...")
    print("-" * 60)

    # Check if SITL port is open
    print(f"\nChecking if SITL is listening on port {PORT_STATE}...")

    # Create sockets
    fdm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    motor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    motor_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    motor_sock.bind(('0.0.0.0', PORT_PWM))
    motor_sock.settimeout(2.0)

    print(f"  Bound motor receiver to port {PORT_PWM}")

    print("\n" + "-" * 60)
    print("Sending test packets...")
    print("-" * 60)

    start_time = time.time()

    # Send multiple packets quickly to trigger SITL debug output
    for i in range(20):
        t = time.time() - start_time

        # Send FDM
        fdm = create_fdm_packet(t)
        sent = fdm_sock.sendto(fdm, ('127.0.0.1', PORT_STATE))

        # Send RC (with ARM on)
        rc = create_rc_packet(t, throttle=1000, arm=True)
        rc_sock.sendto(rc, ('127.0.0.1', PORT_RC))

        if i == 0:
            print(f"  Sent FDM #{i}: {sent} bytes to 127.0.0.1:{PORT_STATE}")
            print(f"  Sent RC  #{i}: {len(rc)} bytes to 127.0.0.1:{PORT_RC}")
            print(f"\n  FDM hex (first 32 bytes): {fdm[:32].hex()}")
            print(f"  RC  hex (first 32 bytes): {rc[:32].hex()}")

        time.sleep(0.05)

    print(f"\nTotal sent: 20 FDM + 20 RC packets")

    print("\n" + "-" * 60)
    print("Waiting for motor packets from SITL...")
    print("-" * 60)

    motor_count = 0
    try:
        for _ in range(5):
            try:
                data, addr = motor_sock.recvfrom(64)
                motor_count += 1
                if len(data) >= 16:
                    m = struct.unpack('<4f', data[:16])
                    print(f"  Motor packet from {addr}: {m[0]:.3f} {m[1]:.3f} {m[2]:.3f} {m[3]:.3f}")
            except socket.timeout:
                break
    except KeyboardInterrupt:
        pass

    print("\n" + "=" * 60)
    print("  Results")
    print("=" * 60)

    if motor_count > 0:
        print(f"\n  Received {motor_count} motor packets from SITL")
        print("  ✓ UDP communication is working!")
    else:
        print("\n  ✗ No motor packets received from SITL")
        print("\n  Troubleshooting:")
        print("  1. Make sure SITL is running: ~/projects/betaflight/obj/betaflight_SITL 127.0.0.1")
        print("  2. Check SITL output for '[SITL] new fdm' and '[SITL] new rc' messages")
        print("  3. If no messages, packet format may be wrong or SITL may not be listening")
        print("  4. Try: netstat -an | grep 900  (to see if ports are open)")

    fdm_sock.close()
    rc_sock.close()
    motor_sock.close()

if __name__ == "__main__":
    main()
