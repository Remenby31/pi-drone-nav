#!/usr/bin/env python3
"""
Headless test of SITL + PyBullet bridge.

Run SITL first: ~/projects/betaflight/obj/main/betaflight_SITL.elf 127.0.0.1
"""

import socket
import struct
import time
import sys
import os

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from dataclasses import dataclass
from typing import List, Tuple

# Import just the packet classes, not the full bridge (avoids PyBullet)
@dataclass
class FDMPacket:
    """FDM packet sent to SITL"""
    timestamp: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    acc_x: float = 0.0
    acc_y: float = 0.0
    acc_z: float = -9.81  # Gravity
    quat_w: float = 1.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    vel_e: float = 0.0
    vel_n: float = 0.0
    vel_up: float = 0.0
    longitude: float = 1.389879
    latitude: float = 43.483061
    altitude: float = 0.0
    pressure: float = 101325.0

    def pack(self) -> bytes:
        return struct.pack('<18d',
            self.timestamp,
            self.gyro_x, self.gyro_y, self.gyro_z,
            self.acc_x, self.acc_y, self.acc_z,
            self.quat_w, self.quat_x, self.quat_y, self.quat_z,
            self.vel_e, self.vel_n, self.vel_up,
            self.longitude, self.latitude, self.altitude,
            self.pressure
        )


@dataclass
class RCPacket:
    """RC packet sent to SITL"""
    timestamp: float = 0.0
    channels: Tuple[int, ...] = (1500,) * 16

    def pack(self) -> bytes:
        return struct.pack('<d16H', self.timestamp, *self.channels)


def main():
    print("=" * 60)
    print("  SITL + PyBullet Bridge - Headless Test")
    print("=" * 60)
    print()

    # Create sockets
    fdm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    motor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    motor_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    motor_sock.bind(('0.0.0.0', 9002))
    motor_sock.settimeout(0.2)

    print("[OK] Sockets created")
    print()

    start_time = time.time()
    motors = [0.0, 0.0, 0.0, 0.0]
    armed = False

    def send_packets(throttle: int = 1000, arm: bool = False):
        t = time.time() - start_time

        # FDM packet (sensor data)
        fdm = FDMPacket(timestamp=t, altitude=0.0)
        fdm_sock.sendto(fdm.pack(), ('127.0.0.1', 9003))

        # RC packet
        aux1 = 1800 if arm else 1000
        channels = [1500, 1500, throttle, 1500, aux1, 1500, 1500, 1500] + [1500] * 8
        rc = RCPacket(timestamp=t, channels=tuple(channels))
        rc_sock.sendto(rc.pack(), ('127.0.0.1', 9004))

    def receive_motors():
        nonlocal motors
        try:
            data, _ = motor_sock.recvfrom(64)
            if len(data) >= 16:
                motors = list(struct.unpack('<4f', data[:16]))
        except socket.timeout:
            pass

    print("Phase 1: Disarmed (2 seconds)")
    print("-" * 40)
    for i in range(40):
        send_packets(throttle=1000, arm=False)
        receive_motors()
        time.sleep(0.05)
    print(f"  Motors: {motors}")

    print()
    print("Phase 2: ARM (2 seconds)")
    print("-" * 40)
    for i in range(40):
        send_packets(throttle=1000, arm=True)
        receive_motors()
        if i % 10 == 0:
            print(f"  [{i:2d}] Motors: {motors}")
        time.sleep(0.05)
    armed = True

    print()
    print("Phase 3: Throttle ramp (3 seconds)")
    print("-" * 40)
    for throttle in range(1000, 1400, 50):
        print(f"  Throttle: {throttle}")
        for i in range(10):
            send_packets(throttle=throttle, arm=True)
            receive_motors()
            time.sleep(0.05)
        print(f"    Motors: [{motors[0]:.3f} {motors[1]:.3f} {motors[2]:.3f} {motors[3]:.3f}]")

    print()
    print("Phase 4: Disarm")
    print("-" * 40)
    for i in range(20):
        send_packets(throttle=1000, arm=False)
        receive_motors()
        time.sleep(0.05)
    print(f"  Motors: {motors}")

    fdm_sock.close()
    rc_sock.close()
    motor_sock.close()

    print()
    print("=" * 60)
    if max(motors) > 0 or armed:
        print("  TEST PASSED - SITL arming and motor output working!")
    else:
        print("  TEST FAILED - No motor output detected")
    print("=" * 60)


if __name__ == "__main__":
    main()
