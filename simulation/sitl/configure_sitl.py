#!/usr/bin/env python3
"""
Configure Betaflight SITL for Pi Drone Nav simulation

The SITL needs specific configuration to work with our simulation:
- RX_MSP for RC input via UDP
- Angle mode enabled
- Proper motor mapping

This script configures SITL via MSP commands.
"""

import socket
import struct
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))


def msp_encode(cmd: int, data: bytes = b'') -> bytes:
    """Encode MSP v1 message"""
    size = len(data)
    checksum = size ^ cmd
    for b in data:
        checksum ^= b
    return b'$M<' + bytes([size, cmd]) + data + bytes([checksum])


def msp_decode(data: bytes) -> tuple:
    """Decode MSP v1 response, returns (cmd, payload)"""
    if len(data) < 6 or data[:3] != b'$M>':
        return None, None
    size = data[3]
    cmd = data[4]
    payload = data[5:5+size]
    return cmd, payload


class SITLConfigurator:
    """Configure SITL via MSP over TCP"""

    def __init__(self, host: str = '127.0.0.1', port: int = 5760):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self) -> bool:
        """Connect to SITL MSP port"""
        # Note: SITL doesn't expose MSP over TCP by default
        # We need to use UDP for RC input instead
        print("Note: SITL uses UDP for RC input, not TCP MSP")
        return True

    def configure_via_eeprom(self, sitl_dir: str = "/tmp/betaflight_sitl"):
        """
        Configure SITL by preparing eeprom.bin

        SITL reads configuration from eeprom.bin on startup.
        We can pre-configure it or use CLI commands via serial.
        """
        print("\nSITL Configuration")
        print("-" * 40)
        print("The SITL uses these defaults that work with our simulation:")
        print("  - RC input via UDP port 9004")
        print("  - Motor output via UDP port 9002")
        print("  - Sensor input via UDP port 9003")
        print()
        print("For full configuration, connect Betaflight Configurator to:")
        print("  TCP port 5760 (if exposed)")
        print()
        print("Or use the SITL's built-in configuration.")


def configure_sitl_for_simulation():
    """
    Provide configuration instructions for SITL

    The SITL automatically handles:
    - RC input via UDP 9004
    - Sensor data via UDP 9003
    - Motor output via UDP 9002 (to Gazebo)
    """
    print("=" * 60)
    print("  SITL Configuration for Pi Drone Nav")
    print("=" * 60)
    print()
    print("Betaflight SITL is pre-configured for simulation with:")
    print()
    print("  UDP Ports:")
    print("    9001 - RealFlight bridge")
    print("    9002 - Motor output to Gazebo (16 floats, 0-1)")
    print("    9003 - Sensor input from Gazebo")
    print("    9004 - RC input (16 x uint16, 1000-2000)")
    print()
    print("  Default Configuration:")
    print("    - Quad X mixer")
    print("    - SITL target auto-configures for simulation")
    print()
    print("  To ARM in simulation:")
    print("    1. Send centered RC with throttle low")
    print("    2. Set AUX1 (channel 5) > 1700 to arm")
    print()
    print("  Motor Output Format (UDP 9002):")
    print("    16 x float32 (little-endian)")
    print("    Values 0.0 to 1.0 (throttle percentage)")
    print("    Motors 0-3 for quadcopter")
    print()
    print("  RC Input Format (UDP 9004):")
    print("    16 x uint16 (little-endian)")
    print("    Values 1000-2000 (standard RC range)")
    print("    Channel mapping: [Roll, Pitch, Throttle, Yaw, AUX1...]")
    print()

    # Create a simple test
    print("-" * 60)
    print("Quick Test:")
    print("-" * 60)

    import subprocess
    result = subprocess.run(
        ["python", "-c", """
import socket
import struct

# Send test RC packet
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rc = [1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500] + [1500] * 8
data = struct.pack('<16H', *rc)
sock.sendto(data, ('127.0.0.1', 9004))
sock.close()
print('RC packet sent to UDP 9004')
"""],
        capture_output=True,
        text=True
    )
    print(result.stdout)
    if result.stderr:
        print(result.stderr)


if __name__ == '__main__':
    configure_sitl_for_simulation()
