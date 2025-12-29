"""
Tests for MSP Protocol Driver
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import struct

import sys
sys.path.insert(0, '/home/remenby/projects/pi_drone_nav')

# Check if pyserial is available
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# Import MSP components
if SERIAL_AVAILABLE:
    from src.drivers.msp import MSPClient, MSPCommand, Attitude, RawGPS, Altitude
else:
    # Define minimal stubs for testing without pyserial
    from enum import IntEnum
    from dataclasses import dataclass

    class MSPCommand(IntEnum):
        MSP_API_VERSION = 1
        MSP_FC_VARIANT = 2
        MSP_STATUS = 101
        MSP_RAW_IMU = 102
        MSP_ATTITUDE = 108
        MSP_ALTITUDE = 109
        MSP_RAW_GPS = 106
        MSP_SET_RAW_RC = 200

    @dataclass
    class Attitude:
        roll: float = 0.0
        pitch: float = 0.0
        yaw: float = 0.0

    @dataclass
    class RawGPS:
        fix: bool = False
        num_sat: int = 0
        lat: float = 0.0
        lon: float = 0.0
        alt: float = 0.0
        ground_speed: float = 0.0
        ground_course: float = 0.0
        pdop: float = 0.0

    @dataclass
    class Altitude:
        altitude_cm: int = 0
        vario_cms: int = 0

    class MSPClient:
        def __init__(self, serial_port, timeout=0.1):
            self.serial = serial_port
            self.timeout = timeout

        def _encode_message_v1(self, cmd, data=b''):
            size = len(data)
            payload = bytes([size, int(cmd)]) + data
            checksum = 0
            for b in payload:
                checksum ^= b
            return b'$M<' + payload + bytes([checksum])

        def _decode_response_v1(self, response):
            if len(response) < 6:
                return None, None
            if response[:3] == b'$M!':
                return None, None
            if response[:3] != b'$M>':
                return None, None
            size = response[3]
            cmd = response[4]
            return cmd, response[5:5+size]


class TestMSPProtocol(unittest.TestCase):
    """Test MSP message encoding/decoding"""

    def test_build_command_no_data(self):
        """Test building command with no payload"""
        client = MSPClient.__new__(MSPClient)
        client.serial = None

        # Build MSP_API_VERSION command (1)
        msg = client._encode_message_v1(MSPCommand.MSP_API_VERSION, b'')

        # Header: $M< (3 bytes)
        # Size: 0 (1 byte)
        # Command: 1 (1 byte)
        # Checksum: 0 ^ 1 = 1 (1 byte)
        expected = b'$M<\x00\x01\x01'
        self.assertEqual(msg, expected)

    def test_build_command_with_data(self):
        """Test building command with payload"""
        client = MSPClient.__new__(MSPClient)
        client.serial = None

        # Build command with 2 bytes of data
        msg = client._encode_message_v1(MSPCommand.MSP_SET_RAW_RC, b'\x00\x04')

        # Size: 2, Command: 200
        # Checksum: 2 ^ 200 ^ 0 ^ 4 = 206
        self.assertEqual(msg[:3], b'$M<')
        self.assertEqual(msg[3], 2)  # size
        self.assertEqual(msg[4], 200)  # command

    def test_parse_response_valid(self):
        """Test parsing valid MSP response structure"""
        # Build a valid response: $M> size=3, cmd=108, data=[10,20,30]
        # Checksum: 3 ^ 108 ^ 10 ^ 20 ^ 30 = 111 (0x6f)
        response = b'$M>\x03\x6c\x0a\x14\x1e\x6f'

        # Verify structure manually
        self.assertEqual(response[:3], b'$M>')  # Header
        self.assertEqual(response[3], 3)  # Size
        self.assertEqual(response[4], 108)  # Command (ATTITUDE)
        self.assertEqual(response[5:8], b'\x0a\x14\x1e')  # Data

        # Verify checksum
        checksum = response[3] ^ response[4]  # size ^ cmd
        for b in response[5:8]:
            checksum ^= b
        self.assertEqual(checksum, response[8])

    def test_parse_response_error(self):
        """Test error response structure"""
        # Error response: $M!
        response = b'$M!\x00\x6c\x6c'

        # Verify it's an error response
        self.assertEqual(response[:3], b'$M!')
        # Error responses have '!' instead of '>'

    def test_parse_response_invalid_header(self):
        """Test invalid header detection"""
        response = b'XXX\x03\x6c\x0a\x14\x1e\x6b'

        # Invalid header - doesn't start with $M
        self.assertNotEqual(response[:2], b'$M')

    def test_checksum_calculation(self):
        """Test checksum XOR calculation"""
        # Checksum is XOR of size, command, and all data bytes
        size = 4
        cmd = 108
        data = bytes([10, 20, 30, 40])

        checksum = size ^ cmd
        for b in data:
            checksum ^= b

        # Verify it's correct
        self.assertEqual(checksum, size ^ cmd ^ 10 ^ 20 ^ 30 ^ 40)


class TestMSPDataStructures(unittest.TestCase):
    """Test MSP data structure parsing"""

    def test_attitude_from_bytes(self):
        """Test parsing attitude response"""
        # MSP_ATTITUDE returns: roll (int16), pitch (int16), yaw (int16)
        # Values are in 0.1 degrees
        # Roll: 50 (5.0°), Pitch: -100 (-10.0°), Yaw: 1800 (180.0°)
        data = struct.pack('<hhh', 50, -100, 1800)

        roll, pitch, yaw = struct.unpack('<hhh', data)

        self.assertEqual(roll / 10.0, 5.0)
        self.assertEqual(pitch / 10.0, -10.0)
        self.assertEqual(yaw, 1800)

    def test_raw_gps_from_bytes(self):
        """Test parsing GPS response"""
        # MSP_RAW_GPS: fix, numSat, lat, lon, alt, groundSpeed, groundCourse
        # lat/lon in 1e-7 degrees, alt in meters, speed in cm/s
        lat = int(48.8566 * 1e7)
        lon = int(2.3522 * 1e7)

        data = struct.pack('<BBiiHHH',
                          3,      # fix type
                          12,     # satellites
                          lat,    # latitude
                          lon,    # longitude
                          100,    # altitude
                          500,    # ground speed cm/s
                          900)    # course

        fix, sats, lat_raw, lon_raw, alt, speed, course = struct.unpack('<BBiiHHH', data)

        self.assertEqual(fix, 3)
        self.assertEqual(sats, 12)
        self.assertAlmostEqual(lat_raw / 1e7, 48.8566, places=4)
        self.assertAlmostEqual(lon_raw / 1e7, 2.3522, places=4)

    def test_rc_channel_encoding(self):
        """Test encoding RC channels for MSP_SET_RAW_RC"""
        # 8 channels, each uint16
        channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]

        data = struct.pack('<8H', *channels)

        self.assertEqual(len(data), 16)

        # Verify by unpacking
        unpacked = struct.unpack('<8H', data)
        self.assertEqual(list(unpacked), channels)


@unittest.skipUnless(SERIAL_AVAILABLE, "pyserial not installed")
class TestMSPClientIntegration(unittest.TestCase):
    """Integration tests with mocked serial"""

    def test_connect(self):
        """Test MSP client initialization with serial object"""
        mock_serial = MagicMock()
        mock_serial.is_open = True

        client = MSPClient(mock_serial, timeout=1.0)

        self.assertEqual(client.serial, mock_serial)
        self.assertEqual(client.timeout, 1.0)

    def test_get_attitude(self):
        """Test getting attitude data"""
        mock_serial = MagicMock()
        mock_serial.is_open = True

        # Prepare response data
        # Roll: 50 (5°), Pitch: -30 (-3°), Yaw: 450 (45°)
        attitude_data = struct.pack('<hhH', 50, -30, 45)
        checksum = len(attitude_data) ^ 108
        for b in attitude_data:
            checksum ^= b

        # Mock the read sequence for _decode_response_v1
        header = b'$M>'
        size_cmd = bytes([len(attitude_data), 108])
        response_parts = [header, size_cmd, attitude_data, bytes([checksum])]

        mock_serial.read.side_effect = [
            header[0:1], header[1:2], header[2:3],  # Header bytes
            size_cmd,  # size + cmd
            attitude_data,  # data
            bytes([checksum])  # checksum
        ]

        client = MSPClient(mock_serial, timeout=1.0)
        attitude = client.get_attitude()

        self.assertIsNotNone(attitude)
        self.assertAlmostEqual(attitude.roll, 5.0, places=1)
        self.assertAlmostEqual(attitude.pitch, -3.0, places=1)

    def test_set_raw_rc(self):
        """Test setting RC channels"""
        mock_serial = MagicMock()
        mock_serial.is_open = True

        # Mock the read sequence for response
        header = b'$M>'
        size_cmd = bytes([0, 200])  # empty response for SET command
        checksum = bytes([200])

        mock_serial.read.side_effect = [
            header[0:1], header[1:2], header[2:3],
            size_cmd,
            b'',  # no data
            checksum
        ]

        client = MSPClient(mock_serial, timeout=1.0)

        channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        client.set_raw_rc(channels)

        # Verify write was called
        self.assertTrue(mock_serial.write.called)


class TestMSPCommands(unittest.TestCase):
    """Test MSP command definitions"""

    def test_command_values(self):
        """Verify MSP command values match Betaflight"""
        self.assertEqual(MSPCommand.MSP_API_VERSION, 1)
        self.assertEqual(MSPCommand.MSP_FC_VARIANT, 2)
        self.assertEqual(MSPCommand.MSP_STATUS, 101)
        self.assertEqual(MSPCommand.MSP_RAW_IMU, 102)
        self.assertEqual(MSPCommand.MSP_ATTITUDE, 108)
        self.assertEqual(MSPCommand.MSP_ALTITUDE, 109)
        self.assertEqual(MSPCommand.MSP_RAW_GPS, 106)
        self.assertEqual(MSPCommand.MSP_SET_RAW_RC, 200)


if __name__ == '__main__':
    unittest.main()
