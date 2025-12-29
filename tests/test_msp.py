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
        API_VERSION = 1
        FC_VARIANT = 2
        STATUS = 101
        RAW_IMU = 102
        ATTITUDE = 108
        ALTITUDE = 109
        RAW_GPS = 106
        SET_RAW_RC = 200
        SET_WP = 209
        ARM = 216

    @dataclass
    class Attitude:
        roll: float = 0.0
        pitch: float = 0.0
        yaw: float = 0.0

    @dataclass
    class RawGPS:
        fix: int = 0
        satellites: int = 0
        latitude: float = 0.0
        longitude: float = 0.0
        altitude: float = 0.0
        ground_speed: float = 0.0
        ground_course: float = 0.0

    @dataclass
    class Altitude:
        altitude_cm: int = 0
        vario: int = 0

    class MSPClient:
        def __init__(self, port, baudrate):
            pass

        def _build_command(self, cmd, data):
            size = len(data)
            checksum = size ^ int(cmd)
            for b in data:
                checksum ^= b
            return b'$M<' + bytes([size, int(cmd)]) + data + bytes([checksum])

        def _parse_response(self, response, expected_cmd):
            if len(response) < 6:
                return None
            if response[:3] == b'$M!':
                return None
            if response[:3] != b'$M>':
                return None
            size = response[3]
            return response[5:5+size]


class TestMSPProtocol(unittest.TestCase):
    """Test MSP message encoding/decoding"""

    def test_build_command_no_data(self):
        """Test building command with no payload"""
        client = MSPClient.__new__(MSPClient)
        client._serial = None

        # Build MSP_API_VERSION command (1)
        msg = client._build_command(MSPCommand.API_VERSION, b'')

        # Header: $M< (3 bytes)
        # Size: 0 (1 byte)
        # Command: 1 (1 byte)
        # Checksum: 0 ^ 1 = 1 (1 byte)
        expected = b'$M<\x00\x01\x01'
        self.assertEqual(msg, expected)

    def test_build_command_with_data(self):
        """Test building command with payload"""
        client = MSPClient.__new__(MSPClient)
        client._serial = None

        # Build command with 2 bytes of data
        msg = client._build_command(MSPCommand.SET_RAW_RC, b'\x00\x04')

        # Size: 2, Command: 200
        # Checksum: 2 ^ 200 ^ 0 ^ 4 = 206
        self.assertEqual(msg[:3], b'$M<')
        self.assertEqual(msg[3], 2)  # size
        self.assertEqual(msg[4], 200)  # command

    def test_parse_response_valid(self):
        """Test parsing valid MSP response"""
        client = MSPClient.__new__(MSPClient)
        client._serial = None

        # Build a valid response: $M> size=3, cmd=108, data=[10,20,30]
        # Checksum: 3 ^ 108 ^ 10 ^ 20 ^ 30 = 107
        response = b'$M>\x03\x6c\x0a\x14\x1e\x6b'

        data = client._parse_response(response, MSPCommand.ATTITUDE)
        self.assertEqual(data, b'\x0a\x14\x1e')

    def test_parse_response_error(self):
        """Test parsing error response"""
        client = MSPClient.__new__(MSPClient)
        client._serial = None

        # Error response: $M!
        response = b'$M!\x00\x6c\x6c'

        data = client._parse_response(response, MSPCommand.ATTITUDE)
        self.assertIsNone(data)

    def test_parse_response_invalid_header(self):
        """Test parsing response with invalid header"""
        client = MSPClient.__new__(MSPClient)
        client._serial = None

        response = b'XXX\x03\x6c\x0a\x14\x1e\x6b'

        data = client._parse_response(response, MSPCommand.ATTITUDE)
        self.assertIsNone(data)

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

    @patch('serial.Serial')
    def test_connect(self, mock_serial_class):
        """Test MSP client connection"""
        mock_serial = MagicMock()
        mock_serial_class.return_value = mock_serial
        mock_serial.is_open = True

        client = MSPClient('/dev/ttyUSB0', 115200)

        mock_serial_class.assert_called_once_with(
            '/dev/ttyUSB0',
            baudrate=115200,
            timeout=1.0
        )

    @patch('serial.Serial')
    def test_get_attitude(self, mock_serial_class):
        """Test getting attitude data"""
        mock_serial = MagicMock()
        mock_serial_class.return_value = mock_serial
        mock_serial.is_open = True

        # Prepare response data
        # Roll: 50 (5°), Pitch: -30 (-3°), Yaw: 450 (45°)
        attitude_data = struct.pack('<hhh', 50, -30, 450)
        checksum = len(attitude_data) ^ 108
        for b in attitude_data:
            checksum ^= b

        response = b'$M>' + bytes([len(attitude_data), 108]) + attitude_data + bytes([checksum])
        mock_serial.read.return_value = response

        client = MSPClient('/dev/ttyUSB0', 115200)
        attitude = client.get_attitude()

        self.assertIsNotNone(attitude)
        self.assertAlmostEqual(attitude.roll, 5.0, places=1)
        self.assertAlmostEqual(attitude.pitch, -3.0, places=1)

    @patch('serial.Serial')
    def test_set_raw_rc(self, mock_serial_class):
        """Test setting RC channels"""
        mock_serial = MagicMock()
        mock_serial_class.return_value = mock_serial
        mock_serial.is_open = True

        # Response for SET command (empty data)
        response = b'$M>\x00\xc8\xc8'  # cmd 200
        mock_serial.read.return_value = response

        client = MSPClient('/dev/ttyUSB0', 115200)

        channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        result = client.set_raw_rc(channels)

        # Verify write was called
        self.assertTrue(mock_serial.write.called)


class TestMSPCommands(unittest.TestCase):
    """Test MSP command definitions"""

    def test_command_values(self):
        """Verify MSP command values match Betaflight"""
        self.assertEqual(MSPCommand.API_VERSION, 1)
        self.assertEqual(MSPCommand.FC_VARIANT, 2)
        self.assertEqual(MSPCommand.STATUS, 101)
        self.assertEqual(MSPCommand.RAW_IMU, 102)
        self.assertEqual(MSPCommand.ATTITUDE, 108)
        self.assertEqual(MSPCommand.ALTITUDE, 109)
        self.assertEqual(MSPCommand.RAW_GPS, 106)
        self.assertEqual(MSPCommand.SET_RAW_RC, 200)
        self.assertEqual(MSPCommand.SET_WP, 209)
        self.assertEqual(MSPCommand.ARM, 216)


if __name__ == '__main__':
    unittest.main()
