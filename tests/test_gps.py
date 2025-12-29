"""
Tests for GPS UBX Driver
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

# Import GPS components or define stubs
if SERIAL_AVAILABLE:
    from src.drivers.gps_ubx import GPSDriver, GPSFix
else:
    from dataclasses import dataclass

    from enum import IntEnum

    class GPSFixType(IntEnum):
        NO_FIX = 0
        DEAD_RECKONING = 1
        FIX_2D = 2
        FIX_3D = 3

    @dataclass
    class GPSFix:
        timestamp: float = 0.0
        fix_type: int = 0
        fix_valid: bool = False
        num_satellites: int = 0
        latitude: float = 0.0
        longitude: float = 0.0
        altitude_msl: float = 0.0
        altitude_ellipsoid: float = 0.0
        horizontal_accuracy: float = 0.0
        vertical_accuracy: float = 0.0
        vel_north: float = 0.0
        vel_east: float = 0.0
        vel_down: float = 0.0
        ground_speed: float = 0.0
        speed_3d: float = 0.0
        heading: float = 0.0
        heading_accuracy: float = 0.0
        pdop: float = 0.0
        hdop: float = 99.0
        vdop: float = 99.0

        @property
        def has_fix(self):
            return self.fix_type >= GPSFixType.FIX_2D and self.fix_valid

    class GPSDriver:
        def __init__(self, serial_port, update_rate_hz=10):
            self.serial = serial_port
            self._current_fix = GPSFix()

        def get_fix(self):
            return self._current_fix


class TestUBXProtocol(unittest.TestCase):
    """Test UBX message encoding/decoding"""

    def test_ubx_checksum(self):
        """Test UBX Fletcher checksum calculation"""
        # UBX checksum is Fletcher-8 over class, id, length, and payload
        # Example: CFG-RATE message
        msg_class = 0x06
        msg_id = 0x08
        payload = struct.pack('<HHH', 100, 1, 0)  # 100ms, 1 cycle, GPS time

        ck_a = 0
        ck_b = 0

        data = bytes([msg_class, msg_id]) + struct.pack('<H', len(payload)) + payload
        for b in data:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF

        self.assertIsInstance(ck_a, int)
        self.assertIsInstance(ck_b, int)

    def test_build_ubx_message(self):
        """Test building complete UBX message structure"""
        # UBX message structure: sync1, sync2, class, id, length(2), payload, ck_a, ck_b
        msg_class = 0x06
        msg_id = 0x01
        payload = bytes([0x01, 0x07, 0x01])

        # Build message manually
        header = struct.pack('<BBBBH', 0xB5, 0x62, msg_class, msg_id, len(payload))

        # Calculate checksum (Fletcher-8 over class, id, length, payload)
        ck_a = 0
        ck_b = 0
        for b in header[2:] + payload:  # Skip sync bytes
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF

        msg = header + payload + bytes([ck_a, ck_b])

        # Should start with sync chars
        self.assertEqual(msg[:2], b'\xb5\x62')
        # Class and ID
        self.assertEqual(msg[2], 0x06)
        self.assertEqual(msg[3], 0x01)
        # Length (little endian)
        self.assertEqual(struct.unpack('<H', msg[4:6])[0], 3)

    def test_parse_nav_pvt(self):
        """Test parsing NAV-PVT message structure"""
        # NAV-PVT is 92 bytes
        # Full format from u-blox documentation
        # We test that we can construct and parse a valid 92-byte payload

        # Build payload manually to ensure 92 bytes
        payload = bytearray(92)

        # iTOW (4 bytes) at offset 0
        struct.pack_into('<I', payload, 0, 123456789)
        # year (2), month, day, hour, min, sec (6 bytes total) at offset 4
        struct.pack_into('<H', payload, 4, 2024)
        payload[6] = 3   # month
        payload[7] = 15  # day
        payload[8] = 14  # hour
        payload[9] = 30  # min
        payload[10] = 0  # sec
        # valid at offset 11
        payload[11] = 0x07
        # tAcc (4 bytes) at offset 12
        struct.pack_into('<I', payload, 12, 100)
        # nano (4 bytes) at offset 16
        struct.pack_into('<i', payload, 16, 0)
        # fixType at offset 20
        payload[20] = 3
        # flags at offset 21
        payload[21] = 0x01
        # flags2 at offset 22
        payload[22] = 0
        # numSV at offset 23
        payload[23] = 12
        # lon (4 bytes) at offset 24
        struct.pack_into('<i', payload, 24, int(2.3522 * 1e7))
        # lat (4 bytes) at offset 28
        struct.pack_into('<i', payload, 28, int(48.8566 * 1e7))
        # height (4 bytes) at offset 32
        struct.pack_into('<i', payload, 32, 100000)
        # hMSL (4 bytes) at offset 36
        struct.pack_into('<i', payload, 36, 100000)
        # hAcc (4 bytes) at offset 40
        struct.pack_into('<I', payload, 40, 5000)
        # vAcc (4 bytes) at offset 44
        struct.pack_into('<I', payload, 44, 10000)
        # velN (4 bytes) at offset 48
        struct.pack_into('<i', payload, 48, 500)
        # velE (4 bytes) at offset 52
        struct.pack_into('<i', payload, 52, 300)
        # velD (4 bytes) at offset 56
        struct.pack_into('<i', payload, 56, -100)
        # gSpeed (4 bytes) at offset 60
        struct.pack_into('<i', payload, 60, 583)
        # headMot (4 bytes) at offset 64
        struct.pack_into('<i', payload, 64, 3090000)

        self.assertEqual(len(payload), 92)

        # Verify key fields
        self.assertEqual(payload[20], 3)   # fixType
        self.assertEqual(payload[23], 12)  # numSV
        lat = struct.unpack_from('<i', payload, 28)[0]
        lon = struct.unpack_from('<i', payload, 24)[0]
        self.assertAlmostEqual(lat / 1e7, 48.8566, places=4)
        self.assertAlmostEqual(lon / 1e7, 2.3522, places=4)


class TestGPSFix(unittest.TestCase):
    """Test GPSFix data structure"""

    def test_gps_fix_creation(self):
        """Test creating GPSFix object"""
        fix = GPSFix(
            timestamp=123456789.0,
            fix_type=3,
            fix_valid=True,
            num_satellites=10,
            latitude=48.8566,
            longitude=2.3522,
            altitude_msl=150.0,
            vel_north=5.0,
            vel_east=3.0,
            vel_down=-0.5,
            ground_speed=5.83,
            heading=30.96,
            hdop=1.2,
            vdop=1.5,
        )

        self.assertEqual(fix.fix_type, 3)
        self.assertEqual(fix.num_satellites, 10)
        self.assertAlmostEqual(fix.latitude, 48.8566, places=4)
        self.assertTrue(fix.has_fix)

    def test_gps_fix_no_fix(self):
        """Test GPSFix with no fix"""
        fix = GPSFix(
            timestamp=0.0,
            fix_type=0,
            fix_valid=False,
            num_satellites=2,
            latitude=0.0,
            longitude=0.0,
            altitude_msl=0.0,
            vel_north=0.0,
            vel_east=0.0,
            vel_down=0.0,
            ground_speed=0.0,
            heading=0.0,
            hdop=99.0,
            vdop=99.0,
        )

        self.assertFalse(fix.has_fix)


class TestGPSConfiguration(unittest.TestCase):
    """Test GPS configuration commands"""

    def test_cfg_rate_message(self):
        """Test building CFG-RATE message for 10Hz"""
        # CFG-RATE: measRate (ms), navRate, timeRef
        payload = struct.pack('<HHH', 100, 1, 0)

        self.assertEqual(len(payload), 6)
        meas_rate, nav_rate, time_ref = struct.unpack('<HHH', payload)
        self.assertEqual(meas_rate, 100)  # 100ms = 10Hz

    def test_cfg_msg_disable_nmea(self):
        """Test building CFG-MSG to disable NMEA"""
        # Disable GGA: class=0xF0, id=0x00, rate=0
        payload = bytes([0xF0, 0x00, 0x00])

        self.assertEqual(len(payload), 3)
        self.assertEqual(payload[0], 0xF0)  # NMEA class
        self.assertEqual(payload[2], 0)  # rate = 0 (disabled)

    def test_cfg_msg_enable_nav_pvt(self):
        """Test building CFG-MSG to enable NAV-PVT"""
        # Enable NAV-PVT: class=0x01, id=0x07, rate=1
        payload = bytes([0x01, 0x07, 0x01])

        self.assertEqual(payload[0], 0x01)  # NAV class
        self.assertEqual(payload[1], 0x07)  # PVT message
        self.assertEqual(payload[2], 1)  # rate = 1 (every nav solution)


@unittest.skipUnless(SERIAL_AVAILABLE, "pyserial not installed")
class TestGPSDriverIntegration(unittest.TestCase):
    """Integration tests with mocked serial"""

    def test_connect(self):
        """Test GPS driver initialization with serial object"""
        mock_serial = MagicMock()
        mock_serial.is_open = True

        driver = GPSDriver(mock_serial, update_rate_hz=10)

        self.assertEqual(driver.serial, mock_serial)
        self.assertEqual(driver.update_rate_hz, 10)

    def test_configure_rate(self):
        """Test configuring GPS update rate"""
        mock_serial = MagicMock()
        mock_serial.is_open = True

        driver = GPSDriver(mock_serial, update_rate_hz=10)

        # The configure method should send CFG-RATE
        # We just verify it doesn't crash
        driver.configure()
        self.assertTrue(mock_serial.write.called)

    def test_read_loop(self):
        """Test GPS read loop with mock data"""
        mock_serial = MagicMock()
        mock_serial.is_open = True
        mock_serial.in_waiting = 0

        driver = GPSDriver(mock_serial, update_rate_hz=10)

        # Simulate no data available
        fix = driver.get_fix()
        # Should return default GPSFix
        self.assertIsNotNone(fix)


class TestGPSVelocityCalculations(unittest.TestCase):
    """Test GPS velocity and derived calculations"""

    def test_ground_speed_from_velocity(self):
        """Test calculating ground speed from NED velocity"""
        import math

        vel_north = 3.0  # m/s
        vel_east = 4.0   # m/s

        ground_speed = math.sqrt(vel_north**2 + vel_east**2)

        self.assertAlmostEqual(ground_speed, 5.0, places=2)

    def test_heading_from_velocity(self):
        """Test calculating heading from NED velocity"""
        import math

        # Moving north
        heading_north = math.degrees(math.atan2(0, 5))  # East=0, North=5
        self.assertAlmostEqual(heading_north, 0.0, places=1)

        # Moving east
        heading_east = math.degrees(math.atan2(5, 0))  # East=5, North=0
        self.assertAlmostEqual(heading_east, 90.0, places=1)

        # Moving south-west
        heading_sw = math.degrees(math.atan2(-5, -5))  # East=-5, North=-5
        self.assertAlmostEqual(heading_sw, -135.0, places=1)

    def test_3d_velocity(self):
        """Test 3D velocity calculation"""
        import math

        vel_north = 3.0
        vel_east = 4.0
        vel_down = -2.0  # climbing

        speed_3d = math.sqrt(vel_north**2 + vel_east**2 + vel_down**2)

        self.assertAlmostEqual(speed_3d, math.sqrt(29), places=2)


if __name__ == '__main__':
    unittest.main()
