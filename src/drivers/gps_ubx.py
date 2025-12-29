"""
GPS driver for u-blox modules (M8N, M10) using UBX protocol

Implements UBX binary protocol for high-performance GPS communication.
Provides position, velocity, and timing data at up to 10Hz.

References:
- u-blox M8 Protocol: https://www.u-blox.com/en/docs/UBX-13003221
- u-blox M10 Protocol: https://www.u-blox.com/en/docs/UBX-21022436
"""

import struct
import time
import threading
from enum import IntEnum
from dataclasses import dataclass, field
from typing import Optional, Callable, List, Tuple
import logging

logger = logging.getLogger(__name__)


class UBXClass(IntEnum):
    """UBX message class IDs"""
    NAV = 0x01
    RXM = 0x02
    INF = 0x04
    ACK = 0x05
    CFG = 0x06
    UPD = 0x09
    MON = 0x0A
    AID = 0x0B
    TIM = 0x0D
    ESF = 0x10
    MGA = 0x13
    LOG = 0x21
    SEC = 0x27
    HNR = 0x28


class UBXNavID(IntEnum):
    """NAV class message IDs"""
    POSLLH = 0x02
    STATUS = 0x03
    DOP = 0x04
    SOL = 0x06
    PVT = 0x07      # Position, Velocity, Time - most useful
    VELNED = 0x12
    TIMEGPS = 0x20
    TIMEUTC = 0x21
    SAT = 0x35


class UBXCfgID(IntEnum):
    """CFG class message IDs"""
    PRT = 0x00
    MSG = 0x01
    RST = 0x04
    RATE = 0x08
    CFG = 0x09
    NAV5 = 0x24
    NAVX5 = 0x23
    GNSS = 0x3E


class GPSFixType(IntEnum):
    """GPS fix types"""
    NO_FIX = 0
    DEAD_RECKONING = 1
    FIX_2D = 2
    FIX_3D = 3
    GPS_DEAD_RECKONING = 4
    TIME_ONLY = 5


@dataclass
class GPSFix:
    """Complete GPS fix data from NAV-PVT"""
    timestamp: float = 0.0           # Unix timestamp

    # Fix quality
    fix_type: GPSFixType = GPSFixType.NO_FIX
    fix_valid: bool = False
    num_satellites: int = 0

    # Position (WGS84)
    latitude: float = 0.0            # degrees
    longitude: float = 0.0           # degrees
    altitude_msl: float = 0.0        # meters above mean sea level
    altitude_ellipsoid: float = 0.0  # meters above WGS84 ellipsoid

    # Accuracy estimates
    horizontal_accuracy: float = 0.0  # meters
    vertical_accuracy: float = 0.0    # meters

    # Velocity (NED frame)
    vel_north: float = 0.0           # m/s
    vel_east: float = 0.0            # m/s
    vel_down: float = 0.0            # m/s
    ground_speed: float = 0.0        # m/s (2D)
    speed_3d: float = 0.0            # m/s (3D)

    # Heading
    heading: float = 0.0             # degrees (ground course)
    heading_accuracy: float = 0.0    # degrees

    # Dilution of precision
    pdop: float = 0.0
    hdop: float = 0.0
    vdop: float = 0.0

    # Time
    utc_year: int = 0
    utc_month: int = 0
    utc_day: int = 0
    utc_hour: int = 0
    utc_minute: int = 0
    utc_second: int = 0
    utc_nano: int = 0

    @property
    def has_fix(self) -> bool:
        return self.fix_type >= GPSFixType.FIX_2D and self.fix_valid

    @property
    def has_3d_fix(self) -> bool:
        return self.fix_type >= GPSFixType.FIX_3D and self.fix_valid


@dataclass
class GPSStatus:
    """GPS receiver status"""
    hw_version: str = ""
    sw_version: str = ""
    is_configured: bool = False
    update_rate_hz: int = 1


class GPSDriver:
    """
    u-blox GPS driver using UBX protocol

    Features:
    - Automatic configuration for high update rate
    - NAV-PVT parsing for complete position/velocity data
    - Thread-safe continuous reading
    - Callbacks for new fix data
    """

    UBX_SYNC1 = 0xB5
    UBX_SYNC2 = 0x62

    def __init__(self, serial_port, update_rate_hz: int = 10):
        """
        Initialize GPS driver

        Args:
            serial_port: pyserial Serial instance
            update_rate_hz: Desired update rate (1-10 Hz for M8N, 1-25 for M10)
        """
        self.serial = serial_port
        self.update_rate_hz = update_rate_hz

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Current fix data
        self._current_fix = GPSFix()
        self._last_fix_time = 0.0

        # Callbacks
        self._fix_callbacks: List[Callable[[GPSFix], None]] = []

        # Statistics
        self._message_count = 0
        self._error_count = 0

    def configure(self) -> bool:
        """
        Configure u-blox module for optimal performance

        - Disables NMEA output
        - Enables UBX NAV-PVT at specified rate
        - Sets navigation mode to airborne

        Returns:
            True if configuration successful
        """
        try:
            # Set update rate
            rate_ms = 1000 // self.update_rate_hz
            self._send_cfg_rate(rate_ms)
            time.sleep(0.1)

            # Disable all NMEA messages
            self._disable_nmea()
            time.sleep(0.1)

            # Enable NAV-PVT
            self._enable_message(UBXClass.NAV, UBXNavID.PVT, 1)
            time.sleep(0.1)

            # Set airborne mode (<4g)
            self._set_nav_mode(airborne=True)
            time.sleep(0.1)

            # Save configuration
            self._save_config()

            logger.info(f"GPS configured for {self.update_rate_hz}Hz update rate")
            return True

        except Exception as e:
            logger.error(f"GPS configuration failed: {e}")
            return False

    def start(self):
        """Start continuous GPS reading in background thread"""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        logger.info("GPS driver started")

    def stop(self):
        """Stop GPS reading"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        logger.info("GPS driver stopped")

    def get_fix(self) -> GPSFix:
        """Get current GPS fix (thread-safe)"""
        with self._lock:
            return self._current_fix

    def add_fix_callback(self, callback: Callable[[GPSFix], None]):
        """Add callback for new GPS fix data"""
        self._fix_callbacks.append(callback)

    def remove_fix_callback(self, callback: Callable[[GPSFix], None]):
        """Remove fix callback"""
        if callback in self._fix_callbacks:
            self._fix_callbacks.remove(callback)

    @property
    def fix_age_ms(self) -> float:
        """Time since last valid fix in milliseconds"""
        return (time.time() - self._last_fix_time) * 1000

    # ==================== UBX Protocol ====================

    def _calculate_checksum(self, data: bytes) -> Tuple[int, int]:
        """Calculate UBX checksum (Fletcher-8)"""
        ck_a = 0
        ck_b = 0
        for byte in data:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return ck_a, ck_b

    def _send_ubx(self, msg_class: int, msg_id: int, payload: bytes = b''):
        """Send UBX message"""
        length = len(payload)
        header = struct.pack('<BBBBH', self.UBX_SYNC1, self.UBX_SYNC2,
                            msg_class, msg_id, length)
        data_for_checksum = header[2:] + payload
        ck_a, ck_b = self._calculate_checksum(data_for_checksum)

        message = header + payload + bytes([ck_a, ck_b])
        self.serial.write(message)
        self.serial.flush()

    def _send_cfg_rate(self, rate_ms: int):
        """Set measurement rate"""
        # CFG-RATE: measRate, navRate, timeRef
        payload = struct.pack('<HHH', rate_ms, 1, 1)  # GPS time
        self._send_ubx(UBXClass.CFG, UBXCfgID.RATE, payload)

    def _disable_nmea(self):
        """Disable all NMEA messages"""
        nmea_messages = [
            (0xF0, 0x00),  # GGA
            (0xF0, 0x01),  # GLL
            (0xF0, 0x02),  # GSA
            (0xF0, 0x03),  # GSV
            (0xF0, 0x04),  # RMC
            (0xF0, 0x05),  # VTG
        ]
        for msg_class, msg_id in nmea_messages:
            self._enable_message(msg_class, msg_id, 0)

    def _enable_message(self, msg_class: int, msg_id: int, rate: int):
        """Enable/disable message at specified rate"""
        # CFG-MSG: class, id, rate (for current port)
        payload = struct.pack('<BBB', msg_class, msg_id, rate)
        self._send_ubx(UBXClass.CFG, UBXCfgID.MSG, payload)

    def _set_nav_mode(self, airborne: bool = True):
        """Set navigation mode"""
        # CFG-NAV5
        # dynModel: 0=portable, 6=airborne<1g, 7=airborne<2g, 8=airborne<4g
        dyn_model = 8 if airborne else 0

        payload = struct.pack('<HBBiIbBHHHHBBBBIIBBBBH',
            0x05,           # mask: apply dynModel and fixMode
            dyn_model,      # dynModel
            3,              # fixMode: auto 2D/3D
            0,              # fixedAlt
            10000,          # fixedAltVar
            5,              # minElev
            0,              # drLimit
            250,            # pDop
            250,            # tDop
            100,            # pAcc
            300,            # tAcc
            0,              # staticHoldThresh
            60,             # dgnssTimeout
            0,              # cnoThreshNumSVs
            0,              # cnoThresh
            0, 0,           # reserved
            0,              # staticHoldMaxDist
            0,              # utcStandard
            0, 0, 0, 0      # reserved
        )
        self._send_ubx(UBXClass.CFG, UBXCfgID.NAV5, payload)

    def _save_config(self):
        """Save configuration to flash"""
        # CFG-CFG: clear, save, load mask
        payload = struct.pack('<III', 0, 0x1F, 0)  # Save all
        self._send_ubx(UBXClass.CFG, UBXCfgID.CFG, payload)

    # ==================== Message Parsing ====================

    def _read_loop(self):
        """Background thread for continuous GPS reading"""
        buffer = bytearray()

        while self._running:
            try:
                # Read available data
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    buffer.extend(data)

                    # Process complete messages
                    while len(buffer) >= 8:
                        msg = self._parse_ubx_message(buffer)
                        if msg is None:
                            break
                else:
                    time.sleep(0.001)  # 1ms sleep if no data

            except Exception as e:
                logger.error(f"GPS read error: {e}")
                self._error_count += 1
                time.sleep(0.1)

    def _parse_ubx_message(self, buffer: bytearray) -> Optional[bytes]:
        """
        Parse UBX message from buffer

        Returns:
            Parsed payload or None if incomplete
        """
        # Find sync bytes
        while len(buffer) >= 2:
            if buffer[0] == self.UBX_SYNC1 and buffer[1] == self.UBX_SYNC2:
                break
            buffer.pop(0)

        if len(buffer) < 8:
            return None

        # Parse header
        msg_class = buffer[2]
        msg_id = buffer[3]
        length = struct.unpack('<H', buffer[4:6])[0]

        # Check if complete message available
        total_length = 8 + length  # sync(2) + class(1) + id(1) + len(2) + payload + ck(2)
        if len(buffer) < total_length:
            return None

        # Extract payload and checksum
        payload = bytes(buffer[6:6+length])
        ck_a = buffer[6+length]
        ck_b = buffer[6+length+1]

        # Verify checksum
        data_for_checksum = bytes(buffer[2:6+length])
        expected_ck_a, expected_ck_b = self._calculate_checksum(data_for_checksum)

        if ck_a != expected_ck_a or ck_b != expected_ck_b:
            logger.warning(f"UBX checksum error")
            del buffer[:1]  # Skip one byte and retry
            return None

        # Remove parsed message from buffer
        del buffer[:total_length]

        # Process message
        self._message_count += 1
        self._handle_message(msg_class, msg_id, payload)

        return payload

    def _handle_message(self, msg_class: int, msg_id: int, payload: bytes):
        """Handle parsed UBX message"""
        if msg_class == UBXClass.NAV:
            if msg_id == UBXNavID.PVT:
                self._parse_nav_pvt(payload)

    def _parse_nav_pvt(self, payload: bytes):
        """Parse NAV-PVT message (92 bytes)"""
        if len(payload) < 92:
            logger.warning(f"NAV-PVT payload too short: {len(payload)}")
            return

        # Unpack NAV-PVT structure
        (
            iTOW,           # GPS time of week (ms)
            year, month, day, hour, minute, second,
            valid,          # Validity flags
            tAcc,           # Time accuracy (ns)
            nano,           # Fraction of second (ns)
            fixType,        # Fix type
            flags,          # Fix flags
            flags2,         # Additional flags
            numSV,          # Number of satellites
            lon, lat,       # 1e-7 degrees
            height,         # Height above ellipsoid (mm)
            hMSL,           # Height above MSL (mm)
            hAcc, vAcc,     # Accuracy (mm)
            velN, velE, velD,  # Velocity NED (mm/s)
            gSpeed,         # Ground speed (mm/s)
            headMot,        # Heading of motion (1e-5 deg)
            sAcc,           # Speed accuracy (mm/s)
            headAcc,        # Heading accuracy (1e-5 deg)
            pDOP,           # Position DOP (0.01)
            flags3,         # More flags
            reserved1,
            headVeh,        # Vehicle heading (1e-5 deg)
            magDec, magAcc  # Magnetic declination
        ) = struct.unpack('<IHBBBBBBIiBBBBiiIIIIiiiIiIIHBBiHH', payload[:92])

        # Create fix object
        fix = GPSFix(
            timestamp=time.time(),

            fix_type=GPSFixType(fixType),
            fix_valid=bool(valid & 0x01),
            num_satellites=numSV,

            latitude=lat / 1e7,
            longitude=lon / 1e7,
            altitude_msl=hMSL / 1000.0,
            altitude_ellipsoid=height / 1000.0,

            horizontal_accuracy=hAcc / 1000.0,
            vertical_accuracy=vAcc / 1000.0,

            vel_north=velN / 1000.0,
            vel_east=velE / 1000.0,
            vel_down=velD / 1000.0,
            ground_speed=gSpeed / 1000.0,
            speed_3d=(velN**2 + velE**2 + velD**2)**0.5 / 1000.0,

            heading=headMot / 1e5,
            heading_accuracy=headAcc / 1e5,

            pdop=pDOP / 100.0,

            utc_year=year,
            utc_month=month,
            utc_day=day,
            utc_hour=hour,
            utc_minute=minute,
            utc_second=second,
            utc_nano=nano
        )

        # Update current fix
        with self._lock:
            self._current_fix = fix
            if fix.has_fix:
                self._last_fix_time = fix.timestamp

        # Notify callbacks
        for callback in self._fix_callbacks:
            try:
                callback(fix)
            except Exception as e:
                logger.error(f"GPS callback error: {e}")

    def close(self):
        """Close GPS driver"""
        self.stop()
        if self.serial:
            self.serial.close()


# Utility function for coordinate conversion
def gps_to_local_ned(lat: float, lon: float, alt: float,
                     ref_lat: float, ref_lon: float, ref_alt: float) -> Tuple[float, float, float]:
    """
    Convert GPS coordinates to local NED frame

    Args:
        lat, lon, alt: Current position (degrees, meters)
        ref_lat, ref_lon, ref_alt: Reference/origin position

    Returns:
        Tuple of (north, east, down) in meters
    """
    import math

    # WGS84 parameters
    EARTH_RADIUS = 6378137.0  # meters

    # Calculate deltas
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    ref_lat_rad = math.radians(ref_lat)

    # North-East calculation (flat Earth approximation, good for <10km)
    north = d_lat * EARTH_RADIUS
    east = d_lon * EARTH_RADIUS * math.cos(ref_lat_rad)
    down = -(alt - ref_alt)

    return north, east, down


def local_ned_to_gps(north: float, east: float, down: float,
                     ref_lat: float, ref_lon: float, ref_alt: float) -> Tuple[float, float, float]:
    """
    Convert local NED to GPS coordinates

    Args:
        north, east, down: Local position in meters
        ref_lat, ref_lon, ref_alt: Reference/origin position

    Returns:
        Tuple of (lat, lon, alt) in degrees and meters
    """
    import math

    EARTH_RADIUS = 6378137.0
    ref_lat_rad = math.radians(ref_lat)

    d_lat = north / EARTH_RADIUS
    d_lon = east / (EARTH_RADIUS * math.cos(ref_lat_rad))

    lat = ref_lat + math.degrees(d_lat)
    lon = ref_lon + math.degrees(d_lon)
    alt = ref_alt - down

    return lat, lon, alt
