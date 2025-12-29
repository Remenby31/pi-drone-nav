"""
MSP (MultiWii Serial Protocol) client for Betaflight

Implements MSP V1 and V2 protocol for communication with Betaflight flight controllers.
Supports both UART and USB connections.

References:
- Betaflight MSP: https://github.com/betaflight/betaflight/tree/master/src/main/msp
- MSP Protocol: https://github.com/iNavFlight/inav/wiki/MSP-V2
"""

import struct
import time
import threading
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional, Tuple, List, Union
import logging

logger = logging.getLogger(__name__)


class MSPError(Exception):
    """MSP communication error"""
    pass


class MSPCommand(IntEnum):
    """MSP command codes (from msp_protocol.h)"""

    # Identification
    MSP_API_VERSION = 1
    MSP_FC_VARIANT = 2
    MSP_FC_VERSION = 3
    MSP_BOARD_INFO = 4
    MSP_BUILD_INFO = 5

    # Status
    MSP_STATUS = 101
    MSP_RAW_IMU = 102
    MSP_SERVO = 103
    MSP_MOTOR = 104
    MSP_RC = 105
    MSP_RAW_GPS = 106
    MSP_COMP_GPS = 107
    MSP_ATTITUDE = 108
    MSP_ALTITUDE = 109
    MSP_ANALOG = 110

    # Configuration read
    MSP_STATUS_EX = 150
    MSP_UID = 160

    # Set commands
    MSP_SET_RAW_RC = 200
    MSP_SET_RAW_GPS = 201
    MSP_ACC_CALIBRATION = 205
    MSP_MAG_CALIBRATION = 206
    MSP_SET_HEADING = 211
    MSP_SET_MOTOR = 214

    # Control
    MSP_SET_ARMING_DISABLED = 99
    MSP_REBOOT = 68

    # EEPROM
    MSP_EEPROM_WRITE = 250


@dataclass
class Attitude:
    """Aircraft attitude from MSP_ATTITUDE"""
    roll: float      # degrees, -180 to 180
    pitch: float     # degrees, -180 to 180
    yaw: float       # degrees, 0 to 360


@dataclass
class RawGPS:
    """GPS data from MSP_RAW_GPS"""
    fix: bool
    num_sat: int
    lat: float       # degrees
    lon: float       # degrees
    alt: float       # meters
    ground_speed: float  # m/s
    ground_course: float  # degrees
    pdop: float      # position dilution of precision


@dataclass
class CompGPS:
    """Computed GPS data from MSP_COMP_GPS"""
    distance_to_home: int  # meters
    direction_to_home: int  # degrees
    gps_update: bool


@dataclass
class Altitude:
    """Altitude data from MSP_ALTITUDE"""
    altitude_cm: int      # cm
    vario_cms: int        # cm/s (vertical speed)

    @property
    def altitude_m(self) -> float:
        return self.altitude_cm / 100.0

    @property
    def vario_ms(self) -> float:
        return self.vario_cms / 100.0


@dataclass
class RawIMU:
    """IMU data from MSP_RAW_IMU"""
    acc_x: int
    acc_y: int
    acc_z: int
    gyro_x: int
    gyro_y: int
    gyro_z: int
    mag_x: int
    mag_y: int
    mag_z: int


@dataclass
class Analog:
    """Analog data from MSP_ANALOG"""
    vbat: float      # Volts
    mah_drawn: int   # mAh
    rssi: int        # 0-1023
    amperage: float  # Amps


@dataclass
class Status:
    """Status from MSP_STATUS"""
    cycle_time: int          # us
    i2c_error_count: int
    sensor_flags: int
    flight_mode_flags: int
    config_profile: int
    cpu_load: int            # percent


class MSPClient:
    """
    MSP protocol client for Betaflight communication

    Thread-safe implementation with support for concurrent reads.
    """

    # MSP protocol constants
    MSP_HEADER = b'$M'
    MSP_V2_HEADER = b'$X'
    MSP_DIRECTION_TO_FC = b'<'
    MSP_DIRECTION_FROM_FC = b'>'

    def __init__(self, serial_port, timeout: float = 0.1):
        """
        Initialize MSP client

        Args:
            serial_port: pyserial Serial instance or compatible
            timeout: Response timeout in seconds
        """
        self.serial = serial_port
        self.timeout = timeout
        self._lock = threading.Lock()
        self._connected = False

        # Cache for frequently requested data
        self._attitude_cache: Optional[Attitude] = None
        self._gps_cache: Optional[RawGPS] = None
        self._altitude_cache: Optional[Altitude] = None

    def connect(self) -> bool:
        """
        Verify connection by requesting API version

        Returns:
            True if connected and responding
        """
        try:
            version = self.get_api_version()
            if version:
                self._connected = True
                logger.info(f"Connected to Betaflight, API version: {version}")
                return True
        except Exception as e:
            logger.error(f"Connection failed: {e}")

        self._connected = False
        return False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def _calculate_checksum_v1(self, data: bytes) -> int:
        """Calculate MSP V1 checksum (XOR of all bytes)"""
        checksum = 0
        for b in data:
            checksum ^= b
        return checksum

    def _encode_message_v1(self, cmd: int, data: bytes = b'') -> bytes:
        """
        Encode MSP V1 message

        Format: $M< + size + cmd + data + checksum
        """
        size = len(data)
        payload = bytes([size, cmd]) + data
        checksum = self._calculate_checksum_v1(payload)
        return self.MSP_HEADER + self.MSP_DIRECTION_TO_FC + payload + bytes([checksum])

    def _decode_response_v1(self) -> Tuple[int, bytes]:
        """
        Decode MSP V1 response

        Returns:
            Tuple of (command, data)

        Raises:
            MSPError on timeout or invalid response
        """
        # Read header
        start_time = time.time()
        header = b''

        while len(header) < 3:
            if time.time() - start_time > self.timeout:
                raise MSPError("Timeout waiting for response header")

            byte = self.serial.read(1)
            if byte:
                header += byte

        if header[:2] != self.MSP_HEADER:
            raise MSPError(f"Invalid header: {header}")

        direction = header[2:3]
        if direction == b'!':
            raise MSPError("FC returned error response")

        # Read size and command
        size_cmd = self._read_bytes(2)
        size = size_cmd[0]
        cmd = size_cmd[1]

        # Read data
        data = self._read_bytes(size) if size > 0 else b''

        # Read and verify checksum
        received_checksum = self._read_bytes(1)[0]
        expected_checksum = self._calculate_checksum_v1(size_cmd + data)

        if received_checksum != expected_checksum:
            raise MSPError(f"Checksum mismatch: {received_checksum} != {expected_checksum}")

        return cmd, data

    def _read_bytes(self, count: int) -> bytes:
        """Read exact number of bytes with timeout"""
        data = b''
        start_time = time.time()

        while len(data) < count:
            if time.time() - start_time > self.timeout:
                raise MSPError(f"Timeout reading {count} bytes, got {len(data)}")

            chunk = self.serial.read(count - len(data))
            if chunk:
                data += chunk

        return data

    def send_command(self, cmd: MSPCommand, data: bytes = b'') -> bytes:
        """
        Send MSP command and wait for response

        Args:
            cmd: MSP command code
            data: Optional payload data

        Returns:
            Response data bytes

        Raises:
            MSPError on communication failure
        """
        with self._lock:
            # Clear input buffer
            self.serial.reset_input_buffer()

            # Send request
            message = self._encode_message_v1(cmd, data)
            self.serial.write(message)
            self.serial.flush()

            # Read response
            resp_cmd, resp_data = self._decode_response_v1()

            if resp_cmd != cmd:
                raise MSPError(f"Response command mismatch: {resp_cmd} != {cmd}")

            return resp_data

    def send_command_no_response(self, cmd: MSPCommand, data: bytes = b''):
        """Send command without waiting for response"""
        with self._lock:
            message = self._encode_message_v1(cmd, data)
            self.serial.write(message)
            self.serial.flush()

    # ==================== High-level API ====================

    def get_api_version(self) -> Optional[str]:
        """Get Betaflight API version"""
        try:
            data = self.send_command(MSPCommand.MSP_API_VERSION)
            if len(data) >= 3:
                protocol_version = data[0]
                api_major = data[1]
                api_minor = data[2]
                return f"{api_major}.{api_minor}"
        except MSPError:
            pass
        return None

    def get_fc_variant(self) -> Optional[str]:
        """Get flight controller variant (e.g., 'BTFL')"""
        try:
            data = self.send_command(MSPCommand.MSP_FC_VARIANT)
            return data[:4].decode('ascii')
        except MSPError:
            pass
        return None

    def get_attitude(self) -> Attitude:
        """
        Get current attitude (roll, pitch, yaw)

        Returns:
            Attitude dataclass with angles in degrees
        """
        data = self.send_command(MSPCommand.MSP_ATTITUDE)

        if len(data) < 6:
            raise MSPError(f"Invalid attitude data length: {len(data)}")

        # Roll and pitch in decidegrees, yaw in degrees
        roll, pitch, yaw = struct.unpack('<hhH', data[:6])

        attitude = Attitude(
            roll=roll / 10.0,    # decidegrees to degrees
            pitch=pitch / 10.0,
            yaw=float(yaw)
        )

        self._attitude_cache = attitude
        return attitude

    def get_raw_gps(self) -> RawGPS:
        """
        Get raw GPS data

        Returns:
            RawGPS dataclass with position and speed
        """
        data = self.send_command(MSPCommand.MSP_RAW_GPS)

        if len(data) < 16:
            raise MSPError(f"Invalid GPS data length: {len(data)}")

        fix = bool(data[0])
        num_sat = data[1]
        lat, lon = struct.unpack('<ii', data[2:10])
        alt, speed, course = struct.unpack('<HHH', data[10:16])

        # PDOP added in API 1.44
        pdop = 0
        if len(data) >= 18:
            pdop = struct.unpack('<H', data[16:18])[0]

        gps = RawGPS(
            fix=fix,
            num_sat=num_sat,
            lat=lat / 10000000.0,        # 1e-7 degrees to degrees
            lon=lon / 10000000.0,
            alt=float(alt),               # meters
            ground_speed=speed / 100.0,   # cm/s to m/s
            ground_course=course / 10.0,  # decidegrees to degrees
            pdop=pdop / 100.0             # centidop to dop
        )

        self._gps_cache = gps
        return gps

    def get_comp_gps(self) -> CompGPS:
        """
        Get computed GPS data (distance/direction to home)

        Returns:
            CompGPS dataclass
        """
        data = self.send_command(MSPCommand.MSP_COMP_GPS)

        if len(data) < 5:
            raise MSPError(f"Invalid comp GPS data length: {len(data)}")

        distance, direction = struct.unpack('<HH', data[:4])
        update = bool(data[4] & 1)

        return CompGPS(
            distance_to_home=distance,
            direction_to_home=direction * 10,  # Restored to original precision
            gps_update=update
        )

    def get_altitude(self) -> Altitude:
        """
        Get altitude from barometer/fusion

        Returns:
            Altitude dataclass with altitude and vario
        """
        data = self.send_command(MSPCommand.MSP_ALTITUDE)

        if len(data) < 6:
            raise MSPError(f"Invalid altitude data length: {len(data)}")

        alt_cm, vario = struct.unpack('<iH', data[:6])

        # Vario is signed but sent as unsigned
        if vario > 32767:
            vario -= 65536

        altitude = Altitude(
            altitude_cm=alt_cm,
            vario_cms=vario
        )

        self._altitude_cache = altitude
        return altitude

    def get_raw_imu(self) -> RawIMU:
        """
        Get raw IMU data (accelerometer, gyroscope, magnetometer)

        Returns:
            RawIMU dataclass with raw sensor values
        """
        data = self.send_command(MSPCommand.MSP_RAW_IMU)

        if len(data) < 18:
            raise MSPError(f"Invalid IMU data length: {len(data)}")

        values = struct.unpack('<9h', data[:18])

        return RawIMU(
            acc_x=values[0],
            acc_y=values[1],
            acc_z=values[2],
            gyro_x=values[3],
            gyro_y=values[4],
            gyro_z=values[5],
            mag_x=values[6],
            mag_y=values[7],
            mag_z=values[8]
        )

    def get_analog(self) -> Analog:
        """
        Get analog data (battery voltage, current, RSSI)

        Returns:
            Analog dataclass
        """
        data = self.send_command(MSPCommand.MSP_ANALOG)

        if len(data) < 7:
            raise MSPError(f"Invalid analog data length: {len(data)}")

        vbat = data[0]
        mah_drawn = struct.unpack('<H', data[1:3])[0]
        rssi = struct.unpack('<H', data[3:5])[0]
        amperage = struct.unpack('<h', data[5:7])[0]

        return Analog(
            vbat=vbat / 10.0,          # decivolts to volts
            mah_drawn=mah_drawn,
            rssi=rssi,
            amperage=amperage / 100.0   # centiamps to amps
        )

    def get_status(self) -> Status:
        """
        Get flight controller status

        Returns:
            Status dataclass
        """
        data = self.send_command(MSPCommand.MSP_STATUS)

        if len(data) < 11:
            raise MSPError(f"Invalid status data length: {len(data)}")

        cycle_time, i2c_errors, sensors, modes = struct.unpack('<HHHH', data[:8])
        profile = data[8]
        cpu_load = struct.unpack('<H', data[9:11])[0]

        return Status(
            cycle_time=cycle_time,
            i2c_error_count=i2c_errors,
            sensor_flags=sensors,
            flight_mode_flags=modes,
            config_profile=profile,
            cpu_load=cpu_load
        )

    def get_rc_channels(self) -> List[int]:
        """
        Get current RC channel values

        Returns:
            List of channel values (1000-2000 typically)
        """
        data = self.send_command(MSPCommand.MSP_RC)

        num_channels = len(data) // 2
        channels = []

        for i in range(num_channels):
            value = struct.unpack('<H', data[i*2:(i+1)*2])[0]
            channels.append(value)

        return channels

    def get_motor_values(self) -> List[int]:
        """
        Get current motor output values

        Returns:
            List of motor values
        """
        data = self.send_command(MSPCommand.MSP_MOTOR)

        num_motors = len(data) // 2
        motors = []

        for i in range(num_motors):
            value = struct.unpack('<H', data[i*2:(i+1)*2])[0]
            motors.append(value)

        return motors

    # ==================== Set Commands ====================

    def set_raw_rc(self, channels: List[int]):
        """
        Send RC channel values (MSP_SET_RAW_RC)

        Args:
            channels: List of 8-18 channel values (1000-2000)

        Note:
            Betaflight must have serialrx_provider = MSP configured
        """
        if len(channels) < 8:
            # Pad to 8 channels minimum
            channels = channels + [1500] * (8 - len(channels))

        # Clamp values
        channels = [max(1000, min(2000, c)) for c in channels]

        data = struct.pack('<' + 'H' * len(channels), *channels)
        self.send_command(MSPCommand.MSP_SET_RAW_RC, data)

    def set_motor_test(self, motor_values: List[int]):
        """
        Set motor test values (MSP_SET_MOTOR)

        Args:
            motor_values: List of motor values

        Warning:
            Only use for testing! Bypasses flight controller safety.
        """
        data = struct.pack('<' + 'H' * len(motor_values), *motor_values)
        self.send_command(MSPCommand.MSP_SET_MOTOR, data)

    def set_arming_disabled(self, disabled: bool, reason: int = 0):
        """
        Enable or disable arming

        Args:
            disabled: True to prevent arming
            reason: Reason code for disabling
        """
        data = struct.pack('<BB', int(disabled), reason)
        self.send_command(MSPCommand.MSP_SET_ARMING_DISABLED, data)

    def reboot(self):
        """Reboot the flight controller"""
        self.send_command_no_response(MSPCommand.MSP_REBOOT, b'\x00')

    def calibrate_acc(self):
        """Start accelerometer calibration"""
        self.send_command(MSPCommand.MSP_ACC_CALIBRATION)

    def calibrate_mag(self):
        """Start magnetometer calibration"""
        self.send_command(MSPCommand.MSP_MAG_CALIBRATION)

    # ==================== Utility ====================

    def close(self):
        """Close the connection"""
        self._connected = False
        if self.serial:
            self.serial.close()
