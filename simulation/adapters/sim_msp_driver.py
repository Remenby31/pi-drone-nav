"""
Simulated MSP Driver

Replaces the real MSP driver when running in simulation.
Communicates with Betaflight SITL via UDP.
"""

import socket
import struct
import time
import threading
from typing import Optional, Dict, Any, List
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class SimulatedState:
    """State received from SITL or simulated"""
    # Attitude
    roll: float = 0.0  # degrees
    pitch: float = 0.0
    yaw: float = 0.0

    # Position (NED from origin)
    x: float = 0.0  # meters
    y: float = 0.0
    z: float = 0.0

    # Velocity
    vx: float = 0.0  # m/s
    vy: float = 0.0
    vz: float = 0.0

    # IMU
    acc_x: int = 0  # raw
    acc_y: int = 0
    acc_z: int = 2048  # 1G
    gyro_x: int = 0
    gyro_y: int = 0
    gyro_z: int = 0

    # Analog
    vbat: float = 16.0  # volts
    current: float = 0.0  # amps

    # GPS
    gps_fix: bool = True
    gps_sats: int = 12
    gps_lat: float = 48.8566
    gps_lon: float = 2.3522
    gps_alt: float = 0.0
    gps_speed: float = 0.0
    gps_course: float = 0.0

    # Status
    armed: bool = False
    motors: List[int] = None

    def __post_init__(self):
        if self.motors is None:
            self.motors = [1000, 1000, 1000, 1000]


class SimulatedMSP:
    """
    Simulated MSP driver for SITL

    Provides the same interface as the real MSP driver,
    but communicates with Betaflight SITL via UDP.
    """

    def __init__(self, sitl_host: str = '127.0.0.1'):
        self.sitl_host = sitl_host

        # SITL UDP ports
        self.rc_port = 9004  # Send RC channels

        # State
        self.state = SimulatedState()
        self._state_lock = threading.Lock()

        # RC channels
        self._rc_channels = [1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500]

        # Connection status
        self.connected = False

        # Socket
        self._socket: Optional[socket.socket] = None

    def connect(self) -> bool:
        """Connect to SITL"""
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.connected = True
            logger.info(f"SimulatedMSP connected to {self.sitl_host}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            return False

    def disconnect(self):
        """Disconnect from SITL"""
        if self._socket:
            self._socket.close()
            self._socket = None
        self.connected = False

    # === MSP-compatible interface ===

    def get_attitude(self) -> Optional[Dict[str, float]]:
        """Get attitude (roll, pitch, yaw in degrees)"""
        with self._state_lock:
            return {
                'roll': self.state.roll,
                'pitch': self.state.pitch,
                'yaw': self.state.yaw
            }

    def get_altitude(self) -> Optional[Dict[str, float]]:
        """Get altitude and climb rate"""
        with self._state_lock:
            return {
                'altitude': -self.state.z,  # Convert NED to altitude
                'vario': -self.state.vz
            }

    def get_raw_imu(self) -> Optional[Dict[str, Any]]:
        """Get raw IMU data"""
        with self._state_lock:
            return {
                'acc': (self.state.acc_x, self.state.acc_y, self.state.acc_z),
                'gyro': (self.state.gyro_x, self.state.gyro_y, self.state.gyro_z),
                'mag': (0, 0, 0)
            }

    def get_analog(self) -> Optional[Dict[str, float]]:
        """Get analog values (battery, current)"""
        with self._state_lock:
            return {
                'vbat': self.state.vbat,
                'current': self.state.current,
                'rssi': 100
            }

    def get_gps_data(self) -> Optional[Dict[str, Any]]:
        """Get GPS data"""
        with self._state_lock:
            return {
                'fix': self.state.gps_fix,
                'numSat': self.state.gps_sats,
                'lat': self.state.gps_lat,
                'lon': self.state.gps_lon,
                'alt': self.state.gps_alt,
                'speed': self.state.gps_speed,
                'course': self.state.gps_course
            }

    def get_rc_channels(self) -> Optional[List[int]]:
        """Get current RC channel values"""
        return self._rc_channels.copy()

    def get_motor_values(self) -> Optional[List[int]]:
        """Get motor output values"""
        with self._state_lock:
            return self.state.motors.copy()

    def set_raw_rc(self, channels: List[int]) -> bool:
        """
        Send RC channel values to SITL

        Args:
            channels: List of 8-16 channel values (1000-2000)

        Returns:
            True if sent successfully
        """
        if not self._socket:
            return False

        # Store locally
        self._rc_channels = channels[:8] + [1500] * max(0, 8 - len(channels))

        # Pad to 16 channels
        while len(channels) < 16:
            channels.append(1500)

        # Pack as 16 x uint16 little-endian
        data = struct.pack('<16H', *channels[:16])

        try:
            self._socket.sendto(data, (self.sitl_host, self.rc_port))

            # Check if armed based on AUX1
            with self._state_lock:
                self.state.armed = channels[4] > 1700

            return True
        except Exception as e:
            logger.error(f"Failed to send RC: {e}")
            return False

    def get_status(self) -> Optional[Dict[str, Any]]:
        """Get FC status"""
        with self._state_lock:
            return {
                'armed': self.state.armed,
                'mode': 'ANGLE' if self.state.armed else 'DISARMED',
                'cycle_time': 500,
                'cpu_load': 10
            }

    # === Simulation-specific methods ===

    def update_state(self, **kwargs):
        """Update simulated state"""
        with self._state_lock:
            for key, value in kwargs.items():
                if hasattr(self.state, key):
                    setattr(self.state, key, value)

    def set_gps_position(self, lat: float, lon: float, alt: float):
        """Set GPS position"""
        with self._state_lock:
            self.state.gps_lat = lat
            self.state.gps_lon = lon
            self.state.gps_alt = alt

    def set_attitude(self, roll: float, pitch: float, yaw: float):
        """Set attitude"""
        with self._state_lock:
            self.state.roll = roll
            self.state.pitch = pitch
            self.state.yaw = yaw


class MSPSimulationBridge:
    """
    Bridge that makes SimulatedMSP work with real SITL

    Receives state from physics simulation and updates SimulatedMSP.
    """

    def __init__(self, msp: SimulatedMSP, physics_bridge):
        self.msp = msp
        self.physics = physics_bridge
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        """Start bridge"""
        self._running = True
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop bridge"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _update_loop(self):
        """Update MSP state from physics"""
        while self._running:
            if self.physics:
                state = self.physics.state

                self.msp.update_state(
                    roll=state.roll,
                    pitch=state.pitch,
                    yaw=state.yaw,
                    x=state.north,
                    y=state.east,
                    z=state.down,
                    vx=state.vel_north,
                    vy=state.vel_east,
                    vz=state.vel_down,
                )

            time.sleep(0.02)  # 50 Hz


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    print("=== Simulated MSP Test ===\n")

    msp = SimulatedMSP()
    if msp.connect():
        print("Connected to simulation")

        # Test sending RC
        print("\nSending centered RC...")
        msp.set_raw_rc([1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500])

        print("\nSending ARM command (AUX1 = 1800)...")
        msp.set_raw_rc([1500, 1500, 1000, 1500, 1800, 1500, 1500, 1500])

        time.sleep(1)

        status = msp.get_status()
        print(f"Status: {status}")

        msp.disconnect()
    else:
        print("Failed to connect")
