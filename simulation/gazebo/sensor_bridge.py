"""
Sensor Bridge: Gazebo -> Betaflight SITL

Reads sensor data from Gazebo and sends to Betaflight SITL.
"""

import socket
import struct
import threading
import time
import math
from dataclasses import dataclass
from typing import Optional
import logging

logger = logging.getLogger(__name__)


@dataclass
class SensorData:
    """Sensor data from Gazebo"""
    # IMU
    acc_x: float = 0.0  # m/s^2
    acc_y: float = 0.0
    acc_z: float = 9.81

    gyro_x: float = 0.0  # rad/s
    gyro_y: float = 0.0
    gyro_z: float = 0.0

    # Barometer
    pressure: float = 101325.0  # Pa
    temperature: float = 25.0  # Celsius

    # GPS
    latitude: float = 48.8566  # degrees
    longitude: float = 2.3522
    altitude: float = 0.0  # m
    ground_speed: float = 0.0  # m/s
    ground_course: float = 0.0  # degrees

    # Magnetometer
    mag_x: float = 0.2  # Gauss
    mag_y: float = 0.0
    mag_z: float = 0.4


class SensorBridge:
    """
    Bridge sensor data from Gazebo to Betaflight SITL

    Betaflight SITL expects sensor data on UDP port 9003 in a specific format.
    """

    def __init__(self):
        self._socket: Optional[socket.socket] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Target SITL address
        self._sitl_addr = ('127.0.0.1', 9003)

        # Current sensor data
        self._sensor_data = SensorData()
        self._data_lock = threading.Lock()

        # Update rate
        self._update_rate = 1000  # Hz

    def start(self, sitl_host: str = '127.0.0.1', sitl_port: int = 9003):
        """Start sensor bridge"""
        self._sitl_addr = (sitl_host, sitl_port)

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._running = True
        self._thread = threading.Thread(target=self._send_loop, daemon=True)
        self._thread.start()

        logger.info(f"Sensor bridge started, sending to {sitl_host}:{sitl_port}")

    def stop(self):
        """Stop sensor bridge"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._socket:
            self._socket.close()

    def update_sensors(self, data: SensorData):
        """Update sensor data"""
        with self._data_lock:
            self._sensor_data = data

    def update_imu(self, acc: tuple, gyro: tuple):
        """Update IMU data"""
        with self._data_lock:
            self._sensor_data.acc_x, self._sensor_data.acc_y, self._sensor_data.acc_z = acc
            self._sensor_data.gyro_x, self._sensor_data.gyro_y, self._sensor_data.gyro_z = gyro

    def update_gps(self, lat: float, lon: float, alt: float, speed: float = 0, course: float = 0):
        """Update GPS data"""
        with self._data_lock:
            self._sensor_data.latitude = lat
            self._sensor_data.longitude = lon
            self._sensor_data.altitude = alt
            self._sensor_data.ground_speed = speed
            self._sensor_data.ground_course = course

    def _send_loop(self):
        """Send sensor data at fixed rate"""
        period = 1.0 / self._update_rate

        while self._running:
            start = time.time()

            self._send_sensor_packet()

            # Maintain update rate
            elapsed = time.time() - start
            if elapsed < period:
                time.sleep(period - elapsed)

    def _send_sensor_packet(self):
        """
        Send sensor data packet to SITL

        The SITL expects data in the fdm_packet structure format.
        See src/main/target/SITL/sitl.c in Betaflight source.
        """
        with self._data_lock:
            data = self._sensor_data

        # Build packet matching fdm_packet structure
        # This is a simplified version - full format in Betaflight SITL source

        # IMU data (body frame)
        # Gyro in deg/s, Accel in m/s^2
        packet = struct.pack(
            '<dddddd',  # 6 doubles
            math.degrees(data.gyro_x),
            math.degrees(data.gyro_y),
            math.degrees(data.gyro_z),
            data.acc_x,
            data.acc_y,
            data.acc_z,
        )

        # Add timestamp
        packet += struct.pack('<Q', int(time.time() * 1e6))

        try:
            self._socket.sendto(packet, self._sitl_addr)
        except Exception as e:
            if self._running:
                logger.debug(f"Failed to send sensors: {e}")


class GazeboSensorReader:
    """
    Read sensor data from Gazebo

    Uses Gazebo transport to subscribe to sensor topics.
    This is a placeholder - actual implementation depends on Gazebo version.
    """

    def __init__(self, sensor_bridge: SensorBridge):
        self.bridge = sensor_bridge
        self._running = False

    def start(self):
        """Start reading from Gazebo"""
        # In real implementation, would subscribe to Gazebo topics:
        # - /gazebo/default/imu/imu
        # - /gazebo/default/gps/gps
        # - etc.

        logger.info("Gazebo sensor reader started (placeholder)")
        self._running = True

    def stop(self):
        """Stop reading"""
        self._running = False

    def simulate_sensors(self, state: dict):
        """
        Simulate sensors from state

        For testing without actual Gazebo.
        state should have: position, velocity, attitude, angular_velocity
        """
        # Calculate IMU from state
        acc = (
            state.get('acc_x', 0),
            state.get('acc_y', 0),
            state.get('acc_z', 9.81),
        )
        gyro = (
            state.get('roll_rate', 0),
            state.get('pitch_rate', 0),
            state.get('yaw_rate', 0),
        )

        self.bridge.update_imu(acc, gyro)

        # GPS
        if 'lat' in state:
            self.bridge.update_gps(
                state['lat'],
                state['lon'],
                state.get('alt', 0),
                state.get('ground_speed', 0),
                state.get('course', 0),
            )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    print("=== Sensor Bridge Test ===\n")

    bridge = SensorBridge()
    bridge.start()

    print("Sending sensor data to SITL at 127.0.0.1:9003")
    print("Press Ctrl+C to stop\n")

    try:
        t = 0
        while True:
            # Simulate some sensor noise
            data = SensorData()
            data.acc_z = 9.81 + 0.1 * math.sin(t)
            data.gyro_z = 0.01 * math.sin(t * 2)

            bridge.update_sensors(data)

            t += 0.1
            time.sleep(0.1)
            print(f"\rTime: {t:.1f}s, AccZ: {data.acc_z:.2f}", end='')

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        bridge.stop()
