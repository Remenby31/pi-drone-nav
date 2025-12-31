"""
Simulated GPS Driver

Provides GPS data from the simulation.
"""

import time
import math
import threading
from typing import Optional, Dict, Callable
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class GPSFix:
    """GPS fix data"""
    valid: bool = True
    fix_type: int = 3  # 3D fix
    satellites: int = 12

    # Position
    latitude: float = 48.8566  # degrees
    longitude: float = 2.3522
    altitude: float = 0.0  # meters MSL

    # Velocity
    vel_north: float = 0.0  # m/s
    vel_east: float = 0.0
    vel_down: float = 0.0
    ground_speed: float = 0.0
    ground_course: float = 0.0  # degrees

    # Accuracy
    h_acc: float = 1.0  # meters
    v_acc: float = 2.0
    speed_acc: float = 0.5

    # Timestamp
    timestamp: float = 0.0


class SimulatedGPS:
    """
    Simulated GPS driver

    Provides GPS data computed from simulation state.
    """

    def __init__(self, origin_lat: float = 48.8566, origin_lon: float = 2.3522):
        # Origin for NED to GPS conversion
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.origin_alt = 0.0

        # Current fix
        self._fix = GPSFix(latitude=origin_lat, longitude=origin_lon)
        self._fix_lock = threading.Lock()

        # Callbacks
        self._on_fix: Optional[Callable[[GPSFix], None]] = None

        # Update thread
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Connection to physics
        self._physics = None

    def connect(self, physics_source=None) -> bool:
        """
        Connect GPS to physics source

        Args:
            physics_source: Object with .state attribute containing position/velocity
        """
        self._physics = physics_source
        logger.info("SimulatedGPS connected")
        return True

    def disconnect(self):
        """Disconnect GPS"""
        self.stop()
        self._physics = None

    def start(self, update_rate: int = 10):
        """
        Start GPS updates

        Args:
            update_rate: Update frequency in Hz
        """
        if self._running:
            return

        self._running = True
        self._update_period = 1.0 / update_rate
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()
        logger.info(f"SimulatedGPS started at {update_rate} Hz")

    def stop(self):
        """Stop GPS updates"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _update_loop(self):
        """Update GPS fix from physics"""
        while self._running:
            start = time.time()

            self._update_fix()

            # Maintain update rate
            elapsed = time.time() - start
            if elapsed < self._update_period:
                time.sleep(self._update_period - elapsed)

    def _update_fix(self):
        """Update GPS fix from physics state"""
        if self._physics is None:
            return

        state = self._physics.state

        # Convert NED position to GPS
        lat, lon, alt = self.ned_to_gps(state.north, state.east, state.down)

        # Calculate ground speed and course
        ground_speed = math.sqrt(state.vel_north**2 + state.vel_east**2)
        ground_course = math.degrees(math.atan2(state.vel_east, state.vel_north))
        if ground_course < 0:
            ground_course += 360

        with self._fix_lock:
            self._fix = GPSFix(
                valid=True,
                fix_type=3,
                satellites=12,
                latitude=lat,
                longitude=lon,
                altitude=alt,
                vel_north=state.vel_north,
                vel_east=state.vel_east,
                vel_down=state.vel_down,
                ground_speed=ground_speed,
                ground_course=ground_course,
                h_acc=0.5,
                v_acc=1.0,
                timestamp=time.time()
            )

        # Callback
        if self._on_fix:
            self._on_fix(self._fix)

    def get_fix(self) -> GPSFix:
        """Get current GPS fix"""
        with self._fix_lock:
            return GPSFix(**vars(self._fix))

    def ned_to_gps(self, north: float, east: float, down: float) -> tuple:
        """
        Convert NED coordinates to GPS

        Args:
            north: North position in meters
            east: East position in meters
            down: Down position in meters (positive down)

        Returns:
            (latitude, longitude, altitude)
        """
        # Earth radius
        R = 6371000  # meters

        # Latitude change
        d_lat = north / R
        lat = self.origin_lat + math.degrees(d_lat)

        # Longitude change (account for latitude)
        d_lon = east / (R * math.cos(math.radians(self.origin_lat)))
        lon = self.origin_lon + math.degrees(d_lon)

        # Altitude (NED down is positive, altitude is up)
        alt = self.origin_alt - down

        return lat, lon, alt

    def gps_to_ned(self, lat: float, lon: float, alt: float) -> tuple:
        """
        Convert GPS coordinates to NED

        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters MSL

        Returns:
            (north, east, down)
        """
        R = 6371000

        d_lat = math.radians(lat - self.origin_lat)
        d_lon = math.radians(lon - self.origin_lon)

        north = d_lat * R
        east = d_lon * R * math.cos(math.radians(self.origin_lat))
        down = self.origin_alt - alt

        return north, east, down

    def set_origin(self, lat: float, lon: float, alt: float = 0.0):
        """Set GPS origin"""
        self.origin_lat = lat
        self.origin_lon = lon
        self.origin_alt = alt

    def on_fix(self, callback: Callable[[GPSFix], None]):
        """Set callback for new GPS fixes"""
        self._on_fix = callback

    # === Compatibility with UBX GPS interface ===

    def get_position(self) -> Optional[Dict]:
        """Get position (UBX-compatible interface)"""
        fix = self.get_fix()
        return {
            'lat': fix.latitude,
            'lon': fix.longitude,
            'alt': fix.altitude,
            'h_acc': fix.h_acc,
            'v_acc': fix.v_acc
        }

    def get_velocity(self) -> Optional[Dict]:
        """Get velocity (UBX-compatible interface)"""
        fix = self.get_fix()
        return {
            'vel_north': fix.vel_north,
            'vel_east': fix.vel_east,
            'vel_down': fix.vel_down,
            'speed_acc': fix.speed_acc
        }

    def get_fix_status(self) -> Optional[Dict]:
        """Get fix status (UBX-compatible interface)"""
        fix = self.get_fix()
        return {
            'fix_type': fix.fix_type,
            'satellites': fix.satellites,
            'valid': fix.valid
        }


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    print("=== Simulated GPS Test ===\n")

    # Create a mock physics state
    class MockPhysics:
        class State:
            north = 10.0
            east = 5.0
            down = -2.0  # 2m altitude
            vel_north = 1.0
            vel_east = 0.5
            vel_down = 0.0

        state = State()

    physics = MockPhysics()

    gps = SimulatedGPS(origin_lat=48.8566, origin_lon=2.3522)
    gps.connect(physics)
    gps.start(update_rate=10)

    print("GPS started. Press Ctrl+C to stop.\n")

    try:
        for _ in range(10):
            fix = gps.get_fix()
            print(f"Position: {fix.latitude:.6f}, {fix.longitude:.6f}, {fix.altitude:.1f}m")
            print(f"Velocity: N={fix.vel_north:.2f}, E={fix.vel_east:.2f}, D={fix.vel_down:.2f} m/s")
            print(f"Ground: {fix.ground_speed:.2f} m/s @ {fix.ground_course:.1f}°")
            print()
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        gps.stop()
        print("Done")
