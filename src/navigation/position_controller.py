"""
Position Controller

Converts position error to velocity commands.
First stage of the cascaded control loop.

Position Error → Velocity Target
"""

import math
from dataclasses import dataclass
from typing import Tuple

from ..config import get_config


@dataclass
class Vector3:
    """3D vector for position/velocity"""
    x: float = 0.0  # North (m)
    y: float = 0.0  # East (m)
    z: float = 0.0  # Down (m) - positive is down

    def __add__(self, other: 'Vector3') -> 'Vector3':
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: 'Vector3') -> 'Vector3':
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> 'Vector3':
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def magnitude(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def magnitude_xy(self) -> float:
        return math.sqrt(self.x**2 + self.y**2)

    def normalized(self) -> 'Vector3':
        mag = self.magnitude()
        if mag > 0:
            return Vector3(self.x / mag, self.y / mag, self.z / mag)
        return Vector3()


class PositionController:
    """
    Position controller using P-control

    Converts position error to velocity target.
    Implements velocity limiting and expo curves.

    Inspired by iNav's updatePositionVelocityController_MC()
    """

    def __init__(self):
        """Initialize position controller"""
        config = get_config()

        # Gains
        self.pos_p_gain = config.navigation.pos_p_gain

        # Limits
        self.max_horizontal_speed = config.navigation.max_horizontal_speed_ms
        self.max_vertical_speed = config.navigation.max_vertical_speed_ms

        # State
        self.target_position = Vector3()
        self.current_position = Vector3()
        self.velocity_target = Vector3()

        # Position hold mode
        self.hold_position: Vector3 = None

    def set_target(self, north: float, east: float, down: float):
        """Set target position in NED frame (meters)"""
        self.target_position = Vector3(north, east, down)

    def set_current_position(self, north: float, east: float, down: float):
        """Update current position from GPS"""
        self.current_position = Vector3(north, east, down)

    def capture_hold_position(self):
        """Capture current position for position hold mode"""
        self.hold_position = Vector3(
            self.current_position.x,
            self.current_position.y,
            self.current_position.z
        )
        self.target_position = self.hold_position

    def update(self) -> Vector3:
        """
        Calculate velocity target from position error

        Returns:
            Velocity target in NED frame (m/s)
        """
        # Calculate position error
        pos_error = self.target_position - self.current_position

        # P-control: position error to velocity
        vel_target = Vector3(
            pos_error.x * self.pos_p_gain,
            pos_error.y * self.pos_p_gain,
            pos_error.z * self.pos_p_gain  # Altitude handled separately
        )

        # Limit horizontal velocity
        vel_horizontal = vel_target.magnitude_xy()
        if vel_horizontal > self.max_horizontal_speed:
            scale = self.max_horizontal_speed / vel_horizontal
            vel_target.x *= scale
            vel_target.y *= scale

        # Limit vertical velocity
        vel_target.z = max(-self.max_vertical_speed,
                          min(self.max_vertical_speed, vel_target.z))

        self.velocity_target = vel_target
        return vel_target

    def get_position_error(self) -> Vector3:
        """Get current position error"""
        return self.target_position - self.current_position

    def get_distance_to_target(self) -> float:
        """Get 3D distance to target in meters"""
        return self.get_position_error().magnitude()

    def get_horizontal_distance_to_target(self) -> float:
        """Get horizontal distance to target in meters"""
        return self.get_position_error().magnitude_xy()

    def is_at_target(self, radius: float = 2.0) -> bool:
        """Check if within target radius"""
        return self.get_horizontal_distance_to_target() < radius


class PositionControllerGPS:
    """
    Position controller with GPS coordinate support

    Wraps PositionController with GPS ↔ NED conversion.
    """

    def __init__(self):
        """Initialize GPS position controller"""
        self.controller = PositionController()

        # Reference point for NED conversion
        self.ref_lat: float = 0.0
        self.ref_lon: float = 0.0
        self.ref_alt: float = 0.0
        self._ref_initialized: bool = False

    def set_reference(self, lat: float, lon: float, alt: float):
        """Set reference point for coordinate conversion"""
        self.ref_lat = lat
        self.ref_lon = lon
        self.ref_alt = alt
        self._ref_initialized = True

    def set_reference_from_current(self, lat: float, lon: float, alt: float):
        """Set current position as reference and capture for hold"""
        self.set_reference(lat, lon, alt)
        self.update_position(lat, lon, alt)
        self.controller.capture_hold_position()

    def update_position(self, lat: float, lon: float, alt: float):
        """Update current position from GPS"""
        if not self._ref_initialized:
            self.set_reference(lat, lon, alt)

        north, east, down = self._gps_to_ned(lat, lon, alt)
        self.controller.set_current_position(north, east, down)

    def set_target_gps(self, lat: float, lon: float, alt: float):
        """Set target as GPS coordinates"""
        if not self._ref_initialized:
            raise ValueError("Reference point not set")

        north, east, down = self._gps_to_ned(lat, lon, alt)
        self.controller.set_target(north, east, down)

    def set_target_relative(self, north: float, east: float, down: float):
        """Set target relative to current position (NED meters)"""
        self.controller.set_target(
            self.controller.current_position.x + north,
            self.controller.current_position.y + east,
            self.controller.current_position.z + down
        )

    def update(self) -> Tuple[float, float, float]:
        """
        Calculate velocity target

        Returns:
            Tuple of (vel_north, vel_east, vel_down) in m/s
        """
        vel = self.controller.update()
        return vel.x, vel.y, vel.z

    def _gps_to_ned(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """Convert GPS to NED coordinates"""
        EARTH_RADIUS = 6378137.0

        d_lat = math.radians(lat - self.ref_lat)
        d_lon = math.radians(lon - self.ref_lon)
        ref_lat_rad = math.radians(self.ref_lat)

        north = d_lat * EARTH_RADIUS
        east = d_lon * EARTH_RADIUS * math.cos(ref_lat_rad)
        down = -(alt - self.ref_alt)

        return north, east, down

    def _ned_to_gps(self, north: float, east: float, down: float) -> Tuple[float, float, float]:
        """Convert NED to GPS coordinates"""
        EARTH_RADIUS = 6378137.0
        ref_lat_rad = math.radians(self.ref_lat)

        d_lat = north / EARTH_RADIUS
        d_lon = east / (EARTH_RADIUS * math.cos(ref_lat_rad))

        lat = self.ref_lat + math.degrees(d_lat)
        lon = self.ref_lon + math.degrees(d_lon)
        alt = self.ref_alt - down

        return lat, lon, alt

    @property
    def distance_to_target(self) -> float:
        return self.controller.get_horizontal_distance_to_target()

    def is_at_target(self, radius: float = 2.0) -> bool:
        return self.controller.is_at_target(radius)
