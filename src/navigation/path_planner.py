"""
Path Planner

Pre-calculates path segments between waypoints including:
- Bearings and distances
- Turn angles and radii
- Turn anticipation distances

Used by L1 controller and waypoint navigator for smooth path following.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple, TYPE_CHECKING, Any

from ..config import get_config

if TYPE_CHECKING:
    from .waypoint_navigator import Waypoint, Mission


# Earth radius for distance calculations
EARTH_RADIUS_M = 6378137.0

# Gravity for turn radius calculations
GRAVITY_MSS = 9.80665


@dataclass
class PathSegment:
    """
    Represents a path segment between two waypoints.

    Includes pre-calculated values for navigation.
    """
    index: int                    # Segment index in mission
    from_wp: Waypoint             # Start waypoint
    to_wp: Waypoint               # End waypoint

    # Geometry
    bearing_rad: float            # Direction from->to in radians (0 = North, CW)
    distance_m: float             # Distance in meters

    # Turn at end of segment (to next segment)
    turn_angle_rad: float         # Angle to turn (signed, + = right)
    turn_radius_m: float          # Calculated turn radius
    turn_start_distance_m: float  # Distance before to_wp to start turn

    # Altitude
    altitude_change_m: float      # Altitude difference (to - from)
    climb_angle_rad: float        # Climb/descent angle

    # Speed targets
    entry_speed_ms: float         # Target speed entering segment
    exit_speed_ms: float          # Target speed exiting (for turn)


class PathPlanner:
    """
    Pre-calculates path information for a mission.

    Computes geometric properties of each segment to enable:
    - L1 path following (needs bearing, distance)
    - Turn anticipation (needs turn angle, radius)
    - Speed adaptation (needs turn angle)
    - Altitude ramping (needs altitude change)
    """

    def __init__(self):
        """Initialize path planner"""
        config = get_config()

        # Configuration
        self.max_bank_angle_deg = config.navigation.max_bank_angle_deg
        self.min_turn_radius_m = config.navigation.min_turn_radius_m
        self.default_speed_ms = config.navigation.max_horizontal_speed_ms
        self.turn_speed_reduction = config.navigation.turn_speed_reduction

        # Calculated segments
        self.segments: List[PathSegment] = []

        # Reference for GPS<->NED conversion
        self.ref_lat = 0.0
        self.ref_lon = 0.0
        self._ref_set = False

    def set_reference(self, lat: float, lon: float):
        """Set reference point for coordinate conversion"""
        self.ref_lat = lat
        self.ref_lon = lon
        self._ref_set = True

    def calculate_mission(self, mission: Mission, default_speed: float = None):
        """
        Calculate path segments for entire mission

        Args:
            mission: Mission with waypoints
            default_speed: Default speed in m/s (uses config if None)
        """
        self.segments.clear()

        if not mission or len(mission.waypoints) < 2:
            return

        if default_speed is None:
            default_speed = self.default_speed_ms

        waypoints = mission.waypoints

        for i in range(len(waypoints) - 1):
            from_wp = waypoints[i]
            to_wp = waypoints[i + 1]

            # Basic geometry
            bearing = self._calculate_bearing(
                from_wp.latitude, from_wp.longitude,
                to_wp.latitude, to_wp.longitude
            )
            distance = self._calculate_distance(
                from_wp.latitude, from_wp.longitude,
                to_wp.latitude, to_wp.longitude
            )

            # Turn angle to next segment
            if i + 2 < len(waypoints):
                next_wp = waypoints[i + 2]
                next_bearing = self._calculate_bearing(
                    to_wp.latitude, to_wp.longitude,
                    next_wp.latitude, next_wp.longitude
                )
                turn_angle = self._wrap_angle(next_bearing - bearing)
            else:
                turn_angle = 0.0

            # Calculate turn geometry
            turn_radius = self._calculate_turn_radius(default_speed)
            turn_start_dist = self._calculate_turn_start_distance(
                turn_angle, turn_radius
            )

            # Speed for this turn
            exit_speed = self._calculate_turn_speed(turn_angle, default_speed)

            # Altitude
            alt_change = to_wp.altitude - from_wp.altitude
            climb_angle = math.atan2(alt_change, distance) if distance > 0 else 0

            segment = PathSegment(
                index=i,
                from_wp=from_wp,
                to_wp=to_wp,
                bearing_rad=bearing,
                distance_m=distance,
                turn_angle_rad=turn_angle,
                turn_radius_m=turn_radius,
                turn_start_distance_m=turn_start_dist,
                altitude_change_m=alt_change,
                climb_angle_rad=climb_angle,
                entry_speed_ms=default_speed,
                exit_speed_ms=exit_speed
            )

            self.segments.append(segment)

    def get_segment(self, index: int) -> Optional[PathSegment]:
        """Get segment by index"""
        if 0 <= index < len(self.segments):
            return self.segments[index]
        return None

    def get_total_distance(self) -> float:
        """Get total mission distance in meters"""
        return sum(seg.distance_m for seg in self.segments)

    def get_segment_count(self) -> int:
        """Get number of segments"""
        return len(self.segments)

    def project_on_segment(self, lat: float, lon: float,
                           segment: PathSegment) -> Tuple[float, float, float]:
        """
        Project current position onto a path segment.

        Returns:
            Tuple of:
            - along_track_m: Distance along track from from_wp (can be negative)
            - cross_track_m: Perpendicular distance from track (+ = right of track)
            - progress: 0-1 progress along segment
        """
        # Convert to local NED coordinates
        from_ned = self._gps_to_local(
            segment.from_wp.latitude, segment.from_wp.longitude
        )
        to_ned = self._gps_to_local(
            segment.to_wp.latitude, segment.to_wp.longitude
        )
        pos_ned = self._gps_to_local(lat, lon)

        # Vector from from_wp to to_wp
        track_x = to_ned[0] - from_ned[0]
        track_y = to_ned[1] - from_ned[1]
        track_len = math.sqrt(track_x**2 + track_y**2)

        if track_len < 0.1:
            # Degenerate segment
            return 0.0, 0.0, 1.0

        # Normalize track vector
        track_ux = track_x / track_len
        track_uy = track_y / track_len

        # Vector from from_wp to current position
        dx = pos_ned[0] - from_ned[0]
        dy = pos_ned[1] - from_ned[1]

        # Project onto track
        along_track = dx * track_ux + dy * track_uy

        # Cross track (perpendicular distance)
        # Positive = right of track
        cross_track = -dx * track_uy + dy * track_ux

        # Progress (0-1)
        progress = along_track / track_len
        progress = max(0.0, min(1.0, progress))

        return along_track, cross_track, progress

    def get_point_on_segment(self, segment: PathSegment,
                             distance_from_start: float) -> Tuple[float, float]:
        """
        Get GPS coordinates of a point along a segment.

        Args:
            segment: Path segment
            distance_from_start: Distance from start of segment in meters

        Returns:
            Tuple of (latitude, longitude)
        """
        if segment.distance_m < 0.1:
            return segment.to_wp.latitude, segment.to_wp.longitude

        # Fraction along segment
        fraction = distance_from_start / segment.distance_m
        fraction = max(0.0, min(1.0, fraction))

        # Linear interpolation of GPS coordinates
        lat = segment.from_wp.latitude + fraction * (
            segment.to_wp.latitude - segment.from_wp.latitude
        )
        lon = segment.from_wp.longitude + fraction * (
            segment.to_wp.longitude - segment.from_wp.longitude
        )

        return lat, lon

    def get_interpolated_altitude(self, segment: PathSegment,
                                  progress: float) -> float:
        """
        Get interpolated altitude along segment.

        Args:
            segment: Path segment
            progress: 0-1 progress along segment

        Returns:
            Interpolated altitude in meters
        """
        progress = max(0.0, min(1.0, progress))
        return (segment.from_wp.altitude +
                progress * segment.altitude_change_m)

    def _calculate_bearing(self, lat1: float, lon1: float,
                          lat2: float, lon2: float) -> float:
        """Calculate bearing from point 1 to point 2 in radians"""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        d_lon = math.radians(lon2 - lon1)

        x = math.sin(d_lon) * math.cos(lat2_rad)
        y = (math.cos(lat1_rad) * math.sin(lat2_rad) -
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(d_lon))

        bearing = math.atan2(x, y)
        return bearing  # 0 = North, positive = East (CW)

    def _calculate_distance(self, lat1: float, lon1: float,
                           lat2: float, lon2: float) -> float:
        """Calculate distance between two GPS points in meters (Haversine)"""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        d_lat = lat2_rad - lat1_rad
        d_lon = math.radians(lon2 - lon1)

        a = (math.sin(d_lat / 2)**2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(d_lon / 2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return EARTH_RADIUS_M * c

    def _calculate_turn_radius(self, speed: float) -> float:
        """
        Calculate turn radius for given speed and max bank angle.

        R = v^2 / (g * tan(bank))
        """
        bank_rad = math.radians(self.max_bank_angle_deg)
        tan_bank = math.tan(bank_rad)

        if tan_bank < 0.1:
            tan_bank = 0.1  # Avoid division issues

        radius = (speed**2) / (GRAVITY_MSS * tan_bank)

        return max(radius, self.min_turn_radius_m)

    def _calculate_turn_start_distance(self, turn_angle: float,
                                       turn_radius: float) -> float:
        """
        Calculate distance before waypoint to start turn.

        Geometry: d = R * tan(|turn_angle| / 2)
        """
        if abs(turn_angle) < math.radians(5):
            # Small turn, no anticipation needed
            return 0.0

        half_angle = abs(turn_angle) / 2
        distance = turn_radius * math.tan(half_angle)

        # Limit to reasonable range
        return min(distance, turn_radius * 2)

    def _calculate_turn_speed(self, turn_angle: float,
                             default_speed: float) -> float:
        """
        Calculate recommended speed for a turn.

        Reduces speed for sharper turns.
        """
        # No reduction for small turns
        if abs(turn_angle) < math.radians(15):
            return default_speed

        # Linear reduction: 180deg = turn_speed_reduction of max
        reduction_factor = abs(turn_angle) / math.pi  # 0 to 1
        min_factor = self.turn_speed_reduction  # e.g., 0.5

        speed_factor = 1.0 - reduction_factor * (1.0 - min_factor)

        return default_speed * speed_factor

    def _gps_to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        """Convert GPS to local NED (North, East) relative to reference"""
        if not self._ref_set:
            self.ref_lat = lat
            self.ref_lon = lon
            self._ref_set = True

        d_lat = math.radians(lat - self.ref_lat)
        d_lon = math.radians(lon - self.ref_lon)
        ref_lat_rad = math.radians(self.ref_lat)

        north = d_lat * EARTH_RADIUS_M
        east = d_lon * EARTH_RADIUS_M * math.cos(ref_lat_rad)

        return north, east

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_debug_info(self) -> dict:
        """Get debug information"""
        return {
            'segment_count': len(self.segments),
            'total_distance_m': self.get_total_distance(),
            'segments': [
                {
                    'index': s.index,
                    'bearing_deg': math.degrees(s.bearing_rad),
                    'distance_m': s.distance_m,
                    'turn_angle_deg': math.degrees(s.turn_angle_rad),
                    'turn_radius_m': s.turn_radius_m,
                    'turn_start_m': s.turn_start_distance_m,
                    'exit_speed_ms': s.exit_speed_ms
                }
                for s in self.segments
            ]
        }
