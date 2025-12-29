"""
L1 Navigation Controller

Implements the L1 Nonlinear Guidance Logic for path following.
Based on ArduPilot's AP_L1_Control implementation.

Reference: Park, Deyst, How - "A New Nonlinear Guidance Logic for
Trajectory Tracking" (AIAA 2004-4900)

Key advantages:
- Simple tuning (2 main parameters)
- Mathematically proven stability
- Speed-adaptive behavior
- Smooth path following with minimal overshoot
"""

import math
import time
from dataclasses import dataclass
from typing import Tuple, Optional

from .position_controller import Vector3
from .path_planner import PathSegment
from ..config import get_config


# Physical constants
GRAVITY_MSS = 9.80665
EARTH_RADIUS_M = 6378137.0


@dataclass
class L1Output:
    """Output from L1 controller"""
    velocity_target: Vector3      # Target velocity in NED frame (m/s)
    lateral_accel: float          # Lateral acceleration command (m/s^2)
    cross_track_error: float      # Distance from path (m, + = right)
    along_track_distance: float   # Distance along path from segment start (m)
    bearing_error: float          # Angle error to L1 point (rad)
    l1_distance: float            # Current L1 lookahead distance (m)


class L1Controller:
    """
    L1 Nonlinear Guidance Controller with XTE integrator.

    The controller computes a virtual reference point ahead of the vehicle
    on the desired path, then commands lateral acceleration to intercept
    this moving reference point.

    Core formula: a_lateral = 2 * V^2 / L1 * sin(eta)

    Where:
    - V = ground speed
    - L1 = lookahead distance (function of speed and L1_period)
    - eta = angle from velocity vector to L1 reference point
    """

    def __init__(self):
        """Initialize L1 controller"""
        config = get_config()

        # Main tuning parameters
        self.l1_period = config.navigation.l1_period        # seconds
        self.l1_damping = config.navigation.l1_damping      # ratio (0.7-0.85)

        # XTE integrator for zero steady-state error
        self.xte_i_gain = config.navigation.xte_i_gain
        self._xte_integrator = 0.0
        self._xte_i_max = 5.0  # Max integrator contribution (m/s^2)

        # Limits
        self.min_l1_distance = 10.0   # Minimum L1 distance (m)
        self.max_l1_distance = 100.0  # Maximum L1 distance (m)
        self.min_speed = 1.0          # Minimum speed for L1 (m/s)

        # State
        self._last_cross_track = 0.0
        self._last_update_time = 0.0

        # Reference point for NED conversion
        self._ref_lat = 0.0
        self._ref_lon = 0.0
        self._ref_set = False

    def update(self, current_lat: float, current_lon: float,
               vel_north: float, vel_east: float,
               segment: PathSegment,
               dt: float = None) -> L1Output:
        """
        Compute L1 guidance output for path following.

        Args:
            current_lat: Current latitude
            current_lon: Current longitude
            vel_north: Velocity north (m/s)
            vel_east: Velocity east (m/s)
            segment: Current path segment to follow
            dt: Time step (None for automatic)

        Returns:
            L1Output with velocity target and debug info
        """
        # Calculate dt
        current_time = time.time()
        if dt is None:
            if self._last_update_time > 0:
                dt = current_time - self._last_update_time
            else:
                dt = 0.02
        self._last_update_time = current_time
        dt = max(0.001, min(dt, 0.5))

        # Current speed
        ground_speed = math.sqrt(vel_north**2 + vel_east**2)
        ground_speed = max(ground_speed, self.min_speed)

        # Current course (direction of travel)
        course_rad = math.atan2(vel_east, vel_north)

        # Set reference if not set
        if not self._ref_set:
            self._ref_lat = current_lat
            self._ref_lon = current_lon
            self._ref_set = True

        # Convert positions to local NED
        pos_ned = self._gps_to_ned(current_lat, current_lon)
        from_ned = self._gps_to_ned(segment.from_wp.latitude,
                                    segment.from_wp.longitude)
        to_ned = self._gps_to_ned(segment.to_wp.latitude,
                                  segment.to_wp.longitude)

        # Calculate L1 distance (scales with speed)
        omega_a = 2 * math.pi / self.l1_period
        l1_dist = ground_speed / omega_a
        l1_dist = max(self.min_l1_distance,
                      min(self.max_l1_distance, l1_dist))

        # Project position onto path line
        along_track, cross_track = self._project_on_line(
            pos_ned, from_ned, to_ned
        )

        # Track length
        track_length = math.sqrt(
            (to_ned[0] - from_ned[0])**2 +
            (to_ned[1] - from_ned[1])**2
        )

        # Find L1 reference point on path
        l1_point = self._get_l1_point(
            from_ned, to_ned, track_length,
            along_track, l1_dist
        )

        # Vector from current position to L1 point
        to_l1_n = l1_point[0] - pos_ned[0]
        to_l1_e = l1_point[1] - pos_ned[1]

        # Bearing to L1 point
        bearing_to_l1 = math.atan2(to_l1_e, to_l1_n)

        # Angle between velocity and bearing to L1 point (eta)
        eta = self._wrap_angle(bearing_to_l1 - course_rad)

        # Core L1 lateral acceleration formula
        # a_lateral = 2 * V^2 / L1 * sin(eta)
        sin_eta = math.sin(eta)
        lateral_accel = 2.0 * ground_speed**2 / l1_dist * sin_eta

        # Add damping term
        # Kv = omega_a * 2*sqrt(2) * (damping - 1/sqrt(2))
        kv = omega_a * 2.8284 * (self.l1_damping - 0.7071)
        lateral_accel += kv * cross_track

        # XTE integrator (ensures convergence to zero cross-track error)
        self._xte_integrator += cross_track * self.xte_i_gain * dt
        self._xte_integrator = max(-self._xte_i_max,
                                   min(self._xte_i_max, self._xte_integrator))
        lateral_accel += self._xte_integrator

        # Convert lateral acceleration to velocity target
        # The velocity target is along the path direction with corrections
        velocity_target = self._accel_to_velocity_target(
            lateral_accel, segment.bearing_rad,
            ground_speed, cross_track
        )

        # Store for next iteration
        self._last_cross_track = cross_track

        return L1Output(
            velocity_target=velocity_target,
            lateral_accel=lateral_accel,
            cross_track_error=cross_track,
            along_track_distance=along_track,
            bearing_error=eta,
            l1_distance=l1_dist
        )

    def update_for_turn(self, current_lat: float, current_lon: float,
                        vel_north: float, vel_east: float,
                        current_segment: PathSegment,
                        next_segment: PathSegment,
                        turn_progress: float,
                        dt: float = None) -> L1Output:
        """
        Compute L1 guidance during turn anticipation.

        Blends between current and next segment bearings for smooth turns.

        Args:
            current_lat, current_lon: Current position
            vel_north, vel_east: Current velocity
            current_segment: Segment we're leaving
            next_segment: Segment we're entering
            turn_progress: 0-1, how far through the turn (0=start, 1=complete)
            dt: Time step

        Returns:
            L1Output with velocity target for turn
        """
        # Interpolate bearing between segments
        current_bearing = current_segment.bearing_rad
        next_bearing = next_segment.bearing_rad

        # Shortest path interpolation for angles
        bearing_diff = self._wrap_angle(next_bearing - current_bearing)
        interpolated_bearing = self._wrap_angle(
            current_bearing + bearing_diff * turn_progress
        )

        # Create a virtual segment with interpolated bearing
        # The "to" point is along the interpolated bearing
        ground_speed = math.sqrt(vel_north**2 + vel_east**2)
        ground_speed = max(ground_speed, self.min_speed)

        omega_a = 2 * math.pi / self.l1_period
        l1_dist = ground_speed / omega_a
        l1_dist = max(self.min_l1_distance, min(self.max_l1_distance, l1_dist))

        # Virtual target point along interpolated bearing
        virtual_target_north = current_lat + (l1_dist / EARTH_RADIUS_M) * \
                              math.degrees(math.cos(interpolated_bearing))
        virtual_target_east = current_lon + (l1_dist / EARTH_RADIUS_M) * \
                             math.degrees(math.sin(interpolated_bearing)) / \
                             math.cos(math.radians(current_lat))

        # Simple pursuit of the virtual target
        pos_ned = self._gps_to_ned(current_lat, current_lon)
        target_ned = self._gps_to_ned(virtual_target_north, virtual_target_east)

        # Vector to target
        to_target_n = target_ned[0] - pos_ned[0]
        to_target_e = target_ned[1] - pos_ned[1]

        # Bearing to target
        bearing_to_target = math.atan2(to_target_e, to_target_n)

        # Current course
        course_rad = math.atan2(vel_east, vel_north)

        # Angle error
        eta = self._wrap_angle(bearing_to_target - course_rad)

        # Lateral acceleration
        sin_eta = math.sin(eta)
        lateral_accel = 2.0 * ground_speed**2 / l1_dist * sin_eta

        # Reduced XTE correction during turn (we're deliberately off-track)
        # Cross track relative to interpolated path
        cross_track = 0.0  # Simplified for turn

        # Convert to velocity target
        velocity_target = self._accel_to_velocity_target(
            lateral_accel, interpolated_bearing,
            ground_speed, cross_track
        )

        return L1Output(
            velocity_target=velocity_target,
            lateral_accel=lateral_accel,
            cross_track_error=cross_track,
            along_track_distance=0.0,
            bearing_error=eta,
            l1_distance=l1_dist
        )

    def _project_on_line(self, pos: Tuple[float, float],
                         line_start: Tuple[float, float],
                         line_end: Tuple[float, float]) -> Tuple[float, float]:
        """
        Project position onto line segment.

        Returns:
            (along_track, cross_track) distances in meters
            along_track: distance from line_start along line
            cross_track: perpendicular distance (+ = right of line)
        """
        # Line vector
        line_n = line_end[0] - line_start[0]
        line_e = line_end[1] - line_start[1]
        line_len = math.sqrt(line_n**2 + line_e**2)

        if line_len < 0.1:
            # Degenerate line
            return 0.0, 0.0

        # Unit vector along line
        line_un = line_n / line_len
        line_ue = line_e / line_len

        # Vector from line start to position
        to_pos_n = pos[0] - line_start[0]
        to_pos_e = pos[1] - line_start[1]

        # Along-track (dot product)
        along_track = to_pos_n * line_un + to_pos_e * line_ue

        # Cross-track (perpendicular, + = right)
        # Cross product in 2D: a x b = a_n * b_e - a_e * b_n
        cross_track = to_pos_n * line_ue - to_pos_e * line_un

        return along_track, cross_track

    def _get_l1_point(self, line_start: Tuple[float, float],
                      line_end: Tuple[float, float],
                      line_len: float,
                      along_track: float,
                      l1_dist: float) -> Tuple[float, float]:
        """
        Get the L1 reference point on the path.

        The L1 point is l1_dist ahead of the current along-track position,
        clamped to the segment.
        """
        if line_len < 0.1:
            return line_end

        # Unit vector along line
        line_un = (line_end[0] - line_start[0]) / line_len
        line_ue = (line_end[1] - line_start[1]) / line_len

        # L1 point is l1_dist ahead of current along-track position
        l1_along = along_track + l1_dist

        # Clamp to segment (with some margin past end for smooth transition)
        l1_along = max(0.0, min(line_len + l1_dist * 0.5, l1_along))

        # Point on line
        l1_n = line_start[0] + l1_along * line_un
        l1_e = line_start[1] + l1_along * line_ue

        return l1_n, l1_e

    def _accel_to_velocity_target(self, lateral_accel: float,
                                  track_bearing: float,
                                  speed: float,
                                  cross_track: float) -> Vector3:
        """
        Convert lateral acceleration to velocity target.

        The velocity target has two components:
        1. Along-track: maintains speed along the path
        2. Cross-track: correction to reduce cross-track error
        """
        # Cross-track velocity correction (proportional to lateral accel)
        # This creates a velocity component perpendicular to the track
        # that will reduce cross-track error
        cross_vel = -lateral_accel * 0.5  # Tuning factor

        # Limit cross-track velocity
        max_cross_vel = speed * 0.5  # Max 50% of speed for correction
        cross_vel = max(-max_cross_vel, min(max_cross_vel, cross_vel))

        # Decompose into NED
        # Along-track direction
        along_n = math.cos(track_bearing)
        along_e = math.sin(track_bearing)

        # Cross-track direction (perpendicular, 90 deg right)
        cross_n = -math.sin(track_bearing)
        cross_e = math.cos(track_bearing)

        # Velocity target
        vel_n = speed * along_n + cross_vel * cross_n
        vel_e = speed * along_e + cross_vel * cross_e

        return Vector3(vel_n, vel_e, 0.0)

    def _gps_to_ned(self, lat: float, lon: float) -> Tuple[float, float]:
        """Convert GPS to local NED (north, east) relative to reference"""
        d_lat = math.radians(lat - self._ref_lat)
        d_lon = math.radians(lon - self._ref_lon)
        ref_lat_rad = math.radians(self._ref_lat)

        north = d_lat * EARTH_RADIUS_M
        east = d_lon * EARTH_RADIUS_M * math.cos(ref_lat_rad)

        return north, east

    def set_reference(self, lat: float, lon: float):
        """Set reference point for NED conversion"""
        self._ref_lat = lat
        self._ref_lon = lon
        self._ref_set = True

    def reset(self):
        """Reset controller state"""
        self._xte_integrator = 0.0
        self._last_cross_track = 0.0
        self._last_update_time = 0.0

    def reset_integrator(self):
        """Reset XTE integrator only (for waypoint transitions)"""
        self._xte_integrator = 0.0

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
            'l1_period': self.l1_period,
            'l1_damping': self.l1_damping,
            'xte_integrator': self._xte_integrator,
            'last_cross_track': self._last_cross_track
        }
