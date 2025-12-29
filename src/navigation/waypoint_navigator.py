"""
Waypoint Navigator

Manages waypoint missions with advanced path following:
- L1 navigation controller for precise path tracking
- Turn anticipation with geometric radius calculation
- Speed adaptation based on turn angle
- Altitude ramping between waypoints

Based on ArduPilot L1 and iNav navigation algorithms.
"""

import json
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Callable, Tuple
from pathlib import Path
import logging

from .position_controller import PositionControllerGPS, Vector3
from .path_planner import PathPlanner, PathSegment
from .l1_controller import L1Controller, L1Output
from ..config import get_config

logger = logging.getLogger(__name__)


class WaypointAction(Enum):
    """Actions at waypoint"""
    FLY_THROUGH = auto()      # Pass without stopping
    HOVER = auto()            # Hover for specified time
    LAND = auto()             # Land at waypoint
    RTH = auto()              # Return to home


@dataclass
class Waypoint:
    """Single waypoint definition"""
    latitude: float
    longitude: float
    altitude: float           # meters MSL or AGL
    action: WaypointAction = WaypointAction.FLY_THROUGH
    hover_time: float = 0.0   # seconds (for HOVER action)
    speed: float = 0.0        # m/s, 0 = use default
    radius: float = 2.0       # acceptance radius in meters

    @classmethod
    def from_dict(cls, d: dict) -> 'Waypoint':
        """Create waypoint from dictionary"""
        action = WaypointAction[d.get('action', 'FLY_THROUGH').upper()]
        return cls(
            latitude=d['latitude'],
            longitude=d['longitude'],
            altitude=d.get('altitude', 10.0),
            action=action,
            hover_time=d.get('hover_time', 0.0),
            speed=d.get('speed', 0.0),
            radius=d.get('radius', 2.0)
        )

    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return {
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude': self.altitude,
            'action': self.action.name,
            'hover_time': self.hover_time,
            'speed': self.speed,
            'radius': self.radius
        }


@dataclass
class Mission:
    """Complete mission definition"""
    name: str = "Unnamed Mission"
    waypoints: List[Waypoint] = field(default_factory=list)
    default_speed: float = 10.0        # m/s
    default_altitude: float = 10.0     # meters
    return_to_home: bool = True

    @classmethod
    def from_json(cls, json_path: str) -> 'Mission':
        """Load mission from JSON file"""
        with open(json_path, 'r') as f:
            data = json.load(f)

        waypoints = [Waypoint.from_dict(wp) for wp in data.get('waypoints', [])]

        return cls(
            name=data.get('name', 'Unnamed'),
            waypoints=waypoints,
            default_speed=data.get('default_speed', 10.0),
            default_altitude=data.get('default_altitude', 10.0),
            return_to_home=data.get('return_to_home', True)
        )

    def to_json(self, json_path: str):
        """Save mission to JSON file"""
        data = {
            'name': self.name,
            'waypoints': [wp.to_dict() for wp in self.waypoints],
            'default_speed': self.default_speed,
            'default_altitude': self.default_altitude,
            'return_to_home': self.return_to_home
        }
        with open(json_path, 'w') as f:
            json.dump(data, f, indent=2)


class MissionState(Enum):
    """Mission execution state"""
    IDLE = auto()
    RUNNING = auto()
    PAUSED = auto()
    COMPLETED = auto()
    ABORTED = auto()


class NavigationMode(Enum):
    """Current navigation mode within mission"""
    STRAIGHT = auto()       # Following straight path to waypoint
    TURNING = auto()        # In turn anticipation phase
    HOVERING = auto()       # Hovering at waypoint
    POSITION_HOLD = auto()  # Holding position (paused)


class WaypointNavigator:
    """
    Advanced waypoint mission navigator

    Executes missions with:
    - L1 path following for precise trajectory tracking
    - Geometric turn anticipation (smooth fly-through)
    - Speed adaptation based on turn angles
    - Linear altitude ramping between waypoints

    Based on ArduPilot L1 and iNav navigation algorithms.
    """

    def __init__(self, position_controller: PositionControllerGPS):
        """
        Initialize waypoint navigator

        Args:
            position_controller: GPS position controller instance
        """
        self.pos_ctrl = position_controller
        config = get_config()

        # Advanced navigation components
        self.path_planner = PathPlanner()
        self.l1_controller = L1Controller()

        # Configuration
        self.enable_turn_anticipation = config.navigation.enable_turn_anticipation
        self.enable_speed_adaptation = config.navigation.enable_speed_adaptation
        self.enable_altitude_ramp = config.navigation.enable_altitude_ramp
        self.turn_speed_reduction = config.navigation.turn_speed_reduction
        self.max_decel = config.navigation.max_decel_mss
        self.default_speed = config.navigation.waypoint_speed_ms

        # Mission
        self.mission: Optional[Mission] = None
        self.state = MissionState.IDLE
        self.nav_mode = NavigationMode.STRAIGHT

        # Progress
        self.current_wp_index = 0
        self._hover_start_time = 0.0
        self._turn_progress = 0.0

        # Current navigation output
        self._current_velocity_target = Vector3()
        self._current_speed_target = 0.0
        self._current_altitude_target = 0.0
        self._cross_track_error = 0.0

        # Home position
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0
        self._home_set = False

        # Current position (updated externally)
        self._current_lat = 0.0
        self._current_lon = 0.0
        self._current_alt = 0.0
        self._vel_north = 0.0
        self._vel_east = 0.0

        # Callbacks
        self._on_waypoint_reached: Optional[Callable[[int, Waypoint], None]] = None
        self._on_mission_complete: Optional[Callable[[], None]] = None

    def load_mission(self, mission: Mission):
        """Load a mission and calculate paths"""
        self.mission = mission
        self.state = MissionState.IDLE
        self.current_wp_index = 0
        self.nav_mode = NavigationMode.STRAIGHT

        # Pre-calculate path segments
        if len(mission.waypoints) >= 2:
            self.path_planner.calculate_mission(
                mission,
                default_speed=mission.default_speed or self.default_speed
            )

        logger.info(f"Mission loaded: {mission.name} with {len(mission.waypoints)} waypoints")

    def load_mission_from_file(self, json_path: str):
        """Load mission from JSON file"""
        mission = Mission.from_json(json_path)
        self.load_mission(mission)

    def set_home(self, lat: float, lon: float, alt: float):
        """Set home position"""
        self.home_lat = lat
        self.home_lon = lon
        self.home_alt = alt
        self._home_set = True

        # Set reference for path planner and L1 controller
        self.path_planner.set_reference(lat, lon)
        self.l1_controller.set_reference(lat, lon)

        logger.info(f"Home set: {lat:.6f}, {lon:.6f}, {alt:.1f}m")

    def update_position(self, lat: float, lon: float, alt: float,
                        vel_north: float, vel_east: float):
        """
        Update current position and velocity.

        Must be called each control cycle before update().
        """
        self._current_lat = lat
        self._current_lon = lon
        self._current_alt = alt
        self._vel_north = vel_north
        self._vel_east = vel_east

    def start(self) -> bool:
        """
        Start mission execution

        Returns:
            True if mission started successfully
        """
        if not self.mission or len(self.mission.waypoints) == 0:
            logger.error("No mission loaded")
            return False

        if not self._home_set:
            logger.error("Home position not set")
            return False

        self.current_wp_index = 0
        self.state = MissionState.RUNNING
        self.nav_mode = NavigationMode.STRAIGHT
        self._turn_progress = 0.0

        # Reset L1 controller
        self.l1_controller.reset()

        # Set initial target
        self._update_navigation_target()

        logger.info("Mission started")
        return True

    def pause(self):
        """Pause mission and hold position"""
        if self.state == MissionState.RUNNING:
            self.state = MissionState.PAUSED
            self.nav_mode = NavigationMode.POSITION_HOLD
            self.pos_ctrl.controller.capture_hold_position()
            logger.info("Mission paused")

    def resume(self):
        """Resume paused mission"""
        if self.state == MissionState.PAUSED:
            self.state = MissionState.RUNNING
            self.nav_mode = NavigationMode.STRAIGHT
            self._update_navigation_target()
            logger.info("Mission resumed")

    def abort(self):
        """Abort mission"""
        self.state = MissionState.ABORTED
        self.nav_mode = NavigationMode.POSITION_HOLD
        logger.warning("Mission aborted")

    def update(self, dt: float = None) -> Tuple[Vector3, float, bool]:
        """
        Update mission progress and compute navigation output.

        Should be called in main control loop.

        Args:
            dt: Time step in seconds

        Returns:
            Tuple of (velocity_target, altitude_target, mission_active)
            - velocity_target: Target velocity in NED frame (m/s)
            - altitude_target: Target altitude (m)
            - mission_active: True if mission is still running
        """
        if self.state != MissionState.RUNNING:
            return (self._current_velocity_target,
                    self._current_altitude_target,
                    self.state not in (MissionState.COMPLETED, MissionState.ABORTED))

        if not self.mission:
            return Vector3(), self._current_alt, False

        # Get current segment
        segment = self.path_planner.get_segment(self.current_wp_index)
        if segment is None:
            # No more segments, use direct navigation
            return self._navigate_direct(dt)

        # Calculate distance to current waypoint
        dist_to_wp = self._calculate_distance_to_wp()

        # Check navigation mode transitions
        self._update_navigation_mode(segment, dist_to_wp)

        # Compute navigation based on mode
        if self.nav_mode == NavigationMode.STRAIGHT:
            velocity_target = self._navigate_straight(segment, dt)
        elif self.nav_mode == NavigationMode.TURNING:
            velocity_target = self._navigate_turn(segment, dt)
        elif self.nav_mode == NavigationMode.HOVERING:
            velocity_target = self._navigate_hover(dt)
        else:
            velocity_target = Vector3()

        # Apply speed adaptation
        if self.enable_speed_adaptation:
            velocity_target = self._apply_speed_profile(
                velocity_target, segment, dist_to_wp
            )

        # Calculate altitude target
        if self.enable_altitude_ramp and segment:
            altitude_target = self._calculate_ramped_altitude(segment)
        else:
            altitude_target = self.mission.waypoints[self.current_wp_index].altitude

        # Store current outputs
        self._current_velocity_target = velocity_target
        self._current_altitude_target = altitude_target

        # Check waypoint reached (for HOVER/LAND actions)
        current_wp = self.mission.waypoints[self.current_wp_index]
        if dist_to_wp < current_wp.radius:
            if not self._handle_waypoint_reached(current_wp):
                return velocity_target, altitude_target, False

        return velocity_target, altitude_target, True

    def _update_navigation_mode(self, segment: PathSegment, dist_to_wp: float):
        """Update navigation mode based on position"""
        current_wp = self.mission.waypoints[self.current_wp_index]

        if self.nav_mode == NavigationMode.HOVERING:
            # Stay in hover mode until hover time complete
            return

        if self.nav_mode == NavigationMode.STRAIGHT:
            # Check if we should start turning
            if (self.enable_turn_anticipation and
                current_wp.action == WaypointAction.FLY_THROUGH and
                segment.turn_start_distance_m > 0 and
                dist_to_wp <= segment.turn_start_distance_m):

                # Start turn anticipation
                self.nav_mode = NavigationMode.TURNING
                self._turn_progress = 0.0
                logger.debug(f"Starting turn at {dist_to_wp:.1f}m from WP{self.current_wp_index}")

        elif self.nav_mode == NavigationMode.TURNING:
            # Check if turn is complete (reached waypoint)
            if dist_to_wp < current_wp.radius:
                # Advance to next waypoint
                if self._advance_waypoint():
                    self.nav_mode = NavigationMode.STRAIGHT
                    self._turn_progress = 0.0
            else:
                # Update turn progress
                if segment.turn_start_distance_m > 0:
                    self._turn_progress = 1.0 - (dist_to_wp / segment.turn_start_distance_m)
                    self._turn_progress = max(0.0, min(1.0, self._turn_progress))

    def _navigate_straight(self, segment: PathSegment, dt: float) -> Vector3:
        """Navigate along straight path using L1 controller"""
        l1_output = self.l1_controller.update(
            self._current_lat, self._current_lon,
            self._vel_north, self._vel_east,
            segment, dt
        )

        self._cross_track_error = l1_output.cross_track_error

        return l1_output.velocity_target

    def _navigate_turn(self, segment: PathSegment, dt: float) -> Vector3:
        """Navigate through turn using blended L1"""
        next_segment = self.path_planner.get_segment(self.current_wp_index + 1)

        if next_segment is None:
            # No next segment, just navigate to waypoint
            return self._navigate_straight(segment, dt)

        l1_output = self.l1_controller.update_for_turn(
            self._current_lat, self._current_lon,
            self._vel_north, self._vel_east,
            segment, next_segment,
            self._turn_progress, dt
        )

        self._cross_track_error = l1_output.cross_track_error

        return l1_output.velocity_target

    def _navigate_hover(self, dt: float) -> Vector3:
        """Navigate for hovering (position hold)"""
        # Use position controller for hover
        self.pos_ctrl.update()
        return self.pos_ctrl.controller.velocity_target

    def _navigate_direct(self, dt: float) -> Tuple[Vector3, float, bool]:
        """Direct navigation when no path segment available"""
        if self.current_wp_index >= len(self.mission.waypoints):
            return Vector3(), self._current_alt, False

        wp = self.mission.waypoints[self.current_wp_index]
        self.pos_ctrl.set_target_gps(wp.latitude, wp.longitude, wp.altitude)
        vel = self.pos_ctrl.update()

        return Vector3(vel[0], vel[1], vel[2]), wp.altitude, True

    def _apply_speed_profile(self, velocity: Vector3, segment: PathSegment,
                            dist_to_wp: float) -> Vector3:
        """Apply speed reduction based on upcoming turn"""
        current_speed = math.sqrt(self._vel_north**2 + self._vel_east**2)
        target_speed = segment.exit_speed_ms

        # Calculate deceleration distance
        if current_speed > target_speed:
            decel_dist = (current_speed**2 - target_speed**2) / (2 * self.max_decel)
        else:
            decel_dist = 0

        # Determine speed limit
        if dist_to_wp < decel_dist:
            # We're in deceleration zone
            speed_limit = math.sqrt(target_speed**2 + 2 * self.max_decel * dist_to_wp)
        else:
            # Full speed
            speed_limit = segment.entry_speed_ms

        # Scale velocity to speed limit
        vel_magnitude = velocity.magnitude_xy()
        if vel_magnitude > speed_limit and vel_magnitude > 0.1:
            scale = speed_limit / vel_magnitude
            return Vector3(velocity.x * scale, velocity.y * scale, velocity.z)

        return velocity

    def _calculate_ramped_altitude(self, segment: PathSegment) -> float:
        """Calculate interpolated altitude along segment"""
        # Get progress along segment
        _, _, progress = self.path_planner.project_on_segment(
            self._current_lat, self._current_lon, segment
        )

        return self.path_planner.get_interpolated_altitude(segment, progress)

    def _calculate_distance_to_wp(self) -> float:
        """Calculate distance to current waypoint"""
        if self.mission is None or self.current_wp_index >= len(self.mission.waypoints):
            return 0.0

        wp = self.mission.waypoints[self.current_wp_index]
        return self._haversine_distance(
            self._current_lat, self._current_lon,
            wp.latitude, wp.longitude
        )

    def _handle_waypoint_reached(self, wp: Waypoint) -> bool:
        """Handle reaching a waypoint"""
        # For FLY_THROUGH, we already advanced in _update_navigation_mode
        if wp.action == WaypointAction.FLY_THROUGH:
            return True

        # Notify callback
        if self._on_waypoint_reached:
            self._on_waypoint_reached(self.current_wp_index, wp)

        if wp.action == WaypointAction.HOVER:
            if self._hover_start_time == 0:
                self._hover_start_time = time.time()
                self.nav_mode = NavigationMode.HOVERING
                self.pos_ctrl.controller.capture_hold_position()
                logger.info(f"Hovering at WP{self.current_wp_index} for {wp.hover_time}s")

            elif time.time() - self._hover_start_time >= wp.hover_time:
                self._hover_start_time = 0
                self.nav_mode = NavigationMode.STRAIGHT
                return self._advance_waypoint()

            return True

        elif wp.action == WaypointAction.LAND:
            self.state = MissionState.COMPLETED
            logger.info("Landing at final waypoint")
            return False

        elif wp.action == WaypointAction.RTH:
            self._set_rth_target()
            return True

        return True

    def _advance_waypoint(self) -> bool:
        """Advance to next waypoint"""
        self.current_wp_index += 1

        # Reset L1 integrator for new segment
        self.l1_controller.reset_integrator()

        if self.current_wp_index >= len(self.mission.waypoints):
            if self.mission.return_to_home:
                self._set_rth_target()
                logger.info("All waypoints complete, returning home")
            else:
                self.state = MissionState.COMPLETED
                if self._on_mission_complete:
                    self._on_mission_complete()
                logger.info("Mission completed")
                return False
        else:
            self._update_navigation_target()
            logger.info(f"Advancing to WP{self.current_wp_index}")

        return True

    def _update_navigation_target(self):
        """Update navigation target for current waypoint"""
        if not self.mission or self.current_wp_index >= len(self.mission.waypoints):
            return

        wp = self.mission.waypoints[self.current_wp_index]
        self.pos_ctrl.set_target_gps(wp.latitude, wp.longitude, wp.altitude)
        logger.debug(f"Target set to WP{self.current_wp_index}: {wp.latitude:.6f}, {wp.longitude:.6f}")

    def _set_rth_target(self):
        """Set target to home position"""
        self.pos_ctrl.set_target_gps(self.home_lat, self.home_lon, self.home_alt)
        logger.info("RTH target set")

    def _haversine_distance(self, lat1: float, lon1: float,
                           lat2: float, lon2: float) -> float:
        """Calculate distance between two GPS points in meters"""
        R = 6378137.0  # Earth radius in meters

        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        d_lat = lat2_rad - lat1_rad
        d_lon = math.radians(lon2 - lon1)

        a = (math.sin(d_lat / 2)**2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(d_lon / 2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    def set_waypoint_callback(self, callback: Callable[[int, Waypoint], None]):
        """Set callback for waypoint reached events"""
        self._on_waypoint_reached = callback

    def set_complete_callback(self, callback: Callable[[], None]):
        """Set callback for mission complete"""
        self._on_mission_complete = callback

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        """Get current target waypoint"""
        if self.mission and 0 <= self.current_wp_index < len(self.mission.waypoints):
            return self.mission.waypoints[self.current_wp_index]
        return None

    @property
    def current_segment(self) -> Optional[PathSegment]:
        """Get current path segment"""
        return self.path_planner.get_segment(self.current_wp_index)

    @property
    def progress(self) -> float:
        """Get mission progress 0.0 to 1.0"""
        if not self.mission or len(self.mission.waypoints) == 0:
            return 0.0
        return self.current_wp_index / len(self.mission.waypoints)

    @property
    def distance_to_waypoint(self) -> float:
        """Get distance to current waypoint in meters"""
        return self._calculate_distance_to_wp()

    @property
    def cross_track_error(self) -> float:
        """Get current cross-track error in meters"""
        return self._cross_track_error

    @property
    def velocity_target(self) -> Vector3:
        """Get current velocity target"""
        return self._current_velocity_target

    @property
    def altitude_target(self) -> float:
        """Get current altitude target"""
        return self._current_altitude_target

    def get_status(self) -> dict:
        """Get navigation status"""
        return {
            'state': self.state.name,
            'nav_mode': self.nav_mode.name,
            'mission_name': self.mission.name if self.mission else None,
            'waypoint_index': self.current_wp_index,
            'waypoint_count': len(self.mission.waypoints) if self.mission else 0,
            'progress': self.progress,
            'distance_to_wp': self.distance_to_waypoint,
            'cross_track_error': self._cross_track_error,
            'turn_progress': self._turn_progress,
            'speed_target': self._current_velocity_target.magnitude_xy(),
            'altitude_target': self._current_altitude_target,
            'current_wp': self.current_waypoint.to_dict() if self.current_waypoint else None
        }

    def get_debug_info(self) -> dict:
        """Get detailed debug information"""
        segment = self.current_segment
        return {
            'nav_mode': self.nav_mode.name,
            'cross_track_error_m': self._cross_track_error,
            'turn_progress': self._turn_progress,
            'velocity_target_n': self._current_velocity_target.x,
            'velocity_target_e': self._current_velocity_target.y,
            'altitude_target': self._current_altitude_target,
            'segment': {
                'index': segment.index if segment else None,
                'bearing_deg': math.degrees(segment.bearing_rad) if segment else None,
                'distance_m': segment.distance_m if segment else None,
                'turn_angle_deg': math.degrees(segment.turn_angle_rad) if segment else None,
                'turn_start_m': segment.turn_start_distance_m if segment else None
            },
            'l1': self.l1_controller.get_debug_info(),
            'path_planner': self.path_planner.get_debug_info()
        }
