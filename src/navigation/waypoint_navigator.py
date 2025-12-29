"""
Waypoint Navigator

Manages waypoint missions with fly-through navigation.
"""

import json
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Callable
from pathlib import Path
import logging

from .position_controller import PositionControllerGPS

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


class WaypointNavigator:
    """
    Waypoint mission navigator

    Executes fly-through waypoint missions with:
    - Smooth transitions between waypoints
    - Speed management
    - Mission state tracking
    """

    def __init__(self, position_controller: PositionControllerGPS):
        """
        Initialize waypoint navigator

        Args:
            position_controller: GPS position controller instance
        """
        self.pos_ctrl = position_controller

        # Mission
        self.mission: Optional[Mission] = None
        self.state = MissionState.IDLE

        # Progress
        self.current_wp_index = 0
        self._hover_start_time = 0.0

        # Home position
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0
        self._home_set = False

        # Callbacks
        self._on_waypoint_reached: Optional[Callable[[int, Waypoint], None]] = None
        self._on_mission_complete: Optional[Callable[[], None]] = None

    def load_mission(self, mission: Mission):
        """Load a mission"""
        self.mission = mission
        self.state = MissionState.IDLE
        self.current_wp_index = 0
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
        logger.info(f"Home set: {lat:.6f}, {lon:.6f}, {alt:.1f}m")

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
        self._set_current_waypoint_target()

        logger.info("Mission started")
        return True

    def pause(self):
        """Pause mission and hold position"""
        if self.state == MissionState.RUNNING:
            self.state = MissionState.PAUSED
            self.pos_ctrl.controller.capture_hold_position()
            logger.info("Mission paused")

    def resume(self):
        """Resume paused mission"""
        if self.state == MissionState.PAUSED:
            self.state = MissionState.RUNNING
            self._set_current_waypoint_target()
            logger.info("Mission resumed")

    def abort(self):
        """Abort mission"""
        self.state = MissionState.ABORTED
        logger.warning("Mission aborted")

    def update(self) -> bool:
        """
        Update mission progress

        Should be called in main control loop.

        Returns:
            True if mission is still active
        """
        if self.state != MissionState.RUNNING:
            return self.state not in (MissionState.COMPLETED, MissionState.ABORTED)

        if not self.mission:
            return False

        current_wp = self.mission.waypoints[self.current_wp_index]

        # Check if reached current waypoint
        if self.pos_ctrl.is_at_target(current_wp.radius):
            return self._handle_waypoint_reached(current_wp)

        return True

    def _handle_waypoint_reached(self, wp: Waypoint) -> bool:
        """Handle reaching a waypoint"""

        # Notify callback
        if self._on_waypoint_reached:
            self._on_waypoint_reached(self.current_wp_index, wp)

        # Handle action
        if wp.action == WaypointAction.FLY_THROUGH:
            # Immediately advance to next waypoint
            return self._advance_waypoint()

        elif wp.action == WaypointAction.HOVER:
            # Hover for specified time
            if self._hover_start_time == 0:
                self._hover_start_time = time.time()
                self.pos_ctrl.controller.capture_hold_position()
                logger.info(f"Hovering at WP{self.current_wp_index} for {wp.hover_time}s")

            elif time.time() - self._hover_start_time >= wp.hover_time:
                self._hover_start_time = 0
                return self._advance_waypoint()

            return True

        elif wp.action == WaypointAction.LAND:
            # Signal landing (handled by flight controller)
            self.state = MissionState.COMPLETED
            logger.info("Landing at final waypoint")
            return False

        elif wp.action == WaypointAction.RTH:
            # Return to home
            self._set_rth_target()
            return True

        return True

    def _advance_waypoint(self) -> bool:
        """Advance to next waypoint"""
        self.current_wp_index += 1

        if self.current_wp_index >= len(self.mission.waypoints):
            # Mission complete
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
            self._set_current_waypoint_target()
            logger.info(f"Advancing to WP{self.current_wp_index}")

        return True

    def _set_current_waypoint_target(self):
        """Set position target to current waypoint"""
        if not self.mission or self.current_wp_index >= len(self.mission.waypoints):
            return

        wp = self.mission.waypoints[self.current_wp_index]
        self.pos_ctrl.set_target_gps(wp.latitude, wp.longitude, wp.altitude)
        logger.debug(f"Target set to WP{self.current_wp_index}: {wp.latitude:.6f}, {wp.longitude:.6f}")

    def _set_rth_target(self):
        """Set target to home position"""
        self.pos_ctrl.set_target_gps(self.home_lat, self.home_lon, self.home_alt)
        logger.info("RTH target set")

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
    def progress(self) -> float:
        """Get mission progress 0.0 to 1.0"""
        if not self.mission or len(self.mission.waypoints) == 0:
            return 0.0
        return self.current_wp_index / len(self.mission.waypoints)

    @property
    def distance_to_waypoint(self) -> float:
        """Get distance to current waypoint in meters"""
        return self.pos_ctrl.distance_to_target

    def get_status(self) -> dict:
        """Get navigation status"""
        return {
            'state': self.state.name,
            'mission_name': self.mission.name if self.mission else None,
            'waypoint_index': self.current_wp_index,
            'waypoint_count': len(self.mission.waypoints) if self.mission else 0,
            'progress': self.progress,
            'distance_to_wp': self.distance_to_waypoint,
            'current_wp': self.current_waypoint.to_dict() if self.current_waypoint else None
        }
