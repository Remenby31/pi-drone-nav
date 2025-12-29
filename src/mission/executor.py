"""
Mission Executor

Executes mission actions sequentially with state management.
"""

import logging
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, List, Optional, Any, TYPE_CHECKING

from .models import (
    Action,
    ActionType,
    DelayAction,
    GotoAction,
    HoverAction,
    LandAction,
    Mission,
    OrientAction,
    PhotoAction,
    RthAction,
    TakeoffAction,
)

if TYPE_CHECKING:
    from ..navigation.path_planner import PathPlanner
    from ..navigation.l1_controller import L1Controller

logger = logging.getLogger(__name__)


class ExecutorState(Enum):
    """Mission executor states"""
    IDLE = auto()       # No mission loaded
    READY = auto()      # Mission loaded, waiting to start
    RUNNING = auto()    # Executing actions
    PAUSED = auto()     # Paused, can resume
    COMPLETED = auto()  # Mission finished successfully
    STOPPED = auto()    # Stopped by user


class ActionState(Enum):
    """State of current action execution"""
    PENDING = auto()    # Not started
    EXECUTING = auto()  # In progress
    COMPLETED = auto()  # Done


@dataclass
class ExecutorOutput:
    """Output from executor update cycle"""
    # Navigation commands
    target_lat: Optional[float] = None
    target_lon: Optional[float] = None
    target_alt: Optional[float] = None
    target_speed: Optional[float] = None
    target_heading: Optional[float] = None

    # State flags
    should_takeoff: bool = False
    should_land: bool = False
    should_rth: bool = False
    should_hold: bool = False
    should_photo: bool = False

    # For path following (segments)
    velocity_target_n: float = 0.0
    velocity_target_e: float = 0.0
    use_velocity_control: bool = False


@dataclass
class ExecutorStatus:
    """Current executor status for API"""
    state: str
    mission_uuid: Optional[str]
    mission_name: Optional[str]
    current_action_index: int
    current_action: Optional[Dict[str, Any]]
    action_count: int
    progress_percent: float
    action_state: str
    action_elapsed_s: float
    segment_index: int
    segment_progress: float


class MissionExecutor:
    """
    Executes mission actions with segment-aware path following

    Handles:
    - Sequential action execution
    - Segment detection (consecutive gotos)
    - Path planning integration
    - Pause/resume/skip functionality
    """

    def __init__(self, defaults_speed: float = 5.0, defaults_alt: float = 10.0):
        """
        Initialize executor

        Args:
            defaults_speed: Default speed in m/s
            defaults_alt: Default altitude in meters
        """
        self.defaults_speed = defaults_speed
        self.defaults_alt = defaults_alt

        # State
        self.state = ExecutorState.IDLE
        self.mission: Optional[Mission] = None

        # Progress tracking
        self.current_action_index = 0
        self.action_state = ActionState.PENDING
        self._action_start_time = 0.0

        # Current position (updated externally)
        self._current_lat = 0.0
        self._current_lon = 0.0
        self._current_alt = 0.0
        self._current_heading = 0.0

        # Home position
        self._home_lat = 0.0
        self._home_lon = 0.0
        self._home_alt = 0.0
        self._home_set = False

        # Segment tracking
        self._segments: List[List[GotoAction]] = []
        self._current_segment_index = 0
        self._segment_wp_index = 0  # Index within current segment

        # Completion thresholds
        self.position_threshold_m = 2.0  # meters
        self.altitude_threshold_m = 1.0  # meters
        self.heading_threshold_deg = 5.0  # degrees

        # Callbacks
        self._on_action_complete: Optional[Callable[[int, Action], None]] = None
        self._on_mission_complete: Optional[Callable[[], None]] = None
        self._on_photo_trigger: Optional[Callable[[], None]] = None
        self._on_disarm_request: Optional[Callable[[], None]] = None

    def load(self, mission: Mission):
        """
        Load a mission for execution

        Args:
            mission: Mission to execute
        """
        self.mission = mission
        self.state = ExecutorState.READY
        self.current_action_index = 0
        self.action_state = ActionState.PENDING

        # Pre-calculate segments
        self._segments = mission.get_segments()
        self._current_segment_index = 0
        self._segment_wp_index = 0

        logger.info(
            f"Loaded mission '{mission.name}' with {mission.action_count} actions, "
            f"{len(self._segments)} segments"
        )

    def set_home(self, lat: float, lon: float, alt: float):
        """Set home position for RTH"""
        self._home_lat = lat
        self._home_lon = lon
        self._home_alt = alt
        self._home_set = True
        logger.debug(f"Home set: {lat:.6f}, {lon:.6f}, {alt:.1f}m")

    def update_position(self, lat: float, lon: float, alt: float, heading: float = 0.0):
        """
        Update current position

        Must be called each control cycle before update().
        """
        self._current_lat = lat
        self._current_lon = lon
        self._current_alt = alt
        self._current_heading = heading

    def start(self) -> bool:
        """
        Start mission execution

        Returns:
            True if started successfully
        """
        if self.state != ExecutorState.READY:
            logger.warning(f"Cannot start: state is {self.state.name}")
            return False

        if not self.mission:
            logger.error("No mission loaded")
            return False

        if not self._home_set:
            logger.warning("Home position not set, using current position")
            self.set_home(self._current_lat, self._current_lon, self._current_alt)

        self.state = ExecutorState.RUNNING
        self.current_action_index = 0
        self.action_state = ActionState.PENDING
        self._action_start_time = time.time()

        logger.info(f"Mission '{self.mission.name}' started")
        return True

    def pause(self):
        """Pause mission execution"""
        if self.state == ExecutorState.RUNNING:
            self.state = ExecutorState.PAUSED
            logger.info("Mission paused")

    def resume(self):
        """Resume paused mission"""
        if self.state == ExecutorState.PAUSED:
            self.state = ExecutorState.RUNNING
            logger.info("Mission resumed")

    def stop(self):
        """Stop mission execution"""
        if self.state in (ExecutorState.RUNNING, ExecutorState.PAUSED):
            self.state = ExecutorState.STOPPED
            logger.info("Mission stopped")

    def skip(self):
        """Skip current action and advance to next"""
        if self.state != ExecutorState.RUNNING:
            return

        logger.info(f"Skipping action {self.current_action_index}")
        self._advance_action()

    def goto_action(self, index: int) -> bool:
        """
        Jump to specific action index

        Args:
            index: Action index to jump to

        Returns:
            True if jump successful
        """
        if self.state not in (ExecutorState.RUNNING, ExecutorState.PAUSED):
            return False

        if not self.mission or not (0 <= index < self.mission.action_count):
            return False

        self.current_action_index = index
        self.action_state = ActionState.PENDING
        self._action_start_time = time.time()

        # Update segment tracking
        self._update_segment_tracking()

        logger.info(f"Jumped to action {index}")
        return True

    def update(self, dt: float = 0.02) -> ExecutorOutput:
        """
        Update mission execution

        Should be called each control cycle.

        Args:
            dt: Time step in seconds

        Returns:
            ExecutorOutput with navigation commands
        """
        output = ExecutorOutput()

        if self.state != ExecutorState.RUNNING:
            output.should_hold = True
            return output

        if not self.mission:
            return output

        # Get current action
        if self.current_action_index >= self.mission.action_count:
            self._complete_mission()
            return output

        action = self.mission.actions[self.current_action_index]

        # Start action if pending
        if self.action_state == ActionState.PENDING:
            self.action_state = ActionState.EXECUTING
            self._action_start_time = time.time()
            logger.debug(f"Starting action {self.current_action_index}: {action.action_type.value}")

        # Execute action based on type
        completed = False

        if isinstance(action, TakeoffAction):
            output, completed = self._execute_takeoff(action)

        elif isinstance(action, GotoAction):
            output, completed = self._execute_goto(action)

        elif isinstance(action, HoverAction):
            output, completed = self._execute_hover(action)

        elif isinstance(action, OrientAction):
            output, completed = self._execute_orient(action)

        elif isinstance(action, DelayAction):
            output, completed = self._execute_delay(action)

        elif isinstance(action, PhotoAction):
            output, completed = self._execute_photo(action)

        elif isinstance(action, LandAction):
            output, completed = self._execute_land(action)

        elif isinstance(action, RthAction):
            output, completed = self._execute_rth(action)

        # Advance if completed
        if completed:
            self._on_action_completed(action)
            self._advance_action()

        return output

    def _execute_takeoff(self, action: TakeoffAction) -> tuple[ExecutorOutput, bool]:
        """Execute takeoff action"""
        output = ExecutorOutput()
        output.should_takeoff = True
        output.target_alt = action.alt

        # Check if reached altitude
        completed = self._current_alt >= (action.alt - self.altitude_threshold_m)

        return output, completed

    def _execute_goto(self, action: GotoAction) -> tuple[ExecutorOutput, bool]:
        """Execute goto action"""
        output = ExecutorOutput()

        # Resolve altitude (use default if not specified)
        target_alt = action.alt if action.alt is not None else self.defaults_alt

        output.target_lat = action.lat
        output.target_lon = action.lon
        output.target_alt = target_alt
        output.target_speed = action.speed if action.speed is not None else self.defaults_speed

        # Check if reached position
        distance = self._distance_to(action.lat, action.lon)
        alt_diff = abs(self._current_alt - target_alt)

        completed = (distance < self.position_threshold_m and
                     alt_diff < self.altitude_threshold_m)

        return output, completed

    def _execute_hover(self, action: HoverAction) -> tuple[ExecutorOutput, bool]:
        """Execute hover action"""
        output = ExecutorOutput()
        output.should_hold = True

        # Check if duration elapsed
        elapsed = time.time() - self._action_start_time
        completed = elapsed >= action.duration

        return output, completed

    def _execute_orient(self, action: OrientAction) -> tuple[ExecutorOutput, bool]:
        """Execute orient action"""
        output = ExecutorOutput()
        output.should_hold = True
        output.target_heading = action.heading

        # Check if heading reached
        heading_diff = abs(self._normalize_heading(
            action.heading - self._current_heading
        ))
        completed = heading_diff < self.heading_threshold_deg

        return output, completed

    def _execute_delay(self, action: DelayAction) -> tuple[ExecutorOutput, bool]:
        """Execute delay action"""
        output = ExecutorOutput()
        output.should_hold = True

        # Check if duration elapsed
        elapsed = time.time() - self._action_start_time
        completed = elapsed >= action.duration

        return output, completed

    def _execute_photo(self, action: PhotoAction) -> tuple[ExecutorOutput, bool]:
        """Execute photo action"""
        output = ExecutorOutput()
        output.should_hold = True
        output.should_photo = True

        # Trigger callback
        if self._on_photo_trigger:
            self._on_photo_trigger()

        # Photo is instant
        return output, True

    def _execute_land(self, action: LandAction) -> tuple[ExecutorOutput, bool]:
        """Execute land action"""
        output = ExecutorOutput()
        output.should_land = True

        # Land action completes when altitude is near zero
        # In practice, flight controller will handle landing detection
        completed = self._current_alt < 0.3

        return output, completed

    def _execute_rth(self, action: RthAction) -> tuple[ExecutorOutput, bool]:
        """Execute RTH action"""
        output = ExecutorOutput()
        output.should_rth = True
        output.target_lat = self._home_lat
        output.target_lon = self._home_lon
        output.target_alt = self._home_alt

        # RTH completes when reached home and landed
        distance = self._distance_to(self._home_lat, self._home_lon)
        completed = distance < self.position_threshold_m and self._current_alt < 0.3

        return output, completed

    def _advance_action(self):
        """Advance to next action"""
        self.current_action_index += 1
        self.action_state = ActionState.PENDING
        self._action_start_time = time.time()

        # Update segment tracking
        self._update_segment_tracking()

        # Check if mission complete
        if self.mission and self.current_action_index >= self.mission.action_count:
            self._complete_mission()

    def _complete_mission(self):
        """Mark mission as completed and request disarm"""
        self.state = ExecutorState.COMPLETED
        logger.info(f"Mission '{self.mission.name}' completed")

        if self._on_mission_complete:
            self._on_mission_complete()

        # Request disarm after mission completes (land/rth finished)
        if self._on_disarm_request:
            self._on_disarm_request()

    def _on_action_completed(self, action: Action):
        """Called when an action completes"""
        logger.debug(f"Action {self.current_action_index} completed: {action.action_type.value}")

        if self._on_action_complete:
            self._on_action_complete(self.current_action_index, action)

    def _update_segment_tracking(self):
        """Update segment index based on current action"""
        if not self.mission:
            return

        # Find which segment contains current goto (if any)
        goto_count = 0
        for i, action in enumerate(self.mission.actions):
            if i == self.current_action_index:
                break
            if isinstance(action, GotoAction):
                goto_count += 1

        # Find segment
        cumulative = 0
        for seg_idx, segment in enumerate(self._segments):
            if cumulative + len(segment) > goto_count:
                self._current_segment_index = seg_idx
                self._segment_wp_index = goto_count - cumulative
                return
            cumulative += len(segment)

    def _distance_to(self, lat: float, lon: float) -> float:
        """Calculate distance to point in meters"""
        R = 6378137.0  # Earth radius

        lat1_rad = math.radians(self._current_lat)
        lat2_rad = math.radians(lat)
        d_lat = lat2_rad - lat1_rad
        d_lon = math.radians(lon - self._current_lon)

        a = (math.sin(d_lat / 2) ** 2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(d_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    @staticmethod
    def _normalize_heading(heading: float) -> float:
        """Normalize heading to -180 to 180"""
        while heading > 180:
            heading -= 360
        while heading < -180:
            heading += 360
        return heading

    def get_status(self) -> ExecutorStatus:
        """Get current executor status"""
        current_action = None
        if self.mission and 0 <= self.current_action_index < self.mission.action_count:
            current_action = self.mission.actions[self.current_action_index].to_dict()

        action_count = self.mission.action_count if self.mission else 0
        progress = (self.current_action_index / action_count * 100) if action_count > 0 else 0

        return ExecutorStatus(
            state=self.state.name,
            mission_uuid=self.mission.uuid if self.mission else None,
            mission_name=self.mission.name if self.mission else None,
            current_action_index=self.current_action_index,
            current_action=current_action,
            action_count=action_count,
            progress_percent=progress,
            action_state=self.action_state.name,
            action_elapsed_s=time.time() - self._action_start_time if self._action_start_time else 0,
            segment_index=self._current_segment_index,
            segment_progress=self._segment_wp_index / len(self._segments[self._current_segment_index])
            if self._segments and self._current_segment_index < len(self._segments)
            else 0,
        )

    def get_status_dict(self) -> Dict[str, Any]:
        """Get status as dictionary for JSON serialization"""
        status = self.get_status()
        return {
            "state": status.state,
            "mission_uuid": status.mission_uuid,
            "mission_name": status.mission_name,
            "current_action_index": status.current_action_index,
            "current_action": status.current_action,
            "action_count": status.action_count,
            "progress_percent": status.progress_percent,
            "action_state": status.action_state,
            "action_elapsed_s": status.action_elapsed_s,
            "segment_index": status.segment_index,
            "segment_progress": status.segment_progress,
        }

    # Callback setters
    def on_action_complete(self, callback: Callable[[int, Action], None]):
        """Set callback for action completion"""
        self._on_action_complete = callback

    def on_mission_complete(self, callback: Callable[[], None]):
        """Set callback for mission completion"""
        self._on_mission_complete = callback

    def on_photo_trigger(self, callback: Callable[[], None]):
        """Set callback for photo trigger"""
        self._on_photo_trigger = callback

    def on_disarm_request(self, callback: Callable[[], None]):
        """Set callback for disarm request (called when mission ends)"""
        self._on_disarm_request = callback
