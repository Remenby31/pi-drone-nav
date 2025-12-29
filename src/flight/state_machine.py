"""
Flight State Machine

Manages flight states and transitions with safety checks.
"""

from enum import Enum, auto
from typing import Optional, Set, Dict, Callable
import time
import logging

logger = logging.getLogger(__name__)


class FlightState(Enum):
    """Flight states"""
    IDLE = auto()           # Disarmed, on ground
    PREFLIGHT = auto()      # Pre-flight checks (legacy)
    PREFLIGHT_CHECK = auto()  # Running pre-flight checks before arming
    ARMED = auto()          # Armed, ready for takeoff
    TAKEOFF = auto()        # Taking off
    HOVER = auto()          # Hovering in place
    POSITION_HOLD = auto()  # GPS position hold
    FLYING = auto()         # Flying to waypoint
    MISSION = auto()        # Executing mission
    RTH = auto()            # Return to home
    LANDING = auto()        # Landing
    LANDED = auto()         # Landed, still armed
    FAILSAFE = auto()       # Failsafe active
    ERROR = auto()          # Error state


# Valid state transitions
VALID_TRANSITIONS: Dict[FlightState, Set[FlightState]] = {
    FlightState.IDLE: {FlightState.PREFLIGHT, FlightState.PREFLIGHT_CHECK, FlightState.ARMED, FlightState.ERROR},
    FlightState.PREFLIGHT: {FlightState.ARMED, FlightState.IDLE, FlightState.ERROR},
    FlightState.PREFLIGHT_CHECK: {FlightState.ARMED, FlightState.IDLE, FlightState.ERROR},
    FlightState.ARMED: {FlightState.TAKEOFF, FlightState.IDLE, FlightState.FAILSAFE, FlightState.ERROR},
    FlightState.TAKEOFF: {FlightState.HOVER, FlightState.POSITION_HOLD, FlightState.FAILSAFE,
                          FlightState.LANDING, FlightState.IDLE, FlightState.ERROR},
    FlightState.HOVER: {FlightState.POSITION_HOLD, FlightState.FLYING, FlightState.MISSION,
                        FlightState.RTH, FlightState.LANDING, FlightState.FAILSAFE, FlightState.ERROR},
    FlightState.POSITION_HOLD: {FlightState.HOVER, FlightState.FLYING, FlightState.MISSION,
                                 FlightState.RTH, FlightState.LANDING, FlightState.FAILSAFE, FlightState.ERROR},
    FlightState.FLYING: {FlightState.HOVER, FlightState.POSITION_HOLD, FlightState.MISSION,
                         FlightState.RTH, FlightState.LANDING, FlightState.FAILSAFE, FlightState.ERROR},
    FlightState.MISSION: {FlightState.HOVER, FlightState.POSITION_HOLD, FlightState.RTH,
                          FlightState.LANDING, FlightState.FAILSAFE, FlightState.ERROR},
    FlightState.RTH: {FlightState.HOVER, FlightState.LANDING, FlightState.FAILSAFE, FlightState.ERROR},
    FlightState.LANDING: {FlightState.LANDED, FlightState.HOVER, FlightState.FAILSAFE, FlightState.ERROR},
    FlightState.LANDED: {FlightState.IDLE, FlightState.TAKEOFF, FlightState.ERROR},
    FlightState.FAILSAFE: {FlightState.LANDING, FlightState.IDLE, FlightState.ERROR},
    FlightState.ERROR: {FlightState.IDLE},
}


class FlightStateMachine:
    """
    Flight state machine with safety enforcement

    Manages state transitions and ensures only valid
    transitions are allowed.
    """

    def __init__(self):
        """Initialize state machine"""
        self._state = FlightState.IDLE
        self._previous_state = FlightState.IDLE
        self._state_enter_time = time.time()

        # State callbacks
        self._on_enter: Dict[FlightState, Callable[[], None]] = {}
        self._on_exit: Dict[FlightState, Callable[[], None]] = {}
        self._on_transition: Optional[Callable[[FlightState, FlightState], None]] = None

        # State timeouts (0 = no timeout)
        self._timeouts: Dict[FlightState, float] = {
            FlightState.TAKEOFF: 30.0,    # 30s max for takeoff
            FlightState.LANDING: 60.0,    # 60s max for landing
        }

    @property
    def state(self) -> FlightState:
        """Current state"""
        return self._state

    @property
    def previous_state(self) -> FlightState:
        """Previous state"""
        return self._previous_state

    @property
    def time_in_state(self) -> float:
        """Time in current state (seconds)"""
        return time.time() - self._state_enter_time

    @property
    def is_flying(self) -> bool:
        """Check if drone is in a flying state"""
        return self._state in {
            FlightState.TAKEOFF,
            FlightState.HOVER,
            FlightState.POSITION_HOLD,
            FlightState.FLYING,
            FlightState.MISSION,
            FlightState.RTH,
            FlightState.LANDING
        }

    @property
    def is_armed(self) -> bool:
        """Check if drone is armed"""
        return self._state not in {FlightState.IDLE, FlightState.PREFLIGHT, FlightState.ERROR}

    def can_transition_to(self, new_state: FlightState) -> bool:
        """Check if transition to new state is valid"""
        return new_state in VALID_TRANSITIONS.get(self._state, set())

    def transition_to(self, new_state: FlightState, force: bool = False) -> bool:
        """
        Attempt to transition to a new state

        Args:
            new_state: Target state
            force: If True, bypass validation (use with caution)

        Returns:
            True if transition successful
        """
        if not force and not self.can_transition_to(new_state):
            logger.warning(f"Invalid transition: {self._state.name} -> {new_state.name}")
            return False

        old_state = self._state

        # Call exit callback
        if old_state in self._on_exit:
            try:
                self._on_exit[old_state]()
            except Exception as e:
                logger.error(f"Error in exit callback for {old_state.name}: {e}")

        # Update state
        self._previous_state = old_state
        self._state = new_state
        self._state_enter_time = time.time()

        logger.info(f"State transition: {old_state.name} -> {new_state.name}")

        # Call enter callback
        if new_state in self._on_enter:
            try:
                self._on_enter[new_state]()
            except Exception as e:
                logger.error(f"Error in enter callback for {new_state.name}: {e}")

        # Call general transition callback
        if self._on_transition:
            try:
                self._on_transition(old_state, new_state)
            except Exception as e:
                logger.error(f"Error in transition callback: {e}")

        return True

    def check_timeout(self) -> bool:
        """
        Check if current state has timed out

        Returns:
            True if timed out
        """
        timeout = self._timeouts.get(self._state, 0)
        if timeout > 0 and self.time_in_state > timeout:
            logger.warning(f"State {self._state.name} timed out after {timeout}s")
            return True
        return False

    def on_enter(self, state: FlightState, callback: Callable[[], None] = None):
        """Register callback for entering a state (can be used as decorator)"""
        def decorator(func):
            self._on_enter[state] = func
            return func

        if callback is not None:
            self._on_enter[state] = callback
            return None
        return decorator

    def on_exit(self, state: FlightState, callback: Callable[[], None] = None):
        """Register callback for exiting a state (can be used as decorator)"""
        def decorator(func):
            self._on_exit[state] = func
            return func

        if callback is not None:
            self._on_exit[state] = callback
            return None
        return decorator

    def on_transition(self, callback: Callable[[FlightState, FlightState], None]):
        """Register callback for any state transition"""
        self._on_transition = callback

    def set_timeout(self, state: FlightState, timeout: float):
        """Set timeout for a state"""
        self._timeouts[state] = timeout

    def reset(self):
        """Reset to IDLE state"""
        self.transition_to(FlightState.IDLE, force=True)

    def trigger_failsafe(self, reason: str = ""):
        """Trigger failsafe mode"""
        logger.error(f"FAILSAFE triggered: {reason}")
        self.transition_to(FlightState.FAILSAFE, force=True)

    def trigger_error(self, reason: str = ""):
        """Trigger error state"""
        logger.error(f"ERROR state triggered: {reason}")
        self.transition_to(FlightState.ERROR, force=True)

    def get_status(self) -> dict:
        """Get state machine status"""
        return {
            'state': self._state.name,
            'previous_state': self._previous_state.name,
            'time_in_state': self.time_in_state,
            'is_flying': self.is_flying,
            'is_armed': self.is_armed,
            'valid_transitions': [s.name for s in VALID_TRANSITIONS.get(self._state, set())]
        }
