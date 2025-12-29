"""
Takeoff Controller - iNav/ArduPilot style phased takeoff

Implements a sophisticated takeoff sequence with:
- Motor spinup phase
- Throttle ramping with jerk limiting
- Liftoff detection (altitude + climb rate)
- Ground effect compensation
- Abort conditions and failsafes

Inspired by:
- iNav NAV_LAUNCH_MODE and takeoff handling
- ArduPilot's Copter takeoff sequence
"""

from enum import Enum, auto
from dataclasses import dataclass
from typing import Optional, Callable, TYPE_CHECKING
import time
import logging

if TYPE_CHECKING:
    from ..config import TakeoffConfig
    from ..navigation.altitude_controller import AltitudeController

logger = logging.getLogger(__name__)


class TakeoffPhase(Enum):
    """
    Takeoff sequence phases (inspired by iNav)

    The takeoff proceeds through these phases in order:
    MOTOR_SPINUP -> THROTTLE_RAMP -> LIFTOFF_DETECT -> CLIMB -> STABILIZE -> COMPLETE

    Any phase can transition to ABORTED on error.
    """
    IDLE = auto()              # Not in takeoff
    MOTOR_SPINUP = auto()      # Spinning motors to idle, checking systems
    THROTTLE_RAMP = auto()     # Ramping throttle, waiting for liftoff
    LIFTOFF_DETECT = auto()    # Confirming liftoff via sensors
    CLIMB = auto()             # Climbing to target altitude
    STABILIZE = auto()         # Brief stabilization before handoff
    COMPLETE = auto()          # Takeoff successful
    ABORTED = auto()           # Takeoff aborted


@dataclass
class TakeoffState:
    """Internal state tracking for takeoff sequence"""
    phase: TakeoffPhase = TakeoffPhase.IDLE
    phase_start_time: float = 0.0

    # Targets
    target_altitude_m: float = 3.0

    # Throttle state
    current_throttle: float = 0.0
    throttle_at_liftoff: float = 0.0

    # Liftoff tracking
    ground_altitude_m: float = 0.0
    liftoff_detected: bool = False
    liftoff_time: float = 0.0

    # Abort tracking
    abort_reason: str = ""
    highest_altitude_m: float = 0.0


class TakeoffController:
    """
    Manages the complete takeoff sequence

    Phases:
    1. MOTOR_SPINUP: Spin motors to minimum, verify systems responding
    2. THROTTLE_RAMP: Smoothly increase throttle until liftoff detected
    3. LIFTOFF_DETECT: Confirm actual liftoff via altitude + climb rate
    4. CLIMB: Use altitude controller to reach target
    5. STABILIZE: Brief hover to ensure stability before position control
    6. COMPLETE: Hand off to position hold

    The controller provides throttle commands during takeoff, then
    hands control to the normal altitude controller.
    """

    def __init__(self, config: 'TakeoffConfig',
                 alt_controller: 'AltitudeController'):
        """
        Initialize takeoff controller

        Args:
            config: TakeoffConfig with takeoff parameters
            alt_controller: AltitudeController for climb phase
        """
        self.config = config
        self.alt_controller = alt_controller
        self.state = TakeoffState()

        # Callbacks
        self._on_complete: Optional[Callable[[], None]] = None
        self._on_abort: Optional[Callable[[str], None]] = None

        # Throttle ramping with jerk limiting
        self._throttle_rate = 0.0  # Current throttle rate of change
        self._max_throttle_jerk = 0.5  # Max rate of change of rate (throttle/s^2)

    def start(self, target_altitude_m: float):
        """
        Begin takeoff sequence

        Args:
            target_altitude_m: Target altitude in meters
        """
        logger.info(f"Starting takeoff to {target_altitude_m}m")

        current_alt = self.alt_controller.current_altitude

        self.state = TakeoffState(
            phase=TakeoffPhase.MOTOR_SPINUP,
            phase_start_time=time.time(),
            target_altitude_m=target_altitude_m,
            ground_altitude_m=current_alt,
            current_throttle=0.0,  # Start from zero
            highest_altitude_m=current_alt
        )

        self._throttle_rate = 0.0

    def abort(self, reason: str):
        """
        Abort takeoff sequence

        Args:
            reason: Description of why takeoff was aborted
        """
        logger.warning(f"Takeoff ABORTED: {reason}")
        self.state.phase = TakeoffPhase.ABORTED
        self.state.abort_reason = reason

        if self._on_abort:
            self._on_abort(reason)

    def update(self, dt: float, altitude_m: float, climb_rate_ms: float,
               roll_deg: float, pitch_deg: float) -> float:
        """
        Update takeoff controller

        This should be called every control loop iteration during takeoff.

        Args:
            dt: Time step in seconds
            altitude_m: Current altitude from FC/GPS in meters
            climb_rate_ms: Current vertical speed in m/s (positive = up)
            roll_deg: Current roll angle in degrees
            pitch_deg: Current pitch angle in degrees

        Returns:
            Throttle command (0.0 - 1.0)
        """
        # No output in terminal states
        if self.state.phase in (TakeoffPhase.IDLE, TakeoffPhase.COMPLETE,
                                 TakeoffPhase.ABORTED):
            return 0.0

        # Check abort conditions (all phases)
        if self._check_abort_conditions(altitude_m, climb_rate_ms, roll_deg, pitch_deg):
            return self.config.initial_throttle  # Safe throttle on abort

        # Phase-specific updates
        phase_handlers = {
            TakeoffPhase.MOTOR_SPINUP: self._update_spinup,
            TakeoffPhase.THROTTLE_RAMP: self._update_throttle_ramp,
            TakeoffPhase.LIFTOFF_DETECT: self._update_liftoff_detect,
            TakeoffPhase.CLIMB: self._update_climb,
            TakeoffPhase.STABILIZE: self._update_stabilize,
        }

        handler = phase_handlers.get(self.state.phase)
        if handler:
            return handler(dt, altitude_m, climb_rate_ms)

        return self.state.current_throttle

    def _check_abort_conditions(self, altitude_m: float, climb_rate_ms: float,
                                 roll_deg: float, pitch_deg: float) -> bool:
        """
        Check for conditions that should abort takeoff

        Returns:
            True if takeoff should be aborted
        """
        # Excessive tilt (could indicate crash or instability)
        tilt = (roll_deg**2 + pitch_deg**2) ** 0.5
        if tilt > self.config.max_tilt_abort_deg:
            self.abort(f"Excessive tilt: {tilt:.1f} deg")
            return True

        # Altitude loss during climb (only after liftoff)
        if self.state.liftoff_detected:
            self.state.highest_altitude_m = max(self.state.highest_altitude_m, altitude_m)
            altitude_loss = self.state.highest_altitude_m - altitude_m

            if altitude_loss > self.config.altitude_loss_abort_m:
                self.abort(f"Altitude loss: {altitude_loss:.2f}m")
                return True

        return False

    def _update_spinup(self, dt: float, altitude_m: float,
                       climb_rate_ms: float) -> float:
        """
        Phase 1: Motor spinup

        Gradually ramp throttle from 0 to initial_throttle over spinup_time.
        This allows motors to spin up smoothly before attempting liftoff.
        """
        phase_time = time.time() - self.state.phase_start_time
        spinup_time_sec = self.config.spinup_time_ms / 1000.0

        # Linear ramp from 0 to initial_throttle over spinup time
        spinup_progress = min(1.0, phase_time / spinup_time_sec)
        self.state.current_throttle = self.config.initial_throttle * spinup_progress

        # Transition to throttle ramp after spinup complete
        if spinup_progress >= 1.0:
            logger.info("Spinup complete, starting throttle ramp")
            self._transition_to(TakeoffPhase.THROTTLE_RAMP)

        return self.state.current_throttle

    def _update_throttle_ramp(self, dt: float, altitude_m: float,
                              climb_rate_ms: float) -> float:
        """
        Phase 2: Throttle ramping with liftoff detection

        Smoothly increase throttle while monitoring for liftoff.
        Uses jerk limiting for smooth throttle changes.
        """
        # Check for liftoff timeout
        total_time_ms = (time.time() - self.state.phase_start_time) * 1000
        total_time_ms += self.config.spinup_time_ms  # Include spinup time

        if total_time_ms > self.config.liftoff_timeout_ms:
            self.abort("Liftoff timeout - motors may be underpowered")
            return self.state.current_throttle

        # Jerk-limited throttle ramp
        # First, smoothly ramp up the throttle rate (jerk limiting)
        target_rate = self.config.ramp_rate_per_sec
        rate_error = target_rate - self._throttle_rate

        # Limit jerk (rate of change of rate)
        max_rate_change = self._max_throttle_jerk * dt
        rate_change = max(-max_rate_change, min(max_rate_change, rate_error))
        self._throttle_rate += rate_change

        # Apply rate to throttle
        self.state.current_throttle += self._throttle_rate * dt
        self.state.current_throttle = min(self.state.current_throttle,
                                          self.config.max_ramp_throttle)

        # Ground effect compensation
        height_agl = altitude_m - self.state.ground_altitude_m
        throttle_with_ge = self._apply_ground_effect(
            self.state.current_throttle, height_agl
        )

        # Check for liftoff
        if self._detect_liftoff(altitude_m, climb_rate_ms):
            logger.info(f"Liftoff detected at throttle={self.state.current_throttle:.2f}")
            self.state.liftoff_detected = True
            self.state.liftoff_time = time.time()
            self.state.throttle_at_liftoff = self.state.current_throttle
            self._transition_to(TakeoffPhase.LIFTOFF_DETECT)

        return throttle_with_ge

    def _update_liftoff_detect(self, dt: float, altitude_m: float,
                               climb_rate_ms: float) -> float:
        """
        Phase 3: Confirm liftoff is sustained

        Brief confirmation period to ensure liftoff is real and sustained,
        not just a sensor glitch or bounce.
        """
        phase_time = time.time() - self.state.phase_start_time

        # Confirm liftoff is sustained for a short period
        confirmation_time_ms = 200
        if phase_time * 1000 >= confirmation_time_ms:
            # Verify still climbing
            if climb_rate_ms > 0:
                logger.info("Liftoff confirmed, transitioning to climb")
                self._transition_to(TakeoffPhase.CLIMB)
            else:
                # Lost lift, back to ramp
                logger.warning("Lost lift during confirmation, resuming ramp")
                self.state.liftoff_detected = False
                self._transition_to(TakeoffPhase.THROTTLE_RAMP)

        # Maintain current throttle during confirmation
        height_agl = altitude_m - self.state.ground_altitude_m
        return self._apply_ground_effect(self.state.current_throttle, height_agl)

    def _update_climb(self, dt: float, altitude_m: float,
                      climb_rate_ms: float) -> float:
        """
        Phase 4: Climb to target altitude using altitude controller

        Hand control to the altitude controller PID for smooth climb
        to target altitude.
        """
        # Set target for altitude controller
        self.alt_controller.set_target_altitude(self.state.target_altitude_m)

        # Get throttle from altitude controller
        throttle = self.alt_controller.update(dt)

        # Ground effect compensation (fades out as we climb)
        height_agl = altitude_m - self.state.ground_altitude_m
        throttle = self._apply_ground_effect(throttle, height_agl)

        self.state.current_throttle = throttle

        # Check if reached target altitude
        altitude_error = self.state.target_altitude_m - altitude_m
        if altitude_error < 0.5 and abs(climb_rate_ms) < 0.5:
            logger.info(f"Reached target altitude {altitude_m:.1f}m")
            self._transition_to(TakeoffPhase.STABILIZE)

        return throttle

    def _update_stabilize(self, dt: float, altitude_m: float,
                          climb_rate_ms: float) -> float:
        """
        Phase 5: Stabilize at target altitude

        Brief period to stabilize before handing off to position control.
        This helps ensure smooth transition and stable hover.
        """
        phase_time = time.time() - self.state.phase_start_time

        # Continue altitude hold
        throttle = self.alt_controller.update(dt)
        self.state.current_throttle = throttle

        # Complete after stabilization time
        if phase_time * 1000 >= self.config.stabilize_time_ms:
            logger.info("Takeoff complete, ready for position hold")
            self.state.phase = TakeoffPhase.COMPLETE

            if self._on_complete:
                self._on_complete()

        return throttle

    def _detect_liftoff(self, altitude_m: float, climb_rate_ms: float) -> bool:
        """
        Detect liftoff using altitude increase + positive climb rate

        Based on iNav's isMulticopterFlying() logic.
        Both conditions must be met to confirm liftoff.

        Args:
            altitude_m: Current altitude
            climb_rate_ms: Current climb rate

        Returns:
            True if liftoff detected
        """
        height_agl = altitude_m - self.state.ground_altitude_m

        return (height_agl >= self.config.liftoff_altitude_threshold_m and
                climb_rate_ms >= self.config.liftoff_climb_rate_threshold_ms)

    def _apply_ground_effect(self, throttle: float, height_agl_m: float) -> float:
        """
        Apply ground effect compensation

        Ground effect increases lift near the ground due to air pressure
        buildup. As the drone climbs out of ground effect, it needs
        more throttle to maintain the same lift.

        Args:
            throttle: Base throttle value
            height_agl_m: Height above ground level in meters

        Returns:
            Throttle with ground effect compensation
        """
        if height_agl_m >= self.config.ground_effect_height_m:
            return throttle

        if height_agl_m < 0:
            height_agl_m = 0

        # Linear interpolation of boost
        # Full boost at ground, no boost at ground_effect_height
        ge_factor = 1.0 - (height_agl_m / self.config.ground_effect_height_m)
        boost = self.config.ground_effect_throttle_boost * ge_factor

        return throttle + boost

    def _transition_to(self, new_phase: TakeoffPhase):
        """Transition to a new takeoff phase"""
        logger.debug(f"Takeoff phase: {self.state.phase.name} -> {new_phase.name}")
        self.state.phase = new_phase
        self.state.phase_start_time = time.time()

    def on_complete(self, callback: Callable[[], None]):
        """Register completion callback"""
        self._on_complete = callback

    def on_abort(self, callback: Callable[[str], None]):
        """Register abort callback"""
        self._on_abort = callback

    @property
    def is_active(self) -> bool:
        """Check if takeoff is in progress"""
        return self.state.phase not in (TakeoffPhase.IDLE, TakeoffPhase.COMPLETE,
                                         TakeoffPhase.ABORTED)

    @property
    def phase(self) -> TakeoffPhase:
        """Current takeoff phase"""
        return self.state.phase

    def get_status(self) -> dict:
        """Get takeoff status for telemetry"""
        return {
            'phase': self.state.phase.name,
            'target_altitude': self.state.target_altitude_m,
            'current_throttle': self.state.current_throttle,
            'liftoff_detected': self.state.liftoff_detected,
            'throttle_at_liftoff': self.state.throttle_at_liftoff,
            'ground_altitude': self.state.ground_altitude_m,
            'abort_reason': self.state.abort_reason,
        }
