"""
Takeoff Controller - iNav-style velocity PID takeoff

Implements iNav's takeoff method:
- Throttle = hover_throttle + velocity_PID_correction
- Liftoff detection: throttle > hover AND gyro > 7°/s
- Simple 3-state machine: SPINUP -> CLIMBING -> COMPLETE

Uses same velocity Z PID gains as iNav (navigation.c:4939-4944):
- P = 100/66.7 = 1.5
- I = 50/20 = 2.5
- D = 10/100 = 0.1

No frequency adaptation needed because PID uses dt correctly.
"""

from enum import Enum, auto
from dataclasses import dataclass
from typing import Optional, Callable
import time
import logging
import math

from ..config import TakeoffConfig

logger = logging.getLogger(__name__)

# Betaflight gyro scale: +-2000 dps = 16.4 LSB/dps
GYRO_SCALE_DPS = 16.4


class TakeoffState(Enum):
    """
    Takeoff states (simplified from iNav)

    3 states vs old 6-phase system:
    - IDLE: Not in takeoff
    - SPINUP: Motor spinup (500ms)
    - CLIMBING: Velocity PID active, climbing to target
    - COMPLETE: Takeoff successful
    - ABORTED: Takeoff failed
    """
    IDLE = auto()
    SPINUP = auto()
    CLIMBING = auto()
    COMPLETE = auto()
    ABORTED = auto()


@dataclass
class TakeoffInternalState:
    """Internal state for takeoff sequence"""
    state: TakeoffState = TakeoffState.IDLE
    state_start_time: float = 0.0

    # Target
    target_altitude_m: float = 3.0
    ground_altitude_m: float = 0.0

    # Throttle
    current_throttle: float = 0.0
    hover_throttle: float = 0.5

    # PID state
    vel_integral: float = 0.0
    vel_error_prev: float = 0.0
    filtered_climb_rate: float = 0.0

    # Liftoff detection
    liftoff_detected: bool = False
    liftoff_start_time: float = 0.0  # When liftoff conditions first met

    # Abort tracking
    abort_reason: str = ""
    highest_altitude_m: float = 0.0


class TakeoffController:
    """
    iNav-style takeoff controller

    Key differences from old 6-phase system:
    1. Uses velocity PID instead of open-loop throttle ramp
    2. Detects liftoff via gyro activity (drone movement)
    3. Adapts to drone weight (PID compensates)
    4. Simpler state machine (3 states vs 6 phases)

    Formula (from iNav):
        throttle = hover_throttle + velocity_PID(target_climb_rate - actual_climb_rate)
    """

    def __init__(self, config: TakeoffConfig, hover_throttle: float = 0.5):
        """
        Initialize takeoff controller

        Args:
            config: TakeoffConfig with PID gains and thresholds
            hover_throttle: Initial hover throttle estimate
        """
        self.config = config
        self._state = TakeoffInternalState()
        self._state.hover_throttle = hover_throttle

        # Callbacks
        self._on_complete: Optional[Callable[[], None]] = None
        self._on_abort: Optional[Callable[[str], None]] = None

        # Filter coefficient for climb rate (RC low-pass)
        # alpha = dt / (RC + dt), where RC = 1 / (2*pi*fc)
        self._filter_rc = 1.0 / (2.0 * math.pi * config.velocity_filter_hz)

    def start(self, target_altitude_m: float, ground_altitude_m: float = 0.0):
        """
        Begin takeoff sequence

        Args:
            target_altitude_m: Target altitude in meters
            ground_altitude_m: Current (ground) altitude reference
        """
        logger.info(f"iNav-style takeoff: target={target_altitude_m}m, hover_thr={self._state.hover_throttle:.2f}")

        self._state = TakeoffInternalState(
            state=TakeoffState.SPINUP,
            state_start_time=time.time(),
            target_altitude_m=target_altitude_m,
            ground_altitude_m=ground_altitude_m,
            hover_throttle=self._state.hover_throttle,
            highest_altitude_m=ground_altitude_m
        )

    def abort(self, reason: str):
        """Abort takeoff sequence"""
        logger.warning(f"Takeoff ABORTED: {reason}")
        self._state.state = TakeoffState.ABORTED
        self._state.abort_reason = reason

        if self._on_abort:
            self._on_abort(reason)

    def update(self, dt: float, altitude_m: float, climb_rate_ms: float,
               gyro_x: int, gyro_y: int, gyro_z: int,
               roll_deg: float, pitch_deg: float) -> float:
        """
        Update takeoff controller

        Args:
            dt: Time step in seconds
            altitude_m: Current altitude in meters
            climb_rate_ms: Current climb rate in m/s (positive = up)
            gyro_x, gyro_y, gyro_z: Raw gyro values from MSP_RAW_IMU
            roll_deg, pitch_deg: Current attitude in degrees

        Returns:
            Throttle command (0.0 - 1.0)
        """
        # Terminal states return no output
        if self._state.state in (TakeoffState.IDLE, TakeoffState.COMPLETE,
                                  TakeoffState.ABORTED):
            return 0.0

        # Check abort conditions
        if self._check_abort_conditions(altitude_m, roll_deg, pitch_deg):
            return self.config.min_throttle

        # Update filtered climb rate (RC low-pass filter)
        alpha = dt / (self._filter_rc + dt)
        self._state.filtered_climb_rate += alpha * (climb_rate_ms - self._state.filtered_climb_rate)

        # State machine
        if self._state.state == TakeoffState.SPINUP:
            return self._update_spinup(dt)

        elif self._state.state == TakeoffState.CLIMBING:
            return self._update_climbing(dt, altitude_m, gyro_x, gyro_y, gyro_z)

        return self._state.current_throttle

    def _update_spinup(self, dt: float) -> float:
        """
        Phase 1: Motor spinup (500ms)

        Linear ramp from 0 to spinup_throttle.
        """
        phase_time = time.time() - self._state.state_start_time
        spinup_time_s = self.config.spinup_time_ms / 1000.0

        # Linear ramp
        progress = min(1.0, phase_time / spinup_time_s)
        self._state.current_throttle = self.config.spinup_throttle * progress

        # Transition to climbing after spinup
        if progress >= 1.0:
            logger.info("Spinup complete, starting velocity PID climb")
            self._transition_to(TakeoffState.CLIMBING)

        return self._state.current_throttle

    def _update_climbing(self, dt: float, altitude_m: float,
                         gyro_x: int, gyro_y: int, gyro_z: int) -> float:
        """
        Phase 2: Velocity PID climbing (iNav-style)

        throttle = hover_throttle + PID(target_climb_rate - actual_climb_rate)

        Liftoff detected when:
        - throttle > hover_throttle + margin
        - gyro magnitude > 7 deg/s
        """
        config = self.config

        # Velocity PID
        target_rate = config.target_climb_rate_ms
        actual_rate = self._state.filtered_climb_rate
        error = target_rate - actual_rate

        # P term
        p_term = config.vel_kp * error

        # I term with anti-windup
        self._state.vel_integral += error * dt
        self._state.vel_integral = max(-config.vel_i_max,
                                        min(config.vel_i_max, self._state.vel_integral))
        i_term = config.vel_ki * self._state.vel_integral

        # D term (derivative of error)
        d_error = (error - self._state.vel_error_prev) / dt if dt > 0 else 0.0
        d_term = config.vel_kd * d_error
        self._state.vel_error_prev = error

        # PID output
        pid_correction = p_term + i_term + d_term

        # iNav formula: throttle = hover + PID
        throttle = self._state.hover_throttle + pid_correction

        # Clamp throttle
        throttle = max(config.min_throttle, min(config.max_throttle, throttle))
        self._state.current_throttle = throttle

        # Track highest altitude
        self._state.highest_altitude_m = max(self._state.highest_altitude_m, altitude_m)

        # iNav-style liftoff detection
        if not self._state.liftoff_detected:
            self._check_liftoff_inav(throttle, gyro_x, gyro_y, gyro_z)

        # Check if reached target altitude
        height_agl = altitude_m - self._state.ground_altitude_m
        altitude_error = self._state.target_altitude_m - altitude_m

        if height_agl >= self._state.target_altitude_m - config.altitude_tolerance_m:
            if abs(actual_rate) < 0.5:  # Stable
                logger.info(f"Takeoff complete at {altitude_m:.1f}m (target: {self._state.target_altitude_m}m)")
                self._state.state = TakeoffState.COMPLETE

                if self._on_complete:
                    self._on_complete()

        return throttle

    def _check_liftoff_inav(self, throttle: float, gyro_x: int, gyro_y: int, gyro_z: int):
        """
        iNav-style liftoff detection

        Both conditions must be true:
        1. throttle > hover_throttle + margin
        2. gyro magnitude > 7 deg/s (drone is actually moving)

        Must persist for liftoff_confirm_time_s (200ms)
        """
        config = self.config

        # Condition 1: Throttle above hover + margin
        throttle_ok = throttle > self._state.hover_throttle + config.liftoff_throttle_margin

        # Condition 2: Gyro activity (convert raw to deg/s)
        gyro_dps = math.sqrt(gyro_x**2 + gyro_y**2 + gyro_z**2) / GYRO_SCALE_DPS
        gyro_ok = gyro_dps > config.liftoff_gyro_threshold_dps

        if throttle_ok and gyro_ok:
            # Start or continue confirmation timer
            if self._state.liftoff_start_time == 0.0:
                self._state.liftoff_start_time = time.time()
                logger.debug(f"Liftoff conditions met: thr={throttle:.2f}, gyro={gyro_dps:.1f} dps")

            # Check confirmation time
            confirm_time = time.time() - self._state.liftoff_start_time
            if confirm_time >= config.liftoff_confirm_time_s:
                self._state.liftoff_detected = True
                logger.info(f"Liftoff CONFIRMED: throttle={throttle:.2f}, gyro={gyro_dps:.1f} dps")
        else:
            # Reset confirmation timer
            if self._state.liftoff_start_time > 0:
                logger.debug("Liftoff conditions lost, resetting timer")
            self._state.liftoff_start_time = 0.0

    def _check_abort_conditions(self, altitude_m: float,
                                 roll_deg: float, pitch_deg: float) -> bool:
        """Check for abort conditions"""
        config = self.config

        # Excessive tilt
        tilt = math.sqrt(roll_deg**2 + pitch_deg**2)
        if tilt > config.max_tilt_deg:
            self.abort(f"Excessive tilt: {tilt:.1f}° > {config.max_tilt_deg}°")
            return True

        # Timeout (no liftoff after timeout_s)
        total_time = time.time() - self._state.state_start_time
        if total_time > config.timeout_s and not self._state.liftoff_detected:
            self.abort(f"Liftoff timeout ({config.timeout_s}s) - check hover_throttle or props")
            return True

        return False

    def _transition_to(self, new_state: TakeoffState):
        """Transition to new state"""
        logger.debug(f"Takeoff: {self._state.state.name} -> {new_state.name}")
        self._state.state = new_state
        self._state.state_start_time = time.time()

    def set_hover_throttle(self, hover_throttle: float):
        """Update hover throttle estimate"""
        self._state.hover_throttle = max(0.2, min(0.8, hover_throttle))

    def on_complete(self, callback: Callable[[], None]):
        """Register completion callback"""
        self._on_complete = callback

    def on_abort(self, callback: Callable[[str], None]):
        """Register abort callback"""
        self._on_abort = callback

    @property
    def is_active(self) -> bool:
        """Check if takeoff is in progress"""
        return self._state.state in (TakeoffState.SPINUP, TakeoffState.CLIMBING)

    @property
    def state(self) -> TakeoffState:
        """Current takeoff state"""
        return self._state.state

    @property
    def liftoff_detected(self) -> bool:
        """Whether liftoff has been detected"""
        return self._state.liftoff_detected

    @property
    def throttle_at_liftoff(self) -> float:
        """Throttle value when liftoff was detected (for hover learning)"""
        return self._state.current_throttle if self._state.liftoff_detected else 0.0

    def get_status(self) -> dict:
        """Get takeoff status for telemetry"""
        return {
            'state': self._state.state.name,
            'target_altitude': self._state.target_altitude_m,
            'current_throttle': self._state.current_throttle,
            'hover_throttle': self._state.hover_throttle,
            'liftoff_detected': self._state.liftoff_detected,
            'filtered_climb_rate': self._state.filtered_climb_rate,
            'pid_integral': self._state.vel_integral,
            'ground_altitude': self._state.ground_altitude_m,
            'abort_reason': self._state.abort_reason,
        }
