"""
Altitude Controller

Controls vertical position and climb rate.
Outputs throttle commands.

Altitude Error → Climb Rate → Throttle

Landing logic inspired by iNav:
- 3-phase descent with variable speeds
- Accelerometer-based touchdown detection
- Position hold during landing
"""

import time
import math
from dataclasses import dataclass
from typing import Optional
from enum import Enum, auto

from .pid import PIDController, PIDGains, PT1Filter
from ..config import get_config


class LandingPhase(Enum):
    """Landing phases (iNav-style)"""
    NOT_LANDING = auto()
    PHASE_HIGH = auto()      # > 5m AGL: fast descent
    PHASE_MID = auto()       # 1-5m AGL: medium descent
    PHASE_FINAL = auto()     # < 1m AGL: slow descent + touchdown detection
    TOUCHDOWN = auto()       # Ground contact detected


@dataclass
class AltitudeState:
    """Current altitude state"""
    altitude_m: float = 0.0      # meters above reference
    climb_rate_ms: float = 0.0   # m/s, positive = up
    timestamp: float = 0.0


@dataclass
class LandingConfig:
    """Landing configuration (iNav-style parameters)"""
    # Descent speeds per phase (m/s, positive = down)
    speed_high: float = 1.5      # Phase haute (> 5m)
    speed_mid: float = 0.7       # Phase moyenne (1-5m)
    speed_final: float = 0.3     # Phase finale (< 1m)

    # Altitude thresholds (meters AGL)
    threshold_high: float = 5.0  # Switch to mid phase
    threshold_mid: float = 1.0   # Switch to final phase

    # Touchdown detection
    acc_threshold_g: float = 1.5  # Acceleration spike for touchdown (in G)
    acc_filter_hz: float = 5.0    # Low-pass filter for accelerometer
    touchdown_alt_max: float = 0.5  # Max altitude for touchdown detection
    touchdown_confirm_time: float = 0.5  # Seconds to confirm touchdown

    # Fallback detection (altitude + vario)
    fallback_alt_threshold: float = 0.3  # meters
    fallback_vario_threshold: float = 0.2  # m/s
    fallback_confirm_time: float = 2.0  # seconds

    # Auto-disarm
    auto_disarm: bool = True
    disarm_delay: float = 2.0  # seconds after touchdown


class AltitudeController:
    """
    Altitude controller with adaptive source selection

    Uses cascaded control:
    1. Altitude error → Climb rate target (P)
    2. Climb rate error → Throttle adjustment (PID)

    Supports multiple altitude sources:
    - Barometer from Betaflight (MSP_ALTITUDE)
    - Barometer on Pi (BMP280/MS5611)
    - GPS altitude (fallback)
    """

    def __init__(self):
        """Initialize altitude controller"""
        config = get_config()
        nav = config.navigation
        alt_config = config.altitude

        # Altitude to climb rate (P control)
        self.alt_p_gain = nav.alt_p_gain

        # Climb rate to throttle (PID)
        # Gains harmonized with takeoff controller for 50Hz MSP
        climb_gains = PIDGains(
            kp=0.15,   # Same as takeoff
            ki=0.05,   # Same as takeoff
            kd=0.02,   # Same as takeoff
            i_max=0.2  # Anti-windup limit
        )
        self.climb_pid = PIDController(climb_gains)
        self.climb_pid.set_output_limits(-nav.throttle_margin, nav.throttle_margin)

        # Limits
        self.max_climb_rate = nav.max_vertical_speed_ms
        self._hover_throttle = nav.hover_throttle  # Use private var to avoid property setter
        self.throttle_margin = nav.throttle_margin

        # Filtering
        self.altitude_filter = PT1Filter(cutoff_hz=alt_config.altitude_filter_hz)
        self.climb_rate_filter = PT1Filter(cutoff_hz=2.0)

        # State
        self.target_altitude = 0.0
        self.target_climb_rate = 0.0

        self._current_altitude = 0.0
        self._current_climb_rate = 0.0
        self._last_altitude = 0.0
        self._last_update_time = 0.0

        # Altitude source
        self._use_baro_fc = alt_config.use_baro_from_fc
        self._use_baro_pi = alt_config.use_baro_on_pi
        self._baro_weight = alt_config.baro_weight

        # Landing detection (legacy)
        self._landing_detected = False
        self._low_altitude_time = 0.0

        # Ground reference for takeoff
        self._ground_reference_alt = 0.0

        # iNav-style landing
        self._landing_config = LandingConfig()
        self._landing_phase = LandingPhase.NOT_LANDING
        self._landing_active = False

        # Accelerometer for touchdown detection
        self._acc_z_filter = PT1Filter(cutoff_hz=self._landing_config.acc_filter_hz)
        self._acc_z_filtered = 1.0  # In G (1G = gravity)
        self._acc_z_baseline = 1.0  # Baseline during descent
        self._acc_spike_time = 0.0  # Time when spike detected

        # Touchdown confirmation
        self._touchdown_detected = False
        self._touchdown_time = 0.0
        self._disarm_requested = False

    @property
    def hover_throttle(self) -> float:
        """Get current hover throttle value"""
        return self._hover_throttle

    @hover_throttle.setter
    def hover_throttle(self, value: float):
        """
        Set hover throttle (used by hover throttle learner)

        Args:
            value: Throttle value that maintains hover (0.0-1.0)
        """
        self._hover_throttle = max(0.2, min(0.8, value))

    def set_ground_reference(self, altitude_m: float):
        """
        Set ground reference altitude for takeoff

        Args:
            altitude_m: Ground altitude in meters
        """
        self._ground_reference_alt = altitude_m

    def get_height_agl(self) -> float:
        """
        Get height above ground reference

        Returns:
            Height above ground in meters
        """
        return self._current_altitude - self._ground_reference_alt

    def set_target_altitude(self, altitude_m: float):
        """Set target altitude in meters"""
        self.target_altitude = altitude_m

    def set_target_climb_rate(self, climb_rate_ms: float):
        """Set target climb rate (overrides altitude control)"""
        self.target_climb_rate = max(-self.max_climb_rate,
                                     min(self.max_climb_rate, climb_rate_ms))

    def update_altitude_from_fc(self, altitude_cm: int, vario_cms: int):
        """
        Update altitude from Betaflight MSP_ALTITUDE

        Args:
            altitude_cm: Altitude in centimeters
            vario_cms: Vertical speed in cm/s
        """
        altitude_m = altitude_cm / 100.0
        climb_rate_ms = vario_cms / 100.0

        self._update_altitude(altitude_m, climb_rate_ms)

    def update_altitude_from_gps(self, altitude_m: float, vel_down_ms: float):
        """
        Update altitude from GPS

        Args:
            altitude_m: GPS altitude in meters
            vel_down_ms: Vertical velocity (down positive) in m/s
        """
        # If we have baro from FC, fuse with GPS
        if self._use_baro_fc and self._current_altitude != 0:
            # Simple complementary filter
            altitude_m = (self._baro_weight * self._current_altitude +
                         (1 - self._baro_weight) * altitude_m)

        # Vertical velocity: GPS gives vel_down (positive = descending)
        climb_rate_ms = -vel_down_ms

        self._update_altitude(altitude_m, climb_rate_ms)

    def _update_altitude(self, altitude_m: float, climb_rate_ms: float):
        """Internal altitude update with filtering"""
        current_time = time.time()
        dt = current_time - self._last_update_time if self._last_update_time > 0 else 0.02
        self._last_update_time = current_time

        # Filter altitude
        altitude_m = self.altitude_filter.apply(altitude_m, dt)

        # Calculate climb rate from altitude if not provided
        if climb_rate_ms == 0 and self._last_altitude != 0:
            climb_rate_ms = (altitude_m - self._last_altitude) / dt

        # Filter climb rate
        climb_rate_ms = self.climb_rate_filter.apply(climb_rate_ms, dt)

        self._last_altitude = altitude_m
        self._current_altitude = altitude_m
        self._current_climb_rate = climb_rate_ms

        # Landing detection
        self._check_landing()

    def update(self, dt: float = None) -> float:
        """
        Calculate throttle output

        Args:
            dt: Time step in seconds (None for automatic)

        Returns:
            Throttle value 0.0 to 1.0
        """
        if dt is None:
            dt = 0.02

        # Altitude to climb rate (P control)
        altitude_error = self.target_altitude - self._current_altitude
        climb_rate_target = altitude_error * self.alt_p_gain

        # Limit climb rate
        climb_rate_target = max(-self.max_climb_rate,
                               min(self.max_climb_rate, climb_rate_target))

        # Allow direct climb rate control to override
        if self.target_climb_rate != 0:
            climb_rate_target = self.target_climb_rate

        # Climb rate to throttle (PID)
        throttle_adjust = self.climb_pid.update(
            climb_rate_target,
            self._current_climb_rate,
            dt
        )

        # Add to hover throttle
        throttle = self.hover_throttle + throttle_adjust

        # Clamp to safe range
        throttle = max(0.1, min(0.9, throttle))

        return throttle

    def _check_landing(self):
        """Check for landing condition (legacy method, kept for compatibility)"""
        if self._landing_active:
            # Use new iNav-style detection
            self._landing_detected = self._touchdown_detected
        else:
            # Legacy detection for non-landing states
            current_time = time.time()
            if (self._current_altitude < 0.5 and
                abs(self._current_climb_rate) < 0.3):
                if self._low_altitude_time == 0:
                    self._low_altitude_time = current_time
                elif current_time - self._low_altitude_time > 2.0:
                    self._landing_detected = True
            else:
                self._low_altitude_time = 0
                self._landing_detected = False

    @property
    def is_landed(self) -> bool:
        return self._landing_detected

    # ==================== iNav-style Landing ====================

    def start_landing(self):
        """Start landing sequence (call when entering LANDING state)"""
        self._landing_active = True
        self._landing_phase = LandingPhase.PHASE_HIGH
        self._touchdown_detected = False
        self._disarm_requested = False
        self._acc_spike_time = 0.0
        self._touchdown_time = 0.0
        self._low_altitude_time = 0.0

        # Set baseline accelerometer value
        self._acc_z_baseline = self._acc_z_filtered

    def abort_landing(self):
        """Abort landing sequence"""
        self._landing_active = False
        self._landing_phase = LandingPhase.NOT_LANDING
        self._touchdown_detected = False
        self._disarm_requested = False

    def update_accelerometer(self, acc_x: int, acc_y: int, acc_z: int, dt: float = 0.02):
        """
        Update accelerometer data for touchdown detection

        Args:
            acc_x, acc_y, acc_z: Raw accelerometer values from MSP_RAW_IMU
            dt: Time step in seconds

        Note: Betaflight reports accelerometer in units that depend on acc range.
              DAKEFPVH743 uses ±16G range: 2048 LSB/G.
        """
        # Convert raw to G
        # Scale depends on accelerometer range config in Betaflight:
        # ±2G: 16384, ±4G: 8192, ±8G: 4096, ±16G: 2048
        # DAKEFPVH743 uses ±16G range by default
        ACC_SCALE = 2048.0  # LSB per G

        # Calculate magnitude (we care about total acceleration, not just Z)
        acc_magnitude = math.sqrt(acc_x**2 + acc_y**2 + acc_z**2) / ACC_SCALE

        # Filter the magnitude
        self._acc_z_filtered = self._acc_z_filter.apply(acc_magnitude, dt)

    def get_landing_descent_rate(self) -> float:
        """
        Get target descent rate based on current landing phase

        Returns:
            Descent rate in m/s (negative = descending)
        """
        if not self._landing_active:
            return 0.0

        height_agl = self.get_height_agl()
        config = self._landing_config

        # Update landing phase based on altitude
        if height_agl > config.threshold_high:
            self._landing_phase = LandingPhase.PHASE_HIGH
            return -config.speed_high
        elif height_agl > config.threshold_mid:
            self._landing_phase = LandingPhase.PHASE_MID
            return -config.speed_mid
        else:
            self._landing_phase = LandingPhase.PHASE_FINAL
            return -config.speed_final

    def check_touchdown(self) -> bool:
        """
        Check for touchdown using accelerometer (iNav-style)

        Returns:
            True if touchdown confirmed
        """
        if not self._landing_active:
            return False

        current_time = time.time()
        height_agl = self.get_height_agl()
        config = self._landing_config

        # Only check touchdown in final phase and below max altitude
        if height_agl > config.touchdown_alt_max:
            self._acc_spike_time = 0.0
            return False

        # Method 1: Accelerometer spike detection (primary)
        # When drone touches ground, there's an acceleration spike
        acc_delta = abs(self._acc_z_filtered - self._acc_z_baseline)

        if acc_delta > (config.acc_threshold_g - 1.0):  # Threshold above baseline
            if self._acc_spike_time == 0.0:
                self._acc_spike_time = current_time
            elif current_time - self._acc_spike_time > config.touchdown_confirm_time:
                self._touchdown_detected = True
                self._touchdown_time = current_time
                self._landing_phase = LandingPhase.TOUCHDOWN
                return True
        else:
            # Reset if no spike
            self._acc_spike_time = 0.0

        # Method 2: Fallback - altitude + vario (if accelerometer fails)
        if (height_agl < config.fallback_alt_threshold and
            abs(self._current_climb_rate) < config.fallback_vario_threshold):

            if self._low_altitude_time == 0.0:
                self._low_altitude_time = current_time
            elif current_time - self._low_altitude_time > config.fallback_confirm_time:
                self._touchdown_detected = True
                self._touchdown_time = current_time
                self._landing_phase = LandingPhase.TOUCHDOWN
                return True
        else:
            self._low_altitude_time = 0.0

        return False

    def should_disarm(self) -> bool:
        """
        Check if auto-disarm should be triggered

        Returns:
            True if drone should disarm
        """
        if not self._landing_config.auto_disarm:
            return False

        if not self._touchdown_detected:
            return False

        current_time = time.time()
        time_since_touchdown = current_time - self._touchdown_time

        if time_since_touchdown >= self._landing_config.disarm_delay:
            self._disarm_requested = True
            return True

        return False

    @property
    def landing_phase(self) -> LandingPhase:
        """Current landing phase"""
        return self._landing_phase

    @property
    def is_landing(self) -> bool:
        """Check if landing is active"""
        return self._landing_active

    @property
    def touchdown_confirmed(self) -> bool:
        """Check if touchdown has been confirmed"""
        return self._touchdown_detected

    @property
    def current_altitude(self) -> float:
        return self._current_altitude

    @property
    def current_climb_rate(self) -> float:
        return self._current_climb_rate

    def reset(self):
        """Reset controller state"""
        self.climb_pid.reset()
        self.altitude_filter.reset()
        self.climb_rate_filter.reset()
        self._acc_z_filter.reset()
        self._last_altitude = 0.0
        self._last_update_time = 0.0
        self._landing_detected = False
        self.abort_landing()

    def calibrate_hover_throttle(self, throttle: float):
        """
        Set hover throttle (should be calibrated in flight)

        Args:
            throttle: Throttle value that maintains hover (0.0-1.0)
        """
        self.hover_throttle = throttle  # Property setter handles clamping

    def get_debug_info(self) -> dict:
        """Get debug information"""
        return {
            'target_alt': self.target_altitude,
            'current_alt': self._current_altitude,
            'altitude_error': self.target_altitude - self._current_altitude,
            'target_climb': self.target_climb_rate,
            'current_climb': self._current_climb_rate,
            'hover_throttle': self.hover_throttle,
            'is_landed': self._landing_detected,
            'pid': self.climb_pid.get_components(),
            # Landing info
            'landing_active': self._landing_active,
            'landing_phase': self._landing_phase.name if self._landing_active else 'N/A',
            'height_agl': self.get_height_agl(),
            'acc_filtered': self._acc_z_filtered,
            'touchdown': self._touchdown_detected,
        }
