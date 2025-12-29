"""
Altitude Controller

Controls vertical position and climb rate.
Outputs throttle commands.

Altitude Error → Climb Rate → Throttle
"""

import time
from dataclasses import dataclass
from typing import Optional

from .pid import PIDController, PIDGains, PT1Filter
from ..config import get_config


@dataclass
class AltitudeState:
    """Current altitude state"""
    altitude_m: float = 0.0      # meters above reference
    climb_rate_ms: float = 0.0   # m/s, positive = up
    timestamp: float = 0.0


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
        climb_gains = PIDGains(
            kp=nav.climb_rate_p_gain * 0.01,  # Scale for throttle output
            ki=0.02,
            kd=0.0,
            i_max=nav.throttle_margin
        )
        self.climb_pid = PIDController(climb_gains)
        self.climb_pid.set_output_limits(-nav.throttle_margin, nav.throttle_margin)

        # Limits
        self.max_climb_rate = nav.max_vertical_speed_ms
        self.hover_throttle = nav.hover_throttle
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

        # Landing detection
        self._landing_detected = False
        self._low_altitude_time = 0.0

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
        """Check for landing condition"""
        current_time = time.time()

        # Landing detection: low altitude + low climb rate
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
        self._last_altitude = 0.0
        self._last_update_time = 0.0
        self._landing_detected = False

    def calibrate_hover_throttle(self, throttle: float):
        """
        Set hover throttle (should be calibrated in flight)

        Args:
            throttle: Throttle value that maintains hover (0.0-1.0)
        """
        self.hover_throttle = max(0.2, min(0.8, throttle))

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
            'pid': self.climb_pid.get_components()
        }
