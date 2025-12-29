"""
PID Controller implementation

Inspired by iNav's fp_pid.c with:
- Anti-windup (clamping and back-calculation)
- D-term filtering
- Feed-forward support
- Setpoint weighting
"""

import time
from dataclasses import dataclass, field
from typing import Optional
import math


@dataclass
class PIDGains:
    """PID controller gains"""
    kp: float = 1.0          # Proportional gain
    ki: float = 0.0          # Integral gain
    kd: float = 0.0          # Derivative gain
    kff: float = 0.0         # Feed-forward gain

    # Anti-windup
    i_max: float = float('inf')   # Maximum integrator value

    # D-term filtering
    d_filter_hz: float = 0.0      # Low-pass filter cutoff (0 = disabled)

    # Setpoint weighting (0-1, reduces derivative kick on setpoint changes)
    setpoint_weight: float = 0.0


@dataclass
class PIDState:
    """Internal PID controller state"""
    integrator: float = 0.0
    last_measurement: float = 0.0
    last_error: float = 0.0
    last_derivative: float = 0.0
    last_setpoint: float = 0.0
    last_output: float = 0.0

    # Components for debugging
    p_term: float = 0.0
    i_term: float = 0.0
    d_term: float = 0.0
    ff_term: float = 0.0


class PIDController:
    """
    PID Controller with advanced features

    Features:
    - Anti-windup with clamping and back-calculation
    - D-term on measurement (avoids derivative kick)
    - Optional D-term filtering
    - Feed-forward term
    - Setpoint weighting for smoother response
    - Output limiting

    Based on iNav's navPidApply3() implementation.
    """

    def __init__(self, gains: Optional[PIDGains] = None):
        """
        Initialize PID controller

        Args:
            gains: PID gains, or None for default gains
        """
        self.gains = gains or PIDGains()
        self.state = PIDState()

        # Output limits
        self.output_min: float = float('-inf')
        self.output_max: float = float('inf')

        # D-term filter state
        self._d_filter_alpha: float = 1.0  # No filtering
        self._last_update_time: float = 0.0

    def set_output_limits(self, min_val: float, max_val: float):
        """Set output limits for anti-windup"""
        self.output_min = min_val
        self.output_max = max_val

    def set_gains(self, kp: float = None, ki: float = None,
                  kd: float = None, kff: float = None):
        """Update individual gains"""
        if kp is not None:
            self.gains.kp = kp
        if ki is not None:
            self.gains.ki = ki
        if kd is not None:
            self.gains.kd = kd
        if kff is not None:
            self.gains.kff = kff

    def reset(self):
        """Reset controller state"""
        self.state = PIDState()
        self._last_update_time = 0.0

    def update(self, setpoint: float, measurement: float,
               dt: Optional[float] = None) -> float:
        """
        Update PID controller

        Args:
            setpoint: Desired value
            measurement: Current measured value
            dt: Time step in seconds (None for automatic)

        Returns:
            Control output
        """
        # Calculate dt
        current_time = time.time()
        if dt is None:
            if self._last_update_time > 0:
                dt = current_time - self._last_update_time
            else:
                dt = 0.02  # Default 50Hz
        self._last_update_time = current_time

        # Clamp dt to reasonable range
        dt = max(0.001, min(dt, 0.5))

        # Calculate error
        error = setpoint - measurement

        # ==================== P-Term ====================
        p_term = self.gains.kp * error

        # ==================== I-Term ====================
        # Update integrator
        self.state.integrator += self.gains.ki * error * dt

        # Anti-windup: clamp integrator
        self.state.integrator = max(-self.gains.i_max,
                                    min(self.gains.i_max, self.state.integrator))

        i_term = self.state.integrator

        # ==================== D-Term ====================
        # D-term on measurement to avoid derivative kick
        # With optional setpoint weighting

        # Weighted error for D-term
        d_input = (self.gains.setpoint_weight * setpoint +
                  (1 - self.gains.setpoint_weight) * (-measurement))

        # Calculate derivative
        if dt > 0:
            derivative = (d_input - self.state.last_measurement) / dt
        else:
            derivative = 0.0

        # Optional low-pass filter on D-term
        if self.gains.d_filter_hz > 0:
            # First-order IIR filter
            rc = 1.0 / (2 * math.pi * self.gains.d_filter_hz)
            alpha = dt / (rc + dt)
            derivative = alpha * derivative + (1 - alpha) * self.state.last_derivative
            self.state.last_derivative = derivative

        d_term = self.gains.kd * derivative

        # Store for next iteration (use measurement for D on measurement)
        self.state.last_measurement = -measurement

        # ==================== Feed-Forward ====================
        # Feed-forward on setpoint change rate
        if dt > 0 and self.gains.kff != 0:
            setpoint_rate = (setpoint - self.state.last_setpoint) / dt
            ff_term = self.gains.kff * setpoint_rate
        else:
            ff_term = 0.0

        self.state.last_setpoint = setpoint

        # ==================== Output ====================
        output = p_term + i_term + d_term + ff_term

        # Output limiting
        output_unlimited = output
        output = max(self.output_min, min(self.output_max, output))

        # Back-calculation anti-windup
        # Reduce integrator if output is saturated
        if output != output_unlimited:
            # Calculate how much we're over the limit
            excess = output_unlimited - output

            # Only reduce integrator, don't reverse it
            if (excess > 0 and self.state.integrator > 0) or \
               (excess < 0 and self.state.integrator < 0):
                # Reduce integrator proportionally
                back_calc_gain = 1.0  # Tracking time constant
                self.state.integrator -= back_calc_gain * excess * dt

        # Store state for debugging
        self.state.p_term = p_term
        self.state.i_term = i_term
        self.state.d_term = d_term
        self.state.ff_term = ff_term
        self.state.last_error = error
        self.state.last_output = output

        return output

    def get_components(self) -> dict:
        """Get individual PID components for debugging"""
        return {
            'p': self.state.p_term,
            'i': self.state.i_term,
            'd': self.state.d_term,
            'ff': self.state.ff_term,
            'error': self.state.last_error,
            'integrator': self.state.integrator,
            'output': self.state.last_output
        }


class PT1Filter:
    """
    First-order low-pass filter (PT1)

    Commonly used for smoothing signals in flight controllers.
    """

    def __init__(self, cutoff_hz: float = 10.0):
        """
        Initialize filter

        Args:
            cutoff_hz: Cutoff frequency in Hz
        """
        self.cutoff_hz = cutoff_hz
        self._state: float = 0.0
        self._initialized: bool = False

    def apply(self, input_val: float, dt: float) -> float:
        """
        Apply filter

        Args:
            input_val: Input value
            dt: Time step in seconds

        Returns:
            Filtered value
        """
        if not self._initialized:
            self._state = input_val
            self._initialized = True
            return input_val

        if self.cutoff_hz <= 0:
            return input_val

        # Calculate filter coefficient
        rc = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        alpha = dt / (rc + dt)

        # Apply filter
        self._state = alpha * input_val + (1.0 - alpha) * self._state

        return self._state

    def reset(self, value: float = 0.0):
        """Reset filter state"""
        self._state = value
        self._initialized = value != 0.0


class RateLimiter:
    """
    Rate limiter for smooth transitions

    Limits the rate of change of a value.
    """

    def __init__(self, max_rate: float):
        """
        Initialize rate limiter

        Args:
            max_rate: Maximum rate of change per second
        """
        self.max_rate = max_rate
        self._last_value: float = 0.0
        self._initialized: bool = False

    def apply(self, target: float, dt: float) -> float:
        """
        Apply rate limiting

        Args:
            target: Target value
            dt: Time step in seconds

        Returns:
            Rate-limited value
        """
        if not self._initialized:
            self._last_value = target
            self._initialized = True
            return target

        max_change = self.max_rate * dt
        delta = target - self._last_value

        # Limit the change
        if abs(delta) > max_change:
            delta = max_change if delta > 0 else -max_change

        self._last_value += delta
        return self._last_value

    def reset(self, value: float = 0.0):
        """Reset to specific value"""
        self._last_value = value
        self._initialized = True
