"""
Velocity Controller

Converts velocity error to acceleration/angle commands.
Second stage of the cascaded control loop.

Velocity Error → Acceleration Target → Roll/Pitch Angles
"""

import math
import time
from dataclasses import dataclass
from typing import Tuple

from .pid import PIDController, PIDGains, RateLimiter
from .position_controller import Vector3
from ..config import get_config


# Physical constants
GRAVITY_MSS = 9.80665  # m/s^2


@dataclass
class AttitudeCommand:
    """Output attitude command"""
    roll_deg: float = 0.0    # Bank angle, positive = right
    pitch_deg: float = 0.0   # Pitch angle, positive = nose up
    yaw_rate_dps: float = 0.0  # Yaw rate, positive = clockwise


class VelocityController:
    """
    Velocity to acceleration/attitude controller

    Uses PID control to convert velocity error to acceleration targets,
    then converts acceleration to roll/pitch angles for the flight controller.

    Implements jerk limiting for smooth control.

    Inspired by iNav's updatePositionAccelController_MC()
    """

    def __init__(self):
        """Initialize velocity controller"""
        config = get_config()
        nav = config.navigation

        # PID controllers for X (North) and Y (East)
        # iNav nav_mc_vel_xy_*: P=40/20=2.0, I=15/100=0.15, D=100/100=1.0
        # D-term filter: nav_mc_vel_xy_dterm_lpf_hz default = 2 Hz
        gains_xy = PIDGains(
            kp=nav.vel_p_gain,      # iNav: 2.0
            ki=nav.vel_i_gain,      # iNav: 0.15
            kd=nav.vel_d_gain,      # iNav: 1.0
            i_max=nav.vel_i_max,
            d_filter_hz=2.0         # iNav: nav_mc_vel_xy_dterm_lpf_hz = 2 Hz
        )

        self.pid_x = PIDController(gains_xy)
        self.pid_y = PIDController(gains_xy)

        # Acceleration limits - iNav: NAV_ACCELERATION_XY_MAX = 980 cm/s² ≈ 9.8 m/s²
        self.max_accel = nav.max_horizontal_accel_mss

        self.pid_x.set_output_limits(-self.max_accel, self.max_accel)
        self.pid_y.set_output_limits(-self.max_accel, self.max_accel)

        # Jerk limiting (rate of acceleration change)
        # iNav: MC_POS_CONTROL_JERK_LIMIT_CMSSS = 1700 cm/s³ = 17 m/s³
        self.jerk_limit = nav.jerk_limit_msss
        self.last_accel_x = 0.0
        self.last_accel_y = 0.0

        # Attitude limits
        self.max_bank_angle = nav.max_bank_angle_deg

        # Current state
        self.velocity_target = Vector3()
        self.current_velocity = Vector3()
        self.current_yaw_rad = 0.0

        # Timing
        self._last_update_time = 0.0

    def set_target_velocity(self, vel_north: float, vel_east: float, vel_down: float = 0.0):
        """Set target velocity in NED frame (m/s)"""
        self.velocity_target = Vector3(vel_north, vel_east, vel_down)

    def set_current_velocity(self, vel_north: float, vel_east: float, vel_down: float):
        """Update current velocity from GPS"""
        self.current_velocity = Vector3(vel_north, vel_east, vel_down)

    def set_current_yaw(self, yaw_rad: float):
        """Update current heading"""
        self.current_yaw_rad = yaw_rad

    def update(self, dt: float = None) -> AttitudeCommand:
        """
        Calculate attitude command from velocity error

        Args:
            dt: Time step in seconds (None for automatic)

        Returns:
            AttitudeCommand with roll, pitch angles
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

        # PID control: velocity error → acceleration target
        accel_target_x = self.pid_x.update(
            self.velocity_target.x,
            self.current_velocity.x,
            dt
        )
        accel_target_y = self.pid_y.update(
            self.velocity_target.y,
            self.current_velocity.y,
            dt
        )

        # Apply jerk limiting
        accel_target_x, accel_target_y = self._apply_jerk_limit(
            accel_target_x, accel_target_y, dt
        )

        # Convert NED acceleration to body-frame angles
        roll_deg, pitch_deg = self._accel_to_angles(
            accel_target_x, accel_target_y
        )

        # Limit bank angles
        roll_deg = max(-self.max_bank_angle, min(self.max_bank_angle, roll_deg))
        pitch_deg = max(-self.max_bank_angle, min(self.max_bank_angle, pitch_deg))

        return AttitudeCommand(
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            yaw_rate_dps=0.0  # Yaw handled separately
        )

    def _apply_jerk_limit(self, accel_x: float, accel_y: float,
                          dt: float) -> Tuple[float, float]:
        """
        Apply jerk limiting to acceleration commands

        Prevents sudden changes in acceleration that could
        destabilize the attitude controller.
        """
        max_change = self.jerk_limit * dt

        # Limit X acceleration change
        delta_x = accel_x - self.last_accel_x
        if abs(delta_x) > max_change:
            delta_x = max_change if delta_x > 0 else -max_change
        accel_x = self.last_accel_x + delta_x

        # Limit Y acceleration change
        delta_y = accel_y - self.last_accel_y
        if abs(delta_y) > max_change:
            delta_y = max_change if delta_y > 0 else -max_change
        accel_y = self.last_accel_y + delta_y

        self.last_accel_x = accel_x
        self.last_accel_y = accel_y

        return accel_x, accel_y

    def _accel_to_angles(self, accel_north: float, accel_east: float) -> Tuple[float, float]:
        """
        Convert NED acceleration to roll/pitch angles

        Rotates acceleration from NED frame to body frame,
        then calculates required bank angles.

        Based on iNav's acceleration to angle conversion.
        """
        cos_yaw = math.cos(self.current_yaw_rad)
        sin_yaw = math.sin(self.current_yaw_rad)

        # Rotate NED acceleration to body frame
        # Forward = North*cos(yaw) + East*sin(yaw)
        # Right = -North*sin(yaw) + East*cos(yaw)
        accel_forward = accel_north * cos_yaw + accel_east * sin_yaw
        accel_right = -accel_north * sin_yaw + accel_east * cos_yaw

        # Convert acceleration to bank angle
        # For a hovering multicopter: a = g * tan(θ)
        # Therefore: θ = atan(a / g)

        # Pitch: nose-down acceleration requires nose-down pitch
        # Negative accel_forward (decelerate) needs positive pitch (nose up)
        pitch_rad = math.atan2(-accel_forward, GRAVITY_MSS)

        # Roll: right acceleration requires right bank
        # Account for pitch when calculating roll
        roll_rad = math.atan2(accel_right * math.cos(pitch_rad), GRAVITY_MSS)

        # Convert to degrees
        roll_deg = math.degrees(roll_rad)
        pitch_deg = math.degrees(pitch_rad)

        return roll_deg, pitch_deg

    def reset(self):
        """Reset controller state"""
        self.pid_x.reset()
        self.pid_y.reset()
        self.last_accel_x = 0.0
        self.last_accel_y = 0.0
        self._last_update_time = 0.0

    def get_debug_info(self) -> dict:
        """Get debug information"""
        return {
            'vel_target_x': self.velocity_target.x,
            'vel_target_y': self.velocity_target.y,
            'vel_current_x': self.current_velocity.x,
            'vel_current_y': self.current_velocity.y,
            'accel_x': self.last_accel_x,
            'accel_y': self.last_accel_y,
            'pid_x': self.pid_x.get_components(),
            'pid_y': self.pid_y.get_components()
        }


def angles_to_rc(roll_deg: float, pitch_deg: float,
                 throttle: float, yaw_rate: float,
                 max_angle: float = 30.0) -> Tuple[int, int, int, int]:
    """
    Convert attitude commands to RC values (1000-2000)

    Args:
        roll_deg: Roll angle in degrees
        pitch_deg: Pitch angle in degrees
        throttle: Throttle 0.0-1.0
        yaw_rate: Yaw rate -1.0 to 1.0
        max_angle: Maximum angle corresponding to full stick

    Returns:
        Tuple of (roll_rc, pitch_rc, throttle_rc, yaw_rc)
    """
    # Roll: positive = right stick
    roll_rc = 1500 + int(roll_deg * 500 / max_angle)

    # Pitch: positive angle = nose up = stick back = higher value
    # But in Betaflight, stick forward (low) = nose down
    # So positive pitch should be lower value
    pitch_rc = 1500 - int(pitch_deg * 500 / max_angle)

    # Throttle: 0-1 → 1000-2000
    throttle_rc = 1000 + int(throttle * 1000)

    # Yaw: positive rate = right = higher value
    yaw_rc = 1500 + int(yaw_rate * 500)

    # Clamp to valid range
    roll_rc = max(1000, min(2000, roll_rc))
    pitch_rc = max(1000, min(2000, pitch_rc))
    throttle_rc = max(1000, min(2000, throttle_rc))
    yaw_rc = max(1000, min(2000, yaw_rc))

    return roll_rc, pitch_rc, throttle_rc, yaw_rc
