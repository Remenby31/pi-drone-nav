"""
Heading Estimator

Fuses gyroscope heading with GPS course-over-ground to compensate
for gyro drift when no magnetometer is available.

Based on complementary filter approach used in ArduPilot/iNav.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional

from ..config import get_config


@dataclass
class HeadingState:
    """Current heading estimator state"""
    heading_rad: float = 0.0
    heading_deg: float = 0.0
    gyro_bias_rad: float = 0.0
    gps_valid: bool = False
    confidence: float = 0.0  # 0-1, higher when GPS contributing


class HeadingEstimator:
    """
    Fuses gyroscope heading with GPS course-over-ground.

    When the drone is moving (speed > threshold), GPS heading is used
    to correct gyro drift. When hovering, only gyro is used but with
    accumulated bias correction.

    This allows reasonable heading estimation without a magnetometer,
    though heading will drift slowly during extended hover.
    """

    def __init__(self):
        """Initialize heading estimator"""
        config = get_config()

        # Configuration
        self.min_speed_for_gps = config.navigation.heading_min_speed_ms
        self.gps_weight = config.navigation.heading_gps_weight
        self.enabled = config.navigation.enable_heading_fusion

        # State
        self.heading_rad = 0.0
        self.gyro_bias_rad = 0.0
        self._initialized = False

        # Bias learning rate (very slow to avoid noise)
        self._bias_learn_rate = 0.001

        # PT1 filter for GPS heading noise
        self._gps_heading_filtered = 0.0
        self._gps_filter_alpha = 0.3  # Higher = more responsive

        # Timing
        self._last_update_time = 0.0

        # Debug/telemetry
        self._last_gps_heading = 0.0
        self._last_gyro_heading = 0.0
        self._last_error = 0.0
        self._gps_contributing = False

    def initialize(self, initial_heading_rad: float):
        """
        Initialize with known heading

        Args:
            initial_heading_rad: Initial heading in radians
        """
        self.heading_rad = self._wrap_angle(initial_heading_rad)
        self._gps_heading_filtered = self.heading_rad
        self._initialized = True

    def update(self, gyro_yaw_rad: float, gps_cog_rad: float,
               ground_speed: float, dt: float = None) -> float:
        """
        Update heading estimate with new sensor data

        Args:
            gyro_yaw_rad: Heading from gyroscope/AHRS (radians)
            gps_cog_rad: Course over ground from GPS (radians)
            ground_speed: Ground speed in m/s
            dt: Time step in seconds (None for automatic)

        Returns:
            Fused heading estimate in radians
        """
        if not self.enabled:
            return gyro_yaw_rad

        # Calculate dt
        current_time = time.time()
        if dt is None:
            if self._last_update_time > 0:
                dt = current_time - self._last_update_time
            else:
                dt = 0.02
        self._last_update_time = current_time
        dt = max(0.001, min(dt, 0.5))

        # Initialize on first update
        if not self._initialized:
            self.initialize(gyro_yaw_rad)

        # Store for debug
        self._last_gyro_heading = gyro_yaw_rad
        self._last_gps_heading = gps_cog_rad

        # Check if GPS heading is usable
        gps_usable = ground_speed > self.min_speed_for_gps

        if gps_usable:
            # Filter GPS heading to reduce noise
            gps_error = self._wrap_angle(gps_cog_rad - self._gps_heading_filtered)
            self._gps_heading_filtered = self._wrap_angle(
                self._gps_heading_filtered + self._gps_filter_alpha * gps_error
            )

            # Calculate weight based on speed (more trust at higher speed)
            # Weight increases linearly from min_speed to 2x min_speed
            speed_factor = min(1.0, (ground_speed - self.min_speed_for_gps) /
                              self.min_speed_for_gps)
            alpha = self.gps_weight * (0.5 + 0.5 * speed_factor)

            # Error between gyro and GPS
            error = self._wrap_angle(self._gps_heading_filtered - gyro_yaw_rad)
            self._last_error = error

            # Complementary filter: blend gyro and GPS
            self.heading_rad = self._wrap_angle(
                gyro_yaw_rad + alpha * error
            )

            # Update gyro bias estimate (very slowly)
            # This corrects for long-term drift
            self.gyro_bias_rad += error * self._bias_learn_rate * dt

            # Limit bias to reasonable range (+/- 10 deg)
            max_bias = math.radians(10)
            self.gyro_bias_rad = max(-max_bias, min(max_bias, self.gyro_bias_rad))

            self._gps_contributing = True

        else:
            # GPS not usable (hovering or slow movement)
            # Use gyro with bias correction
            self.heading_rad = self._wrap_angle(
                gyro_yaw_rad - self.gyro_bias_rad
            )
            self._gps_contributing = False
            self._last_error = 0.0

        return self.heading_rad

    def get_state(self) -> HeadingState:
        """Get current estimator state for telemetry"""
        return HeadingState(
            heading_rad=self.heading_rad,
            heading_deg=math.degrees(self.heading_rad),
            gyro_bias_rad=self.gyro_bias_rad,
            gps_valid=self._gps_contributing,
            confidence=1.0 if self._gps_contributing else 0.5
        )

    def get_debug_info(self) -> dict:
        """Get debug information"""
        return {
            'heading_deg': math.degrees(self.heading_rad),
            'gyro_heading_deg': math.degrees(self._last_gyro_heading),
            'gps_heading_deg': math.degrees(self._last_gps_heading),
            'gps_filtered_deg': math.degrees(self._gps_heading_filtered),
            'error_deg': math.degrees(self._last_error),
            'gyro_bias_deg': math.degrees(self.gyro_bias_rad),
            'gps_contributing': self._gps_contributing,
            'enabled': self.enabled
        }

    def reset(self):
        """Reset estimator state"""
        self.heading_rad = 0.0
        self.gyro_bias_rad = 0.0
        self._gps_heading_filtered = 0.0
        self._initialized = False
        self._last_update_time = 0.0
        self._gps_contributing = False

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
