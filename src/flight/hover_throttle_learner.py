"""
Hover Throttle Learner - ArduPilot MOT_THST_HOVER style

Automatically learns the throttle required to hover by observing
throttle output during stable hover conditions.

Reference: AP_MotorsMulticopter::update_throttle_hover()

The algorithm uses an exponential moving average (EMA) to gradually
update the hover throttle estimate when the drone is in a stable hover.
"""

import json
import os
import time
from typing import Optional, TYPE_CHECKING
import logging

if TYPE_CHECKING:
    from ..config import HoverLearnConfig

logger = logging.getLogger(__name__)


class HoverThrottleLearner:
    """
    Learns hover throttle using exponential moving average

    ArduPilot algorithm:
    hover = hover + (dt / (dt + TC)) * (throttle - hover)

    This is equivalent to an EMA with alpha = dt / (dt + TC)

    Learning only occurs when specific conditions are met:
    - Drone is in position hold or similar mode
    - Above minimum altitude
    - Low vertical speed (actually hovering)
    - Low horizontal speed (not maneuvering)
    """

    def __init__(self, config: 'HoverLearnConfig'):
        """
        Initialize hover throttle learner

        Args:
            config: HoverLearnConfig with learning parameters
        """
        self.config = config

        # Current learned value (start at midpoint)
        self._hover_throttle = (config.min_hover_throttle +
                                config.max_hover_throttle) / 2

        # State
        self._learning_active = False
        self._last_update_time = 0.0
        self._samples_count = 0

        # Load saved value if exists
        self._load_saved_value()

    @property
    def hover_throttle(self) -> float:
        """Current hover throttle estimate"""
        return self._hover_throttle

    @property
    def is_learning(self) -> bool:
        """Check if actively learning"""
        return self._learning_active

    def update(self, dt: float, throttle: float,
               altitude_m: float, climb_rate_ms: float,
               horizontal_speed_ms: float, is_position_hold: bool) -> bool:
        """
        Update hover throttle estimate

        Args:
            dt: Time step in seconds
            throttle: Current throttle output (0.0-1.0)
            altitude_m: Current altitude in meters
            climb_rate_ms: Vertical speed in m/s (positive = up)
            horizontal_speed_ms: Horizontal ground speed in m/s
            is_position_hold: True if in position hold or similar mode

        Returns:
            True if learning occurred this update
        """
        if not self.config.enabled:
            return False

        # Check learning conditions
        can_learn = self._check_learning_conditions(
            altitude_m, climb_rate_ms, horizontal_speed_ms, is_position_hold
        )

        if not can_learn:
            self._learning_active = False
            return False

        self._learning_active = True
        self._samples_count += 1

        # ArduPilot EMA formula
        # alpha = dt / (dt + time_constant)
        # This gives a smooth exponential filter where time_constant
        # determines how quickly the estimate adapts
        tc = self.config.time_constant_sec
        alpha = dt / (dt + tc)

        # Update estimate
        new_hover = self._hover_throttle + alpha * (throttle - self._hover_throttle)

        # Clamp to valid range
        self._hover_throttle = max(
            self.config.min_hover_throttle,
            min(self.config.max_hover_throttle, new_hover)
        )

        self._last_update_time = time.time()

        return True

    def _check_learning_conditions(self, altitude_m: float, climb_rate_ms: float,
                                    horizontal_speed_ms: float,
                                    is_position_hold: bool) -> bool:
        """
        Check if conditions are suitable for learning

        Learning requires:
        - Position hold or similar autonomous mode (not manual)
        - Above minimum altitude (safety margin from ground)
        - Low vertical speed (actually hovering, not climbing/descending)
        - Low horizontal speed (not actively maneuvering)
        """
        # Must be in appropriate mode (not manual)
        if not is_position_hold:
            return False

        # Must be above ground (safety margin)
        if altitude_m < self.config.min_altitude_m:
            return False

        # Must be hovering (low vertical speed)
        if abs(climb_rate_ms) > self.config.max_climb_rate_ms:
            return False

        # Must not be maneuvering (low horizontal speed)
        if horizontal_speed_ms > self.config.max_horizontal_speed_ms:
            return False

        return True

    def save(self):
        """Save learned value to file"""
        if not self.config.save_on_disarm:
            return

        # Don't overwrite saved value if we haven't learned anything
        if self._samples_count == 0:
            logger.debug("Not saving hover throttle: no samples collected")
            return

        try:
            data = {
                'hover_throttle': self._hover_throttle,
                'samples_count': self._samples_count,
                'timestamp': time.time()
            }

            # Save to config directory
            save_path = os.path.join(
                os.path.dirname(__file__), '..', '..', 'config',
                self.config.save_file
            )

            # Ensure directory exists
            os.makedirs(os.path.dirname(save_path), exist_ok=True)

            with open(save_path, 'w') as f:
                json.dump(data, f, indent=2)

            logger.info(f"Saved hover throttle: {self._hover_throttle:.3f} "
                       f"({self._samples_count} samples)")

        except Exception as e:
            logger.warning(f"Failed to save hover throttle: {e}")

    def _load_saved_value(self):
        """Load previously saved hover throttle"""
        try:
            save_path = os.path.join(
                os.path.dirname(__file__), '..', '..', 'config',
                self.config.save_file
            )

            if os.path.exists(save_path):
                with open(save_path, 'r') as f:
                    data = json.load(f)

                loaded_value = data.get('hover_throttle', self._hover_throttle)

                # Validate loaded value is within bounds
                if (self.config.min_hover_throttle <= loaded_value <=
                    self.config.max_hover_throttle):
                    self._hover_throttle = loaded_value
                    logger.info(f"Loaded saved hover throttle: {self._hover_throttle:.3f}")
                else:
                    logger.warning(f"Saved hover throttle {loaded_value} out of bounds, using default")

        except FileNotFoundError:
            logger.debug("No saved hover throttle file found, using default")
        except json.JSONDecodeError as e:
            logger.warning(f"Invalid hover throttle file: {e}")
        except Exception as e:
            logger.debug(f"Could not load saved hover throttle: {e}")

    def reset(self, initial_value: Optional[float] = None):
        """
        Reset learner state

        Args:
            initial_value: Optional starting value for hover throttle
        """
        if initial_value is not None:
            self._hover_throttle = max(
                self.config.min_hover_throttle,
                min(self.config.max_hover_throttle, initial_value)
            )
        self._samples_count = 0
        self._learning_active = False

    def get_status(self) -> dict:
        """Get learner status for telemetry"""
        return {
            'hover_throttle': self._hover_throttle,
            'learning_active': self._learning_active,
            'samples_count': self._samples_count,
            'enabled': self.config.enabled
        }
