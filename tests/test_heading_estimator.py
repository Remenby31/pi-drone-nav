"""
Tests for Heading Estimator
"""

import unittest
import math
import sys

sys.path.insert(0, '/home/remenby/projects/pi_drone_nav')

# Try to import heading estimator
try:
    from src.navigation.heading_estimator import HeadingEstimator, HeadingState
    HEADING_AVAILABLE = True
except ImportError as e:
    print(f"HeadingEstimator import error: {e}")
    HEADING_AVAILABLE = False


@unittest.skipUnless(HEADING_AVAILABLE, "HeadingEstimator module not available")
class TestHeadingEstimator(unittest.TestCase):
    """Tests for HeadingEstimator"""

    def test_init(self):
        """Test estimator initialization"""
        estimator = HeadingEstimator()

        self.assertEqual(estimator.heading_rad, 0.0)
        self.assertEqual(estimator.gyro_bias_rad, 0.0)
        self.assertFalse(estimator._initialized)

    def test_initialize_heading(self):
        """Test heading initialization"""
        estimator = HeadingEstimator()
        estimator.initialize(math.pi / 4)  # 45 degrees

        self.assertAlmostEqual(estimator.heading_rad, math.pi / 4, delta=0.01)
        self.assertTrue(estimator._initialized)

    def test_update_stationary_uses_gyro(self):
        """Test that stationary vehicle uses gyro only"""
        estimator = HeadingEstimator()
        estimator.initialize(0.0)

        # Update with low speed (stationary)
        gyro_yaw = math.pi / 4  # 45 deg
        gps_heading = math.pi / 2  # 90 deg (different)
        ground_speed = 1.0  # Below threshold

        result = estimator.update(gyro_yaw, gps_heading, ground_speed, dt=0.02)

        # Should use gyro (with bias correction which is 0)
        self.assertAlmostEqual(result, gyro_yaw, delta=0.1)
        self.assertFalse(estimator._gps_contributing)

    def test_update_moving_fuses_gps(self):
        """Test that moving vehicle fuses GPS heading"""
        estimator = HeadingEstimator()
        estimator.initialize(0.0)

        # Update with high speed (moving)
        gyro_yaw = 0.0  # 0 deg
        gps_heading = math.pi / 2  # 90 deg
        ground_speed = 10.0  # Above threshold

        result = estimator.update(gyro_yaw, gps_heading, ground_speed, dt=0.02)

        # Should be somewhere between gyro and GPS (GPS has some influence)
        self.assertNotAlmostEqual(result, gyro_yaw, delta=0.001)
        self.assertTrue(estimator._gps_contributing)

    def test_gps_weight_increases_with_speed(self):
        """Test GPS weight increases with speed"""
        estimator1 = HeadingEstimator()
        estimator1.initialize(0.0)

        estimator2 = HeadingEstimator()
        estimator2.initialize(0.0)

        gyro_yaw = 0.0
        gps_heading = math.pi / 4

        # At threshold speed
        result_slow = estimator1.update(gyro_yaw, gps_heading, 4.0, dt=0.02)

        # At higher speed
        result_fast = estimator2.update(gyro_yaw, gps_heading, 20.0, dt=0.02)

        # Higher speed should give more GPS influence (closer to gps_heading)
        self.assertLess(abs(result_fast - gps_heading), abs(result_slow - gps_heading))

    def test_gyro_bias_learning(self):
        """Test gyro bias is learned over time"""
        estimator = HeadingEstimator()
        estimator.initialize(0.0)

        # Consistent error between gyro and GPS
        gyro_yaw = 0.0
        gps_heading = 0.1  # Small consistent error

        # Run many updates
        for _ in range(100):
            estimator.update(gyro_yaw, gps_heading, 10.0, dt=0.02)

        # Bias should have accumulated in direction of error
        self.assertNotEqual(estimator.gyro_bias_rad, 0.0)

    def test_gyro_bias_limited(self):
        """Test gyro bias is limited to reasonable range"""
        estimator = HeadingEstimator()
        estimator.initialize(0.0)

        # Large consistent error
        gyro_yaw = 0.0
        gps_heading = math.pi  # 180 deg error

        # Run many updates
        for _ in range(1000):
            estimator.update(gyro_yaw, gps_heading, 10.0, dt=0.02)

        # Bias should be limited to +/- 10 deg
        max_bias = math.radians(10)
        self.assertLessEqual(abs(estimator.gyro_bias_rad), max_bias + 0.01)

    def test_reset(self):
        """Test reset clears state"""
        estimator = HeadingEstimator()
        estimator.initialize(math.pi)
        estimator.gyro_bias_rad = 0.5
        estimator._gps_contributing = True

        estimator.reset()

        self.assertEqual(estimator.heading_rad, 0.0)
        self.assertEqual(estimator.gyro_bias_rad, 0.0)
        self.assertFalse(estimator._initialized)
        self.assertFalse(estimator._gps_contributing)

    def test_wrap_angle(self):
        """Test angle wrapping"""
        estimator = HeadingEstimator()

        self.assertAlmostEqual(estimator._wrap_angle(0), 0, delta=0.01)
        self.assertAlmostEqual(estimator._wrap_angle(math.pi), math.pi, delta=0.01)
        self.assertAlmostEqual(estimator._wrap_angle(2 * math.pi), 0, delta=0.01)
        self.assertAlmostEqual(estimator._wrap_angle(-math.pi), -math.pi, delta=0.01)

    def test_disabled_returns_gyro(self):
        """Test disabled estimator returns gyro directly"""
        estimator = HeadingEstimator()
        estimator.enabled = False

        gyro_yaw = math.pi / 3
        gps_heading = math.pi / 2
        ground_speed = 20.0

        result = estimator.update(gyro_yaw, gps_heading, ground_speed, dt=0.02)

        self.assertEqual(result, gyro_yaw)

    def test_get_state(self):
        """Test get_state returns correct values"""
        estimator = HeadingEstimator()
        estimator.initialize(math.pi / 4)
        estimator._gps_contributing = True

        state = estimator.get_state()

        self.assertAlmostEqual(state.heading_rad, math.pi / 4, delta=0.01)
        self.assertAlmostEqual(state.heading_deg, 45.0, delta=1.0)
        self.assertTrue(state.gps_valid)
        self.assertEqual(state.confidence, 1.0)

    def test_get_debug_info(self):
        """Test get_debug_info returns all fields"""
        estimator = HeadingEstimator()
        estimator.initialize(0.0)
        estimator.update(0.1, 0.2, 10.0, dt=0.02)

        info = estimator.get_debug_info()

        self.assertIn('heading_deg', info)
        self.assertIn('gyro_heading_deg', info)
        self.assertIn('gps_heading_deg', info)
        self.assertIn('error_deg', info)
        self.assertIn('gyro_bias_deg', info)
        self.assertIn('gps_contributing', info)
        self.assertIn('enabled', info)


@unittest.skipUnless(HEADING_AVAILABLE, "HeadingEstimator module not available")
class TestHeadingState(unittest.TestCase):
    """Tests for HeadingState dataclass"""

    def test_heading_state_creation(self):
        """Test HeadingState can be created"""
        state = HeadingState(
            heading_rad=math.pi / 4,
            heading_deg=45.0,
            gyro_bias_rad=0.01,
            gps_valid=True,
            confidence=0.8
        )

        self.assertEqual(state.heading_deg, 45.0)
        self.assertTrue(state.gps_valid)
        self.assertEqual(state.confidence, 0.8)


if __name__ == '__main__':
    unittest.main()
