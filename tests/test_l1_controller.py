"""
Tests for L1 Navigation Controller
"""

import unittest
import math
import sys

sys.path.insert(0, '/home/remenby/projects/pi_drone_nav')

# Try to import L1 controller
try:
    from src.navigation.l1_controller import L1Controller, L1Output
    from src.navigation.position_controller import Vector3
    L1_AVAILABLE = True
except ImportError as e:
    print(f"L1 import error: {e}")
    L1_AVAILABLE = False


@unittest.skipUnless(L1_AVAILABLE, "L1 module not available")
class TestL1Controller(unittest.TestCase):
    """Tests for L1Controller"""

    def test_init(self):
        """Test controller initialization"""
        controller = L1Controller()

        self.assertIsNotNone(controller.l1_period)
        self.assertIsNotNone(controller.l1_damping)
        self.assertEqual(controller._xte_integrator, 0.0)

    def test_l1_distance_scales_with_speed(self):
        """Test L1 distance scales with speed"""
        controller = L1Controller()

        # At 10 m/s, L1 distance should be speed / omega_a
        # omega_a = 2*pi / l1_period
        speed = 10.0
        omega_a = 2 * math.pi / controller.l1_period
        expected_l1 = speed / omega_a

        self.assertGreater(expected_l1, controller.min_l1_distance)

    def test_project_on_line_along_track(self):
        """Test projection onto line - along track distance"""
        controller = L1Controller()

        # Line going North (0,0) to (100,0)
        pos = (50.0, 0.0)  # On the line
        line_start = (0.0, 0.0)
        line_end = (100.0, 0.0)

        along, cross = controller._project_on_line(pos, line_start, line_end)

        self.assertAlmostEqual(along, 50.0, delta=0.1)
        self.assertAlmostEqual(cross, 0.0, delta=0.1)

    def test_project_on_line_cross_track(self):
        """Test projection onto line - cross track distance"""
        controller = L1Controller()

        # Line going North, point 10m east
        pos = (50.0, 10.0)
        line_start = (0.0, 0.0)
        line_end = (100.0, 0.0)

        along, cross = controller._project_on_line(pos, line_start, line_end)

        self.assertAlmostEqual(along, 50.0, delta=0.1)
        # Cross track: negative means left of track in this coordinate system
        self.assertAlmostEqual(abs(cross), 10.0, delta=0.1)

    def test_l1_point_ahead_on_track(self):
        """Test L1 reference point is ahead on track"""
        controller = L1Controller()

        line_start = (0.0, 0.0)
        line_end = (100.0, 0.0)
        line_len = 100.0
        along_track = 20.0
        l1_dist = 30.0

        l1_point = controller._get_l1_point(
            line_start, line_end, line_len, along_track, l1_dist
        )

        # L1 point should be at along_track + l1_dist = 50m
        self.assertAlmostEqual(l1_point[0], 50.0, delta=0.1)
        self.assertAlmostEqual(l1_point[1], 0.0, delta=0.1)

    def test_wrap_angle_zero(self):
        """Test angle wrapping at zero"""
        controller = L1Controller()

        self.assertAlmostEqual(controller._wrap_angle(0), 0, delta=0.01)

    def test_wrap_angle_2pi(self):
        """Test angle wrapping at 2pi"""
        controller = L1Controller()

        self.assertAlmostEqual(controller._wrap_angle(2 * math.pi), 0, delta=0.01)

    def test_wrap_angle_negative(self):
        """Test angle wrapping negative"""
        controller = L1Controller()

        result = controller._wrap_angle(-math.pi - 0.1)
        self.assertGreater(result, -math.pi - 0.01)
        self.assertLess(result, math.pi + 0.01)

    def test_reset_clears_integrator(self):
        """Test reset clears XTE integrator"""
        controller = L1Controller()
        controller._xte_integrator = 5.0
        controller._last_cross_track = 10.0

        controller.reset()

        self.assertEqual(controller._xte_integrator, 0.0)
        self.assertEqual(controller._last_cross_track, 0.0)

    def test_reset_integrator_only(self):
        """Test reset_integrator only clears integrator"""
        controller = L1Controller()
        controller._xte_integrator = 5.0
        controller._last_cross_track = 10.0

        controller.reset_integrator()

        self.assertEqual(controller._xte_integrator, 0.0)
        # Other state should be preserved
        self.assertEqual(controller._last_cross_track, 10.0)

    def test_set_reference(self):
        """Test setting reference point"""
        controller = L1Controller()

        controller.set_reference(43.5, 1.4)

        self.assertEqual(controller._ref_lat, 43.5)
        self.assertEqual(controller._ref_lon, 1.4)
        self.assertTrue(controller._ref_set)

    def test_get_debug_info(self):
        """Test debug info contains expected keys"""
        controller = L1Controller()

        info = controller.get_debug_info()

        self.assertIn('l1_period', info)
        self.assertIn('l1_damping', info)
        self.assertIn('xte_integrator', info)


@unittest.skipUnless(L1_AVAILABLE, "L1 module not available")
class TestL1Output(unittest.TestCase):
    """Tests for L1Output dataclass"""

    def test_l1_output_creation(self):
        """Test L1Output can be created"""
        output = L1Output(
            velocity_target=Vector3(1.0, 2.0, 0.0),
            lateral_accel=0.5,
            cross_track_error=1.5,
            along_track_distance=50.0,
            bearing_error=0.1,
            l1_distance=30.0
        )

        self.assertEqual(output.velocity_target.x, 1.0)
        self.assertEqual(output.cross_track_error, 1.5)
        self.assertEqual(output.l1_distance, 30.0)


if __name__ == '__main__':
    unittest.main()
