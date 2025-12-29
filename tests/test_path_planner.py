"""
Tests for Path Planner
"""

import unittest
import math
import sys

sys.path.insert(0, '/home/remenby/projects/pi_drone_nav')

# Try to import path planner
try:
    from src.navigation.path_planner import PathPlanner, PathSegment
    from src.navigation.waypoint_navigator import Mission, Waypoint
    PATH_PLANNER_AVAILABLE = True
except ImportError as e:
    print(f"PathPlanner import error: {e}")
    PATH_PLANNER_AVAILABLE = False


@unittest.skipUnless(PATH_PLANNER_AVAILABLE, "PathPlanner module not available")
class TestPathPlanner(unittest.TestCase):
    """Tests for PathPlanner"""

    def test_init(self):
        """Test path planner initialization"""
        planner = PathPlanner()

        self.assertIsNotNone(planner.max_bank_angle_deg)
        self.assertIsNotNone(planner.min_turn_radius_m)
        self.assertEqual(len(planner.segments), 0)

    def test_bearing_calculation_north(self):
        """Test bearing calculation going north"""
        planner = PathPlanner()

        # Going north
        bearing = planner._calculate_bearing(0.0, 0.0, 1.0, 0.0)

        self.assertAlmostEqual(bearing, 0.0, delta=0.01)

    def test_bearing_calculation_east(self):
        """Test bearing calculation going east"""
        planner = PathPlanner()

        # Going east (at equator)
        bearing = planner._calculate_bearing(0.0, 0.0, 0.0, 1.0)

        self.assertAlmostEqual(bearing, math.pi / 2, delta=0.01)

    def test_bearing_calculation_south(self):
        """Test bearing calculation going south"""
        planner = PathPlanner()

        # Going south
        bearing = planner._calculate_bearing(1.0, 0.0, 0.0, 0.0)

        # Should be pi or -pi
        self.assertAlmostEqual(abs(bearing), math.pi, delta=0.01)

    def test_distance_calculation(self):
        """Test distance calculation"""
        planner = PathPlanner()

        # About 111km per degree of latitude at equator
        distance = planner._calculate_distance(0.0, 0.0, 0.001, 0.0)

        # 0.001 degrees ≈ 111m
        self.assertAlmostEqual(distance, 111.0, delta=10.0)

    def test_turn_radius_calculation(self):
        """Test turn radius calculation"""
        planner = PathPlanner()

        # R = v^2 / (g * tan(bank))
        speed = 10.0
        radius = planner._calculate_turn_radius(speed)

        expected = (speed ** 2) / (9.80665 * math.tan(math.radians(planner.max_bank_angle_deg)))
        self.assertAlmostEqual(radius, max(expected, planner.min_turn_radius_m), delta=1.0)

    def test_turn_start_distance_90deg(self):
        """Test turn start distance for 90 degree turn"""
        planner = PathPlanner()

        # For 90 deg turn: d = R * tan(45) = R * 1 = R
        turn_radius = 10.0
        turn_angle = math.pi / 2  # 90 degrees

        distance = planner._calculate_turn_start_distance(turn_angle, turn_radius)

        self.assertAlmostEqual(distance, turn_radius, delta=1.0)

    def test_turn_start_distance_small_angle(self):
        """Test turn start distance for small angle (no turn)"""
        planner = PathPlanner()

        # For small turns, no anticipation needed
        turn_radius = 10.0
        turn_angle = math.radians(3)  # 3 degrees

        distance = planner._calculate_turn_start_distance(turn_angle, turn_radius)

        self.assertEqual(distance, 0.0)

    def test_turn_speed_no_reduction_small_angle(self):
        """Test speed not reduced for small turns"""
        planner = PathPlanner()

        default_speed = 10.0
        turn_angle = math.radians(10)  # Small turn

        speed = planner._calculate_turn_speed(turn_angle, default_speed)

        self.assertEqual(speed, default_speed)

    def test_turn_speed_reduction_sharp_turn(self):
        """Test speed reduced for sharp turns"""
        planner = PathPlanner()

        default_speed = 10.0
        turn_angle = math.pi  # 180 degree turn

        speed = planner._calculate_turn_speed(turn_angle, default_speed)

        # Should be reduced
        self.assertLess(speed, default_speed)
        self.assertGreater(speed, 0)

    def test_calculate_mission_two_waypoints(self):
        """Test mission calculation with two waypoints"""
        planner = PathPlanner()

        # Create simple mission: two waypoints going north
        wp1 = Waypoint(latitude=0.0, longitude=0.0, altitude=10.0)
        wp2 = Waypoint(latitude=0.001, longitude=0.0, altitude=10.0)
        mission = Mission(name="Test", waypoints=[wp1, wp2])

        planner.calculate_mission(mission, default_speed=10.0)

        self.assertEqual(len(planner.segments), 1)
        self.assertAlmostEqual(planner.segments[0].bearing_rad, 0.0, delta=0.01)
        self.assertEqual(planner.segments[0].turn_angle_rad, 0.0)  # No next segment

    def test_calculate_mission_three_waypoints_turn(self):
        """Test mission with turn between segments"""
        planner = PathPlanner()

        # Create mission with 90 degree turn: North then East
        wp1 = Waypoint(latitude=0.0, longitude=0.0, altitude=10.0)
        wp2 = Waypoint(latitude=0.001, longitude=0.0, altitude=10.0)  # North
        wp3 = Waypoint(latitude=0.001, longitude=0.001, altitude=10.0)  # Then East
        mission = Mission(name="Test", waypoints=[wp1, wp2, wp3])

        planner.calculate_mission(mission, default_speed=10.0)

        self.assertEqual(len(planner.segments), 2)

        # First segment goes north, turns right (east) = +90 deg
        self.assertAlmostEqual(planner.segments[0].bearing_rad, 0.0, delta=0.1)
        self.assertAlmostEqual(planner.segments[0].turn_angle_rad, math.pi / 2, delta=0.2)

        # Second segment goes east, no next turn
        self.assertAlmostEqual(planner.segments[1].bearing_rad, math.pi / 2, delta=0.1)
        self.assertEqual(planner.segments[1].turn_angle_rad, 0.0)

    def test_interpolated_altitude(self):
        """Test altitude interpolation"""
        planner = PathPlanner()

        # Create segment with altitude change
        segment = PathSegment(
            index=0,
            from_wp=Waypoint(latitude=0.0, longitude=0.0, altitude=10.0),
            to_wp=Waypoint(latitude=0.001, longitude=0.0, altitude=20.0),
            bearing_rad=0.0,
            distance_m=111.0,
            turn_angle_rad=0.0,
            turn_radius_m=10.0,
            turn_start_distance_m=0.0,
            altitude_change_m=10.0,
            climb_angle_rad=0.1,
            entry_speed_ms=10.0,
            exit_speed_ms=10.0
        )

        # At 50% progress, altitude should be 15m
        alt = planner.get_interpolated_altitude(segment, 0.5)
        self.assertAlmostEqual(alt, 15.0, delta=0.1)

        # At 0% progress, altitude should be 10m
        alt = planner.get_interpolated_altitude(segment, 0.0)
        self.assertAlmostEqual(alt, 10.0, delta=0.1)

        # At 100% progress, altitude should be 20m
        alt = planner.get_interpolated_altitude(segment, 1.0)
        self.assertAlmostEqual(alt, 20.0, delta=0.1)

    def test_wrap_angle(self):
        """Test angle wrapping"""
        planner = PathPlanner()

        self.assertAlmostEqual(planner._wrap_angle(0), 0, delta=0.01)
        self.assertAlmostEqual(planner._wrap_angle(2 * math.pi), 0, delta=0.01)
        self.assertAlmostEqual(planner._wrap_angle(-2 * math.pi), 0, delta=0.01)

    def test_total_distance(self):
        """Test total distance calculation"""
        planner = PathPlanner()

        # Create mission with two segments
        wp1 = Waypoint(latitude=0.0, longitude=0.0, altitude=10.0)
        wp2 = Waypoint(latitude=0.001, longitude=0.0, altitude=10.0)
        wp3 = Waypoint(latitude=0.002, longitude=0.0, altitude=10.0)
        mission = Mission(name="Test", waypoints=[wp1, wp2, wp3])

        planner.calculate_mission(mission, default_speed=10.0)

        # Total should be about 222m (2 x 111m)
        total = planner.get_total_distance()
        self.assertAlmostEqual(total, 222.0, delta=20.0)

    def test_get_segment(self):
        """Test getting segment by index"""
        planner = PathPlanner()

        wp1 = Waypoint(latitude=0.0, longitude=0.0, altitude=10.0)
        wp2 = Waypoint(latitude=0.001, longitude=0.0, altitude=10.0)
        mission = Mission(name="Test", waypoints=[wp1, wp2])

        planner.calculate_mission(mission, default_speed=10.0)

        # Valid index
        segment = planner.get_segment(0)
        self.assertIsNotNone(segment)

        # Invalid index
        segment = planner.get_segment(10)
        self.assertIsNone(segment)

    def test_set_reference(self):
        """Test setting reference point"""
        planner = PathPlanner()

        planner.set_reference(43.5, 1.4)

        self.assertEqual(planner.ref_lat, 43.5)
        self.assertEqual(planner.ref_lon, 1.4)
        self.assertTrue(planner._ref_set)


if __name__ == '__main__':
    unittest.main()
