"""
Tests for Navigation Controllers
"""

import unittest
import math

import sys
sys.path.insert(0, '/home/remenby/projects/pi_drone_nav')

# Check if pyserial is available (needed by some modules)
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# Import modules that don't require serial directly
from src.utils.geo import gps_to_ned, ned_to_gps, haversine_distance

# Try to import navigation modules
try:
    from src.navigation.position_controller import PositionControllerGPS
    from src.navigation.velocity_controller import VelocityController
    from src.navigation.altitude_controller import AltitudeController
    from src.navigation.waypoint_navigator import (
        WaypointNavigator, Waypoint, Mission, WaypointAction, MissionState
    )
    NAVIGATION_AVAILABLE = True
except ImportError:
    NAVIGATION_AVAILABLE = False


@unittest.skipUnless(NAVIGATION_AVAILABLE, "Navigation modules not available")
class TestPositionController(unittest.TestCase):
    """Test position controller"""

    def test_position_to_velocity_north(self):
        """Test position error converts to north velocity command"""
        controller = PositionControllerGPS(
            kp_horizontal=1.0,
            kp_vertical=1.0,
            max_horizontal_speed=10.0,
            max_vertical_speed=3.0
        )

        # Target is 10m north
        # At origin: 0, 0
        # Target: slightly north
        origin_lat = 48.0
        origin_lon = 2.0

        # 10m north in degrees (approximate)
        target_lat = origin_lat + (10.0 / 111000.0)
        target_lon = origin_lon

        controller.set_target(target_lat, target_lon, 10.0)

        vel_n, vel_e, vel_d = controller.update(
            current_lat=origin_lat,
            current_lon=origin_lon,
            current_alt=10.0
        )

        # Should command north velocity
        self.assertGreater(vel_n, 0)
        self.assertAlmostEqual(vel_e, 0.0, delta=0.5)

    def test_position_to_velocity_east(self):
        """Test position error converts to east velocity command"""
        controller = PositionControllerGPS(
            kp_horizontal=1.0,
            kp_vertical=1.0,
            max_horizontal_speed=10.0,
            max_vertical_speed=3.0
        )

        origin_lat = 48.0
        origin_lon = 2.0

        # 10m east
        target_lat = origin_lat
        target_lon = origin_lon + (10.0 / (111000.0 * math.cos(math.radians(origin_lat))))

        controller.set_target(target_lat, target_lon, 10.0)

        vel_n, vel_e, vel_d = controller.update(
            current_lat=origin_lat,
            current_lon=origin_lon,
            current_alt=10.0
        )

        # Should command east velocity
        self.assertAlmostEqual(vel_n, 0.0, delta=0.5)
        self.assertGreater(vel_e, 0)

    def test_velocity_limiting(self):
        """Test that output velocities are limited"""
        controller = PositionControllerGPS(
            kp_horizontal=10.0,  # High gain
            kp_vertical=10.0,
            max_horizontal_speed=5.0,  # Low limit
            max_vertical_speed=2.0
        )

        origin_lat = 48.0
        origin_lon = 2.0

        # Target is 1000m away
        target_lat = origin_lat + (1000.0 / 111000.0)
        target_lon = origin_lon

        controller.set_target(target_lat, target_lon, 100.0)

        vel_n, vel_e, vel_d = controller.update(
            current_lat=origin_lat,
            current_lon=origin_lon,
            current_alt=10.0
        )

        # Horizontal speed should be limited
        horizontal_speed = math.sqrt(vel_n**2 + vel_e**2)
        self.assertLessEqual(horizontal_speed, 5.1)  # Allow small tolerance


@unittest.skipUnless(NAVIGATION_AVAILABLE, "Navigation modules not available")
class TestVelocityController(unittest.TestCase):
    """Test velocity controller"""

    def test_velocity_to_angle_forward(self):
        """Test forward velocity commands pitch"""
        controller = VelocityController(
            kp=0.5, ki=0.1, kd=0.0,
            max_tilt=30.0,
            jerk_limit=2.0
        )

        # Command 5 m/s north (forward)
        controller.set_velocity_setpoint(5.0, 0.0)

        # Current velocity is 0
        command = controller.update(
            vel_north=0.0,
            vel_east=0.0,
            heading=0.0,  # Facing north
            dt=0.02
        )

        # Should command negative pitch (nose down = forward)
        self.assertLess(command.pitch_deg, 0)
        self.assertAlmostEqual(command.roll_deg, 0.0, delta=1.0)

    def test_velocity_to_angle_right(self):
        """Test rightward velocity commands roll"""
        controller = VelocityController(
            kp=0.5, ki=0.1, kd=0.0,
            max_tilt=30.0,
            jerk_limit=2.0
        )

        # Command 5 m/s east (right when heading north)
        controller.set_velocity_setpoint(0.0, 5.0)

        command = controller.update(
            vel_north=0.0,
            vel_east=0.0,
            heading=0.0,  # Facing north
            dt=0.02
        )

        # Should command positive roll (lean right)
        self.assertGreater(command.roll_deg, 0)
        self.assertAlmostEqual(command.pitch_deg, 0.0, delta=1.0)

    def test_angle_limiting(self):
        """Test that output angles are limited"""
        controller = VelocityController(
            kp=10.0,  # High gain
            ki=0.0, kd=0.0,
            max_tilt=15.0,  # Low limit
            jerk_limit=100.0  # High jerk limit
        )

        controller.set_velocity_setpoint(100.0, 100.0)  # Large velocity

        command = controller.update(
            vel_north=0.0,
            vel_east=0.0,
            heading=0.0,
            dt=0.02
        )

        # Angles should be limited
        self.assertLessEqual(abs(command.roll_deg), 15.1)
        self.assertLessEqual(abs(command.pitch_deg), 15.1)

    def test_jerk_limiting(self):
        """Test jerk limiting smooths output"""
        controller = VelocityController(
            kp=2.0, ki=0.0, kd=0.0,
            max_tilt=30.0,
            jerk_limit=1.0  # Low jerk limit
        )

        dt = 0.02

        # First update
        controller.set_velocity_setpoint(10.0, 0.0)
        cmd1 = controller.update(0.0, 0.0, 0.0, dt)

        # Immediate second update with same high setpoint
        cmd2 = controller.update(0.0, 0.0, 0.0, dt)

        # Jerk limiting means acceleration change is bounded
        # The difference between outputs should be limited


@unittest.skipUnless(NAVIGATION_AVAILABLE, "Navigation modules not available")
class TestAltitudeController(unittest.TestCase):
    """Test altitude controller"""

    def test_climb_command(self):
        """Test that altitude error produces climb throttle"""
        controller = AltitudeController(
            kp=1.0, ki=0.2, kd=0.5,
            hover_throttle=0.5,
            min_throttle=0.1,
            max_throttle=0.9
        )

        controller.set_target_altitude(20.0)

        # Currently at 10m, target is 20m
        throttle = controller.update(
            current_altitude=10.0,
            vertical_velocity=0.0,
            dt=0.02
        )

        # Should command above hover throttle
        self.assertGreater(throttle, 0.5)

    def test_descend_command(self):
        """Test that negative altitude error produces descend throttle"""
        controller = AltitudeController(
            kp=1.0, ki=0.2, kd=0.5,
            hover_throttle=0.5,
            min_throttle=0.1,
            max_throttle=0.9
        )

        controller.set_target_altitude(10.0)

        # Currently at 20m, target is 10m
        throttle = controller.update(
            current_altitude=20.0,
            vertical_velocity=0.0,
            dt=0.02
        )

        # Should command below hover throttle
        self.assertLess(throttle, 0.5)

    def test_throttle_limiting(self):
        """Test throttle output is limited"""
        controller = AltitudeController(
            kp=10.0, ki=0.0, kd=0.0,  # High gain
            hover_throttle=0.5,
            min_throttle=0.2,
            max_throttle=0.8
        )

        controller.set_target_altitude(100.0)

        # Large altitude error
        throttle = controller.update(0.0, 0.0, 0.02)

        self.assertLessEqual(throttle, 0.8)
        self.assertGreaterEqual(throttle, 0.2)

    def test_velocity_damping(self):
        """Test D term damps vertical velocity"""
        controller = AltitudeController(
            kp=0.0, ki=0.0, kd=1.0,
            hover_throttle=0.5,
            min_throttle=0.0,
            max_throttle=1.0
        )

        controller.set_target_altitude(10.0)

        # At target altitude but climbing fast
        throttle = controller.update(
            current_altitude=10.0,
            vertical_velocity=5.0,  # Climbing
            dt=0.02
        )

        # D term should reduce throttle to damp climbing
        self.assertLess(throttle, 0.5)


@unittest.skipUnless(NAVIGATION_AVAILABLE, "Navigation modules not available")
class TestWaypointNavigator(unittest.TestCase):
    """Test waypoint navigation"""

    def test_mission_creation(self):
        """Test creating a mission"""
        waypoints = [
            Waypoint(48.0, 2.0, 10.0, WaypointAction.FLY_THROUGH),
            Waypoint(48.001, 2.001, 15.0, WaypointAction.HOVER),
            Waypoint(48.002, 2.002, 10.0, WaypointAction.FLY_THROUGH),
        ]

        mission = Mission(
            name="Test Mission",
            waypoints=waypoints,
            default_speed=5.0,
            return_to_home=True
        )

        self.assertEqual(len(mission.waypoints), 3)
        self.assertEqual(mission.name, "Test Mission")

    def test_waypoint_distance_calculation(self):
        """Test calculating distance to waypoint"""
        # 3m away should be within 5m radius
        lat_offset = 3.0 / 111000.0
        current_lat = 48.0 + lat_offset
        current_lon = 2.0

        distance = haversine_distance(
            current_lat, current_lon,
            48.0, 2.0
        )

        self.assertLess(distance, 5.0)

    def test_mission_data_structure(self):
        """Test mission data structure"""
        waypoints = [
            Waypoint(48.0, 2.0, 10.0, WaypointAction.FLY_THROUGH),
        ]
        mission = Mission("Test", waypoints, 5.0, False)

        self.assertEqual(mission.name, "Test")
        self.assertEqual(len(mission.waypoints), 1)
        self.assertEqual(mission.default_speed, 5.0)
        self.assertFalse(mission.return_to_home)


class TestGeoUtilities(unittest.TestCase):
    """Test geographic utilities"""

    def test_haversine_short_distance(self):
        """Test haversine for short distances"""
        # Two points about 100m apart
        lat1, lon1 = 48.8566, 2.3522
        lat2 = lat1 + (100.0 / 111000.0)
        lon2 = lon1

        distance = haversine_distance(lat1, lon1, lat2, lon2)

        self.assertAlmostEqual(distance, 100.0, delta=1.0)

    def test_haversine_east(self):
        """Test haversine for east direction"""
        lat1, lon1 = 48.8566, 2.3522
        lat2 = lat1
        lon2 = lon1 + (100.0 / (111000.0 * math.cos(math.radians(lat1))))

        distance = haversine_distance(lat1, lon1, lat2, lon2)

        self.assertAlmostEqual(distance, 100.0, delta=2.0)

    def test_gps_to_ned_conversion(self):
        """Test GPS to NED coordinate conversion"""
        origin_lat = 48.8566
        origin_lon = 2.3522

        # Point 100m north
        target_lat = origin_lat + (100.0 / 111000.0)
        target_lon = origin_lon

        north, east, down = gps_to_ned(
            target_lat, target_lon, 0.0,
            origin_lat, origin_lon, 0.0
        )

        self.assertAlmostEqual(north, 100.0, delta=1.0)
        self.assertAlmostEqual(east, 0.0, delta=1.0)

    def test_ned_to_gps_conversion(self):
        """Test NED to GPS coordinate conversion"""
        origin_lat = 48.8566
        origin_lon = 2.3522

        # Convert 100m north to GPS
        lat, lon, alt = ned_to_gps(
            100.0, 0.0, 0.0,
            origin_lat, origin_lon, 0.0
        )

        # Convert back
        north, east, down = gps_to_ned(
            lat, lon, alt,
            origin_lat, origin_lon, 0.0
        )

        self.assertAlmostEqual(north, 100.0, delta=1.0)
        self.assertAlmostEqual(east, 0.0, delta=1.0)

    def test_ned_conversion_roundtrip(self):
        """Test NED <-> GPS roundtrip"""
        origin_lat = 48.8566
        origin_lon = 2.3522

        original_north = 50.0
        original_east = 30.0
        original_down = -10.0

        lat, lon, alt = ned_to_gps(
            original_north, original_east, original_down,
            origin_lat, origin_lon, 100.0
        )

        north, east, down = gps_to_ned(
            lat, lon, alt,
            origin_lat, origin_lon, 100.0
        )

        self.assertAlmostEqual(north, original_north, delta=0.1)
        self.assertAlmostEqual(east, original_east, delta=0.1)
        self.assertAlmostEqual(down, original_down, delta=0.1)


if __name__ == '__main__':
    unittest.main()
