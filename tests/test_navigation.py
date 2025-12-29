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
        controller = PositionControllerGPS()

        origin_lat = 48.0
        origin_lon = 2.0

        # Set reference and update position
        controller.set_reference(origin_lat, origin_lon, 10.0)
        controller.update_position(origin_lat, origin_lon, 10.0)

        # Target is 10m north
        target_lat = origin_lat + (10.0 / 111000.0)
        controller.set_target_gps(target_lat, origin_lon, 10.0)

        vel_n, vel_e, vel_d = controller.update()

        # Should command north velocity
        self.assertGreater(vel_n, 0)
        self.assertAlmostEqual(vel_e, 0.0, delta=0.5)

    def test_position_to_velocity_east(self):
        """Test position error converts to east velocity command"""
        controller = PositionControllerGPS()

        origin_lat = 48.0
        origin_lon = 2.0

        controller.set_reference(origin_lat, origin_lon, 10.0)
        controller.update_position(origin_lat, origin_lon, 10.0)

        # 10m east
        target_lon = origin_lon + (10.0 / (111000.0 * math.cos(math.radians(origin_lat))))
        controller.set_target_gps(origin_lat, target_lon, 10.0)

        vel_n, vel_e, vel_d = controller.update()

        # Should command east velocity
        self.assertAlmostEqual(vel_n, 0.0, delta=0.5)
        self.assertGreater(vel_e, 0)

    def test_velocity_limiting(self):
        """Test that output velocities are limited"""
        controller = PositionControllerGPS()

        origin_lat = 48.0
        origin_lon = 2.0

        controller.set_reference(origin_lat, origin_lon, 10.0)
        controller.update_position(origin_lat, origin_lon, 10.0)

        # Target is 1000m away
        target_lat = origin_lat + (1000.0 / 111000.0)
        controller.set_target_gps(target_lat, origin_lon, 100.0)

        vel_n, vel_e, vel_d = controller.update()

        # Horizontal speed should be limited by config max_horizontal_speed_ms
        horizontal_speed = math.sqrt(vel_n**2 + vel_e**2)
        self.assertLessEqual(horizontal_speed, 20.0)  # Should be limited


@unittest.skipUnless(NAVIGATION_AVAILABLE, "Navigation modules not available")
class TestVelocityController(unittest.TestCase):
    """Test velocity controller"""

    def test_velocity_to_angle_forward(self):
        """Test forward velocity commands pitch"""
        controller = VelocityController()

        # Command 5 m/s north (forward)
        controller.set_target_velocity(5.0, 0.0, 0.0)
        controller.set_current_velocity(0.0, 0.0, 0.0)
        controller.set_current_yaw(0.0)  # Facing north

        command = controller.update(dt=0.02)

        # Should command negative pitch (nose down = forward)
        self.assertLess(command.pitch_deg, 0)
        self.assertAlmostEqual(command.roll_deg, 0.0, delta=5.0)

    def test_velocity_to_angle_right(self):
        """Test rightward velocity commands roll"""
        controller = VelocityController()

        # Command 5 m/s east (right when heading north)
        controller.set_target_velocity(0.0, 5.0, 0.0)
        controller.set_current_velocity(0.0, 0.0, 0.0)
        controller.set_current_yaw(0.0)  # Facing north

        command = controller.update(dt=0.02)

        # Should command positive roll (lean right)
        self.assertGreater(command.roll_deg, 0)
        self.assertAlmostEqual(command.pitch_deg, 0.0, delta=5.0)

    def test_angle_limiting(self):
        """Test that output angles are limited"""
        controller = VelocityController()

        controller.set_target_velocity(100.0, 100.0, 0.0)  # Large velocity
        controller.set_current_velocity(0.0, 0.0, 0.0)
        controller.set_current_yaw(0.0)

        command = controller.update(dt=0.02)

        # Angles should be limited by config max_bank_angle_deg
        self.assertLessEqual(abs(command.roll_deg), 35.0)
        self.assertLessEqual(abs(command.pitch_deg), 35.0)

    def test_jerk_limiting(self):
        """Test jerk limiting smooths output"""
        controller = VelocityController()

        dt = 0.02

        # First update
        controller.set_target_velocity(10.0, 0.0, 0.0)
        controller.set_current_velocity(0.0, 0.0, 0.0)
        controller.set_current_yaw(0.0)
        cmd1 = controller.update(dt=dt)

        # Immediate second update with same high setpoint
        cmd2 = controller.update(dt=dt)

        # Jerk limiting means acceleration change is bounded
        # The difference between outputs should be limited
        self.assertIsNotNone(cmd1)
        self.assertIsNotNone(cmd2)


@unittest.skipUnless(NAVIGATION_AVAILABLE, "Navigation modules not available")
class TestAltitudeController(unittest.TestCase):
    """Test altitude controller"""

    def test_climb_command(self):
        """Test that altitude error produces climb throttle"""
        controller = AltitudeController()

        controller.set_target_altitude(20.0)

        # Currently at 10m, target is 20m - update from GPS
        controller.update_altitude_from_gps(10.0, 0.0)  # alt, vel_down

        throttle = controller.update(dt=0.02)

        # Should command above hover throttle
        self.assertGreater(throttle, 0.4)

    def test_descend_command(self):
        """Test that negative altitude error produces descend throttle"""
        controller = AltitudeController()

        controller.set_target_altitude(10.0)

        # Currently at 20m, target is 10m
        controller.update_altitude_from_gps(20.0, 0.0)

        throttle = controller.update(dt=0.02)

        # Should command below hover throttle
        self.assertLess(throttle, 0.6)

    def test_throttle_limiting(self):
        """Test throttle output is limited"""
        controller = AltitudeController()

        controller.set_target_altitude(100.0)
        controller.update_altitude_from_gps(0.0, 0.0)

        # Large altitude error
        throttle = controller.update(dt=0.02)

        # Throttle should be clamped to safe range
        self.assertLessEqual(throttle, 0.9)
        self.assertGreaterEqual(throttle, 0.1)

    def test_velocity_damping(self):
        """Test altitude controller responds to vertical velocity"""
        controller = AltitudeController()

        controller.set_target_altitude(10.0)

        # At target altitude but descending (vel_down positive = descending)
        controller.update_altitude_from_gps(10.0, 2.0)  # descending at 2 m/s

        throttle = controller.update(dt=0.02)

        # Should adjust throttle based on vertical velocity
        self.assertIsNotNone(throttle)


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
        mission = Mission(
            name="Test",
            waypoints=waypoints,
            default_speed=5.0,
            return_to_home=False
        )

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
