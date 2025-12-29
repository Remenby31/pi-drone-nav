"""
Pytest configuration and fixtures
"""

import pytest
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))


@pytest.fixture
def mock_serial():
    """Fixture for mocked serial port"""
    from unittest.mock import MagicMock
    serial = MagicMock()
    serial.is_open = True
    serial.in_waiting = 0
    return serial


@pytest.fixture
def sample_gps_fix():
    """Fixture for sample GPS fix data"""
    from src.drivers.gps_ubx import GPSFix
    return GPSFix(
        fix_type=3,
        satellites=12,
        latitude=48.8566,
        longitude=2.3522,
        altitude=100.0,
        vel_north=5.0,
        vel_east=3.0,
        vel_down=-0.5,
        ground_speed=5.83,
        heading=30.96,
        hdop=1.2,
        vdop=1.5,
        timestamp=123456789
    )


@pytest.fixture
def sample_attitude():
    """Fixture for sample attitude data"""
    from src.drivers.msp import Attitude
    return Attitude(roll=5.0, pitch=-3.0, yaw=45.0)


@pytest.fixture
def sample_waypoints():
    """Fixture for sample waypoints"""
    from src.navigation.waypoint_navigator import Waypoint, WaypointAction

    return [
        Waypoint(48.8566, 2.3522, 10.0, WaypointAction.FLY_THROUGH, radius=3.0),
        Waypoint(48.8570, 2.3530, 15.0, WaypointAction.HOVER, radius=2.0),
        Waypoint(48.8575, 2.3535, 10.0, WaypointAction.FLY_THROUGH, radius=3.0),
    ]


@pytest.fixture
def sample_mission(sample_waypoints):
    """Fixture for sample mission"""
    from src.navigation.waypoint_navigator import Mission

    return Mission(
        name="Test Mission",
        waypoints=sample_waypoints,
        default_speed=5.0,
        return_to_home=True
    )


@pytest.fixture
def pid_controller():
    """Fixture for standard PID controller"""
    from src.navigation.pid import PIDController

    return PIDController(
        kp=1.0,
        ki=0.5,
        kd=0.1,
        output_limit=10.0,
        integral_limit=5.0
    )


@pytest.fixture
def position_controller():
    """Fixture for position controller"""
    from src.navigation.position_controller import PositionControllerGPS

    return PositionControllerGPS(
        kp_horizontal=0.8,
        kp_vertical=1.0,
        max_horizontal_speed=10.0,
        max_vertical_speed=3.0
    )


@pytest.fixture
def velocity_controller():
    """Fixture for velocity controller"""
    from src.navigation.velocity_controller import VelocityController

    return VelocityController(
        kp=0.5,
        ki=0.1,
        kd=0.05,
        max_tilt=30.0,
        jerk_limit=1.7
    )


@pytest.fixture
def altitude_controller():
    """Fixture for altitude controller"""
    from src.navigation.altitude_controller import AltitudeController

    return AltitudeController(
        kp=1.0,
        ki=0.3,
        kd=0.5,
        hover_throttle=0.5,
        min_throttle=0.1,
        max_throttle=0.9
    )


@pytest.fixture
def flight_state_machine():
    """Fixture for flight state machine"""
    from src.flight.state_machine import FlightStateMachine

    return FlightStateMachine()
