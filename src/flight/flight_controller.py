"""
Main Flight Controller

Integrates all subsystems and runs the main control loop.
"""

import time
import threading
from typing import Optional, Tuple
import logging

from .state_machine import FlightStateMachine, FlightState
from ..drivers.msp import MSPClient, MSPError
from ..drivers.gps_ubx import GPSDriver, GPSFix
from ..drivers.serial_manager import SerialManager
from ..navigation.position_controller import PositionControllerGPS
from ..navigation.velocity_controller import VelocityController, angles_to_rc
from ..navigation.altitude_controller import AltitudeController
from ..navigation.waypoint_navigator import WaypointNavigator, Mission
from ..config import get_config

logger = logging.getLogger(__name__)


class FlightController:
    """
    Main flight controller

    Coordinates:
    - GPS driver
    - MSP client (Betaflight communication)
    - Navigation controllers
    - State machine
    - Mission execution
    """

    def __init__(self, simulation: bool = False):
        """
        Initialize flight controller

        Args:
            simulation: If True, use simulation mode
        """
        self.config = get_config()
        self.simulation = simulation

        # State machine
        self.state_machine = FlightStateMachine()
        self._setup_state_callbacks()

        # Communication
        self.serial_manager: Optional[SerialManager] = None
        self.msp: Optional[MSPClient] = None
        self.gps: Optional[GPSDriver] = None

        # Navigation
        self.pos_controller = PositionControllerGPS()
        self.vel_controller = VelocityController()
        self.alt_controller = AltitudeController()
        self.waypoint_nav: Optional[WaypointNavigator] = None

        # Control loop
        self._running = False
        self._control_thread: Optional[threading.Thread] = None
        self._update_rate = self.config.navigation.update_rate_hz
        self._last_update_time = 0.0

        # State
        self._armed = False
        self._target_altitude = 0.0
        self._current_yaw = 0.0

        # RC output
        self._rc_channels = [1500] * 8

        # Telemetry
        self._last_gps_fix: Optional[GPSFix] = None
        self._last_attitude = None

    def initialize(self) -> bool:
        """
        Initialize all subsystems

        Returns:
            True if initialization successful
        """
        logger.info("Initializing flight controller...")

        try:
            # Initialize serial connection
            if not self.simulation:
                self.serial_manager = SerialManager(
                    uart_port=self.config.serial.port,
                    usb_port=self.config.serial.usb_port,
                    baudrate=self.config.serial.baudrate,
                    prefer_usb=self.config.serial.use_usb
                )

                if not self.serial_manager.connect():
                    logger.error("Failed to connect to flight controller")
                    return False

                # Initialize MSP client
                self.msp = MSPClient(self.serial_manager, timeout=0.1)

                if not self.msp.connect():
                    logger.error("MSP connection failed")
                    return False

                # Initialize GPS (on separate port)
                # TODO: Initialize GPS driver

            else:
                # Simulation mode
                logger.info("Running in simulation mode")
                self._init_simulation()

            # Initialize waypoint navigator
            self.waypoint_nav = WaypointNavigator(self.pos_controller)

            logger.info("Flight controller initialized successfully")
            return True

        except Exception as e:
            logger.error(f"Initialization failed: {e}")
            return False

    def _init_simulation(self):
        """Initialize simulation mode"""
        # Create simulated MSP client
        # In simulation mode, we might connect to Betaflight SITL
        if self.config.simulation.enabled:
            import socket
            # Connect to SITL TCP port
            sitl_host = self.config.simulation.sitl_host
            sitl_port = self.config.simulation.sitl_msp_port

            logger.info(f"Connecting to SITL at {sitl_host}:{sitl_port}")
            # TODO: Implement TCP serial wrapper

    def _setup_state_callbacks(self):
        """Setup state machine callbacks"""

        @self.state_machine.on_enter(FlightState.TAKEOFF)
        def on_takeoff_enter():
            logger.info("Starting takeoff sequence")
            self._target_altitude = 3.0  # Default takeoff altitude

        @self.state_machine.on_enter(FlightState.LANDING)
        def on_landing_enter():
            logger.info("Starting landing sequence")
            self._target_altitude = 0.0

        @self.state_machine.on_enter(FlightState.POSITION_HOLD)
        def on_position_hold():
            logger.info("Position hold activated")
            self.pos_controller.controller.capture_hold_position()

        @self.state_machine.on_enter(FlightState.FAILSAFE)
        def on_failsafe():
            logger.warning("FAILSAFE - Returning control to Betaflight")
            # Stop sending RC commands to let Betaflight GPS Rescue take over

    def start(self):
        """Start the control loop"""
        if self._running:
            return

        self._running = True
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()
        logger.info("Control loop started")

    def stop(self):
        """Stop the control loop"""
        self._running = False
        if self._control_thread:
            self._control_thread.join(timeout=2.0)
        logger.info("Control loop stopped")

    def _control_loop(self):
        """Main control loop"""
        dt = 1.0 / self._update_rate

        while self._running:
            loop_start = time.time()

            try:
                self._update()
            except Exception as e:
                logger.error(f"Control loop error: {e}")

            # Maintain update rate
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _update(self):
        """Single control loop iteration"""
        current_time = time.time()
        dt = current_time - self._last_update_time if self._last_update_time > 0 else 0.02
        self._last_update_time = current_time

        # Read sensors
        self._read_sensors()

        # Check for failsafe conditions
        self._check_failsafe()

        # State-specific updates
        state = self.state_machine.state

        if state == FlightState.IDLE or state == FlightState.ERROR:
            return  # No control in these states

        if state == FlightState.TAKEOFF:
            self._update_takeoff(dt)

        elif state == FlightState.HOVER or state == FlightState.POSITION_HOLD:
            self._update_position_hold(dt)

        elif state == FlightState.FLYING:
            self._update_flying(dt)

        elif state == FlightState.MISSION:
            self._update_mission(dt)

        elif state == FlightState.LANDING:
            self._update_landing(dt)

        elif state == FlightState.RTH:
            self._update_rth(dt)

        # Send RC commands (if armed and not in failsafe)
        if state not in {FlightState.IDLE, FlightState.FAILSAFE, FlightState.ERROR}:
            self._send_rc_commands()

    def _read_sensors(self):
        """Read sensor data from GPS and MSP"""
        if self.simulation:
            return  # Use simulated data

        if self.gps:
            fix = self.gps.get_fix()
            if fix.has_fix:
                self._last_gps_fix = fix
                self.pos_controller.update_position(
                    fix.latitude, fix.longitude, fix.altitude_msl
                )
                self.vel_controller.set_current_velocity(
                    fix.vel_north, fix.vel_east, fix.vel_down
                )
                self.alt_controller.update_altitude_from_gps(
                    fix.altitude_msl, fix.vel_down
                )

        if self.msp:
            try:
                attitude = self.msp.get_attitude()
                self._last_attitude = attitude
                self._current_yaw = attitude.yaw
                self.vel_controller.set_current_yaw(
                    self._current_yaw * 3.14159 / 180.0
                )

                # Also read altitude from FC if available
                altitude = self.msp.get_altitude()
                self.alt_controller.update_altitude_from_fc(
                    altitude.altitude_cm, altitude.vario_cms
                )
            except MSPError as e:
                logger.warning(f"MSP read error: {e}")

    def _check_failsafe(self):
        """Check for failsafe conditions"""
        config = self.config.failsafe

        # GPS loss
        if self._last_gps_fix:
            gps_age_ms = (time.time() - self._last_gps_fix.timestamp) * 1000
            if gps_age_ms > config.gps_timeout_ms:
                if self.state_machine.is_flying:
                    logger.warning(f"GPS timeout: {gps_age_ms:.0f}ms")
                    if config.on_gps_loss == "betaflight":
                        self.state_machine.trigger_failsafe("GPS timeout")

        # State timeout
        if self.state_machine.check_timeout():
            if self.state_machine.state == FlightState.TAKEOFF:
                self.state_machine.transition_to(FlightState.LANDING)
            elif self.state_machine.state == FlightState.LANDING:
                self.state_machine.trigger_failsafe("Landing timeout")

    def _update_takeoff(self, dt: float):
        """Update takeoff sequence"""
        # Set target altitude
        self.alt_controller.set_target_altitude(self._target_altitude)

        # Calculate throttle
        throttle = self.alt_controller.update(dt)

        # Keep level (no position control yet)
        self._rc_channels = [1500, 1500, int(throttle * 1000 + 1000), 1500, 1500, 1500, 1500, 1500]

        # Check if reached altitude
        if self.alt_controller.current_altitude >= self._target_altitude - 0.5:
            logger.info("Takeoff complete")
            self.state_machine.transition_to(FlightState.POSITION_HOLD)

    def _update_position_hold(self, dt: float):
        """Update position hold"""
        # Position → Velocity
        vel_n, vel_e, vel_d = self.pos_controller.update()
        self.vel_controller.set_target_velocity(vel_n, vel_e, vel_d)

        # Velocity → Attitude
        attitude_cmd = self.vel_controller.update(dt)

        # Altitude → Throttle
        throttle = self.alt_controller.update(dt)

        # Convert to RC
        self._rc_channels = list(angles_to_rc(
            attitude_cmd.roll_deg,
            attitude_cmd.pitch_deg,
            throttle,
            0.0,  # No yaw rate for now
            self.config.navigation.max_bank_angle_deg
        )) + [1500, 1500, 1500, 1500]

    def _update_flying(self, dt: float):
        """Update flying to waypoint"""
        # Same as position hold but with waypoint target
        self._update_position_hold(dt)

    def _update_mission(self, dt: float):
        """Update mission execution"""
        if self.waypoint_nav:
            self.waypoint_nav.update()

        self._update_position_hold(dt)

    def _update_landing(self, dt: float):
        """Update landing sequence"""
        # Descend slowly
        self.alt_controller.set_target_climb_rate(-0.5)  # 0.5 m/s descent

        throttle = self.alt_controller.update(dt)

        # Keep level
        self._rc_channels = [1500, 1500, int(throttle * 1000 + 1000), 1500, 1500, 1500, 1500, 1500]

        # Check if landed
        if self.alt_controller.is_landed:
            logger.info("Landing complete")
            self.state_machine.transition_to(FlightState.LANDED)

    def _update_rth(self, dt: float):
        """Update return to home"""
        # Navigate to home position
        self.pos_controller.set_target_gps(
            self.waypoint_nav.home_lat,
            self.waypoint_nav.home_lon,
            self.waypoint_nav.home_alt
        )

        self._update_position_hold(dt)

        # Check if reached home
        if self.pos_controller.is_at_target(radius=3.0):
            logger.info("Reached home, landing")
            self.state_machine.transition_to(FlightState.LANDING)

    def _send_rc_commands(self):
        """Send RC commands to Betaflight"""
        if self.msp:
            try:
                self.msp.set_raw_rc(self._rc_channels)
            except MSPError as e:
                logger.error(f"Failed to send RC: {e}")

    # ==================== Public API ====================

    def arm(self) -> bool:
        """Arm the drone"""
        if self.state_machine.state != FlightState.IDLE:
            return False

        logger.info("Arming...")
        # TODO: Send arm command to Betaflight
        self._armed = True
        return self.state_machine.transition_to(FlightState.ARMED)

    def disarm(self) -> bool:
        """Disarm the drone"""
        if not self.state_machine.is_flying:
            logger.info("Disarming...")
            self._armed = False
            return self.state_machine.transition_to(FlightState.IDLE)
        else:
            logger.warning("Cannot disarm while flying")
            return False

    def takeoff(self, altitude: float = 3.0) -> bool:
        """
        Initiate takeoff

        Args:
            altitude: Target altitude in meters
        """
        if self.state_machine.state != FlightState.ARMED:
            logger.error("Must be armed to takeoff")
            return False

        self._target_altitude = altitude

        # Set home position
        if self._last_gps_fix:
            self.waypoint_nav.set_home(
                self._last_gps_fix.latitude,
                self._last_gps_fix.longitude,
                self._last_gps_fix.altitude_msl
            )
            self.pos_controller.set_reference_from_current(
                self._last_gps_fix.latitude,
                self._last_gps_fix.longitude,
                self._last_gps_fix.altitude_msl
            )

        return self.state_machine.transition_to(FlightState.TAKEOFF)

    def land(self) -> bool:
        """Initiate landing"""
        if not self.state_machine.is_flying:
            return False

        return self.state_machine.transition_to(FlightState.LANDING)

    def goto(self, lat: float, lon: float, alt: float) -> bool:
        """
        Go to GPS position

        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
        """
        if self.state_machine.state not in {
            FlightState.HOVER, FlightState.POSITION_HOLD, FlightState.FLYING
        }:
            return False

        self.pos_controller.set_target_gps(lat, lon, alt)
        self._target_altitude = alt
        self.alt_controller.set_target_altitude(alt)

        return self.state_machine.transition_to(FlightState.FLYING)

    def hold_position(self) -> bool:
        """Hold current position"""
        if not self.state_machine.is_flying:
            return False

        return self.state_machine.transition_to(FlightState.POSITION_HOLD)

    def start_mission(self, mission: Mission) -> bool:
        """
        Start a waypoint mission

        Args:
            mission: Mission to execute
        """
        if self.state_machine.state not in {
            FlightState.HOVER, FlightState.POSITION_HOLD
        }:
            return False

        self.waypoint_nav.load_mission(mission)

        if self.waypoint_nav.start():
            return self.state_machine.transition_to(FlightState.MISSION)

        return False

    def return_to_home(self) -> bool:
        """Return to home position"""
        if not self.state_machine.is_flying:
            return False

        return self.state_machine.transition_to(FlightState.RTH)

    def get_telemetry(self) -> dict:
        """Get current telemetry data"""
        return {
            'state': self.state_machine.get_status(),
            'position': {
                'lat': self._last_gps_fix.latitude if self._last_gps_fix else 0,
                'lon': self._last_gps_fix.longitude if self._last_gps_fix else 0,
                'alt': self.alt_controller.current_altitude,
                'satellites': self._last_gps_fix.num_satellites if self._last_gps_fix else 0
            },
            'attitude': {
                'roll': self._last_attitude.roll if self._last_attitude else 0,
                'pitch': self._last_attitude.pitch if self._last_attitude else 0,
                'yaw': self._current_yaw
            },
            'navigation': self.waypoint_nav.get_status() if self.waypoint_nav else None,
            'armed': self._armed
        }

    def shutdown(self):
        """Shutdown flight controller"""
        logger.info("Shutting down flight controller...")
        self.stop()

        if self.gps:
            self.gps.close()

        if self.msp:
            self.msp.close()

        if self.serial_manager:
            self.serial_manager.close()
