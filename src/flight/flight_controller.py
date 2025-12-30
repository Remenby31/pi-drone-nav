"""
Main Flight Controller

Integrates all subsystems and runs the main control loop.
"""

import time
import threading
from typing import Optional, Tuple
import logging

from .state_machine import FlightStateMachine, FlightState
from .preflight_checks import PreflightChecker, PreflightResult
from ..navigation.takeoff_controller import TakeoffController, TakeoffState
from .hover_throttle_learner import HoverThrottleLearner
from ..drivers.msp import MSPClient, MSPError
from ..drivers.gps_ubx import GPSDriver, GPSFix
from ..drivers.serial_manager import SerialManager
from ..navigation.position_controller import PositionControllerGPS
from ..navigation.velocity_controller import VelocityController, angles_to_rc
from ..navigation.altitude_controller import AltitudeController
from ..navigation.waypoint_navigator import WaypointNavigator, Mission
from ..navigation.heading_estimator import HeadingEstimator
from ..mission import MissionExecutor, ExecutorState
from ..config import get_config

import math

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
        self.heading_estimator = HeadingEstimator()
        self.waypoint_nav: Optional[WaypointNavigator] = None

        # Mission executor (v1.0 action-based missions)
        self.mission_executor: Optional[MissionExecutor] = None

        # Control loop
        self._running = False
        self._control_thread: Optional[threading.Thread] = None
        self._update_rate = self.config.navigation.update_rate_hz
        self._last_update_time = 0.0

        # State
        self._armed = False
        self._target_altitude = 0.0
        self._current_yaw = 0.0

        # GPS source tracking
        self._use_msp_gps = False  # True if using GPS via MSP (fallback)

        # RC output - initialize with safe disarmed values
        # [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
        self._rc_channels = [1500, 1500, 885, 1500, 1000, 1500, 1500, 1500]

        # Telemetry
        self._last_gps_fix: Optional[GPSFix] = None
        self._last_attitude = None

        # Takeoff system (initialized in initialize())
        self.takeoff_controller: Optional[TakeoffController] = None
        self.preflight_checker: Optional[PreflightChecker] = None
        self.hover_learner: Optional[HoverThrottleLearner] = None

        # Preflight state
        self._preflight_result: Optional[PreflightResult] = None

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

                # Auto-detect or use configured ports
                if self.config.serial.auto_detect:
                    logger.info("Auto-detecting Betaflight port...")
                    if not self.serial_manager.auto_detect_and_connect():
                        logger.warning("Auto-detect failed, trying configured ports...")
                        if not self.serial_manager.connect():
                            logger.error("Failed to connect to flight controller")
                            return False
                elif not self.serial_manager.connect():
                    logger.error("Failed to connect to flight controller")
                    return False

                # Initialize MSP client
                self.msp = MSPClient(self.serial_manager, timeout=0.1)

                if not self.msp.connect():
                    logger.error("MSP connection failed")
                    return False

                # Initialize GPS - try UBX first, fallback to MSP if enabled
                self._init_gps()

            else:
                # Simulation mode
                logger.info("Running in simulation mode")
                self._init_simulation()

            # Initialize waypoint navigator
            self.waypoint_nav = WaypointNavigator(self.pos_controller)

            # Initialize takeoff system
            self.preflight_checker = PreflightChecker(self.config.takeoff)
            self.hover_learner = HoverThrottleLearner(self.config.hover_learn)

            # Initialize iNav-style takeoff controller
            initial_hover = self.hover_learner.hover_throttle if self.hover_learner else 0.5
            self.takeoff_controller = TakeoffController(
                self.config.takeoff, hover_throttle=initial_hover
            )

            # Setup takeoff callbacks
            self.takeoff_controller.on_complete(self._on_takeoff_complete)
            self.takeoff_controller.on_abort(self._on_takeoff_abort)

            logger.info("Flight controller initialized successfully")
            return True

        except Exception as e:
            logger.error(f"Initialization failed: {e}")
            return False

    def _init_gps(self):
        """
        Initialize GPS source with fallback logic.

        Priority:
        1. UBX GPS on dedicated serial port (if available)
        2. MSP GPS from Betaflight FC (if msp_fallback enabled)
        """
        import serial

        gps_config = self.config.gps

        # Try to initialize UBX GPS
        try:
            gps_serial = serial.Serial(
                port=gps_config.port,
                baudrate=gps_config.baudrate,
                timeout=0.1
            )
            self.gps = GPSDriver(gps_serial, update_rate_hz=gps_config.update_rate_hz)

            if self.gps.configure():
                self.gps.start()
                logger.info(f"UBX GPS initialized on {gps_config.port}")
                self._use_msp_gps = False
                return
            else:
                logger.warning("UBX GPS configuration failed")
                self.gps.close()
                self.gps = None

        except serial.SerialException as e:
            logger.warning(f"UBX GPS not available on {gps_config.port}: {e}")
            self.gps = None

        # Fallback to MSP GPS if enabled
        if gps_config.msp_fallback and self.msp:
            try:
                # Test if Betaflight has GPS data
                gps_fix = self.msp.get_gps_as_fix()
                if gps_fix is not None:
                    self._use_msp_gps = True
                    logger.info("Using MSP GPS from Betaflight (fallback mode)")
                    logger.warning("MSP GPS: vel_down not available, vertical velocity will be 0")
                    return
                else:
                    logger.warning("MSP GPS: No GPS data from Betaflight")
            except Exception as e:
                logger.warning(f"MSP GPS fallback failed: {e}")

        logger.error("No GPS source available!")

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
            logger.info("Starting iNav-style landing sequence")
            self._target_altitude = 0.0
            # Initialize iNav-style landing
            self.alt_controller.start_landing()

        @self.state_machine.on_enter(FlightState.POSITION_HOLD)
        def on_position_hold():
            logger.info("Position hold activated")
            self.pos_controller.controller.capture_hold_position()

        @self.state_machine.on_enter(FlightState.FAILSAFE)
        def on_failsafe():
            logger.warning("FAILSAFE - Returning control to Betaflight")
            # Stop sending RC commands to let Betaflight GPS Rescue take over

    def _on_takeoff_complete(self):
        """Called when takeoff completes successfully"""
        logger.info("Takeoff complete, transitioning to position hold")

        # Update altitude controller with learned hover throttle if available
        if self.hover_learner and self.takeoff_controller:
            throttle_at_liftoff = self.takeoff_controller.throttle_at_liftoff
            if throttle_at_liftoff > 0:
                # Use liftoff throttle as initial estimate for hover learning
                self.hover_learner.reset(throttle_at_liftoff)
                self.alt_controller.calibrate_hover_throttle(throttle_at_liftoff)
                logger.info(f"Hover throttle from takeoff: {throttle_at_liftoff:.2f}")

        self.state_machine.transition_to(FlightState.POSITION_HOLD)

    def _on_takeoff_abort(self, reason: str):
        """Called when takeoff is aborted"""
        logger.warning(f"Takeoff aborted: {reason}")

        # Transition to landing or idle depending on altitude
        if self.alt_controller.current_altitude > 0.5:
            self.state_machine.transition_to(FlightState.LANDING)
        else:
            self._armed = False
            self.state_machine.transition_to(FlightState.IDLE)

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

        if state == FlightState.ERROR:
            return  # No control in error state

        # In IDLE, still send RC to prevent RXLOSS (but with disarm values)
        if state == FlightState.IDLE:
            self._send_rc_commands()  # Keep RX alive
            return

        # Handle mission executor for ARMED state (to trigger takeoff)
        if state == FlightState.ARMED and self.mission_executor.state == ExecutorState.RUNNING:
            self._update_mission(dt)
            # Don't return - need to send RC commands below

        elif state == FlightState.TAKEOFF:
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

        # Read GPS - either from UBX driver or MSP fallback
        fix = None
        if self._use_msp_gps and self.msp:
            # Get GPS from Betaflight via MSP
            fix = self.msp.get_gps_as_fix()
        elif self.gps:
            # Get GPS from UBX driver
            fix = self.gps.get_fix()

        if fix and fix.has_fix:
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

            # Update waypoint navigator with current position/velocity
            if self.waypoint_nav:
                self.waypoint_nav.update_position(
                    fix.latitude, fix.longitude, fix.altitude_msl,
                    fix.vel_north, fix.vel_east
                )

        if self.msp:
            try:
                attitude = self.msp.get_attitude()
                self._last_attitude = attitude

                # Fuse gyro heading with GPS course-over-ground
                if fix and fix.has_fix:
                    ground_speed = math.sqrt(fix.vel_north**2 + fix.vel_east**2)
                    gps_heading_rad = math.radians(fix.heading) if hasattr(fix, 'heading') else 0.0

                    # Get fused heading
                    fused_yaw_rad = self.heading_estimator.update(
                        gyro_yaw_rad=math.radians(attitude.yaw),
                        gps_cog_rad=gps_heading_rad,
                        ground_speed=ground_speed
                    )
                    self._current_yaw = math.degrees(fused_yaw_rad)
                else:
                    # No GPS, use gyro only
                    self._current_yaw = attitude.yaw
                    fused_yaw_rad = math.radians(attitude.yaw)

                self.vel_controller.set_current_yaw(fused_yaw_rad)

                # Also read altitude from FC if available
                altitude = self.msp.get_altitude()
                self.alt_controller.update_altitude_from_fc(
                    altitude.altitude_cm, altitude.vario_cms
                )

                # Read IMU for landing touchdown detection
                if self.alt_controller.is_landing:
                    imu = self.msp.get_raw_imu()
                    self.alt_controller.update_accelerometer(
                        imu.acc_x, imu.acc_y, imu.acc_z
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
        """
        Update takeoff sequence using iNav-style TakeoffController

        Reads gyro data from MSP_RAW_IMU for liftoff detection.
        """
        if not self.takeoff_controller:
            # Fallback to simple takeoff if controller not available
            self.alt_controller.set_target_altitude(self._target_altitude)
            throttle = self.alt_controller.update(dt)
            self._rc_channels = [1500, 1500, int(throttle * 1000 + 1000), 1500, 1500, 1500, 1500, 1500]
            if self.alt_controller.current_altitude >= self._target_altitude - 0.5:
                self.state_machine.transition_to(FlightState.POSITION_HOLD)
            return

        # Get current sensor data
        altitude = self.alt_controller.current_altitude
        climb_rate = self.alt_controller.current_climb_rate
        roll = self._last_attitude.roll if self._last_attitude else 0
        pitch = self._last_attitude.pitch if self._last_attitude else 0

        # Read gyro for iNav-style liftoff detection
        gyro_x, gyro_y, gyro_z = 0, 0, 0
        if self.msp:
            try:
                imu = self.msp.get_raw_imu()
                gyro_x, gyro_y, gyro_z = imu.gyro_x, imu.gyro_y, imu.gyro_z
            except MSPError as e:
                logger.warning(f"Failed to read IMU for takeoff: {e}")

        # Update iNav-style takeoff controller
        throttle = self.takeoff_controller.update(
            dt=dt,
            altitude_m=altitude,
            climb_rate_ms=climb_rate,
            gyro_x=gyro_x,
            gyro_y=gyro_y,
            gyro_z=gyro_z,
            roll_deg=roll,
            pitch_deg=pitch
        )

        # Keep level (no position control during takeoff)
        # AUX channels: AUX1=armed (1800), AUX2=mode (1800)
        self._rc_channels = [
            1500,                           # Roll - center
            1500,                           # Pitch - center
            int(throttle * 1000 + 1000),    # Throttle
            1500,                           # Yaw - center
            1800,                           # AUX1 - armed
            1800,                           # AUX2 - flight mode
            1500,                           # AUX3
            1500                            # AUX4
        ]

        # Phase transitions are handled by callbacks

    def _update_position_hold(self, dt: float):
        """Update position hold with hover throttle learning"""
        # Position → Velocity
        vel_n, vel_e, vel_d = self.pos_controller.update()
        self.vel_controller.set_target_velocity(vel_n, vel_e, vel_d)

        # Velocity → Attitude
        attitude_cmd = self.vel_controller.update(dt)

        # Altitude → Throttle
        throttle = self.alt_controller.update(dt)

        # Update hover throttle learner
        if self.hover_learner and self._last_gps_fix:
            # Calculate horizontal speed
            horizontal_speed = 0.0
            if hasattr(self._last_gps_fix, 'ground_speed'):
                horizontal_speed = self._last_gps_fix.ground_speed
            elif hasattr(self._last_gps_fix, 'vel_north') and hasattr(self._last_gps_fix, 'vel_east'):
                horizontal_speed = (self._last_gps_fix.vel_north**2 +
                                    self._last_gps_fix.vel_east**2) ** 0.5

            learned = self.hover_learner.update(
                dt=dt,
                throttle=throttle,
                altitude_m=self.alt_controller.current_altitude,
                climb_rate_ms=self.alt_controller.current_climb_rate,
                horizontal_speed_ms=horizontal_speed,
                is_position_hold=True
            )

            if learned:
                # Update altitude controller with learned value
                self.alt_controller.hover_throttle = self.hover_learner.hover_throttle

        # Convert to RC
        # AUX channels: AUX1=armed (1800), AUX2=mode (1800)
        self._rc_channels = list(angles_to_rc(
            attitude_cmd.roll_deg,
            attitude_cmd.pitch_deg,
            throttle,
            0.0,  # No yaw rate for now
            self.config.navigation.max_bank_angle_deg
        )) + [1800, 1800, 1500, 1500]

    def _update_flying(self, dt: float):
        """Update flying to waypoint"""
        # Same as position hold but with waypoint target
        self._update_position_hold(dt)

    def _update_mission(self, dt: float):
        """Update mission execution"""
        # Use new mission executor if available
        if self.mission_executor and self.mission_executor.state == ExecutorState.RUNNING:
            self._update_mission_v2(dt)
            return

        # Fallback to legacy waypoint navigator
        if not self.waypoint_nav:
            self._update_position_hold(dt)
            return

        # Get navigation output from waypoint navigator
        velocity_target, altitude_target, mission_active = self.waypoint_nav.update(dt)

        if not mission_active:
            # Mission completed or aborted
            if self.waypoint_nav.state.name == 'COMPLETED':
                self.state_machine.transition_to(FlightState.POSITION_HOLD)
            return

        # Use L1 velocity target directly
        self.vel_controller.set_target_velocity(
            velocity_target.x, velocity_target.y, 0.0
        )

        # Use ramped altitude from waypoint navigator
        self.alt_controller.set_target_altitude(altitude_target)

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

    def _update_mission_v2(self, dt: float):
        """Update mission execution using v1.0 action-based executor"""
        if not self.mission_executor:
            return

        # Update executor with current position
        if self._last_gps_fix:
            self.mission_executor.update_position(
                self._last_gps_fix.latitude,
                self._last_gps_fix.longitude,
                self.alt_controller.current_altitude,
                self._current_yaw
            )

        # Get executor output
        output = self.mission_executor.update(dt)

        # Handle state transitions based on output
        if output.should_takeoff:
            if self.state_machine.state != FlightState.TAKEOFF:
                self._target_altitude = output.target_alt or 10.0
                if self.takeoff_controller:
                    self.takeoff_controller.start(self._target_altitude)
                self.state_machine.transition_to(FlightState.TAKEOFF)
            self._update_takeoff(dt)
            return

        if output.should_land:
            if self.state_machine.state != FlightState.LANDING:
                self.state_machine.transition_to(FlightState.LANDING)
            self._update_landing(dt)
            return

        if output.should_rth:
            if self.state_machine.state != FlightState.RTH:
                self.state_machine.transition_to(FlightState.RTH)
            self._update_rth(dt)
            return

        if output.should_hold:
            # Position hold (for hover, delay, orient, photo)
            self._update_position_hold(dt)

            # Handle heading change for orient action
            if output.target_heading is not None:
                # TODO: Add yaw control
                pass
            return

        # Navigation to target (goto action)
        if output.target_lat is not None and output.target_lon is not None:
            target_alt = output.target_alt or self.alt_controller.current_altitude

            self.pos_controller.set_target_gps(
                output.target_lat,
                output.target_lon,
                target_alt
            )
            self.alt_controller.set_target_altitude(target_alt)

            # Position → Velocity
            vel_n, vel_e, vel_d = self.pos_controller.update()

            # Apply speed limit if specified
            if output.target_speed:
                speed = math.sqrt(vel_n**2 + vel_e**2)
                if speed > output.target_speed:
                    scale = output.target_speed / speed
                    vel_n *= scale
                    vel_e *= scale

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
                0.0,
                self.config.navigation.max_bank_angle_deg
            )) + [1800, 1800, 1500, 1500]

        # Check if mission completed
        if self.mission_executor.state == ExecutorState.COMPLETED:
            logger.info("Mission v2 completed")
            self.state_machine.transition_to(FlightState.POSITION_HOLD)
        elif self.mission_executor.state == ExecutorState.STOPPED:
            logger.info("Mission v2 stopped")
            self.state_machine.transition_to(FlightState.POSITION_HOLD)

    def _update_landing(self, dt: float):
        """
        Update landing sequence (iNav-style 3-phase landing)

        Phases:
        - PHASE_HIGH (>5m): Fast descent at 1.5 m/s
        - PHASE_MID (1-5m): Medium descent at 0.7 m/s
        - PHASE_FINAL (<1m): Slow descent at 0.3 m/s + touchdown detection
        """
        from ..navigation.altitude_controller import LandingPhase

        # Get variable descent rate based on altitude phase
        descent_rate = self.alt_controller.get_landing_descent_rate()
        self.alt_controller.set_target_climb_rate(descent_rate)

        # Position hold during landing (maintain horizontal position)
        vel_n, vel_e, vel_d = self.pos_controller.update()
        self.vel_controller.set_target_velocity(vel_n, vel_e, 0.0)  # No vertical velocity from pos ctrl

        # Velocity → Attitude (for position hold)
        attitude_cmd = self.vel_controller.update(dt)

        # Altitude → Throttle
        throttle = self.alt_controller.update(dt)

        # Convert to RC with position hold corrections
        self._rc_channels = list(angles_to_rc(
            attitude_cmd.roll_deg,
            attitude_cmd.pitch_deg,
            throttle,
            0.0,  # No yaw rate
            self.config.navigation.max_bank_angle_deg
        )) + [1800, 1800, 1500, 1500]  # Keep armed

        # Log phase transitions
        phase = self.alt_controller.landing_phase
        height_agl = self.alt_controller.get_height_agl()
        if phase != getattr(self, '_last_landing_phase', None):
            logger.info(f"Landing phase: {phase.name} at {height_agl:.1f}m AGL, descent: {-descent_rate:.2f} m/s")
            self._last_landing_phase = phase

        # Check for touchdown (accelerometer + altitude)
        if self.alt_controller.check_touchdown():
            logger.info("Touchdown detected!")

        # Check if landed and ready to disarm
        if self.alt_controller.is_landed:
            logger.info("Landing confirmed")
            self.state_machine.transition_to(FlightState.LANDED)

            # Auto-disarm after delay
            if self.alt_controller.should_disarm():
                logger.info("Auto-disarm triggered")
                self._armed = False
                self.alt_controller.abort_landing()
                self.state_machine.transition_to(FlightState.IDLE)

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
        """Arm the drone with pre-flight checks"""
        if self.state_machine.state != FlightState.IDLE:
            return False

        # Run pre-flight checks if enabled
        if self.config.takeoff.preflight_enabled and self.preflight_checker:
            logger.info("Running pre-flight checks...")
            self.state_machine.transition_to(FlightState.PREFLIGHT_CHECK)

            self._preflight_result = self.preflight_checker.run_checks(
                msp=self.msp,
                gps_fix=self._last_gps_fix,
                attitude=self._last_attitude
            )

            if not self._preflight_result.passed:
                logger.error(f"Pre-flight checks failed: {self._preflight_result.blocking_failures}")
                self.state_machine.transition_to(FlightState.IDLE)
                return False

            # Log warnings
            for warning in self._preflight_result.warnings:
                logger.warning(f"Pre-flight warning: {warning}")

        logger.info("Arming...")
        self._armed = True

        # Send arm command via MSP: AUX1 (index 4) = 1800
        # RC format: [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
        # Throttle must be ≤1050 to arm (min_check), use 885 (rx_min_usec)
        self._rc_channels = [1500, 1500, 885, 1500, 1800, 1800, 1500, 1500]

        if self.msp:
            try:
                # Send arm command immediately
                self.msp.set_raw_rc(self._rc_channels)
                logger.info("ARM command sent: AUX1=1800")
            except MSPError as e:
                logger.error(f"Failed to send arm command: {e}")
                self._armed = False
                return False

        return self.state_machine.transition_to(FlightState.ARMED)

    def disarm(self) -> bool:
        """Disarm the drone"""
        if not self.state_machine.is_flying:
            logger.info("Disarming...")
            self._armed = False

            # Send disarm command via MSP: AUX1 (index 4) = 1000
            self._rc_channels = [1500, 1500, 885, 1500, 1000, 1800, 1500, 1500]

            if self.msp:
                try:
                    self.msp.set_raw_rc(self._rc_channels)
                    logger.info("DISARM command sent: AUX1=1000")
                except MSPError as e:
                    logger.error(f"Failed to send disarm command: {e}")

            return self.state_machine.transition_to(FlightState.IDLE)
        else:
            logger.warning("Cannot disarm while flying")
            return False

    def takeoff(self, altitude: float = None) -> bool:
        """
        Initiate takeoff with improved sequence

        Args:
            altitude: Target altitude in meters (default from config)
        """
        if self.state_machine.state != FlightState.ARMED:
            logger.error("Must be armed to takeoff")
            return False

        # Use config default if not specified
        if altitude is None:
            altitude = self.config.takeoff.default_altitude_m

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
            # Set ground reference for altitude controller
            self.alt_controller.set_ground_reference(
                self.alt_controller.current_altitude
            )

        # Start iNav-style takeoff controller
        if self.takeoff_controller:
            # Update hover throttle from learner
            if self.hover_learner:
                self.takeoff_controller.set_hover_throttle(self.hover_learner.hover_throttle)

            ground_alt = self.alt_controller.current_altitude
            self.takeoff_controller.start(altitude, ground_altitude_m=ground_alt)

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
        Start a waypoint mission (legacy API)

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

    def set_mission_executor(self, executor: MissionExecutor):
        """
        Set the mission executor for v1.0 action-based missions

        Args:
            executor: MissionExecutor instance from REST API
        """
        self.mission_executor = executor

        # Setup callbacks
        executor.on_mission_complete(self._on_mission_v2_complete)
        executor.on_disarm_request(self._on_mission_v2_disarm)

    def start_mission_v2(self) -> bool:
        """
        Start the loaded v1.0 mission

        Returns:
            True if mission started successfully
        """
        if not self.mission_executor:
            logger.error("No mission executor configured")
            return False

        if self.mission_executor.state != ExecutorState.READY:
            logger.error(f"Mission executor not ready: {self.mission_executor.state.name}")
            return False

        # Set home position
        if self._last_gps_fix:
            self.mission_executor.set_home(
                self._last_gps_fix.latitude,
                self._last_gps_fix.longitude,
                self.alt_controller.current_altitude
            )

        # Start executor
        if self.mission_executor.start():
            return self.state_machine.transition_to(FlightState.MISSION)

        return False

    def _on_mission_v2_complete(self):
        """Callback when v1.0 mission completes"""
        logger.info("Mission v2 completed via callback")
        self.state_machine.transition_to(FlightState.LANDED)

    def _on_mission_v2_disarm(self):
        """Callback to disarm after mission completes"""
        logger.info("Mission complete, disarming")
        self.disarm()

    def return_to_home(self) -> bool:
        """Return to home position"""
        if not self.state_machine.is_flying:
            return False

        return self.state_machine.transition_to(FlightState.RTH)

    def get_telemetry(self) -> dict:
        """Get current telemetry data"""
        telemetry = {
            'state': self.state_machine.get_status(),
            'position': {
                'lat': self._last_gps_fix.latitude if self._last_gps_fix else 0,
                'lon': self._last_gps_fix.longitude if self._last_gps_fix else 0,
                'alt': self.alt_controller.current_altitude,
                'satellites': self._last_gps_fix.num_satellites if self._last_gps_fix else 0,
                'gps_source': 'msp' if self._use_msp_gps else 'ubx'
            },
            'attitude': {
                'roll': self._last_attitude.roll if self._last_attitude else 0,
                'pitch': self._last_attitude.pitch if self._last_attitude else 0,
                'yaw': self._current_yaw
            },
            'navigation': self.waypoint_nav.get_status() if self.waypoint_nav else None,
            'armed': self._armed,
            'hover_throttle': self.alt_controller.hover_throttle
        }

        # Add mission executor status (v1.0 missions)
        if self.mission_executor:
            telemetry['mission'] = self.mission_executor.get_status_dict()

        # Add takeoff status if active
        if self.takeoff_controller and self.takeoff_controller.is_active:
            telemetry['takeoff'] = self.takeoff_controller.get_status()

        # Add hover learner status
        if self.hover_learner:
            telemetry['hover_learner'] = self.hover_learner.get_status()

        # Add preflight result if available
        if self._preflight_result:
            telemetry['preflight'] = {
                'passed': self._preflight_result.passed,
                'checks': [c.name for c in self._preflight_result.checks],
                'failures': self._preflight_result.blocking_failures,
                'warnings': self._preflight_result.warnings
            }

        # Add landing status if active
        if self.alt_controller.is_landing:
            telemetry['landing'] = {
                'phase': self.alt_controller.landing_phase.name,
                'height_agl': self.alt_controller.get_height_agl(),
                'descent_rate': self.alt_controller.get_landing_descent_rate(),
                'touchdown': self.alt_controller.touchdown_confirmed,
            }

        return telemetry

    def shutdown(self):
        """Shutdown flight controller"""
        logger.info("Shutting down flight controller...")

        # Save learned hover throttle
        if self.hover_learner:
            self.hover_learner.save()

        self.stop()

        if self.gps:
            self.gps.close()

        if self.msp:
            self.msp.close()

        if self.serial_manager:
            self.serial_manager.close()
