"""
Configuration management for Pi Drone Navigation

All parameters are configurable and can be overridden via:
1. config/default.yaml
2. Environment variables (prefixed with PIDRONE_)
3. Command line arguments
"""

import os
import yaml
from dataclasses import dataclass, field
from typing import Optional
from pathlib import Path


@dataclass
class SerialConfig:
    """Serial connection configuration"""
    port: str = "/dev/ttyAMA0"          # UART on Pi GPIO
    baudrate: int = 115200
    timeout: float = 0.1                 # seconds

    # Alternative USB port
    usb_port: str = "/dev/ttyACM0"
    use_usb: bool = False

    # Auto-detection: scan ports and test MSP response
    auto_detect: bool = True


@dataclass
class GPSConfig:
    """GPS module configuration (u-blox UBX)"""
    port: str = "/dev/ttyAMA1"          # Separate UART for GPS
    baudrate: int = 115200
    update_rate_hz: int = 10            # u-blox M8N/M10 supports up to 10Hz

    # UBX protocol settings
    enable_ubx: bool = True
    enable_nmea: bool = False           # Disable NMEA for efficiency

    # Position filtering
    use_kalman_filter: bool = True
    position_filter_hz: float = 5.0     # Low-pass filter

    # Fallback to MSP GPS (from Betaflight FC) if UBX GPS unavailable at startup
    msp_fallback: bool = True


@dataclass
class NavigationConfig:
    """Navigation controller parameters"""

    # Control loop
    update_rate_hz: float = 50.0        # Main control loop frequency

    # Position controller (P)
    pos_p_gain: float = 1.0             # Position to velocity gain
    max_horizontal_speed_ms: float = 15.0  # m/s (configurable 10-15)
    max_vertical_speed_ms: float = 3.0     # m/s

    # Velocity controller (PID)
    vel_p_gain: float = 2.0
    vel_i_gain: float = 0.5
    vel_d_gain: float = 0.1
    vel_i_max: float = 5.0              # Anti-windup limit

    # Acceleration limits
    max_horizontal_accel_mss: float = 5.0   # m/s^2 (~30 deg lean)
    max_vertical_accel_mss: float = 3.0     # m/s^2
    jerk_limit_msss: float = 1.7            # m/s^3 (from iNav)

    # Altitude controller
    alt_p_gain: float = 0.5
    climb_rate_p_gain: float = 5.0
    hover_throttle: float = 0.5         # To be calibrated
    throttle_margin: float = 0.3        # +/- from hover

    # Attitude limits
    max_bank_angle_deg: float = 30.0    # Configurable
    max_yaw_rate_dps: float = 90.0      # deg/s

    # Waypoint navigation
    waypoint_radius_m: float = 2.0      # Acceptance radius
    waypoint_speed_ms: float = 10.0     # Cruise speed to waypoints

    # Position hold
    position_hold_radius_m: float = 2.0  # Target precision

    # L1 Controller (path following)
    l1_period: float = 20.0             # L1 time constant (seconds, 15-25)
    l1_damping: float = 0.75            # Damping ratio (0.7-0.85)
    xte_i_gain: float = 0.1             # Cross-track error integrator gain

    # Turn anticipation
    enable_turn_anticipation: bool = True
    min_turn_radius_m: float = 5.0      # Minimum turn radius

    # Speed management
    enable_speed_adaptation: bool = True
    turn_speed_reduction: float = 0.5   # Min speed factor in sharp turns (0.5 = 50%)
    max_decel_mss: float = 2.0          # Max deceleration for speed reduction

    # Altitude ramping
    enable_altitude_ramp: bool = True

    # Heading fusion (GPS course-over-ground + gyro)
    enable_heading_fusion: bool = True
    heading_gps_weight: float = 0.1     # GPS heading weight in fusion
    heading_min_speed_ms: float = 3.0   # Min speed for GPS heading validity


@dataclass
class AltitudeConfig:
    """Altitude source configuration"""

    # Priority: baro_fc > baro_pi > gps
    use_baro_from_fc: bool = True       # Read MSP_ALTITUDE
    use_baro_on_pi: bool = False        # BMP280/MS5611 on Pi
    use_gps_altitude: bool = True       # Fallback

    # Fusion parameters
    baro_weight: float = 0.8            # Weight vs GPS in fusion
    altitude_filter_hz: float = 2.0     # Low-pass filter


@dataclass
class FailsafeConfig:
    """Failsafe behavior configuration"""

    # GPS loss
    gps_timeout_ms: int = 3000          # Time before GPS considered lost
    on_gps_loss: str = "betaflight"     # "betaflight", "land", "hover"

    # Communication loss
    msp_timeout_ms: int = 500           # MSP response timeout
    on_msp_loss: str = "disarm"         # "disarm", "land"

    # Low battery (read from Betaflight)
    low_battery_voltage: float = 3.5    # Per cell
    critical_battery_voltage: float = 3.3
    on_low_battery: str = "rth"         # "rth", "land", "warn"


@dataclass
class InterfaceConfig:
    """Interface configuration"""

    # REST API
    rest_enabled: bool = True
    rest_host: str = "0.0.0.0"
    rest_port: int = 8080

    # WebSocket
    websocket_enabled: bool = True
    websocket_port: int = 8081

    # MAVLink
    mavlink_enabled: bool = True
    mavlink_port: int = 14550           # UDP port for QGroundControl
    mavlink_system_id: int = 1

    # Logging
    log_file: str = "flight.log"
    log_level: str = "INFO"


@dataclass
class SimulationConfig:
    """Simulation/SITL configuration"""

    enabled: bool = False

    # Betaflight SITL
    sitl_host: str = "127.0.0.1"
    sitl_msp_port: int = 5761           # TCP port for MSP

    # GPS simulation
    simulate_gps: bool = True
    gps_start_lat: float = 48.8566      # Paris
    gps_start_lon: float = 2.3522
    gps_start_alt: float = 100.0        # meters


@dataclass
class TakeoffConfig:
    """
    Takeoff configuration - iNav-style velocity PID

    Adapted for MSP control frequency (25-50Hz vs iNav's 100-500Hz):
    - PID gains halved from iNav defaults
    - Filter cutoff reduced to 2Hz (iNav uses 4Hz)
    """

    # Target
    default_altitude_m: float = 3.0
    target_climb_rate_ms: float = 0.2  # m/s target climb rate during takeoff (20cm/s for testing)

    # Spinup phase
    spinup_time_ms: int = 500           # Motor spinup duration
    spinup_throttle: float = 0.15       # Throttle at end of spinup

    # Velocity PID (adapted for 50Hz MSP)
    vel_kp: float = 0.15                # Half of iNav's 0.30
    vel_ki: float = 0.05                # Half of iNav's 0.10
    vel_kd: float = 0.02                # Half of iNav's 0.05
    vel_i_max: float = 0.2              # Anti-windup limit
    velocity_filter_hz: float = 2.0     # Low-pass filter (iNav uses 4Hz)

    # Liftoff detection (iNav-style: throttle + gyro)
    liftoff_gyro_threshold_dps: float = 7.0   # Gyro magnitude threshold (deg/s)
    liftoff_throttle_margin: float = 0.05     # Throttle must be > hover + margin
    liftoff_confirm_time_s: float = 0.2       # 200ms confirmation

    # Safety limits
    max_throttle: float = 0.75          # Never exceed this during takeoff
    min_throttle: float = 0.1           # Never go below this
    max_tilt_deg: float = 25.0          # Abort if tilted more
    timeout_s: float = 10.0             # Abort if no liftoff after this
    altitude_tolerance_m: float = 0.5   # Close enough to target

    # Pre-flight checks
    preflight_enabled: bool = True
    min_gps_satellites: int = 5
    max_hdop: float = 3.0
    attitude_check_max_tilt_deg: float = 10.0  # Must be level before takeoff


@dataclass
class HoverLearnConfig:
    """Hover throttle learning configuration (ArduPilot MOT_THST_HOVER style)"""

    enabled: bool = True
    time_constant_sec: float = 2.0      # EMA filter time constant
    min_hover_throttle: float = 0.2
    max_hover_throttle: float = 0.8

    # Learning conditions
    min_altitude_m: float = 1.0         # Must be above this to learn
    max_climb_rate_ms: float = 0.5      # Must be hovering (low vertical speed)
    max_horizontal_speed_ms: float = 1.0  # Must be near stationary

    # Persistence
    save_on_disarm: bool = True
    save_file: str = "hover_throttle.json"


@dataclass
class Config:
    """Main configuration container"""

    serial: SerialConfig = field(default_factory=SerialConfig)
    gps: GPSConfig = field(default_factory=GPSConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    altitude: AltitudeConfig = field(default_factory=AltitudeConfig)
    failsafe: FailsafeConfig = field(default_factory=FailsafeConfig)
    interface: InterfaceConfig = field(default_factory=InterfaceConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    takeoff: TakeoffConfig = field(default_factory=TakeoffConfig)
    hover_learn: HoverLearnConfig = field(default_factory=HoverLearnConfig)

    @classmethod
    def load(cls, config_path: Optional[str] = None) -> "Config":
        """Load configuration from YAML file and environment"""
        config = cls()

        # Load from file if exists
        if config_path is None:
            config_path = Path(__file__).parent.parent / "config" / "default.yaml"

        if Path(config_path).exists():
            with open(config_path, 'r') as f:
                yaml_config = yaml.safe_load(f)
                if yaml_config:
                    config._update_from_dict(yaml_config)

        # Override from environment variables
        config._update_from_env()

        return config

    def _update_from_dict(self, d: dict):
        """Update config from dictionary (e.g., YAML)"""
        for section_name, section_data in d.items():
            if hasattr(self, section_name) and isinstance(section_data, dict):
                section = getattr(self, section_name)
                for key, value in section_data.items():
                    if hasattr(section, key):
                        setattr(section, key, value)

    def _update_from_env(self):
        """Override config from environment variables"""
        prefix = "PIDRONE_"
        for key, value in os.environ.items():
            if key.startswith(prefix):
                # Parse PIDRONE_SECTION_KEY format
                parts = key[len(prefix):].lower().split("_", 1)
                if len(parts) == 2:
                    section_name, param_name = parts
                    if hasattr(self, section_name):
                        section = getattr(self, section_name)
                        if hasattr(section, param_name):
                            # Type conversion
                            current_value = getattr(section, param_name)
                            if isinstance(current_value, bool):
                                setattr(section, param_name, value.lower() in ('true', '1', 'yes'))
                            elif isinstance(current_value, int):
                                setattr(section, param_name, int(value))
                            elif isinstance(current_value, float):
                                setattr(section, param_name, float(value))
                            else:
                                setattr(section, param_name, value)

    def save(self, config_path: str):
        """Save current configuration to YAML file"""
        data = {}
        for section_name in ['serial', 'gps', 'navigation', 'altitude',
                            'failsafe', 'interface', 'simulation',
                            'takeoff', 'hover_learn']:
            section = getattr(self, section_name)
            data[section_name] = {k: v for k, v in section.__dict__.items()}

        with open(config_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)


# Global config instance
_config: Optional[Config] = None


def get_config() -> Config:
    """Get the global configuration instance"""
    global _config
    if _config is None:
        _config = Config.load()
    return _config


def set_config(config: Config):
    """Set the global configuration instance"""
    global _config
    _config = config
