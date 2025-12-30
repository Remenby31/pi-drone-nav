"""
Logging configuration
"""

import logging
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional


def setup_logging(level: int = logging.INFO,
                  log_file: Optional[str] = None,
                  log_format: Optional[str] = None):
    """
    Setup logging configuration

    Args:
        level: Logging level (DEBUG, INFO, WARNING, ERROR)
        log_file: Optional path to log file
        log_format: Optional custom format string
    """
    if log_format is None:
        log_format = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"

    # Create formatter
    formatter = logging.Formatter(log_format, datefmt="%Y-%m-%d %H:%M:%S")

    # Get root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(level)

    # Clear existing handlers
    root_logger.handlers.clear()

    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)

    # File handler (if specified)
    if log_file:
        # Create log directory if needed
        log_path = Path(log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)

        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        root_logger.addHandler(file_handler)

    # Reduce verbosity of some libraries
    logging.getLogger("urllib3").setLevel(logging.WARNING)
    logging.getLogger("asyncio").setLevel(logging.WARNING)


class FlightDataLogger:
    """
    Complete flight data logger for post-flight analysis.

    Logs all flight data at 50Hz in CSV format including:
    - State, position, attitude, velocity
    - Altitude controller (target, current, climb rate)
    - Takeoff/Landing phases
    - PID terms (P, I, D for velocity controllers)
    - IMU raw data (accelerometer, gyroscope)
    - Motor outputs and RC channels
    - Battery status
    """

    # CSV column definitions
    COLUMNS = [
        # Time & State
        "time_s", "loop_idx", "flight_state", "mission_action",
        # GPS Position
        "lat", "lon", "alt_msl", "gps_sats", "gps_hdop",
        # Velocity (m/s)
        "vel_n", "vel_e", "vel_d", "ground_speed",
        # Attitude (degrees)
        "roll", "pitch", "yaw",
        # Altitude control
        "alt_baro", "alt_target", "climb_rate", "climb_rate_target", "height_agl",
        # Throttle & Commands
        "throttle", "roll_cmd", "pitch_cmd", "yaw_cmd",
        # Takeoff state
        "takeoff_state", "takeoff_liftoff", "takeoff_throttle",
        # Landing state
        "landing_phase", "landing_descent_rate", "touchdown",
        # Hover learning
        "hover_throttle",
        # PID velocity X (north)
        "pid_vx_p", "pid_vx_i", "pid_vx_d", "pid_vx_out",
        # PID velocity Y (east)
        "pid_vy_p", "pid_vy_i", "pid_vy_d", "pid_vy_out",
        # PID altitude
        "pid_alt_error", "pid_alt_integral",
        # IMU raw
        "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z",
        # Motors (PWM 1000-2000)
        "motor1", "motor2", "motor3", "motor4",
        # RC channels
        "rc_roll", "rc_pitch", "rc_throttle", "rc_yaw", "rc_aux1", "rc_aux2",
        # Battery
        "vbat", "current_a",
    ]

    def __init__(self, log_dir: str = None):
        """
        Initialize flight data logger.

        Args:
            log_dir: Directory for flight logs (default: ~/.pidrone/logs)
        """
        if log_dir is None:
            log_dir = Path.home() / ".pidrone" / "logs"
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.log_file: Optional[Path] = None
        self._file = None
        self._start_time: float = 0.0
        self._loop_count: int = 0
        self._is_logging: bool = False

    @property
    def is_logging(self) -> bool:
        """Check if currently logging."""
        return self._is_logging and self._file is not None

    def start(self, mission_name: str = None) -> Path:
        """
        Start a new flight log.

        Args:
            mission_name: Optional mission name for the filename

        Returns:
            Path to the log file
        """
        import time

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if mission_name:
            safe_name = "".join(c if c.isalnum() or c in "-_" else "_" for c in mission_name)
            filename = f"flight_{timestamp}_{safe_name}.csv"
        else:
            filename = f"flight_{timestamp}.csv"

        self.log_file = self.log_dir / filename
        self._file = open(self.log_file, 'w', buffering=1)  # Line buffering
        self._start_time = time.time()
        self._loop_count = 0
        self._is_logging = True

        # Write header
        self._file.write(f"# Pi Drone Navigation Flight Log\n")
        self._file.write(f"# Started: {datetime.now().isoformat()}\n")
        self._file.write(f"# Mission: {mission_name or 'None'}\n")
        self._file.write(f"# Frequency: 50Hz\n")
        self._file.write("#\n")
        self._file.write(",".join(self.COLUMNS) + "\n")

        logging.getLogger(__name__).info(f"Flight log started: {self.log_file}")
        return self.log_file

    def log(self,
            # State
            flight_state: str = "",
            mission_action: str = "",
            # GPS
            lat: float = 0.0, lon: float = 0.0, alt_msl: float = 0.0,
            gps_sats: int = 0, gps_hdop: float = 0.0,
            # Velocity
            vel_n: float = 0.0, vel_e: float = 0.0, vel_d: float = 0.0,
            # Attitude
            roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
            # Altitude control
            alt_baro: float = 0.0, alt_target: float = 0.0,
            climb_rate: float = 0.0, climb_rate_target: float = 0.0,
            height_agl: float = 0.0,
            # Commands
            throttle: float = 0.0, roll_cmd: float = 0.0,
            pitch_cmd: float = 0.0, yaw_cmd: float = 0.0,
            # Takeoff
            takeoff_state: str = "", takeoff_liftoff: bool = False,
            takeoff_throttle: float = 0.0,
            # Landing
            landing_phase: str = "", landing_descent_rate: float = 0.0,
            touchdown: bool = False,
            # Hover
            hover_throttle: float = 0.0,
            # PID velocity X
            pid_vx_p: float = 0.0, pid_vx_i: float = 0.0,
            pid_vx_d: float = 0.0, pid_vx_out: float = 0.0,
            # PID velocity Y
            pid_vy_p: float = 0.0, pid_vy_i: float = 0.0,
            pid_vy_d: float = 0.0, pid_vy_out: float = 0.0,
            # PID altitude
            pid_alt_error: float = 0.0, pid_alt_integral: float = 0.0,
            # IMU
            acc_x: int = 0, acc_y: int = 0, acc_z: int = 0,
            gyro_x: int = 0, gyro_y: int = 0, gyro_z: int = 0,
            # Motors
            motor1: int = 0, motor2: int = 0, motor3: int = 0, motor4: int = 0,
            # RC
            rc_roll: int = 1500, rc_pitch: int = 1500, rc_throttle: int = 1000,
            rc_yaw: int = 1500, rc_aux1: int = 1000, rc_aux2: int = 1500,
            # Battery
            vbat: float = 0.0, current_a: float = 0.0,
            ):
        """Log one data point (called at 50Hz)."""
        if not self._is_logging or self._file is None:
            return

        import time
        elapsed = time.time() - self._start_time
        self._loop_count += 1

        # Compute ground speed
        ground_speed = (vel_n**2 + vel_e**2) ** 0.5

        # Format line
        values = [
            f"{elapsed:.3f}",
            str(self._loop_count),
            flight_state,
            mission_action,
            f"{lat:.7f}",
            f"{lon:.7f}",
            f"{alt_msl:.2f}",
            str(gps_sats),
            f"{gps_hdop:.1f}",
            f"{vel_n:.3f}",
            f"{vel_e:.3f}",
            f"{vel_d:.3f}",
            f"{ground_speed:.3f}",
            f"{roll:.2f}",
            f"{pitch:.2f}",
            f"{yaw:.2f}",
            f"{alt_baro:.2f}",
            f"{alt_target:.2f}",
            f"{climb_rate:.3f}",
            f"{climb_rate_target:.3f}",
            f"{height_agl:.2f}",
            f"{throttle:.4f}",
            f"{roll_cmd:.2f}",
            f"{pitch_cmd:.2f}",
            f"{yaw_cmd:.2f}",
            takeoff_state,
            "1" if takeoff_liftoff else "0",
            f"{takeoff_throttle:.4f}",
            landing_phase,
            f"{landing_descent_rate:.3f}",
            "1" if touchdown else "0",
            f"{hover_throttle:.4f}",
            f"{pid_vx_p:.4f}",
            f"{pid_vx_i:.4f}",
            f"{pid_vx_d:.4f}",
            f"{pid_vx_out:.4f}",
            f"{pid_vy_p:.4f}",
            f"{pid_vy_i:.4f}",
            f"{pid_vy_d:.4f}",
            f"{pid_vy_out:.4f}",
            f"{pid_alt_error:.4f}",
            f"{pid_alt_integral:.4f}",
            str(acc_x),
            str(acc_y),
            str(acc_z),
            str(gyro_x),
            str(gyro_y),
            str(gyro_z),
            str(motor1),
            str(motor2),
            str(motor3),
            str(motor4),
            str(rc_roll),
            str(rc_pitch),
            str(rc_throttle),
            str(rc_yaw),
            str(rc_aux1),
            str(rc_aux2),
            f"{vbat:.2f}",
            f"{current_a:.2f}",
        ]

        self._file.write(",".join(values) + "\n")

    def stop(self):
        """Stop logging and close file."""
        if self._file:
            self._file.write(f"# Ended: {datetime.now().isoformat()}\n")
            self._file.write(f"# Total samples: {self._loop_count}\n")
            self._file.write(f"# Duration: {(self._loop_count / 50.0):.1f}s\n")
            self._file.close()
            self._file = None
            self._is_logging = False
            logging.getLogger(__name__).info(f"Flight log stopped: {self._loop_count} samples")

    def __del__(self):
        """Ensure file is closed on destruction."""
        self.stop()


# Keep old class for backward compatibility
FlightLogger = FlightDataLogger
