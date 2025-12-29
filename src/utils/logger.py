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


class FlightLogger:
    """
    Specialized flight data logger

    Logs flight data in a structured format for post-flight analysis.
    """

    def __init__(self, log_dir: str = "logs"):
        """
        Initialize flight logger

        Args:
            log_dir: Directory for flight logs
        """
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.log_file = None
        self._file = None

    def start_flight_log(self):
        """Start a new flight log"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = self.log_dir / f"flight_{timestamp}.log"
        self._file = open(self.log_file, 'w')

        # Write header
        self._file.write("# Pi Drone Navigation Flight Log\n")
        self._file.write(f"# Started: {datetime.now().isoformat()}\n")
        self._file.write("#\n")
        self._file.write("# time_ms,state,lat,lon,alt,roll,pitch,yaw,")
        self._file.write("vel_n,vel_e,vel_d,throttle,target_lat,target_lon,target_alt\n")

    def log_state(self, time_ms: int, state: str,
                  lat: float, lon: float, alt: float,
                  roll: float, pitch: float, yaw: float,
                  vel_n: float, vel_e: float, vel_d: float,
                  throttle: float,
                  target_lat: float = 0, target_lon: float = 0, target_alt: float = 0):
        """Log current flight state"""
        if self._file is None:
            return

        line = (f"{time_ms},{state},"
                f"{lat:.7f},{lon:.7f},{alt:.2f},"
                f"{roll:.2f},{pitch:.2f},{yaw:.2f},"
                f"{vel_n:.2f},{vel_e:.2f},{vel_d:.2f},"
                f"{throttle:.3f},"
                f"{target_lat:.7f},{target_lon:.7f},{target_alt:.2f}\n")

        self._file.write(line)
        self._file.flush()

    def end_flight_log(self):
        """End current flight log"""
        if self._file:
            self._file.write(f"# Ended: {datetime.now().isoformat()}\n")
            self._file.close()
            self._file = None
