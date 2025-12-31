"""
Betaflight SITL Process Manager

Manages the Betaflight SITL executable lifecycle and provides
TCP connection for MSP communication.
"""

import subprocess
import socket
import time
import os
import signal
from pathlib import Path
from dataclasses import dataclass
from typing import Optional
import logging

logger = logging.getLogger(__name__)


@dataclass
class SITLConfig:
    """Configuration for Betaflight SITL"""
    # Paths
    sitl_path: str = ""  # Auto-detect if empty
    working_dir: str = "/tmp/betaflight_sitl"

    # Network ports (Betaflight SITL defaults)
    gazebo_port: int = 9002      # PWM output to Gazebo
    realflight_port: int = 9001  # RealFlight bridge
    udp_server_port: int = 9003  # Telemetry/state
    rc_input_port: int = 9004    # RC channel input

    # MSP over TCP (we'll add this via socat or direct TCP)
    msp_tcp_port: int = 5760

    # Simulation parameters
    target_ip: str = "127.0.0.1"


class BetaflightSITL:
    """
    Manages Betaflight SITL process

    Usage:
        sitl = BetaflightSITL()
        sitl.start()
        # ... use simulation ...
        sitl.stop()
    """

    def __init__(self, config: Optional[SITLConfig] = None):
        self.config = config or SITLConfig()
        self._process: Optional[subprocess.Popen] = None
        self._msp_socket: Optional[socket.socket] = None

        # Auto-detect SITL path
        if not self.config.sitl_path:
            self.config.sitl_path = self._find_sitl_binary()

    def _find_sitl_binary(self) -> str:
        """Find Betaflight SITL binary"""
        search_paths = [
            Path.home() / "projects/betaflight/obj/betaflight_2026.6.0-alpha_SITL",
            Path.home() / "projects/betaflight/obj/main/betaflight_SITL.elf",
            Path("/usr/local/bin/betaflight_SITL"),
        ]

        for path in search_paths:
            if path.exists():
                logger.info(f"Found SITL binary: {path}")
                return str(path)

        raise FileNotFoundError(
            "Betaflight SITL binary not found. "
            "Compile with: cd ~/projects/betaflight && make SITL"
        )

    def start(self, wait_ready: bool = True, timeout: float = 10.0) -> bool:
        """
        Start Betaflight SITL process

        Args:
            wait_ready: Wait for SITL to be ready
            timeout: Timeout for waiting

        Returns:
            True if started successfully
        """
        if self._process is not None:
            logger.warning("SITL already running")
            return True

        # Create working directory
        work_dir = Path(self.config.working_dir)
        work_dir.mkdir(parents=True, exist_ok=True)

        # Build command
        cmd = [
            self.config.sitl_path,
            self.config.target_ip,
        ]

        logger.info(f"Starting Betaflight SITL: {' '.join(cmd)}")

        try:
            self._process = subprocess.Popen(
                cmd,
                cwd=str(work_dir),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )

            if wait_ready:
                return self._wait_ready(timeout)

            return True

        except Exception as e:
            logger.error(f"Failed to start SITL: {e}")
            return False

    def _wait_ready(self, timeout: float) -> bool:
        """Wait for SITL to be ready (UDP ports open)"""
        start = time.time()

        while time.time() - start < timeout:
            # Check if process died
            if self._process.poll() is not None:
                logger.error("SITL process died during startup")
                return False

            # Try to connect to RC input port
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(0.5)
                sock.sendto(b'\x00' * 16, (self.config.target_ip, self.config.rc_input_port))
                sock.close()
                logger.info("SITL ready")
                return True
            except Exception:
                time.sleep(0.2)

        logger.error("Timeout waiting for SITL to be ready")
        return False

    def stop(self):
        """Stop Betaflight SITL process"""
        if self._process is None:
            return

        logger.info("Stopping Betaflight SITL")

        # Try graceful shutdown
        self._process.terminate()

        try:
            self._process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            logger.warning("SITL didn't terminate, killing")
            self._process.kill()
            self._process.wait()

        self._process = None

    def is_running(self) -> bool:
        """Check if SITL is running"""
        if self._process is None:
            return False
        return self._process.poll() is None

    def send_rc_channels(self, channels: list[int]) -> bool:
        """
        Send RC channel values to SITL

        Args:
            channels: List of 8-16 channel values (1000-2000)

        Returns:
            True if sent successfully
        """
        if not self.is_running():
            return False

        # Pad to 16 channels
        while len(channels) < 16:
            channels.append(1500)

        # Pack as 16 x uint16 little-endian
        import struct
        data = struct.pack('<16H', *channels[:16])

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(data, (self.config.target_ip, self.config.rc_input_port))
            sock.close()
            return True
        except Exception as e:
            logger.error(f"Failed to send RC: {e}")
            return False

    def get_output(self) -> Optional[str]:
        """Get SITL stdout output (non-blocking)"""
        if self._process is None:
            return None

        try:
            # Non-blocking read
            import select
            if select.select([self._process.stdout], [], [], 0)[0]:
                return self._process.stdout.readline()
        except Exception:
            pass

        return None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


if __name__ == "__main__":
    # Test SITL process
    logging.basicConfig(level=logging.INFO)

    print("=== Betaflight SITL Process Test ===\n")

    sitl = BetaflightSITL()

    try:
        print("Starting SITL...")
        if sitl.start():
            print("SITL started successfully!")

            # Send some RC values
            print("\nSending RC channels (centered)...")
            sitl.send_rc_channels([1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500])

            print("\nWaiting 3 seconds...")
            time.sleep(3)

            print("\nSending ARM (AUX1 = 1800)...")
            sitl.send_rc_channels([1500, 1500, 1000, 1500, 1800, 1500, 1500, 1500])

            time.sleep(2)

            print("\nSITL is running:", sitl.is_running())
        else:
            print("Failed to start SITL")

    finally:
        print("\nStopping SITL...")
        sitl.stop()
        print("Done")
