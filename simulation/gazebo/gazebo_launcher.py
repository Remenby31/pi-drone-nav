"""
Gazebo Launcher

Manages Gazebo simulation via Docker for easy setup.
"""

import subprocess
import time
import os
from pathlib import Path
from dataclasses import dataclass
from typing import Optional
import logging

logger = logging.getLogger(__name__)


@dataclass
class GazeboConfig:
    """Gazebo configuration"""
    # Docker settings
    docker_image: str = "gazebo:libgazebo11"  # Classic Gazebo 11
    container_name: str = "pidrone_gazebo"

    # World and model paths
    world_file: str = ""  # Will use default if empty
    models_path: str = ""  # Additional models path

    # Display
    display: str = ""  # Auto-detect if empty

    # Network
    host_network: bool = True  # Use host network for UDP communication


class GazeboLauncher:
    """
    Launch Gazebo in Docker container

    This provides an easy way to run Gazebo without complex installation.
    """

    def __init__(self, config: Optional[GazeboConfig] = None):
        self.config = config or GazeboConfig()
        self._container_id: Optional[str] = None

        # Set display
        if not self.config.display:
            self.config.display = os.environ.get("DISPLAY", ":0")

        # Set default paths
        self._setup_paths()

    def _setup_paths(self):
        """Setup model and world paths"""
        sim_dir = Path(__file__).parent.parent

        if not self.config.world_file:
            self.config.world_file = str(sim_dir / "gazebo" / "worlds" / "drone_test.world")

        if not self.config.models_path:
            self.config.models_path = str(sim_dir / "gazebo" / "models")

    def check_docker(self) -> bool:
        """Check if Docker is available and running"""
        try:
            result = subprocess.run(
                ["docker", "info"],
                capture_output=True,
                text=True,
                timeout=5
            )
            return result.returncode == 0
        except Exception as e:
            logger.error(f"Docker not available: {e}")
            return False

    def pull_image(self) -> bool:
        """Pull Gazebo Docker image"""
        logger.info(f"Pulling Docker image: {self.config.docker_image}")

        try:
            result = subprocess.run(
                ["docker", "pull", self.config.docker_image],
                capture_output=True,
                text=True,
                timeout=300  # 5 min timeout for pull
            )

            if result.returncode != 0:
                logger.error(f"Failed to pull image: {result.stderr}")
                return False

            return True

        except subprocess.TimeoutExpired:
            logger.error("Timeout pulling Docker image")
            return False

    def start(self, headless: bool = False) -> bool:
        """
        Start Gazebo in Docker container

        Args:
            headless: Run without GUI (for CI/testing)

        Returns:
            True if started successfully
        """
        if not self.check_docker():
            logger.error("Docker not available")
            return False

        # Stop any existing container
        self.stop()

        # Build docker run command
        cmd = ["docker", "run", "-d", "--rm"]
        cmd += ["--name", self.config.container_name]

        # Network mode
        if self.config.host_network:
            cmd += ["--network", "host"]

        # Display for GUI
        if not headless:
            cmd += ["-e", f"DISPLAY={self.config.display}"]
            cmd += ["-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw"]
            # Allow X11 access
            subprocess.run(["xhost", "+local:docker"], capture_output=True)

        # Mount models and worlds
        models_dir = Path(self.config.models_path)
        worlds_dir = Path(self.config.world_file).parent

        if models_dir.exists():
            cmd += ["-v", f"{models_dir}:/models:ro"]
            cmd += ["-e", "GAZEBO_MODEL_PATH=/models"]

        if worlds_dir.exists():
            cmd += ["-v", f"{worlds_dir}:/worlds:ro"]

        # Mount our custom plugin if exists
        plugins_dir = Path(__file__).parent / "plugins"
        if plugins_dir.exists():
            cmd += ["-v", f"{plugins_dir}:/plugins:ro"]

        # Image and command
        cmd += [self.config.docker_image]

        if headless:
            cmd += ["gzserver", "--verbose"]
        else:
            cmd += ["gazebo", "--verbose"]

        # Add world file
        world_name = Path(self.config.world_file).name
        cmd += [f"/worlds/{world_name}"]

        logger.info(f"Starting Gazebo: {' '.join(cmd)}")

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

            if result.returncode != 0:
                logger.error(f"Failed to start Gazebo: {result.stderr}")
                return False

            self._container_id = result.stdout.strip()
            logger.info(f"Gazebo container started: {self._container_id[:12]}")

            # Wait for Gazebo to be ready
            return self._wait_ready()

        except Exception as e:
            logger.error(f"Failed to start Gazebo: {e}")
            return False

    def _wait_ready(self, timeout: float = 30.0) -> bool:
        """Wait for Gazebo to be ready"""
        start = time.time()

        while time.time() - start < timeout:
            if not self.is_running():
                logger.error("Gazebo container died during startup")
                return False

            # Check if Gazebo is responding
            # For now, just wait a bit
            time.sleep(2)
            return True

        return False

    def stop(self):
        """Stop Gazebo container"""
        try:
            subprocess.run(
                ["docker", "stop", self.config.container_name],
                capture_output=True,
                timeout=10
            )
        except Exception:
            pass

        try:
            subprocess.run(
                ["docker", "rm", "-f", self.config.container_name],
                capture_output=True,
                timeout=5
            )
        except Exception:
            pass

        self._container_id = None

    def is_running(self) -> bool:
        """Check if Gazebo container is running"""
        try:
            result = subprocess.run(
                ["docker", "ps", "-q", "-f", f"name={self.config.container_name}"],
                capture_output=True,
                text=True,
                timeout=5
            )
            return bool(result.stdout.strip())
        except Exception:
            return False

    def get_logs(self, lines: int = 50) -> str:
        """Get container logs"""
        try:
            result = subprocess.run(
                ["docker", "logs", "--tail", str(lines), self.config.container_name],
                capture_output=True,
                text=True,
                timeout=5
            )
            return result.stdout + result.stderr
        except Exception as e:
            return f"Failed to get logs: {e}"

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


class NativeGazeboLauncher:
    """
    Launch Gazebo natively (if installed on system)

    Alternative to Docker when Gazebo is installed locally.
    """

    def __init__(self, config: Optional[GazeboConfig] = None):
        self.config = config or GazeboConfig()
        self._process: Optional[subprocess.Popen] = None

    def check_gazebo(self) -> bool:
        """Check if Gazebo is installed"""
        try:
            result = subprocess.run(
                ["which", "gazebo"],
                capture_output=True,
                text=True
            )
            return result.returncode == 0
        except Exception:
            return False

    def start(self, headless: bool = False) -> bool:
        """Start Gazebo natively"""
        if not self.check_gazebo():
            logger.error("Gazebo not installed. Use Docker launcher instead.")
            return False

        cmd = ["gzserver" if headless else "gazebo", "--verbose"]

        if self.config.world_file and Path(self.config.world_file).exists():
            cmd.append(self.config.world_file)

        # Set model path
        env = os.environ.copy()
        if self.config.models_path:
            env["GAZEBO_MODEL_PATH"] = self.config.models_path

        logger.info(f"Starting Gazebo: {' '.join(cmd)}")

        try:
            self._process = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            time.sleep(3)  # Wait for startup
            return self.is_running()

        except Exception as e:
            logger.error(f"Failed to start Gazebo: {e}")
            return False

    def stop(self):
        """Stop Gazebo"""
        if self._process:
            self._process.terminate()
            try:
                self._process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None

    def is_running(self) -> bool:
        """Check if Gazebo is running"""
        return self._process is not None and self._process.poll() is None


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    print("=== Gazebo Launcher Test ===\n")

    launcher = GazeboLauncher()

    print("Checking Docker...")
    if launcher.check_docker():
        print("Docker OK")
    else:
        print("Docker not available")
        exit(1)

    print("\nNote: To run Gazebo, first create the world file.")
    print("Then run: python -m simulation.gazebo.gazebo_launcher")
