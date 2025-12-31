#!/usr/bin/env python3
"""
Pi Drone Navigation - Simulation Launcher

This is the main entry point for running the full simulation:
1. Betaflight SITL
2. Gazebo (via Docker)
3. Physics/Sensor bridges
4. pi_drone_nav

Usage:
    python -m simulation.launcher [options]

Options:
    --no-gazebo     Run without Gazebo (SITL + bridges only)
    --headless      Run Gazebo without GUI
    --mission FILE  Start a mission after launch
"""

import argparse
import time
import sys
import signal
import threading
from pathlib import Path
from typing import Optional
import logging

# Add parent to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from simulation.sitl.betaflight_process import BetaflightSITL, SITLConfig
from simulation.gazebo.physics_bridge import PhysicsBridge, SimplePhysicsSimulator
from simulation.gazebo.sensor_bridge import SensorBridge

logger = logging.getLogger(__name__)


class SimulationLauncher:
    """
    Main simulation launcher

    Coordinates all simulation components.
    """

    def __init__(self):
        self.sitl: Optional[BetaflightSITL] = None
        self.physics_bridge: Optional[PhysicsBridge] = None
        self.sensor_bridge: Optional[SensorBridge] = None
        self.simple_sim: Optional[SimplePhysicsSimulator] = None

        self._running = False
        self._shutdown_event = threading.Event()

    def start(self, use_gazebo: bool = True, headless: bool = False) -> bool:
        """
        Start all simulation components

        Args:
            use_gazebo: Use Gazebo for physics (else simple sim)
            headless: Run without GUI

        Returns:
            True if started successfully
        """
        logger.info("=" * 60)
        logger.info("  Pi Drone Navigation - Simulation")
        logger.info("=" * 60)

        # 1. Start Betaflight SITL
        logger.info("\n[1/4] Starting Betaflight SITL...")
        self.sitl = BetaflightSITL()

        if not self.sitl.start():
            logger.error("Failed to start SITL")
            return False

        logger.info("SITL started successfully")

        # 2. Start physics bridge (receives PWM from SITL)
        logger.info("\n[2/4] Starting physics bridge...")
        self.physics_bridge = PhysicsBridge()
        self.physics_bridge.start(sitl_port=9002)
        logger.info("Physics bridge listening on UDP 9002")

        # 3. Start sensor bridge (sends sensors to SITL)
        logger.info("\n[3/4] Starting sensor bridge...")
        self.sensor_bridge = SensorBridge()
        self.sensor_bridge.start(sitl_port=9003)
        logger.info("Sensor bridge sending to UDP 9003")

        # 4. Start physics simulation
        if use_gazebo:
            logger.info("\n[4/4] Gazebo mode selected")
            logger.info("Note: Full Gazebo integration requires additional setup")
            logger.info("Using simple physics simulation for now...")
            use_gazebo = False  # Fall back to simple sim

        if not use_gazebo:
            logger.info("\n[4/4] Starting simple physics simulation...")
            self.simple_sim = SimplePhysicsSimulator()
            self.simple_sim.start()
            logger.info("Simple physics simulation running")

        self._running = True

        # Print connection info
        self._print_connection_info()

        return True

    def _print_connection_info(self):
        """Print how to connect pi_drone_nav"""
        print("\n" + "=" * 60)
        print("  SIMULATION READY")
        print("=" * 60)
        print("\nBetaflight SITL ports:")
        print("  - PWM output (to Gazebo):  UDP 9002")
        print("  - Sensor input:            UDP 9003")
        print("  - RC input:                UDP 9004")
        print("\nTo connect pi_drone_nav, use the TCP adapter:")
        print("  python -m simulation.adapters.run_with_sim")
        print("\nOr manually:")
        print("  python -m src.main --simulate")
        print("\nPress Ctrl+C to stop simulation")
        print("=" * 60 + "\n")

    def stop(self):
        """Stop all simulation components"""
        logger.info("\nStopping simulation...")
        self._running = False
        self._shutdown_event.set()

        if self.simple_sim:
            self.simple_sim.stop()

        if self.sensor_bridge:
            self.sensor_bridge.stop()

        if self.physics_bridge:
            self.physics_bridge.stop()

        if self.sitl:
            self.sitl.stop()

        logger.info("Simulation stopped")

    def run_forever(self):
        """Run until interrupted"""
        try:
            while self._running:
                # Display status
                if self.physics_bridge:
                    motors = self.physics_bridge.get_motor_pwm()
                    print(f"\rMotors: {motors}  ", end='', flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

    def wait_for_shutdown(self):
        """Wait for shutdown signal"""
        self._shutdown_event.wait()


def main():
    parser = argparse.ArgumentParser(description='Pi Drone Nav Simulation Launcher')
    parser.add_argument('--no-gazebo', action='store_true',
                       help='Run without Gazebo (simple physics only)')
    parser.add_argument('--headless', action='store_true',
                       help='Run without GUI')
    parser.add_argument('--mission', type=str,
                       help='Mission file to run after startup')
    parser.add_argument('-v', '--verbose', action='store_true',
                       help='Verbose output')

    args = parser.parse_args()

    # Setup logging
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S'
    )

    # Create launcher
    launcher = SimulationLauncher()

    # Handle signals
    def signal_handler(sig, frame):
        launcher.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Start simulation
    use_gazebo = not args.no_gazebo

    if launcher.start(use_gazebo=use_gazebo, headless=args.headless):
        launcher.run_forever()
    else:
        logger.error("Failed to start simulation")
        sys.exit(1)


if __name__ == '__main__':
    main()
