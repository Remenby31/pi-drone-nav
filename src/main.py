#!/usr/bin/env python3
"""
Pi Drone Navigation - Main Entry Point

Autonomous flight controller for Raspberry Pi + Betaflight
"""

import argparse
import signal
import sys
import time
import logging
from pathlib import Path

from .config import Config, set_config
from .flight.flight_controller import FlightController
from .utils.logger import setup_logging

# Global flight controller instance for signal handling
_flight_controller: FlightController = None


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully"""
    global _flight_controller

    logging.info(f"Received signal {signum}, shutting down...")

    if _flight_controller:
        # Attempt safe landing if flying
        if _flight_controller.state_machine.is_flying:
            logging.warning("Attempting emergency landing...")
            _flight_controller.land()
            time.sleep(5)  # Give time to land

        _flight_controller.shutdown()

    sys.exit(0)


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Pi Drone Navigation - Autonomous Flight Controller"
    )

    parser.add_argument(
        "-c", "--config",
        type=str,
        default=None,
        help="Path to configuration file (YAML)"
    )

    parser.add_argument(
        "-s", "--simulation",
        action="store_true",
        help="Run in simulation mode (SITL)"
    )

    parser.add_argument(
        "--uart",
        type=str,
        default=None,
        help="UART port for Betaflight (e.g., /dev/ttyAMA0)"
    )

    parser.add_argument(
        "--usb",
        type=str,
        default=None,
        help="USB port for Betaflight (e.g., /dev/ttyACM0)"
    )

    parser.add_argument(
        "--gps-port",
        type=str,
        default=None,
        help="GPS module port (e.g., /dev/ttyAMA1)"
    )

    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose logging"
    )

    parser.add_argument(
        "--log-file",
        type=str,
        default=None,
        help="Log file path"
    )

    parser.add_argument(
        "--cli",
        action="store_true",
        default=False,
        help="Enable interactive CLI interface"
    )

    parser.add_argument(
        "--rest-api",
        action="store_true",
        default=True,
        help="Enable REST API (default: True)"
    )

    parser.add_argument(
        "--rest-port",
        type=int,
        default=8080,
        help="REST API port (default: 8080)"
    )

    parser.add_argument(
        "--mavlink",
        action="store_true",
        default=True,
        help="Enable MAVLink bridge (default: True)"
    )

    parser.add_argument(
        "--mavlink-port",
        type=int,
        default=14550,
        help="MAVLink UDP port (default: 14550)"
    )

    return parser.parse_args()


def main():
    """Main entry point"""
    global _flight_controller

    args = parse_args()

    # Setup logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    setup_logging(level=log_level, log_file=args.log_file)

    logger = logging.getLogger(__name__)
    logger.info("Pi Drone Navigation starting...")

    # Load configuration
    config = Config.load(args.config)

    # Override config from command line
    if args.simulation:
        config.simulation.enabled = True

    if args.uart:
        config.serial.port = args.uart

    if args.usb:
        config.serial.usb_port = args.usb
        config.serial.use_usb = True

    if args.gps_port:
        config.gps.port = args.gps_port

    config.interface.rest_enabled = args.rest_api
    config.interface.rest_port = args.rest_port
    config.interface.mavlink_enabled = args.mavlink
    config.interface.mavlink_port = args.mavlink_port

    set_config(config)

    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Initialize flight controller
    _flight_controller = FlightController(simulation=config.simulation.enabled)

    if not _flight_controller.initialize():
        logger.error("Failed to initialize flight controller")
        sys.exit(1)

    # Start interfaces
    interfaces = []

    if config.interface.rest_enabled:
        try:
            from .interfaces.rest_api import create_api_server
            api_server = create_api_server(_flight_controller, config.interface.rest_port)
            interfaces.append(api_server)
            logger.info(f"REST API started on port {config.interface.rest_port}")
        except ImportError as e:
            logger.warning(f"REST API not available: {e}")

    if config.interface.mavlink_enabled:
        try:
            from .interfaces.mavlink_bridge import MAVLinkBridge
            mavlink = MAVLinkBridge(_flight_controller, config.interface.mavlink_port)
            mavlink.start()
            interfaces.append(mavlink)
            logger.info(f"MAVLink bridge started on UDP:{config.interface.mavlink_port}")
        except ImportError as e:
            logger.warning(f"MAVLink not available: {e}")

    # Start flight controller
    _flight_controller.start()

    logger.info("Flight controller running. Press Ctrl+C to stop.")

    # Main loop - CLI or background
    try:
        if args.cli:
            # Run interactive CLI (blocks until quit)
            from .interfaces.cli import CLI
            cli = CLI(_flight_controller)
            logger.info("Starting interactive CLI...")
            cli.cmdloop()
        else:
            # Background mode - just print status periodically
            while True:
                time.sleep(1)

                # Print status periodically
                telemetry = _flight_controller.get_telemetry()
                state = telemetry['state']['state']
                alt = telemetry['position']['alt']
                sats = telemetry['position']['satellites']

                logger.debug(f"State: {state}, Alt: {alt:.1f}m, Sats: {sats}")

    except KeyboardInterrupt:
        pass
    finally:
        logger.info("Shutting down...")

        for interface in interfaces:
            if hasattr(interface, 'stop'):
                interface.stop()

        _flight_controller.shutdown()


if __name__ == "__main__":
    main()
