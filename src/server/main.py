#!/usr/bin/env python3
"""
Pi Drone Navigation Server - Entry Point

Daemon service that manages FlightController and exposes REST API.
"""

import argparse
import signal
import sys
import time
import logging
from pathlib import Path

from ..config import Config, set_config
from ..flight.flight_controller import FlightController
from ..utils.logger import setup_logging

# Global flight controller instance for signal handling
_flight_controller: FlightController = None
_api_server = None


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully"""
    global _flight_controller, _api_server

    logging.info(f"Received signal {signum}, shutting down...")

    if _flight_controller:
        # Attempt safe landing if flying
        if _flight_controller.state_machine.is_flying:
            logging.warning("Attempting emergency landing...")
            _flight_controller.land()
            time.sleep(5)  # Give time to land

        # Always disarm on shutdown (safety)
        if _flight_controller._armed:
            logging.info("Disarming on shutdown...")
            _flight_controller.disarm()

        _flight_controller.shutdown()

    sys.exit(0)


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Pi Drone Navigation Server",
        prog="pidrone-server"
    )

    parser.add_argument(
        "-c", "--config",
        type=str,
        default=None,
        help="Path to configuration file (YAML)"
    )

    parser.add_argument(
        "-s", "--simulate",
        action="store_true",
        help="Run in simulation mode (no hardware)"
    )

    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="REST API port (default: 8080)"
    )

    parser.add_argument(
        "--host",
        type=str,
        default="0.0.0.0",
        help="REST API host (default: 0.0.0.0)"
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
        "--foreground",
        action="store_true",
        default=True,
        help="Run in foreground (default)"
    )

    return parser.parse_args()


def main():
    """Main entry point for pidrone-server"""
    global _flight_controller, _api_server

    args = parse_args()

    # Setup logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    setup_logging(level=log_level, log_file=args.log_file)

    logger = logging.getLogger(__name__)
    logger.info("Pi Drone Navigation Server starting...")

    # Load configuration
    config = Config.load(args.config)

    # Override config from command line
    if args.simulate:
        config.simulation.enabled = True

    if args.uart:
        config.serial.port = args.uart
        config.serial.auto_detect = False  # Explicit port disables auto-detect

    if args.usb:
        config.serial.usb_port = args.usb
        config.serial.use_usb = True
        config.serial.auto_detect = False  # Explicit port disables auto-detect

    if args.gps_port:
        config.gps.port = args.gps_port

    config.interface.rest_port = args.port

    set_config(config)

    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Initialize flight controller
    _flight_controller = FlightController(simulation=config.simulation.enabled)

    if not _flight_controller.initialize():
        logger.error("Failed to initialize flight controller")
        sys.exit(1)

    # Start REST API
    try:
        from .api import create_api_server
        _api_server = create_api_server(
            _flight_controller,
            port=args.port,
            host=args.host
        )
        if _api_server:
            logger.info(f"REST API listening on http://{args.host}:{args.port}")
        else:
            logger.error("Failed to start REST API")
            sys.exit(1)
    except ImportError as e:
        logger.error(f"REST API not available: {e}")
        sys.exit(1)

    # Start flight controller
    _flight_controller.start()

    logger.info("Server running. Press Ctrl+C to stop.")

    # Main loop - keep alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        logger.info("Shutting down...")
        if _api_server and hasattr(_api_server, 'stop'):
            _api_server.stop()
        _flight_controller.shutdown()


if __name__ == "__main__":
    main()
