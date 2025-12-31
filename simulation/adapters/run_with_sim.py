#!/usr/bin/env python3
"""
Run pi_drone_nav with simulation

This script:
1. Starts Betaflight SITL
2. Starts physics simulation
3. Runs pi_drone_nav REST API server
4. Connects everything together

You can then send missions via the REST API and watch them execute.

Usage:
    python -m simulation.adapters.run_with_sim [options]

    --port PORT     REST API port (default: 8080)
    --origin LAT,LON  GPS origin (default: 48.8566,2.3522)
    -v              Verbose output
"""

import argparse
import signal
import sys
import time
import threading
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import logging
from simulation.sitl.betaflight_process import BetaflightSITL
from simulation.gazebo.physics_bridge import PhysicsBridge, SimplePhysicsSimulator
from simulation.adapters.sim_msp_driver import SimulatedMSP
from simulation.adapters.sim_gps_driver import SimulatedGPS

logger = logging.getLogger(__name__)


class SimulatedFlightController:
    """
    Simulated flight controller for pi_drone_nav

    Replaces the real FlightController when running in simulation.
    """

    def __init__(self, msp: SimulatedMSP, gps: SimulatedGPS):
        self.msp = msp
        self.gps = gps

        # Flight state
        self._armed = False
        self._mode = "IDLE"

        # RC channels
        self._rc = [1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500]

        # Control loop
        self._running = False
        self._thread = None

    def start(self):
        """Start control loop"""
        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop control loop"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _control_loop(self):
        """Main control loop at 50Hz"""
        while self._running:
            # Send RC values
            self.msp.set_raw_rc(self._rc)
            time.sleep(0.02)

    def arm(self) -> bool:
        """Arm the drone"""
        self._rc[4] = 1800  # AUX1 high
        self._armed = True
        self._mode = "ARMED"
        logger.info("ARMED")
        return True

    def disarm(self) -> bool:
        """Disarm the drone"""
        self._rc[4] = 1000  # AUX1 low
        self._rc[2] = 1000  # Throttle low
        self._armed = False
        self._mode = "IDLE"
        logger.info("DISARMED")
        return True

    def set_throttle(self, value: float):
        """Set throttle (0.0 - 1.0)"""
        self._rc[2] = int(1000 + value * 1000)

    def set_attitude(self, roll: float, pitch: float, yaw_rate: float):
        """Set attitude commands in degrees"""
        # Convert degrees to RC values
        self._rc[0] = int(1500 + roll * 500 / 30)  # Roll
        self._rc[1] = int(1500 + pitch * 500 / 30)  # Pitch
        self._rc[3] = int(1500 + yaw_rate * 500 / 180)  # Yaw

    def get_state(self) -> dict:
        """Get current state"""
        attitude = self.msp.get_attitude() or {}
        altitude = self.msp.get_altitude() or {}
        gps_fix = self.gps.get_fix()

        return {
            "armed": self._armed,
            "mode": self._mode,
            "attitude": attitude,
            "altitude": altitude.get("altitude", 0),
            "vario": altitude.get("vario", 0),
            "gps": {
                "lat": gps_fix.latitude,
                "lon": gps_fix.longitude,
                "alt": gps_fix.altitude,
                "satellites": gps_fix.satellites,
            }
        }


def run_simulation_server(port: int = 8080, origin: tuple = (48.8566, 2.3522)):
    """
    Run the full simulation with REST API

    Args:
        port: REST API port
        origin: GPS origin (lat, lon)
    """
    print("=" * 60)
    print("  Pi Drone Navigation - Simulation Mode")
    print("=" * 60)

    # 1. Start Betaflight SITL
    print("\n[1/5] Starting Betaflight SITL...")
    sitl = BetaflightSITL()
    if not sitl.start():
        print("ERROR: Failed to start SITL")
        return False

    # 2. Start physics simulation
    print("\n[2/5] Starting physics simulation...")
    physics = SimplePhysicsSimulator()
    physics.start()

    # 3. Create simulated drivers
    print("\n[3/5] Creating simulated drivers...")
    msp = SimulatedMSP()
    msp.connect()

    gps = SimulatedGPS(origin_lat=origin[0], origin_lon=origin[1])
    gps.connect(physics)
    gps.start(update_rate=10)

    # 4. Create flight controller
    print("\n[4/5] Creating flight controller...")
    fc = SimulatedFlightController(msp, gps)
    fc.start()

    # 5. Start REST API
    print("\n[5/5] Starting REST API server...")

    try:
        from flask import Flask, jsonify, request
    except ImportError:
        print("\nFlask not installed. Install with: pip install flask")
        print("Running without REST API - use interactive mode instead.\n")
        return run_interactive(fc, physics, sitl)

    app = Flask(__name__)

    @app.route('/api/health')
    def health():
        return jsonify({"status": "ok", "mode": "simulation"})

    @app.route('/api/state')
    def get_state():
        return jsonify(fc.get_state())

    @app.route('/api/arm', methods=['POST'])
    def arm():
        fc.arm()
        return jsonify({"success": True, "armed": True})

    @app.route('/api/disarm', methods=['POST'])
    def disarm():
        fc.disarm()
        return jsonify({"success": True, "armed": False})

    @app.route('/api/takeoff', methods=['POST'])
    def takeoff():
        data = request.json or {}
        alt = data.get('altitude', 2.0)

        fc.arm()
        time.sleep(0.5)

        # Simple takeoff: ramp throttle
        for t in range(0, 60):
            throttle = min(0.6, 0.3 + t * 0.01)
            fc.set_throttle(throttle)
            time.sleep(0.05)

        return jsonify({"success": True, "target_altitude": alt})

    @app.route('/api/land', methods=['POST'])
    def land():
        # Simple landing: reduce throttle
        for t in range(60, 0, -1):
            throttle = max(0, 0.3 + t * 0.005)
            fc.set_throttle(throttle)
            time.sleep(0.05)

        fc.set_throttle(0)
        fc.disarm()
        return jsonify({"success": True})

    print(f"\nREST API running on http://localhost:{port}")
    print("\nEndpoints:")
    print("  GET  /api/health  - Health check")
    print("  GET  /api/state   - Get drone state")
    print("  POST /api/arm     - Arm drone")
    print("  POST /api/disarm  - Disarm drone")
    print("  POST /api/takeoff - Take off")
    print("  POST /api/land    - Land")
    print("\nPress Ctrl+C to stop\n")

    def cleanup():
        fc.stop()
        gps.stop()
        physics.stop()
        sitl.stop()

    # Handle shutdown
    import atexit
    atexit.register(cleanup)

    try:
        app.run(host='0.0.0.0', port=port, debug=False)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()


def run_interactive(fc, physics, sitl):
    """Run in interactive mode without Flask"""
    print("\n" + "=" * 60)
    print("  Interactive Simulation Mode")
    print("=" * 60)
    print("\nCommands:")
    print("  arm     - Arm the drone")
    print("  disarm  - Disarm the drone")
    print("  takeoff - Take off to 2m")
    print("  land    - Land")
    print("  state   - Show current state")
    print("  quit    - Exit")
    print()

    try:
        while True:
            cmd = input("> ").strip().lower()

            if cmd == "arm":
                fc.arm()
            elif cmd == "disarm":
                fc.disarm()
            elif cmd == "takeoff":
                fc.arm()
                time.sleep(0.5)
                for t in range(60):
                    fc.set_throttle(min(0.6, 0.3 + t * 0.01))
                    time.sleep(0.05)
                print("Takeoff complete")
            elif cmd == "land":
                for t in range(60, 0, -1):
                    fc.set_throttle(max(0, 0.3 + t * 0.005))
                    time.sleep(0.05)
                fc.set_throttle(0)
                fc.disarm()
                print("Landed")
            elif cmd == "state":
                state = fc.get_state()
                print(f"  Armed: {state['armed']}")
                print(f"  Mode: {state['mode']}")
                print(f"  Altitude: {state['altitude']:.1f}m")
                print(f"  GPS: {state['gps']['lat']:.6f}, {state['gps']['lon']:.6f}")
            elif cmd in ("quit", "exit", "q"):
                break
            else:
                print("Unknown command")

    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        fc.stop()
        physics.stop()
        sitl.stop()
        print("\nSimulation stopped")


def main():
    parser = argparse.ArgumentParser(description='Run pi_drone_nav in simulation')
    parser.add_argument('--port', type=int, default=8080,
                       help='REST API port')
    parser.add_argument('--origin', type=str, default='48.8566,2.3522',
                       help='GPS origin as lat,lon')
    parser.add_argument('-v', '--verbose', action='store_true')

    args = parser.parse_args()

    # Parse origin
    try:
        lat, lon = map(float, args.origin.split(','))
    except ValueError:
        print("Invalid origin format. Use: --origin LAT,LON")
        sys.exit(1)

    # Setup logging
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%H:%M:%S'
    )

    run_simulation_server(port=args.port, origin=(lat, lon))


if __name__ == '__main__':
    main()
