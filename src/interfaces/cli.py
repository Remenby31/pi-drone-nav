"""
Command Line Interface for Pi Drone Navigation

Interactive terminal interface for drone control.
"""

import cmd
import threading
import time
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from ..flight.flight_controller import FlightController


class CLI(cmd.Cmd):
    """
    Interactive command-line interface

    Provides commands for:
    - Arming/disarming
    - Takeoff/landing
    - Navigation control
    - Status monitoring
    - Mission management
    """

    intro = """
    ╔═══════════════════════════════════════════════════════════════╗
    ║           Pi Drone Navigation - Command Interface             ║
    ╠═══════════════════════════════════════════════════════════════╣
    ║  Type 'help' for available commands                           ║
    ║  Type 'status' for current drone state                        ║
    ║  Type 'quit' to exit                                          ║
    ╚═══════════════════════════════════════════════════════════════╝
    """
    prompt = "drone> "

    def __init__(self, flight_controller: 'FlightController'):
        super().__init__()
        self.fc = flight_controller
        self._status_thread: Optional[threading.Thread] = None
        self._show_continuous_status = False

    # ==================== Status Commands ====================

    def do_status(self, arg):
        """Show current drone status"""
        telemetry = self.fc.get_telemetry()

        print("\n=== Drone Status ===")
        print(f"State: {telemetry['state']['state']}")
        print(f"Armed: {telemetry['armed']}")
        print(f"Flying: {telemetry['state']['is_flying']}")

        print("\n--- Position ---")
        pos = telemetry['position']
        print(f"Lat: {pos['lat']:.6f}")
        print(f"Lon: {pos['lon']:.6f}")
        print(f"Alt: {pos['alt']:.1f} m")
        print(f"Satellites: {pos['satellites']}")

        print("\n--- Attitude ---")
        att = telemetry['attitude']
        print(f"Roll:  {att['roll']:.1f}°")
        print(f"Pitch: {att['pitch']:.1f}°")
        print(f"Yaw:   {att['yaw']:.1f}°")

        if telemetry['navigation']:
            nav = telemetry['navigation']
            print("\n--- Navigation ---")
            print(f"Mission: {nav['mission_name'] or 'None'}")
            print(f"State: {nav['state']}")
            if nav['waypoint_count'] > 0:
                print(f"Waypoint: {nav['waypoint_index']+1}/{nav['waypoint_count']}")
                print(f"Distance: {nav['distance_to_wp']:.1f} m")

        print()

    def do_watch(self, arg):
        """
        Toggle continuous status display
        Usage: watch [on|off]
        """
        if arg.lower() == 'off':
            self._show_continuous_status = False
            print("Status watch disabled")
        elif arg.lower() == 'on' or not arg:
            self._show_continuous_status = True
            self._start_status_watch()
            print("Status watch enabled (Ctrl+C to stop)")

    def _start_status_watch(self):
        """Start continuous status display"""
        def watch_loop():
            while self._show_continuous_status:
                telemetry = self.fc.get_telemetry()
                state = telemetry['state']['state']
                alt = telemetry['position']['alt']
                sats = telemetry['position']['satellites']
                print(f"\rState: {state:15} Alt: {alt:6.1f}m Sats: {sats}", end='', flush=True)
                time.sleep(0.5)
            print()

        self._status_thread = threading.Thread(target=watch_loop, daemon=True)
        self._status_thread.start()

    # ==================== Arming Commands ====================

    def do_arm(self, arg):
        """Arm the drone"""
        if self.fc.arm():
            print("Drone armed")
        else:
            print("ERROR: Failed to arm")

    def do_disarm(self, arg):
        """Disarm the drone (only when not flying)"""
        if self.fc.disarm():
            print("Drone disarmed")
        else:
            print("ERROR: Cannot disarm (flying?)")

    # ==================== Flight Commands ====================

    def do_takeoff(self, arg):
        """
        Takeoff to specified altitude
        Usage: takeoff [altitude_m]
        Default: 3 meters
        """
        try:
            altitude = float(arg) if arg else 3.0
        except ValueError:
            print("ERROR: Invalid altitude")
            return

        if altitude < 1 or altitude > 50:
            print("ERROR: Altitude must be between 1 and 50 meters")
            return

        if self.fc.takeoff(altitude):
            print(f"Taking off to {altitude}m")
        else:
            print("ERROR: Takeoff failed (not armed?)")

    def do_land(self, arg):
        """Initiate landing"""
        if self.fc.land():
            print("Landing initiated")
        else:
            print("ERROR: Landing failed")

    def do_hold(self, arg):
        """Hold current position"""
        if self.fc.hold_position():
            print("Position hold activated")
        else:
            print("ERROR: Position hold failed")

    def do_rth(self, arg):
        """Return to home"""
        if self.fc.return_to_home():
            print("Returning to home")
        else:
            print("ERROR: RTH failed")

    # ==================== Navigation Commands ====================

    def do_goto(self, arg):
        """
        Go to GPS coordinates
        Usage: goto <lat> <lon> [alt]
        Example: goto 48.8566 2.3522 20
        """
        parts = arg.split()
        if len(parts) < 2:
            print("Usage: goto <lat> <lon> [alt]")
            return

        try:
            lat = float(parts[0])
            lon = float(parts[1])
            alt = float(parts[2]) if len(parts) > 2 else 10.0
        except ValueError:
            print("ERROR: Invalid coordinates")
            return

        if self.fc.goto(lat, lon, alt):
            print(f"Flying to {lat:.6f}, {lon:.6f} at {alt}m")
        else:
            print("ERROR: Goto failed")

    def do_mission(self, arg):
        """
        Mission management
        Usage:
            mission load <file.json>  - Load mission from file
            mission start             - Start loaded mission
            mission pause             - Pause mission
            mission resume            - Resume mission
            mission abort             - Abort mission
            mission status            - Show mission status
        """
        parts = arg.split()
        if not parts:
            print("Usage: mission <load|start|pause|resume|abort|status> [file]")
            return

        cmd = parts[0].lower()

        if cmd == 'load' and len(parts) > 1:
            filepath = parts[1]
            try:
                from ..navigation.waypoint_navigator import Mission
                mission = Mission.from_json(filepath)
                self.fc.waypoint_nav.load_mission(mission)
                print(f"Mission '{mission.name}' loaded with {len(mission.waypoints)} waypoints")
            except Exception as e:
                print(f"ERROR: Failed to load mission: {e}")

        elif cmd == 'start':
            if self.fc.waypoint_nav and self.fc.waypoint_nav.mission:
                if self.fc.start_mission(self.fc.waypoint_nav.mission):
                    print("Mission started")
                else:
                    print("ERROR: Failed to start mission")
            else:
                print("ERROR: No mission loaded")

        elif cmd == 'pause':
            if self.fc.waypoint_nav:
                self.fc.waypoint_nav.pause()
                print("Mission paused")

        elif cmd == 'resume':
            if self.fc.waypoint_nav:
                self.fc.waypoint_nav.resume()
                print("Mission resumed")

        elif cmd == 'abort':
            if self.fc.waypoint_nav:
                self.fc.waypoint_nav.abort()
                print("Mission aborted")

        elif cmd == 'status':
            if self.fc.waypoint_nav:
                status = self.fc.waypoint_nav.get_status()
                print(f"Mission: {status['mission_name']}")
                print(f"State: {status['state']}")
                print(f"Progress: {status['progress']*100:.0f}%")
                if status['current_wp']:
                    wp = status['current_wp']
                    print(f"Current WP: {wp['latitude']:.6f}, {wp['longitude']:.6f}")
            else:
                print("No mission loaded")

    # ==================== Configuration Commands ====================

    def do_config(self, arg):
        """
        Show or modify configuration
        Usage:
            config                     - Show all config
            config <section>           - Show section
            config <section.key> <val> - Set value
        """
        from ..config import get_config
        config = get_config()

        parts = arg.split()

        if not parts:
            # Show all config sections
            print("\n=== Configuration ===")
            for section in ['serial', 'gps', 'navigation', 'altitude', 'failsafe', 'interface']:
                print(f"\n[{section}]")
                sect = getattr(config, section)
                for key, val in sect.__dict__.items():
                    print(f"  {key}: {val}")
            return

        if len(parts) == 1:
            # Show specific section
            section = parts[0]
            if hasattr(config, section):
                print(f"\n[{section}]")
                sect = getattr(config, section)
                for key, val in sect.__dict__.items():
                    print(f"  {key}: {val}")
            else:
                print(f"Unknown section: {section}")
            return

        # Set value
        key_path = parts[0].split('.')
        value = parts[1]

        if len(key_path) == 2:
            section, key = key_path
            if hasattr(config, section):
                sect = getattr(config, section)
                if hasattr(sect, key):
                    current = getattr(sect, key)
                    # Type conversion
                    if isinstance(current, bool):
                        value = value.lower() in ('true', '1', 'yes')
                    elif isinstance(current, int):
                        value = int(value)
                    elif isinstance(current, float):
                        value = float(value)

                    setattr(sect, key, value)
                    print(f"Set {section}.{key} = {value}")
                else:
                    print(f"Unknown key: {key}")
            else:
                print(f"Unknown section: {section}")

    # ==================== System Commands ====================

    def do_calibrate(self, arg):
        """
        Run calibration
        Usage:
            calibrate acc   - Calibrate accelerometer
            calibrate mag   - Calibrate magnetometer
            calibrate hover - Calibrate hover throttle
        """
        if not arg:
            print("Usage: calibrate <acc|mag|hover>")
            return

        if arg == 'acc':
            if self.fc.msp:
                self.fc.msp.calibrate_acc()
                print("Accelerometer calibration started - keep drone level!")
        elif arg == 'mag':
            if self.fc.msp:
                self.fc.msp.calibrate_mag()
                print("Magnetometer calibration started - rotate drone in all axes!")
        elif arg == 'hover':
            print("Hover throttle calibration not yet implemented")

    def do_reboot(self, arg):
        """Reboot the flight controller"""
        confirm = input("Are you sure you want to reboot the FC? (yes/no): ")
        if confirm.lower() == 'yes':
            if self.fc.msp:
                self.fc.msp.reboot()
                print("Rebooting flight controller...")
        else:
            print("Cancelled")

    def do_quit(self, arg):
        """Exit the CLI"""
        self._show_continuous_status = False
        print("Goodbye!")
        return True

    def do_exit(self, arg):
        """Exit the CLI"""
        return self.do_quit(arg)

    # ==================== Help Overrides ====================

    def help_takeoff(self):
        print("Takeoff to specified altitude (default 3m)")
        print("Usage: takeoff [altitude_m]")
        print("Example: takeoff 5")

    def help_goto(self):
        print("Navigate to GPS coordinates")
        print("Usage: goto <lat> <lon> [alt]")
        print("Example: goto 48.8566 2.3522 20")

    def default(self, line):
        print(f"Unknown command: {line}")
        print("Type 'help' for available commands")


def run_cli(flight_controller: 'FlightController'):
    """Run the CLI in the current thread"""
    cli = CLI(flight_controller)
    cli.cmdloop()
