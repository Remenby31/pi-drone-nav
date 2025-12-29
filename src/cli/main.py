#!/usr/bin/env python3
"""
Pi Drone Navigation CLI

Command-line client for pidrone-server.
"""

import argparse
import subprocess
import sys
import time
from typing import Optional

from .client import PidroneClient, ServerError, ConnectionError


# ANSI colors
class Colors:
    RESET = '\033[0m'
    BOLD = '\033[1m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'


def color(text: str, c: str) -> str:
    """Apply color to text"""
    return f"{c}{text}{Colors.RESET}"


def print_error(msg: str):
    """Print error message"""
    print(color(f"Error: {msg}", Colors.RED))


def print_success(msg: str):
    """Print success message"""
    print(color(msg, Colors.GREEN))


def print_warning(msg: str):
    """Print warning message"""
    print(color(msg, Colors.YELLOW))


def prompt_start_server() -> bool:
    """Ask user if they want to start the server"""
    print("Server is not running.")
    try:
        response = input("Do you want to start it? [y/N]: ").strip().lower()
        return response in ('y', 'yes')
    except (EOFError, KeyboardInterrupt):
        return False


def ensure_server_running(client: PidroneClient) -> bool:
    """Ensure server is running, optionally start it"""
    if client.is_server_running():
        return True

    if prompt_start_server():
        print("Starting server...")
        try:
            subprocess.run(['sudo', 'systemctl', 'start', 'pidrone'], check=True)
            # Wait for server to start
            for _ in range(10):
                time.sleep(0.5)
                if client.is_server_running():
                    print_success("Server started")
                    return True
            print_error("Server did not start in time")
        except subprocess.CalledProcessError:
            print_error("Failed to start server (systemctl)")
        except FileNotFoundError:
            print_warning("systemctl not found. Try 'pidrone serve' manually.")

    return False


# ==================== Commands ====================

def cmd_status(client: PidroneClient, args):
    """Show drone status"""
    try:
        status = client.get_status()

        print()
        print(color("=== Drone Status ===", Colors.BOLD))

        # State
        state = status.get('state', {})
        armed = status.get('armed', False)
        state_name = state.get('state', 'UNKNOWN')
        state_color = Colors.GREEN if state_name == 'IDLE' else Colors.YELLOW
        print(f"State: {color(state_name, state_color)}")
        print(f"Armed: {color('YES', Colors.RED) if armed else color('NO', Colors.GREEN)}")
        print(f"Flying: {'Yes' if state.get('is_flying') else 'No'}")

        # Position
        pos = status.get('position', {})
        print()
        print(color("--- Position ---", Colors.CYAN))
        print(f"  Lat: {pos.get('lat', 0):.6f}")
        print(f"  Lon: {pos.get('lon', 0):.6f}")
        print(f"  Alt: {pos.get('alt', 0):.1f} m")
        print(f"  Satellites: {pos.get('satellites', 0)}")

        # Attitude
        att = status.get('attitude', {})
        print()
        print(color("--- Attitude ---", Colors.CYAN))
        print(f"  Roll:  {att.get('roll', 0):.1f}°")
        print(f"  Pitch: {att.get('pitch', 0):.1f}°")
        print(f"  Yaw:   {att.get('yaw', 0):.1f}°")

        # Battery
        battery = status.get('battery', {})
        if battery:
            print()
            print(color("--- Battery ---", Colors.CYAN))
            vbat = battery.get('voltage', 0)
            vbat_color = Colors.GREEN if vbat > 14.0 else Colors.YELLOW if vbat > 12.0 else Colors.RED
            print(f"  Voltage: {color(f'{vbat:.1f}V', vbat_color)}")
            print(f"  Current: {battery.get('current', 0):.1f}A")

        # Mission
        nav = status.get('navigation')
        if nav and nav.get('mission_name'):
            print()
            print(color("--- Navigation ---", Colors.CYAN))
            print(f"  Mission: {nav['mission_name']}")
            print(f"  State: {nav['state']}")
            if nav.get('waypoint_count', 0) > 0:
                print(f"  Progress: {nav['waypoint_index']+1}/{nav['waypoint_count']}")
                print(f"  Distance: {nav.get('distance_to_wp', 0):.1f} m")

        print()

    except ServerError as e:
        print_error(str(e))
    except ConnectionError as e:
        print_error(str(e))


def cmd_missions(client: PidroneClient, args):
    """List all missions"""
    try:
        missions = client.list_missions()

        if not missions:
            print("No missions stored.")
            return

        print()
        print(color("=== Stored Missions ===", Colors.BOLD))
        print()
        print(f"{'Name':<30} {'Actions':<10} {'UUID':<36}")
        print("-" * 76)

        for m in missions:
            print(f"{m['name']:<30} {m.get('action_count', '?'):<10} {m['uuid']:<36}")

        print()

    except ServerError as e:
        print_error(str(e))
    except ConnectionError as e:
        print_error(str(e))


def cmd_upload(client: PidroneClient, args):
    """Upload a mission file"""
    try:
        result = client.upload_mission(args.file)
        print_success(f"Mission '{result['name']}' uploaded ({result['action_count']} actions)")
        print(f"UUID: {result['uuid']}")

    except FileNotFoundError as e:
        print_error(str(e))
    except ServerError as e:
        print_error(str(e))
    except ConnectionError as e:
        print_error(str(e))


def cmd_delete(client: PidroneClient, args):
    """Delete a mission"""
    try:
        client.delete_mission(args.name)
        print_success(f"Mission '{args.name}' deleted")

    except ServerError as e:
        print_error(str(e))
    except ConnectionError as e:
        print_error(str(e))


def cmd_start(client: PidroneClient, args):
    """Start a mission"""
    try:
        result = client.start_mission(args.name)
        print_success(result.get('message', 'Mission started'))

    except ServerError as e:
        print_error(str(e))
    except ConnectionError as e:
        print_error(str(e))


def cmd_stop(client: PidroneClient, args):
    """Stop active mission"""
    try:
        result = client.stop_mission()
        print_success(result.get('message', 'Mission stopped'))

    except ServerError as e:
        print_error(str(e))
    except ConnectionError as e:
        print_error(str(e))


def cmd_pause(client: PidroneClient, args):
    """Pause active mission"""
    try:
        result = client.pause_mission()
        print_success(result.get('message', 'Mission paused'))

    except ServerError as e:
        print_error(str(e))
    except ConnectionError as e:
        print_error(str(e))


def cmd_resume(client: PidroneClient, args):
    """Resume paused mission"""
    try:
        result = client.resume_mission()
        print_success(result.get('message', 'Mission resumed'))

    except ServerError as e:
        print_error(str(e))
    except ConnectionError as e:
        print_error(str(e))


def cmd_diag(client: PidroneClient, args):
    """Show system diagnostics"""
    try:
        diag = client.get_diagnostics()

        print()
        print(color("=" * 60, Colors.BOLD))
        print(color("         SYSTEM DIAGNOSTICS", Colors.BOLD))
        print(color("=" * 60, Colors.BOLD))

        # Flight state
        state = diag.get('flight_state', {})
        print()
        print(color("--- Flight State ---", Colors.CYAN))
        print(f"  State: {state.get('state', 'UNKNOWN')}")
        print(f"  Previous: {state.get('previous_state', 'UNKNOWN')}")
        print(f"  Time in state: {state.get('time_in_state', 0):.1f}s")
        print(f"  Is flying: {state.get('is_flying', False)}")

        # MSP
        msp = diag.get('msp', {})
        print()
        print(color("--- MSP Connection ---", Colors.CYAN))
        connected = msp.get('connected', False)
        print(f"  Connected: {color('Yes', Colors.GREEN) if connected else color('No', Colors.RED)}")
        if connected:
            print(f"  API Version: {msp.get('api_version', 'N/A')}")

        # GPS
        gps = diag.get('gps', {})
        print()
        print(color("--- GPS ---", Colors.CYAN))
        print(f"  Source: {gps.get('source', 'unknown')}")
        if 'satellites' in gps:
            print(f"  Satellites: {gps['satellites']}")
            print(f"  Fix: {gps.get('fix_valid', False)}")
            if gps.get('latitude'):
                print(f"  Position: {gps['latitude']:.6f}, {gps['longitude']:.6f}")

        # Controllers
        controllers = diag.get('controllers', {})
        if controllers.get('altitude'):
            alt = controllers['altitude']
            print()
            print(color("--- Altitude Controller ---", Colors.CYAN))
            print(f"  Target: {alt.get('target', 0):.1f}m")
            print(f"  Current: {alt.get('current', 0):.1f}m")
            print(f"  Hover throttle: {alt.get('hover_throttle', 0):.3f}")

        # Takeoff
        takeoff = diag.get('takeoff', {})
        if takeoff:
            print()
            print(color("--- Takeoff Controller ---", Colors.CYAN))
            print(f"  Phase: {takeoff.get('phase', 'N/A')}")
            print(f"  Target altitude: {takeoff.get('target_altitude', 0)}m")

        # Hover learner
        hover = diag.get('hover_learner', {})
        if hover:
            print()
            print(color("--- Hover Throttle Learner ---", Colors.CYAN))
            print(f"  Learned throttle: {hover.get('hover_throttle', 0):.3f}")
            print(f"  Samples: {hover.get('samples_count', 0)}")

        print()
        print(color("=" * 60, Colors.BOLD))

    except ServerError as e:
        print_error(str(e))
    except ConnectionError as e:
        print_error(str(e))


def cmd_test(client: PidroneClient, args):
    """Run diagnostic test"""
    component = args.component

    try:
        # Motors require confirmation
        confirm = False
        if component == 'motors':
            print_warning("WARNING: Motors will spin! Remove propellers first.")
            try:
                response = input("Type 'yes' to continue: ").strip().lower()
                if response != 'yes':
                    print("Cancelled")
                    return
                confirm = True
            except (EOFError, KeyboardInterrupt):
                print("\nCancelled")
                return

        print(f"Running {component} test...")
        result = client.run_test(component, confirm=confirm)

        print()
        print(color(f"=== Test: {component.upper()} ===", Colors.BOLD))
        print()

        for test in result.get('tests', []):
            name = test.get('name', 'Unknown')
            res = test.get('result', 'UNKNOWN')

            if res == 'PASS':
                symbol = color('PASS', Colors.GREEN)
            elif res == 'WARN':
                symbol = color('WARN', Colors.YELLOW)
            else:
                symbol = color('FAIL', Colors.RED)

            value = test.get('value', test.get('message', ''))
            print(f"  [{symbol}] {name}: {value}")

        print()
        passed = result.get('passed', False)
        if passed:
            print_success("Overall: PASS")
        else:
            print_error("Overall: FAIL")

    except ServerError as e:
        print_error(str(e))
    except ConnectionError as e:
        print_error(str(e))


def cmd_logs(client: PidroneClient, args):
    """Show server logs"""
    cmd = ['journalctl', '-u', 'pidrone', '--no-pager']
    if args.follow:
        cmd.append('-f')
    else:
        cmd.extend(['-n', '50'])

    try:
        subprocess.run(cmd)
    except FileNotFoundError:
        print_error("journalctl not found. Logs are only available with systemd.")
    except KeyboardInterrupt:
        pass


def cmd_serve(client: PidroneClient, args):
    """Start server in foreground"""
    print("Starting server...")
    try:
        from ..server.main import main as server_main
        server_main()
    except KeyboardInterrupt:
        print("\nServer stopped")


# ==================== Main ====================

def main():
    """Main entry point for pidrone CLI"""
    parser = argparse.ArgumentParser(
        prog='pidrone',
        description='Pi Drone Navigation CLI',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  pidrone status              Show drone status
  pidrone missions            List stored missions
  pidrone upload flight.json  Upload a mission
  pidrone start my-mission    Start a mission
  pidrone stop                Stop active mission
  pidrone diag                Run diagnostics
  pidrone test gps            Test GPS
  pidrone serve               Start server (dev mode)
"""
    )

    parser.add_argument(
        '--url',
        default='http://localhost:8080',
        help='Server URL (default: http://localhost:8080)'
    )

    subparsers = parser.add_subparsers(dest='command', help='Command')

    # status
    subparsers.add_parser('status', help='Show drone status')

    # missions
    subparsers.add_parser('missions', help='List stored missions')

    # upload
    p = subparsers.add_parser('upload', help='Upload a mission file')
    p.add_argument('file', help='Mission JSON file path')

    # delete
    p = subparsers.add_parser('delete', help='Delete a mission')
    p.add_argument('name', help='Mission name or UUID')

    # start
    p = subparsers.add_parser('start', help='Start a mission')
    p.add_argument('name', help='Mission name or UUID')

    # stop
    subparsers.add_parser('stop', help='Stop active mission (land and disarm)')

    # pause
    subparsers.add_parser('pause', help='Pause active mission')

    # resume
    subparsers.add_parser('resume', help='Resume paused mission')

    # diag
    subparsers.add_parser('diag', help='Show system diagnostics')

    # test
    p = subparsers.add_parser('test', help='Run diagnostic test')
    p.add_argument('component', choices=['msp', 'gps', 'sensors', 'motors', 'rc', 'preflight'],
                   help='Component to test')

    # logs
    p = subparsers.add_parser('logs', help='Show server logs')
    p.add_argument('-f', '--follow', action='store_true', help='Follow log output')

    # serve
    subparsers.add_parser('serve', help='Start server in foreground (dev mode)')

    args = parser.parse_args()

    # No command - show help
    if not args.command:
        parser.print_help()
        return 0

    # Create client
    client = PidroneClient(args.url)

    # Commands that don't need server
    if args.command == 'serve':
        cmd_serve(client, args)
        return 0

    if args.command == 'logs':
        cmd_logs(client, args)
        return 0

    # Commands that need server
    if not ensure_server_running(client):
        return 1

    commands = {
        'status': cmd_status,
        'missions': cmd_missions,
        'upload': cmd_upload,
        'delete': cmd_delete,
        'start': cmd_start,
        'stop': cmd_stop,
        'pause': cmd_pause,
        'resume': cmd_resume,
        'diag': cmd_diag,
        'test': cmd_test,
    }

    if args.command in commands:
        commands[args.command](client, args)

    return 0


if __name__ == '__main__':
    sys.exit(main())
