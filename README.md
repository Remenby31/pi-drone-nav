# Pi Drone Navigation

Autonomous drone navigation system for Raspberry Pi with Betaflight flight controller.

## Overview

Pi Drone Navigation provides high-level autonomous flight capabilities for drones running Betaflight firmware. It communicates with the flight controller via MSP (MultiWii Serial Protocol) and provides GPS-based navigation features.

### Features

- **Autonomous Navigation**: GPS waypoint navigation with fly-through and hover modes
- **Position Control**: Cascaded control system (Position → Velocity → Angle)
- **Multiple Interfaces**: CLI, REST API, MAVLink (QGroundControl compatible)
- **Betaflight Integration**: Works with Betaflight in Angle mode
- **Mission Planning**: JSON-based mission files
- **Failsafe**: Configurable GPS loss and low battery handling
- **Simulation**: Full SITL support for testing without hardware

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Raspberry Pi Zero 2 W                    │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────────────┐ │
│  │   CLI   │  │REST API │  │MAVLink  │  │   Mission       │ │
│  └────┬────┘  └────┬────┘  └────┬────┘  │   Planner       │ │
│       │            │            │       └────────┬────────┘ │
│       └────────────┴────────────┴────────────────┘          │
│                           │                                  │
│                  ┌────────┴────────┐                        │
│                  │ Flight Controller│                        │
│                  │   (Main Logic)   │                        │
│                  └────────┬────────┘                        │
│         ┌─────────────────┼─────────────────┐               │
│         │                 │                 │               │
│  ┌──────┴──────┐  ┌───────┴───────┐  ┌──────┴──────┐       │
│  │   Position  │  │   Velocity    │  │  Altitude   │       │
│  │  Controller │  │  Controller   │  │ Controller  │       │
│  └──────┬──────┘  └───────┬───────┘  └──────┬──────┘       │
│         │                 │                 │               │
│         └─────────────────┴─────────────────┘               │
│                           │                                  │
│         ┌─────────────────┼─────────────────┐               │
│         │                 │                 │               │
│  ┌──────┴──────┐  ┌───────┴───────┐  ┌──────┴──────┐       │
│  │ MSP Driver  │  │  GPS Driver   │  │   Filters   │       │
│  │ (Betaflight)│  │   (u-blox)    │  │             │       │
│  └──────┬──────┘  └───────┬───────┘  └─────────────┘       │
│         │                 │                                  │
└─────────┼─────────────────┼──────────────────────────────────┘
          │                 │
     ┌────┴────┐       ┌────┴────┐
     │ UART/USB│       │  UART   │
     └────┬────┘       └────┬────┘
          │                 │
     ┌────┴────┐       ┌────┴────┐
     │Betaflight│      │  u-blox │
     │   FC    │       │  GPS    │
     └─────────┘       └─────────┘
```

## Requirements

### Hardware

- Raspberry Pi Zero 2 W (or any Pi with UART)
- Betaflight-compatible flight controller
- u-blox GPS module (M8N or M10 recommended)
- UART connections for FC and GPS

### Software

- Python 3.9+
- Betaflight 4.3+ (with MSP enabled)

## Installation

### Quick Install

```bash
git clone https://github.com/yourusername/pi-drone-nav.git
cd pi-drone-nav
./scripts/install.sh
```

### Manual Install

```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Install package
pip install -e .
```

## Configuration

1. Copy the default configuration:
   ```bash
   cp config/default.yaml config.yaml
   ```

2. Edit `config.yaml` with your settings:
   - Serial ports for MSP and GPS
   - Navigation parameters
   - Failsafe settings

3. Configure Betaflight:
   - Enable MSP on a UART
   - Set ANGLE mode on an AUX channel
   - See `config/betaflight_setup.txt` for CLI commands

## Usage

### CLI Mode

```bash
# Activate virtual environment
source .venv/bin/activate

# Run with CLI interface
python -m src.main --usb /dev/ttyACM0
```

#### Available Commands

| Command | Description |
|---------|-------------|
| **Status** | |
| `status` | Show drone status (position, attitude, GPS, state) |
| `watch [on\|off]` | Toggle continuous status display |
| `diag` | Full system diagnostics |
| **Flight Control** | |
| `arm` | Arm the drone |
| `disarm` | Disarm the drone (only when not flying) |
| `takeoff [alt]` | Takeoff to altitude (default: 3m) |
| `land` | Initiate landing |
| `hold` | Hold current position (GPS) |
| `rth` | Return to home |
| **Navigation** | |
| `goto <lat> <lon> [alt]` | Fly to GPS coordinates |
| `mission load <file>` | Load mission from JSON file |
| `mission start` | Start loaded mission |
| `mission pause` | Pause mission |
| `mission resume` | Resume mission |
| `mission abort` | Abort mission |
| `mission status` | Show mission status |
| **Diagnostics** | |
| `test msp` | Test MSP communication |
| `test rc` | Test RC channel control |
| `test motors` | Test motors individually (WARNING: motors spin!) |
| `test gps` | Test GPS reception |
| `test sensors` | Test attitude, altitude, IMU, battery |
| `test preflight` | Run pre-flight checks |
| `test all` | Run all safe tests (excludes motors) |
| **Configuration** | |
| `config` | Show all configuration |
| `config <section>` | Show section (e.g., `config navigation`) |
| `config <section.key> <value>` | Set value |
| `calibrate acc` | Calibrate accelerometer |
| `calibrate mag` | Calibrate magnetometer |
| `reboot` | Reboot flight controller |
| `quit` / `exit` | Exit CLI |

### REST API Mode

```bash
python -m src.main --rest-api

# API Endpoints:
# GET  /api/status     - Get drone status
# POST /api/arm        - Arm drone
# POST /api/disarm     - Disarm drone
# POST /api/takeoff    - Takeoff {"altitude": 10}
# POST /api/land       - Land
# POST /api/goto       - Go to position {"lat": 48.8, "lon": 2.3, "alt": 15}
# POST /api/hold       - Position hold
# POST /api/rth        - Return to home
# GET  /api/mission    - Get mission status
# POST /api/mission    - Load mission
# POST /api/mission/start - Start mission
```

### MAVLink Mode

```bash
python -m src.main --mavlink

# Connect with QGroundControl on UDP port 14550
```

### Simulation Mode

```bash
# Terminal 1: Start simulator
python scripts/simulate.py

# Terminal 2: Run navigation
python -m src.main --simulate
```

## Mission Files

Missions are defined in JSON format:

```json
{
  "name": "Survey Mission",
  "waypoints": [
    {"lat": 48.8566, "lon": 2.3522, "alt": 10, "action": "TAKEOFF"},
    {"lat": 48.8570, "lon": 2.3525, "alt": 15, "action": "FLY_THROUGH"},
    {"lat": 48.8575, "lon": 2.3530, "alt": 15, "action": "HOVER", "hover_time": 5},
    {"lat": 48.8566, "lon": 2.3522, "alt": 10, "action": "LAND"}
  ],
  "default_speed": 5.0,
  "return_to_home": true
}
```

## Safety

⚠️ **WARNING**: This is experimental software for autonomous drone flight.

- Always test in simulation first
- Remove propellers during initial bench testing
- Start with conservative settings
- Have a manual override ready
- Follow local drone regulations
- Never fly over people or restricted areas

## Development

### Running Tests

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=src

# Run specific test file
pytest tests/test_pid.py
```

### Project Structure

```
pi_drone_nav/
├── src/
│   ├── __init__.py
│   ├── config.py           # Configuration management
│   ├── main.py             # Entry point
│   ├── drivers/
│   │   ├── msp.py          # MSP protocol driver
│   │   ├── gps_ubx.py      # u-blox GPS driver
│   │   └── serial_manager.py
│   ├── navigation/
│   │   ├── pid.py          # PID controller
│   │   ├── position_controller.py
│   │   ├── velocity_controller.py
│   │   ├── altitude_controller.py
│   │   └── waypoint_navigator.py
│   ├── flight/
│   │   ├── state_machine.py
│   │   └── flight_controller.py
│   ├── interfaces/
│   │   ├── cli.py
│   │   ├── rest_api.py
│   │   └── mavlink_bridge.py
│   └── utils/
│       ├── geo.py
│       └── logger.py
├── tests/
├── config/
├── scripts/
├── requirements.txt
└── setup.py
```

## Control System

The navigation uses a cascaded control architecture inspired by iNav:

```
Position Error → [P Controller] → Velocity Setpoint
                                        ↓
Velocity Error → [PID Controller] → Acceleration Command
                                        ↓
                    [Accel to Angle] → Pitch/Roll Commands
                                        ↓
                         [MSP_SET_RAW_RC] → Betaflight
```

Key parameters:
- Position Kp: 0.8 (converts meters to m/s)
- Velocity Kp/Ki/Kd: 0.5/0.1/0.05
- Max tilt: 30°
- Jerk limit: 1.7 m/s³

## License

MIT License - See LICENSE file for details.

## Acknowledgments

- [Betaflight](https://github.com/betaflight/betaflight) - Flight controller firmware
- [iNav](https://github.com/iNavFlight/inav) - Navigation inspiration
- [ArduPilot](https://github.com/ArduPilot/ardupilot) - Reference implementation
