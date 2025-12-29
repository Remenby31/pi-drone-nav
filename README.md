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
python -m src.main --cli --usb /dev/ttyACM0
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
```

#### API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| **Status** |||
| `GET` | `/api/status` | Complete telemetry |
| `GET` | `/api/state` | Flight state machine |
| **Missions** |||
| `GET` | `/api/missions` | List all missions |
| `POST` | `/api/missions` | Upload mission |
| `GET` | `/api/missions/{uuid}` | Get mission details |
| `DELETE` | `/api/missions/{uuid}` | Delete mission |
| **Execution** |||
| `POST` | `/api/missions/{uuid}/start` | Start mission (auto-arm) |
| `GET` | `/api/missions/active` | Active mission status |
| `POST` | `/api/missions/active/pause` | Pause mission |
| `POST` | `/api/missions/active/resume` | Resume mission |
| `POST` | `/api/missions/active/stop` | Stop mission |
| `POST` | `/api/missions/active/skip` | Skip current action |
| `POST` | `/api/missions/active/goto/{n}` | Jump to action N |
| **Config** |||
| `GET` | `/api/config` | Get configuration |
| `POST` | `/api/config` | Update configuration |

All flight control goes through missions. Arming/disarming is automatic.

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

## Mission Format (v1.0)

Missions are action-based sequences in JSON format:

```json
{
  "version": "1.0",
  "name": "Survey Mission",
  "defaults": {
    "speed": 5.0,
    "alt": 10.0
  },
  "actions": [
    {"type": "takeoff", "alt": 10},
    {"type": "goto", "lat": 48.8570, "lon": 2.3525, "alt": 15},
    {"type": "goto", "lat": 48.8575, "lon": 2.3530},
    {"type": "hover", "duration": 5},
    {"type": "orient", "heading": 0},
    {"type": "photo"},
    {"type": "goto", "lat": 48.8566, "lon": 2.3522, "alt": 10},
    {"type": "land"}
  ]
}
```

### Available Actions

| Action | Parameters | Description |
|--------|------------|-------------|
| `takeoff` | `alt` | Take off to altitude (meters, relative) |
| `goto` | `lat`, `lon`, `alt?`, `speed?` | Fly to GPS position |
| `hover` | `duration` | Hover in place (seconds) |
| `orient` | `heading` | Set yaw orientation (0-360°, 0=North) |
| `delay` | `duration` | Wait without action (seconds) |
| `photo` | - | Trigger camera |
| `land` | - | Land at current position |
| `rth` | - | Return to home and land |

**Rules:**
- First action should be `takeoff`
- Last action must be `land` or `rth`
- Altitude is relative to takeoff point (barometer-based)
- Segments are implicit (consecutive `goto` actions)

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
│   ├── config.py              # Configuration management
│   ├── main.py                # Entry point
│   ├── drivers/
│   │   ├── msp.py             # MSP protocol driver
│   │   ├── gps_ubx.py         # u-blox GPS driver
│   │   └── serial_manager.py
│   ├── navigation/
│   │   ├── pid.py             # PID controller
│   │   ├── position_controller.py
│   │   ├── velocity_controller.py
│   │   ├── altitude_controller.py
│   │   ├── l1_controller.py   # L1 path following
│   │   └── path_planner.py    # Segment calculation
│   ├── mission/
│   │   ├── models.py          # Actions, Mission, validation
│   │   ├── store.py           # Persistent storage (JSON/UUID)
│   │   └── executor.py        # Action execution state machine
│   ├── flight/
│   │   ├── state_machine.py
│   │   ├── flight_controller.py
│   │   └── takeoff_controller.py
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
└── pyproject.toml
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
