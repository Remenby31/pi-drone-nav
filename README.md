# Pi Drone Navigation

Autonomous drone navigation system for Raspberry Pi with Betaflight flight controller.

## Overview

Pi Drone Navigation provides high-level autonomous flight capabilities for drones running Betaflight firmware. It communicates with the flight controller via MSP (MultiWii Serial Protocol) and provides GPS-based navigation features.

### Features

- **Autonomous Navigation**: GPS waypoint navigation with fly-through and hover modes
- **Position Control**: Cascaded control system (Position → Velocity → Angle)
- **Client/Server Architecture**: Server daemon with REST API, lightweight CLI client
- **Betaflight Integration**: Works with Betaflight in Angle mode
- **Mission Planning**: JSON-based action sequences
- **Failsafe**: Configurable GPS loss and low battery handling
- **Simulation**: Full SITL support for testing without hardware

## Architecture

```
┌─────────────────┐         HTTP         ┌──────────────────────────────┐
│   pidrone CLI   │ ◄──────────────────► │     pidrone-server           │
│   (client)      │      localhost:8080  │   (systemd service)          │
└─────────────────┘                      │                              │
                                         │  ┌────────────────────────┐  │
                                         │  │   FlightController     │  │
                                         │  │   - MSP (Betaflight)   │  │
                                         │  │   - GPS (UBX)          │  │
                                         │  │   - Navigation         │  │
                                         │  │   - MissionExecutor    │  │
                                         │  └────────────────────────┘  │
                                         │                              │
                                         │  ┌────────────────────────┐  │
                                         │  │   REST API (Flask)     │  │
                                         │  │   Port 8080            │  │
                                         │  └────────────────────────┘  │
                                         └──────────────────────────────┘
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

```bash
# Clone repository
git clone https://github.com/yourusername/pi-drone-nav.git
cd pi-drone-nav

# Create virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install with all features
pip install -e ".[full]"
```

### Install as Service (Production)

```bash
# Copy systemd service file
sudo cp config/pidrone.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable service at boot
sudo systemctl enable pidrone

# Start service
sudo systemctl start pidrone
```

## Configuration

1. Copy the default configuration:
   ```bash
   mkdir -p ~/.pidrone/missions
   cp config/default.yaml ~/.pidrone/config.yaml
   ```

2. Edit `~/.pidrone/config.yaml` with your settings:
   - Serial ports for MSP and GPS
   - Navigation parameters
   - Failsafe settings

3. Configure Betaflight:
   - Enable MSP on a UART
   - Set ANGLE mode on an AUX channel
   - For pure MSP control: `set serialrx_provider = 0` and enable RX_MSP feature

## Usage

### CLI Commands

The `pidrone` command communicates with the server via REST API:

```bash
# Status
pidrone status              # Show drone status (position, attitude, battery)

# Mission management
pidrone missions            # List stored missions
pidrone upload flight.json  # Upload a mission
pidrone delete my-mission   # Delete a mission

# Mission execution
pidrone start my-mission    # Start mission (auto-arms)
pidrone stop                # Stop mission (land + disarm)
pidrone pause               # Pause mission (hover)
pidrone resume              # Resume paused mission

# Diagnostics
pidrone diag                # Full system diagnostics
pidrone test msp            # Test MSP communication
pidrone test gps            # Test GPS reception
pidrone test sensors        # Test attitude, IMU, battery
pidrone test motors         # Test motors (requires confirmation)
pidrone test preflight      # Run pre-flight checks

# Server management
pidrone serve               # Start server in foreground (dev mode)
pidrone logs                # Show server logs
pidrone logs -f             # Follow server logs
```

### Server Commands

```bash
# Start server (foreground)
pidrone-server --usb /dev/ttyACM0

# Start in simulation mode
pidrone-server --simulate

# Custom port
pidrone-server --port 9000

# With verbose logging
pidrone-server -v --log-file /var/log/pidrone.log
```

### REST API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| **Health** |||
| `GET` | `/api/health` | Health check |
| `GET` | `/api/status` | Complete telemetry |
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
| `POST` | `/api/missions/active/stop` | Stop mission (land + disarm) |
| **Diagnostics** |||
| `GET` | `/api/diagnostics` | Full system diagnostics |
| `POST` | `/api/test/{component}` | Run diagnostic test |

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
│   ├── server/                # Server daemon
│   │   ├── main.py            # Server entry point
│   │   └── api.py             # REST API endpoints
│   ├── cli/                   # CLI client
│   │   ├── main.py            # CLI entry point
│   │   └── client.py          # HTTP client
│   ├── drivers/
│   │   ├── msp.py             # MSP protocol driver
│   │   ├── gps_ubx.py         # u-blox GPS driver
│   │   └── serial_manager.py
│   ├── navigation/
│   │   ├── pid.py             # PID controller
│   │   ├── position_controller.py
│   │   ├── velocity_controller.py
│   │   ├── altitude_controller.py
│   │   └── path_planner.py
│   ├── mission/
│   │   ├── models.py          # Actions, Mission, validation
│   │   ├── store.py           # Persistent storage
│   │   └── executor.py        # Action execution
│   ├── flight/
│   │   ├── state_machine.py
│   │   ├── flight_controller.py
│   │   └── takeoff_controller.py
│   └── utils/
│       ├── geo.py
│       └── logger.py
├── tests/
├── config/
│   ├── default.yaml
│   └── pidrone.service        # Systemd service file
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
