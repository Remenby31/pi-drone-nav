# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Related Documentation

- **[TODO.md](TODO.md)** - Current tasks and urgent issues
- **[TIMELINE.md](TIMELINE.md)** - History of tests, incidents, and discoveries

## Betaflight Source Reference

When unsure about Betaflight CLI commands or settings, check the source code at `../betaflight/`. Key files:
- `src/main/cli/cli.c` - CLI commands (e.g., `map` not `rcmap`)
- `src/main/pg/` - Parameter groups and settings
- `src/main/msp/msp.c` - MSP protocol implementation

## Project Overview

Pi Drone Navigation is an autonomous drone navigation system for Raspberry Pi with Betaflight flight controller. It provides GPS-based waypoint navigation by sending MSP commands to Betaflight running in Angle mode.

## Commands

```bash
# Install with dev dependencies
pip install -e ".[dev]"

# Install with all features (Flask, MAVLink, PyYAML)
pip install -e ".[full]"

# Run tests
pytest

# Run with coverage
pytest --cov=src

# Run single test file
pytest tests/test_pid.py

# Type checking
mypy src/

# Run application
python -m src.main --cli          # CLI mode
python -m src.main --rest-api     # REST API on port 8080
python -m src.main --mavlink      # MAVLink on UDP 14550
python -m src.main --simulate     # Simulation mode (no hardware)
```

## Architecture

### Control Cascade

The system uses a cascaded control architecture inspired by iNav:

```
GPS Position (10Hz)
       ↓
Position Controller (P)  →  Velocity Setpoint
       ↓
Velocity Controller (PID) → Acceleration Command
       ↓
Jerk Limiter + Accel-to-Angle → Pitch/Roll Commands
       ↓
MSP_SET_RAW_RC → Betaflight (Angle Mode) → Motors
```

### Module Structure

- **`src/drivers/`** - Hardware communication
  - `msp.py` - MSP v1/v2 protocol for Betaflight (UART/USB)
  - `gps_ubx.py` - u-blox UBX binary protocol (M8N/M10)
  - `serial_manager.py` - Connection management

- **`src/navigation/`** - Control algorithms
  - `pid.py` - PID with anti-windup, D-term filtering, feed-forward
  - `position_controller.py` - GPS position → velocity setpoint
  - `velocity_controller.py` - Velocity → acceleration → pitch/roll angles
  - `altitude_controller.py` - Altitude hold, throttle control, iNav-style landing
  - `takeoff_controller.py` - iNav-style takeoff with velocity PID and gyro liftoff detection
  - `waypoint_navigator.py` - Mission execution with fly-through/hover modes

- **`src/flight/`** - Flight orchestration
  - `flight_controller.py` - Main 50Hz control loop
  - `state_machine.py` - Flight states with enforced transitions (IDLE → ARMED → TAKING_OFF → FLYING → etc.)

- **`src/interfaces/`** - User interfaces (all optional)
  - `cli.py` - Interactive command line
  - `rest_api.py` - Flask REST API (port 8080)
  - `mavlink_bridge.py` - QGroundControl compatibility (UDP 14550)

- **`src/config.py`** - YAML config with environment variable overrides (`PIDRONE_*` prefix)

### Key Configuration

All parameters are in `config/default.yaml`. Values aligned with iNav defaults (navigation.c:4898-4945):

**Position/Velocity XY:**
- Position Kp: 0.65 (iNav: 65/100)
- Velocity PID: Kp=2.0, Ki=0.15, Kd=1.0 (iNav: 40/20, 15/100, 100/100)
- Jerk limit: 17 m/s³ (iNav: 1700 cm/s³)

**Altitude (Velocity Z):**
- Position Z Kp: 0.5 (iNav: 50/100)
- Velocity Z PID: Kp=1.5, Ki=2.5, Kd=0.1 (iNav: 100/66.7, 50/20, 10/100)

**Other:**
- Max tilt: 30°, Max horizontal speed: 10 m/s
- Control loop: 50Hz, GPS: 10Hz
- Failsafe: GPS loss → hold, low battery → RTH

---

## Landing System (iNav-style)

### Phases

```
PHASE_HIGH (>5m AGL)   →  descent 1.5 m/s  (fast)
        ↓
PHASE_MID (1-5m AGL)   →  descent 0.7 m/s  (medium)
        ↓
PHASE_FINAL (<1m AGL)  →  descent 0.3 m/s  (slow) + touchdown detection
        ↓
TOUCHDOWN              →  motors idle, wait 2s
        ↓
AUTO-DISARM            →  transition to IDLE
```

### Touchdown Detection

**Primary method: Accelerometer spike**
- Reads IMU via `MSP_RAW_IMU` during landing
- Detects acceleration spike > 1.5G (above baseline)
- Must persist for 0.5s at altitude < 0.5m AGL

**Fallback method: Altitude + Vario**
- Altitude < 0.3m AND climb rate < 0.2 m/s for 2.0s

### Key Files

| File | Function |
|------|----------|
| `src/navigation/altitude_controller.py` | `LandingPhase` enum, touchdown detection |
| `src/flight/flight_controller.py` | `_update_landing()` - main landing loop |
| `config/default.yaml` | `landing:` section with all parameters |

---

## Takeoff System (iNav-style)

### States

```
SPINUP (500ms)    →  Linear ramp 0% → 15%
       ↓
CLIMBING          →  Velocity PID active: throttle = hover + PID(target - actual)
       ↓
COMPLETE/ABORTED  →  Success or timeout/tilt abort
```

### Liftoff Detection

Both conditions must be true for 200ms:
```python
throttle > hover_throttle + 0.05  # Throttle above hover
AND
gyro_magnitude > 7.0 deg/s        # Drone actually moving
```

### PID Gains (aligned with iNav)

| Parameter | Value |
|-----------|-------|
| Kp | 1.5 |
| Ki | 2.5 |
| Kd | 0.1 |
| Filter cutoff | 5 Hz |

### Key Files

| File | Function |
|------|----------|
| `src/navigation/takeoff_controller.py` | TakeoffController, TakeoffState |
| `src/flight/flight_controller.py` | `_update_takeoff()` |
| `config/default.yaml` | `takeoff:` section |

---

## GPS Sources

The system supports two GPS sources with automatic fallback:

1. **UBX GPS (primary)** - Direct connection to u-blox GPS on dedicated UART
   - Full data: position, velocity NED, accuracy
   - 10Hz update rate

2. **MSP GPS (fallback)** - GPS data from Betaflight via MSP_RAW_GPS
   - Enabled by `gps.msp_fallback: true` in config
   - **Limitation**: `vel_down = 0` (vertical velocity unavailable)

---

## RC Control via MSP_SET_RAW_RC

### Configuration requise

```bash
# Betaflight CLI
set serialrx_provider = 0  # NONE (use MSP only)
set feature RX_MSP         # Enable RX_MSP feature
```

### Channel Mapping

Avec `map AETR1234`:
```python
# [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
RC_DISARMED = [1500, 1500, 885, 1500, 1000, 1800, 1500, 1500]
RC_ARMED    = [1500, 1500, 885, 1500, 1800, 1800, 1500, 1500]
```

### Important
- Must send MSP_SET_RAW_RC at 50Hz to avoid RX_FAILSAFE
- AUX1 = 1800 to arm, 1000 to disarm
- Throttle must be ≤1050 to arm (use 885)

---

## Mission Format v1.0

### Action Types

| Type | Parameters | Description |
|------|------------|-------------|
| `takeoff` | `alt` | Takeoff to altitude (m) |
| `goto` | `lat`, `lon`, `alt`, `speed` | Fly to GPS position |
| `hover` | `duration` | Hold position for N seconds |
| `land` | - | Land and disarm |
| `rth` | - | Return to home then land |

### Example

```json
{
  "version": "1.0",
  "name": "Test Hover 2m",
  "actions": [
    {"type": "takeoff", "alt": 2.0},
    {"type": "hover", "duration": 3.0},
    {"type": "land"}
  ]
}
```

---

## Flight Data Logging

### Fonctionnement

- **ARM** → démarre automatiquement le logging
- **DISARM** → arrête le logging
- **Fichiers** : `~/.pidrone/logs/flight_YYYYMMDD_HHMMSS_<mission>.csv`

### Données loggées (50Hz)

| Catégorie | Colonnes |
|-----------|----------|
| Time/State | time_s, loop_idx, flight_state, mission_action |
| GPS | lat, lon, alt_msl, gps_sats, vel_n, vel_e, vel_d |
| Attitude | roll, pitch, yaw |
| Altitude | alt_baro, alt_target, climb_rate, height_agl |
| Commands | throttle, roll_cmd, pitch_cmd, yaw_cmd |
| Takeoff | takeoff_state, takeoff_liftoff, takeoff_throttle |
| Landing | landing_phase, landing_descent_rate, touchdown |
| IMU Raw | acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z |
| Motors | motor1, motor2, motor3, motor4 |
| Battery | vbat, current_a |

---

## Raspberry Pi Connection

```
IP: 192.168.1.114
User: drone
Password: drone
```

### Commandes daemon

```bash
sudo systemctl status pidrone    # État
sudo systemctl restart pidrone   # Redémarrer
sudo systemctl stop pidrone      # Arrêter
sudo journalctl -u pidrone -f    # Logs temps réel
```

### API REST

| Endpoint | Description |
|----------|-------------|
| `GET /api/health` | Status rapide |
| `GET /api/status` | Status complet |
| `GET /api/missions` | Liste missions |
| `POST /api/missions/<uuid>/start` | Lancer mission |
| `POST /api/missions/active/stop` | Stop + land + disarm |
| `POST /api/emergency/disarm` | **Coupe moteurs immédiat** |

---

## Development Notes

- Betaflight must be in **Angle mode** with MSP enabled on a UART
- Test with `--simulate` flag before hardware testing
- State machine enforces valid transitions only - check `state_machine.py`
- All serial communication goes through `serial_manager.py`
- Configuration supports environment variable overrides: `PIDRONE_NAVIGATION_POSITION_KP=1.0`

### Betaflight CLI Mode - Important

When entering Betaflight CLI mode via serial (`#`), the FC stops responding to MSP binary commands. **Always exit CLI properly** with `exit` or power cycle required.

---

## Testing

Tests are in `tests/` directory:
- `test_pid.py` - PID controller unit tests
- `test_state_machine.py` - Flight state transition tests
- `test_msp.py` - MSP protocol encoding/decoding

Simulation mode (`scripts/simulate.py`) provides a full SITL environment.
