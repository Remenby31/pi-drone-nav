# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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
  - `altitude_controller.py` - Altitude hold, throttle control, **iNav-style landing**
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

All parameters are in `config/default.yaml`. Key values:
- Position Kp: 0.8, Velocity PID: 0.5/0.1/0.05
- Max tilt: 30°, Max horizontal speed: 10 m/s
- Control loop: 50Hz, GPS: 10Hz
- Failsafe: GPS loss → hold, low battery → RTH

## Landing System (iNav-style)

Implementation date: 30 Dec 2025

### Overview

The landing system uses a 3-phase approach inspired by iNav, with accelerometer-based touchdown detection and position hold during descent.

### Landing Phases

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
- Inspired by iNav's `navigationDetectLandingEvent()`

**Fallback method: Altitude + Vario**
- If accelerometer doesn't trigger
- Altitude < 0.3m AND climb rate < 0.2 m/s
- Must persist for 2.0s

### Key Files

| File | Function |
|------|----------|
| `src/navigation/altitude_controller.py` | `LandingPhase` enum, `LandingConfig`, touchdown detection |
| `src/flight/flight_controller.py:675` | `_update_landing()` - main landing loop |
| `config/default.yaml` | `landing:` section with all parameters |

### API

```python
# In AltitudeController
alt_controller.start_landing()           # Initialize landing sequence
alt_controller.get_landing_descent_rate() # Get current phase descent rate
alt_controller.check_touchdown()          # Check for ground contact
alt_controller.should_disarm()            # Check if auto-disarm ready
alt_controller.landing_phase              # Current LandingPhase enum
alt_controller.touchdown_confirmed        # Bool: touchdown detected
```

### Configuration

```yaml
# config/default.yaml
landing:
  # Descent speeds (m/s)
  speed_high: 1.5
  speed_mid: 0.7
  speed_final: 0.3

  # Altitude thresholds (m AGL)
  threshold_high: 5.0
  threshold_mid: 1.0

  # Accelerometer detection
  acc_threshold_g: 1.5
  touchdown_alt_max: 0.5
  touchdown_confirm_time: 0.5

  # Auto-disarm
  auto_disarm: true
  disarm_delay: 2.0
```

### Telemetry

When landing is active, telemetry includes:
```json
{
  "landing": {
    "phase": "PHASE_MID",
    "height_agl": 2.3,
    "descent_rate": -0.7,
    "touchdown": false
  }
}
```

### Comparison with iNav/ArduPilot

| Feature | pi_drone_nav | iNav | ArduPilot |
|---------|-------------|------|-----------|
| 3-phase descent | ✓ | ✓ | ✓ |
| Position hold | ✓ | ✓ | ✓ |
| Accelerometer detection | ✓ | ✓ | ✓ |
| Rangefinder support | ✗ | ✓ | ✓ |
| Terrain following | ✗ | ✓ | ✓ |
| Precision landing | ✗ | ✗ | ✓ |

## GPS Sources

The system supports two GPS sources with automatic fallback at startup:

1. **UBX GPS (primary)** - Direct connection to u-blox GPS on dedicated UART
   - Full data: position, velocity NED (vel_north, vel_east, vel_down), accuracy
   - 10Hz update rate

2. **MSP GPS (fallback)** - GPS data from Betaflight via MSP_RAW_GPS
   - Enabled by `gps.msp_fallback: true` in config (default)
   - Velocity NED calculated from `ground_speed * cos/sin(course)`
   - **Limitation**: `vel_down = 0` (vertical velocity unavailable)
   - Useful when GPS is connected to FC instead of Pi

Telemetry includes `gps_source: "ubx"` or `"msp"` to indicate active source.

## Development Notes

- Betaflight must be in **Angle mode** with MSP enabled on a UART
- Test with `--simulate` flag before hardware testing
- State machine enforces valid transitions only - check `state_machine.py` for allowed transitions
- All serial communication goes through `serial_manager.py` for proper resource handling
- Configuration supports environment variable overrides: `PIDRONE_NAVIGATION_POSITION_KP=1.0`

## Testing

Tests are in `tests/` directory. Key test files:
- `test_pid.py` - PID controller unit tests
- `test_state_machine.py` - Flight state transition tests
- `test_msp.py` - MSP protocol encoding/decoding

Simulation mode (`scripts/simulate.py`) provides a full SITL environment for testing without hardware.

## RC Control via MSP_SET_RAW_RC - Critical Findings

### Problem: MSP_SET_RAW_RC Not Working

**Root Cause Identified (29 Dec 2025):**
- Betaflight can only use ONE receiver type at a time (priority order in `rxInit()`)
- Current config: `serialrx_provider = 9 (CRSF)` - physical receiver active
- This means RX_SERIAL has priority over RX_MSP
- **MSP_SET_RAW_RC cannot override a physical receiver**

### Solutions

**Option 1: Pure MSP Control (Recommended)**
```bash
set serialrx_provider = 0  # NONE (use MSP only)
set feature RX_MSP        # Enable RX_MSP feature
```
Then `MSP_SET_RAW_RC` will have full control.

**Option 2: Keep Physical Receiver + MSP Override** (Hybrid)
- Requires physical AUX channel switch to activate BOXMSPOVERRIDE mode
- Complex: involves chicken-and-egg deadlock (documented in code analysis)
- Not viable for pure MSP control

### MSP Config Verification Commands

```python
# Check receiver type via MSP
MSP_FEATURE_CONFIG (code 36)  # Returns feature bitmask
  - bit 14: FEATURE_RX_MSP
  - bit 3: FEATURE_RX_SERIAL

MSP_RX_CONFIG (code 44)       # Returns serialrx_provider
  - 0 = MSP (RX_MSP)
  - 9 = CRSF (physical receiver)
```

### Key Implementation Details

- **File**: `src/drivers/msp.py` - Contains `set_raw_rc()` function (line 564)
- **Flow**: `src/flight/flight_controller.py:441` - Calls `msp.set_raw_rc()` in 50Hz control loop
- **All RC channels (0-7) must be sent** - including AUX channels for flight mode control
- **No flight mode checking needed** - when using pure MSP, all input comes from Pi

### Verification Results (29 Dec 2025)

**✓ RC Control IS WORKING - All 8 Channels Verified**

Comprehensive testing with RX_MSP enabled confirmed:

1. **Individual Channel Control**: Each of 8 RC channels can be set independently
   - Channels 0, 1, 4-7: Correct 1:1 mapping (send index N → receive index N)
   - **✓ FIXED: Channels 2 & 3 swap compensated in `msp.py:set_raw_rc()`**
     - Betaflight internal order is AERT, but `map AETR1234` causes swap
     - `set_raw_rc()` swaps ch2/ch3 before sending to compensate
     - Caller uses standard AETR order: [Roll, Pitch, Throttle, Yaw, AUX...]

2. **Timing Characteristics**:
   - **Fast response**: Updates appear immediately (~10ms)
   - No 30ms buffer lag observed (may have been optimized in current firmware version)
   - Actual round-trip latency: <20ms

3. **Motor Control**:
   - RC values correctly propagate through Betaflight's control chain
   - Full end-to-end verified: Pi → MSP_SET_RAW_RC → Betaflight → Motor outputs
   - Motors respond to throttle changes when armed

4. **Channel 2/3 Swap - RESOLVED (30 Dec 2025)**:
   - Root cause: Betaflight internal order is AERT, `map AETR1234` swaps ch2/ch3
   - Fix: `msp.py:set_raw_rc()` swaps ch2/ch3 before sending
   - Verified working: send Thr=1200 on ch2 → received correctly on ch2

### Implementation Notes

- ✓ RC control via MSP_SET_RAW_RC is fully functional
- ✓ All 8 channels can be controlled independently
- ✓ Channel 2/3 swap compensated in `msp.py:set_raw_rc()`
- Caller uses standard order: [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]

### Next Steps for Flight Testing

1. Verify RC control: increase ch2 (throttle) → all motors increase proportionally
2. Verify pitch/roll/yaw channels control correct axes
3. Test with motors spinning in safe conditions (guards/no props)
4. Once verified, proceed with actual flight testing

---

## Hardware Testing - DAKEFPVH743 (29 Dec 2025)

### Firmware Compilation

Custom Betaflight firmware compiled for DAKEFPVH743 with MSP support:

```bash
# Build command (from betaflight repo)
make CONFIG=DAKEFPVH743

# Output file
obj/betaflight_2026.6.0-alpha_STM32H743_DAKEFPVH743.hex
```

**Important**: Use `CONFIG=` (not `TARGET=`) for board-specific builds.

### Features Enabled by Default

In `src/config/configs/DAKEFPVH743/config.h`:
```c
#define DEFAULT_FEATURES (FEATURE_GPS | FEATURE_TELEMETRY | FEATURE_LED_STRIP | FEATURE_OSD | FEATURE_ESC_SENSOR)
```

### MSP Control Verification - FULLY WORKING

**Test Date**: 29 Dec 2025
**Hardware**: DAKEFPVH743 (STM32H743)
**Connection**: USB (/dev/ttyACM0)

#### Communication Test Results

```
=== Test MSP sur /dev/ttyACM0 ===
API Version: 1.47 (protocol 0)
Firmware: BTFL
Board: H743
```

#### Motor Control - MSP_SET_MOTOR (Code 214)

**Direct motor control works perfectly!**

```python
# Individual motor control verified
Motor 1: 1000 → 1050 → 1100 → 1150 → 1200 → stop
Motor 2: 1000 → 1050 → 1100 → 1150 → 1200 → stop
Motor 3: 1000 → 1050 → 1100 → 1150 → 1200 → stop
Motor 4: 1000 → 1050 → 1100 → 1150 → 1200 → stop
```

- Motors respond immediately to MSP_SET_MOTOR commands
- Values 1000 = stop, ~1100+ = motors start spinning
- Ramp control (progressive speed changes) works smoothly
- **CRITICAL**: Always send 1000 to all motors at end of test to stop them

#### MSP Override Configuration

For RC channel control via `MSP_SET_RAW_RC`:

```bash
# In Betaflight CLI
set msp_override_channels_mask = 15   # Override channels 1-4 (AETR)
save
```

Mask values:
- `15` = channels 1-4 (Roll, Pitch, Yaw, Throttle)
- `255` = channels 1-8
- `65535` = all 16 channels

### Test Scripts

Test scripts available in `/home/remenby/projects/betaflight/`:

1. **`msp_test.py`** - Basic MSP communication test
2. **`msp_motor_test.py`** - Motor control with safety shutdown

Usage:
```bash
cd /home/remenby/projects/betaflight
.venv/bin/python3 msp_test.py
.venv/bin/python3 msp_motor_test.py
```

### Safety Notes

1. **ALWAYS remove propellers before testing**
2. Motors may not stop automatically - always send stop command (1000)
3. Scripts include automatic motor shutdown on exit and Ctrl+C
4. Battery + USB connection required for motor tests

### MSP Command Summary

| Command | Code | Description |
|---------|------|-------------|
| MSP_MOTOR | 104 | Read motor values |
| MSP_RC | 105 | Read RC channel values |
| MSP_SET_RAW_RC | 200 | Send RC values (needs msp_override) |
| MSP_SET_MOTOR | 214 | Direct motor control (bypasses FC) |

### Integration with pi_drone_nav

The existing `src/drivers/msp.py` already has:
- `set_raw_rc(channels)` - For RC override control
- `set_motor_test(motor_values)` - For direct motor control
- `get_motor_values()` - Read current motor outputs
- `get_rc_channels()` - Read current RC inputs

All functions are verified working with DAKEFPVH743 hardware.

### Full Driver Test Results (29 Dec 2025)

Complete test of all MSP functions in `src/drivers/msp.py`:

```
==================================================
  TEST COMPLET DU DRIVER MSP - pi_drone_nav
==================================================

1. CONNEXION
  ✓ Port série ouvert: /dev/ttyACM0
  ✓ Connexion MSP établie

2. IDENTIFICATION
  ✓ API Version: 1.47
  ✓ FC Variant: BTFL

3. LECTURE ÉTAT (GET)
  ✓ Status: cycle=125µs, cpu=0%
  ✓ Attitude: roll=7.5°, pitch=-0.7°, yaw=2.0°
  ✓ Altitude: 212.73m, vario=0.00m/s
  ✓ IMU: acc=(25,273,2073), gyro=(-1,0,0)
  ✓ Analog: vbat=16.0V, current=0.33A, rssi=0
  ✓ RC Channels: [1500, 1500, 1500, 885, 1500, 1500, 1500, 1500]
  ✓ Motors: [1000, 1000, 1000, 1000]
  ✓ GPS: fix=True, sats=21, lat=43.483061, lon=1.389879
  ✓ GPS Fix: valid=True, vel_n=0.13m/s

4. COMMANDES RC (MSP_SET_RAW_RC)
  ✓ set_raw_rc() envoyé et vérifié

5. CONTRÔLE MOTEURS (MSP_SET_MOTOR)
  ✓ Motor 1: set=1100, read=1100
  ✓ Motor 2: set=1100, read=1100
  ✓ Motor 3: set=1100, read=1100
  ✓ Motor 4: set=1100, read=1100

6. TEST RAMPE MOTEURS
  ✓ Rampe 1000->1150->1000 complète

7. ARRÊT D'URGENCE
  ✓ stop_all_motors(): tous arrêtés
```

**All 14 MSP functions tested and working.**

### Hardware Status During Test

| Parameter | Value | Status |
|-----------|-------|--------|
| Battery | 16.0V | 4S fully charged |
| GPS Satellites | 21 | Excellent fix |
| GPS Position | 43.48°N, 1.39°E | Valid |
| CPU Load | 0% | Idle |
| All 4 Motors | Responding | OK |

### Arming Test Results (29 Dec 2025)

**✓ Full MSP Control Chain Verified:**

1. **RX_MSP Configuration**: `serialrx_provider = NONE` + `FEATURE_RX_MSP`
2. **Continuous RC Required**: Must send MSP_SET_RAW_RC at 50Hz to avoid RX_FAILSAFE
3. **Arming via AUX1**: Setting AUX1 to 1800 arms the drone
4. **Throttle Control**: Throttle ramp (1000 → 1100 → 1000) works while armed
5. **Disarming**: Setting AUX1 back to 1000 disarms

**Channel Mapping:**
```python
# RC array indices for MSP_SET_RAW_RC:
# [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
#   0     1       2        3     4     5     6     7
# With "map AETR1234": ch2→THROTTLE, ch3→YAW (no swap needed)
# Throttle must be ≤1050 (min_check) to arm, use 885 (rx_min_usec)
RC_DISARMED = [1500, 1500, 885, 1500, 1000, 1800, 1500, 1500]
RC_ARMED    = [1500, 1500, 885, 1500, 1800, 1800, 1500, 1500]
```

### ✅ Channel Mapping - CORRECTED (30 Dec 2025)

With `map AETR1234` in Betaflight, the rcmap does:
- ch2 → THROTTLE (internal index 3)
- ch3 → YAW (internal index 2)

**No swap needed in msp.py!** We send `[Roll, Pitch, Throttle, Yaw, AUX...]` directly.

The earlier swap was incorrect and caused throttle to go to the wrong channel.

---

## ARM/DISARM Implementation (30 Dec 2025)

### How ARM Works

The `FlightController.arm()` method now sends actual MSP commands:

```python
# flight_controller.py:783-800
def arm(self) -> bool:
    self._armed = True
    # AUX1 (index 4) = 1800 to arm
    self._rc_channels = [1500, 1500, 1000, 1500, 1800, 1800, 1500, 1500]
    self.msp.set_raw_rc(self._rc_channels)
    return self.state_machine.transition_to(FlightState.ARMED)
```

### How DISARM Works

```python
# AUX1 (index 4) = 1000 to disarm
self._rc_channels = [1500, 1500, 1000, 1500, 1000, 1800, 1500, 1500]
self.msp.set_raw_rc(self._rc_channels)
```

### State Timeouts

Reduced for faster failure detection:

| State | Timeout | Action on timeout |
|-------|---------|-------------------|
| TAKEOFF | 10s | → LANDING |
| LANDING | 20s | → FAILSAFE |

---

## Betaflight RX_MSP Bugs (30 Dec 2025)

### Bug 1: RX_MSP missing from CLI featureNames

**Symptôme**: `feature RX_MSP` retourne `INVALID NAME`

**Cause**: Le nom "RX_MSP" était absent du tableau `featureNames[]` dans `cli/cli.c`, même si `FEATURE_RX_MSP` (bit 14) existe dans `feature.h`.

**Fix appliqué**: Ajouté `_R(FEATURE_RX_MSP, "RX_MSP"),` dans `cli/cli.c:247`

**Issue GitHub**: `docs/betaflight_rx_msp_cli_bug.md`

### Bug 2: RX rate = 0 avec RX_MSP - CORRIGÉ (30 Dec 2025)

**Symptôme**: `status` montre `RX rate: 0` et les flags `RXLOSS MSP` bloquent l'armement.

**Cause racine**: `rxMspFrameStatus()` dans `rx/msp.c` ne mettait pas à jour `lastRcFrameTimeUs`.

**Fix appliqué** dans `rx/msp.c`:
```c
static uint8_t rxMspFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    if (!rxMspFrameDone) {
        return RX_FRAME_PENDING;
    }
    rxMspFrameDone = false;
    rxRuntimeState->lastRcFrameTimeUs = micros();  // FIX
    return RX_FRAME_COMPLETE;
}
```

**Fork Betaflight**: https://github.com/Remenby31/betaflight (branche `fix/rx-msp-cli-support`)

---

## Mission Format v1.0 (30 Dec 2025)

### Action Types

| Type | Parameters | Description |
|------|------------|-------------|
| `takeoff` | `alt` | Takeoff to altitude (m) |
| `goto` | `lat`, `lon`, `alt`, `speed` | Fly to GPS position |
| `hover` | `duration` | Hold position for N seconds |
| `delay` | `duration` | Wait (no position control) |
| `orient` | `heading` | Turn to heading (degrees) |
| `land` | - | Land and disarm |
| `rth` | - | Return to home then land |

### Example Mission

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

### Available Test Missions

| File | Description |
|------|-------------|
| `config/missions/test_hover_2m.json` | Simple 2m hover test |
| `config/missions/sample_mission.json` | Square pattern demo |

---

## Propless Tests (30 Dec 2025)

### Hover Mission Simulation ✓

Test sans hélices validant la chaîne de contrôle complète:

```
1. Connexion OK
2. ARM OK - Motors: [1067, 1063, 1056, 1056]
3. Throttle ramp: 950 → 1000 → 1100 → 1200
   - @1200: Motors [1327, 1083, 1323, 1084]
4. Hover 3s OK
5. Landing sim: 1200 → 1100 → 1000 → 885
6. DISARM OK - Motors: [1000, 1000, 1000, 1000]
```

### Landing Sequence ✓

iNav-style 3-phase landing vérifié:

| Phase | Altitude | Descent Rate |
|-------|----------|--------------|
| PHASE_HIGH | >5m | 1.5 m/s |
| PHASE_MID | 1-5m | 0.7 m/s |
| PHASE_FINAL | <1m | 0.3 m/s |

Touchdown detection:
- **Accelerometer**: Spike >0.5G above baseline for 0.5s
- **Fallback**: Altitude <0.3m + vario <0.2m/s for 2.0s
- **Auto-disarm**: 2.0s after touchdown

### IMU Configuration

```
Accelerometer scale: 2048 LSB/G (±16G range)
Stationary reading: ~1.00G (2048 raw)
```

ACC_SCALE updated in `altitude_controller.py:345`

---

## TODO - Test Glissière (Prochaine étape)

### Objectif
Test sur glissière verticale: décollage → hover 2m → atterrissage

### Séquence automatique
```
1. SPINUP (500ms)     : 0% → 15% throttle
2. THROTTLE_RAMP      : +30%/sec jusqu'à liftoff (max 70%)
3. LIFTOFF_DETECT     : altitude > 0.3m ET climb > 0.2m/s
4. CLIMB              : Monte vers 2m
5. HOVER (3s)         : Stabilise + hover learning
6. LAND               : Descente 3 phases (1.5/0.7/0.3 m/s)
7. TOUCHDOWN          : Détection accéléromètre
8. DISARM             : Auto après 2s
```

### Commandes
```bash
# Démarrer serveur
python -m src.server.main --usb /dev/ttyACM0 -v

# Lancer mission
curl -X POST http://localhost:8080/api/missions/3f6cebcc-6508-4cd1-9d7d-dc195736f38b/start

# ARRÊT D'URGENCE
curl -X POST http://localhost:8080/api/missions/active/stop

# Status
curl http://localhost:8080/api/health
curl http://localhost:8080/api/missions/active
```

### Points de contrôle
- [ ] Drone sur glissière, hélices montées
- [ ] Batterie chargée (>14.8V pour 4S)
- [ ] GPS fix (>5 sats)
- [ ] Serveur démarré et connecté
- [ ] Zone dégagée

### Paramètres clés
| Paramètre | Valeur | Fichier |
|-----------|--------|---------|
| Hover throttle initial | 0.5 | config/hover_throttle.json |
| Target altitude | 2m | mission |
| Hover duration | 3s | mission |
| Touchdown acc threshold | 0.5G | altitude_controller.py |
| Auto-disarm delay | 2s | config/default.yaml |
