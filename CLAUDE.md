# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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
  - `altitude_controller.py` - Altitude hold and throttle control
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
   - **⚠️ CRITICAL: Channels 2 & 3 are SWAPPED** in Betaflight rcmap
     - Send ch2 → Received at index 3
     - Send ch3 → Received at index 2
     - This is a Betaflight `rcmap` configuration (receiver input remapping)

2. **Timing Characteristics**:
   - **Fast response**: Updates appear immediately (~10ms)
   - No 30ms buffer lag observed (may have been optimized in current firmware version)
   - Actual round-trip latency: <20ms

3. **Motor Control**:
   - RC values correctly propagate through Betaflight's control chain
   - Full end-to-end verified: Pi → MSP_SET_RAW_RC → Betaflight → Motor outputs
   - Motors respond to throttle changes when armed

4. **CRITICAL: Channel 2/3 Swap Impact**:
   - **This MUST be fixed before flight testing** - affects control accuracy
   - If flight_controller sets ch2 for pitch, it will control ch3 (yaw) instead
   - Results in inverted/incorrect axis control
   - Fix options:
     a) **Recommended: Fix in Betaflight configurator** (change rcmap setting)
     b) Compensate in Python (pre-swap before sending)

### Implementation Notes

- ✓ RC control via MSP_SET_RAW_RC is fully functional
- ✓ All 8 channels can be controlled independently
- **⚠️ BLOCKING ISSUE: Channel 2/3 swap must be resolved**
- Recommend fixing in Betaflight rather than adding compensation code
- This is a receiver/hardware configuration issue, not a software bug

### Next Steps for Flight Testing

1. **CRITICAL: Resolve ch2/ch3 swap** in Betaflight configurator or CLI
2. Verify RC control: increase ch0 (throttle) → all motors increase proportionally
3. Verify pitch/roll/yaw channels control correct axes
4. Test with motors spinning in safe conditions (guards/no props)
5. Once verified, proceed with actual flight testing

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

**Channel Mapping (with ch2/ch3 swap):**
```python
# RC array indices for MSP_SET_RAW_RC:
# [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
#   0     1       2        3     4     5     6     7
# Note: Throttle at index 2 goes to ch3, Yaw at index 3 goes to ch2 (Betaflight rcmap)
RC_DISARMED = [1500, 1500, 1000, 1500, 1000, 1800, 1500, 1500]
RC_ARMED    = [1500, 1500, 1000, 1500, 1800, 1800, 1500, 1500]
```

### Remaining Issue

**Channel 2/3 swap** still present in RC readings - needs to be fixed in Betaflight CLI:
```bash
# Check current rcmap
get rcmap

# Fix if needed (default AETR1234)
set rcmap = AETR1234
save
```
