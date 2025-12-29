# Pi Drone Navigation - Plan de Développement

## Vue d'ensemble

Système de vol autonome pour Raspberry Pi Zero 2 W + Betaflight en mode Angle.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         RASPBERRY PI ZERO 2 W                           │
│                                                                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │  GPS UBX    │  │  Position   │  │  Velocity   │  │  Altitude   │    │
│  │  Driver     │─►│  Controller │─►│  Controller │  │  Controller │    │
│  │  (10Hz)     │  │  (P)        │  │  (PID)      │  │  (PID)      │    │
│  └─────────────┘  └─────────────┘  └──────┬──────┘  └──────┬──────┘    │
│        │                                  │                 │           │
│        │                                  ▼                 ▼           │
│        │                          ┌─────────────────────────────┐       │
│        │                          │     Flight Controller       │       │
│        │                          │     (State Machine)         │       │
│        │                          └──────────────┬──────────────┘       │
│        │                                         │                      │
│        │              ┌──────────────────────────┼──────────────────┐   │
│        │              │                          │                  │   │
│        ▼              ▼                          ▼                  ▼   │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │  Waypoint   │  │  REST API   │  │  WebSocket  │  │  MAVLink    │    │
│  │  Navigator  │  │  :8080      │  │  :8081      │  │  UDP:14550  │    │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘    │
│                                                                         │
└───────────────────────────────────┬─────────────────────────────────────┘
                                    │
                              UART / USB
                              MSP Protocol
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                           BETAFLIGHT FC                                 │
│                                                                         │
│   Mode: ANGLE (stabilisation automatique)                               │
│   Failsafe: GPS Rescue                                                  │
│   Input: MSP_SET_RAW_RC (200)                                           │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## Cascade de Contrôle

```
GPS Position (10Hz)
      │
      ▼
┌──────────────────┐
│ Position Error   │  target_pos - current_pos
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Position Ctrl    │  vel_target = error × Kp_pos
│ (P Controller)   │  Kp_pos = 1.0
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Velocity Error   │  target_vel - current_vel
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Velocity Ctrl    │  accel_target = PID(vel_error)
│ (PID Controller) │  Kp=2.0, Ki=0.5, Kd=0.1
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Jerk Limiter     │  limit Δaccel to 1.7 m/s³
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Accel → Angles   │  pitch = atan2(-accel_fwd, g)
│ (NED → Body)     │  roll = atan2(accel_right, g)
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ RC Commands      │  roll/pitch → 1000-2000
│ MSP_SET_RAW_RC   │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Betaflight       │  Angle mode → Rate → Motors
│ (Angle Mode)     │
└──────────────────┘
```

## Structure des Fichiers

```
pi_drone_nav/
├── src/
│   ├── __init__.py                 # Package init
│   ├── config.py                   # Configuration management
│   ├── main.py                     # Point d'entrée principal
│   │
│   ├── drivers/
│   │   ├── __init__.py
│   │   ├── msp.py                  # Client MSP Betaflight
│   │   ├── gps_ubx.py              # Driver GPS u-blox
│   │   └── serial_manager.py       # Gestion ports série
│   │
│   ├── navigation/
│   │   ├── __init__.py
│   │   ├── pid.py                  # Contrôleur PID
│   │   ├── position_controller.py  # Position → Velocity
│   │   ├── velocity_controller.py  # Velocity → Angles
│   │   ├── altitude_controller.py  # Altitude → Throttle
│   │   └── waypoint_navigator.py   # Navigation waypoints
│   │
│   ├── flight/
│   │   ├── __init__.py
│   │   ├── state_machine.py        # Machine à états
│   │   └── flight_controller.py    # Contrôleur principal
│   │
│   ├── interfaces/
│   │   ├── __init__.py
│   │   ├── cli.py                  # Interface terminal
│   │   ├── rest_api.py             # API REST Flask
│   │   ├── websocket_server.py     # WebSocket temps réel
│   │   └── mavlink_bridge.py       # Bridge MAVLink
│   │
│   └── utils/
│       ├── __init__.py
│       ├── geo.py                  # Conversions GPS
│       ├── filters.py              # Filtres numériques
│       └── logger.py               # Configuration logging
│
├── tests/
│   ├── __init__.py
│   ├── test_msp.py                 # Tests driver MSP
│   ├── test_gps.py                 # Tests driver GPS
│   ├── test_pid.py                 # Tests contrôleur PID
│   ├── test_navigation.py          # Tests navigation
│   └── test_state_machine.py       # Tests machine à états
│
├── config/
│   ├── default.yaml                # Configuration par défaut
│   └── betaflight_setup.txt        # Commandes CLI Betaflight
│
├── scripts/
│   ├── install.sh                  # Installation
│   └── simulate.py                 # Simulation SITL
│
├── docs/
│   └── wiring.md                   # Schéma de câblage
│
├── PLAN.md                         # Ce fichier
├── README.md                       # Documentation
├── requirements.txt                # Dépendances Python
└── setup.py                        # Installation package
```

## Paramètres de Navigation

### Gains PID (inspirés iNav)

| Paramètre | Valeur | Description |
|-----------|--------|-------------|
| pos_p_gain | 1.0 | Position → Velocity |
| vel_p_gain | 2.0 | Velocity P |
| vel_i_gain | 0.5 | Velocity I |
| vel_d_gain | 0.1 | Velocity D |
| alt_p_gain | 0.5 | Altitude → Climb rate |
| climb_p_gain | 5.0 | Climb rate → Throttle |

### Limites

| Paramètre | Valeur | Description |
|-----------|--------|-------------|
| max_horizontal_speed | 15.0 m/s | Vitesse horizontale max |
| max_vertical_speed | 3.0 m/s | Vitesse verticale max |
| max_horizontal_accel | 5.0 m/s² | Accélération horizontale max |
| jerk_limit | 1.7 m/s³ | Limite de jerk |
| max_bank_angle | 30.0° | Angle max d'inclinaison |

## États de Vol

```
IDLE ──────────────► PREFLIGHT ─────► ARMED
  ▲                                     │
  │                                     ▼
  │                                 TAKEOFF
  │                                     │
  │    ┌────────────────────────────────┼───────────────────────────┐
  │    │                                ▼                           │
  │    │   ┌───────────────────────► HOVER ◄────────────────────┐   │
  │    │   │                           │ ▲                      │   │
  │    │   │                           │ │                      │   │
  │    │   │                           ▼ │                      │   │
  │    │   │                    POSITION_HOLD                   │   │
  │    │   │                           │                        │   │
  │    │   │              ┌────────────┼────────────┐           │   │
  │    │   │              ▼            ▼            ▼           │   │
  │    │   │          FLYING       MISSION         RTH          │   │
  │    │   │              │            │            │           │   │
  │    │   │              └────────────┼────────────┘           │   │
  │    │   │                           │                        │   │
  │    │   │                           ▼                        │   │
  │    │   │                       LANDING                      │   │
  │    │   │                           │                        │   │
  │    │   │                           ▼                        │   │
  │    │   └─────────────────────── LANDED ─────────────────────┘   │
  │    │                               │                            │
  │    │                               ▼                            │
  └────┴───────────────────────────── IDLE ◄────────────────────────┘
                                       ▲
                                       │
                              FAILSAFE ─┘
```

## Protocole MSP Utilisé

### Commandes de lecture

| Commande | Code | Usage |
|----------|------|-------|
| MSP_API_VERSION | 1 | Vérification connexion |
| MSP_ATTITUDE | 108 | Roll, Pitch, Yaw |
| MSP_ALTITUDE | 109 | Altitude baro, vario |
| MSP_RAW_GPS | 106 | Position GPS (backup) |
| MSP_ANALOG | 110 | Batterie, RSSI |
| MSP_STATUS | 101 | État FC |

### Commandes d'écriture

| Commande | Code | Usage |
|----------|------|-------|
| MSP_SET_RAW_RC | 200 | Injection RC |
| MSP_SET_ARMING_DISABLED | 99 | Contrôle arm |

## Format des Missions (JSON)

```json
{
  "name": "Test Mission",
  "default_speed": 10.0,
  "default_altitude": 10.0,
  "return_to_home": true,
  "waypoints": [
    {
      "latitude": 48.8566,
      "longitude": 2.3522,
      "altitude": 15.0,
      "action": "FLY_THROUGH",
      "radius": 2.0
    },
    {
      "latitude": 48.8570,
      "longitude": 2.3530,
      "altitude": 15.0,
      "action": "FLY_THROUGH"
    }
  ]
}
```

## API REST

### Endpoints

| Méthode | Endpoint | Description |
|---------|----------|-------------|
| GET | /status | État complet |
| GET | /telemetry | Télémétrie temps réel |
| POST | /arm | Armer le drone |
| POST | /disarm | Désarmer |
| POST | /takeoff | Décollage |
| POST | /land | Atterrissage |
| POST | /goto | Aller à position GPS |
| POST | /hold | Maintien position |
| POST | /rth | Retour maison |
| POST | /mission | Charger mission |
| POST | /mission/start | Démarrer mission |
| POST | /mission/pause | Pause mission |
| POST | /mission/abort | Annuler mission |

## Configuration Betaflight

```
# CLI Betaflight

# Activer MSP comme source RC
set serialrx_provider = MSP

# Mode Angle permanent (channel 5 toujours ON)
aux 0 0 0 900 2100

# GPS Rescue comme failsafe
set failsafe_procedure = GPS-RESCUE
set gps_rescue_min_sats = 8

# Limites pour vol autonome
set angle_limit = 45
```

## Tests

### Tests Unitaires

1. **test_msp.py**
   - Encodage/décodage trames MSP
   - Checksum
   - Parsing réponses

2. **test_gps.py**
   - Parsing UBX NAV-PVT
   - Conversion GPS ↔ NED

3. **test_pid.py**
   - Réponse P, I, D
   - Anti-windup
   - Filtrage D

4. **test_navigation.py**
   - Position controller
   - Velocity controller
   - Conversion accel → angles

5. **test_state_machine.py**
   - Transitions valides
   - Transitions invalides
   - Timeouts

### Tests d'Intégration

1. **Connexion SITL**
   - Connexion TCP au SITL Betaflight
   - Envoi/réception MSP

2. **Boucle complète**
   - Takeoff → Hover → Land

## Simulation SITL

```bash
# 1. Compiler Betaflight SITL
cd ~/projects/betaflight
make TARGET=SITL

# 2. Lancer SITL
./obj/main/betaflight_SITL.elf

# 3. Lancer le simulateur
cd ~/projects/pi_drone_nav
python scripts/simulate.py

# 4. Tester
python -m src.main --simulation
```

## Phases de Développement

### Phase 1: Infrastructure (FAIT)
- [x] Structure projet
- [x] Configuration
- [x] Driver MSP
- [x] Driver GPS
- [x] Contrôleurs PID

### Phase 2: Navigation (FAIT)
- [x] Position controller
- [x] Velocity controller
- [x] Altitude controller
- [x] Waypoint navigator

### Phase 3: Vol (FAIT)
- [x] State machine
- [x] Flight controller
- [x] Intégration

### Phase 4: Interfaces (EN COURS)
- [ ] CLI
- [ ] REST API
- [ ] WebSocket
- [ ] MAVLink

### Phase 5: Tests
- [ ] Tests unitaires
- [ ] Tests intégration SITL
- [ ] Tests terrain

## Références

- [iNav Navigation](https://github.com/iNavFlight/inav/tree/master/src/main/navigation)
- [ArduPilot AC_PosControl](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AC_PosControl)
- [Betaflight MSP](https://github.com/betaflight/betaflight/tree/master/src/main/msp)
- [u-blox Protocol](https://www.u-blox.com/en/docs/UBX-13003221)
