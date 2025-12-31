# Timeline - Pi Drone Nav

Historique des tests, incidents et decouvertes.

---

## 31 Dec 2025 - Tests avec ficelle

### Test 1 - 09:22
**Mission**: Test Hover 2m
**Resultat**: ECHEC - Erreur code

**Problemes**:
- `AttributeError: 'Mission' object has no attribute 'get'` - crash au demarrage
- hover_throttle charge a 0.2 au lieu de 0.5 (fichier dans mauvais repertoire)
- Flight logging crashait a chaque boucle (`MissionExecutor.current_action` n'existe pas)

**Fixes appliques**:
- `mission.name` au lieu de `mission.get("name")`
- Ne pas sauvegarder hover_throttle si 0 samples
- Corriger acces a l'action courante via `mission.actions[idx]`

---

### Test 2 - 09:22:45
**Mission**: Test Hover 2m
**Resultat**: ABORT - Tilt excessif

**Sequence**:
- 09:22:45 - ARM, takeoff demarre
- 09:22:45 - hover_throttle = 0.50 (correct)
- 09:22:45 - SPINUP -> CLIMBING
- 09:22:46 - Liftoff conditions met: thr=0.75, gyro=10 dps
- 09:22:46 - Liftoff lost, reset timer
- 09:22:48 - Liftoff conditions met: thr=0.75, gyro=46.5 dps
- 09:22:48 - Liftoff lost again
- 09:22:50 - **ABORT: Excessive tilt 52.2 > 25.0**
- 09:22:50 - Transition TAKEOFF -> LANDING
- 09:22:51 - Emergency disarm tente (404 - endpoint manquant)

**Observations**:
- Drone montait par a-coups
- Tilt 52.2 degres (ficelle tirait?)
- Endpoint `/api/emergency/disarm` n'existait pas

**Fixes appliques**:
- Ajoute endpoint `/api/emergency/disarm`

---

### Test 3 - 09:33:08
**Mission**: Test Hover 2m
**Resultat**: ABORT - Tilt excessif (mais moins)

**Sequence**:
- 09:33:08 - ARM, takeoff demarre
- 09:33:08 - hover_throttle = 0.50
- 09:33:08 - SPINUP -> CLIMBING immediat
- 09:33:08 - Liftoff conditions met: thr=0.75, gyro=14.8 dps
- 09:33:09 - Liftoff lost
- 09:33:10 - Liftoff conditions met: thr=0.75, gyro=29.7 dps
- 09:33:10 - **Liftoff CONFIRMED**: throttle=0.75, gyro=137.9 dps
- 09:33:10 - **ABORT: Excessive tilt 26.8 > 25.0**
- 09:33:10 - Transition TAKEOFF -> LANDING
- 09:33:30 - FAILSAFE: Landing timeout

**Observations**:
- Le drone a bien decolle (liftoff confirme)
- Abort immediat car tilt 26.8 > limite 25
- Limite trop stricte pour test avec ficelle

**Fix applique**:
- Augmente max_tilt de 25 a 35 degres

---

### Test 4 - ~09:35
**Mission**: Test Hover 2m
**Resultat**: INCIDENT - Raspberry Pi mouille

**Sequence**:
- Mission lancee
- Drone a decolle
- [Details a analyser dans les logs]
- Drone tombe dans l'herbe mouillee
- Raspberry Pi mouille

**Etat actuel**:
- Raspberry Pi clignote 8x vert (erreur boot/SD card?)
- Ne demarre pas
- En attente de sechage

---

## 30 Dec 2025 - Test Glissiere (INCIDENT)

### Ce qui s'est passe

**Mission**: Test Hover 1m sur glissiere verticale

**Problemes rencontres**:
1. **RXLOSS** bloquait l'armement → corrige en envoyant RC meme en IDLE
2. **hover_throttle = 0.20** (20%) au lieu de 0.50 → systeme a compense en montant throttle trop vite
3. **ramp_rate = 0.30/s** trop rapide pour un premier test
4. Drone monte a **1m80** au lieu de 1m, inclinaison **36.8°** → abort
5. **Glissiere cassee**, drone atterri sur les operateurs

### Bugs corriges (flight_controller.py)

1. **RC en IDLE** : Ajout envoi RC continu meme en etat IDLE pour eviter RXLOSS
2. **Init RC channels** : Valeurs desarmees par defaut `[1500,1500,885,1500,1000,...]`
3. **ARMED + mission** : Handling de l'etat ARMED quand mission active

### Solution implementee

L'ancien systeme (6-phase avec ramp) a ete **remplace** par le systeme iNav-style:
- **Throttle = hover_throttle + velocity_PID** (s'adapte au poids)
- **Detection liftoff via gyro > 7°/s** (detecte l'instabilite)
- **3 etats simples** au lieu de 6 phases

---

## 30 Dec 2025 - Tests sans helices (Propless)

### Hover Mission Simulation

Test sans helices validant la chaine de controle complete:

```
1. Connexion OK
2. ARM OK - Motors: [1067, 1063, 1056, 1056]
3. Throttle ramp: 950 → 1000 → 1100 → 1200
   - @1200: Motors [1327, 1083, 1323, 1084]
4. Hover 3s OK
5. Landing sim: 1200 → 1100 → 1000 → 885
6. DISARM OK - Motors: [1000, 1000, 1000, 1000]
```

### IMU Configuration

```
Accelerometer scale: 2048 LSB/G (±16G range)
Stationary reading: ~1.00G (2048 raw)
```

---

## 30 Dec 2025 - Magnetometer Calibration

```
mag_align_yaw = -490  (soit -49.0°)
```

Verifie: drone pointant nord reel → heading affiche 359° (correct)

---

## 30 Dec 2025 - Betaflight RX_MSP Bugs

### Bug 1: RX_MSP missing from CLI featureNames

**Symptome**: `feature RX_MSP` retourne `INVALID NAME`

**Cause**: Le nom "RX_MSP" etait absent du tableau `featureNames[]` dans `cli/cli.c`, meme si `FEATURE_RX_MSP` (bit 14) existe dans `feature.h`.

**Fix applique**: Ajoute `_R(FEATURE_RX_MSP, "RX_MSP"),` dans `cli/cli.c:247`

### Bug 2: RX rate = 0 avec RX_MSP

**Symptome**: `status` montre `RX rate: 0` et les flags `RXLOSS MSP` bloquent l'armement.

**Cause racine**: `rxMspFrameStatus()` dans `rx/msp.c` ne mettait pas a jour `lastRcFrameTimeUs`.

**Fix applique** dans `rx/msp.c`:
```c
rxRuntimeState->lastRcFrameTimeUs = micros();  // FIX
```

**Fork Betaflight**: https://github.com/Remenby31/betaflight (branche `fix/rx-msp-cli-support`)

---

## 29 Dec 2025 - Hardware Testing DAKEFPVH743

### Firmware Compilation

```bash
make CONFIG=DAKEFPVH743
# Output: obj/betaflight_2026.6.0-alpha_STM32H743_DAKEFPVH743.hex
```

### MSP Control Verification

**Hardware**: DAKEFPVH743 (STM32H743)
**Connection**: USB (/dev/ttyACM0)

```
API Version: 1.47 (protocol 0)
Firmware: BTFL
Board: H743
```

### Motor Control Test (MSP_SET_MOTOR)

```python
Motor 1: 1000 → 1050 → 1100 → 1150 → 1200 → stop
Motor 2: 1000 → 1050 → 1100 → 1150 → 1200 → stop
Motor 3: 1000 → 1050 → 1100 → 1150 → 1200 → stop
Motor 4: 1000 → 1050 → 1100 → 1150 → 1200 → stop
```

### Full Driver Test Results

```
1. CONNEXION        ✓ Port serie ouvert: /dev/ttyACM0
2. IDENTIFICATION   ✓ API Version: 1.47, FC Variant: BTFL
3. LECTURE ETAT     ✓ Status, Attitude, Altitude, IMU, Analog, RC, Motors, GPS
4. COMMANDES RC     ✓ set_raw_rc() envoye et verifie
5. CONTROLE MOTEURS ✓ Motor 1-4: set=1100, read=1100
6. TEST RAMPE       ✓ Rampe 1000->1150->1000 complete
7. ARRET D'URGENCE  ✓ stop_all_motors(): tous arretes
```

**All 14 MSP functions tested and working.**

### Hardware Status

| Parameter | Value |
|-----------|-------|
| Battery | 16.0V (4S) |
| GPS Satellites | 21 |
| GPS Position | 43.48°N, 1.39°E |
| CPU Load | 0% |

---

## 29 Dec 2025 - RC Control Verification

### Channel Mapping Resolution

Avec `map AETR1234` dans Betaflight:
- ch2 → THROTTLE (internal index 3)
- ch3 → YAW (internal index 2)

**Solution**: Envoyer `[Roll, Pitch, Throttle, Yaw, AUX...]` directement, pas de swap necessaire.

### Arming Test

1. **RX_MSP Configuration**: `serialrx_provider = NONE` + `FEATURE_RX_MSP`
2. **Continuous RC Required**: Must send MSP_SET_RAW_RC at 50Hz
3. **Arming via AUX1**: Setting AUX1 to 1800 arms
4. **Throttle Control**: Works while armed
5. **Disarming**: AUX1 back to 1000

```python
RC_DISARMED = [1500, 1500, 885, 1500, 1000, 1800, 1500, 1500]
RC_ARMED    = [1500, 1500, 885, 1500, 1800, 1800, 1500, 1500]
```

---

## Commits du 31 Dec 2025

| Commit | Description |
|--------|-------------|
| 332067f | fix: Mission.name attribute access |
| 8dfb07d | fix: Flight logging crash and hover_throttle save |
| c41d140 | feat: Add /api/emergency/disarm endpoint |
| 36df22b | config: Increase takeoff max_tilt to 35° |
