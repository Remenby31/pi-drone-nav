# Bug : La fonctionnalité RX_MSP ne peut pas être activée via CLI

## Résumé

La fonctionnalité `RX_MSP` existe dans le code Betaflight (bit 14 de `FEATURE_RX_MSP`) mais son nom est **absent du tableau `featureNames[]`** dans `cli/cli.c`. Cela rend impossible l'activation de cette fonctionnalité via la commande CLI `feature RX_MSP`, qui retourne l'erreur `INVALID NAME`.

## Environnement

- **Version Betaflight** : 2026.6.0-alpha (compilé depuis source)
- **Board** : STM32H743 (DAKEFPVH743)
- **Connexion** : USB (/dev/ttyACM0)

## Étapes pour reproduire

1. Flasher Betaflight 2026.6.0-alpha sur un FC compatible
2. Se connecter au CLI via USB
3. Tenter d'activer RX_MSP :
   ```bash
   feature RX_MSP
   ```
4. **Résultat** : `INVALID NAME`

## Comportement attendu vs actuel

### Attendu
La commande `feature RX_MSP` devrait activer le bit 14 (`FEATURE_RX_MSP`) dans la configuration des fonctionnalités, permettant le contrôle MSP via `MSP_SET_RAW_RC`.

### Actuel
```
# feature RX_MSP
INVALID NAME
```

La fonctionnalité ne peut pas être activée via CLI, obligeant à modifier manuellement le firmware.

## Analyse du bug

### 1. La fonctionnalité existe bien dans le code

**Fichier : `src/main/fc/feature.h`**
```c
#define FEATURE_RX_MSP         (1 << 14)
```

**Fichier : `src/main/common/common_pre.h`**
```c
#define USE_RX_MSP
```

Le bit 14 est défini et le code RX_MSP est compilé par défaut.

### 2. Mais le nom est absent du tableau CLI

**Fichier : `src/main/cli/cli.c` (ligne ~1000-1020)**
```c
static const char * const featureNames[] = {
    "RX_PPM", "VBAT", "INFLIGHT_ACC_CAL", "RX_SERIAL", "MOTOR_STOP",
    "SERVO_TILT", "SOFTSERIAL", "GPS", "FAILSAFE",
    "SONAR", "TELEMETRY", "CURRENT_METER", "3D", "RX_PARALLEL_PWM",
    // RX_MSP devrait être ici (bit 14) mais il est ABSENT
    "RSSI_ADC", "LED_STRIP", "DISPLAY", "OSD",
    "BLACKBOX", "CHANNEL_FORWARDING", "TRANSPONDER", "AIRMODE",
    "RX_SPI", "SOFTSPI", "ESC_SENSOR", "ANTI_GRAVITY",
    "DYNAMIC_FILTER", "RACE_PRO",
    NULL
};
```

Le tableau `featureNames[]` saute l'indice 14, causant un décalage entre les indices du tableau et les bits de fonctionnalité.

## Solution appliquée

Nous avons ajouté manuellement le nom manquant dans `cli/cli.c` :

```c
static const char * const featureNames[] = {
    "RX_PPM", "VBAT", "INFLIGHT_ACC_CAL", "RX_SERIAL", "MOTOR_STOP",
    "SERVO_TILT", "SOFTSERIAL", "GPS", "FAILSAFE",
    "SONAR", "TELEMETRY", "CURRENT_METER", "3D", "RX_PARALLEL_PWM",
    _R(FEATURE_RX_MSP, "RX_MSP"),  // <-- AJOUT ICI
    "RSSI_ADC", "LED_STRIP", "DISPLAY", "OSD",
    "BLACKBOX", "CHANNEL_FORWARDING", "TRANSPONDER", "AIRMODE",
    "RX_SPI", "SOFTSPI", "ESC_SENSOR", "ANTI_GRAVITY",
    "DYNAMIC_FILTER", "RACE_PRO",
    NULL
};
```

Après recompilation :
```bash
# feature RX_MSP
Enabled:  GPS TELEMETRY LED_STRIP OSD ESC_SENSOR RX_MSP
Disabled: RX_PPM VBAT INFLIGHT_ACC_CAL RX_SERIAL MOTOR_STOP ...
```

La fonctionnalité s'active correctement et `MSP_SET_RAW_RC` fonctionne.

## Impact

Ce bug affecte tous les utilisateurs qui veulent utiliser le contrôle MSP pur (drone autonome, contrôle par ordinateur, etc.) car :
- Impossible d'activer `RX_MSP` via Betaflight Configurator
- Impossible d'activer via CLI
- Nécessite une recompilation custom du firmware

## Proposition de fix

Ajouter `"RX_MSP"` à la position 14 dans le tableau `featureNames[]` de `src/main/cli/cli.c`.

Alternative avec macro `_R()` pour éviter les désynchronisations futures :
```c
_R(FEATURE_RX_MSP, "RX_MSP"),
```

## Vérification

Après le fix, ces commandes devraient fonctionner :
```bash
# feature RX_MSP          # Active la fonctionnalité
# feature -RX_MSP         # Désactive la fonctionnalité
# feature                 # Liste toutes les fonctionnalités (RX_MSP visible)
```

---

**Note** : Ce bug existe probablement dans toutes les versions récentes de Betaflight où `FEATURE_RX_MSP` est défini mais absent de `featureNames[]`.
