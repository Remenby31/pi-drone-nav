# TODO - 31 Dec 2025

## Test sans hélices

### Pré-requis
- [ ] Hélices retirées
- [ ] Batterie chargée
- [ ] FC connecté USB (/dev/ttyACM0)
- [ ] GPS fix (optionnel pour test indoor)

### 1. Vérifier fix map (inversion ch2/ch3) - DONE
```bash
# Betaflight CLI: map AETR1234
# Fix appliqué dans msp.py:set_raw_rc() - swap ch2/ch3 avant envoi
# Testé et validé 30 Dec 2025
```

### 2. Test ARM/DISARM
```bash
python -m src.main --cli
```

Dans le CLI:
```
> status          # Vérifier état IDLE
> arm             # Doit afficher "ARM command sent: AUX1=1800"
> status          # Vérifier état ARMED
> disarm          # Doit afficher "DISARM command sent: AUX1=1000"
```

### 3. Test mission hover 2m (sans props!)
```
> load config/missions/test_hover_2m.json
> arm
> takeoff 2
```

Observer les logs:
- [ ] `Starting takeoff sequence`
- [ ] `Motor spinup phase`
- [ ] `Throttle ramp phase`
- [ ] Phase transitions dans les logs
- [ ] Timeout 10s si pas de liftoff détecté (normal sans props)

### 4. Test landing (simulé)
Après takeoff timeout → devrait passer en LANDING:
- [ ] `Starting iNav-style landing sequence`
- [ ] `Landing phase: PHASE_HIGH`
- [ ] `Landing phase: PHASE_MID`
- [ ] `Landing phase: PHASE_FINAL`
- [ ] Touchdown detection (fallback après 2s)
- [ ] Auto-disarm

## Problèmes connus à vérifier

| Issue | Status | Action |
|-------|--------|--------|
| map ch2/ch3 | FIXED | Vérifier avec `map` dans CLI |
| ARM via MSP | FIXED | Vérifier AUX1=1800 arme vraiment |
| Timeouts | 10s/20s | OK pour test rapide |

## Notes
- Garder main sur kill switch (déconnecter batterie si problème)
- Les moteurs NE DOIVENT PAS tourner sans hélices en test normal
- Si moteurs tournent → disarm immédiat ou déconnecter batterie
