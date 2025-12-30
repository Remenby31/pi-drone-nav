# TODO - Pi Drone Navigation

---

## Test avec ficelle (31 Dec 2025)

### Changement de méthode
Utiliser une **ficelle** au lieu d'une glissière :
- Le drone sera plus libre de ses mouvements
- Moins de risque de blocage mécanique
- Permet de tester les corrections d'attitude

### Mission à tester
`Test Hover 1m`
1. Takeoff → 1m
2. Hover 3s
3. Land

### Checklist avant test
- [ ] hover_throttle = 0.5 (vérifier `~/.pidrone/hover_throttle.json`)
- [ ] Flight logging actif (automatique au ARM)
- [ ] Ficelle assez longue (~2m)
- [ ] Point d'attache solide au-dessus du drone
- [ ] Hélices montées
- [ ] Batterie chargée (>14.8V pour 4S)
- [ ] GPS fix (>5 sats)
- [ ] Zone dégagée

### Commandes
```bash
# Vérifier hover_throttle
ssh drone@192.168.1.114 "cat ~/.pidrone/hover_throttle.json"

# Lancer la mission
curl -X POST http://192.168.1.114:8080/api/missions/50d4ebda-b217-47a5-9d30-a077c848bb93/start

# ARRÊT D'URGENCE
curl -X POST http://192.168.1.114:8080/api/missions/active/stop

# Récupérer les logs après le test
scp drone@192.168.1.114:~/.pidrone/logs/*.csv .
```

---

## Test sans hélices (30 Dec 2025) - DONE

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
