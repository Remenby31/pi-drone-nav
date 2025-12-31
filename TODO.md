# TODO - Pi Drone Navigation

## URGENT - Raspberry Pi mouille (31 Dec 2025)

### Etat actuel
- Raspberry Pi tombe dans l'herbe mouillee lors du test 4
- Clignote 8x vert au boot (erreur SD card ou boot?)
- Ne demarre pas
- En attente de sechage

### Actions a faire
- [ ] Attendre sechage complet (plusieurs heures minimum)
- [ ] Tenter redemarrage
- [ ] Si ne demarre pas: verifier carte SD sur autre lecteur
- [ ] Recuperer les logs du vol (`~/.pidrone/logs/`)
- [ ] Analyser le CSV du test 4 pour comprendre ce qui s'est passe
- [ ] Evaluer les degats materiels

### Code clignotant Raspberry Pi
- 8 clignotements verts = probleme lecture carte SD
- Solutions possibles:
  - Reinserer la carte SD
  - Verifier contacts de la carte
  - Lire la carte sur PC pour backup
  - Reflasher si necessaire

---

## Analyse logs a faire (quand Pi accessible)

### Fichiers a recuperer
```bash
# Logs CSV de vol
scp drone@192.168.1.114:~/.pidrone/logs/*.csv .

# Logs systeme
sshpass -p "drone" ssh drone@192.168.1.114 "sudo journalctl -u pidrone --since '2025-12-31 09:30' --no-pager" > flight_logs.txt
```

### Points a analyser
- [ ] Sequence complete du test 4
- [ ] Altitude max atteinte
- [ ] Throttle applique
- [ ] Cause de la chute (tilt? timeout? autre?)
- [ ] Duree du vol

---

## Fixes appliques aujourd'hui (31 Dec 2025)

| Fix | Commit | Description |
|-----|--------|-------------|
| Mission.name | 332067f | Acces attribut au lieu de dict.get() |
| Flight logging | 8dfb07d | current_action via mission.actions[idx] |
| hover_throttle save | 8dfb07d | Ne pas ecraser si 0 samples |
| Emergency disarm | c41d140 | Endpoint /api/emergency/disarm |
| max_tilt | 36df22b | 25 -> 35 degres |

---

## Prochains tests (quand Pi OK)

### Test avec ficelle (reprise)
- [ ] Verifier que le Pi fonctionne
- [ ] Verifier integrite du code et config
- [ ] Secher completement le drone
- [ ] Refaire test hover 2m

### Ameliorations a considerer
- [ ] Ajouter watchdog reseau (disarm si perte connexion)
- [ ] Logger plus de details sur les abort
- [ ] Considerer protection etanche pour le Pi

---

## Commandes utiles

```bash
# Arrêt d'urgence
curl -X POST http://192.168.1.114:8080/api/emergency/disarm

# Status
curl http://192.168.1.114:8080/api/status

# Lancer mission 2m
curl -X POST http://192.168.1.114:8080/api/missions/aff0b25b-510e-47d6-9d60-a568386d1568/start

# Stop mission (land + disarm)
curl -X POST http://192.168.1.114:8080/api/missions/active/stop
```
