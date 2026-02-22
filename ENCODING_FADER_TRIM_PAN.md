# Codage BLE Fader / Trim / Pan (Zoom F8n Pro)

Les commandes sont envoyées sur la caractéristique RX (write) sous forme de trames **A1** (famille 0xA1). Format exact par type de contrôle.

---

## 1. Fader (input, canaux 0..7)

**Trame : 5 octets**

| Octet | Valeur   | Rôle        |
|-------|----------|-------------|
| 0     | `0xA1`   | Famille     |
| 1     | `0x03`   | Groupe      |
| 2     | `0x05`   | Commande    |
| 3     | `ch`     | Canal 0..7  |
| 4     | `val`    | Niveau 0..125 |

```
A1 03 05 ch val
```

- **Valeur brute** : `val` dans [0, 125]. 0 = fader à fond bas, 125 = à fond haut.
- **Depuis normalisé 0..1023** : `val = (norm * 125 + 511) / 1023`, puis clamp à 125.

---

## 2. Fader “output” (LR / MAIN / SUB, bank HOLD)

Même famille **A1 03 05**, mais l’octet 3 est un **ID fixe** (pas le canal) :

| Contrôle | ID (octet 3) | Plage val |
|----------|--------------|-----------|
| LR       | `0x08`       | 0..121 (0x79) |
| MAIN     | `0x0A`       | 0..121 |
| SUB      | `0x0C`       | 0..121 |

```
A1 03 05 <ID> val
```

- **Valeur brute** : `val` dans [0, 121] (FADER_OUT_VAL_MAX).

---

## 3. Trim (canaux 0..7)

**Trame : 6 octets**

| Octet | Valeur   | Rôle        |
|-------|----------|-------------|
| 0     | `0xA1`   | Famille     |
| 1     | `0x04`   | Groupe      |
| 2     | `0x03`   | Commande trim |
| 3     | `ch`     | Canal 0..7  |
| 4     | `trimVal`| Trim 0..65  |
| 5     | `0x00`   | Réservé     |

```
A1 04 03 ch trimVal 00
```

- **Valeur brute** : `trimVal` dans [0, 65] (observé en btsnoop).
- **Depuis normalisé 0..1023** : `trimVal = (norm * 65 + 511) / 1023`, puis clamp à 65.

---

## 4. Pan (canaux 0..7)

**Trame : 6 octets**

| Octet | Valeur   | Rôle        |
|-------|----------|-------------|
| 0     | `0xA1`   | Famille     |
| 1     | `0x04`   | Groupe      |
| 2     | `0x04`   | Commande pan |
| 3     | `ch`     | Canal 0..7  |
| 4     | `amount`| Montant (voir ci‑dessous) |
| 5     | `side`  | 0 = gauche/centre, 1 = droite |

```
A1 04 04 ch amount side
```

- **side = 0** (gauche / centre) : `amount` 0..127  
  - 0 = L100 (tout à gauche)  
  - 127 = centre  
- **side = 1** (droite) : `amount` 0..72 (74 en code avec marge)  
  - 0 ≈ centre  
  - 72 (ou 74) = R100 (tout à droite)

**Mapping depuis normalisé 0..1023** :

- Zone morte centre : `norm` dans [504, 520] → `side=0`, `amount=127`.
- `norm` < 504 (gauche) : `side=0`, `amount` linéaire 0..126.
- `norm` > 520 (droite) : `side=1`, `amount` linéaire 1..72 (PAN_RIGHT_MAX).

Constantes (dans le sketch) : `PAN_LEFT_MAX = 127`, `PAN_RIGHT_MAX = 74`, `PAN_CENTER_DEAD = 8`.

---

## Récapitulatif

| Contrôle      | Trame BLE (hex)     | Plage valeur / remarque   |
|---------------|---------------------|---------------------------|
| Fader input   | `A1 03 05 ch val`   | ch 0..7, val 0..125       |
| Fader LR      | `A1 03 05 08 val`   | val 0..121                |
| Fader MAIN    | `A1 03 05 0A val`   | val 0..121                |
| Fader SUB     | `A1 03 05 0C val`   | val 0..121                |
| Trim          | `A1 04 03 ch val 00`| ch 0..7, val 0..65        |
| Pan           | `A1 04 04 ch amt side` | ch 0..7 ; side 0 → amt 0..127, side 1 → amt 0..72 |

Source : `ZOOM_CONTROL_V4.ino` (sendFaderRaw, sendTrimRaw, sendPanRaw, mapNormTo*).
