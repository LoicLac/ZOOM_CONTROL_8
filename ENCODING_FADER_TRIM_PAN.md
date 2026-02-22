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

---

## 5. Réception (Zoom → contrôleur) — State Dump initial

Lors de la connexion, le Zoom envoie un burst de **notifications ATT (opcode 0x1B)** sur le handle TX (0x0012). Les 27 paramètres sont encodés dans **3 paquets A1**, identifiés par leur sous-commande (octet 2) :

### Sub-cmd 0x00 — TRIM (8 canaux)

```
A1 11 00 <tr0> 00 <tr1> 00 <tr2> 00 <tr3> 00 <tr4> 00 <tr5> 00 <tr6> 00 <tr7> 00
```

| Octet | Contenu | Rôle |
|-------|---------|------|
| 0 | `0xA1` | Famille |
| 1 | `0x11` | Longueur (17 octets suivants) |
| 2 | `0x00` | Sous-commande TRIM |
| 3 | `tr0` | Trim canal 0, plage 0..65 (0x41) |
| 4 | `0x00` | Padding |
| 5 | `tr1` | Trim canal 1 |
| … | … | Alternance valeur / padding |
| 17 | `tr7` | Trim canal 7 |
| 18 | `0x00` | Padding |

- **Valeur 0** = trim minimum, **0x41 (65)** = trim maximum.
- Encodage identique à l'envoi (`sendTrimRaw`).

### Sub-cmd 0x01 — PAN (8 canaux, 2 octets chacun)

```
A1 11 01 <amt0> <side0> <amt1> <side1> ... <amt7> <side7>
```

| Octet | Contenu | Rôle |
|-------|---------|------|
| 0 | `0xA1` | Famille |
| 1 | `0x11` | Longueur (17 octets suivants) |
| 2 | `0x01` | Sous-commande PAN |
| 3 | `amt0` | Amount canal 0 |
| 4 | `side0` | Side canal 0 (0=gauche/centre, 1=droite) |
| 5 | `amt1` | Amount canal 1 |
| … | … | Paires (amount, side) |
| 17 | `amt7` | Amount canal 7 |
| 18 | `side7` | Side canal 7 |

- Encodage **identique** à l'envoi : `side=0, amt=0` → L100 ; `side=0, amt=127` → centre ; `side=1, amt=72` → R100.
- Confirmé par captures : ALL_0 → `amt=0x00, side=0x00` (L100), PAN_x_max → `amt=0x48, side=0x01` (R100).

### Sub-cmd 0x02 — FADERS (8 input + 3 output)

```
A1 0F 02 <f0> <f1> <f2> <f3> <f4> <f5> <f6> <f7> <LR> 00 <MAIN> 00 <SUB> 00
```

| Octet | Contenu | Rôle |
|-------|---------|------|
| 0 | `0xA1` | Famille |
| 1 | `0x0F` | Longueur (15 octets suivants) |
| 2 | `0x02` | Sous-commande FADER |
| 3–10 | `f0..f7` | Faders input canaux 0..7, plage 0..121 (0x79) |
| 11 | `LR` | Fader output LR, plage 0..109 (0x6D) |
| 12 | `0x00` | Padding |
| 13 | `MAIN` | Fader output MAIN, plage 0..121 (0x79) |
| 14 | `0x00` | Padding |
| 15 | `SUB` | Fader output SUB, plage 0..121 (0x79) |
| 16 | `0x00` | Padding |

**Remarques :**
- Les faders input en réception ont un max de **121** (0x79), non 125 comme en envoi.
- Le fader LR a un max observé de **109** (0x6D), inférieur aux 121 des autres output.

### Récapitulatif réception

| Paquet | Sous-cmd | Paramètres | Plages |
|--------|----------|------------|--------|
| `A1 11 00 …` | 0x00 | Trim ch 0..7 | 0..65 par canal, octets impairs (3,5,7…17) |
| `A1 11 01 …` | 0x01 | Pan ch 0..7 | Paires (amt, side) à octets 3-18 |
| `A1 0F 02 …` | 0x02 | Faders 0..7 + LR + MAIN + SUB | Faders: 0..121, LR: 0..109 |

Source : analyse `CampagneBtsnoopReceive/` — captures ALL_0, ALL_MAX, FAD_x_max, TRIM_x_max, PAN_x_max.
Scripts : `analyze_receive.py`, `analyze_a1_decode.py`.

---

## 6. Flow Control — `80 01 00`

Le Zoom utilise un mécanisme de flow control par ACK : après chaque notification envoyée par le Zoom, le contrôleur doit répondre `80 01 00` sur la caractéristique RX (write). Sans cet ACK, le Zoom suspend l'envoi des notifications suivantes.

```
Contrôleur → Zoom : 80 01 00   (ACK, 3 octets)
```

Ce mécanisme est critique pour la réception du state dump initial. Sans les ACK :
- Seul le premier paquet A1 (sub 0x00, trim) arrive
- Les paquets sub 0x01 (pan) et sub 0x02 (fader) ne sont jamais émis

Dans le sketch, l'ACK est envoyé automatiquement dans le callback `onTxNotification()` après chaque notification reçue, exactement comme le fait l'application iPad officielle.

Découvert par analyse comparative du btsnoop iPad (séquence SENT/RECV intercalée) vs le log serial Arduino (notifications perdues).
