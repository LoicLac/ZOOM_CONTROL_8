#!/usr/bin/env python3
"""
Analyse des trames 87 et 83 (état statique Zoom F8n Pro BLE).

Usage:
  python3 analyze_87_83.py fichier_log.txt
  python3 analyze_87_83.py campaign1.txt campaign2.txt ...
  python3 analyze_87_83.py --analyze-block recv.txt   # si fichier contient blocs === ANALYZE 87 ===

Deux modes d'entrée:
1) Log brut (Receive Mcu...): découpe par campagne (BOOT..DONE), extrait TX n=6 87 04 et TX n=16 83 0E.
2) Blocs ANALYZE: extrait les lignes 87,i,b1,b2,b3 et 83,i,b2..b15 entre === ANALYZE 87/83 ===.
"""

import re
import sys
from collections import defaultdict

def parse_hex_line(s):
    """Extrait les octets hex d'une ligne (ex: '87 04 0D 29 12 00' ou '0 00 00')."""
    parts = re.findall(r'[0-9A-Fa-f]{1,2}', s)
    return [int(x, 16) for x in parts]

def parse_campaign_blocks_from_raw(content):
    """
    Découpe le fichier en campagnes (bloc BOOT..DONE/SUMMARY).
    Titre = dernière ligne non vide avant "--> BOOT" qui ne ressemble pas à une date.
    Retourne [(name, block_text), ...].
    """
    campaigns = []
    lines = content.split('\n')
    i = 0
    while i < len(lines):
        line = lines[i]
        # Détection début de campagne: "--> BOOT"
        if re.search(r'-->\s*BOOT', line):
            title = 'unnamed'
            j = i - 1
            while j >= 0:
                prev = lines[j].strip()
                if prev and '-->' not in prev and not re.match(r'\d{4}-\d{2}-\d{2}', prev):
                    title = prev.rstrip(':').strip() or title
                    break
                j -= 1
            block = [line]
            i += 1
            while i < len(lines):
                l = lines[i]
                block.append(l)
                if re.search(r'===\s*END\s+SUMMARY\s*===', l):
                    i += 1
                    break
                i += 1
            campaigns.append((title, '\n'.join(block)))
            continue
        i += 1
    return campaigns

def extract_87_83_from_block(block_text):
    """
    Extrait listes de 87 (b1,b2,b3) et 83 (14 bytes payload) depuis un bloc de log brut.
    Gère les lignes de continuation (hex sans "TX n=").
    """
    f87_list = []
    f83_list = []
    lines = block_text.split('\n')
    i = 0
    while i < len(lines):
        line = lines[i]
        # TX n=6 : 87 04 b1 b2 b3 00 (peut être coupé: "87 04 b1 b2" puis "b3 00")
        m = re.search(r'TX\s+n=\d+\s*:\s*87\s+04\s*(.*)', line)
        if m:
            bytes_hex = re.findall(r'[0-9A-Fa-f]{1,2}', m.group(1))
            j = i + 1
            while len(bytes_hex) < 4 and j < len(lines):
                bytes_hex.extend(re.findall(r'[0-9A-Fa-f]{1,2}', lines[j]))
                j += 1
            i = j
            if len(bytes_hex) >= 4:
                b1, b2, b3 = int(bytes_hex[0], 16), int(bytes_hex[1], 16), int(bytes_hex[2], 16)
                f87_list.append((b1, b2, b3))
            continue
        # TX n=16 : 83 0E ... (payload peut être sur lignes suivantes)
        m = re.search(r'TX\s+n=16\s*:\s*83\s+0E\s*(.*)', line)
        if m:
            rest = m.group(1).strip()
            bytes_hex = re.findall(r'[0-9A-Fa-f]{1,2}', rest)
            j = i + 1
            while len(bytes_hex) < 14 and j < len(lines):
                bytes_hex.extend(re.findall(r'[0-9A-Fa-f]{1,2}', lines[j]))
                j += 1
            i = j
            if len(bytes_hex) >= 14:
                f83_list.append(tuple(int(bytes_hex[k], 16) for k in range(14)))
            continue
        i += 1
    return f87_list, f83_list

def extract_from_analyze_blocks(content):
    """
    Extrait campagnes depuis blocs === ANALYZE 87 === ... === END ANALYZE 87 === et idem 83.
    Un couple (liste 87, liste 83) = une campagne (on suppose même nombre de blocs 87 et 83).
    """
    campaigns = []
    parts_87 = re.findall(r'=== ANALYZE 87 ===\s*(.*?)\s*=== END ANALYZE 87 ===', content, re.DOTALL)
    parts_83 = re.findall(r'=== ANALYZE 83 ===\s*(.*?)\s*=== END ANALYZE 83 ===', content, re.DOTALL)
    for idx, (blk87, blk83) in enumerate(zip(parts_87, parts_83)):
        f87 = []
        for line in blk87.strip().split('\n'):
            m = re.match(r'87,\d+,([0-9A-Fa-f]+),([0-9A-Fa-f]+),([0-9A-Fa-f]+)', line)
            if m:
                f87.append((int(m.group(1), 16), int(m.group(2), 16), int(m.group(3), 16)))
        f83 = []
        for line in blk83.strip().split('\n'):
            m = re.match(r'83,\d+,(.+)', line)
            if m:
                bytes_hex = re.findall(r'[0-9A-Fa-f]{1,2}', m.group(1))
                if len(bytes_hex) >= 14:
                    f83.append(tuple(int(bytes_hex[j], 16) for j in range(14)))
        campaigns.append((f'campaign_{idx}', f87, f83))
    return campaigns

def run_analysis(campaigns):
    """
    campaigns: [(name, list_87, list_83), ...]
    list_87: [(b1,b2,b3), ...]
    list_83: [(14 bytes), ...]
    """
    # ---- 87: clé = (b1,b2), valeur = b3 (par séquence: même (b1,b2) peut revenir à des positions différentes)
    # On regroupe par (b1,b2) et on affiche les b3 par campagne (dernier b3 vu pour cette clé dans la campagne).
    key87_to_campaigns = defaultdict(dict)  # (b1,b2) -> { campaign_name: (seq, b3) }
    for name, f87, _ in campaigns:
        for idx, (b1, b2, b3) in enumerate(f87):
            key87_to_campaigns[(b1, b2)][name] = (idx, b3)

    # ---- 83: première trame 83 par campagne = état statique (14 bytes)
    first83_per_campaign = {name: f83[0] for name, f87, f83 in campaigns if f83}

    # Afficher
    print("========== 87 (b1,b2) -> b3 par campagne ==========")
    for key in sorted(key87_to_campaigns.keys()):
        vals = key87_to_campaigns[key]
        if len(set(v for _, v in vals.values())) > 1:  # au moins 2 valeurs différentes
            print(f"  87 key {key} (0x{key[0]:02X},0x{key[1]:02X}):")
            for cname in sorted(vals.keys()):
                idx, b3 = vals[cname]
                print(f"    {cname}: seq={idx} b3=0x{b3:02X} ({b3})")
    print()

    print("========== 83 payload (14 bytes) - index qui varient ==========")
    for byte_idx in range(14):
        per_campaign = {name: first83_per_campaign[name][byte_idx] for name in first83_per_campaign}
        if len(set(per_campaign.values())) > 1:
            print(f"  83 byte[{byte_idx}]:")
            for cname in sorted(per_campaign.keys()):
                print(f"    {cname}: 0x{per_campaign[cname]:02X} ({per_campaign[cname]})")
    print()

    # Tableau récap: 87 (b1,b2) en lignes, campagnes en colonnes
    print("========== Table 87 (ligne = (b1,b2), colonne = campagne, cellule = b3) ==========")
    all_keys = sorted(key87_to_campaigns.keys())
    names = sorted(set(n for n, _, _ in campaigns))
    if all_keys and names:
        header = "key(b1,b2)" + "".join(f";{n[:20]}" for n in names)
        print(header)
        for key in all_keys:
            row = f"0x{key[0]:02X}_{key[1]:02X}"
            for n in names:
                v = key87_to_campaigns.get(key, {}).get(n)
                row += f";{v[1] if v else '-'}"
            print(row)
    print()

    print("========== Nombre de 87 et 83 par campagne ==========")
    for name, f87, f83 in campaigns:
        print(f"  {name}: n87={len(f87)} n83={len(f83)}")

    # ---- Inférence 83: quels octets parser/stocker pour un tableau de paramètres utilisable
    varying_83 = []
    for byte_idx in range(14):
        per_campaign = {name: first83_per_campaign[name][byte_idx] for name in first83_per_campaign}
        if len(set(per_campaign.values())) > 1:
            varying_83.append(byte_idx)
    print()
    print("========== INFERENCE 83: octets à parser / stocker ==========")
    print("  Indices dont la valeur change entre campagnes:", varying_83 if varying_83 else "aucun")
    print("  Recommandation: stocker au minimum ces octets (première trame 83 = état statique).")
    print("  Option: stocker les 14 octets du payload pour extensibilité.")
    print()
    print("  --- Snippet C proposé (à coller dans ton code) ---")
    print("  // Payload 83 0E = 14 octets après l'en-tête. Premier 83 reçu = état statique.")
    print("  #define NOTIF83_PAYLOAD_LEN  14")
    print("  #define NOTIF83_VARYING_IDX  9   // premier indice qui varie dans nos campagnes")
    print("  #define NOTIF83_VARYING_NUM  5   // nombre d'octets utiles observés (indices 9..13)")
    print("  static uint8_t notif83_payload[NOTIF83_PAYLOAD_LEN];  // ou [NOTIF83_VARYING_NUM] pour 9..13 seul")
    print("  // Lors de réception 83 0E:")
    print("  //   memcpy(notif83_payload, p + 2, 14);")
    print("  // Pour n'utiliser que les octets utiles (9..13):")
    if varying_83:
        print("  //   notif83_byte9  = p[2+9];   // observé: MAIN max=0x20, zero=0")
        print("  //   notif83_byte10 = p[2+10];  // observé: MAIN max=0x26, zero=0")
        print("  //   notif83_byte11 = p[2+11];  // observé: MAIN max=0x02, zero=0")
        print("  //   notif83_byte12 = p[2+12];  // observé: zero=0x20, MAIN=0x16")
        print("  //   notif83_byte13 = p[2+13];  // observé: zero=0x26, MAIN=0x15")
    print("  --- fin snippet ---")

    # ---- Table (b1,b2) → (bank, channel) pour init des banks V4 ----
    build_87_map(key87_to_campaigns, campaigns)

def build_87_map(key87_to_campaigns, campaigns):
    """
    Construit la table (b1,b2) → (bank, channel) à partir des campagnes "un paramètre au max".
    bank: 0=FADER, 1=PAN, 2=TRIM, 3=HOLD. channel: 0..7 (HOLD: 0=LR, 1=MAIN, 2=SUB).
    """
    campaign_names = [name for name, _, _ in campaigns]
    # Référence: campagne "tout à zéro"
    zero_name = None
    for n in campaign_names:
        if "zero" in n.lower() and "max" not in n.lower():
            zero_name = n
            break
    if not zero_name:
        print("========== TABLE 87 → (bank, ch): pas de campagne 'zero' trouvée ==========")
        return
    # Référence de secours: "tout au MAX" (certaines clés n’apparaissent que dans single-param + MAX, pas dans zero)
    max_name = None
    for n in campaign_names:
        if "max" in n.lower() and "parametres" in n.lower() and "zero" not in n.lower():
            max_name = n
            break

    # Campagnes "un seul paramètre au max" → (bank, ch). Ordre: motifs uniques pour éviter faux positifs.
    single_param = [
        ("fad 0", 0, 0),   # FADER ch0
        ("fad 7", 0, 7),   # FADER ch7 (FAD 7)
        ("trim 0", 2, 0),  # TRIM ch0
        ("trim 7", 2, 7),  # TRIM ch7
        ("pan 0", 1, 0),   # PAN ch0
        ("pan 7", 1, 7),   # PAN ch7
        ("out lr", 3, 0),  # HOLD LR
        ("out main", 3, 1),  # HOLD MAIN
        ("fader sub", 3, 2),  # HOLD SUB
    ]

    # Pour chaque campagne single-param, trouver les clés dont b3 diffère de "zero" → assignation (bank, ch)
    # (Inverser la logique évite qu’une campagne avec peu de trames 87 n’apparaisse pas pour certaines clés.)
    key_to_bank_ch = {}
    key_ambiguous = []

    def find_campaign(pattern):
        for n in campaign_names:
            if pattern in n.lower() and n != zero_name:
                return n
        return None

    assigned_per_campaign = []
    for pattern, bank, ch in single_param:
        cname = find_campaign(pattern)
        if not cname:
            continue
        count = 0
        for key in key87_to_campaigns:
            vals = key87_to_campaigns[key]
            if cname not in vals:
                continue
            ref = vals.get(zero_name) or (vals.get(max_name) if max_name else None)
            if ref is None:
                continue
            ref_b3 = ref[1]
            if vals[cname][1] != ref_b3:
                if key in key_to_bank_ch:
                    prev = key_to_bank_ch[key]
                    if prev != (bank, ch):
                        key_ambiguous.append((key, prev, (bank, ch), cname))
                else:
                    key_to_bank_ch[key] = (bank, ch)
                    count += 1
        assigned_per_campaign.append((pattern, (bank, ch), count))

    print()
    print("========== TABLE 87 (b1,b2) → (bank, channel) ==========")
    print("  bank: 0=FADER 1=PAN 2=TRIM 3=HOLD(0=LR,1=MAIN,2=SUB)")
    print("  Clés assignées par campagne (clé présente + b3 ≠ ref zero/MAX):")
    for pattern, (bank, ch), count in assigned_per_campaign:
        print(f"    {pattern!r} → bank={bank} ch={ch}: {count} clé(s)")
    missing = [p for p, _, c in assigned_per_campaign if c == 0]
    if missing:
        print("  (0 clé: ces (b1,b2) n’apparaissent pas dans zero ni tout MAX pour ce log; recapture ou autre ref possible.)")
    print("  Table finale (clé → bank, ch):")
    for key in sorted(key_to_bank_ch.keys()):
        bank, ch = key_to_bank_ch[key]
        print(f"    0x{key[0]:02X}, 0x{key[1]:02X}  →  bank={bank} ch={ch}")
    if key_ambiguous:
        print("  Clés ambiguës (plusieurs params possibles):")
        for key, prev, new, cname in key_ambiguous[:20]:
            print(f"    0x{key[0]:02X}, 0x{key[1]:02X}  →  déjà {prev}, aussi {new} ({cname})")
    print()

    # Export C: table pour lookup (b1,b2) → bank, ch
    print("  --- Snippet C: table 87 → bank/ch (à coller dans ZOOM_CONTROL_V4 ou init) ---")
    print("  // Généré par analyze_87_83.py. (b1,b2) → bank 0..3, ch 0..7.")
    print("  #define MAP87_MAX  " + str(len(key_to_bank_ch)) + "")
    print("  static const struct { uint8_t b1, b2, bank, ch; } map87[MAP87_MAX] = {")
    for key in sorted(key_to_bank_ch.keys()):
        bank, ch = key_to_bank_ch[key]
        print(f"    {{ 0x{key[0]:02X}, 0x{key[1]:02X}, {bank}, {ch} }},")
    print("  };")
    print("  // Lookup: pour chaque trame 87 04 b1 b2 b3 00, parcourir map87; si b1,b2 match alors")
    print("  //   gBankVal[bank][ch] = rawToNorm(b3, ...);  (selon bank: fader 0..125, trim 0..65, etc.)")
    print("  --- fin snippet ---")

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)

    use_analyze_block = '--analyze-block' in sys.argv
    if use_analyze_block:
        sys.argv.remove('--analyze-block')

    campaigns = []

    for path in sys.argv[1:]:
        if path.startswith('--'):
            continue
        try:
            with open(path, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
        except Exception as e:
            print(f"Erreur lecture {path}: {e}", file=sys.stderr)
            continue

        if use_analyze_block or '=== ANALYZE 87 ===' in content:
            for name, f87, f83 in extract_from_analyze_blocks(content):
                campaigns.append((f"{name}_{path}", f87, f83))
        else:
            for name, block in parse_campaign_blocks_from_raw(content):
                f87, f83 = extract_87_83_from_block(block)
                if f87 or f83:
                    campaigns.append((name, f87, f83))

    if not campaigns:
        print("Aucune campagne extraite. Vérifiez le format du fichier.", file=sys.stderr)
        sys.exit(1)

    run_analysis(campaigns)

if __name__ == '__main__':
    main()
