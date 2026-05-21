#!/usr/bin/env python3
"""
vdes_decode.py — VDES ASM VDE-TER frame decoder

Avkodningskedja (ITU-R M.2092-2):
  [sync] → [32-bit Link ID] → [descramble] → [depuncture] → [turbo decode] → [CRC check]

Användning:
  # Live pipeline (läser JSONL från stdin):
  build/tools/vdes_replay --iq FILE.iq16 --rate 2048000 --freq 161862500 \
    --demod vdes_burst_demod --decoder vdes_burst_decoder \
    --param candidate_bits=2000 --jsonl 2>/dev/null | python3 tools/vdes_decode.py

  # Offline mot hårdkodade testkandidater:
  python3 tools/vdes_decode.py
"""

import json
import sys
from itertools import product

# ── Kända Link-ID-koder (32 bitar, Reed-Muller (32,6) kodord) ─────────────
# Källa: ITU-R M.2092-2, Tabell 3.
LINK_IDS = {
    5:  list(map(int, '11010101111011010111111010111111')),
    11: list(map(int, '11101101001011101100001001111100')),
    17: list(map(int, '10000111001101110010010011100101')),
    19: list(map(int, '10001111010010000010010000011010')),
}

# §A2-1.2.3.4: XOR-mask som appliceras på Link ID-kodordet innan sändning.
LINK_ID_MASK = list(map(int, '11000010111000101000111001001111'))

# Faktiska bitmönster i luften = RM-kodord XOR mask.
LINK_IDS_OTA = {lid: [b ^ m for b, m in zip(pat, LINK_ID_MASK)]
                for lid, pat in LINK_IDS.items()}

LINK_ID_INFO = {
    5:  {'data_bits': 256,  'fec': '3/4 turbo', 'k1': 2,  'k2': 144},
    11: {'data_bits': 400,  'fec': 'none',       'k1': 2,  'k2': 216},
    17: {'data_bits': 1840, 'fec': 'none',       'k1': 6,  'k2': 312},
    19: {'data_bits': 5584, 'fec': '3/4 turbo',  'k1': 16, 'k2': 351},
}

# ── LFSR-scrambler (x^15 + x^14 + 1) ─────────────────────────────────────
LFSR_INIT = [1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # 15 bitar

def lfsr_generate(n_bits, init=None):
    """Genererar n_bits av VDES PRBS-15-scramblersekvens."""
    state = list(init or LFSR_INIT)
    seq = []
    for _ in range(n_bits):
        fb = state[14] ^ state[13]   # taps vid x^15 och x^14
        seq.append(fb)
        state = [fb] + state[:-1]
    return seq

def lfsr_descramble(bits, offset=0):
    """Descramblerar bits med VDES LFSR. offset: antal genererade bitar att hoppa."""
    keystream = lfsr_generate(offset + len(bits))
    return [b ^ k for b, k in zip(bits, keystream[offset:])]

# ── CRC-32 MPEG-2 ─────────────────────────────────────────────────────────
def crc32_mpeg(bits):
    """CRC-32/MPEG-2: poly=0x04C11DB7, init=0xFFFFFFFF, no reflection."""
    crc = 0xFFFFFFFF
    for b in bits:
        if ((crc >> 31) ^ b) & 1:
            crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
        else:
            crc = (crc << 1) & 0xFFFFFFFF
    return crc

# ── Link-ID-sökning ───────────────────────────────────────────────────────
def hamming(a, b):
    return sum(x != y for x, y in zip(a, b))

def find_link_id(bits, max_offset=64, max_errors=4):
    """
    Söker igenom bits[offset:offset+32] för offset 0..max_offset.
    Testar både RM-kodord (oscramblat) och OTA-mönster (XOR mask §A2-1.2.3.4).
    Returnerar lista av (offset, link_id, errors, variant) sorterad på errors.
    """
    hits = []
    patterns = (
        [(lid, pat, 'raw') for lid, pat in LINK_IDS.items()] +
        [(lid, pat, 'ota') for lid, pat in LINK_IDS_OTA.items()]
    )
    for offset in range(max_offset):
        if offset + 32 > len(bits):
            break
        window = bits[offset:offset + 32]
        for lid, pattern, variant in patterns:
            err = hamming(window, pattern)
            if err <= max_errors:
                hits.append((offset, lid, err, variant))
    hits.sort(key=lambda x: x[2])
    return hits

# ── QPP-interleaver (VDES-specifik, för framtida turbo-avkodning) ─────────
def qpp_interleave(bits, k1, k2, primes):
    """
    Genererar QPP-permutationen för VDES turbo-interleaver.
    Returnerar permuterade bitar (för referens — behövs vid turbo-avkodning).
    """
    n = len(bits)
    out = list(bits)
    for s in range(1, n + 1):
        m = (s - 1) % 2
        i = (s - 1) // (2 * k2)
        j = (s - 1) // 2 - i * k2
        t = (19 * i + 1) % (k1 // 2)
        q = (t % 8)
        c = (primes[q] * j + 21 * m) % k2
        pi_s = 2 * (t + c * (k1 // 2) + 1) - m
        if 1 <= pi_s <= n:
            out[pi_s - 1] = bits[s - 1]
    return out

# ── Enkel RSC-dekodar (K=4, för Link ID 11/17 utan FEC kan hoppas) ────────
RSC_POLY_FB = 0b1011   # x^3 + x + 1 (feedback)
RSC_POLY_G2 = 0b1101   # 015 octal
RSC_POLY_G3 = 0b1111   # 017 octal

def rsc_encode_bit(state, input_bit):
    """Enkelt RSC-enkodersteg (K=4, 3 tillståndbitar)."""
    fb = ((input_bit ^ (state >> 2) ^ (state >> 1)) & 1)
    x  = input_bit
    y2 = (fb ^ state ^ (state >> 2)) & 1
    y3 = (fb ^ state ^ (state >> 1) ^ (state >> 2)) & 1
    new_state = ((state << 1) | fb) & 0b111
    return x, y2, y3, new_state

# ── Huvud-avkodningsfunktion ──────────────────────────────────────────────
def decode_candidate(candidate_hex, label="", inverted=False):
    print(f"\n{'='*60}")
    print(f"Kandidat: {label or candidate_hex[:24]}...")
    print(f"{'='*60}")

    data = bytes.fromhex(candidate_hex)
    bits = [int(b) for byte in data for b in f'{byte:08b}']
    if inverted:
        bits = [1 - b for b in bits]
        print(f"(inverterad) ", end="")
    print(f"Längd: {len(bits)} bitar ({len(data)} bytes)")

    # Diagnos: bias och bitfördelning i de första 64 bitarna
    head = bits[:64]
    ones_head = sum(head)
    print(f"Första 64 bitar: {ones_head} ettor ({100*ones_head/len(head):.0f}%)")

    # Steg 1: Sök Link ID (testar både råa RM-kodord och OTA-mönster XOR mask)
    hits = find_link_id(bits, max_offset=64, max_errors=4)
    if not hits:
        print("Inget Link ID hittades (max 4 fel inom 64-bitars offset, raw+OTA).")
        # Visa råa bitar som hex för vidare diagnos
        raw_hex = ''.join(f'{byte:02X}' for byte in data[:16])
        print(f"Råa bytes (16): {raw_hex}")
        return

    best_off, best_lid, best_err, best_var = hits[0]
    info = LINK_ID_INFO[best_lid]
    print(f"\nLink ID funnet: {best_lid}  (offset={best_off}, fel={best_err}, {best_var})")
    print(f"  → {info['data_bits']} databitar, FEC: {info['fec']}")
    print(f"  → QPP-parametrar: k1={info['k1']}, k2={info['k2']}")

    if len(hits) > 1:
        print(f"  Övriga träffar:")
        for off, lid, err, var in hits[1:6]:
            print(f"    offset={off}, Link ID={lid}, fel={err} ({var})")

    # Steg 2: Payload-bitar efter Link ID
    payload_start = best_off + 32
    payload = bits[payload_start:]
    print(f"\nPayload-bitar tillgängliga: {len(payload)}")

    # Steg 3: LFSR-descramble (§A2-1.2.6, gäller payload — Link ID är redan klartext)
    descrambled = lfsr_descramble(payload, offset=0)
    ones_pct = 100.0 * sum(descrambled) / max(1, len(descrambled))
    print(f"Efter descrambling: {ones_pct:.1f}% ettor  "
          f"({'ok — nära 50%' if 35 < ones_pct < 65 else 'AVVIKER'})")

    # Steg 4: CRC-32 på descramblad data
    if info['fec'] == 'none':
        data_bits = info['data_bits']
        if len(descrambled) >= data_bits:
            frame_bits = descrambled[:data_bits]
            payload_only = frame_bits[:-32]
            crc_field_bits = frame_bits[-32:]
            crc_field = int(''.join(map(str, crc_field_bits)), 2)
            crc_calc = crc32_mpeg(payload_only)

            print(f"\nCRC-32 (Link ID {best_lid}, ingen FEC):")
            print(f"  Beräknad: 0x{crc_calc:08X}  |  I frame: 0x{crc_field:08X}")
            print(f"  Match: {'✓ OK' if crc_calc == crc_field else '✗ FAIL'}")

            if crc_calc == crc_field:
                info_bytes = bytes(int(''.join(map(str, payload_only[i:i+8])), 2)
                                   for i in range(0, len(payload_only) - len(payload_only)%8, 8))
                print(f"\nAvkodad payload ({len(info_bytes)} bytes):")
                print(f"  Hex:  {info_bytes.hex().upper()}")
                print(f"  ASCII: {''.join(chr(b) if 32 <= b < 127 else '.' for b in info_bytes)}")
        else:
            print(f"För få bitar för Link ID {best_lid} ({len(descrambled)} < {data_bits})")
    else:
        print(f"\nLink ID {best_lid} kräver turbo-avkodning (R=3/4).")
        preview = bytes(int(''.join(map(str, descrambled[i:i+8])), 2)
                        for i in range(0, min(16*8, len(descrambled)), 8))
        print(f"  Descramblad (16 bytes): {preview.hex().upper()}")

    return descrambled


# ── Testkörning mot alla kandidater ──────────────────────────────────────
if __name__ == "__main__":
    if not sys.stdin.isatty():
        # Läs JSONL från stdin (från vdes_replay --jsonl | ...)
        n_total = 0
        n_decoded = 0
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            if obj.get("signal_type") != "VDES_ASM_CANDIDATE":
                continue
            n_total += 1
            hex_data = obj.get("payload", "")
            fields   = obj.get("fields", {})
            inverted = fields.get("sync_inverted", "0") == "1"
            label = (f"f={obj.get('frequency_hz',0):.0f}Hz "
                     f"pat={fields.get('pattern','?')} "
                     f"off={fields.get('sync_offset_bits','?')} "
                     f"err={fields.get('sync_errors','?')}"
                     f"{' INV' if inverted else ''}")
            result = decode_candidate(hex_data, label, inverted=inverted)
            if result is not None:
                n_decoded += 1
        print(f"\n--- {n_total} kandidater, {n_decoded} med Link ID funnet ---")
    else:
        # Offline mot hårdkodade testkandidater
        candidates = [
            ("BB72BA37640BE983BE29C7EFC6AAEC6AAD79DF3CEB7CBB98FCE31AB5EC5A6BCFFED704489EEA762BFA71096E6D0AA189AA3A9A641B2E61EA3A3CBA0ADCE666218A418297ACB6502D77236EF7A89B9B8F68A8EA0DA80C613AEC89C1ED14F088CFDF362ACF21BDC6F29F53886A24EBEE1FAE623E24806BC0A9",  "hit1 vdes_uw err=1"),
            ("7A1ABAEA9422A78464E9C8A7593EC836A133FFFEAA0157D868A92F37EB1B35FF50753F463FEDF990FAA8327BA9EC8EEFE111F5E519E81A18A8766BC7DE4BDBAA08A8EE262ACAAC82DA82680E18FEDA6082F51F51DA93BE71B04EF6B838DB260DCAD9FA3A3AF4CB64C6F6C46F989A3E0FAA8D94FBA5722F0B",  "hit2 vdes_uw err=1"),
            ("E6A8A0EE967F481E7B0201A82D1C7F263E9BCAD39DA12AE2DED8B068BA839B9B7C9ECA87FB73989A06BF2AB8FFF799ABFAEE27912DFF16F76D5C4FD5819C7A38ACE9DDADC489DEA1E0CD00FF6AD2D53C8FB09F3232C7B770DF8CAEFBFA9E9B6A4DD18B0D7DBF9FB83232333EAC32CD8F608562FEFD8EF1CB",  "hit3 sync28 err=2"),
            ("0ABE651DA7719A43C0DDF4F26D9257C91E19FC7A7A594B3E7D93CF47B86AD1D5742EED595165194AAF73FDBCDF04C70B1B1542859A9B056E9776ED780D1457F16F7227F235D37C5DB724BC409E518F0D85154ACB5B40937162F704989159632771F3065DA8554960D051E7F5E6475ED875F95654F1B45511",  "hit9 vdes_uw err=0"),
        ]
        for hex_data, label in candidates:
            decode_candidate(hex_data, label)
