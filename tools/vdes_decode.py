#!/usr/bin/env python3
"""
vdes_decode.py — VDES ASM VDE-TER frame decoder

Steg 1: Link-ID-sökning + LFSR-descrambling + CRC-32-validering
Steg 2: Turbo-avkodning för Link ID 5 och 19 (kräver scipy/turbodecoder)

Avkodningskedja (ITU-R M.2092 / IALA G1139 Ed.3):
  [sync] → [32-bit Link ID] → [descramble] → [depuncture] → [turbo decode] → [CRC check]

Kör mot 0-fels-kandidaten från vdes_replay:
  python3 vdes_decode.py
"""

import numpy as np
from itertools import product

# ── Kända Link-ID-koder (32 bitar, LSB-first i standard → vi testar båda) ──
LINK_IDS = {
    5:  list(map(int, '11010101111011010111111010111111')),
    11: list(map(int, '11101101001011101100001001111100')),
    17: list(map(int, '10000111001101110010010011100101')),
    19: list(map(int, '10001111010010000010010000011010')),
}

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
    Söker igenom bits[offset:offset+32] för offset 0..max_offset
    och jämför mot alla fyra kända Link-ID-koder.
    Returnerar lista av (offset, link_id, errors) sorterad på errors.
    """
    hits = []
    for offset in range(max_offset):
        if offset + 32 > len(bits):
            break
        window = bits[offset:offset + 32]
        for lid, pattern in LINK_IDS.items():
            err = hamming(window, pattern)
            if err <= max_errors:
                hits.append((offset, lid, err))
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
def decode_candidate(candidate_hex, label=""):
    print(f"\n{'='*60}")
    print(f"Kandidat: {label or candidate_hex[:24]}...")
    print(f"{'='*60}")

    data = bytes.fromhex(candidate_hex)
    bits = [int(b) for byte in data for b in f'{byte:08b}']
    print(f"Längd: {len(bits)} bitar ({len(data)} bytes)")

    # Steg 1: Sök Link ID
    hits = find_link_id(bits, max_offset=60, max_errors=3)
    if not hits:
        print("Inget Link ID hittades (max 3 fel inom 60-bitars offset).")
        return

    best_off, best_lid, best_err = hits[0]
    info = LINK_ID_INFO[best_lid]
    print(f"\nLink ID funnet: {best_lid}  (offset={best_off}, fel={best_err})")
    print(f"  → {info['data_bits']} databitar, FEC: {info['fec']}")
    print(f"  → QPP-parametrar: k1={info['k1']}, k2={info['k2']}")

    # Visa alla nära träffar
    if len(hits) > 1:
        print(f"  Övriga träffar (max 3 fel):")
        for off, lid, err in hits[1:5]:
            print(f"    offset={off}, Link ID={lid}, fel={err}")

    # Steg 2: Payload-bitar efter Link ID
    payload_start = best_off + 32
    payload = bits[payload_start:]
    print(f"\nPayload-bitar tillgängliga: {len(payload)}")

    # Steg 3: Descramble (LFSR-offset = 0, scramblingen gäller från payload-start)
    # Notera: Link ID-koden descrambles INTE — den sänds i klartext
    descrambled = lfsr_descramble(payload, offset=0)
    ones_pct = 100.0 * sum(descrambled) / max(1, len(descrambled))
    print(f"Efter descrambling: {ones_pct:.1f}% ettor  "
          f"({'bra — nära 50%' if 35 < ones_pct < 65 else 'AVVIKER — kan indikera problem'})")

    # Steg 4: CRC-32 på descramblad data
    # För Link ID 11 (ingen FEC): datagram = 400 bitar direkt
    # För Link ID 5 (turbo R=3/4): behöver turbo-avkodning först
    if info['fec'] == 'none':
        data_bits = info['data_bits']
        if len(descrambled) >= data_bits:
            frame_bits = descrambled[:data_bits]
            # CRC-32 täcker alla bitar inklusive CRC-fältet självt → residue ≠ 0
            # Prova med CRC i slutet (sista 32 bitarna)
            payload_only = frame_bits[:-32]
            crc_field_bits = frame_bits[-32:]
            crc_field = int(''.join(map(str, crc_field_bits)), 2)
            crc_calc = crc32_mpeg(payload_only)

            print(f"\nCRC-32 check (Link ID {best_lid}, ingen FEC):")
            print(f"  Beräknad CRC: 0x{crc_calc:08X}")
            print(f"  CRC i frame:  0x{crc_field:08X}")
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
        print("  → Implementeras i nästa fas (BCJR/MAP-algoritm)")
        # Visa ändå de första descramblad-bytarna som diagnos
        preview = bytes(int(''.join(map(str, descrambled[i:i+8])), 2)
                        for i in range(0, min(16*8, len(descrambled)), 8))
        print(f"  Descramblad (16 bytes för diagnos): {preview.hex().upper()}")

    return descrambled


# ── Testkörning mot alla kandidater ──────────────────────────────────────
if __name__ == "__main__":
    candidates = [
        ("BB72BA37640BE983BE29C7EFC6AAEC6AAD79DF3CEB7CBB98FCE31AB5EC5A6BCFFED704489EEA762BFA71096E6D0AA189AA3A9A641B2E61EA3A3CBA0ADCE666218A418297ACB6502D77236EF7A89B9B8F68A8EA0DA80C613AEC89C1ED14F088CFDF362ACF21BDC6F29F53886A24EBEE1FAE623E24806BC0A9",  "hit1 vdes_uw err=1"),
        ("7A1ABAEA9422A78464E9C8A7593EC836A133FFFEAA0157D868A92F37EB1B35FF50753F463FEDF990FAA8327BA9EC8EEFE111F5E519E81A18A8766BC7DE4BDBAA08A8EE262ACAAC82DA82680E18FEDA6082F51F51DA93BE71B04EF6B838DB260DCAD9FA3A3AF4CB64C6F6C46F989A3E0FAA8D94FBA5722F0B",  "hit2 vdes_uw err=1"),
        ("E6A8A0EE967F481E7B0201A82D1C7F263E9BCAD39DA12AE2DED8B068BA839B9B7C9ECA87FB73989A06BF2AB8FFF799ABFAEE27912DFF16F76D5C4FD5819C7A38ACE9DDADC489DEA1E0CD00FF6AD2D53C8FB09F3232C7B770DF8CAEFBFA9E9B6A4DD18B0D7DBF9FB83232333EAC32CD8F608562FEFD8EF1CB",  "hit3 sync28 err=2"),
        ("0ABE651DA7719A43C0DDF4F26D9257C91E19FC7A7A594B3E7D93CF47B86AD1D5742EED595165194AAF73FDBCDF04C70B1B1542859A9B056E9776ED780D1457F16F7227F235D37C5DB724BC409E518F0D85154ACB5B40937162F704989159632771F3065DA8554960D051E7F5E6475ED875F95654F1B45511",  "hit9 vdes_uw err=0"),
    ]

    for hex_data, label in candidates:
        decode_candidate(hex_data, label)
