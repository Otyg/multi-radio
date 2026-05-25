#!/usr/bin/env python3
"""
vdes_synth.py — Syntetisk VDES VDE-TER pi/4-DQPSK-signalgenerator.

Skapar en .iq16-inspelningsfil med ett eller flera syntetiska VDES ASM-burstar
(Link ID 11, ingen FEC, känd nyttolast) för att verifiera demodulator/decoder-kedjan.

Användning:
  python3 tools/vdes_synth.py OUTPUT.iq16 [OPTIONS]

Options (alla med standardvärden):
  --rate HZ         Samplingsfrekvens (standard 2048000)
  --freq HZ         Centerfrekvens för filen (standard 161862500)
  --offset HZ       Kanaloffset från center (standard -50000 = 161.8125 MHz)
  --sym-rate BAUD   Symbolhastighet (standard 76800)
  --snr-db DB       SNR (standard 20)
  --n-bursts N      Antal burstar att generera (standard 3)
  --burst-gap-ms MS Tystnad mellan burstar i ms (standard 50)
  --lid LID         Link ID att använda: 11 eller 17 (standard 11)
  --payload HEX     Nyttolast som hex (standard: slumpad data)
"""

import sys
import struct
import math
import random
import argparse

# ── VDES-parametrar ────────────────────────────────────────────────────────

TRAINING_BITS = [1,1,1,1,1,1,0,0,1,1,0,1,0,1,0,0,0,0,0,1,1,0,0,1,0,1,0]  # 27 bitar

LINK_IDS = {
    5:  [1,1,0,1,0,1,0,1, 1,1,1,0,1,1,0,1, 0,1,1,1,1,1,1,0, 1,0,1,1,1,1,1,1],
    11: [1,1,1,0,1,1,0,1, 0,0,1,0,1,1,1,0, 1,1,0,0,0,0,1,0, 0,1,1,1,1,1,0,0],
    17: [1,0,0,0,0,1,1,1, 0,0,1,1,0,1,1,1, 0,0,1,0,0,1,0,0, 1,1,1,0,0,1,0,1],
}
LINK_ID_MASK = [1,1,0,0,0,0,1,0, 1,1,1,0,0,0,1,0, 1,0,0,0,1,1,1,0, 0,1,0,0,1,1,1,1]
LINK_ID_INFO = {
    11: {'data_bits': 400, 'fec': 'none'},
    17: {'data_bits': 1840, 'fec': 'none'},
}

# ── LFSR-scrambler ─────────────────────────────────────────────────────────

LFSR_INIT = [1,0,0,1,0,1,0,0,0,0,0,0,0,0,0]

def lfsr_keystream(n):
    state = list(LFSR_INIT)
    out = []
    for _ in range(n):
        fb = state[14] ^ state[13]
        out.append(fb)
        state = [fb] + state[:-1]
    return out

# ── CRC-32/MPEG-2 ─────────────────────────────────────────────────────────

def crc32_mpeg(bits):
    crc = 0xFFFFFFFF
    for b in bits:
        if ((crc >> 31) ^ b) & 1:
            crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
        else:
            crc = (crc << 1) & 0xFFFFFFFF
    return crc

# ── pi/4-DQPSK-modulator ──────────────────────────────────────────────────

DIBIT_PHASE = {
    (0, 0): +math.pi / 4,
    (0, 1): +3 * math.pi / 4,
    (1, 1): -3 * math.pi / 4,
    (1, 0): -math.pi / 4,
}

# Exakta RRC-koefficienter från libliquid (k=4, m=5, beta=0.35):
# liquid_firdes_prototype(LIQUID_FIRFILT_RRC, 4, 5, 0.35, 0.0, h)
# Filterets fördröjning = m*k/2 = 10 sampler vid 4×symbolhastighet.
_LIQUID_RRC_K4M5B35 = [
    0.0075026425, -0.0023777592, -0.0116266143, -0.0102706710,
    0.0020429923,  0.0133202625,  0.0095723830, -0.0095490692,
   -0.0254536718, -0.0147751728,  0.0256149787,  0.0653443933,
    0.0571193360, -0.0220726319, -0.1351641417, -0.1886331439,
   -0.0846904367,  0.2068709880,  0.6077735424,  0.9571257830,
    1.0956338644,  0.9571257830,  0.6077735424,  0.2068709880,
   -0.0846904367, -0.1886331439, -0.1351641417, -0.0220726319,
    0.0571193360,  0.0653443933,  0.0256149787, -0.0147751728,
   -0.0254536718, -0.0095490692,  0.0095723830,  0.0133202625,
    0.0020429923, -0.0102706710, -0.0116266143, -0.0023777592,
    0.0075026425,
]
_RRC_K = 4   # sampler per symbol vid filterets samplingshastighet


def modulate_bits(bits, sym_rate, sample_rate, freq_offset_hz=0.0,
                  beta=0.35, rolloff_syms=5):
    """
    Modulerar ett bitfält till komplexa baseband-sampler med pi/4-DQPSK.
    Använder exakt samma RRC-prototype (k=4, m=5, beta=0.35) som libliquid
    firinterp_crcf_create_prototype och symsync_crcf, vilket garanterar
    noll ISI vid korrekt samplings-tidpunkt.
    """
    assert len(bits) % 2 == 0, "Antalet bitar måste vara jämnt (dibitar)"
    import numpy as np

    n_syms = len(bits) // 2

    # Generera symbolfaserna
    theta = 0.0
    syms  = []
    for i in range(n_syms):
        d0, d1 = bits[2*i], bits[2*i + 1]
        theta += DIBIT_PHASE[(d0, d1)]
        syms.append(complex(math.cos(theta), math.sin(theta)))

    # Bygg impulstog vid k×symbolhastighet (4×76800 = 307200 Hz).
    # Prepend m=5 nollsymboler för att värma upp TX RRC-filtret (samma som
    # vdes_synth_liq.c BD_SYMSYNC_M pre-flush), sedan trimma bort dessa m*k
    # sampler ur utgången.  Utan pre-flush lägger np.convolve till en
    # m*k = 20-samplers TX-grupp­fördröjning som förskjuter tau i mottagaren.
    k = _RRC_K
    m = rolloff_syms   # = 5; BD_SYMSYNC_M i libliquid-kedjan
    n_total = (m + n_syms) * k
    impulse = np.zeros(n_total, dtype=complex)
    impulse[m * k :: k] = syms   # faktiska symboler börjar vid m*k

    # Filtrera med RRC-prototyp → signal vid 307200 Hz
    h = np.array(_LIQUID_RRC_K4M5B35)
    full = np.convolve(impulse, h)   # längd = n_total + len(h) - 1

    # Ta bort pre-flush-rampen (m*k sampler) och håll resten inkl. TX RRC-svansen
    # (2*m*k = 40 extra sampler = 10 symbolperioder).  Svansen innehåller
    # huvudloberna för de sista m symbolerna och är nödvändig för att mottagarens
    # symsync-MF (fördröjning m symboler) ska hinna dekodera alla n_syms symboler.
    filtered = full[m * k :]

    # Rationell resampling 307200 → sample_rate via FFT (ideal sinc-interpolation).
    # sample_rate/307200 = sample_rate/(k*sym_rate) — måste vara rationellt.
    fs_interp = k * sym_rate   # = 307200 Hz
    ratio = sample_rate / fs_interp
    n_in  = len(filtered)
    n_out = int(round(n_in * ratio))
    F = np.fft.fft(filtered, n_in)
    # Zero-pad/truncate i frekvensdomänen
    F_out = np.zeros(n_out, dtype=complex)
    half = min(n_in, n_out) // 2
    F_out[:half]    = F[:half]
    F_out[n_out-half:] = F[n_in-half:n_in]
    out_np = np.fft.ifft(F_out) * (n_out / n_in)

    # Normalisera amplituden
    peak = np.max(np.abs(out_np))
    if peak > 0:
        out_np /= peak

    # Applicera frekvensoffset
    if freq_offset_hz != 0.0:
        t = np.arange(len(out_np)) / sample_rate
        out_np *= np.exp(2j * math.pi * freq_offset_hz * t)

    return out_np.tolist()


def add_awgn(samples, snr_db):
    """Lägger till komplex AWGN med given SNR i dB."""
    sig_power = sum(abs(s)**2 for s in samples) / len(samples)
    noise_power = sig_power / (10 ** (snr_db / 10))
    std = math.sqrt(noise_power / 2)
    return [s + complex(random.gauss(0, std), random.gauss(0, std))
            for s in samples]


# ── Ramkonstruktion ────────────────────────────────────────────────────────

def build_frame(lid, payload_hex=None):
    """
    Bygger ett VDES ASM-burst (bitvektor):
      [27-bit training] [32-bit Link ID OTA] [n-bit scramblad data + CRC-32]
    """
    info = LINK_ID_INFO[lid]
    n_data = info['data_bits']
    n_payload = n_data - 32   # CRC-32 ingår i data_bits

    # Nyttolast
    if payload_hex:
        payload_bytes = bytes.fromhex(payload_hex)
        payload_bits = [int(b) for byte in payload_bytes for b in f'{byte:08b}']
        payload_bits = payload_bits[:n_payload]
        while len(payload_bits) < n_payload:
            payload_bits.append(0)
    else:
        payload_bits = [random.randint(0, 1) for _ in range(n_payload)]

    # CRC-32 på payload
    crc = crc32_mpeg(payload_bits)
    crc_bits = [(crc >> (31 - i)) & 1 for i in range(32)]
    data_bits = payload_bits + crc_bits    # = n_data bitar

    # LFSR-scramble
    ks = lfsr_keystream(len(data_bits))
    scrambled = [d ^ k for d, k in zip(data_bits, ks)]

    # Link ID OTA
    lid_raw = LINK_IDS[lid]
    lid_ota = [b ^ m for b, m in zip(lid_raw, LINK_ID_MASK)]

    # Training sequence: varje bit expanderas till ett dibit (0→00, 1→11)
    # så att modulate_bits kan behandla hela fältet som 2 bitar per symbol.
    training_doubled = [b for t in TRAINING_BITS for b in (t, t)]  # 27 → 54 bitar

    # Komplett burst: 54 + 32 + n_data bitar (alltid jämnt)
    frame = training_doubled + lid_ota + scrambled
    print(f"  Burst LID{lid}: {len(training_doubled)} training + {len(lid_ota)} LID "
          f"+ {len(scrambled)} scrambled data = {len(frame)} bitar total", file=sys.stderr)
    print(f"  LID OTA: {''.join(map(str, lid_ota))}", file=sys.stderr)
    print(f"  CRC:     0x{crc:08X}", file=sys.stderr)
    return frame


# ── Tyst intervall ─────────────────────────────────────────────────────────

def silence(n_samples):
    return [0+0j] * n_samples


# ── Huvud ──────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('output', help='Utdatafil (.iq16)')
    ap.add_argument('--rate',         type=int,   default=2048000, metavar='HZ')
    ap.add_argument('--freq',         type=int,   default=161862500, metavar='HZ')
    ap.add_argument('--offset',       type=int,   default=-50000, metavar='HZ',
                    help='Kanaloffset från centerfrekvensen (standard -50000)')
    ap.add_argument('--sym-rate',     type=int,   default=76800, metavar='BAUD')
    ap.add_argument('--snr-db',       type=float, default=20.0, metavar='DB')
    ap.add_argument('--n-bursts',     type=int,   default=3, metavar='N')
    ap.add_argument('--burst-gap-ms', type=float, default=50.0, metavar='MS')
    ap.add_argument('--lid',          type=int,   default=11, choices=[11, 17])
    ap.add_argument('--payload',      type=str,   default=None, metavar='HEX')
    ap.add_argument('--seed',         type=int,   default=None, metavar='N',
                    help='PRNG-seed för deterministisk brusgenerering (standard: slumpmässigt)')
    args = ap.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    sample_rate = args.rate
    sym_rate    = args.sym_rate
    snr_db      = args.snr_db
    freq_offset = args.offset

    gap_samples = int(sample_rate * args.burst_gap_ms / 1000)

    print(f"Genererar {args.n_bursts} burst(ar) av LID{args.lid}...", file=sys.stderr)
    print(f"  Centerfrekvens: {args.freq/1e6:.5f} MHz  Kanaloffset: {freq_offset:+.0f} Hz",
          file=sys.stderr)
    print(f"  Symbolhastighet: {sym_rate} baud  SNR: {snr_db:.1f} dB", file=sys.stderr)
    print(f"  Utdatafil: {args.output}", file=sys.stderr)

    all_samples = []

    # Inledande tystnad (100 ms)
    all_samples.extend(silence(int(sample_rate * 0.1)))

    for i in range(args.n_bursts):
        frame_bits = build_frame(args.lid, args.payload)

        # Modulera
        samples = modulate_bits(frame_bits, sym_rate, sample_rate,
                                 freq_offset_hz=float(freq_offset))
        samples = add_awgn(samples, snr_db)
        all_samples.extend(samples)

        # Tystnad efter burst (utom sista)
        if i < args.n_bursts - 1:
            all_samples.extend(silence(gap_samples))

    # Avslutande tystnad (100 ms)
    all_samples.extend(silence(int(sample_rate * 0.1)))

    # Skriv .iq16 (signed int16, interleaved I/Q)
    max_amp = max(abs(s) for s in all_samples) if all_samples else 1.0
    scale = 30000.0 / max(max_amp, 1e-9)

    with open(args.output, 'wb') as f:
        for s in all_samples:
            i16 = max(-32767, min(32767, int(round(s.real * scale))))
            q16 = max(-32767, min(32767, int(round(s.imag * scale))))
            f.write(struct.pack('<hh', i16, q16))

    duration_s = len(all_samples) / sample_rate
    print(f"\nSkrivet {len(all_samples)} sampler ({duration_s:.3f} s) till {args.output}",
          file=sys.stderr)
    print(f"Kör nu:", file=sys.stderr)
    print(f"  MR_AIS_DEBUG=1 build/tools/vdes_replay \\", file=sys.stderr)
    print(f"    --iq {args.output} --rate {sample_rate} --freq {args.freq} \\",
          file=sys.stderr)
    print(f"    --freq-offset {freq_offset} \\", file=sys.stderr)
    print(f"    --demod vdes_burst_demod --decoder vdes_burst_decoder \\", file=sys.stderr)
    print(f"    --param symbol_rate_baud={sym_rate} --param squelch_db=3 \\",
          file=sys.stderr)
    print(f"    --param candidate_bits=2000 \\", file=sys.stderr)
    print(f"    --jsonl 2>/dev/null | python3 tools/vdes_decode.py", file=sys.stderr)


if __name__ == '__main__':
    main()
