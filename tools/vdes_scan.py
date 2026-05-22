#!/usr/bin/env python3
"""
vdes_scan.py — Snabb kanalscanner för VDE-TER IQ-inspelningar.

Beräknar medeleffekt per 25 kHz-kanal inom det avsatta VDE-TER-bandet
(161.7875–161.9375 MHz) och pekar ut var signalaktivitet finns.

Användning:
  python3 tools/vdes_scan.py RAW.iq16 [CENTER_HZ] [SAMPLE_RATE_HZ]

Standardvärden: center=161862500, rate=2048000
"""

import sys
import struct
import math

CENTER_HZ   = 161_862_500
SAMPLE_RATE = 2_048_000
BLOCK       = 65536          # samples per FFT
SKIP_BLOCKS = 0              # hoppa över de första N blocken

# VDE-TER-band: 161.7875–161.9375 MHz, 25 kHz-kanalsteg
BAND_LO  = 161_787_500
BAND_HI  = 161_937_500
CH_STEP  =      25_000

def channel_centers(center, rate, band_lo, band_hi, step):
    """Alla kanalcentra som faller inom bandet OCH inom FFT-täckning."""
    half = rate / 2
    freqs = []
    f = band_lo
    while f <= band_hi:
        offset = f - center
        if abs(offset) <= half:
            freqs.append(f)
        f += step
    return freqs

def fft_mag_sq(samples):
    """Radix-2 Cooley-Tukey DFT (enkel Python, tillräcklig för power scan)."""
    import cmath
    n = len(samples)
    if n == 1:
        return [abs(samples[0])**2]
    even = fft_mag_sq(samples[0::2])
    odd  = fft_mag_sq(samples[1::2])
    # We only need magnitudes; use numpy-free approach via manual recursion —
    # but for speed just return a simple periodogram via Welch-style binning.
    # (Replaced below by numpy when available.)
    raise NotImplementedError

def run(iq_path, center_hz, sample_rate):
    ch_freqs = channel_centers(center_hz, sample_rate, BAND_LO, BAND_HI, CH_STEP)
    if not ch_freqs:
        print("Inga kanaler inom täckning.")
        return

    # Bin-index per kanalcentrum
    def freq_to_bin(f):
        offset = f - center_hz
        return round(offset / sample_rate * BLOCK) % BLOCK

    try:
        import numpy as np
        use_numpy = True
    except ImportError:
        use_numpy = False
        print("(numpy saknas — installera för bättre precision)", file=sys.stderr)

    # Ackumulera effekt per frekvensbin
    if use_numpy:
        power = np.zeros(BLOCK, dtype=np.float64)
        n_blocks = 0
        with open(iq_path, 'rb') as f:
            for _ in range(SKIP_BLOCKS):
                f.read(BLOCK * 4)
            while True:
                raw = f.read(BLOCK * 4)
                if len(raw) < BLOCK * 4:
                    break
                iq = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
                c  = (iq[0::2] + 1j * iq[1::2]) / 32768.0
                # Hamming window
                c *= np.hamming(BLOCK)
                F  = np.fft.fft(c)
                power += np.abs(F)**2
                n_blocks += 1
        if n_blocks == 0:
            print("Inga fullständiga block hittades.", file=sys.stderr)
            return
        power /= n_blocks
        # FFT-frekvens: bin k → frekvens center + k/N*rate (med wrap för negativa)
        freq_axis = center_hz + np.fft.fftfreq(BLOCK, d=1/sample_rate)
        # Per kanal: snitt av binnar inom ±CH_STEP/2
        half_ch = CH_STEP // 2
        results = []
        for cf in ch_freqs:
            mask = np.abs(freq_axis - cf) < half_ch
            p = float(np.mean(power[mask])) if np.any(mask) else 0.0
            results.append((cf, p))
    else:
        # Fallback utan numpy: bara total power per block
        results = [(cf, 0.0) for cf in ch_freqs]
        print("Kör utan numpy — kan bara rapportera total effekt.", file=sys.stderr)

    # Sortera efter effekt, skriv ut
    ref_db = max(p for _, p in results) if results else 1.0
    if ref_db <= 0:
        ref_db = 1.0

    print(f"\nVDE-TER kanalscanner  center={center_hz/1e6:.4f} MHz  "
          f"rate={sample_rate/1e6:.3f} MHz  block={BLOCK}")
    print(f"{'Kanal (MHz)':>14}  {'Offset (kHz)':>14}  {'Rel. effekt (dB)':>18}  Kommentar")
    print("-" * 65)

    sorted_r = sorted(results, key=lambda x: -x[1])
    top_p    = sorted_r[0][1] if sorted_r else 1.0

    for cf, p in results:
        offset_khz = (cf - center_hz) / 1000
        db = 10 * math.log10(p / ref_db) if p > 0 else -99.0
        rel = 10 * math.log10(p / top_p) if p > 0 and top_p > 0 else -99.0
        comment = ""
        if rel > -3:
            comment = "*** TOPP"
        elif rel > -10:
            comment = "* aktiv?"
        print(f"  {cf/1e6:12.5f}  {offset_khz:+14.1f}  {db:+18.1f}  {comment}")

    print()
    print("Kanalcentra med högst effekt (sorterat):")
    for cf, p in sorted_r[:5]:
        offset_khz = (cf - center_hz) / 1000
        db = 10 * math.log10(p / ref_db) if p > 0 else -99.0
        print(f"  {cf/1e6:.5f} MHz  offset={offset_khz:+.0f} kHz  {db:+.1f} dB (rel)")

    print()
    print("Förslag på vdes_replay-kommando för den starkaste kanalen:")
    best_cf, _ = sorted_r[0]
    best_offset = best_cf - center_hz
    print(f"  --freq-offset {best_offset:.0f} "
          f"  # kanalnr ca {best_cf/1e6:.4f} MHz")


if __name__ == "__main__":
    path   = sys.argv[1] if len(sys.argv) > 1 else None
    center = int(sys.argv[2]) if len(sys.argv) > 2 else CENTER_HZ
    rate   = int(sys.argv[3]) if len(sys.argv) > 3 else SAMPLE_RATE

    if path is None:
        print(__doc__)
        sys.exit(1)

    run(path, center, rate)
