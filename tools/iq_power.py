#!/usr/bin/env python3
"""
iq_power.py — Grov signal/brus-analys av IQ16-inspelning.

Läser en .iq16-fil (signed int16, interleaved I/Q), delar upp i
tidsfönster och uppskattar:
  - Brusgolv (median av block-effekter)
  - Signaltoppar och SNR-marginal
  - Var i tid signalaktivitet finns (ASCII-tidslinje)
  - Var i frekvens signalen sitter (FFT-topp inom filens bandbredd)

Användning:
  python3 tools/iq_power.py <fil.iq16> [--rate HZ] [--block-ms MS]
"""

import sys
import os
import argparse
import math


def _dbfs(linear_power):
    """Linjär effekt (normaliserad till int16 full scale = 1.0) → dBFS."""
    return 10.0 * math.log10(max(float(linear_power), 1e-15))


def _percentile(sorted_list, frac):
    idx = int(len(sorted_list) * frac)
    return sorted_list[min(idx, len(sorted_list) - 1)]


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('file', help='IQ16-fil (.iq16)')
    ap.add_argument('--rate', type=int, default=2048000,
                    help='Samplingsfrekvens Hz (standard: 2048000)')
    ap.add_argument('--block-ms', type=float, default=10.0,
                    help='Blockstorlek ms för tidslinjen (standard: 10)')
    ap.add_argument('--width', type=int, default=70,
                    help='Bredd på ASCII-tidslinje (standard: 70)')
    ap.add_argument('--fft-size', type=int, default=65536,
                    help='FFT-storlek för frekvensanalys (standard: 65536)')
    args = ap.parse_args()

    file_size = os.path.getsize(args.file)
    n_pairs   = file_size // 4           # 2 × int16 per IQ-par
    duration  = n_pairs / args.rate

    block_pairs = max(int(args.rate * args.block_ms / 1000), 64)
    scale_sq    = 32768.0 ** 2          # normalisera till int16 full scale

    try:
        import numpy as np
        _numpy = True
    except ImportError:
        _numpy = False

    # ── Tidsdomenanalys ──────────────────────────────────────────────────────
    powers = []
    if _numpy:
        import numpy as np
        with open(args.file, 'rb') as fh:
            while True:
                raw = fh.read(block_pairs * 4)
                if len(raw) < 4:
                    break
                iq = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
                n  = len(iq) // 2
                if n == 0:
                    break
                pwr = float(np.mean(iq[0::2]**2 + iq[1::2]**2)) / scale_sq
                powers.append(pwr)
    else:
        import struct
        with open(args.file, 'rb') as fh:
            while True:
                raw = fh.read(block_pairs * 4)
                if len(raw) < 4:
                    break
                n = len(raw) // 4
                acc = 0.0
                for k in range(n):
                    i16 = struct.unpack_from('<h', raw, k * 4)[0]
                    q16 = struct.unpack_from('<h', raw, k * 4 + 2)[0]
                    acc += i16 * i16 + q16 * q16
                powers.append(acc / n / scale_sq)

    if not powers:
        print('Filen är tom eller för liten.', file=sys.stderr)
        sys.exit(1)

    sorted_p   = sorted(powers)
    n_blocks   = len(powers)
    noise_floor = _percentile(sorted_p, 0.50)   # median
    p10        = _percentile(sorted_p, 0.10)
    p90        = _percentile(sorted_p, 0.90)
    p99        = _percentile(sorted_p, 0.99)
    peak       = sorted_p[-1]

    floor_db   = _dbfs(noise_floor)
    snr_90     = _dbfs(p90)  - floor_db
    snr_99     = _dbfs(p99)  - floor_db
    snr_peak   = _dbfs(peak) - floor_db

    # ── Frekvensanalys: Welch-medelvärde över hela filen (numpy) ────────────
    fft_result = None
    if _numpy:
        import numpy as np
        fft_n = min(args.fft_size, n_pairs)
        fft_n = int(2 ** math.floor(math.log2(max(fft_n, 64))))
        win   = np.hanning(fft_n).astype(np.float32)
        F_acc = np.zeros(fft_n, dtype=np.float64)
        n_fft = 0
        with open(args.file, 'rb') as fh:
            while True:
                raw = fh.read(fft_n * 4)
                if len(raw) < fft_n * 4:
                    break
                iq = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
                c  = (iq[0::2] + 1j * iq[1::2]) / 32768.0
                F_acc += np.abs(np.fft.fft(c * win)) ** 2
                n_fft += 1
        if n_fft > 0:
            F_acc /= n_fft
            fq = np.fft.fftfreq(fft_n, d=1.0 / args.rate)
            # Ignorera DC och närmaste ±1 % av bandbredd (oscillatorläckage)
            dc_bins = max(int(fft_n * 0.01), 4)
            mask    = np.ones(fft_n, dtype=bool)
            mask[:dc_bins]  = False
            mask[-dc_bins:] = False
            peak_bin = int(np.argmax(F_acc * mask))
            fft_result = {
                'peak_hz':  float(fq[peak_bin]),
                'peak_db':  10.0 * math.log10(max(float(F_acc[peak_bin]), 1e-15)),
                'noise_db': 10.0 * math.log10(max(float(np.median(F_acc)), 1e-15)),
                'n_avg':    n_fft,
            }
            fft_result['snr_db'] = fft_result['peak_db'] - fft_result['noise_db']

    # ── Utskrift ─────────────────────────────────────────────────────────────
    print(f'\nIQ-effektanalys: {os.path.basename(args.file)}')
    print(f'  Varaktighet : {duration:.2f} s'
          f'  |  samplingsfrekvens: {args.rate/1e6:.3f} MHz'
          f'  |  {file_size/1e6:.1f} MB')
    print(f'  Block       : {block_pairs} sampler ({args.block_ms:.0f} ms)'
          f'  |  {n_blocks} block')
    print()
    print(f'  Tidsdomen effektstatistik (dBFS, full scale = int16 max):')
    print(f'    Brusgolv (median p50) : {floor_db:+7.1f} dBFS')
    print(f'    10:e percentilen      : {_dbfs(p10):+7.1f} dBFS')
    print(f'    90:e percentilen      : {_dbfs(p90):+7.1f} dBFS   '
          f'({snr_90:+.1f} dB över golv)')
    print(f'    99:e percentilen      : {_dbfs(p99):+7.1f} dBFS   '
          f'({snr_99:+.1f} dB över golv)')
    print(f'    Topp                  : {_dbfs(peak):+7.1f} dBFS   '
          f'({snr_peak:+.1f} dB över golv)')

    if fft_result:
        print()
        print(f'  Frekvensdomän (Welch FFT {fft_n}-p × {fft_result["n_avg"]} fönster):')
        print(f'    Starkaste bin  : {fft_result["peak_hz"]:+.0f} Hz från center')
        print(f'    Spektral SNR   : {fft_result["snr_db"]:.1f} dB  '
              f'(bin-topp vs. median-brus)')

    # ── ASCII-tidslinje ───────────────────────────────────────────────────────
    # Tecken representerar energi relativt brusgolvet:
    #   · = ±3 dB   ▁ = +3–8 dB   ▄ = +8–15 dB   █ = ≥+15 dB
    chars = []
    for p in powers:
        above = _dbfs(p) - floor_db
        if   above < 3:   c = '·'
        elif above < 8:   c = '▁'
        elif above < 15:  c = '▄'
        else:             c = '█'
        chars.append(c)

    width = args.width
    print()
    print(f'  Tidslinje  (varje tecken = {args.block_ms:.0f} ms)  '
          f'· <+3dB  ▁ +3–8  ▄ +8–15  █ ≥+15 dB')
    for i in range(0, len(chars), width):
        t = i * args.block_ms / 1000.0
        print(f'  {t:6.2f}s │{"".join(chars[i:i+width])}')

    # ── Bedömning ─────────────────────────────────────────────────────────────
    print()
    margin = snr_peak
    if margin >= 20:
        verdict = f'Tydlig signal ({margin:.0f} dB) — bör vara avkodningsbar'
    elif margin >= 10:
        verdict = f'Svag signal ({margin:.0f} dB) — möjlig avkodning vid rätt kanal/symbolhastighet'
    elif margin >= 5:
        verdict = f'Mycket svag signal ({margin:.0f} dB) — tveksam avkodning'
    else:
        verdict = f'Ingen tydlig signal ({margin:.0f} dB SNR) — troligen bara brus'

    print(f'  Bedömning: {verdict}')
    print()


if __name__ == '__main__':
    main()
