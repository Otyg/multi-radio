#!/usr/bin/env python3
"""
iq_channels.py — Identifierar aktiva frekvenskanaler i en IQ16-inspelning.

Beräknar medeleffekten per kanal för fyra kanalnät (25/50/75/100 kHz steg)
och listar offset-värden med aktivitet, klara att klistra in i vdes_scan.py.

Användning:
  python3 tools/iq_channels.py <fil.iq16> [--rate HZ] [--threshold DB]
"""

import sys
import os
import argparse
import math


def welch_spectrum(path, fft_n, rate):
    """Returnerar (power_array, freq_array) via Welch-medelvärde."""
    import numpy as np
    win   = np.hanning(fft_n).astype(np.float32)
    F_acc = np.zeros(fft_n, dtype=np.float64)
    n_avg = 0
    with open(path, 'rb') as fh:
        while True:
            raw = fh.read(fft_n * 4)
            if len(raw) < fft_n * 4:
                break
            iq = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
            c  = (iq[0::2] + 1j * iq[1::2]) / 32768.0
            F_acc += np.abs(np.fft.fft(c * win)) ** 2
            n_avg += 1
    if n_avg == 0:
        return None, None, 0
    F_acc /= n_avg
    freq  = np.fft.fftfreq(fft_n, d=1.0 / rate)
    return F_acc, freq, n_avg


def channel_powers(F, freq, ch_step, rate):
    """
    Beräknar medeleffekt (linjär) per kanal för givet kanalsteg.
    Returnerar dict {offset_hz: mean_power} för alla kanaler inom bandwidth.
    """
    import numpy as np
    half_ch  = ch_step / 2
    half_bw  = rate / 2 - half_ch
    offsets  = []
    f = 0
    while f <= half_bw:
        offsets.append(int(f))
        if f != 0:
            offsets.append(int(-f))
        f += ch_step

    result = {}
    for off in offsets:
        mask = np.abs(freq - off) < half_ch
        if np.any(mask):
            result[off] = float(np.mean(F[mask]))
        else:
            result[off] = 0.0
    return result


def active_offsets(ch_powers, threshold_db):
    """
    Hittar kanaler med effekt > median + threshold_db.
    Returnerar sorterad lista med offset-värden.
    """
    import numpy as np
    powers = list(ch_powers.values())
    if not powers:
        return []
    floor  = float(np.median(powers))
    thresh = floor * (10.0 ** (threshold_db / 10.0))
    active = sorted(
        off for off, p in ch_powers.items() if p > thresh
    )
    return active


def db(p):
    return 10.0 * math.log10(max(p, 1e-15))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('file',            help='IQ16-fil')
    ap.add_argument('--rate',          type=int,   default=2048000,
                    help='Samplingsfrekvens Hz (standard: 2048000)')
    ap.add_argument('--threshold',     type=float, default=6.0,
                    help='Tröskel dB över median kanaleffekt (standard: 6)')
    ap.add_argument('--fft-size',      type=int,   default=65536,
                    help='FFT-storlek (standard: 65536)')
    ap.add_argument('--no-table',      action='store_true',
                    help='Skriv bara offset-listor, ingen detaljerad tabell')
    args = ap.parse_args()

    try:
        import numpy as np
    except ImportError:
        print('numpy krävs: pip install numpy', file=sys.stderr)
        sys.exit(1)

    file_size = os.path.getsize(args.file)
    n_pairs   = file_size // 4
    if n_pairs == 0:
        print('Filen är tom.', file=sys.stderr)
        sys.exit(1)

    fft_n = min(args.fft_size, n_pairs)
    fft_n = int(2 ** math.floor(math.log2(max(fft_n, 64))))

    print(f'Analyserar {os.path.basename(args.file)}'
          f'  rate={args.rate/1e6:.3f} MHz  FFT={fft_n}  '
          f'tröskel={args.threshold:.0f} dB ...', flush=True)

    F, freq, n_avg = welch_spectrum(args.file, fft_n, args.rate)
    if F is None:
        print('För liten fil för FFT-analys.', file=sys.stderr)
        sys.exit(1)

    print(f'  {n_avg} FFT-fönster medelvärdesbildade\n')

    grids  = [25_000, 50_000, 75_000, 100_000]
    result = {}   # step → sorted active offsets

    for step in grids:
        cp     = channel_powers(F, freq, step, args.rate)
        active = active_offsets(cp, args.threshold)
        result[step] = (cp, active)

    # ── Detaljerad tabell per grid ────────────────────────────────────────────
    if not args.no_table:
        for step in grids:
            cp, active = result[step]
            floor_lin  = float(np.median(list(cp.values())))
            floor_db_v = db(floor_lin)

            offsets_sorted = sorted(cp.keys())
            label_width    = 10

            print('-' * 60)
            print(f'  {step//1000} kHz-raster   '
                  f'({len(cp)} kanaler, golv={floor_db_v:.1f} dBFS, '
                  f'tröskel=+{args.threshold:.0f} dB)')
            print(f'  {"Offset":>{label_width}}    {"dB":>7}  {"ΔdB":>6}  (ΔdB = avvikelse från medianbrus)')
            print()
            for off in offsets_sorted:
                p     = cp[off]
                d     = db(p)
                delta = d - floor_db_v
                mark  = '  ◀' if off in active else ''
                print(f'  {off:>{label_width}}  {d:>+8.1f}  {delta:>+6.1f}{mark}')
            print()

    # ── Rena offset-listor ────────────────────────────────────────────────────
    print('═' * 60)
    print(f'  Aktiva kanaler (tröskel: +{args.threshold:.0f} dB över median)')
    print('═' * 60)
    print()

    for step in grids:
        _, active = result[step]
        label     = f'{step//1000} kHz'
        if active:
            space_list = ' '.join(str(o) for o in active)
            comma_list = ','.join(str(o) for o in active)
        else:
            space_list = '(inga)'
            comma_list = ''

        print(f'  {label:8}  {space_list}')

    print()
    print('  --offsets kommandon (klistra direkt i vdes_scan.py):')
    print()
    for step in grids:
        _, active = result[step]
        label     = f'{step//1000}kHz:'
        if active:
            val = ','.join(str(o) for o in active)
            print(f'    {label:8}  --offsets {val}')
        else:
            print(f'    {label:8}  (inga aktiva kanaler)')
    print()


if __name__ == '__main__':
    main()
