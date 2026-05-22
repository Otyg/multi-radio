#!/usr/bin/env python3
"""
waterfall.py — Spektrogram/vattenfall för IQ-inspelningsfiler (.iq16).

Användning:
  python3 tools/waterfall.py FILE.iq16 [OPTIONS]

Alternativ:
  --rate HZ        Samplingsfrekvens (standard 2048000)
  --freq HZ        Centerfrekvens i Hz (standard 0, visas som offset)
  --fft N          FFT-storlek i sampler (standard 2048)
  --overlap F      Överlapp 0.0–0.95 (standard 0.5)
  --vmin DB        Minsta effekt i dBFS (standard auto)
  --vmax DB        Högsta effekt i dBFS (standard auto)
  --cmap NAME      Matplotlib-colormap (standard inferno)
  --out FILE       Spara bild istället för att visa (png/svg/pdf)
  --time-start S   Börja från denna sekund i filen (standard 0)
  --time-end S     Sluta vid denna sekund (standard: hela filen)
  --freq-low HZ    Begränsa frekvensaxeln (relativ offset från center)
  --freq-high HZ   Begränsa frekvensaxeln
  --no-dc          Nollställ DC-binet (hjälper vid DC-spik från SDR)
  --title TEXT     Titel på diagrammet
"""

import sys
import argparse
import math


def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('file', help='IQ-fil (.iq16, signed int16 interleaved I/Q)')
    p.add_argument('--rate',       type=float, default=2_048_000, metavar='HZ')
    p.add_argument('--freq',       type=float, default=0.0, metavar='HZ',
                   help='Centerfrekvens (visas på frekvensaxeln)')
    p.add_argument('--fft',        type=int,   default=2048, metavar='N')
    p.add_argument('--overlap',    type=float, default=0.5, metavar='F')
    p.add_argument('--vmin',       type=float, default=None, metavar='DB')
    p.add_argument('--vmax',       type=float, default=None, metavar='DB')
    p.add_argument('--cmap',       type=str,   default='rainbow', metavar='NAME')
    p.add_argument('--out',        type=str,   default=None, metavar='FILE')
    p.add_argument('--time-start', type=float, default=0.0, metavar='S')
    p.add_argument('--time-end',   type=float, default=None, metavar='S')
    p.add_argument('--freq-low',   type=float, default=None, metavar='HZ',
                   help='Undre frekvens (offset från center)')
    p.add_argument('--freq-high',  type=float, default=None, metavar='HZ',
                   help='Övre frekvens (offset från center)')
    p.add_argument('--no-dc',      action='store_true',
                   help='Nollställ DC-binet (tar bort SDR DC-spik)')
    p.add_argument('--title',      type=str,   default=None)
    return p


def read_iq16(path, sample_rate, time_start, time_end):
    """Läser int16 IQ-fil, returnerar komplex float64-array."""
    import numpy as np
    import os

    file_size = os.path.getsize(path)
    total_pairs = file_size // 4  # 2×int16 per sampler

    start_pair = int(time_start * sample_rate)
    start_pair = max(0, min(start_pair, total_pairs))

    if time_end is not None:
        end_pair = int(time_end * sample_rate)
    else:
        end_pair = total_pairs
    end_pair = max(start_pair, min(end_pair, total_pairs))

    n_pairs = end_pair - start_pair
    if n_pairs <= 0:
        raise ValueError(f"Tomt tidsintervall: start={time_start}s, slut={time_end}s")

    with open(path, 'rb') as f:
        f.seek(start_pair * 4)
        raw = np.frombuffer(f.read(n_pairs * 4), dtype='<i2')

    iq = raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)
    return iq / 32768.0


def compute_waterfall(iq, fft_size, overlap, no_dc):
    """
    Returnerar (spectrogram_dB, times, freqs_norm).
    freqs_norm är normaliserade frekvenser i [-0.5, 0.5] (gånger sample_rate = Hz).
    """
    import numpy as np

    step = max(1, int(fft_size * (1.0 - overlap)))
    window = np.hanning(fft_size).astype(np.float32)
    win_power = np.sum(window ** 2)

    n_frames = max(1, (len(iq) - fft_size) // step + 1)

    spec = np.zeros((n_frames, fft_size), dtype=np.float32)
    for i in range(n_frames):
        seg = iq[i * step: i * step + fft_size]
        if len(seg) < fft_size:
            break
        frame = seg * window
        F = np.fft.fft(frame)
        F = np.fft.fftshift(F)
        mag2 = (np.abs(F) ** 2) / win_power
        if no_dc:
            mag2[fft_size // 2] = (mag2[fft_size // 2 - 1] + mag2[fft_size // 2 + 1]) / 2
        spec[i] = mag2

    # Konvertera till dBFS (normaliserat mot full scale)
    spec_db = 10 * np.log10(np.maximum(spec, 1e-12))

    # Tidsvektor (centrum av varje fönster)
    times = (np.arange(n_frames) * step + fft_size / 2)  # i sampler

    # Normaliserade frekvenser: [-0.5 … +0.5] × sample_rate
    freqs_norm = np.fft.fftshift(np.fft.fftfreq(fft_size))  # i cykler/sampler

    return spec_db, times, freqs_norm


def main():
    args = build_parser().parse_args()

    try:
        import numpy as np
    except ImportError:
        sys.exit('Kräver numpy:  pip install numpy')
    try:
        import matplotlib
        if args.out:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.ticker as ticker
    except ImportError:
        sys.exit('Kräver matplotlib:  pip install matplotlib')

    # ── Läs data ──────────────────────────────────────────────────────────
    print(f"Läser {args.file} …")
    iq = read_iq16(args.file, args.rate, args.time_start, args.time_end)
    duration = len(iq) / args.rate
    print(f"  {len(iq)} sampler  ({duration:.3f} s)  sample_rate={args.rate/1e6:.3f} MHz")

    # ── Beräkna vattenfall ─────────────────────────────────────────────────
    step = max(1, int(args.fft * (1.0 - args.overlap)))
    print(f"  FFT-storlek={args.fft}  överlapp={args.overlap:.0%}  steg={step} sampler  "
          f"frekvensupplösning={args.rate/args.fft:.1f} Hz/bin")

    spec_db, times_samp, freqs_norm = compute_waterfall(iq, args.fft, args.overlap,
                                                         args.no_dc)

    # Axlar i riktiga enheter
    times_s   = times_samp / args.rate + args.time_start  # sekunder från filenstart
    freqs_hz  = freqs_norm * args.rate + args.freq         # absolut Hz (eller offset)

    # ── Frekvensklippning ─────────────────────────────────────────────────
    f_low  = args.freq + (args.freq_low  if args.freq_low  is not None else -args.rate / 2)
    f_high = args.freq + (args.freq_high if args.freq_high is not None else +args.rate / 2)
    bin_lo = np.searchsorted(freqs_hz, f_low)
    bin_hi = np.searchsorted(freqs_hz, f_high)
    bin_lo = max(0, min(bin_lo, args.fft - 1))
    bin_hi = max(bin_lo + 1, min(bin_hi, args.fft))

    spec_clip  = spec_db[:, bin_lo:bin_hi]
    freqs_clip = freqs_hz[bin_lo:bin_hi]

    # ── Autoskalning ──────────────────────────────────────────────────────
    vmin = args.vmin if args.vmin is not None else float(np.percentile(spec_clip, 5))
    vmax = args.vmax if args.vmax is not None else float(np.percentile(spec_clip, 99))
    print(f"  Effektskala: {vmin:.1f} … {vmax:.1f} dBFS  "
          f"(auto={args.vmin is None and args.vmax is None})")

    # ── Rita ──────────────────────────────────────────────────────────────
    fig_w = max(10, 12)
    fig_h = max(5, min(12, len(times_s) / 200 + 2))
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    extent = [freqs_clip[0] / 1e6, freqs_clip[-1] / 1e6,
              times_s[-1],          times_s[0]]
    im = ax.imshow(spec_clip, aspect='auto', origin='upper',
                   extent=extent, cmap=args.cmap,
                   vmin=vmin, vmax=vmax,
                   interpolation='nearest')

    cbar = fig.colorbar(im, ax=ax, pad=0.01, fraction=0.02)
    cbar.set_label('dBFS', fontsize=9)

    ax.set_xlabel('Frekvens (MHz)', fontsize=10)
    ax.set_ylabel('Tid (s)', fontsize=10)

    title = args.title
    if title is None:
        import os
        title = (f"{os.path.basename(args.file)}"
                 f"  center={args.freq/1e6:.5f} MHz"
                 f"  rate={args.rate/1e6:.3f} MHz"
                 f"  fft={args.fft}")
    ax.set_title(title, fontsize=9)

    # Markera VDES VDE-TER-bandet om det är synligt
    vdes_lo = 161.7875e6
    vdes_hi = 161.9375e6
    ax_lo = freqs_clip[0]
    ax_hi = freqs_clip[-1]
    if ax_lo < vdes_hi and ax_hi > vdes_lo:
        lo_mhz = max(vdes_lo, ax_lo) / 1e6
        hi_mhz = min(vdes_hi, ax_hi) / 1e6
        ax.axvspan(lo_mhz, hi_mhz, alpha=0.08, color='cyan',
                   label=f'VDE-TER ({lo_mhz:.4f}–{hi_mhz:.4f} MHz)')
        ax.legend(loc='upper right', fontsize=7)

    ax.xaxis.set_major_formatter(ticker.FormatStrFormatter('%.4f'))
    plt.tight_layout()

    if args.out:
        fig.savefig(args.out, dpi=150, bbox_inches='tight')
        print(f"Sparad: {args.out}")
    else:
        plt.show()


if __name__ == '__main__':
    main()
