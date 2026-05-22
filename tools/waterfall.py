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
  --max-rows N     Max antal tidsrader i utdata (standard 2000); långa
                   inspelningar decimeras automatiskt i tid
  --vmin DB        Minsta effekt i dBFS (standard auto)
  --vmax DB        Högsta effekt i dBFS (standard auto)
  --cmap NAME      Matplotlib-colormap (standard rainbow)
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
    p.add_argument('--max-rows',   type=int,   default=2000, metavar='N',
                   help='Max tidsrader (längre inspelningar decimeras, standard 2000)')
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


def compute_waterfall_streaming(path, sample_rate, fft_size, overlap,
                                no_dc, time_start, time_end, max_rows):
    """
    Strömmande beräkning av spektrogram — läser filen i block om
    CHUNK_FRAMES FFT-fönster i taget för att hålla minnesanvändningen låg.

    Returnerar (spec_db [n_rows × fft_size], times_s [n_rows], freqs_hz [fft_size]).
    Decimerar i tid om antalet ramar > max_rows.
    """
    import numpy as np
    import os

    CHUNK_FRAMES = 512   # FFT-fönster per I/O-block

    file_size   = os.path.getsize(path)
    total_pairs = file_size // 4
    step        = max(1, int(fft_size * (1.0 - overlap)))

    start_pair = int(time_start * sample_rate)
    start_pair = max(0, min(start_pair, total_pairs))
    end_pair   = int(time_end * sample_rate) if time_end is not None else total_pairs
    end_pair   = max(start_pair, min(end_pair, total_pairs))
    n_pairs    = end_pair - start_pair

    if n_pairs < fft_size:
        raise ValueError('För kort tidsintervall för vald FFT-storlek.')

    # Antal ramar totalt och decimeringskvot
    n_frames_total = max(1, (n_pairs - fft_size) // step + 1)
    # Hur många ramar vi slår ihop till en utdatarad (medelvärde av effekt)
    decimate = max(1, math.ceil(n_frames_total / max_rows))
    n_rows   = math.ceil(n_frames_total / decimate)
    print(f'  Totalt {n_frames_total} ramar  →  {n_rows} utdatarader '
          f'(decimering {decimate}×)', file=sys.stderr)

    window    = np.hanning(fft_size).astype(np.float32)
    win_power = float(np.sum(window ** 2))
    freqs_hz  = (np.fft.fftshift(np.fft.fftfreq(fft_size)) * sample_rate)

    spec_db = np.zeros((n_rows, fft_size), dtype=np.float32)
    times_s = np.zeros(n_rows, dtype=np.float64)

    # Kvarvarande sampler från föregående block (för överlapp över blockgränser)
    tail      = np.zeros(0, dtype=np.complex64)
    frame_idx = 0   # globalt FFT-fönsterindex
    row_acc   = np.zeros(fft_size, dtype=np.float64)  # ackumulering för decimering
    row_n     = 0   # antal fönster i pågående rad
    row_idx   = 0   # utdataradindex
    row_t_acc = 0.0

    samples_per_chunk = CHUNK_FRAMES * step + fft_size

    with open(path, 'rb') as f:
        f.seek(start_pair * 4)
        remaining = n_pairs

        while remaining > 0 and row_idx < n_rows:
            to_read = min(samples_per_chunk, remaining)
            raw = np.frombuffer(f.read(to_read * 4), dtype='<i2')
            if len(raw) < 2:
                break
            chunk = (raw[0::2].astype(np.float32)
                     + 1j * raw[1::2].astype(np.float32)) / 32768.0
            remaining -= len(chunk)

            buf = np.concatenate((tail, chunk))

            pos = 0
            while pos + fft_size <= len(buf) and row_idx < n_rows:
                seg = buf[pos: pos + fft_size] * window
                F   = np.fft.fftshift(np.fft.fft(seg))
                mag2 = np.abs(F) ** 2 / win_power
                if no_dc:
                    mid = fft_size // 2
                    mag2[mid] = (mag2[mid - 1] + mag2[mid + 1]) / 2.0

                t_center = (start_pair + (frame_idx * step + fft_size / 2)) / sample_rate
                row_acc  += mag2
                row_t_acc += t_center
                row_n     += 1
                frame_idx += 1
                pos       += step

                if row_n >= decimate:
                    spec_db[row_idx] = (10 * np.log10(
                        np.maximum(row_acc / row_n, 1e-12))).astype(np.float32)
                    times_s[row_idx] = row_t_acc / row_n
                    row_idx  += 1
                    row_acc[:]= 0.0
                    row_t_acc = 0.0
                    row_n     = 0

            # Behåll svansen (överlapp in i nästa block)
            tail_len = max(0, len(buf) - pos)
            tail = buf[-tail_len:] if tail_len > 0 else np.zeros(0, dtype=np.complex64)

    # Spola ut sista ofullständiga rad
    if row_n > 0 and row_idx < n_rows:
        spec_db[row_idx] = (10 * np.log10(
            np.maximum(row_acc / row_n, 1e-12))).astype(np.float32)
        times_s[row_idx] = row_t_acc / row_n
        row_idx += 1

    return spec_db[:row_idx], times_s[:row_idx], freqs_hz


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

    # ── Beräkna vattenfall (strömmande) ───────────────────────────────────
    import os
    file_size = os.path.getsize(args.file)
    total_pairs = file_size // 4
    duration_total = total_pairs / args.rate
    step = max(1, int(args.fft * (1.0 - args.overlap)))

    print(f"Läser {args.file} …")
    print(f"  {total_pairs} sampler  ({duration_total:.3f} s)  "
          f"sample_rate={args.rate/1e6:.3f} MHz")
    print(f"  FFT-storlek={args.fft}  överlapp={args.overlap:.0%}  steg={step} sampler  "
          f"frekvensupplösning={args.rate/args.fft:.1f} Hz/bin  "
          f"max-rader={args.max_rows}")

    spec_db, times_s, freqs_norm = compute_waterfall_streaming(
        args.file, args.rate, args.fft, args.overlap,
        args.no_dc, args.time_start, args.time_end, args.max_rows)

    freqs_hz = freqs_norm + args.freq   # absolut Hz

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
