#!/usr/bin/env python3
"""
vdes_burst_viz.py — Tiddomänanalys av IQ-inspelning för VDE-TER-burstdetektering.

Visar energi vs tid för en given frekvensoffset, så vi kan se om det
finns burstaktivitet överhuvudtaget.

Användning:
  python3 tools/vdes_burst_viz.py RAW.iq16 [freq_offset_hz] [center_hz] [rate_hz]

Standardvärden: offset=0, center=161862500, rate=2048000
"""

import sys
import struct
import math

def bandpass_energy(iq_path, center_hz, sample_rate, freq_offset_hz,
                    channel_bw_hz=100_000, block_ms=5):
    """
    Returnerar lista av (tid_s, energi_dB) med upplösning block_ms millisekunder.
    Frekvensoffset mixas ned till DC och sedan filtreras till ±channel_bw/2.
    """
    import cmath

    block_samples = int(sample_rate * block_ms / 1000)
    # Decimation: håll bara de binnar som faller inom ±channel_bw/2
    # i FFT av varje block.
    fft_n = 1
    while fft_n < block_samples:
        fft_n <<= 1

    try:
        import numpy as np
        use_np = True
    except ImportError:
        use_np = False

    results = []
    t = 0.0
    dt = block_samples / sample_rate

    with open(iq_path, 'rb') as f:
        block_idx = 0
        while True:
            raw = f.read(block_samples * 4)
            if len(raw) < block_samples * 4:
                break

            if use_np:
                iq = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
                c  = iq[0::2] + 1j * iq[1::2]
                # Mix ned
                n  = np.arange(block_samples)
                c *= np.exp(-2j * math.pi * freq_offset_hz / sample_rate * n)
                # Lågpassfilter: behåll binnar inom ±channel_bw/2
                F  = np.fft.fft(c, n=fft_n)
                freq_res = sample_rate / fft_n
                n_bins = int(channel_bw_hz / 2 / freq_res)
                power = (np.sum(np.abs(F[:n_bins])**2) +
                         np.sum(np.abs(F[-n_bins:])**2)) / (2 * n_bins)
                db = 10 * math.log10(float(power) + 1e-30)
            else:
                # Fallback utan numpy: total RMS
                vals = struct.unpack(f'<{block_samples*2}h', raw)
                power = sum(v*v for v in vals) / len(vals) / (32768**2)
                db = 10 * math.log10(power + 1e-30)

            results.append((t, db))
            t += dt
            block_idx += 1

    return results


def find_bursts(energies, threshold_db=None, min_burst_ms=5, merge_gap_ms=50,
                block_ms=5):
    """Identifierar burst-interval baserat på energi över tröskelvärdet."""
    if threshold_db is None:
        vals = [e for _, e in energies]
        noise = sorted(vals)[len(vals) // 4]   # 25:e percentilen ≈ brustnivå
        threshold_db = noise + 6.0              # 6 dB över brus

    bursts = []
    in_burst = False
    burst_start = 0.0
    gap_blocks = int(merge_gap_ms / block_ms)
    below_count = 0

    for t, db in energies:
        if db >= threshold_db:
            if not in_burst:
                burst_start = t
                in_burst = True
            below_count = 0
        else:
            if in_burst:
                below_count += 1
                if below_count >= gap_blocks:
                    burst_end = t - below_count * (block_ms / 1000)
                    if (burst_end - burst_start) * 1000 >= min_burst_ms:
                        bursts.append((burst_start, burst_end))
                    in_burst = False
                    below_count = 0

    if in_burst:
        bursts.append((burst_start, energies[-1][0]))

    return bursts, threshold_db


def run(iq_path, freq_offset_hz, center_hz, sample_rate, channel_bw=100_000):
    print(f"Analyserar: {iq_path}")
    print(f"  Center: {center_hz/1e6:.5f} MHz  Rate: {sample_rate/1e6:.3f} MHz")
    print(f"  Kanaloffset: {freq_offset_hz:+.0f} Hz  "
          f"→ kanal vid {(center_hz+freq_offset_hz)/1e6:.5f} MHz")
    print(f"  Kanalbandbredd: {channel_bw/1e3:.0f} kHz\n")

    energies = bandpass_energy(iq_path, center_hz, sample_rate,
                               freq_offset_hz, channel_bw)

    if not energies:
        print("Inga data.")
        return

    vals = [e for _, e in energies]
    total_s = energies[-1][0]
    print(f"Inspelningslängd: {total_s:.1f} s  ({len(energies)} block à 5 ms)")
    print(f"Energi: min={min(vals):.1f} dB  max={max(vals):.1f} dB  "
          f"medel={sum(vals)/len(vals):.1f} dB\n")

    bursts, thresh = find_bursts(energies)
    print(f"Brusuppskattning: ~{thresh-6:.1f} dB  Tröskel: {thresh:.1f} dB")
    print(f"Antal burstar: {len(bursts)}\n")

    if bursts:
        print(f"{'Start (s)':>10}  {'Slut (s)':>10}  {'Längd (ms)':>12}")
        print("-" * 40)
        for s, e in bursts[:30]:
            print(f"  {s:8.3f}  {e:8.3f}  {(e-s)*1000:10.1f}")
        if len(bursts) > 30:
            print(f"  ... ({len(bursts)-30} fler)")

        # Enkel ASCII-tidslinje
        print("\nTidslinje (varje char = 0.1 s, '*' = burst, '.' = tyst):")
        width = min(100, int(total_s * 10))
        line = ['.'] * width
        for s, e in bursts:
            i0 = int(s * 10)
            i1 = min(width - 1, int(e * 10))
            for i in range(i0, i1 + 1):
                line[i] = '*'
        for row in range(0, width, 100):
            print('  ' + ''.join(line[row:row+100]))
    else:
        print("Inga burstar detekterade ovanför tröskeln.")
        print("Prova med lägre kanalbandbredd eller annan kanaloffset.")

    print(f"\nFörslag på vdes_replay-kommando (om burst hittas vid detta offset):")
    print(f"  --freq-offset {freq_offset_hz:.0f} \\")
    print(f"  --param squelch_db=3 \\")
    print(f"  --param symbol_rate_baud=76800  # byt till 38400 / 19200 vid 50/25 kHz kanal")


if __name__ == "__main__":
    path       = sys.argv[1] if len(sys.argv) > 1 else None
    offset_hz  = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    center_hz  = int(sys.argv[3]) if len(sys.argv) > 3 else 161_862_500
    sample_rate= int(sys.argv[4]) if len(sys.argv) > 4 else 2_048_000
    ch_bw      = int(sys.argv[5]) if len(sys.argv) > 5 else 100_000

    if path is None:
        print(__doc__)
        sys.exit(1)

    run(path, offset_hz, center_hz, sample_rate, ch_bw)
