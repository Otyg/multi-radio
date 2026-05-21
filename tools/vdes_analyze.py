#!/usr/bin/env python3
"""
vdes_analyze.py — Snabb VDES-kanalanalys av IQ-inspelningar

Analyserar kanalerna 24/84/25/85/26/86 i en IQ-fil centrerad på
157.2625 eller 161.8625 MHz och avgör om det finns VDES-trafik.

Användning:
  python3 vdes_analyze.py <fil1.iq16> [fil2.iq16 ...]
  python3 vdes_analyze.py rec_157mhz.iq16 rec_161mhz.iq16

Utdata per kanal:
  - SNR och burstdetektion
  - Bandbredd vid -10 dB (→ modulationstyp)
  - Duty cycle och burstlängder
  - Slutsats: VDES pi/4-DQPSK / AIS GMSK / Ingen signal
"""
import sys, os, struct
import numpy as np
from collections import defaultdict

# ── Konstanter ────────────────────────────────────────────────────────
SR          = 2_048_000      # sample rate default (overridden per file)
BLOCK       = 4096           # IQ-par per analysblock (~2 ms)
CHANNEL_HZ  = [-62500, -37500, -12500, +12500, +37500, +62500]
CHANNEL_NAMES = ["CH24","CH84","CH25","CH85","CH26","CH86"]

BW_GMSK_LOW  =  7_000        # GMSK 9600 bps: typisk -10 dB BW 8–16 kHz
BW_GMSK_HIGH = 22_000
BW_VDES_LOW  = 25_000        # pi/4-DQPSK 28800 bps: typisk -10 dB BW 28–45 kHz
BW_VDES_HIGH = 55_000

NOISE_PERCENTILE = 15        # andel "tysta" block för brusgolvsberäkning
BURST_THRESHOLD  = 6.0       # dB över brusgolv för burstdetektion

# ── Hjälpfunktioner ───────────────────────────────────────────────────

def load_blocks(path):
    """Generator: returnerar IQ-block som komplexa float32-arrayer."""
    with open(path, "rb") as f:
        while True:
            raw = f.read(BLOCK * 4)
            if len(raw) < BLOCK * 4:
                break
            d = np.frombuffer(raw, dtype=np.int16)
            iq = d[0::2].astype(np.float32) + 1j * d[1::2].astype(np.float32)
            yield iq / 32768.0

def channel_power(iq, offset_hz, sr, bw_hz=20_000):
    """Effekt i ett smalt band runt offset_hz (enkel FFT-filterimitation)."""
    n = len(iq)
    spec = np.abs(np.fft.fft(iq)) ** 2
    freqs = np.fft.fftfreq(n, 1.0 / sr)
    mask = np.abs(freqs - offset_hz) < bw_hz / 2
    return float(spec[mask].mean()) if mask.any() else 0.0

def measure_bandwidth(iq, offset_hz, sr, fft_n=8192):
    """
    Mäter -10 dB-bandbredd centrerad runt offset_hz.
    Returnerar (peak_offset_hz, bw_hz) eller (None, None) om ingen signal.
    """
    spec  = np.abs(np.fft.fftshift(np.fft.fft(iq, n=fft_n))) ** 2
    freqs = np.fft.fftshift(np.fft.fftfreq(fft_n, 1.0 / sr))

    # Begränsa till ±80 kHz runt offset
    roi   = np.abs(freqs - offset_hz) < 80_000
    if not roi.any():
        return None, None

    local_spec  = spec.copy()
    local_spec[~roi] = 0
    peak_idx    = int(np.argmax(local_spec))
    peak_pow    = local_spec[peak_idx]
    noise_floor = np.median(spec)
    if peak_pow < noise_floor * 4:   # < 6 dB SNR → ingen signal
        return None, None

    peak_off = float(freqs[peak_idx])
    threshold = peak_pow / 10.0      # -10 dB

    above = local_spec > threshold
    if not above.any():
        return peak_off, 0.0

    bw = float(freqs[above][-1] - freqs[above][0])
    return peak_off, bw

def modulation_guess(bw_hz):
    if bw_hz is None:
        return "—"
    if BW_GMSK_LOW <= bw_hz <= BW_GMSK_HIGH:
        return "GMSK 9600 bps (AIS-liknande)"
    if BW_VDES_LOW <= bw_hz <= BW_VDES_HIGH:
        return "pi/4-DQPSK 28800 bps (VDES!)"
    if bw_hz < BW_GMSK_LOW:
        return f"Smalband {bw_hz/1000:.1f} kHz (CW/FSK?)"
    return f"Bred {bw_hz/1000:.1f} kHz (okänd)"

# ── Huvud-analys ──────────────────────────────────────────────────────

def analyze_file(path, center_mhz, sr):
    print(f"\n{'='*70}")
    print(f"Fil:    {os.path.basename(path)}")
    print(f"Center: {center_mhz:.4f} MHz   SR: {sr/1e6:.3f} Msps")
    print(f"{'='*70}")

    # Pass 1: Beräkna burstförekomst och brusgolv per kanal.
    # Vi sparar effekten i en smal bandbredd runt varje kanal för varje block.
    powers  = defaultdict(list)   # ch_idx -> [power per block]
    n_blocks = 0

    for iq in load_blocks(path):
        for ci, off in enumerate(CHANNEL_HZ):
            powers[ci].append(channel_power(iq, off, sr))
        n_blocks += 1

    if n_blocks == 0:
        print("Tom fil.")
        return

    duration_s = n_blocks * BLOCK / sr
    print(f"Längd:  {duration_s:.1f} s  ({n_blocks} block à {BLOCK} IQ-par)")

    # Pass 2: Per-kanal-statistik + burstdetektion.
    burst_blocks = {}   # ch_idx -> block-index med burst
    for ci in range(len(CHANNEL_HZ)):
        p = np.array(powers[ci])
        noise = np.percentile(p, NOISE_PERCENTILE)
        thresh = noise * (10 ** (BURST_THRESHOLD / 10))
        bursts = np.where(p > thresh)[0]
        burst_blocks[ci] = bursts

    # Hitta det block med mest aktivitet för bandbredds-mätning.
    # Välj burst-block med högst effekt-summa över alla kanaler.
    all_active = set()
    for ci in range(len(CHANNEL_HZ)):
        all_active.update(burst_blocks[ci].tolist())

    best_block_iq = None
    if all_active:
        # Ladda om för att hämta ett representativt burst-block.
        pick_block = sorted(all_active)[len(all_active) // 2]
        with open(path, "rb") as f:
            f.seek(pick_block * BLOCK * 4)
            raw = f.read(BLOCK * 64 * 4)   # 64 block för stabilare FFT
        if len(raw) >= BLOCK * 4:
            d = np.frombuffer(raw[:BLOCK*64*4], dtype=np.int16)
            iq = d[0::2].astype(np.float32) + 1j * d[1::2].astype(np.float32)
            best_block_iq = iq / 32768.0

    # Pass 3: Rapportera per kanal.
    print(f"\n{'Kanal':<6} {'Offset':>8} {'Burstar':>8} {'Duty%':>6} "
          f"{'BW -10dB':>10}  Slutsats")
    print("-" * 70)

    any_vdes = False
    for ci, (off, name) in enumerate(zip(CHANNEL_HZ, CHANNEL_NAMES)):
        p = np.array(powers[ci])
        noise = np.percentile(p, NOISE_PERCENTILE)
        thresh = noise * (10 ** (BURST_THRESHOLD / 10))
        bursts = burst_blocks[ci]
        n_bursts = len(bursts)
        duty = 100.0 * n_bursts / n_blocks

        bw, mod = None, "Ingen signal"
        if n_bursts > 0 and best_block_iq is not None:
            peak_off, bw_hz = measure_bandwidth(best_block_iq, off, sr)
            if bw_hz is not None and bw_hz > 1000:
                bw = bw_hz
                mod = modulation_guess(bw_hz)
                if "VDES" in mod:
                    any_vdes = True

        bw_str = f"{bw/1000:.1f} kHz" if bw else "—"
        rf_mhz = center_mhz + off / 1e6
        print(f"{name:<6} {off/1000:>+7.1f}kHz ({rf_mhz:.4f} MHz)"
              f"  {n_bursts:>5}  {duty:>5.1f}%  {bw_str:>8}  {mod}")

    # Burstlängder (sammanhängande aktiva block) på den aktiva kanalen.
    most_active_ci = max(range(len(CHANNEL_HZ)),
                         key=lambda i: len(burst_blocks[i]))
    if burst_blocks[most_active_ci].size > 0:
        print(f"\nBurststruktur (aktiv kanal {CHANNEL_NAMES[most_active_ci]}):")
        runs, run_ms = [], []
        ci_bursts = sorted(burst_blocks[most_active_ci])
        if ci_bursts:
            start = ci_bursts[0]; prev = ci_bursts[0]
            for b in ci_bursts[1:]:
                if b == prev + 1:
                    prev = b
                else:
                    runs.append(prev - start + 1)
                    start = b; prev = b
            runs.append(prev - start + 1)
            run_ms = [r * BLOCK / sr * 1000 for r in runs]
            run_ms_arr = np.array(run_ms)
            print(f"  Antal burstar: {len(runs)}")
            print(f"  Längd: min={run_ms_arr.min():.1f} ms  "
                  f"median={np.median(run_ms_arr):.1f} ms  "
                  f"max={run_ms_arr.max():.1f} ms")
            if len(runs) > 1:
                gaps = np.diff(sorted(burst_blocks[most_active_ci]))
                gap_ms = gaps[gaps > 1] * BLOCK / SR * 1000
                if len(gap_ms):
                    print(f"  Gap (mellanrum): median={np.median(gap_ms):.0f} ms")

    print(f"\n>>> {'VDES-trafik troligen FUNNEN' if any_vdes else 'Ingen VDES-trafik detekterad'} <<<")

# ── Jämförelse av två filer (fartygs-TX vs kust-TX) ──────────────────

def compare_files(paths):
    print("\n" + "="*70)
    print("JÄMFÖRELSE: duplex-länk")
    print("  157.2625 MHz = fartygs-TX   161.8625 MHz = kust-TX")
    print("="*70)
    print("Aktivitet på kust-TX-sidan indikerar att kustsändaren sänder VDES.")
    print("Aktivitet på fartygs-TX-sidan indikerar fartyg i närheten.")

# ── Entry point ───────────────────────────────────────────────────────

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    files = sys.argv[1:]
    for path in files:
        if not os.path.exists(path):
            print(f"Hittar inte filen: {path}")
            continue

        # Gissa centreringsfrekvens och samplingsfrekvens från filnamn.
        # Format: prefix_<center_hz>_<sample_rate_hz>_<index>.iq16
        center_mhz = None
        sample_rate = None
        parts = os.path.basename(path).split("_")
        numeric = []
        for part in parts:
            try:
                numeric.append(int(part.split(".")[0]))
            except ValueError:
                pass

        for v in numeric:
            if 150_000_000 <= v <= 170_000_000 and center_mhz is None:
                center_mhz = v / 1e6
            elif 100_000 <= v <= 20_000_000 and sample_rate is None:
                sample_rate = v

        if center_mhz is None:
            ans = input(f"Centreringsfrekvens för {os.path.basename(path)} (MHz): ").strip()
            try:
                center_mhz = float(ans)
            except ValueError:
                print("Ogiltig frekvens, hoppar över.")
                continue

        if sample_rate is None:
            ans = input(f"Samplingsfrekvens för {os.path.basename(path)} (Hz): ").strip()
            try:
                sample_rate = int(ans)
            except ValueError:
                print("Ogiltig samplingsfrekvens, hoppar över.")
                continue

        analyze_file(path, center_mhz, sample_rate)

    if len(files) == 2:
        compare_files(files)
