#!/usr/bin/env python3
"""
vdes_scan.py — Scannar en katalog med IQ-inspelningar efter VDES-meddelanden.

Filnamnsformat: raw_<timestamp>_<center-freq>_<sample-rate>_<n>.iq16

Söker som standard igenom hela filens bandbredd med 25 kHz kanalsteg och
symbolhastigheterna 76800, 38400 och 19200 baud (25/50/100 kHz-kanaler).
FFT-prescan begränsar automatiskt vilka kanalpositioner som provas.

Användning:
  python3 tools/vdes_scan.py <katalog> [alternativ]

Alternativ:
  --sym-rates RATE[,RATE...]   Symbolhastigheter att prova, kommaseparerade
                               (standard: 76800,38400,19200)
  --offsets OFF[,OFF...]       Fasta offset-värden i Hz (ersätter kanal-sweep)
  --ch-step HZ                 Kanalsteg för offset-sweep (standard: 25000)
  --band-lo HZ                 Undre bandgräns Hz; standard: hela filens bandbredd
  --band-hi HZ                 Övre bandgräns Hz; standard: hela filens bandbredd
  --squelch DB                 Squelch-tröskel i dB (standard: 3)
  --no-squelch                 Stäng av squelch (sätter tröskeln till -100 dB)
  --no-prescan                 Hoppa över FFT-förhandsscan (försök alla kanaler)
  --prescan-margin DB          Dynamisk marginal under toppsignal vid prescan
                               (standard: 30 dB); kanaler under tröskeln hoppas
  -v, --verbose                Visa varje sondering
"""

import sys
import os
import re
import argparse
import json
import subprocess
import io
from contextlib import redirect_stdout

ROOT   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPLAY = os.path.join(ROOT, 'build', 'tools', 'vdes_replay')

sys.path.insert(0, os.path.join(ROOT, 'tools'))
from vdes_decode import decode_candidate

# Filnamnsformat: raw_<timestamp>_<centerfreq>_<samplerate>_<n>.iq16
# Timestamp kan vara Unix-heltal (1716624000) eller ISO 8601 (20260521T054751Z)
_IQ_RE = re.compile(r'^raw_([^_]+)_(\d+)_(\d+)_(\d+)\.iq16$')

CH_STEP_DEFAULT = 25_000
SYM_RATES_DEFAULT = '76800,38400,19200'

# candidate_bits: generöst fönster som täcker LID11 (≈482) och LID17 (≈1922)
CANDIDATE_BITS = 1922


def parse_filename(name):
    """Returnerar (timestamp_str, center_hz, sample_rate, n) eller None."""
    m = _IQ_RE.match(name)
    if not m:
        return None
    return m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4))


def channel_offsets(center_hz, sample_rate, band_lo, band_hi, ch_step):
    """
    Alla kanaloffset (Hz) att prova.

    Om band_lo/band_hi är None sweepas hela filens bandbredd (center ± rate/2)
    med ett ch_step-marginellt avstånd från kanterna.
    Om band_lo/band_hi är satta begränsas sökningen till det bandet.
    """
    half_bw = sample_rate / 2 - ch_step  # lite marginal mot kanten

    if band_lo is None or band_hi is None:
        # Hela bandbredden: jämna steg centrade kring 0
        start = -(int(half_bw) // ch_step) * ch_step
        stop  =  (int(half_bw) // ch_step) * ch_step
        return list(range(start, stop + 1, ch_step))

    offsets = []
    f = band_lo
    while f <= band_hi:
        offset = f - center_hz
        if abs(offset) <= half_bw:
            offsets.append(int(offset))
        f += ch_step
    return offsets


def power_prescan(iq_path, center_hz, sample_rate, offsets,
                  margin_db=30, block=65536, ch_step=CH_STEP_DEFAULT):
    """
    Snabb FFT-effektscan. Returnerar (active_offsets, powers_dict).
    active_offsets innehåller de kanaler som är inom margin_db under toppeffekten.
    Kräver numpy; utan numpy returneras alla offsets ofiltrerade.
    """
    try:
        import numpy as np
    except ImportError:
        return offsets, {}

    power = np.zeros(block, dtype=np.float64)
    n_blocks = 0
    try:
        with open(iq_path, 'rb') as fh:
            while True:
                raw = fh.read(block * 4)
                if len(raw) < block * 4:
                    break
                iq = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
                c  = (iq[0::2] + 1j * iq[1::2]) / 32768.0
                c *= np.hamming(block)
                power += np.abs(np.fft.fft(c)) ** 2
                n_blocks += 1
    except OSError:
        return offsets, {}

    if n_blocks == 0:
        return offsets, {}

    power /= n_blocks
    freq_axis = center_hz + np.fft.fftfreq(block, d=1.0 / sample_rate)
    half_ch   = ch_step // 2

    powers = {}
    for offset in offsets:
        cf   = center_hz + offset
        mask = np.abs(freq_axis - cf) < half_ch
        p    = float(np.mean(power[mask])) if np.any(mask) else 0.0
        powers[offset] = 10.0 * np.log10(p) if p > 0 else -99.0

    if not powers:
        return offsets, powers

    max_p     = max(powers.values())
    threshold = max_p - margin_db
    active    = [o for o in offsets if powers.get(o, -99.0) >= threshold]
    return active, powers


def run_replay(iq_path, center_hz, sample_rate, freq_offset, sym_rate,
               squelch_db, verbose):
    """
    Kör vdes_replay för en given fil och kanaloffset.
    Returnerar lista med dicts: {crc_ok, crc_fail, decode_output, raw_payload}.
    """
    cmd = [
        REPLAY,
        '--iq',          iq_path,
        '--rate',        str(sample_rate),
        '--freq',        str(center_hz),
        '--freq-offset', str(freq_offset),
        '--demod',       'vdes_burst_demod',
        '--decoder',     'vdes_burst_decoder',
        '--param',       f'symbol_rate_baud={sym_rate}',
        '--param',       f'candidate_bits={CANDIDATE_BITS}',
        '--param',       f'squelch_db={squelch_db}',
        '--jsonl',
    ]

    try:
        proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True,
                              timeout=120)
    except subprocess.TimeoutExpired:
        if verbose:
            print('    [timeout]')
        return []

    hits = []
    for line in proc.stdout.splitlines():
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        if obj.get('signal_type') != 'VDES_ASM_CANDIDATE':
            continue

        payload  = obj.get('payload', '')
        fields   = obj.get('fields', {})
        inverted = fields.get('sync_inverted', '0') == '1'

        buf = io.StringIO()
        with redirect_stdout(buf):
            decode_candidate(payload, label='', inverted=inverted)
        output = buf.getvalue().strip()

        hits.append({
            'crc_ok':       '✓ OK' in output,
            'crc_fail':     'CRC' in output and '✓ OK' not in output,
            'decode_output': output,
            'raw_payload':  payload,
        })
    return hits


def scan_file(iq_path, center_hz, sample_rate, sym_rates, offsets,
              squelch_db, prescan, prescan_margin, ch_step, verbose):
    """
    Scannar en fil. Returnerar lista med fynd-dicts.
    """
    fname    = os.path.basename(iq_path)
    findings = []

    if prescan and offsets:
        active, powers = power_prescan(iq_path, center_hz, sample_rate,
                                        offsets, margin_db=prescan_margin,
                                        ch_step=ch_step)
        if verbose and powers:
            print(f'  Prescan: {len(active)}/{len(offsets)} aktiva kanaler '
                  f'(tröskelmarginal {prescan_margin} dB)')
    else:
        active = offsets

    if not active:
        active = offsets  # fallback: prova alla om prescan ger noll

    for offset in active:
        for sym_rate in sym_rates:
            chan_mhz = (center_hz + offset) / 1e6
            if verbose:
                print(f'  {chan_mhz:.4f} MHz  ({offset:+d} Hz)  '
                      f'{sym_rate} baud ... ', end='', flush=True)

            hits = run_replay(iq_path, center_hz, sample_rate, offset,
                              sym_rate, squelch_db, verbose)

            if verbose:
                ok   = sum(1 for h in hits if h['crc_ok'])
                fail = sum(1 for h in hits if h['crc_fail'])
                print(f'{len(hits)} kandidat(er)'
                      + (f'  ✓ {ok} CRC OK' if ok else '')
                      + (f'  ✗ {fail} CRC fel' if fail else ''))

            for h in hits:
                findings.append({
                    'file':         fname,
                    'offset_hz':    offset,
                    'chan_mhz':     chan_mhz,
                    'sym_rate':     sym_rate,
                    'crc_ok':       h['crc_ok'],
                    'crc_fail':     h['crc_fail'],
                    'decode_output': h['decode_output'],
                })

    return findings


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('directory', help='Katalog med IQ-filer')
    ap.add_argument('--sym-rates', default=SYM_RATES_DEFAULT,
                    help=f'Symbolhastigheter, kommaseparerade (standard: {SYM_RATES_DEFAULT})')
    ap.add_argument('--offsets',
                    help='Fasta offset-värden i Hz, kommaseparerade')
    ap.add_argument('--ch-step', type=int, default=CH_STEP_DEFAULT,
                    help=f'Kanalsteg Hz (standard: {CH_STEP_DEFAULT})')
    ap.add_argument('--band-lo', type=int, default=None,
                    help='Undre bandgräns Hz (standard: hela filens bandbredd)')
    ap.add_argument('--band-hi', type=int, default=None,
                    help='Övre bandgräns Hz (standard: hela filens bandbredd)')
    ap.add_argument('--squelch', type=float, default=3.0,
                    help='Squelch dB (standard: 3)')
    ap.add_argument('--no-squelch', action='store_true',
                    help='Stäng av squelch (sätter tröskeln till -100 dB)')
    ap.add_argument('--no-prescan', action='store_true',
                    help='Hoppa över FFT-förhandsscanning')
    ap.add_argument('--prescan-margin', type=float, default=30.0,
                    help='Marginal under toppsignal vid prescan i dB (standard: 30)')
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()

    sym_rates   = [int(r.strip()) for r in args.sym_rates.split(',')]
    squelch_db  = -100.0 if args.no_squelch else args.squelch

    if not os.path.isfile(REPLAY):
        print(f'Fel: vdes_replay saknas: {REPLAY}', file=sys.stderr)
        sys.exit(1)

    directory = os.path.abspath(args.directory)
    if not os.path.isdir(directory):
        print(f'Fel: katalogen finns inte: {directory}', file=sys.stderr)
        sys.exit(1)

    # Hitta alla matchande IQ-filer
    iq_files = []
    for name in sorted(os.listdir(directory)):
        parsed = parse_filename(name)
        if parsed is None:
            continue
        ts, center_hz, sample_rate, n = parsed
        iq_files.append((os.path.join(directory, name), center_hz, sample_rate))

    if not iq_files:
        print(f'Inga IQ-filer med rätt namnformat hittades i {directory}')
        print('Förväntat format: raw_<timestamp>_<center-hz>_<sample-rate>_<n>.iq16')
        sys.exit(1)

    print(f'\nVDES-scanner  {len(iq_files)} fil(er)  sym_rates={sym_rates}')
    print('=' * 60)

    all_findings = []

    for iq_path, center_hz, sample_rate in iq_files:
        if args.offsets:
            offsets = [int(o.strip()) for o in args.offsets.split(',')]
        else:
            offsets = channel_offsets(center_hz, sample_rate,
                                       args.band_lo, args.band_hi, args.ch_step)

        print(f'\n{os.path.basename(iq_path)}')
        print(f'  center={center_hz/1e6:.4f} MHz  '
              f'rate={sample_rate/1e6:.3f} MHz  '
              f'{len(offsets)} kanalposition(er)  '
              f'{len(offsets)*len(sym_rates)} sondering(ar)')

        findings = scan_file(
            iq_path, center_hz, sample_rate,
            sym_rates, offsets,
            squelch_db=squelch_db,
            prescan=not args.no_prescan,
            prescan_margin=args.prescan_margin,
            ch_step=args.ch_step,
            verbose=args.verbose,
        )
        all_findings.extend(findings)

        ok_hits   = [f for f in findings if f['crc_ok']]
        fail_hits = [f for f in findings if f['crc_fail'] and not f['crc_ok']]

        if ok_hits:
            print(f'  ✓ {len(ok_hits)} VDES-meddelande(n) med CRC OK:')
            for h in ok_hits:
                print(f'    {h["chan_mhz"]:.4f} MHz  ({h["offset_hz"]:+d} Hz)  '
                      f'{h["sym_rate"]} baud')
                for line in h['decode_output'].splitlines():
                    print(f'      {line}')
        if fail_hits:
            print(f'  ? {len(fail_hits)} kandidat(er) med CRC-fel '
                  f'(möjlig signal men avkodning misslyckades)')
        if not ok_hits and not fail_hits:
            print('  — inget hittat')

    print(f'\n{"="*60}')
    total_ok   = sum(1 for f in all_findings if f['crc_ok'])
    total_fail = sum(1 for f in all_findings if f['crc_fail'] and not f['crc_ok'])
    print(f'  Totalt: {total_ok} CRC OK  {total_fail} CRC-fel  '
          f'i {len(iq_files)} fil(er)')
    print(f'{"="*60}\n')

    sys.exit(0 if total_ok > 0 or total_fail == 0 else 1)


if __name__ == '__main__':
    main()
