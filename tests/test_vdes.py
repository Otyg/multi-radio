#!/usr/bin/env python3
"""
tests/test_vdes.py — VDES loopback-tester

Verifierar hela avkodningskedjan:
  vdes_synth.py → .iq16 → vdes_replay (demod+decoder) → vdes_decode.py → CRC OK

Kör med:
  python3 tests/test_vdes.py            # alla tester
  python3 tests/test_vdes.py -v         # verbose (visa per-burst-utdata)
  python3 tests/test_vdes.py --quick    # bara SNR=20 dB, offset=0

  pytest tests/test_vdes.py             # med pytest
"""

import sys
import os
import subprocess
import tempfile
import json
import argparse

# ── Sökvägar ──────────────────────────────────────────────────────────────
ROOT     = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SYNTH    = os.path.join(ROOT, 'tools', 'vdes_synth.py')
DECODE   = os.path.join(ROOT, 'tools', 'vdes_decode.py')
REPLAY   = os.path.join(ROOT, 'build', 'tools', 'vdes_replay')

# Importera decode-logik direkt
sys.path.insert(0, os.path.join(ROOT, 'tools'))
from vdes_decode import decode_candidate, LINK_IDS

# ── Hjälpfunktioner ───────────────────────────────────────────────────────

# Hur många bitar ska decoder-fönstret extrahera? Måste rymma hela nyttolasten
# (LID 32b + data) men vara kortare än burstens totala bitstorlek så att
# nästa burst inte hoppas över i sökfönstret.
_LID_DATA_BITS = {11: 400, 17: 1840, 5: 400}

def _candidate_bits(lid: int) -> int:
    data = _LID_DATA_BITS.get(lid, 500)
    return 32 + data + 50   # LID + data + litet marginaltillägg

# Fixerade nyttolastmönster: namn → bytevärde som repeteras
# n_payload = n_data - 32 (CRC ingår i data_bits)
_N_PAYLOAD_BITS = {11: 368, 17: 1808}   # = data_bits - 32
_PAYLOAD_BITS = {
    'zeros':        '00000000',
    'ones':         '11111111',
    'rep01':        '01010101',
    'rep10':        '10101010',
    'rep00001111':  '00001111',
    'rep11110000':  '11110000',
    # 7-bit single-hot / single-cold patterns
    '1000000':      '1000000',
    '0100000':      '0100000',
    '0010000':      '0010000',
    '0001000':      '0001000',
    '0000100':      '0000100',
    '0000010':      '0000010',
    '0111111':      '0111111',
    '1011111':      '1011111',
    '1101111':      '1101111',
    '1110111':      '1110111',
    '1111011':      '1111011',
    '1111101':      '1111101',
    '1111110':      '1111110',
}
PAYLOAD_PATTERNS = list(_PAYLOAD_BITS.keys())

def _payload_hex(pattern: str, lid: int) -> str:
    n_bits = _N_PAYLOAD_BITS[lid]
    pat = _PAYLOAD_BITS[pattern]
    tiled = (pat * (n_bits // len(pat) + 1))[:n_bits]
    n_bytes = n_bits // 8  # n_bits is always a multiple of 8
    return ''.join(format(int(tiled[i*8:(i+1)*8], 2), '02x') for i in range(n_bytes))


# ── Loopback-funktion ─────────────────────────────────────────────────────

def run_loopback(lid: int, snr_db: float, carrier_offset_hz: int = 0,
                 sym_rate: int = 76800, n_bursts: int = 3,
                 payload_hex: str = None, seed: int = 42,
                 verbose: bool = False) -> dict:
    """
    Genererar syntetisk VDES-signal, kör pipeline och returnerar:
    {
      'candidates': int,      # antal VDES_ASM_CANDIDATE-meddelanden
      'crc_ok':     int,      # antal med CRC OK
      'crc_fail':   int,      # antal med CRC FAIL
      'link_ids':   list[int] # identifierade Link ID:n
      'errors':     list[str] # eventuella felmeddelanden
    }
    """
    result = {'candidates': 0, 'crc_ok': 0, 'crc_fail': 0,
              'link_ids': [], 'errors': []}

    if not os.path.isfile(REPLAY):
        result['errors'].append(f'vdes_replay saknas: {REPLAY}')
        return result

    with tempfile.NamedTemporaryFile(suffix='.iq16', delete=False) as f:
        iq_path = f.name

    try:
        # Generera syntetisk signal
        synth_cmd = [sys.executable, SYNTH, iq_path,
                     '--lid', str(lid),
                     '--snr-db', str(snr_db),
                     '--n-bursts', str(n_bursts),
                     '--offset', str(carrier_offset_hz),
                     '--sym-rate', str(sym_rate),
                     '--seed', str(seed)]
        if payload_hex is not None:
            synth_cmd += ['--payload', payload_hex]
        synth = subprocess.run(synth_cmd, cwd=ROOT, capture_output=True, text=True)
        if synth.returncode != 0:
            result['errors'].append(f'vdes_synth misslyckades: {synth.stderr[:200]}')
            return result

        # Kör demod + decoder
        replay = subprocess.run(
            [REPLAY,
             '--iq', iq_path,
             '--rate', '2048000',
             '--freq', '161862500',
             '--freq-offset', str(carrier_offset_hz),
             '--demod', 'vdes_burst_demod',
             '--decoder', 'vdes_burst_decoder',
             '--param', f'symbol_rate_baud={sym_rate}',
             '--param', f'candidate_bits={_candidate_bits(lid)}',
             '--param', 'squelch_db=3',
             '--jsonl'],
            cwd=ROOT, capture_output=True, text=True)

        # Tolka JSONL
        for line in replay.stdout.splitlines():
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            if obj.get('signal_type') != 'VDES_ASM_CANDIDATE':
                continue
            result['candidates'] += 1
            payload  = obj.get('payload', '')
            fields   = obj.get('fields', {})
            inverted = fields.get('sync_inverted', '0') == '1'

            # Kör avkodning (tyst — utan print)
            import io
            from contextlib import redirect_stdout
            buf = io.StringIO()
            with redirect_stdout(buf):
                r = decode_candidate(payload, label='', inverted=inverted)
            output = buf.getvalue()

            if 'CRC-32' in output:
                if '✓ OK' in output:
                    result['crc_ok'] += 1
                    # Extrahera Link ID
                    for line2 in output.splitlines():
                        if 'Link ID funnet:' in line2:
                            try:
                                lid_found = int(line2.split(':')[1].split()[0])
                                result['link_ids'].append(lid_found)
                            except (ValueError, IndexError):
                                pass
                    if verbose:
                        print(f"    ✓ CRC OK  (LID={result['link_ids'][-1] if result['link_ids'] else '?'})")
                else:
                    result['crc_fail'] += 1
                    if verbose:
                        print(f"    ✗ CRC FAIL")
            else:
                if verbose:
                    print(f"    - inget Link ID")

    finally:
        if os.path.exists(iq_path):
            os.unlink(iq_path)

    return result


# ── Testfall ──────────────────────────────────────────────────────────────

class VdesTestCase:
    def __init__(self, name: str, lid: int, snr_db: float,
                 carrier_offset_hz: int = 0, sym_rate: int = 76800,
                 n_bursts: int = 3, min_crc_ok: int = None,
                 expected_lid: int = None,
                 payload_hex: str = None, seed: int = 42):
        self.name               = name
        self.lid                = lid
        self.snr_db             = snr_db
        self.carrier_offset_hz  = carrier_offset_hz
        self.sym_rate           = sym_rate
        self.n_bursts           = n_bursts
        self.min_crc_ok         = n_bursts if min_crc_ok is None else min_crc_ok
        self.expected_lid       = expected_lid if expected_lid is not None else lid
        self.payload_hex        = payload_hex
        self.seed               = seed


# ── Bastestfall (expanderas med PAYLOAD_PATTERNS nedan) ──────────────────

_BASE_ALL = [
    # ── Grundläggande SNR-sweep, LID 11 ──────────────────────────────────
    VdesTestCase('snr_25dB_lid11',    lid=11, snr_db=25),
    VdesTestCase('snr_20dB_lid11',    lid=11, snr_db=20),
    VdesTestCase('snr_15dB_lid11',    lid=11, snr_db=15),
    VdesTestCase('snr_10dB_lid11',    lid=11, snr_db=10, min_crc_ok=2),
    VdesTestCase('snr_8dB_lid11',     lid=11, snr_db=8,  min_crc_ok=1),

    # ── Bärvågsavvikelse, LID 11 @ 20 dB SNR ────────────────────────────
    VdesTestCase('offset_+1kHz',      lid=11, snr_db=20, carrier_offset_hz= 1000),
    VdesTestCase('offset_-1kHz',      lid=11, snr_db=20, carrier_offset_hz=-1000),
    VdesTestCase('offset_+3kHz',      lid=11, snr_db=20, carrier_offset_hz= 3000),
    VdesTestCase('offset_-3kHz',      lid=11, snr_db=20, carrier_offset_hz=-3000),

    # ── LID 17 (större ram, ingen FEC) ───────────────────────────────────
    VdesTestCase('snr_20dB_lid17',    lid=17, snr_db=20, n_bursts=2),
    VdesTestCase('snr_15dB_lid17',    lid=17, snr_db=15, n_bursts=2, min_crc_ok=1),

    # ── Flera burstar ────────────────────────────────────────────────────
    VdesTestCase('multi_burst_5x',    lid=11, snr_db=20, n_bursts=5),
]

_BASE_QUICK = [
    VdesTestCase('quick_snr20',       lid=11, snr_db=20),
    VdesTestCase('quick_lid17',       lid=17, snr_db=20, n_bursts=2),
]


def _expand_payloads(base: list) -> list:
    """Expanderar varje bastestfall till sex varianter med fixerade nyttolaster."""
    out = []
    for tc in base:
        for pat in PAYLOAD_PATTERNS:
            out.append(VdesTestCase(
                name               = f'{tc.name}__{pat}',
                lid                = tc.lid,
                snr_db             = tc.snr_db,
                carrier_offset_hz  = tc.carrier_offset_hz,
                sym_rate           = tc.sym_rate,
                n_bursts           = tc.n_bursts,
                min_crc_ok         = tc.min_crc_ok,
                expected_lid       = tc.expected_lid,
                payload_hex        = _payload_hex(pat, tc.lid),
                seed               = 42,
            ))
    return out


ALL_TESTS   = _expand_payloads(_BASE_ALL)
QUICK_TESTS = _expand_payloads(_BASE_QUICK)


# ── Testmotor ─────────────────────────────────────────────────────────────

def run_tests(tests: list, verbose: bool = False) -> tuple:
    passed = failed = skipped = 0
    print(f"\n{'='*60}")
    print(f"  VDES LOOPBACK-TESTER  ({len(tests)} testfall)")
    print(f"{'='*60}\n")

    for tc in tests:
        tag = (f"LID{tc.lid} SNR={tc.snr_db}dB "
               f"off={tc.carrier_offset_hz:+}Hz n={tc.n_bursts}")
        print(f"  {tc.name:<30} {tag}")

        if verbose:
            print()

        r = run_loopback(
            lid=tc.lid, snr_db=tc.snr_db,
            carrier_offset_hz=tc.carrier_offset_hz,
            sym_rate=tc.sym_rate,
            n_bursts=tc.n_bursts,
            payload_hex=tc.payload_hex,
            seed=tc.seed,
            verbose=verbose)

        if r['errors']:
            print(f"    ⚠  FEL: {r['errors'][0]}")
            skipped += 1
            continue

        ok = r['crc_ok']
        total = r['candidates']
        wrong_lid = [x for x in r['link_ids'] if x != tc.expected_lid]

        status = '✓' if ok >= tc.min_crc_ok else '✗'
        lid_warn = f"  [LID-fel: {wrong_lid}]" if wrong_lid else ""
        print(f"    {status}  CRC OK {ok}/{total} kandidater "
              f"(kräver ≥{tc.min_crc_ok}){lid_warn}")

        if ok >= tc.min_crc_ok and not wrong_lid:
            passed += 1
        else:
            failed += 1
            if verbose and not ok:
                print(f"       Inga kandidater — kontrollera att "
                      f"build/plugins/vdes_burst_demod.so är uppdaterad")

    print(f"\n{'='*60}")
    print(f"  Resultat: {passed} OK  {failed} FAIL  {skipped} SKIP")
    print(f"{'='*60}\n")
    return passed, failed, skipped


# ── pytest-gränssnitt ─────────────────────────────────────────────────────

def _make_pytest_test(tc: VdesTestCase):
    """Dynamiskt genererar en pytest-testfunktion för ett VdesTestCase."""
    def test_fn():
        if not os.path.isfile(REPLAY):
            import pytest
            pytest.skip(f'vdes_replay saknas: {REPLAY}')
        r = run_loopback(lid=tc.lid, snr_db=tc.snr_db,
                         carrier_offset_hz=tc.carrier_offset_hz,
                         sym_rate=tc.sym_rate, n_bursts=tc.n_bursts,
                         payload_hex=tc.payload_hex, seed=tc.seed)
        assert not r['errors'], f"Pipeline-fel: {r['errors']}"
        assert r['crc_ok'] >= tc.min_crc_ok, (
            f"CRC OK {r['crc_ok']}/{r['candidates']}, krävde {tc.min_crc_ok}")
        wrong = [x for x in r['link_ids'] if x != tc.expected_lid]
        assert not wrong, f"Fel Link ID detekterat: {wrong}"
    test_fn.__name__ = f'test_{tc.name}'
    return test_fn


# Exponera pytest-testfunktioner i modulens namnrum
for _tc in ALL_TESTS:
    globals()[f'test_{_tc.name}'] = _make_pytest_test(_tc)


# ── Huvudprogram ──────────────────────────────────────────────────────────

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-v', '--verbose', action='store_true')
    ap.add_argument('--quick', action='store_true',
                    help='Kör bara ett fåtal snabba tester')
    args = ap.parse_args()

    tests = QUICK_TESTS if args.quick else ALL_TESTS
    passed, failed, skipped = run_tests(tests, verbose=args.verbose)
    sys.exit(0 if failed == 0 else 1)
