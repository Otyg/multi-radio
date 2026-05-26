#!/usr/bin/env python3
"""
dsc_synth.py - Syntetisk VHF-DSC signalgenerator (steg 3).

Generatorn skapar en basbands-IQ-signal (.iq16) med 2-FSK symboler enligt
VHF-DSC parametrar:
  - 1200 baud
  - mark/space-toner 1300/2100 Hz
  - subcarrier 1700 Hz (=> +/-400 Hz i komplex basband)

Verktyget kan även skriva en WAV (audio-referens) och JSON-metadata.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import random
import struct
import wave
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


def bits_from_bytes(data: bytes) -> List[int]:
    out: List[int] = []
    for b in data:
        out.extend(((b >> (7 - i)) & 1) for i in range(8))
    return out


def bits_to_bytes(bits: Sequence[int]) -> bytes:
    if not bits:
        return b""
    out = bytearray((len(bits) + 7) // 8)
    for i, bit in enumerate(bits):
        if bit:
            out[i // 8] |= 1 << (7 - (i % 8))
    return bytes(out)


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_dsc_word(symbol: int) -> List[int]:
    if symbol < 0 or symbol > 127:
        raise ValueError(f"DSC symbol out of range: {symbol}")
    info = [((symbol >> (6 - i)) & 1) for i in range(7)]
    zero_count = sum(1 for b in info if b == 0)
    check = [((zero_count >> 2) & 1), ((zero_count >> 1) & 1), (zero_count & 1)]
    return info + check


def parse_symbol_list(raw: str) -> List[int]:
    out: List[int] = []
    for token in raw.split(","):
        token = token.strip()
        if not token:
            continue
        out.append(int(token, 10))
    if not out:
        raise ValueError("empty --dsc-symbols list")
    return out


def parse_bit_string(raw: str) -> List[int]:
    bits: List[int] = []
    for c in raw:
        if c == "0":
            bits.append(0)
        elif c == "1":
            bits.append(1)
        else:
            raise ValueError(f"invalid bit '{c}' in '{raw}'")
    if not bits:
        raise ValueError("bit string cannot be empty")
    return bits


@dataclass
class SynthConfig:
    sample_rate: int
    symbol_rate: float
    mark_hz: float
    space_hz: float
    subcarrier_hz: float
    amplitude: float
    snr_db: float
    carrier_offset_hz: float
    phase_jitter_deg: float
    ramp_ms: float
    bit1_is_mark: bool


def build_payload_bits(args: argparse.Namespace, rng: random.Random) -> Tuple[List[int], str]:
    if args.dsc_symbols:
        symbols = parse_symbol_list(args.dsc_symbols)
        payload: List[int] = []
        for sym in symbols:
            payload.extend(encode_dsc_word(sym))
        return payload, "dsc_symbols"

    if args.payload_hex:
        payload_bytes = bytes.fromhex(args.payload_hex)
    else:
        payload_bytes = bytes(rng.getrandbits(8) for _ in range(args.payload_bytes))
    return bits_from_bytes(payload_bytes), "bytes"


def build_frame_bits(args: argparse.Namespace, rng: random.Random) -> Tuple[List[int], Optional[int], str]:
    preamble_pat = parse_bit_string(args.preamble_pattern)
    sync_bits = parse_bit_string(args.sync_bits) if args.sync_bits else []

    bits: List[int] = []
    for i in range(args.preamble_bits):
        bits.append(preamble_pat[i % len(preamble_pat)])
    bits.extend(sync_bits)

    payload_bits, payload_mode = build_payload_bits(args, rng)
    bits.extend(payload_bits)

    crc: Optional[int] = None
    if args.append_crc16:
        crc = crc16_ccitt_false(bits_to_bytes(payload_bits))
        bits.extend(((crc >> (15 - i)) & 1) for i in range(16))

    return bits, crc, payload_mode


def _awgn_complex(rng: random.Random, sigma: float) -> complex:
    return complex(rng.gauss(0.0, sigma), rng.gauss(0.0, sigma))


def generate_iq_for_bits(bits: Sequence[int], cfg: SynthConfig, rng: random.Random) -> List[complex]:
    sps = cfg.sample_rate / cfg.symbol_rate
    if sps <= 0.0:
        raise ValueError("invalid samples/symbol")

    base_freq_mark = cfg.mark_hz - cfg.subcarrier_hz
    base_freq_space = cfg.space_hz - cfg.subcarrier_hz

    phase = 0.0
    acc = 0.0
    burst: List[complex] = []

    for bit in bits:
        is_mark = (bit == 1 and cfg.bit1_is_mark) or (bit == 0 and not cfg.bit1_is_mark)
        tone = base_freq_mark if is_mark else base_freq_space
        freq = tone + cfg.carrier_offset_hz

        acc += sps
        n_samp = int(acc)
        acc -= n_samp
        if n_samp <= 0:
            n_samp = 1

        for _ in range(n_samp):
            if cfg.phase_jitter_deg > 0.0:
                jitter_rad = math.radians(cfg.phase_jitter_deg) * rng.gauss(0.0, 1.0)
            else:
                jitter_rad = 0.0
            phase += 2.0 * math.pi * freq / cfg.sample_rate + jitter_rad
            if phase > math.pi:
                phase -= 2.0 * math.pi
            elif phase < -math.pi:
                phase += 2.0 * math.pi
            burst.append(complex(math.cos(phase), math.sin(phase)))

    if not burst:
        return []

    ramp_samples = int(cfg.ramp_ms * cfg.sample_rate / 1000.0)
    if ramp_samples > 0:
        n = len(burst)
        for i in range(min(ramp_samples, n)):
            w = 0.5 - 0.5 * math.cos(math.pi * (i + 1) / (ramp_samples + 1))
            burst[i] *= w
            burst[n - 1 - i] *= w

    for i in range(len(burst)):
        burst[i] *= cfg.amplitude

    if cfg.snr_db < 120.0:
        signal_power = (cfg.amplitude * cfg.amplitude) if cfg.amplitude > 0 else 1e-12
        noise_power = signal_power / (10.0 ** (cfg.snr_db / 10.0))
        sigma = math.sqrt(noise_power / 2.0)
        for i in range(len(burst)):
            burst[i] += _awgn_complex(rng, sigma)

    return burst


def write_iq16(path: str, samples: Sequence[complex]) -> None:
    with open(path, "wb") as f:
        for s in samples:
            i = int(max(-1.0, min(1.0, s.real)) * 32767.0)
            q = int(max(-1.0, min(1.0, s.imag)) * 32767.0)
            f.write(struct.pack("<hh", i, q))


def write_wav(path: str, bits: Sequence[int], args: argparse.Namespace) -> None:
    sps = args.sample_rate / args.symbol_rate
    acc = 0.0
    phase = 0.0
    pcm = bytearray()

    for bit in bits:
        if args.bit1_is_mark:
            tone_hz = args.mark_hz if bit == 1 else args.space_hz
        else:
            tone_hz = args.space_hz if bit == 1 else args.mark_hz

        acc += sps
        n_samp = int(acc)
        acc -= n_samp
        if n_samp <= 0:
            n_samp = 1

        for _ in range(n_samp):
            phase += 2.0 * math.pi * tone_hz / args.sample_rate
            if phase > math.pi:
                phase -= 2.0 * math.pi
            x = args.amplitude * math.sin(phase)
            v = int(max(-1.0, min(1.0, x)) * 32767.0)
            pcm.extend(struct.pack("<h", v))

    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(args.sample_rate)
        wf.writeframes(bytes(pcm))


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def ensure_parent(path: str) -> None:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("output_iq16", help="Utdatafil (.iq16)")
    ap.add_argument("--meta", default=None, help="Valfri metadata-json")
    ap.add_argument("--wav", default=None, help="Valfri audio-referens (.wav)")
    ap.add_argument("--sample-rate", type=int, default=48000)
    ap.add_argument("--symbol-rate", type=float, default=1200.0)
    ap.add_argument("--mark-hz", type=float, default=1300.0)
    ap.add_argument("--space-hz", type=float, default=2100.0)
    ap.add_argument("--subcarrier-hz", type=float, default=1700.0)
    ap.add_argument("--carrier-offset-hz", type=float, default=0.0)
    ap.add_argument("--snr-db", type=float, default=35.0)
    ap.add_argument("--amplitude", type=float, default=0.85)
    ap.add_argument("--phase-jitter-deg", type=float, default=0.0)
    ap.add_argument("--ramp-ms", type=float, default=1.5)
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--repeat", type=int, default=1, help="Antal burst-repetitioner")
    ap.add_argument("--gap-ms", type=float, default=200.0, help="Tystnad mellan repetitioner")
    ap.add_argument("--bit1-is-mark", action="store_true", default=True)
    ap.add_argument("--bit1-is-space", action="store_true", help="Överskriv mappning: bit 1 -> space")
    ap.add_argument("--preamble-bits", type=int, default=200)
    ap.add_argument("--preamble-pattern", default="10")
    ap.add_argument("--sync-bits", default="1110010110100011")
    ap.add_argument("--payload-hex", default=None)
    ap.add_argument("--payload-bytes", type=int, default=64)
    ap.add_argument("--dsc-symbols", default=None, help="CSV med symbolvärden 0..127 (kodas till 10-bit ord)")
    ap.add_argument("--append-crc16", action="store_true", default=True)
    ap.add_argument("--no-append-crc16", dest="append_crc16", action="store_false")
    args = ap.parse_args()

    if args.bit1_is_space:
        args.bit1_is_mark = False

    if args.sample_rate <= 0:
        raise SystemExit("sample-rate must be > 0")
    if args.symbol_rate <= 0.0:
        raise SystemExit("symbol-rate must be > 0")
    if args.repeat <= 0:
        raise SystemExit("repeat must be > 0")
    if args.payload_hex and args.dsc_symbols:
        raise SystemExit("use only one of --payload-hex or --dsc-symbols")

    rng = random.Random(args.seed)
    frame_bits, crc, payload_mode = build_frame_bits(args, rng)

    cfg = SynthConfig(
        sample_rate=args.sample_rate,
        symbol_rate=args.symbol_rate,
        mark_hz=args.mark_hz,
        space_hz=args.space_hz,
        subcarrier_hz=args.subcarrier_hz,
        amplitude=args.amplitude,
        snr_db=args.snr_db,
        carrier_offset_hz=args.carrier_offset_hz,
        phase_jitter_deg=args.phase_jitter_deg,
        ramp_ms=args.ramp_ms,
        bit1_is_mark=args.bit1_is_mark,
    )

    burst = generate_iq_for_bits(frame_bits, cfg, rng)
    gap_samples = int(args.sample_rate * args.gap_ms / 1000.0)
    silence = [0j] * max(0, gap_samples)

    all_samples: List[complex] = []
    for idx in range(args.repeat):
        if idx > 0 and silence:
            all_samples.extend(silence)
        all_samples.extend(burst)

    ensure_parent(args.output_iq16)
    write_iq16(args.output_iq16, all_samples)

    if args.wav:
        ensure_parent(args.wav)
        write_wav(args.wav, frame_bits, args)

    meta_path = args.meta if args.meta else args.output_iq16 + ".json"
    ensure_parent(meta_path)
    metadata = {
        "generator": "tools/dsc_synth.py",
        "seed": args.seed,
        "sample_rate_hz": args.sample_rate,
        "symbol_rate_baud": args.symbol_rate,
        "mark_hz": args.mark_hz,
        "space_hz": args.space_hz,
        "subcarrier_hz": args.subcarrier_hz,
        "carrier_offset_hz": args.carrier_offset_hz,
        "snr_db": args.snr_db,
        "amplitude": args.amplitude,
        "phase_jitter_deg": args.phase_jitter_deg,
        "repeat": args.repeat,
        "gap_ms": args.gap_ms,
        "bit1_is_mark": args.bit1_is_mark,
        "frame_bits": len(frame_bits),
        "payload_mode": payload_mode,
        "crc16_ccitt_false": f"0x{crc:04X}" if crc is not None else None,
        "files": {
            "iq16": args.output_iq16,
            "wav": args.wav,
        },
        "hashes": {
            "iq16_sha256": sha256_file(args.output_iq16),
        },
    }
    if args.wav:
        metadata["hashes"]["wav_sha256"] = sha256_file(args.wav)

    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2, sort_keys=True)
        f.write("\n")

    print(f"Generated IQ: {args.output_iq16}")
    print(f"Metadata:     {meta_path}")
    if args.wav:
        print(f"WAV:          {args.wav}")
    print(f"Bits/frame:   {len(frame_bits)}")
    print(f"Samples:      {len(all_samples)}")
    print("Suggested replay params for plugins/fsk_demod.c:")
    print("  MR_FSK_BAUD_RATE=1200")
    print("  MR_FSK_DEVIATION_HZ=400")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
