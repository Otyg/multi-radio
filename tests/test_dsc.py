#!/usr/bin/env python3
"""
tests/test_dsc.py - Basic integration tests for DSC demod + sequence parser.

Pipeline:
  tools/dsc_synth.py -> WAV -> IQ16 (I=audio,Q=0) ->
  build/tools/vdes_replay --demod dsc_afsk_demod --decoder dsc_decoder
"""

from __future__ import annotations

import json
import os
import struct
import subprocess
import sys
import tempfile
import wave
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SYNTH = ROOT / "tools" / "dsc_synth.py"
REPLAY = ROOT / "build" / "tools" / "vdes_replay"


def _run(cmd: list[str], cwd: Path = ROOT) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=str(cwd), text=True, capture_output=True)


def _wav_to_iq16(wav_path: Path, iq_path: Path) -> None:
    with wave.open(str(wav_path), "rb") as wf:
        assert wf.getsampwidth() == 2
        assert wf.getnchannels() == 1
        pcm = wf.readframes(wf.getnframes())
    samples = struct.unpack("<" + ("h" * (len(pcm) // 2)), pcm)
    with iq_path.open("wb") as f:
        for s in samples:
            f.write(struct.pack("<hh", s, 0))


def _collect_jsonl_lines(text: str) -> list[dict]:
    out: list[dict] = []
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return out


def _generate_and_replay() -> list[dict]:
    if not REPLAY.exists():
        raise RuntimeError(f"missing replay tool: {REPLAY}")

    with tempfile.TemporaryDirectory(prefix="dsc_test_") as td:
        tmp = Path(td)
        wav_path = tmp / "dsc.wav"
        iq_path = tmp / "dsc_audio.iq16"

        synth = _run(
            [
                sys.executable,
                str(SYNTH),
                str(tmp / "unused.iq16"),
                "--wav",
                str(wav_path),
                "--sample-rate",
                "48000",
                "--symbol-rate",
                "1200",
                "--snr-db",
                "35",
                "--preamble-bits",
                "120",
                "--repeat",
                "1",
                "--seed",
                "7",
                "--dsc-symbols",
                "112,120,0,23,20,0,10,108,23,54,48,0,0,117,117,117",
                "--no-append-crc16",
            ]
        )
        if synth.returncode != 0:
            raise RuntimeError(f"synth failed: {synth.stderr}")

        _wav_to_iq16(wav_path, iq_path)

        replay = _run(
            [
                str(REPLAY),
                "--iq",
                str(iq_path),
                "--rate",
                "48000",
                "--freq",
                "156525000",
                "--plugin-dir",
                "build/plugins",
                "--demod",
                "dsc_afsk_demod",
                "--decoder",
                "dsc_decoder",
                "--param",
                "dsc_min_words=3",
                "--param",
                "dsc_min_valid_ratio_pct=20",
                "--param",
                "squelch_db=-20",
                "--jsonl",
            ]
        )
        if replay.returncode != 0:
            raise RuntimeError(f"replay failed: {replay.stderr}\n{replay.stdout}")
        return _collect_jsonl_lines(replay.stdout)


def test_dsc_chain_emits_message() -> None:
    rows = _generate_and_replay()
    dsc_rows = [r for r in rows if r.get("signal_type") == "DSC"]
    assert dsc_rows, "expected at least one DSC decoded message"


def test_dsc_sequence_fields_present() -> None:
    rows = _generate_and_replay()
    dsc_rows = [r for r in rows if r.get("signal_type") == "DSC"]
    assert dsc_rows, "expected DSC decoded rows"

    fields = dsc_rows[0].get("fields", {})
    required = [
        "format_specifier",
        "format_label",
        "message_class",
        "parse_status",
        "symbols_csv",
        "words_total",
        "words_valid",
        "sequence_status",
        "eos_symbol",
        "eos_label",
        "telecommand1_label",
        "message_symbols",
        "metric_frames_parsed",
    ]
    for k in required:
        assert k in fields, f"missing field: {k}"

    words_total = int(fields["words_total"])
    words_valid = int(fields["words_valid"])
    assert words_total >= words_valid >= 0
    assert fields["sequence_status"] in ("ok", "partial")
    assert fields["parse_status"] in ("ok", "partial", "weak")
    assert fields["message_class"] in ("distress", "all_ships", "individual", "selective_other")


def test_dsc_metrics_monotonicity() -> None:
    rows = _generate_and_replay()
    dsc_rows = [r for r in rows if r.get("signal_type") == "DSC"]
    assert dsc_rows, "expected DSC decoded rows"
    fields = dsc_rows[0].get("fields", {})

    blocks = int(fields.get("metric_blocks", "0"))
    candidates = int(fields.get("metric_candidates", "0"))
    emitted = int(fields.get("metric_emitted", "0"))
    parsed = int(fields.get("metric_frames_parsed", "0"))
    ecc_ok = int(fields.get("metric_ecc_ok", "0"))
    ecc_fail = int(fields.get("metric_ecc_fail", "0"))

    assert blocks >= 1
    assert candidates >= 1
    assert emitted >= 1
    assert parsed >= 1
    assert (ecc_ok + ecc_fail) >= 1
