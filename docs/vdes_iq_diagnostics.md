# VDES IQ-fil diagnostik — steg för steg

Guide för att analysera IQ-inspelningar som kan innehålla VDES VDE-TER-trafik.
Alla verktyg finns i `tools/` och `build/tools/`.

## Förutsättningar

```bash
# Bygg verktygen om det inte är gjort
cmake --build build --target vdes_burst_demod vdes_burst_decoder vdes_replay -j$(nproc)
```

---

## Steg 1 — Verifiera att pipeline fungerar (gör detta en gång)

Innan du testar mot riktiga filer, bekräfta att allt fungerar mot en syntetisk signal:

```bash
python3 tools/vdes_synth.py /tmp/vdes_pipeline_test.iq16 \
  --n-bursts 3 --snr-db 20 --lid 11

build/tools/vdes_replay \
  --iq /tmp/vdes_pipeline_test.iq16 \
  --rate 2048000 --freq 161862500 --freq-offset -50000 \
  --demod vdes_burst_demod --decoder vdes_burst_decoder \
  --param candidate_bits=2000 --param squelch_db=3 \
  --jsonl 2>/dev/null | python3 tools/vdes_decode.py
```

**Förväntat resultat:** `CRC-32 ✓ OK` för varje burst. Om detta misslyckas — lös det innan du går vidare.

---

## Steg 2 — Visuell inspektion

```bash
# Översiktsbild, hela bandet
python3 tools/waterfall.py FIL.iq16 \
  --rate RATE --freq CENTER_HZ \
  --fft 2048 --no-dc --out ~/waterfall.png

# Zooma in på VDE-TER-bandet (161.7875–161.9375 MHz)
python3 tools/waterfall.py FIL.iq16 \
  --rate RATE --freq CENTER_HZ \
  --fft 4096 --no-dc \
  --freq-low -200000 --freq-high 200000 \
  --out ~/waterfall_zoom.png
```

**Vad du letar efter:** Horisontella ljusa streck = burstar. Breda ljusa band = kontinuerlig signal. Notera vid vilka frekvenser och tider du ser aktivitet.

---

## Steg 3 — Hitta aktiva kanaler

```bash
python3 tools/vdes_scan.py FIL.iq16 CENTER_HZ RATE
```

Verktyget rapporterar effekt per 25 kHz-kanal i VDE-TER-bandet och pekar ut vilken offset du bör titta på. Notera de starkaste kanalerna.

---

## Steg 4 — Bekräfta burstaktivitet per kanal

För varje kandidatoffset från steg 3:

```bash
# 100 kHz fönster (för 76800 baud kanal)
python3 tools/vdes_burst_viz.py FIL.iq16 OFFSET_HZ CENTER_HZ RATE 100000

# Om du misstänker 25 kHz kanal (19200 baud):
python3 tools/vdes_burst_viz.py FIL.iq16 OFFSET_HZ CENTER_HZ RATE 25000
```

**Tolka ASCII-tidslinjen:**
- Allt `.` → ingen signal alls på den kanalen, hoppa till nästa offset
- `*`-mönster → burstaktivitet, gå vidare
- Om `-75`, `-50`, `-25` kHz ger *identiska* mönster → signalen är minst 100 kHz bred (konsistent med VDES 100 kHz kanal)

---

## Steg 5 — Demodulera och diagnostisera

Kör med debug-output (utan decoder) för att se vad grinden faktiskt öppnas för:

```bash
MR_AIS_DEBUG=1 build/tools/vdes_replay \
  --iq FIL.iq16 --rate RATE --freq CENTER_HZ \
  --freq-offset OFFSET_HZ \
  --demod vdes_burst_demod \
  --param squelch_db=5 \
  2>&1 >/dev/null | grep -E "gate open|preamble|freq_err"
```

**Tolka utdatan:**

| `avg_phase`          | `freq_err`  | Slutsats                          |
|----------------------|-------------|-----------------------------------|
| nära −2.36 (= −3π/4) | < ±3 kHz    | Trolig VDES-burst                 |
| nära −2.36           | 3–10 kHz    | Möjlig VDES, lite frekvensskift   |
| allt annat           | valfri      | Annan signal (AIS, röst, etc.)    |

Om alla `freq_err` är > 10 kHz: prova nästa offset eller nästa symbolhastighet (steg 6).

Om inga grindöppningar alls: sänk `--param squelch_db=3` eller `--param squelch_db=1`.

---

## Steg 6 — Prova symbolhastigheter och offset-kombinationer

```bash
for offset in -75000 -50000 -25000 0 25000 50000 75000; do
  for sym_rate in 76800 38400 19200; do
    echo "=== offset=$offset sym_rate=$sym_rate ==="
    build/tools/vdes_replay \
      --iq FIL.iq16 --rate RATE --freq CENTER_HZ \
      --freq-offset $offset \
      --demod vdes_burst_demod \
      --param symbol_rate_baud=$sym_rate \
      --param squelch_db=5 \
      --jsonl 2>/dev/null | python3 tools/vdes_decode.py 2>&1 \
      | grep -E "Hamming|freq_err=[-0-9.]+ Hz bits=[0-9]+|kandidater"
  done
done
```

**Vad du letar efter:**
- `Hamming=0/32` till `Hamming=3/32` → Link ID hittad, gå till steg 7
- `freq_err` < ±3 kHz OCH `Hamming` ≤ 5/32 → trolig VDES men lite brus, öka SNR
- Alla `Hamming` ≥ 6/32 oavsett kombination → inspelningen innehåller antagligen ingen VDES-trafik

---

## Steg 7 — Fullavkodning

När du hittat rätt offset och symbolhastighet:

```bash
build/tools/vdes_replay \
  --iq FIL.iq16 --rate RATE --freq CENTER_HZ \
  --freq-offset OFFSET_HZ \
  --demod vdes_burst_demod --decoder vdes_burst_decoder \
  --param symbol_rate_baud=SYM_RATE \
  --param candidate_bits=2000 \
  --param squelch_db=5 \
  --jsonl 2>/dev/null | python3 tools/vdes_decode.py
```

**Förväntat utfall för lyckad avkodning:**
```
Link ID funnet: 11  (offset=0, fel=0, ota)
CRC-32: ✓ OK
Avkodad payload (46 bytes): ...
```

---

## Snabbreferens — feldiagnostik

| Symptom                          | Trolig orsak           | Åtgärd                          |
|----------------------------------|------------------------|---------------------------------|
| Inga grindöppningar              | Squelch för högt       | Sänk `squelch_db`               |
| `avg_phase` aldrig nära −3π/4   | Fel frekvensoffset     | Prova alla offset i steg 6      |
| `freq_err` alltid > 10 kHz       | Fel symbolhastighet    | Prova 38400 / 19200 baud        |
| `Hamming` konsekvent 6–10/32     | Inte VDES (AIS, röst)  | Ny inspelning behövs            |
| CRC misslyckas trots lågt Hamming| Hög brus / svag signal | Inspelning med bättre SNR       |
