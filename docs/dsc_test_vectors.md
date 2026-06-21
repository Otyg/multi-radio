# VHF-DSC Steg 2 - Referensdata och testvektorer

Datum: 2026-05-26  
Status: Initial plan + syntetisk generator tillagd

## Mål

- Definiera reproducerbara testvektorer för demodulator/decoder.
- Knyta varje vektor till ett eller flera krav-ID i kravmatrisen.
- Tydlig separering mellan syntetiska och inspelade verkliga signaler.

## Rekommenderad struktur

```text
testdata/
  dsc/
    manifests/
      dsc_vectors_v1.json
    synthetic/
      (genereras lokalt med tools/dsc_synth.py)
    captured/
      (verkliga inspelningar, ej i git om stora filer)
```

## Vektorprofiler (baseline)

| Profil-ID | Beskrivning | Parametrar |
|---|---|---|
| DSC-VEC-001 | Referens, ren kanal | SNR=35 dB, offset=0 Hz, 1200 baud |
| DSC-VEC-002 | Normal drift | SNR=20 dB, offset=0 Hz |
| DSC-VEC-003 | Låg SNR | SNR=10 dB, offset=0 Hz |
| DSC-VEC-004 | Frekvensfel positiv | SNR=20 dB, offset=+120 Hz |
| DSC-VEC-005 | Frekvensfel negativ | SNR=20 dB, offset=-120 Hz |
| DSC-VEC-006 | Timingstress | sample_rate ej jämnt delbar med 1200 baud |
| DSC-VEC-007 | Burst-serie | 3 repetitioner med gap 200 ms |
| DSC-VEC-008 | Payload all-zero | kantfall för run-length |
| DSC-VEC-009 | Payload all-one | kantfall för run-length |
| DSC-VEC-010 | Pseudorandom payload | seedad PRNG för reproducerbarhet |

## Manifestfält (minimum)

- `vector_id`
- `generator`: version + commit
- `sample_rate_hz`
- `symbol_rate_baud`
- `mark_hz`
- `space_hz`
- `subcarrier_hz`
- `carrier_offset_hz`
- `snr_db`
- `seed`
- `payload_mode`
- `payload_bits`
- `repeat_count`
- `gap_ms`
- `files`: iq16/wav/json metadata
- `requirements`: lista med krav-ID (ex `["DSC-RQ-001","DSC-RQ-006"]`)

## Generering (initial)

Exempel:

```bash
python3 tools/dsc_synth.py testdata/dsc/synthetic/dsc_vec_001.iq16 \
  --wav testdata/dsc/synthetic/dsc_vec_001.wav \
  --meta testdata/dsc/synthetic/dsc_vec_001.json \
  --sample-rate 48000 \
  --symbol-rate 1200 \
  --snr-db 35 \
  --seed 1001
```

## Definition av klart för steg 2

- Minst 10 syntetiska profiler definierade.
- Manifestformat låst och dokumenterat.
- Varje profil mappad till minst ett krav-ID.
