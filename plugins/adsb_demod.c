/**
 * adsb_demod.c — ADS-B / Mode S demodulator
 *
 * Kräver IQ vid 2 Msps (eller 2048 ksps) inställt på 1090 MHz.
 *
 * Algoritm:
 *   1. Beräkna magnituden (I²+Q²) per sampel.
 *   2. Sök efter Mode S-preambel (8 µs = 16 samplar vid 2 Msps):
 *        HÖG: sampelpositioner 0, 2, 7, 9
 *        LÅG: sampelpositioner 1, 3–6, 8, 10–15
 *   3. PPM-avkoda bitar (hög+låg=1, låg+hög=0, 1 µs = 2 samplar/bit).
 *   4. Validera CRC-24.
 *   5. Emittera godkänd ram som "ADSB" med råhex som payload.
 *
 * Normaliserade fält: df, icao, bits
 */

#include "mr_plugin_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Mode S-konstanter                                                    */
/* ------------------------------------------------------------------ */

#define PREAMBLE_SAMPS  16u   /* 8 µs × 2 samplar/µs */
#define BIT_SAMPS        2u   /* 1 µs × 2 samplar/µs */
#define SHORT_BITS      56u
#define LONG_BITS      112u
#define LONG_SAMPS     (LONG_BITS * BIT_SAMPS)          /* 224 */
#define FRAME_SAMPS    (PREAMBLE_SAMPS + LONG_SAMPS)    /* 240 */

#define MODES_POLY 0xFFF409u  /* CRC-24-generatorpolynom för Mode S */

/* ------------------------------------------------------------------ */
/* Kontext                                                              */
/* ------------------------------------------------------------------ */

typedef struct {
  uint32_t* mag;
  uint32_t  mag_cap;
  uint32_t  mag_len;
  uint64_t  preamble_count;
  uint64_t  crc_ok_count;
  uint64_t  crc_fail_count;
  uint64_t  df17_ok_count;
  uint64_t  df17_fail_count;
  uint64_t  last_stats_ms;
} AdsbCtx;

/* ------------------------------------------------------------------ */
/* CRC-24                                                               */
/* ------------------------------------------------------------------ */

static uint32_t crc24(const uint8_t* data, uint32_t n) {
  uint32_t crc = 0;
  for (uint32_t i = 0; i < n; ++i) {
    crc ^= (uint32_t)data[i] << 16;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x800000u) ? (crc << 1) ^ MODES_POLY : crc << 1;
      crc &= 0xFFFFFFu;
    }
  }
  return crc;
}

/* ------------------------------------------------------------------ */
/* Preambeldetektion                                                    */
/* ------------------------------------------------------------------ */

/* HIGH vid 0,2,7,9 — LÅG vid 1,3–6,8 och guard-intervall 10–15 */
static int preamble_ok(const uint32_t* m) {
  /* Lokal kontrast: varje hög-puls måste överstiga sina direkta grannar × 2 */
  if (m[0] < m[1] * 2u) return 0;
  if (m[2] < m[1] * 2u || m[2] < m[3] * 2u) return 0;
  if (m[7] < m[6] * 2u || m[7] < m[8] * 2u) return 0;
  if (m[9] < m[8] * 2u || m[9] < m[10] * 2u) return 0;

  /* Pulslikhet: de fyra hög-pulserna ska vara inom 3:1 i effekt */
  uint32_t hi_min = m[0], hi_max = m[0];
  if (m[2] < hi_min) hi_min = m[2]; else if (m[2] > hi_max) hi_max = m[2];
  if (m[7] < hi_min) hi_min = m[7]; else if (m[7] > hi_max) hi_max = m[7];
  if (m[9] < hi_min) hi_min = m[9]; else if (m[9] > hi_max) hi_max = m[9];
  if (hi_min * 3u < hi_max) return 0;

  /* Summa-SNR: summan av 4 höga ≥ summan av 12 låga inkl. guard-intervall */
  uint32_t h = m[0] + m[2] + m[7] + m[9];
  uint32_t l = m[1] + m[3] + m[4] + m[5] + m[6] + m[8]
             + m[10] + m[11] + m[12] + m[13] + m[14] + m[15];
  return h >= l;
}

/* ------------------------------------------------------------------ */
/* Bitavkodning (PPM)                                                   */
/* ------------------------------------------------------------------ */

static int decode_bit(uint32_t s0, uint32_t s1) {
  return (s0 > s1) ? 1 : 0;
}

/* ------------------------------------------------------------------ */
/* Ramemission                                                          */
/* ------------------------------------------------------------------ */

static void emit_frame(const uint8_t* frame, uint32_t n_bytes,
                       double freq_hz, uint64_t unix_ms,
                       MrEmitFn emit_fn, void* user_data) {
  char* hex = (char*)malloc(n_bytes * 2u + 1u);
  if (!hex) return;
  for (uint32_t i = 0; i < n_bytes; ++i)
    snprintf(hex + i * 2u, 3, "%02X", (unsigned)frame[i]);

  uint32_t df   = (frame[0] >> 3) & 0x1Fu;
  uint32_t icao = ((uint32_t)frame[1] << 16) |
                  ((uint32_t)frame[2] <<  8) |
                   (uint32_t)frame[3];

  char kv[96];
  snprintf(kv, sizeof(kv),
           "{\"df\":\"%u\",\"icao\":\"%06X\",\"bits\":\"%u\"}",
           df, icao, n_bytes * 8u);

  emit_fn("ADSB", hex, freq_hz, unix_ms, kv, user_data);
  free(hex);
}

/* ================================================================== */
/* Plugin-gränssnitt                                                    */
/* ================================================================== */

MrPluginCtx* mr_plugin_create(void) {
  AdsbCtx* ctx = (AdsbCtx*)calloc(1, sizeof(AdsbCtx));
  if (!ctx) return NULL;
  ctx->mag_cap = FRAME_SAMPS + 65536u;
  ctx->mag = (uint32_t*)malloc(ctx->mag_cap * sizeof(uint32_t));
  if (!ctx->mag) { free(ctx); return NULL; }
  return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
  if (!raw) return;
  AdsbCtx* ctx = (AdsbCtx*)raw;
  free(ctx->mag);
  free(ctx);
}

static const MrPluginMeta kMeta = {
  "adsb_demod", "1.0.0", MR_PLUGIN_API_VERSION,
  "ADS-B / Mode S-demodulator (2 Msps, CRC-24)",
  MR_PLUGIN_ROLE_DEMODULATOR
};
const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
  (void)raw; (void)key; (void)value;
  return 0;
}

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
  if (!raw || !iq || !num_pairs || !emit_fn) return;
  AdsbCtx* ctx = (AdsbCtx*)raw;

  /* Kräver ~2 Msps (acceptera 1.8–2.2 Msps) */
  if (sr < 1800000u || sr > 2200000u) return;

  /* Utöka buffert vid behov */
  const uint32_t needed = ctx->mag_len + num_pairs;
  if (needed > ctx->mag_cap) {
    uint32_t new_cap = needed + 65536u;
    uint32_t* nb = (uint32_t*)realloc(ctx->mag, new_cap * sizeof(uint32_t));
    if (!nb) { ctx->mag_len = 0u; return; }
    ctx->mag = nb;
    ctx->mag_cap = new_cap;
  }

  /* IQ → magnitud (I²+Q²), undviker heltalsspill via int32_t */
  for (uint32_t n = 0; n < num_pairs; ++n) {
    int32_t i = (int32_t)iq[n * 2u];
    int32_t q = (int32_t)iq[n * 2u + 1u];
    ctx->mag[ctx->mag_len++] = (uint32_t)(i * i) + (uint32_t)(q * q);
  }

  /* Samplar per bit ur faktisk sample rate (t.ex. 2.0 vid 2 Msps, 2.048 vid 2.048 Msps) */
  float spb = (float)sr * 1e-6f;
  /* Minsta buffert för att kunna avkoda en max-lång ram */
  uint32_t min_frame = PREAMBLE_SAMPS + (uint32_t)((float)LONG_BITS * spb) + 2u;

  /* Sök efter preamblar och avkoda ramar */
  uint32_t p = 0;
  while (p + min_frame <= ctx->mag_len) {
    if (!preamble_ok(ctx->mag + p)) { ++p; continue; }
    ++ctx->preamble_count;

    const uint32_t* data = ctx->mag + p + PREAMBLE_SAMPS;

    /* DF-fält (5 bitar) med korrekt sampelposition för faktisk sr */
    uint32_t df5 = 0;
    for (int b = 0; b < 5; ++b) {
      uint32_t s = (uint32_t)((float)b * spb);
      df5 = (df5 << 1) | (uint32_t)decode_bit(data[s], data[s + 1u]);
    }
    uint32_t n_bits  = (df5 >= 16u) ? LONG_BITS : SHORT_BITS;
    uint32_t n_samps = PREAMBLE_SAMPS + (uint32_t)((float)n_bits * spb) + 2u;

    /* Vänta om vi inte har tillräckligt med samplar för hela ramen */
    if (p + n_samps > ctx->mag_len) break;

    /* Avkoda alla bitar med korrekt sampelposition för faktisk sr */
    uint8_t frame[LONG_BITS / 8u];
    memset(frame, 0, sizeof(frame));
    for (uint32_t bit = 0; bit < n_bits; ++bit) {
      uint32_t s = (uint32_t)((float)bit * spb);
      if (decode_bit(data[s], data[s + 1u]))
        frame[bit / 8u] |= (uint8_t)(0x80u >> (bit % 8u));
    }

    uint32_t n_bytes = n_bits / 8u;

    /* CRC-24-validering */
    uint32_t crc_calc = crc24(frame, n_bytes - 3u);
    uint32_t crc_fram = ((uint32_t)frame[n_bytes - 3u] << 16) |
                        ((uint32_t)frame[n_bytes - 2u] <<  8) |
                         (uint32_t)frame[n_bytes - 1u];

    uint32_t df = (frame[0] >> 3) & 0x1Fu;
    if (crc_calc == crc_fram) {
      ++ctx->crc_ok_count;
      if (df == 17u) ++ctx->df17_ok_count;
      emit_frame(frame, n_bytes, freq_hz, unix_ms, emit_fn, user_data);
      p += n_samps - 2u;
    } else {
      if (df == 17u) {
        if (ctx->df17_fail_count < 5u) {
          fprintf(stderr, "[ADS-B DBG] DF17 #%llu sr=%u spb=%.3f: ",
                  (unsigned long long)ctx->df17_fail_count + 1u, sr, (double)spb);
          for (uint32_t x = 0; x < n_bytes; ++x)
            fprintf(stderr, "%02X", frame[x]);
          fprintf(stderr, "  calc=%06X rx=%06X\n", crc_calc, crc_fram);
        }
        ++ctx->df17_fail_count;
      }
      ++ctx->crc_fail_count;
      ++p;
    }
  }

  /* Skriv ut statistik var 10:e sekund */
  if (unix_ms - ctx->last_stats_ms >= 10000u) {
    ctx->last_stats_ms = unix_ms;
    fprintf(stderr,
            "[ADS-B] preamble=%llu  crc_ok=%llu  crc_fail=%llu"
            "  df17_ok=%llu  df17_fail=%llu\n",
            (unsigned long long)ctx->preamble_count,
            (unsigned long long)ctx->crc_ok_count,
            (unsigned long long)ctx->crc_fail_count,
            (unsigned long long)ctx->df17_ok_count,
            (unsigned long long)ctx->df17_fail_count);
  }

  /* Behåll obearbetade samplar för nästa block */
  uint32_t keep = (p < ctx->mag_len) ? (ctx->mag_len - p) : 0u;
  if (p > 0u && keep > 0u)
    memmove(ctx->mag, ctx->mag + p, keep * sizeof(uint32_t));
  ctx->mag_len = keep;
}
