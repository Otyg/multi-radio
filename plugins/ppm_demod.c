/**
 * ppm_demod.c — Biphase energi-demodulator
 *
 * Varje bit är uppdelad i två lika långa halvperioder ("chirps"):
 *   Logisk 1:  energi HÖG första halvan, LÅG andra halvan  (h0 > h1)
 *   Logisk 0:  energi LÅG första halvan, HÖG andra halvan  (h1 > h0)
 *
 * Parameter:  bit_duration_us  (mikrosekunder per bit, default 10)
 */

#include "mr_plugin_api.h"
#include "mr_signal_gate.h"
#include "mr_bit_buf.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PPM_DEFAULT_BIT_US     10u   /* 10 µs = 100 kbit/s */
#define PPM_MAX_BITS           2048
#define PPM_MIN_BITS           8
#define PPM_MIN_SAMPLES_PER_SYM 4u  /* minimum för pålitlig halvperiodsdetektion */

typedef struct {
  uint32_t bit_duration_us;
  uint32_t sample_rate_hz;
  uint32_t samples_per_sym;

  uint32_t sym_pos;
  float    half0_energy;
  float    half1_energy;

  MrSignalGate gate;

  uint8_t  bit_buf[PPM_MAX_BITS / 8 + 1];
  uint32_t bit_count;
} PpmCtx;

static void ppm_flush(PpmCtx* ctx, MrEmitFn emit_fn, void* user_data,
                      double freq_hz, uint64_t unix_ms) {
  if (ctx->bit_count < (uint32_t)PPM_MIN_BITS) { ctx->bit_count = 0; return; }
  const uint32_t byte_count = ctx->bit_count / 8;
  char* hex = (char*)malloc(byte_count * 2 + 1);
  if (!hex) { ctx->bit_count = 0; return; }
  for (uint32_t i = 0; i < byte_count; ++i)
    snprintf(hex + i * 2, 3, "%02X", (unsigned)ctx->bit_buf[i]);
  char kv[96];
  snprintf(kv, sizeof(kv),
           "{\"bit_duration_us\":\"%u\",\"bit_count\":\"%u\"}",
           ctx->bit_duration_us, byte_count * 8);
  if (emit_fn) emit_fn("PPM_DATA", hex, freq_hz, unix_ms, kv, user_data);
  free(hex);
  ctx->bit_count = 0;
  memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
}

static void ppm_reconfigure(PpmCtx* ctx, uint32_t sample_rate_hz) {
  if (sample_rate_hz == ctx->sample_rate_hz) return;
  ctx->sample_rate_hz = sample_rate_hz;
  /* samples_per_sym = sample_rate * bit_duration_us / 1e6 */
  ctx->samples_per_sym = (ctx->bit_duration_us > 0u)
      ? (uint32_t)((uint64_t)sample_rate_hz * ctx->bit_duration_us / 1000000u)
      : 2u;
  if (ctx->samples_per_sym < 2u) ctx->samples_per_sym = 2u;
  ctx->sym_pos      = 0;
  ctx->half0_energy = 0.0f;
  ctx->half1_energy = 0.0f;
  mr_signal_gate_reset(&ctx->gate);
}

MrPluginCtx* mr_plugin_create(void) {
  PpmCtx* ctx = (PpmCtx*)calloc(1, sizeof(PpmCtx));
  if (!ctx) return NULL;
  const char* env = getenv("MR_PPM_BIT_DURATION_US");
  ctx->bit_duration_us = (env && atoi(env) > 0)
      ? (uint32_t)atoi(env) : PPM_DEFAULT_BIT_US;
  mr_signal_gate_init(&ctx->gate, MR_GATE_SQUELCH_RATIO);
  return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
  free(raw);
}

static const MrPluginMeta kMeta = {
  "ppm_demod", "1.1.0", MR_PLUGIN_API_VERSION,
  "Biphase demodulator: hög+låg=1, låg+hög=0 (halvperiods-energijämförelse)",
  MR_PLUGIN_ROLE_DEMODULATOR
};
const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
  if (!raw || !key || !value) return 0;
  PpmCtx* ctx = (PpmCtx*)raw;
  if (strcmp(key, "bit_duration_us") == 0) {
    const long v = atol(value);
    if (v > 0) {
      ctx->bit_duration_us = (uint32_t)v;
      ctx->sample_rate_hz  = 0;
    }
    return 1;
  }
  /* data_rate (bps) används som fallback om bit_duration_us inte satts */
  if (strcmp(key, "data_rate") == 0) {
    const long v = atol(value);
    if (v > 0 && ctx->bit_duration_us == 0) {
      ctx->bit_duration_us = (uint32_t)(1000000ul / (unsigned long)v);
      if (!ctx->bit_duration_us) ctx->bit_duration_us = 1u;
      ctx->sample_rate_hz  = 0;
    }
    return 1;
  }
  return 0;
}

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
  if (!raw || !iq || !num_pairs) return;
  PpmCtx* ctx = (PpmCtx*)raw;
  if (!sr) sr = 2048000u;
  ppm_reconfigure(ctx, sr);

  if (ctx->samples_per_sym < PPM_MIN_SAMPLES_PER_SYM) return;

  const uint32_t half = ctx->samples_per_sym / 2u;
  const float norm = 1.0f / 32768.0f;

  for (uint32_t n = 0; n < num_pairs; ++n) {
    const float ii = (float)iq[n * 2]     * norm;
    const float qq = (float)iq[n * 2 + 1] * norm;
    const float energy = ii * ii + qq * qq;

    if (ctx->sym_pos < half)
      ctx->half0_energy += energy;
    else
      ctx->half1_energy += energy;

    ctx->sym_pos++;
    if (ctx->sym_pos < ctx->samples_per_sym) continue;

    /* Slut på symbolfönster */
    ctx->sym_pos = 0;
    const float total   = ctx->half0_energy + ctx->half1_energy;
    const float avg     = total / (float)ctx->samples_per_sym;
    const float h0      = ctx->half0_energy;
    const float h1      = ctx->half1_energy;
    ctx->half0_energy   = 0.0f;
    ctx->half1_energy   = 0.0f;

    const int falling = mr_signal_gate_update(&ctx->gate, avg, MR_GATE_HOLD_SYMS);

    if (!ctx->gate.gate_open) {
      if (falling)
        ppm_flush(ctx, emit_fn, user_data, freq_hz, unix_ms);
      else {
        ctx->bit_count = 0;
        memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
      }
      continue;
    }

    /* Hög första halvperiod → 1, hög andra halvperiod → 0 */
    mr_push_bit(ctx->bit_buf, &ctx->bit_count, PPM_MAX_BITS, (h0 > h1) ? 1 : 0);

    if (ctx->bit_count >= (uint32_t)PPM_MAX_BITS)
      ppm_flush(ctx, emit_fn, user_data, freq_hz, unix_ms);
  }
}
