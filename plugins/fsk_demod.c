/**
 * fsk_demod.c — 2-FSK demodulator plugin
 *
 * When libliquid is available (MR_PLUGIN_HAS_LIQUID):
 *   msresamp_crcf  — resample hardware SR → k * baud_rate
 *   fskdem         — frequency-bank FSK demodulator
 *
 * Fallback (no libliquid):
 *   FM discriminator + boxcar LPF + simple clock recovery
 *
 * Environment variables:
 *   MR_FSK_BAUD_RATE      (default 4800)
 *   MR_FSK_DEVIATION_HZ   (default 2400)
 */

#include "mr_plugin_api.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FSK_DEFAULT_BAUD_RATE    4800
#define FSK_DEFAULT_DEVIATION_HZ 2400
#define FSK_K                    8     /* samples per symbol fed to demodulator */
#define FSK_MAX_BITS             1024
#define FSK_MIN_BITS             8
#define FSK_IDLE_GAP_SYMS        8     /* symbols of silence before emitting */

/* ------------------------------------------------------------------ */
/* Shared bit-buffer helpers                                            */
/* ------------------------------------------------------------------ */

static void push_bit(uint8_t* buf, uint32_t* count, unsigned int bit) {
  if (*count >= FSK_MAX_BITS) return;
  const uint32_t byte_idx = *count / 8;
  const uint32_t bit_idx  = 7 - (*count % 8);
  if (bit) buf[byte_idx] |=  (uint8_t)(1u << bit_idx);
  else     buf[byte_idx] &= (uint8_t)~(1u << bit_idx);
  ++(*count);
}

static void emit_bits(uint8_t* bit_buf, uint32_t* bit_count,
                      MrEmitFn emit_fn, void* user_data,
                      double freq_hz, uint64_t unix_ms,
                      unsigned int baud_rate, unsigned int deviation_hz) {
  if (*bit_count < (uint32_t)FSK_MIN_BITS) {
    *bit_count = 0;
    return;
  }
  const uint32_t byte_count = *bit_count / 8;
  char* hex = (char*)malloc(byte_count * 2 + 1);
  if (!hex) { *bit_count = 0; return; }
  for (uint32_t i = 0; i < byte_count; ++i)
    snprintf(hex + i * 2, 3, "%02X", (unsigned)bit_buf[i]);
  char kv[128];
  snprintf(kv, sizeof(kv),
           "{\"baud_rate\":\"%u\",\"deviation_hz\":\"%u\",\"bit_count\":\"%u\"}",
           baud_rate, deviation_hz, byte_count * 8);
  if (emit_fn) emit_fn("FSK_DATA", hex, freq_hz, unix_ms, kv, user_data);
  free(hex);
  *bit_count = 0;
  memset(bit_buf, 0, FSK_MAX_BITS / 8 + 1);
}

/* ================================================================== */
/* libliquid path                                                       */
/* ================================================================== */

#if defined(MR_PLUGIN_HAS_LIQUID) && MR_PLUGIN_HAS_LIQUID
#include <liquid/liquid.h>

typedef struct {
  uint32_t baud_rate;
  uint32_t deviation_hz;

  uint32_t sample_rate_hz;  /* last configured */

  msresamp_crcf    resampler;
  fskdem           demodulator;

  /* partial symbol accumulation buffer */
  liquid_float_complex* sym_buf;
  uint32_t sym_buf_fill;

  /* output buffer for resampler (heap-allocated, grown as needed) */
  liquid_float_complex* resamp_out;
  uint32_t resamp_out_cap;

  /* idle detection: count symbols with near-zero energy */
  uint32_t idle_sym_count;

  uint8_t  bit_buf[FSK_MAX_BITS / 8 + 1];
  uint32_t bit_count;
} FskCtx;

static void fsk_teardown(FskCtx* ctx) {
  if (ctx->resampler)   { msresamp_crcf_destroy(ctx->resampler);  ctx->resampler   = NULL; }
  if (ctx->demodulator) { fskdem_destroy(ctx->demodulator);        ctx->demodulator = NULL; }
  free(ctx->sym_buf);   ctx->sym_buf     = NULL;
  free(ctx->resamp_out); ctx->resamp_out = NULL;
  ctx->resamp_out_cap = 0;
  ctx->sym_buf_fill   = 0;
}

static int fsk_configure(FskCtx* ctx, uint32_t sample_rate_hz) {
  if (sample_rate_hz == ctx->sample_rate_hz) return 1;
  fsk_teardown(ctx);
  ctx->sample_rate_hz = sample_rate_hz;

  /* Resample to exactly FSK_K samples per symbol */
  const float target_sr = (float)ctx->baud_rate * FSK_K;
  const float rate      = target_sr / (float)sample_rate_hz;
  ctx->resampler = msresamp_crcf_create(rate, 60.0f);
  if (!ctx->resampler) return 0;

  /* normalized bandwidth ≈ deviation / (baud/2), clamped to (0, 0.5) */
  float bw = (float)ctx->deviation_hz / ((float)ctx->baud_rate * 0.5f);
  if (bw <= 0.0f) bw = 0.1f;
  if (bw >= 0.5f) bw = 0.45f;
  ctx->demodulator = fskdem_create(2, FSK_K, bw);
  if (!ctx->demodulator) return 0;

  ctx->sym_buf = (liquid_float_complex*)calloc(FSK_K, sizeof(liquid_float_complex));
  if (!ctx->sym_buf) return 0;

  /* initial output buffer: 2× worst-case upsample headroom */
  const uint32_t init_cap = (uint32_t)(2048.0f / rate) + 64;
  ctx->resamp_out = (liquid_float_complex*)malloc(init_cap * sizeof(liquid_float_complex));
  if (!ctx->resamp_out) return 0;
  ctx->resamp_out_cap = init_cap;

  ctx->idle_sym_count = 0;
  ctx->sym_buf_fill   = 0;
  return 1;
}

MrPluginCtx* mr_plugin_create(void) {
  FskCtx* ctx = (FskCtx*)calloc(1, sizeof(FskCtx));
  if (!ctx) return NULL;
  const char* b = getenv("MR_FSK_BAUD_RATE");
  const char* d = getenv("MR_FSK_DEVIATION_HZ");
  ctx->baud_rate    = (b && atoi(b) > 0) ? (uint32_t)atoi(b) : FSK_DEFAULT_BAUD_RATE;
  ctx->deviation_hz = (d && atoi(d) > 0) ? (uint32_t)atoi(d) : FSK_DEFAULT_DEVIATION_HZ;
  return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
  if (!raw) return;
  FskCtx* ctx = (FskCtx*)raw;
  fsk_teardown(ctx);
  free(ctx);
}

static const MrPluginMeta kMeta = {
  "fsk_demod", "2.0.0", MR_PLUGIN_API_VERSION,
  "2-FSK demodulator (libliquid fskdem + msresamp_crcf)"
};
const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
  if (!raw || !iq || num_pairs == 0) return;
  FskCtx* ctx = (FskCtx*)raw;
  if (!sr) sr = 2048000;
  if (!fsk_configure(ctx, sr)) return;

  /* Grow resampler output buffer if needed */
  const uint32_t needed = (uint32_t)((float)num_pairs *
      ((float)ctx->baud_rate * FSK_K / (float)sr) + 64);
  if (needed > ctx->resamp_out_cap) {
    liquid_float_complex* nb =
        (liquid_float_complex*)realloc(ctx->resamp_out, needed * sizeof(*nb));
    if (!nb) return;
    ctx->resamp_out     = nb;
    ctx->resamp_out_cap = needed;
  }

  /* Convert int16 IQ → liquid_float_complex */
  liquid_float_complex* in_buf =
      (liquid_float_complex*)malloc(num_pairs * sizeof(liquid_float_complex));
  if (!in_buf) return;
  const float norm = 1.0f / 32768.0f;
  for (uint32_t n = 0; n < num_pairs; ++n) {
    __real__ in_buf[n] = (float)iq[n * 2]     * norm;
    __imag__ in_buf[n] = (float)iq[n * 2 + 1] * norm;
  }

  /* Resample */
  unsigned int n_out = 0;
  msresamp_crcf_execute(ctx->resampler, in_buf, num_pairs, ctx->resamp_out, &n_out);
  free(in_buf);

  /* Feed resampled samples into fskdem FSK_K at a time */
  for (unsigned int i = 0; i < n_out; ++i) {
    ctx->sym_buf[ctx->sym_buf_fill++] = ctx->resamp_out[i];
    if (ctx->sym_buf_fill < (uint32_t)FSK_K) continue;
    ctx->sym_buf_fill = 0;

    /* Energy check for idle detection */
    float energy = 0.0f;
    for (int s = 0; s < FSK_K; ++s) {
      const float re = __real__ ctx->sym_buf[s];
      const float im = __imag__ ctx->sym_buf[s];
      energy += re * re + im * im;
    }
    if (energy < 1e-6f) {
      if (++ctx->idle_sym_count >= (uint32_t)FSK_IDLE_GAP_SYMS && ctx->bit_count >= FSK_MIN_BITS) {
        emit_bits(ctx->bit_buf, &ctx->bit_count, emit_fn, user_data,
                  freq_hz, unix_ms, ctx->baud_rate, ctx->deviation_hz);
        ctx->idle_sym_count = 0;
      }
      continue;
    }
    ctx->idle_sym_count = 0;

    unsigned int sym = 0;
    fskdem_demodulate(ctx->demodulator, ctx->sym_buf, &sym);
    push_bit(ctx->bit_buf, &ctx->bit_count, sym & 1u);

    if (ctx->bit_count >= FSK_MAX_BITS)
      emit_bits(ctx->bit_buf, &ctx->bit_count, emit_fn, user_data,
                freq_hz, unix_ms, ctx->baud_rate, ctx->deviation_hz);
  }
}

/* ================================================================== */
/* Fallback: no libliquid                                               */
/* ================================================================== */
#else /* !MR_PLUGIN_HAS_LIQUID */

typedef struct {
  uint32_t baud_rate;
  uint32_t deviation_hz;
  float prev_i, prev_q;
  float*   lpf_buf;
  uint32_t lpf_len, lpf_pos;
  float    lpf_sum;
  uint32_t samples_per_symbol, sample_rate_hz;
  uint32_t sym_acc;
  float    sym_val;
  uint32_t sym_n;
  uint32_t idle_samples;
  uint8_t  bit_buf[FSK_MAX_BITS / 8 + 1];
  uint32_t bit_count;
} FskCtx;

static void fsk_reconfigure_fallback(FskCtx* ctx, uint32_t sr) {
  if (sr == ctx->sample_rate_hz) return;
  ctx->sample_rate_hz      = sr;
  ctx->samples_per_symbol  = sr / ctx->baud_rate;
  if (!ctx->samples_per_symbol) ctx->samples_per_symbol = 1;
  const uint32_t lpf_len = ctx->samples_per_symbol / 2 ? ctx->samples_per_symbol / 2 : 1;
  if (lpf_len != ctx->lpf_len) {
    free(ctx->lpf_buf);
    ctx->lpf_buf = (float*)calloc(lpf_len, sizeof(float));
    ctx->lpf_len = lpf_len; ctx->lpf_pos = 0; ctx->lpf_sum = 0.0f;
  }
  ctx->sym_acc = 0; ctx->sym_val = 0.0f; ctx->sym_n = 0;
}

MrPluginCtx* mr_plugin_create(void) {
  FskCtx* ctx = (FskCtx*)calloc(1, sizeof(FskCtx));
  if (!ctx) return NULL;
  const char* b = getenv("MR_FSK_BAUD_RATE");
  const char* d = getenv("MR_FSK_DEVIATION_HZ");
  ctx->baud_rate    = (b && atoi(b) > 0) ? (uint32_t)atoi(b) : FSK_DEFAULT_BAUD_RATE;
  ctx->deviation_hz = (d && atoi(d) > 0) ? (uint32_t)atoi(d) : FSK_DEFAULT_DEVIATION_HZ;
  ctx->lpf_len = 4;
  ctx->lpf_buf = (float*)calloc(4, sizeof(float));
  return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
  if (!raw) return;
  FskCtx* ctx = (FskCtx*)raw;
  free(ctx->lpf_buf);
  free(ctx);
}

static const MrPluginMeta kMeta = {
  "fsk_demod", "2.0.0", MR_PLUGIN_API_VERSION,
  "2-FSK demodulator (FM discriminator fallback)"
};
const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
  if (!raw || !iq || !num_pairs) return;
  FskCtx* ctx = (FskCtx*)raw;
  if (!sr) sr = 2048000;
  fsk_reconfigure_fallback(ctx, sr);
  const float norm = 1.0f / 32768.0f;
  for (uint32_t n = 0; n < num_pairs; ++n) {
    const float is = (float)iq[n*2]   * norm;
    const float qs = (float)iq[n*2+1] * norm;
    const float cross = qs * ctx->prev_i - is * ctx->prev_q;
    const float dot   = is * ctx->prev_i + qs * ctx->prev_q;
    float angle = (dot != 0.0f || cross != 0.0f) ? atan2f(cross, dot) : 0.0f;
    ctx->prev_i = is; ctx->prev_q = qs;
    /* boxcar LPF */
    ctx->lpf_sum -= ctx->lpf_buf[ctx->lpf_pos];
    ctx->lpf_buf[ctx->lpf_pos] = angle;
    ctx->lpf_sum += angle;
    ctx->lpf_pos = (ctx->lpf_pos + 1) % ctx->lpf_len;
    const float filtered = ctx->lpf_sum / (float)ctx->lpf_len;
    const float af = filtered < 0.0f ? -filtered : filtered;
    if (af < 0.01f) {
      if (++ctx->idle_samples >= FSK_IDLE_GAP_SYMS * ctx->samples_per_symbol
          && ctx->bit_count >= FSK_MIN_BITS) {
        emit_bits(ctx->bit_buf, &ctx->bit_count, emit_fn, user_data,
                  freq_hz, unix_ms, ctx->baud_rate, ctx->deviation_hz);
        ctx->idle_samples = 0;
      }
    } else { ctx->idle_samples = 0; }
    ctx->sym_val += filtered; ctx->sym_n++; ctx->sym_acc++;
    if (ctx->sym_acc >= ctx->samples_per_symbol) {
      push_bit(ctx->bit_buf, &ctx->bit_count,
               ctx->sym_val / (float)ctx->sym_n > 0.0f ? 1 : 0);
      ctx->sym_acc = 0; ctx->sym_val = 0.0f; ctx->sym_n = 0;
      if (ctx->bit_count >= FSK_MAX_BITS)
        emit_bits(ctx->bit_buf, &ctx->bit_count, emit_fn, user_data,
                  freq_hz, unix_ms, ctx->baud_rate, ctx->deviation_hz);
    }
  }
}
#endif /* MR_PLUGIN_HAS_LIQUID */
