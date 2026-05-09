/**
 * gmsk_demod.c — GMSK demodulator plugin
 *
 * When libliquid is available (MR_PLUGIN_HAS_LIQUID):
 *   msresamp_crcf  — resample hardware SR → k * baud_rate
 *   gmskdem        — GMSK demodulator with built-in Gaussian matched filter
 *
 * Fallback (no libliquid):
 *   FM discriminator + Gaussian FIR LPF + simple clock recovery
 *
 * Environment variables:
 *   MR_GMSK_BAUD_RATE   (default 9600)
 *   MR_GMSK_BT          (default 0.3)
 */

#include "mr_plugin_api.h"
#include "mr_signal_gate.h"
#include "mr_afc.h"
#include "mr_bit_buf.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GMSK_DEFAULT_BAUD_RATE 9600
#define GMSK_DEFAULT_BT        0.3f
#define GMSK_K                 8     /* samples per symbol */
#define GMSK_M                 5     /* filter delay in symbols; 5 gives better BER for BT 0.3-0.5 */
#define GMSK_MAX_BITS          1024
#define GMSK_MIN_BITS          8
#define GMSK_IDLE_GAP_SYMS     8

/* ------------------------------------------------------------------ */
/* GMSK-specific bit-emission (builds protocol KV, then delegates)     */
/* ------------------------------------------------------------------ */

static void emit_bits(uint8_t* bit_buf, uint32_t* bit_count,
                      MrEmitFn emit_fn, void* user_data,
                      double freq_hz, uint64_t unix_ms,
                      unsigned int baud_rate, float bt,
                      uint32_t channel_bandwidth_hz) {
  char kv[200];
  snprintf(kv, sizeof(kv),
           "{\"baud_rate\":\"%u\",\"bt\":\"%.3f\",\"channel_bw_hz\":\"%u\",\"bit_count\":\"%u\"}",
           baud_rate, (double)bt, channel_bandwidth_hz, (*bit_count / 8u) * 8u);
  mr_emit_bits(bit_buf, bit_count, GMSK_MIN_BITS, GMSK_MAX_BITS / 8u + 1u,
               "GMSK_DATA", kv, freq_hz, unix_ms, emit_fn, user_data);
}

/* ================================================================== */
/* libliquid path                                                       */
/* ================================================================== */

#if defined(MR_PLUGIN_HAS_LIQUID) && MR_PLUGIN_HAS_LIQUID
#include <liquid/liquid.h>
#include "mr_iir_prefilter.h"

typedef struct {
  uint32_t baud_rate;
  float    bt;
  float    modulation_index;
  uint32_t channel_bandwidth_hz;
  int      needs_reconfigure;

  uint32_t sample_rate_hz;

  msresamp_crcf  resampler;
  gmskdem        demodulator;
  MrIirPrefilter pre_filter;  /* channel-select LP before resampling */

  liquid_float_complex* sym_buf;
  uint32_t sym_buf_fill;

  liquid_float_complex* resamp_out;
  uint32_t resamp_out_cap;

  MrSignalGate gate;   /* energy-based signal gate */
  MrAfc        afc;    /* carrier frequency correction */

  uint8_t  bit_buf[GMSK_MAX_BITS / 8 + 1];
  uint32_t bit_count;
} GmskCtx;

static void gmsk_teardown(GmskCtx* ctx) {
  if (ctx->resampler)   { msresamp_crcf_destroy(ctx->resampler); ctx->resampler  = NULL; }
  if (ctx->demodulator) { gmskdem_destroy(ctx->demodulator);      ctx->demodulator = NULL; }
  mr_iir_prefilter_destroy(&ctx->pre_filter);
  free(ctx->sym_buf);    ctx->sym_buf    = NULL;
  free(ctx->resamp_out); ctx->resamp_out = NULL;
  ctx->resamp_out_cap = 0;
  ctx->sym_buf_fill   = 0;
}

static int gmsk_configure(GmskCtx* ctx, uint32_t sr) {
  if (sr == ctx->sample_rate_hz && !ctx->needs_reconfigure) return 1;
  ctx->needs_reconfigure = 0;
  gmsk_teardown(ctx);
  ctx->sample_rate_hz = sr;
  /* Discard stale bits accumulated with old parameters. */
  ctx->bit_count = 0;
  memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
  mr_signal_gate_reset(&ctx->gate);
  mr_afc_init(&ctx->afc);

  /* Channel-select pre-filter:
     - if channel bandwidth is set, use half-bandwidth as LP cutoff
     - otherwise fall back to 2×baud_rate */
  float cutoff_hz = 2.0f * (float)ctx->baud_rate;
  if (ctx->channel_bandwidth_hz > 0u) cutoff_hz = 0.5f * (float)ctx->channel_bandwidth_hz;
  float cutoff = cutoff_hz / (float)sr;
  if (cutoff > 0.44f) cutoff = 0.44f;
  mr_iir_prefilter_create(&ctx->pre_filter, cutoff);

  const float rate = (float)ctx->baud_rate * GMSK_K / (float)sr;
  ctx->resampler = msresamp_crcf_create(rate, 60.0f);
  if (!ctx->resampler) return 0;

  ctx->demodulator = gmskdem_create(GMSK_K, GMSK_M, ctx->bt);
  if (!ctx->demodulator) return 0;

  ctx->sym_buf = (liquid_float_complex*)calloc(GMSK_K, sizeof(liquid_float_complex));
  if (!ctx->sym_buf) return 0;

  const uint32_t init_cap = (uint32_t)(2048.0f / rate) + 64;
  ctx->resamp_out = (liquid_float_complex*)malloc(init_cap * sizeof(liquid_float_complex));
  if (!ctx->resamp_out) return 0;
  ctx->resamp_out_cap = init_cap;

  ctx->sym_buf_fill = 0;
  return 1;
}

MrPluginCtx* mr_plugin_create(void) {
  GmskCtx* ctx = (GmskCtx*)calloc(1, sizeof(GmskCtx));
  if (!ctx) return NULL;
  const char* b  = getenv("MR_GMSK_BAUD_RATE");
  const char* bt = getenv("MR_GMSK_BT");
  ctx->baud_rate        = (b  && atoi(b)  > 0)   ? (uint32_t)atoi(b) : GMSK_DEFAULT_BAUD_RATE;
  ctx->bt               = (bt && atof(bt) > 0.0) ? (float)atof(bt)   : GMSK_DEFAULT_BT;
  ctx->modulation_index = 0.5f;
  ctx->channel_bandwidth_hz = 0u;
  mr_signal_gate_init(&ctx->gate, MR_GATE_SQUELCH_RATIO);
  mr_afc_init(&ctx->afc);
  mr_iir_prefilter_init(&ctx->pre_filter);
  return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
  if (!raw) return;
  GmskCtx* ctx = (GmskCtx*)raw;
  gmsk_teardown(ctx);
  free(ctx);
}

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
  if (!raw || !key || !value) return 0;
  GmskCtx* ctx = (GmskCtx*)raw;
  if (strcmp(key, "baud_rate") == 0) {
    const int v = atoi(value);
    if (v > 0) { ctx->baud_rate = (uint32_t)v; ctx->needs_reconfigure = 1; }
    return 1;
  }
  if (strcmp(key, "bt") == 0) {
    const float v = (float)atof(value);
    if (v > 0.0f) { ctx->bt = v; ctx->needs_reconfigure = 1; }
    return 1;
  }
  if (strcmp(key, "modulation_index") == 0) {
    const float v = (float)atof(value);
    if (v > 0.0f) ctx->modulation_index = v;
    return 1;
  }
  if (strcmp(key, "channel_bandwidth_hz") == 0) {
    const int v = atoi(value);
    if (v >= 0) { ctx->channel_bandwidth_hz = (uint32_t)v; ctx->needs_reconfigure = 1; }
    return 1;
  }
  if (strcmp(key, "squelch_db") == 0) {
    const float db = (float)atof(value);
    ctx->gate.squelch_ratio = (db <= 0.0f) ? 0.0f : powf(10.0f, db / 10.0f);
    return 1;
  }
  return 0;
}

static const MrPluginMeta kMeta = {
  "gmsk_demod", "2.0.0", MR_PLUGIN_API_VERSION,
  "GMSK demodulator (libliquid gmskdem + msresamp_crcf)"
, MR_PLUGIN_ROLE_DEMODULATOR
};
const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

/* ------------------------------------------------------------------ */
/* Per-symbol processing: energy gate, AFC update, demodulation        */
/* Resets bit_buf when gate closes.                                    */
/* ------------------------------------------------------------------ */

static void process_symbol(GmskCtx* ctx, liquid_float_complex* sym,
                            double freq_hz, uint64_t unix_ms,
                            MrEmitFn emit_fn, void* user_data) {
  float energy = 0.0f;
  for (int s = 0; s < GMSK_K; ++s) {
    const float re = __real__ sym[s];
    const float im = __imag__ sym[s];
    energy += re * re + im * im;
  }

  const int falling = mr_signal_gate_update(&ctx->gate,
                                             energy / (float)GMSK_K,
                                             MR_GATE_HOLD_SYMS);
  if (!ctx->gate.gate_open) {
    if (falling)
      emit_bits(ctx->bit_buf, &ctx->bit_count, emit_fn, user_data,
                freq_hz, unix_ms, ctx->baud_rate, ctx->bt, ctx->channel_bandwidth_hz);
    else {
      ctx->bit_count = 0;
      memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
    }
    return;
  }

  mr_afc_update(&ctx->afc, sym, GMSK_K);

  unsigned int bit = 0;
  gmskdem_demodulate(ctx->demodulator, sym, &bit);
  mr_push_bit(ctx->bit_buf, &ctx->bit_count, GMSK_MAX_BITS, bit & 1u);
  if (ctx->bit_count >= GMSK_MAX_BITS)
    emit_bits(ctx->bit_buf, &ctx->bit_count, emit_fn, user_data,
              freq_hz, unix_ms, ctx->baud_rate, ctx->bt, ctx->channel_bandwidth_hz);
}

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
  if (!raw || !iq || !num_pairs) return;
  GmskCtx* ctx = (GmskCtx*)raw;
  if (!sr) sr = 2048000;
  if (!gmsk_configure(ctx, sr)) return;

  const uint32_t needed = (uint32_t)((float)num_pairs *
      ((float)ctx->baud_rate * GMSK_K / (float)sr) + 64);
  if (needed > ctx->resamp_out_cap) {
    liquid_float_complex* nb =
        (liquid_float_complex*)realloc(ctx->resamp_out, needed * sizeof(*nb));
    if (!nb) return;
    ctx->resamp_out     = nb;
    ctx->resamp_out_cap = needed;
  }

  liquid_float_complex* in_buf =
      (liquid_float_complex*)malloc(num_pairs * sizeof(liquid_float_complex));
  if (!in_buf) return;
  const float norm = 1.0f / 32768.0f;
  for (uint32_t n = 0; n < num_pairs; ++n) {
    liquid_float_complex s;
    __real__ s = (float)iq[n * 2]     * norm;
    __imag__ s = (float)iq[n * 2 + 1] * norm;
    mr_iir_prefilter_execute(&ctx->pre_filter, s, &in_buf[n]);
  }

  unsigned int n_out = 0;
  msresamp_crcf_execute(ctx->resampler, in_buf, num_pairs, ctx->resamp_out, &n_out);
  free(in_buf);

  for (unsigned int i = 0; i < n_out; ++i) {
    /* NCO frequency correction via AFC */
    mr_afc_correct(&ctx->afc, ctx->resamp_out[i],
                   &ctx->sym_buf[ctx->sym_buf_fill]);
    ctx->sym_buf_fill++;
    if (ctx->sym_buf_fill < (uint32_t)GMSK_K) continue;
    ctx->sym_buf_fill = 0;
    process_symbol(ctx, ctx->sym_buf, freq_hz, unix_ms, emit_fn, user_data);
  }
}

/* ================================================================== */
/* Fallback: no libliquid                                               */
/* ================================================================== */
#else /* !MR_PLUGIN_HAS_LIQUID */

#define GMSK_MAX_FILTER_TAPS 512

typedef struct {
  uint32_t baud_rate;
  float    bt;
  float    modulation_index;
  uint32_t channel_bandwidth_hz;
  int      needs_reconfigure;
  float prev_i, prev_q;
  float    gauss_c[GMSK_MAX_FILTER_TAPS];
  float    gauss_d[GMSK_MAX_FILTER_TAPS];
  uint32_t gauss_len, gauss_pos;
  uint32_t samples_per_symbol, sample_rate_hz;
  uint32_t sym_acc;
  float    sym_val;
  uint32_t sym_n, idle_samples;
  MrSignalGate gate;
  uint8_t  bit_buf[GMSK_MAX_BITS / 8 + 1];
  uint32_t bit_count;
} GmskCtx;

static uint32_t build_gauss(float* c, uint32_t max, uint32_t sps, float bt) {
  if (!sps || bt <= 0.0f) { if (max) c[0] = 1.0f; return 1; }
  const float s = 0.8325546f / (6.2831853f * bt) * (float)sps;
  const uint32_t half = 3 * sps;
  uint32_t n = 2 * half + 1; if (n > max) n = max;
  float sum = 0.0f;
  for (uint32_t k = 0; k < n; ++k) {
    const float x = ((float)(int32_t)k - (float)half) / s;
    c[k] = expf(-0.5f * x * x); sum += c[k];
  }
  if (sum > 0.0f) for (uint32_t k = 0; k < n; ++k) c[k] /= sum;
  return n;
}

static void gmsk_reconfigure(GmskCtx* ctx, uint32_t sr) {
  if (sr == ctx->sample_rate_hz && !ctx->needs_reconfigure) return;
  ctx->needs_reconfigure = 0;
  ctx->bit_count = 0;
  memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
  ctx->idle_samples = 0;
  ctx->sample_rate_hz     = sr;
  ctx->samples_per_symbol = sr / ctx->baud_rate;
  if (!ctx->samples_per_symbol) ctx->samples_per_symbol = 1;
  ctx->gauss_len = build_gauss(ctx->gauss_c, GMSK_MAX_FILTER_TAPS,
                               ctx->samples_per_symbol, ctx->bt);
  memset(ctx->gauss_d, 0, sizeof(float) * ctx->gauss_len);
  ctx->gauss_pos = 0;
  ctx->sym_acc = 0; ctx->sym_val = 0.0f; ctx->sym_n = 0;
}

MrPluginCtx* mr_plugin_create(void) {
  GmskCtx* ctx = (GmskCtx*)calloc(1, sizeof(GmskCtx));
  if (!ctx) return NULL;
  const char* b  = getenv("MR_GMSK_BAUD_RATE");
  const char* bt = getenv("MR_GMSK_BT");
  ctx->baud_rate        = (b  && atoi(b)  > 0)   ? (uint32_t)atoi(b) : GMSK_DEFAULT_BAUD_RATE;
  ctx->bt               = (bt && atof(bt) > 0.0) ? (float)atof(bt)   : GMSK_DEFAULT_BT;
  ctx->modulation_index = 0.5f;
  ctx->channel_bandwidth_hz = 0u;
  ctx->gauss_len = 1; ctx->gauss_c[0] = 1.0f;
  mr_signal_gate_init(&ctx->gate, MR_GATE_SQUELCH_RATIO);
  return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) { free(raw); }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
  if (!raw || !key || !value) return 0;
  GmskCtx* ctx = (GmskCtx*)raw;
  if (strcmp(key, "baud_rate") == 0) {
    const int v = atoi(value);
    if (v > 0) { ctx->baud_rate = (uint32_t)v; ctx->needs_reconfigure = 1; }
    return 1;
  }
  if (strcmp(key, "bt") == 0) {
    const float v = (float)atof(value);
    if (v > 0.0f) { ctx->bt = v; ctx->needs_reconfigure = 1; }
    return 1;
  }
  if (strcmp(key, "modulation_index") == 0) {
    const float v = (float)atof(value);
    if (v > 0.0f) ctx->modulation_index = v;
    return 1;
  }
  if (strcmp(key, "channel_bandwidth_hz") == 0) {
    const int v = atoi(value);
    if (v >= 0) { ctx->channel_bandwidth_hz = (uint32_t)v; }
    return 1;
  }
  if (strcmp(key, "squelch_db") == 0) {
    const float db = (float)atof(value);
    ctx->gate.squelch_ratio = (db <= 0.0f) ? 0.0f : powf(10.0f, db / 10.0f);
    return 1;
  }
  return 0;
}

static const MrPluginMeta kMeta = {
  "gmsk_demod", "2.0.0", MR_PLUGIN_API_VERSION,
  "GMSK demodulator (Gaussian FIR fallback)"
, MR_PLUGIN_ROLE_DEMODULATOR
};
const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
  if (!raw || !iq || !num_pairs) return;
  GmskCtx* ctx = (GmskCtx*)raw;
  if (!sr) sr = 2048000;
  gmsk_reconfigure(ctx, sr);
  const float norm = 1.0f / 32768.0f;
  for (uint32_t n = 0; n < num_pairs; ++n) {
    const float is = (float)iq[n*2]   * norm;
    const float qs = (float)iq[n*2+1] * norm;
    const float cross = qs * ctx->prev_i - is * ctx->prev_q;
    const float dot   = is * ctx->prev_i + qs * ctx->prev_q;
    float angle = (dot != 0.0f || cross != 0.0f) ? atan2f(cross, dot) : 0.0f;
    ctx->prev_i = is; ctx->prev_q = qs;
    /* Gaussian FIR */
    ctx->gauss_d[ctx->gauss_pos] = angle;
    ctx->gauss_pos = (ctx->gauss_pos + 1) % ctx->gauss_len;
    float out = 0.0f;
    uint32_t bi = ctx->gauss_pos;
    for (uint32_t t = 0; t < ctx->gauss_len; ++t) {
      out += ctx->gauss_c[t] * ctx->gauss_d[bi];
      bi = (bi + 1) % ctx->gauss_len;
    }
    /* Update gate with per-sample energy (FM disc² as proxy for IQ power) */
    mr_signal_gate_update(&ctx->gate, out * out, MR_GATE_HOLD_SYMS);

    ctx->sym_val += out; ctx->sym_n++; ctx->sym_acc++;
    if (ctx->sym_acc >= ctx->samples_per_symbol) {
      if (!ctx->gate.gate_open) {
        emit_bits(ctx->bit_buf, &ctx->bit_count, emit_fn, user_data,
                  freq_hz, unix_ms, ctx->baud_rate, ctx->bt, ctx->channel_bandwidth_hz);
        ctx->bit_count = 0;
        memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
      } else {
        mr_push_bit(ctx->bit_buf, &ctx->bit_count, GMSK_MAX_BITS,
                    ctx->sym_val / (float)ctx->sym_n > 0.0f ? 1 : 0);
        if (ctx->bit_count >= GMSK_MAX_BITS)
          emit_bits(ctx->bit_buf, &ctx->bit_count, emit_fn, user_data,
                    freq_hz, unix_ms, ctx->baud_rate, ctx->bt, ctx->channel_bandwidth_hz);
      }
      ctx->sym_acc = 0; ctx->sym_val = 0.0f; ctx->sym_n = 0;
    }
  }
}
#endif /* MR_PLUGIN_HAS_LIQUID */
