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

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GMSK_DEFAULT_BAUD_RATE 9600
#define GMSK_DEFAULT_BT        0.3f
#define GMSK_K                 8     /* samples per symbol */
#define GMSK_M                 3     /* filter delay in symbols */
#define GMSK_MAX_BITS          1024
#define GMSK_MIN_BITS          8
#define GMSK_IDLE_GAP_SYMS     8

/* Signal gate constants */
#define GMSK_SQUELCH_RATIO  4.0f   /* default: signal must be 6 dB above noise floor */
#define GMSK_HOLD_SYMS      48     /* keep gate open N symbols after energy drops */
#define GMSK_NOISE_ALPHA    0.001f /* noise floor EMA alpha (~1000 sym time constant) */
#define GMSK_ENERGY_ALPHA   0.10f  /* signal energy EMA alpha (~10 sym time constant) */
#define GMSK_NOISE_INIT     1e-4f  /* initial noise floor estimate */

/* AFC (Automatic Frequency Correction) */
#define GMSK_AFC_ALPHA      0.005f /* frequency error EMA alpha (~200 sym time constant) */
#define GMSK_AFC_MAX_RAD    0.30f  /* max correction in rad/sample of resampled signal */

/* ------------------------------------------------------------------ */
/* Shared bit-buffer helpers                                            */
/* ------------------------------------------------------------------ */

static void push_bit(uint8_t* buf, uint32_t* count, unsigned int bit) {
  if (*count >= GMSK_MAX_BITS) return;
  const uint32_t byte_idx = *count / 8;
  const uint32_t bit_idx  = 7 - (*count % 8);
  if (bit) buf[byte_idx] |=  (uint8_t)(1u << bit_idx);
  else     buf[byte_idx] &= (uint8_t)~(1u << bit_idx);
  ++(*count);
}

static void emit_bits(uint8_t* bit_buf, uint32_t* bit_count,
                      MrEmitFn emit_fn, void* user_data,
                      double freq_hz, uint64_t unix_ms,
                      unsigned int baud_rate, float bt) {
  if (*bit_count < (uint32_t)GMSK_MIN_BITS) { *bit_count = 0; return; }
  const uint32_t byte_count = *bit_count / 8;
  char* hex = (char*)malloc(byte_count * 2 + 1);
  if (!hex) { *bit_count = 0; return; }
  for (uint32_t i = 0; i < byte_count; ++i)
    snprintf(hex + i * 2, 3, "%02X", (unsigned)bit_buf[i]);
  char kv[160];
  snprintf(kv, sizeof(kv),
           "{\"baud_rate\":\"%u\",\"bt\":\"%.3f\",\"bit_count\":\"%u\"}",
           baud_rate, (double)bt, byte_count * 8);
  if (emit_fn) emit_fn("GMSK_DATA", hex, freq_hz, unix_ms, kv, user_data);
  free(hex);
  *bit_count = 0;
  memset(bit_buf, 0, GMSK_MAX_BITS / 8 + 1);
}

/* ================================================================== */
/* libliquid path                                                       */
/* ================================================================== */

#if defined(MR_PLUGIN_HAS_LIQUID) && MR_PLUGIN_HAS_LIQUID
#include <liquid/liquid.h>

typedef struct {
  uint32_t baud_rate;
  float    bt;
  float    modulation_index;  /* informational; affects discriminator scaling in fallback */
  int      needs_reconfigure; /* set to 1 by set_param to trigger teardown on next call */

  uint32_t sample_rate_hz;

  msresamp_crcf resampler;
  gmskdem       demodulator;

  liquid_float_complex* sym_buf;
  uint32_t sym_buf_fill;

  liquid_float_complex* resamp_out;
  uint32_t resamp_out_cap;

  uint32_t idle_sym_count;

  /* Signal gate */
  float    noise_floor;    /* slow EMA of per-sample energy during quiet periods */
  float    signal_energy;  /* fast EMA of per-sample energy */
  float    squelch_ratio;  /* gate opens when signal_energy > noise_floor * ratio */
  uint32_t hold_syms;      /* symbols remaining in hold-off after energy drops */
  int      gate_open;      /* 1 = demodulating, 0 = gated off */

  /* AFC: carrier frequency offset correction */
  float    nco_phase;      /* current NCO phase accumulator, radians */
  float    nco_freq;       /* current frequency correction, rad/sample (resampled) */
  float    freq_est;       /* slow EMA of frequency error estimate */

  uint8_t  bit_buf[GMSK_MAX_BITS / 8 + 1];
  uint32_t bit_count;
} GmskCtx;

static void gmsk_teardown(GmskCtx* ctx) {
  if (ctx->resampler)    { msresamp_crcf_destroy(ctx->resampler);  ctx->resampler   = NULL; }
  if (ctx->demodulator)  { gmskdem_destroy(ctx->demodulator);       ctx->demodulator = NULL; }
  free(ctx->sym_buf);    ctx->sym_buf     = NULL;
  free(ctx->resamp_out); ctx->resamp_out  = NULL;
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
  ctx->idle_sym_count = 0;
  ctx->signal_energy  = 0.0f;
  ctx->noise_floor    = GMSK_NOISE_INIT;
  ctx->hold_syms      = 0;
  ctx->gate_open      = 0;
  ctx->nco_phase      = 0.0f;
  ctx->nco_freq       = 0.0f;
  ctx->freq_est       = 0.0f;

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

  ctx->idle_sym_count = 0;
  ctx->sym_buf_fill   = 0;
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
  ctx->noise_floor      = GMSK_NOISE_INIT;
  ctx->squelch_ratio    = GMSK_SQUELCH_RATIO;
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
  if (strcmp(key, "squelch_db") == 0) {
    /* Convert dB to linear ratio: e.g. "6" → 4.0, "10" → 10.0, "0" disables gate */
    const float db = (float)atof(value);
    ctx->squelch_ratio = (db <= 0.0f) ? 0.0f : powf(10.0f, db / 10.0f);
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
    __real__ in_buf[n] = (float)iq[n * 2]     * norm;
    __imag__ in_buf[n] = (float)iq[n * 2 + 1] * norm;
  }

  unsigned int n_out = 0;
  msresamp_crcf_execute(ctx->resampler, in_buf, num_pairs, ctx->resamp_out, &n_out);
  free(in_buf);

  for (unsigned int i = 0; i < n_out; ++i) {
    /* Apply NCO frequency correction before storing sample */
    {
      const float cp = cosf(ctx->nco_phase);
      const float sp = sinf(ctx->nco_phase);
      const float re = __real__ ctx->resamp_out[i];
      const float im = __imag__ ctx->resamp_out[i];
      __real__ ctx->sym_buf[ctx->sym_buf_fill] = re * cp + im * sp;
      __imag__ ctx->sym_buf[ctx->sym_buf_fill] = im * cp - re * sp;
      ctx->nco_phase += ctx->nco_freq;
      if (ctx->nco_phase >  3.14159265f) ctx->nco_phase -= 6.28318530f;
      if (ctx->nco_phase < -3.14159265f) ctx->nco_phase += 6.28318530f;
    }
    ctx->sym_buf_fill++;
    if (ctx->sym_buf_fill < (uint32_t)GMSK_K) continue;
    ctx->sym_buf_fill = 0;

    float energy = 0.0f;
    for (int s = 0; s < GMSK_K; ++s) {
      const float re = __real__ ctx->sym_buf[s];
      const float im = __imag__ ctx->sym_buf[s];
      energy += re * re + im * im;
    }
    const float per_sample_energy = energy / (float)GMSK_K;

    /* Noise floor EMA: only update during quiet periods to avoid signal inflating it */
    if (per_sample_energy < ctx->noise_floor * ctx->squelch_ratio) {
      ctx->noise_floor = ctx->noise_floor * (1.0f - GMSK_NOISE_ALPHA)
                       + per_sample_energy * GMSK_NOISE_ALPHA;
    }
    if (ctx->noise_floor < 1e-12f) ctx->noise_floor = 1e-12f;

    /* Fast signal energy EMA */
    ctx->signal_energy = ctx->signal_energy * (1.0f - GMSK_ENERGY_ALPHA)
                       + per_sample_energy * GMSK_ENERGY_ALPHA;

    /* Gate: open when signal energy exceeds noise floor by squelch_ratio */
    const int above = (ctx->squelch_ratio > 0.0f)
                    && (ctx->signal_energy > ctx->noise_floor * ctx->squelch_ratio);
    if (above) {
      ctx->hold_syms = GMSK_HOLD_SYMS;
    } else if (ctx->hold_syms > 0) {
      ctx->hold_syms--;
    }
    const int prev_gate = ctx->gate_open;
    ctx->gate_open = above | (ctx->hold_syms > 0);

    if (!ctx->gate_open) {
      /* Gate closed: flush on falling edge, then discard */
      if (prev_gate && ctx->bit_count >= GMSK_MIN_BITS) {
        emit_bits(ctx->bit_buf, &ctx->bit_count, emit_fn, user_data,
                  freq_hz, unix_ms, ctx->baud_rate, ctx->bt);
      }
      ctx->bit_count = 0;
      memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
      continue;
    }

    /* Gate open: update AFC from FM discriminator mean over this symbol */
    {
      float fm_sum = 0.0f;
      for (int s = 1; s < GMSK_K; ++s) {
        const float re0 = __real__ ctx->sym_buf[s - 1];
        const float im0 = __imag__ ctx->sym_buf[s - 1];
        const float re1 = __real__ ctx->sym_buf[s];
        const float im1 = __imag__ ctx->sym_buf[s];
        /* cross / |z1||z0| = sin(Δphase) ≈ Δphase (instantaneous frequency) */
        const float cross = im1 * re0 - re1 * im0;
        const float dot   = re1 * re0 + im1 * im0;
        const float mag2  = cross * cross + dot * dot;
        fm_sum += (mag2 > 1e-12f) ? (cross / sqrtf(mag2)) : 0.0f;
      }
      const float fm_mean = fm_sum / (float)(GMSK_K - 1);
      /* Integrate: mean FM output = carrier frequency offset */
      ctx->freq_est = ctx->freq_est * (1.0f - GMSK_AFC_ALPHA) + fm_mean * GMSK_AFC_ALPHA;
      ctx->nco_freq = -ctx->freq_est;
      if (ctx->nco_freq >  GMSK_AFC_MAX_RAD) ctx->nco_freq =  GMSK_AFC_MAX_RAD;
      if (ctx->nco_freq < -GMSK_AFC_MAX_RAD) ctx->nco_freq = -GMSK_AFC_MAX_RAD;
    }

    /* Demodulate */
    unsigned int bit = 0;
    gmskdem_demodulate(ctx->demodulator, ctx->sym_buf, &bit);
    push_bit(ctx->bit_buf, &ctx->bit_count, bit & 1u);

    if (ctx->bit_count >= GMSK_MAX_BITS)
      emit_bits(ctx->bit_buf, &ctx->bit_count, emit_fn, user_data,
                freq_hz, unix_ms, ctx->baud_rate, ctx->bt);
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
  int      needs_reconfigure;
  float prev_i, prev_q;
  float    gauss_c[GMSK_MAX_FILTER_TAPS];
  float    gauss_d[GMSK_MAX_FILTER_TAPS];
  uint32_t gauss_len, gauss_pos;
  uint32_t samples_per_symbol, sample_rate_hz;
  uint32_t sym_acc;
  float    sym_val;
  uint32_t sym_n, idle_samples;
  /* Signal gate */
  float    noise_floor;
  float    signal_energy;
  float    squelch_ratio;
  uint32_t hold_syms;
  int      gate_open;
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
  ctx->gauss_len     = 1; ctx->gauss_c[0] = 1.0f;
  ctx->noise_floor   = GMSK_NOISE_INIT;
  ctx->squelch_ratio = GMSK_SQUELCH_RATIO;
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
  if (strcmp(key, "squelch_db") == 0) {
    const float db = (float)atof(value);
    ctx->squelch_ratio = (db <= 0.0f) ? 0.0f : powf(10.0f, db / 10.0f);
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
    /* Per-sample energy for gate (use squared discriminator output as proxy) */
    const float per_sample_energy = out * out;
    if (per_sample_energy < ctx->noise_floor * ctx->squelch_ratio) {
      ctx->noise_floor = ctx->noise_floor * (1.0f - GMSK_NOISE_ALPHA)
                       + per_sample_energy * GMSK_NOISE_ALPHA;
    }
    if (ctx->noise_floor < 1e-12f) ctx->noise_floor = 1e-12f;
    ctx->signal_energy = ctx->signal_energy * (1.0f - GMSK_ENERGY_ALPHA)
                       + per_sample_energy * GMSK_ENERGY_ALPHA;

    ctx->sym_val += out; ctx->sym_n++; ctx->sym_acc++;
    if (ctx->sym_acc >= ctx->samples_per_symbol) {
      /* Evaluate gate once per symbol */
      const int above = (ctx->squelch_ratio > 0.0f)
                      && (ctx->signal_energy > ctx->noise_floor * ctx->squelch_ratio);
      if (above) {
        ctx->hold_syms = GMSK_HOLD_SYMS;
      } else if (ctx->hold_syms > 0) {
        ctx->hold_syms--;
      }
      const int prev_gate = ctx->gate_open;
      ctx->gate_open = above | (ctx->hold_syms > 0);

      if (!ctx->gate_open) {
        if (prev_gate && ctx->bit_count >= GMSK_MIN_BITS) {
          emit_bits(ctx->bit_buf, &ctx->bit_count, emit_fn, user_data,
                    freq_hz, unix_ms, ctx->baud_rate, ctx->bt);
        }
        ctx->bit_count = 0;
        memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
      } else {
        push_bit(ctx->bit_buf, &ctx->bit_count,
                 ctx->sym_val / (float)ctx->sym_n > 0.0f ? 1 : 0);
        if (ctx->bit_count >= GMSK_MAX_BITS)
          emit_bits(ctx->bit_buf, &ctx->bit_count, emit_fn, user_data,
                    freq_hz, unix_ms, ctx->baud_rate, ctx->bt);
      }
      ctx->sym_acc = 0; ctx->sym_val = 0.0f; ctx->sym_n = 0;
    }
  }
}
#endif /* MR_PLUGIN_HAS_LIQUID */
