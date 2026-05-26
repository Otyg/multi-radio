/**
 * dsc_afsk_demod.c - AFSK/2-FSK demodulator for VHF-DSC audio.
 *
 * Role: MR_PLUGIN_ROLE_DEMODULATOR
 *
 * Input convention:
 *   - mr_plugin_process_iq receives interleaved int16 pairs.
 *   - This plugin treats I as audio sample and ignores Q.
 *   - ReceiverWorker can feed NFM-demodulated PCM as I+j0 to this plugin.
 *
 * Output:
 *   - Emits packed bits as signal_type "FSK_DATA" for decoder chaining.
 *
 * Defaults (VHF-DSC):
 *   baud=1200, mark=1300 Hz, space=2100 Hz
 */

#include "mr_plugin_api.h"
#include "mr_signal_gate.h"
#include "mr_bit_buf.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DSC_AFSK_DEFAULT_BAUD      1200u
#define DSC_AFSK_DEFAULT_MARK_HZ   1300.0f
#define DSC_AFSK_DEFAULT_SPACE_HZ  2100.0f
#define DSC_AFSK_MAX_BITS          4096u
#define DSC_AFSK_MIN_BITS          80u
#define DSC_AFSK_EMIT_CHUNK_BITS   256u
#define DSC_AFSK_IDLE_HOLD_SYMS    8u

typedef struct {
  uint32_t baud_rate;
  float mark_hz;
  float space_hz;
  int bit1_is_mark;
  int needs_reconfigure;

  uint32_t sample_rate_hz;
  float phase_mark;
  float phase_space;
  float step_mark;
  float step_space;

  float lp_mark_i;
  float lp_mark_q;
  float lp_space_i;
  float lp_space_q;
  float lp_alpha;

  float samples_per_symbol;
  float sym_accum;
  float score_accum;
  uint32_t score_count;

  MrSignalGate gate;
  uint8_t bit_buf[DSC_AFSK_MAX_BITS / 8u + 1u];
  uint32_t bit_count;
} DscAfskCtx;

static const MrPluginMeta kMeta = {
    "dsc_afsk_demod",
    "0.1.0",
    MR_PLUGIN_API_VERSION,
    "DSC AFSK/2-FSK demodulator for NFM audio (I=audio, Q ignored)",
    MR_PLUGIN_ROLE_DEMODULATOR};

static void wrap_phase(float* phase) {
  if (*phase > (float)M_PI) *phase -= 2.0f * (float)M_PI;
  if (*phase < (float)-M_PI) *phase += 2.0f * (float)M_PI;
}

static void emit_bits(DscAfskCtx* ctx, MrEmitFn emit_fn, void* user_data,
                      double freq_hz, uint64_t unix_ms) {
  char kv[256];
  snprintf(kv, sizeof(kv),
           "{\"baud_rate\":\"%u\",\"mark_hz\":\"%.1f\",\"space_hz\":\"%.1f\","
           "\"bit_count\":\"%u\",\"demod\":\"dsc_afsk\"}",
           ctx->baud_rate,
           (double)ctx->mark_hz,
           (double)ctx->space_hz,
           ctx->bit_count);
  mr_emit_bits(ctx->bit_buf, &ctx->bit_count,
               DSC_AFSK_MIN_BITS,
               DSC_AFSK_MAX_BITS / 8u + 1u,
               "FSK_DATA",
               kv,
               freq_hz, unix_ms,
               emit_fn, user_data);
}

static int configure(DscAfskCtx* ctx, uint32_t sample_rate_hz) {
  if (!ctx) return 0;
  if (sample_rate_hz == 0) sample_rate_hz = 12000u;
  if (ctx->sample_rate_hz == sample_rate_hz && !ctx->needs_reconfigure) return 1;

  ctx->sample_rate_hz = sample_rate_hz;
  ctx->needs_reconfigure = 0;

  ctx->phase_mark = 0.0f;
  ctx->phase_space = 0.0f;
  ctx->step_mark = 2.0f * (float)M_PI * ctx->mark_hz / (float)ctx->sample_rate_hz;
  ctx->step_space = 2.0f * (float)M_PI * ctx->space_hz / (float)ctx->sample_rate_hz;

  ctx->lp_mark_i = ctx->lp_mark_q = 0.0f;
  ctx->lp_space_i = ctx->lp_space_q = 0.0f;
  ctx->samples_per_symbol = (float)ctx->sample_rate_hz / (float)ctx->baud_rate;
  if (ctx->samples_per_symbol < 1.0f) ctx->samples_per_symbol = 1.0f;
  ctx->sym_accum = 0.0f;
  ctx->score_accum = 0.0f;
  ctx->score_count = 0u;

  /* One-pole low-pass after tone-mixing; cutoff around baud keeps symbol energy. */
  {
    const float cutoff_hz = fmaxf(80.0f, 0.8f * (float)ctx->baud_rate);
    const float x = 2.0f * (float)M_PI * cutoff_hz / (float)ctx->sample_rate_hz;
    ctx->lp_alpha = expf(-x);
  }

  mr_signal_gate_reset(&ctx->gate);
  ctx->bit_count = 0u;
  memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
  return 1;
}

MrPluginCtx* mr_plugin_create(void) {
  DscAfskCtx* ctx = (DscAfskCtx*)calloc(1, sizeof(DscAfskCtx));
  if (!ctx) return NULL;

  {
    const char* v = getenv("MR_DSC_BAUD_RATE");
    ctx->baud_rate = (v && atoi(v) > 0) ? (uint32_t)atoi(v) : DSC_AFSK_DEFAULT_BAUD;
  }
  {
    const char* v = getenv("MR_DSC_MARK_HZ");
    ctx->mark_hz = (v && atof(v) > 0.0) ? (float)atof(v) : DSC_AFSK_DEFAULT_MARK_HZ;
  }
  {
    const char* v = getenv("MR_DSC_SPACE_HZ");
    ctx->space_hz = (v && atof(v) > 0.0) ? (float)atof(v) : DSC_AFSK_DEFAULT_SPACE_HZ;
  }
  {
    const char* v = getenv("MR_DSC_BIT1_IS_MARK");
    ctx->bit1_is_mark = (v && atoi(v) == 0) ? 0 : 1;
  }

  mr_signal_gate_init(&ctx->gate, MR_GATE_SQUELCH_RATIO);
  ctx->needs_reconfigure = 1;
  return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
  free(raw);
}

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
  DscAfskCtx* ctx = (DscAfskCtx*)raw;
  if (!ctx || !key || !value) return 0;

  if (strcmp(key, "baud_rate") == 0) {
    const int v = atoi(value);
    if (v > 0) {
      ctx->baud_rate = (uint32_t)v;
      ctx->needs_reconfigure = 1;
    }
    return 1;
  }
  if (strcmp(key, "mark_hz") == 0) {
    const float v = (float)atof(value);
    if (v > 0.0f) {
      ctx->mark_hz = v;
      ctx->needs_reconfigure = 1;
    }
    return 1;
  }
  if (strcmp(key, "space_hz") == 0) {
    const float v = (float)atof(value);
    if (v > 0.0f) {
      ctx->space_hz = v;
      ctx->needs_reconfigure = 1;
    }
    return 1;
  }
  if (strcmp(key, "bit1_is_mark") == 0) {
    ctx->bit1_is_mark = atoi(value) ? 1 : 0;
    return 1;
  }
  if (strcmp(key, "squelch_db") == 0) {
    const float db = (float)atof(value);
    ctx->gate.squelch_ratio = powf(10.0f, db / 10.0f);
    if (ctx->gate.squelch_ratio < 1.0e-6f) ctx->gate.squelch_ratio = 1.0e-6f;
    return 1;
  }
  return 0;
}

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sample_rate_hz,
                          double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
  DscAfskCtx* ctx = (DscAfskCtx*)raw;
  if (!ctx || !iq || num_pairs == 0u) return;
  if (!configure(ctx, sample_rate_hz)) return;

  for (uint32_t i = 0; i < num_pairs; ++i) {
    const float x = (float)iq[i * 2u] / 32768.0f;  /* I = audio */
    const float e = x * x;

    const int falling = mr_signal_gate_update(&ctx->gate, e, DSC_AFSK_IDLE_HOLD_SYMS);
    if (!ctx->gate.gate_open) {
      if (falling) emit_bits(ctx, emit_fn, user_data, freq_hz, unix_ms);
      continue;
    }

    const float cm = cosf(ctx->phase_mark), sm = sinf(ctx->phase_mark);
    const float cs = cosf(ctx->phase_space), ss = sinf(ctx->phase_space);

    const float mix_m_i = x * cm;
    const float mix_m_q = -x * sm;
    const float mix_s_i = x * cs;
    const float mix_s_q = -x * ss;

    ctx->lp_mark_i = ctx->lp_alpha * ctx->lp_mark_i + (1.0f - ctx->lp_alpha) * mix_m_i;
    ctx->lp_mark_q = ctx->lp_alpha * ctx->lp_mark_q + (1.0f - ctx->lp_alpha) * mix_m_q;
    ctx->lp_space_i = ctx->lp_alpha * ctx->lp_space_i + (1.0f - ctx->lp_alpha) * mix_s_i;
    ctx->lp_space_q = ctx->lp_alpha * ctx->lp_space_q + (1.0f - ctx->lp_alpha) * mix_s_q;

    ctx->phase_mark += ctx->step_mark;
    ctx->phase_space += ctx->step_space;
    wrap_phase(&ctx->phase_mark);
    wrap_phase(&ctx->phase_space);

    {
      const float em = ctx->lp_mark_i * ctx->lp_mark_i + ctx->lp_mark_q * ctx->lp_mark_q;
      const float es = ctx->lp_space_i * ctx->lp_space_i + ctx->lp_space_q * ctx->lp_space_q;
      const float score = em - es;

      ctx->sym_accum += 1.0f;
      ctx->score_accum += score;
      ctx->score_count += 1u;
    }

    if (ctx->sym_accum >= ctx->samples_per_symbol) {
      const float avg = (ctx->score_count > 0u) ? (ctx->score_accum / (float)ctx->score_count) : 0.0f;
      unsigned int bit;
      if (avg >= 0.0f) bit = ctx->bit1_is_mark ? 1u : 0u;
      else             bit = ctx->bit1_is_mark ? 0u : 1u;
      mr_push_bit(ctx->bit_buf, &ctx->bit_count, DSC_AFSK_MAX_BITS, bit);
      ctx->sym_accum -= ctx->samples_per_symbol;
      ctx->score_accum = 0.0f;
      ctx->score_count = 0u;
      if (ctx->bit_count >= DSC_AFSK_EMIT_CHUNK_BITS) {
        emit_bits(ctx, emit_fn, user_data, freq_hz, unix_ms);
      }
    }
  }
}

void mr_plugin_process_bits(MrPluginCtx* raw,
                            const uint8_t* bit_bytes, uint32_t bit_count,
                            double freq_hz, uint64_t unix_ms,
                            const char* source_type,
                            MrEmitFn emit_fn, void* user_data) {
  (void)raw;
  (void)bit_bytes;
  (void)bit_count;
  (void)freq_hz;
  (void)unix_ms;
  (void)source_type;
  (void)emit_fn;
  (void)user_data;
}
