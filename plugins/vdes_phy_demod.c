/**
 * vdes_phy_demod.c - first-pass VDES physical-layer diagnostics
 *
 * Role: MR_PLUGIN_ROLE_DEMODULATOR
 *
 * This is intentionally not a frame decoder yet. It consumes raw IQ and emits
 * lightweight diagnostic messages that make recorded/live signals measurable:
 * level, clipping, DC offset, coarse carrier offset, and energy gate state.
 */

#include "mr_plugin_api.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define VDES_PHY_DEFAULT_DIAG_INTERVAL_BLOCKS 20u
#define VDES_PHY_DEFAULT_SQUELCH_DB 10.0f
#define VDES_PHY_NOISE_ALPHA 0.02f
#define VDES_PHY_SIGNAL_ALPHA 0.15f
#define VDES_PHY_CLIP_S16 32256

typedef struct {
  uint32_t sample_rate_hz;
  uint32_t diag_interval_blocks;
  float squelch_db;
  float squelch_ratio;

  uint64_t blocks_seen;
  uint64_t gate_opens;
  uint64_t gate_closes;
  int gate_open;
  uint32_t hold_blocks;

  float noise_floor;
  float signal_level;
  int have_prev;
  float prev_i;
  float prev_q;
} VdesPhyCtx;

static const MrPluginMeta kMeta = {
  "vdes_phy_demod",
  "0.1.0",
  MR_PLUGIN_API_VERSION,
  "VDES physical-layer diagnostics (no frame decode)",
  MR_PLUGIN_ROLE_DEMODULATOR
};

static float db_from_power(float p) {
  if (p < 1e-12f) p = 1e-12f;
  return 10.0f * log10f(p);
}

static void update_squelch(VdesPhyCtx* ctx) {
  ctx->squelch_ratio = powf(10.0f, ctx->squelch_db / 10.0f);
  if (ctx->squelch_ratio < 1.0f) ctx->squelch_ratio = 1.0f;
}

static void emit_diag(VdesPhyCtx* ctx,
                      double freq_hz,
                      uint64_t unix_ms,
                      MrEmitFn emit_fn,
                      void* user_data,
                      const char* reason,
                      uint32_t num_pairs,
                      float rms_dbfs,
                      float peak_dbfs,
                      float clip_pct,
                      float dc_i,
                      float dc_q,
                      float carrier_offset_hz,
                      float snr_est_db) {
  char payload[256];
  char kv[768];

  if (!emit_fn) return;
  snprintf(payload, sizeof(payload),
           "VDES PHY %s level=%.1f dBFS snr=%.1f dB carrier=%.1f Hz gate=%d",
           reason ? reason : "diag",
           (double)rms_dbfs,
           (double)snr_est_db,
           (double)carrier_offset_hz,
           ctx->gate_open);
  snprintf(kv, sizeof(kv),
           "{\"signal_type\":\"VDES_PHY_DIAG\","
           "\"diag_kind\":\"phy\","
           "\"reason\":\"%s\","
           "\"sample_rate_hz\":\"%u\","
           "\"block_iq_pairs\":\"%u\","
           "\"blocks_seen\":\"%llu\","
           "\"gate_open\":\"%d\","
           "\"gate_opens\":\"%llu\","
           "\"gate_closes\":\"%llu\","
           "\"rms_dbfs\":\"%.3f\","
           "\"peak_dbfs\":\"%.3f\","
           "\"clip_pct\":\"%.4f\","
           "\"dc_i\":\"%.7f\","
           "\"dc_q\":\"%.7f\","
           "\"noise_floor_db\":\"%.3f\","
           "\"signal_level_db\":\"%.3f\","
           "\"snr_est_db\":\"%.3f\","
           "\"carrier_offset_hz\":\"%.3f\"}",
           reason ? reason : "diag",
           ctx->sample_rate_hz,
           num_pairs,
           (unsigned long long)ctx->blocks_seen,
           ctx->gate_open,
           (unsigned long long)ctx->gate_opens,
           (unsigned long long)ctx->gate_closes,
           (double)rms_dbfs,
           (double)peak_dbfs,
           (double)clip_pct,
           (double)dc_i,
           (double)dc_q,
           (double)db_from_power(ctx->noise_floor),
           (double)db_from_power(ctx->signal_level),
           (double)snr_est_db,
           (double)carrier_offset_hz);
  emit_fn("VDES_PHY_DIAG", payload, freq_hz, unix_ms, kv, user_data);
}

MrPluginCtx* mr_plugin_create(void) {
  VdesPhyCtx* ctx = (VdesPhyCtx*)calloc(1, sizeof(VdesPhyCtx));
  const char* interval_env;
  const char* squelch_env;
  if (!ctx) return NULL;

  ctx->diag_interval_blocks = VDES_PHY_DEFAULT_DIAG_INTERVAL_BLOCKS;
  ctx->squelch_db = VDES_PHY_DEFAULT_SQUELCH_DB;
  interval_env = getenv("MR_VDES_PHY_DIAG_INTERVAL_BLOCKS");
  squelch_env = getenv("MR_VDES_PHY_SQUELCH_DB");
  if (interval_env && atoi(interval_env) >= 0) {
    ctx->diag_interval_blocks = (uint32_t)atoi(interval_env);
  }
  if (squelch_env && atof(squelch_env) >= 0.0) {
    ctx->squelch_db = (float)atof(squelch_env);
  }
  update_squelch(ctx);
  return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
  free(raw);
}

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
  VdesPhyCtx* ctx = (VdesPhyCtx*)raw;
  if (!ctx || !key || !value) return 0;

  if (strcmp(key, "diag_interval_blocks") == 0) {
    const int v = atoi(value);
    if (v >= 0) ctx->diag_interval_blocks = (uint32_t)v;
    return 1;
  }
  if (strcmp(key, "squelch_db") == 0) {
    const float v = (float)atof(value);
    if (v >= 0.0f && v <= 80.0f) {
      ctx->squelch_db = v;
      update_squelch(ctx);
    }
    return 1;
  }
  return 0;
}

void mr_plugin_process_bits(MrPluginCtx* ctx,
                            const uint8_t* bit_bytes,
                            uint32_t bit_count,
                            double freq_hz,
                            uint64_t unix_ms,
                            const char* source_type,
                            MrEmitFn emit_fn,
                            void* user_data) {
  (void)ctx;
  (void)bit_bytes;
  (void)bit_count;
  (void)freq_hz;
  (void)unix_ms;
  (void)source_type;
  (void)emit_fn;
  (void)user_data;
}

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq,
                          uint32_t num_pairs,
                          uint32_t sample_rate_hz,
                          double center_freq_hz,
                          uint64_t unix_ms,
                          MrEmitFn emit_fn,
                          void* user_data) {
  VdesPhyCtx* ctx = (VdesPhyCtx*)raw;
  double sum_i = 0.0;
  double sum_q = 0.0;
  double power_sum = 0.0;
  double peak_power = 0.0;
  double phase_sum_i = 0.0;
  double phase_sum_q = 0.0;
  uint64_t clipped = 0;
  float rms_power;
  float peak_dbfs;
  float rms_dbfs;
  float dc_i;
  float dc_q;
  float carrier_offset_hz = 0.0f;
  float snr_est_db = 0.0f;
  int prev_gate;
  const float norm = 1.0f / 32768.0f;

  if (!ctx || !iq || num_pairs == 0) return;
  if (sample_rate_hz == 0) sample_rate_hz = 2048000u;
  ctx->sample_rate_hz = sample_rate_hz;

  for (uint32_t n = 0; n < num_pairs; ++n) {
    const int16_t si = iq[n * 2u];
    const int16_t sq = iq[n * 2u + 1u];
    const float ii = (float)si * norm;
    const float qq = (float)sq * norm;
    const double p = (double)ii * (double)ii + (double)qq * (double)qq;

    sum_i += ii;
    sum_q += qq;
    power_sum += p;
    if (p > peak_power) peak_power = p;
    if (si >= VDES_PHY_CLIP_S16 || si <= -VDES_PHY_CLIP_S16 ||
        sq >= VDES_PHY_CLIP_S16 || sq <= -VDES_PHY_CLIP_S16) {
      clipped++;
    }

    if (ctx->have_prev) {
      phase_sum_i += (double)ctx->prev_i * (double)ii + (double)ctx->prev_q * (double)qq;
      phase_sum_q += (double)ctx->prev_i * (double)qq - (double)ctx->prev_q * (double)ii;
    }
    ctx->prev_i = ii;
    ctx->prev_q = qq;
    ctx->have_prev = 1;
  }

  rms_power = (float)(power_sum / (double)num_pairs);
  rms_dbfs = db_from_power(rms_power);
  peak_dbfs = db_from_power((float)peak_power);
  dc_i = (float)(sum_i / (double)num_pairs);
  dc_q = (float)(sum_q / (double)num_pairs);

  if (phase_sum_i != 0.0 || phase_sum_q != 0.0) {
    const double phase = atan2(phase_sum_q, phase_sum_i);
    carrier_offset_hz = (float)(phase * (double)sample_rate_hz / (2.0 * M_PI));
  }

  if (ctx->noise_floor <= 0.0f) ctx->noise_floor = rms_power;
  if (ctx->signal_level <= 0.0f) ctx->signal_level = rms_power;

  prev_gate = ctx->gate_open;
  if (!ctx->gate_open) {
    ctx->noise_floor =
        (1.0f - VDES_PHY_NOISE_ALPHA) * ctx->noise_floor + VDES_PHY_NOISE_ALPHA * rms_power;
    if (rms_power > ctx->noise_floor * ctx->squelch_ratio) {
      ctx->gate_open = 1;
      ctx->hold_blocks = 6u;
      ctx->gate_opens++;
    }
  } else {
    ctx->signal_level =
        (1.0f - VDES_PHY_SIGNAL_ALPHA) * ctx->signal_level + VDES_PHY_SIGNAL_ALPHA * rms_power;
    if (rms_power > ctx->noise_floor * (ctx->squelch_ratio * 0.5f)) {
      ctx->hold_blocks = 6u;
    } else if (ctx->hold_blocks > 0) {
      ctx->hold_blocks--;
    } else {
      ctx->gate_open = 0;
      ctx->gate_closes++;
    }
  }

  if (ctx->noise_floor > 0.0f) {
    snr_est_db = db_from_power(rms_power) - db_from_power(ctx->noise_floor);
  }

  ctx->blocks_seen++;

  if (ctx->gate_open != prev_gate) {
    emit_diag(ctx, center_freq_hz, unix_ms, emit_fn, user_data,
              ctx->gate_open ? "gate-open" : "gate-close",
              num_pairs, rms_dbfs, peak_dbfs,
              100.0f * (float)clipped / (float)num_pairs,
              dc_i, dc_q, carrier_offset_hz, snr_est_db);
    return;
  }

  if (ctx->diag_interval_blocks > 0 &&
      (ctx->blocks_seen % ctx->diag_interval_blocks) == 0) {
    emit_diag(ctx, center_freq_hz, unix_ms, emit_fn, user_data,
              "periodic",
              num_pairs, rms_dbfs, peak_dbfs,
              100.0f * (float)clipped / (float)num_pairs,
              dc_i, dc_q, carrier_offset_hz, snr_est_db);
  }
}
