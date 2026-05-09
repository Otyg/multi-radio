/**
 * vdes_asm_demod.c — EXPERIMENTAL VDES ASM demodulator sketch
 *
 * Role: MR_PLUGIN_ROLE_DEMODULATOR
 *
 * Scope:
 *   This is a scaffold for a dedicated VDES ASM pipeline and is intentionally
 *   lightweight. It does a coarse symbol-rate reduction and differential phase
 *   slicing to produce provisional dibits.
 *
 * NOT IMPLEMENTED (design TODO):
 *   - proper timing recovery and carrier recovery loop
 *   - standardized pi/4-DQPSK mapping validation
 *   - interleaving/FEC/descrambling chain per ITU-R M.2092 Annex ASM
 */

#include "mr_plugin_api.h"
#include "mr_bit_buf.h"
#include "mr_iir_prefilter.h"
#include "mr_signal_gate.h"

#include <complex.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#define VDES_MAX_BITS 2048u
#define VDES_MIN_BITS 16u
#define VDES_DEFAULT_BPS 28800u

typedef struct {
    uint32_t sample_rate_hz;
    uint32_t bit_rate_bps;
    uint32_t samples_per_symbol;
    uint32_t decim_counter;
    int      has_prev;
    float complex prev_sym;
    MrIirPrefilter pre;
    MrSignalGate gate;
    uint8_t  bit_buf[VDES_MAX_BITS / 8u + 1u];
    uint32_t bit_count;
} VdesAsmDemodCtx;

static int vdes_dbg(void) {
    static int v = -1;
    if (v < 0) {
        const char* e = getenv("MR_AIS_DEBUG");
        v = (e && e[0] != '0') ? 1 : 0;
    }
    return v;
}

#define VLOG(...) do { if (vdes_dbg()) fprintf(stderr, "[vdes_asm_demod] " __VA_ARGS__); } while (0)

static void emit_bits(VdesAsmDemodCtx* ctx, double freq_hz, uint64_t unix_ms,
                      MrEmitFn emit_fn, void* user_data) {
    char kv[224];
    snprintf(kv, sizeof(kv),
             "{\"signal_type\":\"VDES_ASM_DATA\","
             "\"modulation\":\"pi4_dqpsk_sketch\","
             "\"bit_rate_bps\":\"%u\","
             "\"bit_count\":\"%u\"}",
             ctx->bit_rate_bps, (ctx->bit_count / 8u) * 8u);
    mr_emit_bits(ctx->bit_buf, &ctx->bit_count, VDES_MIN_BITS,
                 VDES_MAX_BITS / 8u + 1u,
                 "VDES_ASM_DATA", kv, freq_hz, unix_ms, emit_fn, user_data);
}

static int reconfigure(VdesAsmDemodCtx* ctx, uint32_t sr) {
    if (sr == 0u) sr = 2048000u;
    if (ctx->sample_rate_hz == sr && ctx->samples_per_symbol != 0u) return 1;

    ctx->sample_rate_hz = sr;
    ctx->samples_per_symbol = sr / (ctx->bit_rate_bps / 2u);
    if (ctx->samples_per_symbol < 2u) ctx->samples_per_symbol = 2u;
    ctx->decim_counter = 0u;
    ctx->has_prev = 0;
    ctx->bit_count = 0u;
    memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
    mr_signal_gate_reset(&ctx->gate);
    mr_iir_prefilter_destroy(&ctx->pre);
    mr_iir_prefilter_create(&ctx->pre, 0.06f);

    VLOG("reconfigure sr=%u bps=%u sps=%u\n",
         ctx->sample_rate_hz, ctx->bit_rate_bps, ctx->samples_per_symbol);
    return 1;
}

static const MrPluginMeta kMeta = {
    "vdes_asm_demod",
    "0.1.0",
    MR_PLUGIN_API_VERSION,
    "EXPERIMENTAL: VDES ASM demod scaffold (pi/4-DQPSK sketch)",
    MR_PLUGIN_ROLE_DEMODULATOR
};

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

MrPluginCtx* mr_plugin_create(void) {
    VdesAsmDemodCtx* ctx = (VdesAsmDemodCtx*)calloc(1, sizeof(VdesAsmDemodCtx));
    if (!ctx) return NULL;
    ctx->bit_rate_bps = VDES_DEFAULT_BPS;
    mr_iir_prefilter_init(&ctx->pre);
    mr_signal_gate_init(&ctx->gate, MR_GATE_SQUELCH_RATIO);
    return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
    if (!raw) return;
    VdesAsmDemodCtx* ctx = (VdesAsmDemodCtx*)raw;
    mr_iir_prefilter_destroy(&ctx->pre);
    free(ctx);
}

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
    VdesAsmDemodCtx* ctx = (VdesAsmDemodCtx*)raw;
    if (!ctx || !key || !value) return 0;
    if (strcmp(key, "bit_rate_bps") == 0) {
        const int v = atoi(value);
        if (v > 1000) {
            ctx->bit_rate_bps = (uint32_t)v;
            ctx->sample_rate_hz = 0;
        }
        return 1;
    }
    return 0;
}

void mr_plugin_process_bits(MrPluginCtx* ctx,
                            const uint8_t* bit_bytes, uint32_t bit_count,
                            double freq_hz, uint64_t unix_ms,
                            const char* source_type,
                            MrEmitFn emit_fn, void* user_data) {
    (void)ctx; (void)bit_bytes; (void)bit_count;
    (void)freq_hz; (void)unix_ms; (void)source_type; (void)emit_fn; (void)user_data;
}

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sample_rate_hz, double center_freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
    VdesAsmDemodCtx* ctx = (VdesAsmDemodCtx*)raw;
    uint32_t i;
    if (!ctx || !iq || num_pairs == 0u) return;
    if (!reconfigure(ctx, sample_rate_hz)) return;

    for (i = 0; i < num_pairs; ++i) {
        const float in_re = (float)iq[i * 2u] / 32768.0f;
        const float in_im = (float)iq[i * 2u + 1u] / 32768.0f;
        float complex filt_out;
        float complex x;
        float energy;
        int falling;
        mr_iir_prefilter_execute(&ctx->pre, in_re + _Complex_I * in_im, &filt_out);
        x = filt_out;

        energy = crealf(x) * crealf(x) + cimagf(x) * cimagf(x);
        falling = mr_signal_gate_update(&ctx->gate, energy, MR_GATE_HOLD_SYMS);
        if (!ctx->gate.gate_open) {
            if (falling && ctx->bit_count >= VDES_MIN_BITS) {
                emit_bits(ctx, center_freq_hz, unix_ms, emit_fn, user_data);
            } else if (!falling) {
                ctx->bit_count = 0u;
                memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
            }
            continue;
        }

        if (++ctx->decim_counter < ctx->samples_per_symbol) continue;
        ctx->decim_counter = 0u;

        if (!ctx->has_prev) {
            ctx->prev_sym = x;
            ctx->has_prev = 1;
            continue;
        }

        {
            const float complex d = conjf(ctx->prev_sym) * x;
            const float phi = atan2f(cimagf(d), crealf(d));
            uint8_t b0 = 0u, b1 = 0u;

            /* Coarse differential phase slicing (sketch mapping). */
            if (phi >= -0.25f * (float)M_PI && phi < 0.25f * (float)M_PI) {
                b0 = 0u; b1 = 0u;
            } else if (phi >= 0.25f * (float)M_PI && phi < 0.75f * (float)M_PI) {
                b0 = 0u; b1 = 1u;
            } else if (phi >= -0.75f * (float)M_PI && phi < -0.25f * (float)M_PI) {
                b0 = 1u; b1 = 1u;
            } else {
                b0 = 1u; b1 = 0u;
            }

            mr_push_bit(ctx->bit_buf, &ctx->bit_count, VDES_MAX_BITS, b0);
            mr_push_bit(ctx->bit_buf, &ctx->bit_count, VDES_MAX_BITS, b1);
            ctx->prev_sym = x;
        }

        if (ctx->bit_count >= VDES_MAX_BITS) {
            emit_bits(ctx, center_freq_hz, unix_ms, emit_fn, user_data);
        }
    }
}
