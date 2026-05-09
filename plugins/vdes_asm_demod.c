/**
 * vdes_asm_demod.c — EXPERIMENTAL VDES ASM demodulator (libliquid)
 *
 * Role: MR_PLUGIN_ROLE_DEMODULATOR
 *
 * This implementation is a practical first step beyond the earlier sketch:
 *   IQ -> prefilter -> msresamp -> AGC -> symbol sync -> diff pi/4-DQPSK slicer
 *
 * Remaining TODO for full standard compliance:
 *   - robust burst acquisition and framing
 *   - full VDES ASM interleaving/scrambling/FEC chain
 */

#include "mr_plugin_api.h"
#include "mr_bit_buf.h"
#include "mr_iir_prefilter.h"
#include "mr_signal_gate.h"

#include <liquid/liquid.h>

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#define VDES_MAX_BITS         4096u
#define VDES_MIN_BITS         32u
#define VDES_DEFAULT_BITRATE  28800u
#define VDES_DIFF_M           5u
#define VDES_DIFF_BETA        0.35f
#define VDES_DIFF_K           4u
#define VDES_PLL_BW_DEFAULT   0.01f

typedef struct {
    uint32_t sample_rate_hz;
    uint32_t bit_rate_bps;
    uint32_t sym_rate_baud;
    uint32_t k;
    float    pll_bw;

    int needs_reconfigure;

    MrIirPrefilter pre;
    MrSignalGate   gate;

    msresamp_crcf resamp;
    symsync_crcf  symsync;
    agc_crcf      agc;
    nco_crcf      pll;

    liquid_float_complex* in_buf;
    uint32_t in_cap;

    liquid_float_complex* resamp_out;
    uint32_t resamp_cap;

    liquid_float_complex* sym_out;
    uint32_t sym_out_cap;

    uint8_t  bit_buf[VDES_MAX_BITS / 8u + 1u];
    uint32_t bit_count;

    int gate_prev_open;
    int have_prev_sym;
    liquid_float_complex prev_sym;

    uint64_t blocks;
} VdesAsmCtx;

static int vdbg(void) {
    static int v = -1;
    if (v < 0) {
        const char* e = getenv("MR_AIS_DEBUG");
        v = (e && e[0] != '0') ? 1 : 0;
    }
    return v;
}
#define VLOG(...) do { if (vdbg()) fprintf(stderr, "[vdes_asm_demod] " __VA_ARGS__); } while (0)

static float wrap_pi(float x) {
    while (x >  (float)M_PI) x -= 2.0f * (float)M_PI;
    while (x < -(float)M_PI) x += 2.0f * (float)M_PI;
    return x;
}

static float nearest_pi4_dqpsk_phase(float phi) {
    const float p1 =  0.25f * (float)M_PI;
    const float p2 =  0.75f * (float)M_PI;
    const float p3 = -0.75f * (float)M_PI;
    const float p4 = -0.25f * (float)M_PI;
    float best = p1;
    float best_e = fabsf(wrap_pi(phi - p1));
    float e = fabsf(wrap_pi(phi - p2));
    if (e < best_e) { best = p2; best_e = e; }
    e = fabsf(wrap_pi(phi - p3));
    if (e < best_e) { best = p3; best_e = e; }
    e = fabsf(wrap_pi(phi - p4));
    if (e < best_e) { best = p4; }
    return best;
}

static void map_phase_to_bits(float target_phi, uint8_t* b0, uint8_t* b1) {
    const float eps = 1e-3f;
    if (fabsf(target_phi - 0.25f * (float)M_PI) < eps) {
        *b0 = 0u; *b1 = 0u;
    } else if (fabsf(target_phi - 0.75f * (float)M_PI) < eps) {
        *b0 = 0u; *b1 = 1u;
    } else if (fabsf(target_phi + 0.75f * (float)M_PI) < eps) {
        *b0 = 1u; *b1 = 1u;
    } else {
        *b0 = 1u; *b1 = 0u;
    }
}

static void emit_bits(VdesAsmCtx* ctx,
                      double freq_hz, uint64_t unix_ms,
                      MrEmitFn emit_fn, void* user_data,
                      const char* reason) {
    char kv[320];
    snprintf(kv, sizeof(kv),
             "{\"signal_type\":\"VDES_ASM_DATA\","
             "\"modulation\":\"pi4_dqpsk\","
             "\"reason\":\"%s\","
             "\"bit_rate_bps\":\"%u\","
             "\"sym_rate_baud\":\"%u\","
             "\"k\":\"%u\","
             "\"bit_count\":\"%u\"}",
             reason ? reason : "", ctx->bit_rate_bps, ctx->sym_rate_baud,
             ctx->k, (ctx->bit_count / 8u) * 8u);
    mr_emit_bits(ctx->bit_buf, &ctx->bit_count, VDES_MIN_BITS,
                 VDES_MAX_BITS / 8u + 1u,
                 "VDES_ASM_DATA", kv, freq_hz, unix_ms, emit_fn, user_data);
}

static void teardown_dsp(VdesAsmCtx* ctx) {
    if (ctx->resamp) { msresamp_crcf_destroy(ctx->resamp); ctx->resamp = NULL; }
    if (ctx->symsync) { symsync_crcf_destroy(ctx->symsync); ctx->symsync = NULL; }
    if (ctx->agc) { agc_crcf_destroy(ctx->agc); ctx->agc = NULL; }
    if (ctx->pll) { nco_crcf_destroy(ctx->pll); ctx->pll = NULL; }

    free(ctx->in_buf); ctx->in_buf = NULL; ctx->in_cap = 0;
    free(ctx->resamp_out); ctx->resamp_out = NULL; ctx->resamp_cap = 0;
    free(ctx->sym_out); ctx->sym_out = NULL; ctx->sym_out_cap = 0;
}

static int ensure_capacity(liquid_float_complex** buf, uint32_t* cap, uint32_t need) {
    liquid_float_complex* nbuf;
    if (need <= *cap) return 1;
    nbuf = (liquid_float_complex*)realloc(*buf, (size_t)need * sizeof(liquid_float_complex));
    if (!nbuf) return 0;
    *buf = nbuf;
    *cap = need;
    return 1;
}

static int reconfigure(VdesAsmCtx* ctx, uint32_t sample_rate_hz) {
    float target_sr;
    float rate;

    if (sample_rate_hz == 0u) sample_rate_hz = 2048000u;
    if (!ctx->needs_reconfigure && ctx->sample_rate_hz == sample_rate_hz &&
        ctx->resamp && ctx->symsync && ctx->agc && ctx->pll) {
        return 1;
    }

    teardown_dsp(ctx);

    ctx->sample_rate_hz = sample_rate_hz;
    ctx->sym_rate_baud = (ctx->bit_rate_bps >= 2u) ? (ctx->bit_rate_bps / 2u) : 14400u;
    if (ctx->sym_rate_baud < 1000u) ctx->sym_rate_baud = 1000u;

    target_sr = (float)ctx->sym_rate_baud * (float)ctx->k;
    rate = target_sr / (float)ctx->sample_rate_hz;
    if (rate <= 0.0f) return 0;

    ctx->resamp = msresamp_crcf_create(rate, 60.0f);
    if (!ctx->resamp) return 0;

    ctx->symsync = symsync_crcf_create_kaiser(ctx->k, VDES_DIFF_M, VDES_DIFF_BETA, 32u);
    if (!ctx->symsync) return 0;
    symsync_crcf_set_output_rate(ctx->symsync, 1u);
    symsync_crcf_set_lf_bw(ctx->symsync, 0.02f);

    ctx->agc = agc_crcf_create();
    if (!ctx->agc) return 0;
    agc_crcf_set_bandwidth(ctx->agc, 2e-3f);

    ctx->pll = nco_crcf_create(LIQUID_NCO);
    if (!ctx->pll) return 0;
    nco_crcf_pll_set_bandwidth(ctx->pll, ctx->pll_bw);

    mr_iir_prefilter_destroy(&ctx->pre);
    mr_iir_prefilter_create(&ctx->pre, 0.08f);

    ctx->bit_count = 0u;
    memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
    ctx->have_prev_sym = 0;
    ctx->gate_prev_open = 0;
    mr_signal_gate_reset(&ctx->gate);
    ctx->needs_reconfigure = 0;

    VLOG("reconfigure sr=%u bitrate=%u sym=%u k=%u rate=%.6f\n",
         ctx->sample_rate_hz, ctx->bit_rate_bps, ctx->sym_rate_baud,
         ctx->k, (double)rate);
    return 1;
}

static const MrPluginMeta kMeta = {
    "vdes_asm_demod",
    "0.2.0",
    MR_PLUGIN_API_VERSION,
    "EXPERIMENTAL: VDES ASM demod (libliquid symsync + diff pi/4-DQPSK)",
    MR_PLUGIN_ROLE_DEMODULATOR
};

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

MrPluginCtx* mr_plugin_create(void) {
    VdesAsmCtx* ctx = (VdesAsmCtx*)calloc(1, sizeof(VdesAsmCtx));
    if (!ctx) return NULL;

    ctx->bit_rate_bps = VDES_DEFAULT_BITRATE;
    ctx->k = VDES_DIFF_K;
    ctx->pll_bw = VDES_PLL_BW_DEFAULT;
    ctx->needs_reconfigure = 1;

    mr_iir_prefilter_init(&ctx->pre);
    mr_signal_gate_init(&ctx->gate, MR_GATE_SQUELCH_RATIO);
    return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
    VdesAsmCtx* ctx;
    if (!raw) return;
    ctx = (VdesAsmCtx*)raw;
    teardown_dsp(ctx);
    mr_iir_prefilter_destroy(&ctx->pre);
    free(ctx);
}

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
    VdesAsmCtx* ctx = (VdesAsmCtx*)raw;
    if (!ctx || !key || !value) return 0;

    if (strcmp(key, "bit_rate_bps") == 0) {
        const int v = atoi(value);
        if (v > 2000) {
            ctx->bit_rate_bps = (uint32_t)v;
            ctx->needs_reconfigure = 1;
        }
        return 1;
    }
    if (strcmp(key, "pll_bw") == 0) {
        const float v = (float)atof(value);
        if (v > 0.0f && v < 0.2f) {
            ctx->pll_bw = v;
            if (ctx->pll) nco_crcf_pll_set_bandwidth(ctx->pll, ctx->pll_bw);
        }
        return 1;
    }
    if (strcmp(key, "squelch_db") == 0) {
        const float db = (float)atof(value);
        ctx->gate.squelch_ratio = (db <= 0.0f) ? 0.0f : powf(10.0f, db / 10.0f);
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
    (void)freq_hz; (void)unix_ms; (void)source_type;
    (void)emit_fn; (void)user_data;
}

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sample_rate_hz, double center_freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
    VdesAsmCtx* ctx = (VdesAsmCtx*)raw;
    uint32_t i;
    unsigned int nr = 0u;

    if (!ctx || !iq || num_pairs == 0u) return;
    if (!reconfigure(ctx, sample_rate_hz)) return;

    if (!ensure_capacity(&ctx->in_buf, &ctx->in_cap, num_pairs)) return;

    for (i = 0; i < num_pairs; ++i) {
        liquid_float_complex x;
        __real__ x = (float)iq[i * 2u] / 32768.0f;
        __imag__ x = (float)iq[i * 2u + 1u] / 32768.0f;
        mr_iir_prefilter_execute(&ctx->pre, x, &ctx->in_buf[i]);
    }

    {
        const float rate = (float)ctx->sym_rate_baud * (float)ctx->k / (float)ctx->sample_rate_hz;
        uint32_t need = (uint32_t)((float)num_pairs * rate + 64.0f);
        if (need < 128u) need = 128u;
        if (!ensure_capacity(&ctx->resamp_out, &ctx->resamp_cap, need)) return;
    }

    msresamp_crcf_execute(ctx->resamp, ctx->in_buf, num_pairs, ctx->resamp_out, &nr);

    if (!ensure_capacity(&ctx->sym_out, &ctx->sym_out_cap, 16u)) return;

    for (i = 0; i < nr; ++i) {
        liquid_float_complex agc_out;
        unsigned int nsym = 0u;

        agc_crcf_execute(ctx->agc, ctx->resamp_out[i], &agc_out);
        symsync_crcf_execute(ctx->symsync, &agc_out, 1u, ctx->sym_out, &nsym);

        for (unsigned int s = 0; s < nsym; ++s) {
            liquid_float_complex sym_corr;
            float energy;
            int falling;

            nco_crcf_mix_down(ctx->pll, ctx->sym_out[s], &sym_corr);
            nco_crcf_step(ctx->pll);

            energy = (__real__ sym_corr) * (__real__ sym_corr) +
                     (__imag__ sym_corr) * (__imag__ sym_corr);
            falling = mr_signal_gate_update(&ctx->gate, energy, MR_GATE_HOLD_SYMS);

            if (ctx->gate_prev_open != ctx->gate.gate_open) {
                VLOG("gate %s energy=%.3e noise=%.3e bits=%u\n",
                     ctx->gate.gate_open ? "OPEN" : "CLOSED",
                     (double)ctx->gate.signal_energy,
                     (double)ctx->gate.noise_floor,
                     ctx->bit_count);
                ctx->gate_prev_open = ctx->gate.gate_open;
            }

            if (!ctx->gate.gate_open) {
                if (falling && ctx->bit_count >= VDES_MIN_BITS) {
                    emit_bits(ctx, center_freq_hz, unix_ms, emit_fn, user_data, "gate-close");
                } else if (!falling) {
                    ctx->bit_count = 0u;
                    memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
                }
                ctx->have_prev_sym = 0;
                continue;
            }

            if (!ctx->have_prev_sym) {
                ctx->prev_sym = sym_corr;
                ctx->have_prev_sym = 1;
                continue;
            }

            {
                liquid_float_complex d = conjf(ctx->prev_sym) * sym_corr;
                float phi = atan2f(__imag__ d, __real__ d);
                float target = nearest_pi4_dqpsk_phase(phi);
                float err = wrap_pi(phi - target);
                uint8_t b0 = 0u;
                uint8_t b1 = 0u;

                map_phase_to_bits(target, &b0, &b1);
                mr_push_bit(ctx->bit_buf, &ctx->bit_count, VDES_MAX_BITS, b0);
                mr_push_bit(ctx->bit_buf, &ctx->bit_count, VDES_MAX_BITS, b1);

                nco_crcf_pll_step(ctx->pll, 0.25f * err);
                ctx->prev_sym = sym_corr;
            }

            if (ctx->bit_count >= VDES_MAX_BITS) {
                emit_bits(ctx, center_freq_hz, unix_ms, emit_fn, user_data, "buf-full");
            }
        }
    }

    if (vdbg() && (++ctx->blocks % 200u) == 0u) {
        VLOG("stats blocks=%llu nr=%u bits=%u gate=%d\n",
             (unsigned long long)ctx->blocks, nr, ctx->bit_count, ctx->gate.gate_open);
    }
}
