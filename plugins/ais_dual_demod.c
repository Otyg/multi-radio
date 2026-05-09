/**
 * ais_dual_demod.c — Dual-channel AIS splitter + GMSK demod
 *
 * Role: MR_PLUGIN_ROLE_DEMODULATOR
 *
 * Receives one wideband IQ stream tuned near 162.000 MHz and demodulates
 * both AIS channels in parallel by splitting to:
 *   CH A = center - channel_offset_hz (default 25 kHz)
 *   CH B = center + channel_offset_hz (default 25 kHz)
 *
 * Each branch reuses the same demodulation chain behavior as gmsk_demod:
 *   NCO shift -> channel-select LP -> resample -> AFC -> gmskdem -> GMSK_DATA
 *
 * Downstream chain remains unchanged:
 *   GMSK_DATA -> nrzi_decoder -> ais_decoder
 */

#include "mr_plugin_api.h"
#include "mr_signal_gate.h"
#include "mr_bit_buf.h"
#include "mr_afc.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <liquid/liquid.h>
#include "mr_iir_prefilter.h"

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#define AIS_BAUD_RATE_DEFAULT 9600u
#define AIS_BT_DEFAULT        0.4f
#define AIS_CHANNEL_OFFSET_HZ 25000.0f

#define GMSK_K                 8
#define GMSK_M                 5
#define AIS_MAX_BITS           1024
#define AIS_MIN_BITS           8

static int ais_dbg(void) {
    static int v = -1;
    if (v < 0) {
        const char* e = getenv("MR_AIS_DEBUG");
        v = (e && e[0] != '0') ? 1 : 0;
    }
    return v;
}

#define DLOG(...) do { if (ais_dbg()) fprintf(stderr, "[ais_dual] " __VA_ARGS__); } while (0)

typedef struct {
    float freq_offset_hz;

    /* Per-branch frequency translation */
    float nco_phase;
    float nco_step;

    msresamp_crcf  resampler;
    gmskdem        demod;
    MrIirPrefilter pre_filter;

    liquid_float_complex* sym_buf;
    uint32_t              sym_fill;

    liquid_float_complex* resamp_out;
    uint32_t              resamp_out_cap;

    MrSignalGate gate;
    MrAfc        afc;
    int          gate_was_open;

    uint8_t  bit_buf[AIS_MAX_BITS / 8 + 1];
    uint32_t bit_count;
} AisBranch;

typedef struct {
    uint32_t baud_rate;
    float    bt;
    float    channel_offset_hz;

    int      needs_reconfigure;
    uint32_t sample_rate_hz;

    uint32_t block_count;

    AisBranch ch[2];
    liquid_float_complex* mixed[2];
    uint32_t mixed_cap;
} AisDualCtx;

static void emit_bits_branch(AisBranch* ch,
                             MrEmitFn emit_fn, void* user_data,
                             double freq_hz, uint64_t unix_ms,
                             uint32_t baud_rate, float bt,
                             const char* reason) {
    if (ais_dbg() && reason != NULL) {
        DLOG("emit(%s): %.3f MHz  bits=%u\n",
             reason, freq_hz / 1e6, (ch->bit_count / 8u) * 8u);
    }

    char kv[160];
    snprintf(kv, sizeof(kv),
             "{\"baud_rate\":\"%u\",\"bt\":\"%.3f\",\"bit_count\":\"%u\"}",
             baud_rate, (double)bt, (ch->bit_count / 8u) * 8u);

    mr_emit_bits(ch->bit_buf, &ch->bit_count, AIS_MIN_BITS,
                 AIS_MAX_BITS / 8u + 1u,
                 "GMSK_DATA", kv, freq_hz, unix_ms, emit_fn, user_data);
}

static void branch_teardown(AisBranch* ch) {
    if (ch->resampler) { msresamp_crcf_destroy(ch->resampler); ch->resampler = NULL; }
    if (ch->demod)     { gmskdem_destroy(ch->demod);           ch->demod = NULL; }
    mr_iir_prefilter_destroy(&ch->pre_filter);
    free(ch->sym_buf);    ch->sym_buf = NULL;
    free(ch->resamp_out); ch->resamp_out = NULL;
    ch->resamp_out_cap = 0;
    ch->sym_fill = 0;
}

static int branch_configure(AisBranch* ch,
                            float offset_hz,
                            uint32_t sr,
                            uint32_t baud_rate,
                            float bt) {
    branch_teardown(ch);

    ch->freq_offset_hz = offset_hz;
    ch->bit_count = 0;
    memset(ch->bit_buf, 0, sizeof(ch->bit_buf));
    mr_signal_gate_reset(&ch->gate);
    mr_afc_init(&ch->afc);

    /* Mix branch target to baseband */
    ch->nco_phase = 0.0f;
    ch->nco_step = -2.0f * (float)M_PI * offset_hz / (float)sr;

    /* Same pre-filter setting as gmsk_demod */
    mr_iir_prefilter_create(&ch->pre_filter, 2.0f * (float)baud_rate / (float)sr);

    const float rate = (float)baud_rate * (float)GMSK_K / (float)sr;
    ch->resampler = msresamp_crcf_create(rate, 60.0f);
    if (!ch->resampler) return 0;

    ch->demod = gmskdem_create(GMSK_K, GMSK_M, bt);
    if (!ch->demod) return 0;

    ch->sym_buf = (liquid_float_complex*)calloc(GMSK_K, sizeof(liquid_float_complex));
    if (!ch->sym_buf) return 0;

    const uint32_t init_cap = (uint32_t)(2048.0f / rate) + 64u;
    ch->resamp_out = (liquid_float_complex*)malloc(init_cap * sizeof(liquid_float_complex));
    if (!ch->resamp_out) return 0;
    ch->resamp_out_cap = init_cap;

    ch->sym_fill = 0;
    ch->gate_was_open = 0;

    DLOG("configure ch%c: offset=%.0f Hz sr=%u nco_step=%.6f\n",
         offset_hz < 0.0f ? 'A' : 'B', (double)offset_hz, sr, (double)ch->nco_step);
    return 1;
}

static int dual_reconfigure(AisDualCtx* ctx, uint32_t sr) {
    if (sr == ctx->sample_rate_hz && !ctx->needs_reconfigure) return 1;

    ctx->needs_reconfigure = 0;
    ctx->sample_rate_hz = sr;

    if (!branch_configure(&ctx->ch[0], -ctx->channel_offset_hz, sr, ctx->baud_rate, ctx->bt)) {
        DLOG("configure failed for channel A\n");
        return 0;
    }
    if (!branch_configure(&ctx->ch[1], +ctx->channel_offset_hz, sr, ctx->baud_rate, ctx->bt)) {
        DLOG("configure failed for channel B\n");
        return 0;
    }
    return 1;
}

static int ensure_mixed_capacity(AisDualCtx* ctx, uint32_t num_pairs) {
    if (num_pairs <= ctx->mixed_cap) return 1;

    liquid_float_complex* a = (liquid_float_complex*)malloc(
        (size_t)num_pairs * sizeof(liquid_float_complex));
    liquid_float_complex* b = (liquid_float_complex*)malloc(
        (size_t)num_pairs * sizeof(liquid_float_complex));
    if (!a || !b) {
        free(a);
        free(b);
        return 0;
    }

    free(ctx->mixed[0]);
    free(ctx->mixed[1]);
    ctx->mixed[0] = a;
    ctx->mixed[1] = b;
    ctx->mixed_cap = num_pairs;
    return 1;
}

static void process_symbol(AisDualCtx* ctx,
                           AisBranch* ch,
                           liquid_float_complex* sym,
                           double freq_hz,
                           uint64_t unix_ms,
                           MrEmitFn emit_fn,
                           void* user_data) {
    float energy = 0.0f;
    for (int i = 0; i < GMSK_K; ++i) {
        const float re = __real__ sym[i];
        const float im = __imag__ sym[i];
        energy += re * re + im * im;
    }

    const int falling = mr_signal_gate_update(&ch->gate,
                                              energy / (float)GMSK_K,
                                              MR_GATE_HOLD_SYMS);
    if (ais_dbg() && ch->gate.gate_open != ch->gate_was_open) {
        DLOG("gate %s ch%c %.3f MHz sig=%.2e noise=%.2e\n",
             ch->gate.gate_open ? "OPEN" : "CLOSED",
             ch->freq_offset_hz < 0.0f ? 'A' : 'B',
             freq_hz / 1e6,
             (double)ch->gate.signal_energy,
             (double)ch->gate.noise_floor);
        ch->gate_was_open = ch->gate.gate_open;
    }

    if (!ch->gate.gate_open) {
        if (falling) {
            emit_bits_branch(ch, emit_fn, user_data, freq_hz, unix_ms,
                             ctx->baud_rate, ctx->bt, "gate-close");
        } else {
            ch->bit_count = 0;
            memset(ch->bit_buf, 0, sizeof(ch->bit_buf));
        }
        return;
    }

    mr_afc_update(&ch->afc, sym, GMSK_K);

    unsigned int bit = 0u;
    gmskdem_demodulate(ch->demod, sym, &bit);
    mr_push_bit(ch->bit_buf, &ch->bit_count, AIS_MAX_BITS, bit & 1u);

    if (ch->bit_count >= AIS_MAX_BITS) {
        emit_bits_branch(ch, emit_fn, user_data, freq_hz, unix_ms,
                         ctx->baud_rate, ctx->bt, "buf-full");
    }
}

static const MrPluginMeta kMeta = {
    "ais_dual_demod", "3.0.0", MR_PLUGIN_API_VERSION,
    "Dual-channel AIS splitter + GMSK demod (libliquid path)",
    MR_PLUGIN_ROLE_DEMODULATOR
};

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

MrPluginCtx* mr_plugin_create(void) {
    AisDualCtx* ctx = (AisDualCtx*)calloc(1, sizeof(AisDualCtx));
    if (!ctx) return NULL;

    ctx->baud_rate         = AIS_BAUD_RATE_DEFAULT;
    ctx->bt                = AIS_BT_DEFAULT;
    ctx->channel_offset_hz = AIS_CHANNEL_OFFSET_HZ;

    for (int i = 0; i < 2; ++i) {
        mr_signal_gate_init(&ctx->ch[i].gate, MR_GATE_SQUELCH_RATIO);
        mr_afc_init(&ctx->ch[i].afc);
        mr_iir_prefilter_init(&ctx->ch[i].pre_filter);
    }

    DLOG("created baud=%u bt=%.2f offset=%.0f\n",
         ctx->baud_rate, (double)ctx->bt, (double)ctx->channel_offset_hz);
    return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
    if (!raw) return;
    AisDualCtx* ctx = (AisDualCtx*)raw;

    for (int i = 0; i < 2; ++i) branch_teardown(&ctx->ch[i]);
    free(ctx->mixed[0]);
    free(ctx->mixed[1]);
    free(ctx);
}

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
    if (!raw || !key || !value) return 0;
    AisDualCtx* ctx = (AisDualCtx*)raw;

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
    if (strcmp(key, "channel_offset_hz") == 0) {
        const float v = (float)atof(value);
        if (v > 0.0f) { ctx->channel_offset_hz = v; ctx->needs_reconfigure = 1; }
        return 1;
    }
    if (strcmp(key, "squelch_db") == 0) {
        const float db = (float)atof(value);
        const float ratio = (db <= 0.0f) ? 0.0f : powf(10.0f, db / 10.0f);
        ctx->ch[0].gate.squelch_ratio = ratio;
        ctx->ch[1].gate.squelch_ratio = ratio;
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
    (void)ctx; (void)bit_bytes; (void)bit_count;
    (void)freq_hz; (void)unix_ms; (void)source_type;
    (void)emit_fn; (void)user_data;
}

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq,
                          uint32_t num_pairs,
                          uint32_t sr,
                          double freq_hz,
                          uint64_t unix_ms,
                          MrEmitFn emit_fn,
                          void* user_data) {
    if (!raw || !iq || !num_pairs) return;

    AisDualCtx* ctx = (AisDualCtx*)raw;
    if (!sr) sr = 2048000u;

    if (!dual_reconfigure(ctx, sr)) return;
    if (!ensure_mixed_capacity(ctx, num_pairs)) return;

    if (ais_dbg() && ++ctx->block_count % 100u == 0u) {
        for (int ci = 0; ci < 2; ++ci) {
            const AisBranch* ch = &ctx->ch[ci];
            DLOG("stats ch%c gate=%s sig=%.2e noise=%.2e bits=%u\n",
                 ci == 0 ? 'A' : 'B',
                 ch->gate.gate_open ? "OPEN" : "closed",
                 (double)ch->gate.signal_energy,
                 (double)ch->gate.noise_floor,
                 ch->bit_count);
        }
    }

    const float norm = 1.0f / 32768.0f;

    for (uint32_t n = 0; n < num_pairs; ++n) {
        const float is = (float)iq[n * 2u] * norm;
        const float qs = (float)iq[n * 2u + 1u] * norm;

        for (int ci = 0; ci < 2; ++ci) {
            AisBranch* ch = &ctx->ch[ci];

            const float c = cosf(ch->nco_phase);
            const float s = sinf(ch->nco_phase);
            liquid_float_complex mixed;
            __real__ mixed = is * c - qs * s;
            __imag__ mixed = is * s + qs * c;

            ch->nco_phase += ch->nco_step;
            if (ch->nco_phase >  (float)M_PI) ch->nco_phase -= 2.0f * (float)M_PI;
            if (ch->nco_phase < -(float)M_PI) ch->nco_phase += 2.0f * (float)M_PI;

            mr_iir_prefilter_execute(&ch->pre_filter, mixed, &ctx->mixed[ci][n]);
        }
    }

    for (int ci = 0; ci < 2; ++ci) {
        AisBranch* ch = &ctx->ch[ci];
        const float rate = (float)ctx->baud_rate * (float)GMSK_K / (float)sr;
        const uint32_t needed = (uint32_t)((float)num_pairs * rate + 64.0f);

        if (needed > ch->resamp_out_cap) {
            liquid_float_complex* nb = (liquid_float_complex*)realloc(
                ch->resamp_out, (size_t)needed * sizeof(liquid_float_complex));
            if (!nb) continue;
            ch->resamp_out = nb;
            ch->resamp_out_cap = needed;
        }

        unsigned int n_out = 0u;
        msresamp_crcf_execute(ch->resampler, ctx->mixed[ci], num_pairs,
                              ch->resamp_out, &n_out);

        const double branch_freq_hz = freq_hz + (double)ch->freq_offset_hz;

        for (unsigned int i = 0; i < n_out; ++i) {
            mr_afc_correct(&ch->afc,
                           ch->resamp_out[i],
                           &ch->sym_buf[ch->sym_fill]);
            ch->sym_fill++;
            if (ch->sym_fill < (uint32_t)GMSK_K) continue;
            ch->sym_fill = 0;
            process_symbol(ctx, ch, ch->sym_buf,
                           branch_freq_hz, unix_ms,
                           emit_fn, user_data);
        }
    }
}
