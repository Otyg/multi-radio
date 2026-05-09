/**
 * ais_dual_demod.c — Dual-channel AIS GMSK demodulator
 *
 * Role: MR_PLUGIN_ROLE_DEMODULATOR
 *
 * Receives one wideband IQ stream tuned near 162.000 MHz and simultaneously
 * demodulates both AIS channels:
 *   CH A  161.975 MHz  (centre − 25 kHz)
 *   CH B  162.025 MHz  (centre + 25 kHz)
 *
 * Per channel:
 *   1. NCO frequency shift  →  baseband
 *   2. 2nd-order Butterworth IIR channel-select LP filter
 *   3. FM discriminator (differential phase)
 *   4. Gaussian FIR smoothing
 *   5. Energy gate + symbol decision at baud_rate
 *   6. Emit GMSK_DATA bits  →  downstream nrzi_decoder → ais_decoder
 *
 * Minimum sample rate: 5 × channel_offset_hz (≥ 125 kHz recommended).
 *
 * Parameters (mr_plugin_set_param):
 *   channel_offset_hz  — half-spacing from centre (default 25000)
 *   baud_rate          — symbol rate (default 9600)
 *   bt                 — GMSK BT product (default 0.4, per ITU-R M.1371)
 *   squelch_db         — gate threshold above noise floor (dB)
 */

#include "mr_plugin_api.h"
#include "mr_signal_gate.h"
#include "mr_bit_buf.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#define AIS_BAUD_RATE_DEFAULT   9600u
#define AIS_BT_DEFAULT          0.4f
#define AIS_CHANNEL_OFFSET_HZ   25000.0f
#define AIS_MAX_BITS            1024
#define AIS_MIN_BITS            8
#define AIS_MAX_GAUSS_TAPS      512

/* ------------------------------------------------------------------ */
/* Debug (set MR_AIS_DEBUG=1 to enable)                                */
/* ------------------------------------------------------------------ */

static int ais_dbg(void) {
    static int v = -1;
    if (v < 0) {
        const char* e = getenv("MR_AIS_DEBUG");
        v = (e && e[0] != '0') ? 1 : 0;
    }
    return v;
}
#define DLOG(...) do { if (ais_dbg()) fprintf(stderr, "[ais_dual] " __VA_ARGS__); } while (0)

/* ------------------------------------------------------------------ */
/* Per-channel state                                                    */
/* ------------------------------------------------------------------ */

typedef struct {
    float    freq_offset_hz;

    /* NCO */
    float    nco_phase;
    float    nco_step;           /* radians/input-sample */

    /* Channel-select LP: 2nd-order Butterworth biquad, real coefficients
       applied to I and Q independently. */
    float    b0, b1, b2, a1, a2;
    float    xi1, xi2, yi1, yi2; /* I delay */
    float    xq1, xq2, yq1, yq2; /* Q delay */

    /* FM discriminator */
    float    prev_re, prev_im;

    /* Gaussian FIR on FM-disc output */
    float    gauss_c[AIS_MAX_GAUSS_TAPS];
    float    gauss_d[AIS_MAX_GAUSS_TAPS];
    uint32_t gauss_len;
    uint32_t gauss_pos;

    /* Symbol accumulation */
    uint32_t samples_per_symbol;
    uint32_t sym_acc;
    float    sym_val;
    float    sym_energy;
    uint32_t sym_n;

    MrSignalGate gate;
    int      gate_was_open;   /* tracks gate transitions for debug */
    uint8_t  bit_buf[AIS_MAX_BITS / 8 + 1];
    uint32_t bit_count;
} ChannelCtx;

typedef struct {
    uint32_t   baud_rate;
    float      bt;
    float      channel_offset_hz;
    int        needs_reconfigure;
    uint32_t   sample_rate_hz;
    uint32_t   block_count;   /* counts process_iq calls, for periodic stats */
    ChannelCtx ch[2];   /* ch[0] = CH_A (−offset), ch[1] = CH_B (+offset) */
} AisDualCtx;

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

static uint32_t build_gauss(float* c, uint32_t max, uint32_t sps, float bt) {
    if (!sps || bt <= 0.0f) { if (max) c[0] = 1.0f; return 1; }
    const float    s    = 0.8325546f / (6.2831853f * bt) * (float)sps;
    const uint32_t half = 3u * sps;
    uint32_t       n    = 2u * half + 1u;
    if (n > max) n = max;
    float sum = 0.0f;
    for (uint32_t k = 0; k < n; ++k) {
        const float x = ((float)(int32_t)k - (float)half) / s;
        c[k] = expf(-0.5f * x * x);
        sum += c[k];
    }
    if (sum > 0.0f)
        for (uint32_t k = 0; k < n; ++k) c[k] /= sum;
    return n;
}

/* Bilinear-transform 2nd-order Butterworth LP. cutoff_norm = fc/fs ∈ (0, 0.5). */
static void biquad_lp(ChannelCtx* ch, float cutoff_norm) {
    const float K     = tanf((float)M_PI * cutoff_norm);
    const float sqrt2 = 1.41421356f;
    const float norm  = 1.0f / (1.0f + sqrt2 * K + K * K);
    ch->b0 =  K * K * norm;
    ch->b1 =  2.0f * ch->b0;
    ch->b2 =  ch->b0;
    ch->a1 =  2.0f * (K * K - 1.0f) * norm;
    ch->a2 = (1.0f - sqrt2 * K + K * K) * norm;
}

static void channel_reconfigure(ChannelCtx* ch, float offset_hz,
                                uint32_t sr, uint32_t baud, float bt) {
    ch->freq_offset_hz = offset_hz;
    ch->bit_count = 0;
    memset(ch->bit_buf, 0, sizeof(ch->bit_buf));
    mr_signal_gate_reset(&ch->gate);

    /* NCO: rotation by nco_step/sample gives exp(j·nco_step·n).
       To bring signal at offset_hz to baseband: nco_step = −2π·offset_hz/sr. */
    ch->nco_phase = 0.0f;
    ch->nco_step  = -2.0f * (float)M_PI * offset_hz / (float)sr;

    /* Channel-select LP at 2·baud/sr */
    biquad_lp(ch, 2.0f * (float)baud / (float)sr);
    ch->xi1 = ch->xi2 = ch->yi1 = ch->yi2 = 0.0f;
    ch->xq1 = ch->xq2 = ch->yq1 = ch->yq2 = 0.0f;

    /* FM discriminator */
    ch->prev_re = 1.0f; ch->prev_im = 0.0f;

    /* Gaussian FIR */
    ch->samples_per_symbol = sr / baud;
    if (!ch->samples_per_symbol) ch->samples_per_symbol = 1;
    ch->gauss_len = build_gauss(ch->gauss_c, AIS_MAX_GAUSS_TAPS,
                                ch->samples_per_symbol, bt);
    memset(ch->gauss_d, 0, sizeof(float) * ch->gauss_len);
    ch->gauss_pos = 0;

    ch->sym_acc = 0; ch->sym_val = 0.0f; ch->sym_energy = 0.0f; ch->sym_n = 0;
    ch->gate_was_open = 0;

    DLOG("configure ch%c: offset=%.0f Hz  sr=%u  sps=%u  gauss_len=%u  "
         "cutoff=%.4f  nco_step=%.6f rad/smp\n",
         offset_hz < 0.0f ? 'A' : 'B',
         (double)offset_hz, sr, ch->samples_per_symbol, ch->gauss_len,
         2.0f * (float)baud / (float)sr, (double)ch->nco_step);
}

static void dual_reconfigure(AisDualCtx* ctx, uint32_t sr) {
    if (sr == ctx->sample_rate_hz && !ctx->needs_reconfigure) return;
    ctx->needs_reconfigure = 0;
    ctx->sample_rate_hz    = sr;
    channel_reconfigure(&ctx->ch[0], -ctx->channel_offset_hz,
                        sr, ctx->baud_rate, ctx->bt);
    channel_reconfigure(&ctx->ch[1], +ctx->channel_offset_hz,
                        sr, ctx->baud_rate, ctx->bt);
}

/* ------------------------------------------------------------------ */
/* Plugin API                                                           */
/* ------------------------------------------------------------------ */

static const MrPluginMeta kMeta = {
    "ais_dual_demod", "2.0.0", MR_PLUGIN_API_VERSION,
    "Dual-channel AIS demodulator: CH A (-25 kHz) + CH B (+25 kHz)",
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
        ctx->ch[i].gauss_c[0] = 1.0f;
        ctx->ch[i].gauss_len  = 1;
        ctx->ch[i].prev_re    = 1.0f;
    }
    DLOG("created  baud=%u  bt=%.2f  offset=%.0f Hz  squelch_ratio=%.1f\n",
         ctx->baud_rate, (double)ctx->bt, (double)ctx->channel_offset_hz,
         (double)ctx->ch[0].gate.squelch_ratio);
    return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) { free(raw); }

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
        const float db    = (float)atof(value);
        const float ratio = (db <= 0.0f) ? 0.0f : powf(10.0f, db / 10.0f);
        ctx->ch[0].gate.squelch_ratio = ratio;
        ctx->ch[1].gate.squelch_ratio = ratio;
        return 1;
    }
    return 0;
}

void mr_plugin_process_bits(MrPluginCtx* ctx,
                            const uint8_t* bit_bytes, uint32_t bit_count,
                            double freq_hz, uint64_t unix_ms,
                            const char* source_type,
                            MrEmitFn emit_fn, void* user_data) {
    (void)ctx; (void)bit_bytes; (void)bit_count; (void)freq_hz;
    (void)unix_ms; (void)source_type; (void)emit_fn; (void)user_data;
}

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
    if (!raw || !iq || !num_pairs) return;
    AisDualCtx* ctx = (AisDualCtx*)raw;
    if (!sr) sr = 2048000;
    dual_reconfigure(ctx, sr);

    /* Periodic stats every 100 blocks (~3 s at 250 kHz / 8192 pairs per block) */
    if (ais_dbg() && ++ctx->block_count % 100 == 0) {
        for (int ci = 0; ci < 2; ++ci) {
            const ChannelCtx* ch = &ctx->ch[ci];
            fprintf(stderr,
                    "[ais_dual] stats ch%c: gate=%s  sig=%.2e  noise=%.2e"
                    "  ratio=%.1f  bits_acc=%u\n",
                    ci == 0 ? 'A' : 'B',
                    ch->gate.gate_open ? "OPEN  " : "closed",
                    (double)ch->gate.signal_energy,
                    (double)ch->gate.noise_floor,
                    ch->gate.noise_floor > 0.0f
                        ? (double)(ch->gate.signal_energy / ch->gate.noise_floor) : 0.0,
                    ch->bit_count);
        }
    }

    const float norm = 1.0f / 32768.0f;

    for (uint32_t n = 0; n < num_pairs; ++n) {
        const float is = (float)iq[n * 2]     * norm;
        const float qs = (float)iq[n * 2 + 1] * norm;

        for (int ci = 0; ci < 2; ++ci) {
            ChannelCtx* ch          = &ctx->ch[ci];
            const double freq_actual = freq_hz + (double)ch->freq_offset_hz;

            /* 1. NCO: rotate by nco_phase to shift channel to baseband */
            const float cos_p = cosf(ch->nco_phase);
            const float sin_p = sinf(ch->nco_phase);
            float re_m = is * cos_p - qs * sin_p;
            float im_m = is * sin_p + qs * cos_p;
            ch->nco_phase += ch->nco_step;
            if      (ch->nco_phase >  (float)M_PI) ch->nco_phase -= 2.0f * (float)M_PI;
            else if (ch->nco_phase < -(float)M_PI) ch->nco_phase += 2.0f * (float)M_PI;

            /* 2. Biquad LP — I */
            const float re_f = ch->b0 * re_m + ch->b1 * ch->xi1 + ch->b2 * ch->xi2
                             - ch->a1 * ch->yi1 - ch->a2 * ch->yi2;
            ch->xi2 = ch->xi1; ch->xi1 = re_m;
            ch->yi2 = ch->yi1; ch->yi1 = re_f;

            /* 2. Biquad LP — Q */
            const float im_f = ch->b0 * im_m + ch->b1 * ch->xq1 + ch->b2 * ch->xq2
                             - ch->a1 * ch->yq1 - ch->a2 * ch->yq2;
            ch->xq2 = ch->xq1; ch->xq1 = im_m;
            ch->yq2 = ch->yq1; ch->yq1 = im_f;

            /* 3. FM discriminator: instantaneous phase difference */
            const float cross = im_f * ch->prev_re - re_f * ch->prev_im;
            const float dot   = re_f * ch->prev_re + im_f * ch->prev_im;
            const float angle = (dot != 0.0f || cross != 0.0f)
                              ? atan2f(cross, dot) : 0.0f;
            ch->prev_re = re_f;
            ch->prev_im = im_f;

            /* 4. Gaussian FIR */
            ch->gauss_d[ch->gauss_pos] = angle;
            ch->gauss_pos = (ch->gauss_pos + 1) % ch->gauss_len;
            float out = 0.0f;
            uint32_t bi = ch->gauss_pos;
            for (uint32_t t = 0; t < ch->gauss_len; ++t) {
                out += ch->gauss_c[t] * ch->gauss_d[bi];
                bi = (bi + 1) % ch->gauss_len;
            }

            /* 5. Symbol accumulation */
            ch->sym_val += out;
            ch->sym_energy += out * out;
            ch->sym_n++;
            ch->sym_acc++;
            if (ch->sym_acc >= ch->samples_per_symbol) {
                /* Gate must be updated per symbol, not per sample, so hold_syms
                   and EMA constants match the intended time base. */
                mr_signal_gate_update(&ch->gate,
                                      ch->sym_energy / (float)ch->sym_n,
                                      MR_GATE_HOLD_SYMS);
                if (ais_dbg() && ch->gate.gate_open != ch->gate_was_open) {
                    fprintf(stderr,
                            "[ais_dual] gate ch%c %s  freq=%.3f MHz  "
                            "sig=%.2e  noise=%.2e\n",
                            ci == 0 ? 'A' : 'B',
                            ch->gate.gate_open ? "OPEN" : "CLOSED",
                            (freq_hz + (double)ch->freq_offset_hz) / 1e6,
                            (double)ch->gate.signal_energy,
                            (double)ch->gate.noise_floor);
                    ch->gate_was_open = ch->gate.gate_open;
                }

                /* 6. Symbol decision / emission */
                if (!ch->gate.gate_open) {
                    if (ch->bit_count >= AIS_MIN_BITS)
                        DLOG("emit(gate-close) ch%c: %u bits → GMSK_DATA\n",
                             ci == 0 ? 'A' : 'B', (ch->bit_count / 8u) * 8u);
                    char kv[128];
                    snprintf(kv, sizeof(kv),
                             "{\"baud_rate\":\"%u\",\"bt\":\"%.2f\","
                             "\"bit_count\":\"%u\"}",
                             ctx->baud_rate, (double)ctx->bt,
                             (ch->bit_count / 8u) * 8u);
                    mr_emit_bits(ch->bit_buf, &ch->bit_count, AIS_MIN_BITS,
                                 sizeof(ch->bit_buf), "GMSK_DATA", kv,
                                 freq_actual, unix_ms, emit_fn, user_data);
                } else {
                    mr_push_bit(ch->bit_buf, &ch->bit_count, AIS_MAX_BITS,
                                ch->sym_val / (float)ch->sym_n > 0.0f ? 1u : 0u);
                    if (ch->bit_count >= AIS_MAX_BITS) {
                        DLOG("emit(buf-full) ch%c: %u bits → GMSK_DATA\n",
                             ci == 0 ? 'A' : 'B', ch->bit_count);
                        char kv[128];
                        snprintf(kv, sizeof(kv),
                                 "{\"baud_rate\":\"%u\",\"bt\":\"%.2f\","
                                 "\"bit_count\":\"%u\"}",
                                 ctx->baud_rate, (double)ctx->bt,
                                 (ch->bit_count / 8u) * 8u);
                        mr_emit_bits(ch->bit_buf, &ch->bit_count, AIS_MIN_BITS,
                                     sizeof(ch->bit_buf), "GMSK_DATA", kv,
                                     freq_actual, unix_ms, emit_fn, user_data);
                    }
                }
                ch->sym_acc = 0; ch->sym_val = 0.0f; ch->sym_energy = 0.0f; ch->sym_n = 0;
            }
        }
    }
}
