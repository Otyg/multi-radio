/**
 * ais_dual_demod.c — Dual-channel AIS GMSK demodulator
 *
 * Role: MR_PLUGIN_ROLE_DEMODULATOR
 *
 * Simultaneously demodulates both AIS channels from a single wideband IQ
 * stream.  The two ITU-R M.1371 channels (A = 161.975 MHz, B = 162.025 MHz)
 * sit ±25 kHz from a nominal tuning of 162.000 MHz.
 *
 * For each channel the plugin:
 *   1. Frequency-shifts the wideband IQ to baseband with a digital NCO.
 *   2. Applies a channel-select Butterworth low-pass filter.
 *   3. Resamples to K × baud_rate (K = 8 samples/symbol).
 *   4. GMSK-demodulates using an energy-gated gmskdem (libliquid path) or a
 *      Gaussian-FIR + FM-discriminator fallback.
 *   5. Emits GMSK_DATA bits for the downstream NRZI → HDLC → AIS decoder
 *      chain, with frequency_hz set to the actual channel centre frequency.
 *
 * Minimum recommended sample rate: 5 × channel_offset_hz (≥ 125 kHz).
 *
 * Parameters (mr_plugin_set_param):
 *   channel_offset_hz  — half-spacing from centre (default 25000)
 *   baud_rate          — symbol rate (default 9600)
 *   bt                 — GMSK BT product (default 0.4, per ITU-R M.1371)
 *   squelch_db         — energy gate threshold above noise floor (dB)
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
#define AIS_GMSK_K              8          /* samples per symbol */
#define AIS_GMSK_M              5          /* filter delay in symbols */
#define AIS_MAX_BITS            1024
#define AIS_MIN_BITS            8

/* ================================================================== */
/* libliquid path                                                       */
/* ================================================================== */
#if defined(MR_PLUGIN_HAS_LIQUID) && MR_PLUGIN_HAS_LIQUID
#include <liquid/liquid.h>
#include "mr_iir_prefilter.h"

typedef struct {
    float                 freq_offset_hz;  /* −offset = ch_a, +offset = ch_b */
    nco_crcf              nco;             /* frequency shifter; mix_down steps internally */
    MrIirPrefilter        lpf;             /* channel-select LP */
    msresamp_crcf         resampler;       /* sr → K·baud */
    gmskdem               demodulator;
    liquid_float_complex* sym_buf;         /* K-sample symbol window */
    uint32_t              sym_buf_fill;
    liquid_float_complex* resamp_out;
    uint32_t              resamp_out_cap;
    MrSignalGate          gate;
    uint8_t               bit_buf[AIS_MAX_BITS / 8 + 1];
    uint32_t              bit_count;
} ChannelCtx;

typedef struct {
    uint32_t   baud_rate;
    float      bt;
    float      channel_offset_hz;
    int        needs_reconfigure;
    uint32_t   sample_rate_hz;
    ChannelCtx ch[2];   /* ch[0] = CH_A (−offset), ch[1] = CH_B (+offset) */
} AisDualCtx;

/* ------------------------------------------------------------------ */

static void channel_teardown(ChannelCtx* ch) {
    if (ch->nco)         { nco_crcf_destroy(ch->nco);            ch->nco         = NULL; }
    if (ch->resampler)   { msresamp_crcf_destroy(ch->resampler);  ch->resampler   = NULL; }
    if (ch->demodulator) { gmskdem_destroy(ch->demodulator);      ch->demodulator = NULL; }
    mr_iir_prefilter_destroy(&ch->lpf);
    free(ch->sym_buf);    ch->sym_buf    = NULL;
    free(ch->resamp_out); ch->resamp_out = NULL;
    ch->resamp_out_cap = 0;
    ch->sym_buf_fill   = 0;
}

static int dual_configure(AisDualCtx* ctx, uint32_t sr) {
    if (sr == ctx->sample_rate_hz && !ctx->needs_reconfigure) return 1;
    ctx->needs_reconfigure = 0;
    ctx->sample_rate_hz    = sr;

    /* ch[0] = CH_A at (centre − offset), ch[1] = CH_B at (centre + offset) */
    const float offsets[2] = { -ctx->channel_offset_hz, +ctx->channel_offset_hz };

    for (int i = 0; i < 2; ++i) {
        ChannelCtx* ch = &ctx->ch[i];
        channel_teardown(ch);
        ch->freq_offset_hz = offsets[i];
        ch->bit_count = 0;
        memset(ch->bit_buf, 0, sizeof(ch->bit_buf));
        mr_signal_gate_reset(&ch->gate);

        /* NCO: mix_down computes out = in · exp(−j·phi) and then steps phi.
           To bring channel at offsets[i] Hz to baseband, set freq = 2π·offsets[i]/sr
           so that exp(−j·freq·n) = exp(−j·2π·offsets[i]/sr·n). */
        ch->nco = nco_crcf_create(LIQUID_NCO);
        if (!ch->nco) return 0;
        nco_crcf_set_frequency(ch->nco,
            2.0f * (float)M_PI * offsets[i] / (float)sr);
        nco_crcf_reset(ch->nco);

        /* Channel-select LP at 2·baud/sr; passes AIS, rejects partner channel */
        const float cutoff = 2.0f * (float)ctx->baud_rate / (float)sr;
        mr_iir_prefilter_create(&ch->lpf, cutoff);

        const float rate = (float)ctx->baud_rate * AIS_GMSK_K / (float)sr;
        ch->resampler = msresamp_crcf_create(rate, 60.0f);
        if (!ch->resampler) return 0;

        ch->demodulator = gmskdem_create(AIS_GMSK_K, AIS_GMSK_M, ctx->bt);
        if (!ch->demodulator) return 0;

        ch->sym_buf = (liquid_float_complex*)calloc(AIS_GMSK_K,
                                                    sizeof(liquid_float_complex));
        if (!ch->sym_buf) return 0;

        const uint32_t init_cap = (uint32_t)(2048.0f / rate) + 64u;
        ch->resamp_out = (liquid_float_complex*)malloc(
                              init_cap * sizeof(liquid_float_complex));
        if (!ch->resamp_out) return 0;
        ch->resamp_out_cap = init_cap;
        ch->sym_buf_fill   = 0;
    }
    return 1;
}

static void emit_ch_bits(ChannelCtx* ch, uint32_t baud_rate, float bt,
                          double freq_actual, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
    char kv[128];
    snprintf(kv, sizeof(kv),
             "{\"baud_rate\":\"%u\",\"bt\":\"%.2f\",\"bit_count\":\"%u\"}",
             baud_rate, (double)bt, (ch->bit_count / 8u) * 8u);
    mr_emit_bits(ch->bit_buf, &ch->bit_count, AIS_MIN_BITS,
                 sizeof(ch->bit_buf), "GMSK_DATA", kv,
                 freq_actual, unix_ms, emit_fn, user_data);
}

static void process_symbol(ChannelCtx* ch, uint32_t baud_rate, float bt,
                            double freq_actual, uint64_t unix_ms,
                            MrEmitFn emit_fn, void* user_data) {
    float energy = 0.0f;
    for (int s = 0; s < AIS_GMSK_K; ++s) {
        const float re = __real__ ch->sym_buf[s];
        const float im = __imag__ ch->sym_buf[s];
        energy += re * re + im * im;
    }

    const int falling = mr_signal_gate_update(&ch->gate,
                                               energy / (float)AIS_GMSK_K,
                                               MR_GATE_HOLD_SYMS);
    if (!ch->gate.gate_open) {
        if (falling)
            emit_ch_bits(ch, baud_rate, bt, freq_actual, unix_ms, emit_fn, user_data);
        else {
            ch->bit_count = 0;
            memset(ch->bit_buf, 0, sizeof(ch->bit_buf));
        }
        return;
    }

    unsigned int bit = 0;
    gmskdem_demodulate(ch->demodulator, ch->sym_buf, &bit);
    mr_push_bit(ch->bit_buf, &ch->bit_count, AIS_MAX_BITS, bit & 1u);
    if (ch->bit_count >= AIS_MAX_BITS)
        emit_ch_bits(ch, baud_rate, bt, freq_actual, unix_ms, emit_fn, user_data);
}

/* ------------------------------------------------------------------ */
/* Plugin API — libliquid                                               */
/* ------------------------------------------------------------------ */

static const MrPluginMeta kMeta = {
    "ais_dual_demod", "1.0.0", MR_PLUGIN_API_VERSION,
    "Dual-channel AIS GMSK demodulator: CH A (-25 kHz) + CH B (+25 kHz)",
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
        mr_iir_prefilter_init(&ctx->ch[i].lpf);
    }
    return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
    if (!raw) return;
    AisDualCtx* ctx = (AisDualCtx*)raw;
    for (int i = 0; i < 2; ++i) channel_teardown(&ctx->ch[i]);
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
    if (!dual_configure(ctx, sr)) return;

    liquid_float_complex* ch_buf[2];
    ch_buf[0] = (liquid_float_complex*)malloc(num_pairs * sizeof(liquid_float_complex));
    ch_buf[1] = (liquid_float_complex*)malloc(num_pairs * sizeof(liquid_float_complex));
    if (!ch_buf[0] || !ch_buf[1]) {
        free(ch_buf[0]); free(ch_buf[1]);
        return;
    }

    /* Frequency-shift and channel-select both channels in a single sample loop */
    const float norm = 1.0f / 32768.0f;
    for (uint32_t n = 0; n < num_pairs; ++n) {
        liquid_float_complex s;
        __real__ s = (float)iq[n * 2]     * norm;
        __imag__ s = (float)iq[n * 2 + 1] * norm;

        for (int i = 0; i < 2; ++i) {
            liquid_float_complex mixed, filtered;
            nco_crcf_mix_down(ctx->ch[i].nco, s, &mixed); /* steps NCO internally */
            mr_iir_prefilter_execute(&ctx->ch[i].lpf, mixed, &filtered);
            ch_buf[i][n] = filtered;
        }
    }

    /* Resample and GMSK-demodulate each channel */
    const float rate = (float)ctx->baud_rate * AIS_GMSK_K / (float)sr;
    for (int i = 0; i < 2; ++i) {
        ChannelCtx* ch         = &ctx->ch[i];
        const double freq_actual = freq_hz + (double)ch->freq_offset_hz;

        const uint32_t needed = (uint32_t)((float)num_pairs * rate + 64u);
        if (needed > ch->resamp_out_cap) {
            liquid_float_complex* nb = (liquid_float_complex*)realloc(
                ch->resamp_out, needed * sizeof(liquid_float_complex));
            if (!nb) continue;
            ch->resamp_out     = nb;
            ch->resamp_out_cap = needed;
        }

        unsigned int n_out = 0;
        msresamp_crcf_execute(ch->resampler, ch_buf[i], num_pairs,
                              ch->resamp_out, &n_out);

        for (unsigned int j = 0; j < n_out; ++j) {
            ch->sym_buf[ch->sym_buf_fill++] = ch->resamp_out[j];
            if (ch->sym_buf_fill < (uint32_t)AIS_GMSK_K) continue;
            ch->sym_buf_fill = 0;
            process_symbol(ch, ctx->baud_rate, ctx->bt,
                           freq_actual, unix_ms, emit_fn, user_data);
        }
    }

    free(ch_buf[0]);
    free(ch_buf[1]);
}

/* ================================================================== */
/* Fallback path — no libliquid                                         */
/* ================================================================== */
#else /* !MR_PLUGIN_HAS_LIQUID */

#define DUAL_MAX_GAUSS_TAPS 512

typedef struct {
    float    freq_offset_hz;

    /* NCO: rotation by nco_step per sample gives exp(j·nco_step·n) */
    float    nco_phase;
    float    nco_step;          /* radians/sample */

    /* Channel-select LP: 2nd-order Butterworth biquad, real coefficients
       applied independently to I and Q. */
    float    b0, b1, b2, a1, a2;
    float    xi1, xi2, yi1, yi2;   /* I delay line */
    float    xq1, xq2, yq1, yq2;   /* Q delay line */

    /* FM discriminator */
    float    prev_i, prev_q;

    /* Gaussian FIR (post-FM-disc smoothing) */
    float    gauss_c[DUAL_MAX_GAUSS_TAPS];
    float    gauss_d[DUAL_MAX_GAUSS_TAPS];
    uint32_t gauss_len, gauss_pos;

    /* Symbol accumulation */
    uint32_t samples_per_symbol;
    uint32_t sym_acc;
    float    sym_val;
    uint32_t sym_n;

    MrSignalGate gate;
    uint8_t  bit_buf[AIS_MAX_BITS / 8 + 1];
    uint32_t bit_count;
} FbChannelCtx;

typedef struct {
    uint32_t     baud_rate;
    float        bt;
    float        channel_offset_hz;
    int          needs_reconfigure;
    uint32_t     sample_rate_hz;
    FbChannelCtx ch[2];
} AisDualCtx;

static uint32_t build_gauss(float* c, uint32_t max, uint32_t sps, float bt) {
    if (!sps || bt <= 0.0f) { if (max) c[0] = 1.0f; return 1; }
    const float    s    = 0.8325546f / (6.2831853f * bt) * (float)sps;
    const uint32_t half = 3 * sps;
    uint32_t       n    = 2 * half + 1;
    if (n > max) n = max;
    float sum = 0.0f;
    for (uint32_t k = 0; k < n; ++k) {
        const float x = ((float)(int32_t)k - (float)half) / s;
        c[k] = expf(-0.5f * x * x);
        sum += c[k];
    }
    if (sum > 0.0f) for (uint32_t k = 0; k < n; ++k) c[k] /= sum;
    return n;
}

/* Bilinear-transform 2nd-order Butterworth LP coefficients. */
static void biquad_butter2_lp(FbChannelCtx* ch, float cutoff_norm) {
    const float K     = tanf((float)M_PI * cutoff_norm);
    const float sqrt2 = 1.41421356f;
    const float norm  = 1.0f / (1.0f + sqrt2 * K + K * K);
    ch->b0 =  K * K * norm;
    ch->b1 =  2.0f * ch->b0;
    ch->b2 =  ch->b0;
    ch->a1 =  2.0f * (K * K - 1.0f) * norm;
    ch->a2 = (1.0f - sqrt2 * K + K * K) * norm;
}

static void dual_reconfigure(AisDualCtx* ctx, uint32_t sr) {
    if (sr == ctx->sample_rate_hz && !ctx->needs_reconfigure) return;
    ctx->needs_reconfigure = 0;
    ctx->sample_rate_hz = sr;

    const float offsets[2] = { -ctx->channel_offset_hz, +ctx->channel_offset_hz };

    for (int i = 0; i < 2; ++i) {
        FbChannelCtx* ch = &ctx->ch[i];
        ch->freq_offset_hz = offsets[i];
        ch->bit_count      = 0;
        memset(ch->bit_buf, 0, sizeof(ch->bit_buf));
        mr_signal_gate_reset(&ch->gate);

        /* NCO rotation by nco_step/sample gives exp(j·nco_step·n).
           To bring channel at offsets[i] Hz to baseband, nco_step = −2π·offsets[i]/sr. */
        ch->nco_phase = 0.0f;
        ch->nco_step  = -2.0f * (float)M_PI * offsets[i] / (float)sr;

        const float cutoff = 2.0f * (float)ctx->baud_rate / (float)sr;
        biquad_butter2_lp(ch, cutoff);
        ch->xi1 = ch->xi2 = ch->yi1 = ch->yi2 = 0.0f;
        ch->xq1 = ch->xq2 = ch->yq1 = ch->yq2 = 0.0f;

        ch->prev_i = 1.0f; ch->prev_q = 0.0f;

        ch->samples_per_symbol = sr / ctx->baud_rate;
        if (!ch->samples_per_symbol) ch->samples_per_symbol = 1;
        ch->gauss_len = build_gauss(ch->gauss_c, DUAL_MAX_GAUSS_TAPS,
                                    ch->samples_per_symbol, ctx->bt);
        memset(ch->gauss_d, 0, sizeof(float) * ch->gauss_len);
        ch->gauss_pos = 0;
        ch->sym_acc = 0; ch->sym_val = 0.0f; ch->sym_n = 0;
    }
}

static const MrPluginMeta kMeta = {
    "ais_dual_demod", "1.0.0", MR_PLUGIN_API_VERSION,
    "Dual-channel AIS GMSK demodulator: CH A (-25 kHz) + CH B (+25 kHz) [fallback]",
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
        ctx->ch[i].prev_i     = 1.0f;
    }
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

    const float norm = 1.0f / 32768.0f;
    for (uint32_t n = 0; n < num_pairs; ++n) {
        const float is = (float)iq[n * 2]     * norm;
        const float qs = (float)iq[n * 2 + 1] * norm;

        for (int ci = 0; ci < 2; ++ci) {
            FbChannelCtx* ch         = &ctx->ch[ci];
            const double  freq_actual = freq_hz + (double)ch->freq_offset_hz;

            /* Complex rotation: (is + j·qs) · exp(j·nco_phase) */
            const float cos_p = cosf(ch->nco_phase);
            const float sin_p = sinf(ch->nco_phase);
            float is_m = is * cos_p - qs * sin_p;
            float qs_m = is * sin_p + qs * cos_p;
            ch->nco_phase += ch->nco_step;
            if      (ch->nco_phase >  (float)M_PI) ch->nco_phase -= 2.0f * (float)M_PI;
            else if (ch->nco_phase < -(float)M_PI) ch->nco_phase += 2.0f * (float)M_PI;

            /* Biquad LP — I component */
            const float is_f = ch->b0 * is_m + ch->b1 * ch->xi1 + ch->b2 * ch->xi2
                             - ch->a1 * ch->yi1 - ch->a2 * ch->yi2;
            ch->xi2 = ch->xi1; ch->xi1 = is_m;
            ch->yi2 = ch->yi1; ch->yi1 = is_f;

            /* Biquad LP — Q component */
            const float qs_f = ch->b0 * qs_m + ch->b1 * ch->xq1 + ch->b2 * ch->xq2
                             - ch->a1 * ch->yq1 - ch->a2 * ch->yq2;
            ch->xq2 = ch->xq1; ch->xq1 = qs_m;
            ch->yq2 = ch->yq1; ch->yq1 = qs_f;

            /* FM discriminator */
            const float cross = qs_f * ch->prev_i - is_f * ch->prev_q;
            const float dot   = is_f * ch->prev_i + qs_f * ch->prev_q;
            const float angle = (dot != 0.0f || cross != 0.0f)
                              ? atan2f(cross, dot) : 0.0f;
            ch->prev_i = is_f;
            ch->prev_q = qs_f;

            /* Gaussian FIR */
            ch->gauss_d[ch->gauss_pos] = angle;
            ch->gauss_pos = (ch->gauss_pos + 1) % ch->gauss_len;
            float out = 0.0f;
            uint32_t bi = ch->gauss_pos;
            for (uint32_t t = 0; t < ch->gauss_len; ++t) {
                out += ch->gauss_c[t] * ch->gauss_d[bi];
                bi = (bi + 1) % ch->gauss_len;
            }

            /* Energy gate (FM-disc² as signal-power proxy) */
            mr_signal_gate_update(&ch->gate, out * out, MR_GATE_HOLD_SYMS);

            /* Symbol accumulation */
            ch->sym_val += out;
            ch->sym_n++;
            ch->sym_acc++;
            if (ch->sym_acc >= ch->samples_per_symbol) {
                if (!ch->gate.gate_open) {
                    char kv[128];
                    snprintf(kv, sizeof(kv),
                             "{\"baud_rate\":\"%u\",\"bt\":\"%.2f\",\"bit_count\":\"%u\"}",
                             ctx->baud_rate, (double)ctx->bt,
                             (ch->bit_count / 8u) * 8u);
                    mr_emit_bits(ch->bit_buf, &ch->bit_count, AIS_MIN_BITS,
                                 sizeof(ch->bit_buf), "GMSK_DATA", kv,
                                 freq_actual, unix_ms, emit_fn, user_data);
                } else {
                    mr_push_bit(ch->bit_buf, &ch->bit_count, AIS_MAX_BITS,
                                ch->sym_val / (float)ch->sym_n > 0.0f ? 1u : 0u);
                    if (ch->bit_count >= AIS_MAX_BITS) {
                        char kv[128];
                        snprintf(kv, sizeof(kv),
                                 "{\"baud_rate\":\"%u\",\"bt\":\"%.2f\",\"bit_count\":\"%u\"}",
                                 ctx->baud_rate, (double)ctx->bt,
                                 (ch->bit_count / 8u) * 8u);
                        mr_emit_bits(ch->bit_buf, &ch->bit_count, AIS_MIN_BITS,
                                     sizeof(ch->bit_buf), "GMSK_DATA", kv,
                                     freq_actual, unix_ms, emit_fn, user_data);
                    }
                }
                ch->sym_acc = 0; ch->sym_val = 0.0f; ch->sym_n = 0;
            }
        }
    }
}

#endif /* MR_PLUGIN_HAS_LIQUID */
