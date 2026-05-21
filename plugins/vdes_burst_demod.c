/**
 * vdes_burst_demod.c — VDES ASM burst-mode pi/4-DQPSK demodulator
 *
 * Role: MR_PLUGIN_ROLE_DEMODULATOR
 *
 * The fundamental problem with vdes_asm_demod for short VDES bursts is that
 * the adaptive carrier PLL and symbol-sync loop keep running during payload,
 * causing them to wander on random-looking data.
 *
 * This plugin uses a two-phase approach:
 *
 *   PREAMBLE phase (gate just opened):
 *     - Run AGC, symbol sync, and PLL with full adaptive bandwidth.
 *     - Accumulate differential-phase measurements to build a carrier-offset
 *       estimate (average diff-phase should equal +π/4 during the all-zeros
 *       preamble; any excess is the carrier offset).
 *     - Detect the preamble→sync transition (diff-phase stops being ≈ +π/4).
 *
 *   DEMOD phase (after preamble acquisition):
 *     - Apply the estimated carrier offset as a one-shot NCO correction.
 *     - FREEZE the symbol sync and stop PLL updates so adaptive loops
 *       cannot wander during the payload.
 *     - Decode differential phases to bits and accumulate.
 *
 * Emits: VDES_BURST_DATA (to be passed to vdes_burst_decoder)
 *
 * Default parameters (set via mr_plugin_set_param):
 *   symbol_rate_baud  75000      (100 kHz VDES channel)
 *   preamble_min_syms 16         (symbols before forced-freeze)
 *   preamble_max_syms 256
 *   preamble_tol_deg  30         (±degrees around +π/4 for preamble detection)
 *   pll_bw            0.05       (PLL bandwidth during preamble, normalised)
 *   sync_bw           0.05       (symbol-sync bandwidth during preamble)
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

/* ── tunables ─────────────────────────────────────────────────────────── */

#define BD_DEFAULT_SYM_RATE   75000u   /* baud  — 100 kHz VDES channel  */
#define BD_K                  4u       /* samples per symbol             */
#define BD_SYMSYNC_M          5u       /* symsync filter half-length     */
#define BD_SYMSYNC_BETA       0.35f    /* raised-cosine roll-off         */
#define BD_MAX_BITS           8192u
#define BD_MIN_BITS           54u      /* sync + a few payload bits      */
#define BD_PREAMBLE_MIN_SYMS  16u
#define BD_PREAMBLE_MAX_SYMS  256u
#define BD_PREAMBLE_TOL_DEG   30.0f   /* ±° around +π/4 for preamble    */
#define BD_PLL_BW_PREAMBLE    0.05f
#define BD_SYNC_BW_PREAMBLE   0.05f
#define BD_SYNC_BW_FROZEN     0.001f  /* near-zero, not fully frozen     */

/* ── state machine ────────────────────────────────────────────────────── */

typedef enum {
    BD_IDLE,
    BD_PREAMBLE,
    BD_DEMOD
} BurstDemodState;

/* ── context ──────────────────────────────────────────────────────────── */

typedef struct {
    /* config */
    uint32_t sample_rate_hz;
    uint32_t symbol_rate_baud;
    float    pll_bw_preamble;
    float    sync_bw_preamble;
    uint32_t preamble_min_syms;
    uint32_t preamble_max_syms;
    float    preamble_tol_rad;  /* tolerance around +π/4 in radians */
    int      needs_reconfigure;

    /* libliquid DSP chain */
    msresamp_crcf  resamp;
    agc_crcf       agc;
    symsync_crcf   symsync;
    nco_crcf       pll;
    MrIirPrefilter prefilter;

    /* buffers */
    liquid_float_complex* in_buf;
    uint32_t              in_cap;
    liquid_float_complex* resamp_out;
    uint32_t              resamp_cap;
    liquid_float_complex  sym_buf[BD_K]; /* one symbol group */

    /* state machine */
    BurstDemodState state;

    /* preamble accumulator for carrier-offset estimation */
    double   pream_acc_i;   /* Re Σ conj(prev)·curr */
    double   pream_acc_q;   /* Im Σ conj(prev)·curr */
    uint32_t pream_n;       /* preamble-like symbols counted */
    uint32_t pream_total;   /* all symbols since gate opened */

    /* differential decoder state */
    liquid_float_complex prev_sym;
    int                  have_prev;

    /* signal gate */
    MrSignalGate gate;
    int          gate_was_open;

    /* bit buffer */
    uint8_t  bit_buf[BD_MAX_BITS / 8u + 1u];
    uint32_t bit_count;

    /* diagnostics */
    uint64_t blocks_seen;
    uint64_t gate_opens;
    float    last_freq_err_hz;
    uint32_t last_pream_syms;
} VdesBurstDemodCtx;

/* ── debug ────────────────────────────────────────────────────────────── */

static int vdbg(void) {
    static int v = -1;
    if (v < 0) { const char* e = getenv("MR_AIS_DEBUG"); v = (e && e[0] != '0'); }
    return v;
}
#define VLOG(...) do { if (vdbg()) fprintf(stderr, "[vdes_burst_demod] " __VA_ARGS__); } while(0)

/* ── pi/4-DQPSK helpers ───────────────────────────────────────────────── */

static float wrap_pi(float x) {
    while (x >  (float)M_PI) x -= 2.0f * (float)M_PI;
    while (x < -(float)M_PI) x += 2.0f * (float)M_PI;
    return x;
}

static float nearest_pi4(float phi) {
    const float pts[4] = {
        (float)( M_PI / 4.0),
        (float)( 3.0 * M_PI / 4.0),
        (float)(-3.0 * M_PI / 4.0),
        (float)(-M_PI / 4.0)
    };
    float best_e = 1e9f;
    float best   = pts[0];
    int i;
    for (i = 0; i < 4; ++i) {
        float e = fabsf(wrap_pi(phi - pts[i]));
        if (e < best_e) { best_e = e; best = pts[i]; }
    }
    return best;
}

static void phase_to_bits(float target, uint8_t* b0, uint8_t* b1) {
    const float eps = 1e-3f;
    if (fabsf(target - (float)( M_PI / 4.0)) < eps) { *b0=0; *b1=0; }
    else if (fabsf(target - (float)(3.0*M_PI/4.0)) < eps) { *b0=0; *b1=1; }
    else if (fabsf(target + (float)(3.0*M_PI/4.0)) < eps) { *b0=1; *b1=1; }
    else                                                    { *b0=1; *b1=0; }
}

/* ── DSP lifecycle ────────────────────────────────────────────────────── */

static void teardown_dsp(VdesBurstDemodCtx* ctx) {
    if (ctx->resamp)  { msresamp_crcf_destroy(ctx->resamp);   ctx->resamp  = NULL; }
    if (ctx->agc)     { agc_crcf_destroy(ctx->agc);           ctx->agc     = NULL; }
    if (ctx->symsync) { symsync_crcf_destroy(ctx->symsync);   ctx->symsync = NULL; }
    if (ctx->pll)     { nco_crcf_destroy(ctx->pll);           ctx->pll     = NULL; }
    mr_iir_prefilter_destroy(&ctx->prefilter);
    free(ctx->in_buf);    ctx->in_buf    = NULL; ctx->in_cap    = 0;
    free(ctx->resamp_out);ctx->resamp_out= NULL; ctx->resamp_cap= 0;
}

static int ensure_cap(liquid_float_complex** buf, uint32_t* cap, uint32_t need) {
    liquid_float_complex* nb;
    if (need <= *cap) return 1;
    nb = (liquid_float_complex*)realloc(*buf, (size_t)need * sizeof(liquid_float_complex));
    if (!nb) return 0;
    *buf = nb; *cap = need;
    return 1;
}

static int configure(VdesBurstDemodCtx* ctx, uint32_t sr) {
    float rate, cutoff;
    if (!ctx->needs_reconfigure && ctx->sample_rate_hz == sr &&
        ctx->resamp && ctx->agc && ctx->symsync && ctx->pll) return 1;

    teardown_dsp(ctx);
    if (sr == 0) sr = 2048000u;
    ctx->sample_rate_hz = sr;

    rate = (float)ctx->symbol_rate_baud * (float)BD_K / (float)sr;
    if (rate <= 0.0f || rate > 1.0f) return 0;

    ctx->resamp = msresamp_crcf_create(rate, 60.0f);
    if (!ctx->resamp) return 0;

    ctx->agc = agc_crcf_create();
    if (!ctx->agc) return 0;
    agc_crcf_set_bandwidth(ctx->agc, 2e-3f);

    ctx->symsync = symsync_crcf_create_kaiser(BD_K, BD_SYMSYNC_M,
                                               BD_SYMSYNC_BETA, 32u);
    if (!ctx->symsync) return 0;
    symsync_crcf_set_output_rate(ctx->symsync, 1u);
    symsync_crcf_set_lf_bw(ctx->symsync, ctx->sync_bw_preamble);

    ctx->pll = nco_crcf_create(LIQUID_NCO);
    if (!ctx->pll) return 0;
    nco_crcf_pll_set_bandwidth(ctx->pll, ctx->pll_bw_preamble);

    cutoff = 2.0f * (float)ctx->symbol_rate_baud / (float)sr;
    mr_iir_prefilter_destroy(&ctx->prefilter);
    mr_iir_prefilter_create(&ctx->prefilter, cutoff < 0.45f ? cutoff : 0.44f);

    ctx->needs_reconfigure = 0;
    VLOG("configure sr=%u sym=%u rate=%.5f\n", sr, ctx->symbol_rate_baud, (double)rate);
    return 1;
}

/* ── state helpers ────────────────────────────────────────────────────── */

static void reset_to_idle(VdesBurstDemodCtx* ctx) {
    ctx->state         = BD_IDLE;
    ctx->have_prev     = 0;
    ctx->pream_acc_i   = 0.0;
    ctx->pream_acc_q   = 0.0;
    ctx->pream_n       = 0;
    ctx->pream_total   = 0;
    ctx->bit_count     = 0;
    memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
    if (ctx->symsync)
        symsync_crcf_set_lf_bw(ctx->symsync, ctx->sync_bw_preamble);
    if (ctx->pll) {
        nco_crcf_pll_set_bandwidth(ctx->pll, ctx->pll_bw_preamble);
        /* Reset NCO frequency to zero so each burst estimates its own carrier
         * offset from scratch.  Without this, corrections from the previous
         * burst pollute the preamble accumulator of the next one. */
        nco_crcf_set_frequency(ctx->pll, 0.0f);
        nco_crcf_set_phase(ctx->pll, 0.0f);
    }
}

static void emit_bits(VdesBurstDemodCtx* ctx, double freq_hz, uint64_t unix_ms,
                       MrEmitFn emit_fn, void* user_data, const char* reason) {
    char kv[256];
    snprintf(kv, sizeof(kv),
             "{\"signal_type\":\"VDES_BURST_DATA\","
             "\"reason\":\"%s\","
             "\"symbol_rate_baud\":\"%u\","
             "\"bit_count\":\"%u\","
             "\"preamble_syms\":\"%u\","
             "\"freq_err_hz\":\"%.1f\"}",
             reason ? reason : "",
             ctx->symbol_rate_baud,
             (ctx->bit_count / 8u) * 8u,
             ctx->last_pream_syms,
             (double)ctx->last_freq_err_hz);
    mr_emit_bits(ctx->bit_buf, &ctx->bit_count, BD_MIN_BITS,
                 BD_MAX_BITS / 8u + 1u,
                 "VDES_BURST_DATA", kv,
                 freq_hz, unix_ms, emit_fn, user_data);
}

/* ── per-symbol processing ────────────────────────────────────────────── */

static void process_sym(VdesBurstDemodCtx* ctx,
                         liquid_float_complex sym,
                         float pre_agc_energy,   /* avg per-sample energy BEFORE AGC */
                         double freq_hz, uint64_t unix_ms,
                         MrEmitFn emit_fn, void* user_data) {
    liquid_float_complex corrected;
    float phi;
    int falling;

    /* carrier correction */
    nco_crcf_mix_down(ctx->pll, sym, &corrected);
    nco_crcf_step(ctx->pll);

    /* signal gate — use pre-AGC energy so AGC normalisation doesn't mask noise */
    falling = mr_signal_gate_update(&ctx->gate, pre_agc_energy, MR_GATE_HOLD_SYMS);

    if (!ctx->gate.gate_open) {
        if (falling && ctx->bit_count >= BD_MIN_BITS) {
            VLOG("gate-close emit bits=%u pream=%u freq_err=%.1fHz\n",
                 ctx->bit_count, ctx->last_pream_syms,
                 (double)ctx->last_freq_err_hz);
            emit_bits(ctx, freq_hz, unix_ms, emit_fn, user_data, "gate-close");
        }
        if (falling) reset_to_idle(ctx);
        return;
    }

    /* gate just opened → start preamble phase */
    if (ctx->state == BD_IDLE) {
        ctx->state = BD_PREAMBLE;
        ctx->gate_opens++;
        VLOG("gate open — preamble phase\n");
    }

    /* first symbol: just store as reference, no differential yet */
    if (!ctx->have_prev) {
        ctx->prev_sym = corrected;
        ctx->have_prev = 1;
        return;
    }

    /* Differential phase — compute d ONCE and reuse for both phi and accumulator.
       Save prev_sym BEFORE updating so d is conj(prev)·curr, not |curr|². */
    {
        liquid_float_complex d = conjf(ctx->prev_sym) * corrected;
        phi = atan2f(cimagf(d), crealf(d));

        if (ctx->state == BD_PREAMBLE) {
            /* Accumulate ALL symbols during preamble — no per-symbol filter.
               The VDES preamble is ALL +π/4 transitions by design, so the
               vector average of d gives exp(j·(π/4 + carrier_offset_per_sym)).
               Works even for large carrier offsets. */
            ctx->pream_acc_i += crealf(d);
            ctx->pream_acc_q += cimagf(d);
            ctx->pream_n++;
        }
    }
    /* Update reference for next symbol (after d is computed). */
    ctx->prev_sym = corrected;
    ctx->pream_total++;

    if (ctx->state == BD_PREAMBLE) {
        /* PLL active during preamble — drives toward nearest constellation. */
        {
            float tgt = nearest_pi4(phi);
            float err = wrap_pi(phi - tgt);
            nco_crcf_pll_step(ctx->pll, 0.25f * err);
        }

        /* Transition after preamble_min_syms or preamble_max_syms symbols. */
        if (ctx->pream_n >= ctx->preamble_min_syms ||
            ctx->pream_total >= ctx->preamble_max_syms) {

            /* One-shot carrier correction from averaged preamble phase.
             * excess_per_sym is in rad/symbol.  The NCO is stepped once per
             * symbol output (not once per input sample), so the correction
             * unit is rad/step = rad/symbol — no BD_K division needed. */
            float avg_phase = atan2f((float)ctx->pream_acc_q,
                                     (float)ctx->pream_acc_i);
            float excess_per_sym = wrap_pi(avg_phase - (float)(-3.0 * M_PI / 4.0));
            /* corrected = sym × exp(-j·ω_nco·n).  Residual carrier per symbol =
             * ω_c - ω_nco.  excess_per_sym estimates this residual.  To cancel
             * it we must increase ω_nco by +excess, not decrease it. */
            nco_crcf_adjust_frequency(ctx->pll, +excess_per_sym);

            ctx->last_freq_err_hz  = excess_per_sym *
                                     (float)ctx->symbol_rate_baud / (2.0f * (float)M_PI);
            ctx->last_pream_syms   = ctx->pream_n;

            VLOG("preamble→demod: n=%u avg_phase=%.3f freq_err=%.1fHz\n",
                 ctx->pream_n, (double)avg_phase, (double)ctx->last_freq_err_hz);

            /* Freeze symbol-sync loop. */
            symsync_crcf_set_lf_bw(ctx->symsync, BD_SYNC_BW_FROZEN);
            ctx->state = BD_DEMOD;
        }

    } else { /* BD_DEMOD — PLL frozen, carrier correction applied */
        float tgt = nearest_pi4(phi);
        uint8_t b0, b1;
        phase_to_bits(tgt, &b0, &b1);
        mr_push_bit(ctx->bit_buf, &ctx->bit_count, BD_MAX_BITS, b0);
        mr_push_bit(ctx->bit_buf, &ctx->bit_count, BD_MAX_BITS, b1);

        if (ctx->bit_count >= BD_MAX_BITS) {
            emit_bits(ctx, freq_hz, unix_ms, emit_fn, user_data, "buf-full");
        }
    }
}

/* ── Plugin API ───────────────────────────────────────────────────────── */

static const MrPluginMeta kMeta = {
    "vdes_burst_demod",
    "0.1.0",
    MR_PLUGIN_API_VERSION,
    "VDES ASM burst-mode pi/4-DQPSK demod with preamble acquisition + frozen payload",
    MR_PLUGIN_ROLE_DEMODULATOR
};

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

MrPluginCtx* mr_plugin_create(void) {
    VdesBurstDemodCtx* ctx = (VdesBurstDemodCtx*)calloc(1, sizeof(*ctx));
    if (!ctx) return NULL;

    ctx->symbol_rate_baud  = BD_DEFAULT_SYM_RATE;
    ctx->pll_bw_preamble   = BD_PLL_BW_PREAMBLE;
    ctx->sync_bw_preamble  = BD_SYNC_BW_PREAMBLE;
    ctx->preamble_min_syms = BD_PREAMBLE_MIN_SYMS;
    ctx->preamble_max_syms = BD_PREAMBLE_MAX_SYMS;
    ctx->preamble_tol_rad  = BD_PREAMBLE_TOL_DEG * (float)M_PI / 180.0f;
    ctx->needs_reconfigure = 1;
    ctx->state             = BD_IDLE;

    mr_iir_prefilter_init(&ctx->prefilter);
    mr_signal_gate_init(&ctx->gate, MR_GATE_SQUELCH_RATIO);
    return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
    VdesBurstDemodCtx* ctx = (VdesBurstDemodCtx*)raw;
    if (!ctx) return;
    teardown_dsp(ctx);
    mr_iir_prefilter_destroy(&ctx->prefilter);
    free(ctx);
}

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
    VdesBurstDemodCtx* ctx = (VdesBurstDemodCtx*)raw;
    if (!ctx || !key || !value) return 0;

    if (strcmp(key, "symbol_rate_baud") == 0) {
        const int v = atoi(value);
        if (v >= 1000 && v <= 500000) {
            ctx->symbol_rate_baud = (uint32_t)v;
            ctx->needs_reconfigure = 1; return 1;
        }
        return 0;
    }
    if (strcmp(key, "pll_bw") == 0) {
        const float v = (float)atof(value);
        if (v > 0.0f && v < 0.5f) {
            ctx->pll_bw_preamble = v; return 1;
        }
        return 0;
    }
    if (strcmp(key, "preamble_min_syms") == 0) {
        const int v = atoi(value);
        if (v >= 4 && v <= 512) { ctx->preamble_min_syms = (uint32_t)v; return 1; }
        return 0;
    }
    if (strcmp(key, "preamble_max_syms") == 0) {
        const int v = atoi(value);
        if (v >= 8 && v <= 1024) { ctx->preamble_max_syms = (uint32_t)v; return 1; }
        return 0;
    }
    if (strcmp(key, "preamble_tol_deg") == 0) {
        const float v = (float)atof(value);
        if (v > 0.0f && v < 90.0f) {
            ctx->preamble_tol_rad = v * (float)M_PI / 180.0f; return 1;
        }
        return 0;
    }
    if (strcmp(key, "squelch_db") == 0) {
        const float db = (float)atof(value);
        ctx->gate.squelch_ratio = (db <= 0.0f) ? 0.0f : powf(10.0f, db / 10.0f);
        return 1;
    }
    return 0;
}

void mr_plugin_process_bits(MrPluginCtx* ctx,
                             const uint8_t* bb, uint32_t bc,
                             double fhz, uint64_t ms,
                             const char* st, MrEmitFn ef, void* ud) {
    (void)ctx; (void)bb; (void)bc; (void)fhz; (void)ms;
    (void)st; (void)ef; (void)ud;
}

void mr_plugin_process_iq(MrPluginCtx* raw,
                           const int16_t* iq, uint32_t num_pairs,
                           uint32_t sr, double freq_hz, uint64_t unix_ms,
                           MrEmitFn emit_fn, void* user_data) {
    VdesBurstDemodCtx* ctx = (VdesBurstDemodCtx*)raw;
    uint32_t i;
    unsigned int nr = 0u;

    if (!ctx || !iq || num_pairs == 0u) return;
    if (!configure(ctx, sr)) return;
    ctx->blocks_seen++;

    if (!ensure_cap(&ctx->in_buf, &ctx->in_cap, num_pairs)) return;

    /* Convert int16 → float complex and apply prefilter */
    for (i = 0u; i < num_pairs; ++i) {
        liquid_float_complex x;
        __real__ x = (float)iq[i * 2u] / 32768.0f;
        __imag__ x = (float)iq[i * 2u + 1u] / 32768.0f;
        mr_iir_prefilter_execute(&ctx->prefilter, x, &ctx->in_buf[i]);
    }

    /* Resample to K×symbol_rate */
    {
        float rate = (float)ctx->symbol_rate_baud * (float)BD_K / (float)sr;
        uint32_t need = (uint32_t)((float)num_pairs * rate + 64.0f);
        if (need < 128u) need = 128u;
        if (!ensure_cap(&ctx->resamp_out, &ctx->resamp_cap, need)) return;
    }
    msresamp_crcf_execute(ctx->resamp, ctx->in_buf, num_pairs, ctx->resamp_out, &nr);

    /* AGC and symbol sync.
     * Pre-AGC energy is accumulated over the samples that correspond to each
     * output symbol so the gate can distinguish signal from noise even though
     * AGC normalises amplitude to unity. */
    {
        float pre_energy_acc = 0.0f;
        uint32_t pre_energy_n = 0u;
        for (i = 0u; i < nr; ++i) {
            liquid_float_complex agc_in = ctx->resamp_out[i];
            float re = crealf(agc_in), im = cimagf(agc_in);
            pre_energy_acc += re*re + im*im;
            pre_energy_n++;

            liquid_float_complex agc_out;
            unsigned int nsym = 0u;
            agc_crcf_execute(ctx->agc, agc_in, &agc_out);
            symsync_crcf_execute(ctx->symsync, &agc_out, 1u, ctx->sym_buf, &nsym);

            for (unsigned int s = 0u; s < nsym; ++s) {
                float sym_e = (pre_energy_n > 0u)
                            ? pre_energy_acc / (float)pre_energy_n
                            : 0.0f;
                pre_energy_acc = 0.0f;
                pre_energy_n   = 0u;
                process_sym(ctx, ctx->sym_buf[s], sym_e,
                            freq_hz, unix_ms, emit_fn, user_data);
            }
        }
    }

    if (vdbg() && (ctx->blocks_seen % 200u) == 0u) {
        VLOG("blocks=%llu gate=%d state=%d bits=%u gate_opens=%llu\n",
             (unsigned long long)ctx->blocks_seen,
             ctx->gate.gate_open, (int)ctx->state,
             ctx->bit_count,
             (unsigned long long)ctx->gate_opens);
    }
}
