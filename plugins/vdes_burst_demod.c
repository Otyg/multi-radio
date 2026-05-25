/**
 * vdes_burst_demod.c — VDES VDE-TER pi/4-DQPSK mottagare
 *
 * Roll: MR_PLUGIN_ROLE_DEMODULATOR
 * Sänder: VDES_BURST_DATA → vdes_burst_decoder
 *
 * All DSP delegeras till libliquid i enlighet med vdes.md-planen:
 *
 *   msresamp_crcf  — Rational resampler + antialias-filter
 *   agc_crcf       — AGC, normaliserar amplituden till 1
 *   symsync_crcf   — Polyfas RRC matchat filter + timingreglering
 *   nco_crcf       — NCO/PLL för bärvågskorrigering
 *   firfilt_cccf   — Matchat filter (27 komplexa tappar) för UW-detektion
 *   modemcf        — Dataväg A (hårda bitar) via modemcf_demodulate()
 *                    Dataväg B (mjuka bitar / LLR) via modemcf_demodulate_soft()
 *                    PLL-felsignal via modemcf_get_demodulator_phase_error()
 *
 * Tillståndsmaskin (vdes.md):
 *   VR_SEARCH  — firfilt_cccf korskorreleras mot träningssekvensens
 *                27 differentialfasorer.  Triggning när |C| > tröskel.
 *                Bärvågsoffset δω = arg(C) (oberoende av amplitud).
 *   VR_BURST   — modemcf demodulerar differentialsymboler till
 *                hårda bitar (dataväg A).  PLL-felet från modemcf
 *                spårar kvarstående frekvensavvikelse.
 *                (Dataväg B / LLR aktiveras vid integration av Aff3ct.)
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

/* ── Konstanter (speglar BD_K/BD_SYMSYNC_* i vdes_synth_liq.c) ─────────── */
#define VR_DEFAULT_SYM_RATE  76800u
#define VR_K                 4u       /* sampler/symbol in symsync            */
#define VR_SYMSYNC_M         5u       /* filter half-length                   */
#define VR_SYMSYNC_BETA      0.35f    /* RRC roll-off                         */
#define VR_UW_LEN            27u      /* träningssekvensens längd i symboler  */
#define VR_CORR_THRESH_FRAC  0.80f    /* tröskel = frac × VR_UW_LEN = 21.6   */
#define VR_MAX_BITS          8192u
#define VR_MIN_BITS          64u
#define VR_PLL_BW            0.05f    /* PLL-bandbredd under sökning          */
#define VR_PLL_BW_BURST      0.05f    /* PLL-bandbredd under burst             */
#define VR_SYNC_BW_SEARCH    0.05f   /* AGC locked at gain=1 (amplitude≈0.65); BW scaled up to maintain TED loop gain */
#define VR_SYNC_BW_LOCKED    0.005f  /* slow tracking during burst */
#define VR_LOCKOUT_SYMS      300u

/* ── Träningssekvens (ITU-R M.2092-2, Tabell 1) ─────────────────────────── */
static const uint8_t kTrainBits[VR_UW_LEN] = {
    1,1,1,1,1,1,0,0, 1,1,0,1,0,1,0,0, 0,0,0,1,1,0,0,1, 0,1,0
};

/* ── Tillståndsmaskin ────────────────────────────────────────────────────── */
typedef enum { VR_SEARCH, VR_BURST } VdesRxState;

/* ── Kontext ─────────────────────────────────────────────────────────────── */
typedef struct {
    /* libliquid DSP-kedja */
    msresamp_crcf   resamp;
    agc_crcf        agc;
    symsync_crcf    symsync;
    nco_crcf        pll;          /* bärvågskorrigering (NCO + PLL)  */
    firfilt_cccf    uw_mf;        /* matchat filter, 27 komplexa tappar */
    modemcf         modem;        /* QPSK-demodulator (hårda/mjuka bitar) */
    MrIirPrefilter  prefilter;
    MrSignalGate    gate;         /* energigrind för burst-SLUT-detektion */

    /* Buffertar */
    liquid_float_complex* in_buf;
    uint32_t              in_cap;
    liquid_float_complex* resamp_out;
    uint32_t              resamp_cap;
    liquid_float_complex  sym_buf[VR_K];

    /* Symbolstatus */
    liquid_float_complex prev_sym;
    int                  have_prev;

    /* Ringbuffert: senaste VR_UW_LEN normaliserade differentialfasorer */
    liquid_float_complex diff_ring[VR_UW_LEN];
    uint32_t             diff_head;   /* nästa skrivindex (mod VR_UW_LEN) */
    uint32_t             diff_count;

    /* Tillståndsmaskin */
    VdesRxState state;
    uint32_t    lockout_syms;
    float       last_freq_err_hz;
    float       freq_ema;           /* EMA av δω-estimat över burstar */

    /* Bitbuffert */
    uint8_t  bit_buf[VR_MAX_BITS / 8u + 4u];
    uint32_t bit_count;

    /* Konfiguration */
    uint32_t sample_rate_hz;
    uint32_t symbol_rate_baud;
    float    corr_threshold;
    int      needs_reconfigure;

    /* Diagnostik */
    uint64_t blocks_seen;
    uint64_t bursts_detected;
} VdesRxCtx;

/* ── Debug ────────────────────────────────────────────────────────────────── */
static int vdbg(void) {
    static int v = -1;
    if (v < 0) { const char* e = getenv("MR_AIS_DEBUG"); v = (e && e[0] != '0'); }
    return v;
}
#define VLOG(...) do { if (vdbg()) fprintf(stderr, "[vdes_burst_demod] " __VA_ARGS__); } while(0)

/* ── Matchat filter för UW-detektion ─────────────────────────────────────── */
/*
 * Skapar firfilt_cccf med tappar h_k = conj(d_{N-1-k}) där
 * d_m = exp(j·Δθ_m) är m:te träningssymbolens differentialfasor.
 *
 * Vid korrekt justering: |C| = N (oberoende av δω),  arg(C) = δω.
 * Detta ger direkt bärvågsuppskattning utan separat PLL-inlåsning.
 */
static firfilt_cccf build_uw_mf(void) {
    liquid_float_complex taps[VR_UW_LEN];
    unsigned k;
    for (k = 0; k < VR_UW_LEN; ++k) {
        float dp = kTrainBits[VR_UW_LEN - 1u - k]
                   ? (float)(-3.0 * M_PI / 4.0)
                   : (float)(     M_PI / 4.0);
        __real__ taps[k] =  cosf(dp);   /* conj(exp(j·dp)) = exp(-j·dp) */
        __imag__ taps[k] = -sinf(dp);
    }
    return firfilt_cccf_create(taps, VR_UW_LEN);
}

/* ── DSP-livscykel ────────────────────────────────────────────────────────── */
static void teardown_dsp(VdesRxCtx* ctx) {
    if (ctx->resamp)  { msresamp_crcf_destroy(ctx->resamp);   ctx->resamp  = NULL; }
    if (ctx->agc)     { agc_crcf_destroy(ctx->agc);           ctx->agc     = NULL; }
    if (ctx->symsync) { symsync_crcf_destroy(ctx->symsync);   ctx->symsync = NULL; }
    if (ctx->pll)     { nco_crcf_destroy(ctx->pll);           ctx->pll     = NULL; }
    if (ctx->uw_mf)   { firfilt_cccf_destroy(ctx->uw_mf);     ctx->uw_mf   = NULL; }
    if (ctx->modem)   { modemcf_destroy(ctx->modem);          ctx->modem   = NULL; }
    mr_iir_prefilter_destroy(&ctx->prefilter);
    free(ctx->in_buf);     ctx->in_buf     = NULL; ctx->in_cap     = 0;
    free(ctx->resamp_out); ctx->resamp_out = NULL; ctx->resamp_cap = 0;
}

static int ensure_cap(liquid_float_complex** buf, uint32_t* cap, uint32_t need) {
    if (need <= *cap) return 1;
    liquid_float_complex* nb = (liquid_float_complex*)
        realloc(*buf, (size_t)need * sizeof(liquid_float_complex));
    if (!nb) return 0;
    *buf = nb; *cap = need;
    return 1;
}

static int configure(VdesRxCtx* ctx, uint32_t sr) {
    float rate, cutoff;
    if (!ctx->needs_reconfigure && ctx->sample_rate_hz == sr) return 1;
    ctx->sample_rate_hz = sr;
    teardown_dsp(ctx);

    rate = (float)ctx->symbol_rate_baud * (float)VR_K / (float)sr;

    ctx->resamp = msresamp_crcf_create(rate, 60.0f);
    if (!ctx->resamp) return 0;

    ctx->agc = agc_crcf_create();
    agc_crcf_set_bandwidth(ctx->agc, 2e-3f);
    agc_crcf_lock(ctx->agc);   /* lock immediately — keeps gain=1 during initial silence */

    ctx->symsync = symsync_crcf_create_rnyquist(LIQUID_FIRFILT_RRC, VR_K, VR_SYMSYNC_M, VR_SYMSYNC_BETA, 32u);
    if (!ctx->symsync) return 0;
    symsync_crcf_set_output_rate(ctx->symsync, 1.0f);
    symsync_crcf_set_lf_bw(ctx->symsync, VR_SYNC_BW_SEARCH);

    ctx->pll = nco_crcf_create(LIQUID_NCO);
    if (!ctx->pll) return 0;
    nco_crcf_pll_set_bandwidth(ctx->pll, VR_PLL_BW);

    ctx->uw_mf = build_uw_mf();
    if (!ctx->uw_mf) return 0;

    /*
     * QPSK-modem för differentialsymboldemodulering (dataväg A och B).
     *
     * Differentialsymbolen d = conj(s_{k-1}) × s_k är ett QPSK-symbol
     * vars konstellationspunkter matchar pi/4-DQPSK:s differentialfaser
     * {+π/4, +3π/4, -3π/4, -π/4}.  libliquid QPSK använder samma
     * Gray-kodade 4-punktskonstellaton.
     *
     * Bitextraktion (VDES-konvention, ITU-R M.2092-2):
     *   sym=0 → +π/4  → (b0=0, b1=0)
     *   sym=1 → +3π/4 → (b0=0, b1=1)
     *   sym=2 → -π/4  → (b0=1, b1=0)
     *   sym=3 → -3π/4 → (b0=1, b1=1)
     * → b0 = (sym >> 1) & 1,  b1 = sym & 1
     *
     * Fasfel från modemcf_get_demodulator_phase_error() matas direkt
     * till nco_crcf_pll_step() för spårning av kvarstående δω.
     */
    ctx->modem = modemcf_create(LIQUID_MODEM_QPSK);
    if (!ctx->modem) return 0;

    ctx->corr_threshold = VR_CORR_THRESH_FRAC * (float)VR_UW_LEN;

    cutoff = 2.0f * (float)ctx->symbol_rate_baud / (float)sr;
    mr_iir_prefilter_destroy(&ctx->prefilter);
    mr_iir_prefilter_create(&ctx->prefilter, cutoff < 0.45f ? cutoff : 0.44f);

    ctx->needs_reconfigure = 0;
    VLOG("configure sr=%u sym=%u rate=%.5f corr_thr=%.2f\n",
         sr, ctx->symbol_rate_baud, (double)rate, (double)ctx->corr_threshold);
    return 1;
}

/* ── Tillståndshjälpare ──────────────────────────────────────────────────── */
static void reset_to_search(VdesRxCtx* ctx) {
    ctx->state         = VR_SEARCH;
    ctx->have_prev     = 0;
    ctx->diff_head     = 0;
    ctx->diff_count    = 0;
    ctx->bit_count     = 0;
    memset(ctx->bit_buf, 0, sizeof(ctx->bit_buf));
    if (ctx->uw_mf)   firfilt_cccf_reset(ctx->uw_mf);
    if (ctx->modem)   modemcf_reset(ctx->modem);
    if (ctx->symsync) {
        symsync_crcf_reset(ctx->symsync);
        symsync_crcf_set_lf_bw(ctx->symsync, VR_SYNC_BW_SEARCH);
    }
    if (ctx->agc) {
        agc_crcf_reset(ctx->agc);  /* gain=1, unlocked */
        agc_crcf_lock(ctx->agc);   /* lock immediately — prevents gain from growing toward 1e6 during silence */
    }
    if (ctx->pll) {
        nco_crcf_pll_set_bandwidth(ctx->pll, VR_PLL_BW);
        nco_crcf_set_frequency(ctx->pll, 0.0f);
        nco_crcf_set_phase(ctx->pll,    0.0f);
    }
}

static void emit_bits(VdesRxCtx* ctx, double freq_hz, uint64_t unix_ms,
                      MrEmitFn emit_fn, void* user_data, const char* reason) {
    char kv[256];
    snprintf(kv, sizeof(kv),
             "{\"signal_type\":\"VDES_BURST_DATA\","
             "\"reason\":\"%s\","
             "\"symbol_rate_baud\":\"%u\","
             "\"bit_count\":\"%u\","
             "\"freq_err_hz\":\"%.1f\"}",
             reason ? reason : "",
             ctx->symbol_rate_baud,
             (ctx->bit_count / 8u) * 8u,
             (double)ctx->last_freq_err_hz);
    mr_emit_bits(ctx->bit_buf, &ctx->bit_count, VR_MIN_BITS,
                 VR_MAX_BITS / 8u + 1u,
                 "VDES_BURST_DATA", kv, freq_hz, unix_ms, emit_fn, user_data);
}

/* Emit the known training sequence directly (avoids ring-buffer timing issues). */
static void emit_training_bits(VdesRxCtx* ctx) {
    unsigned k;
    for (k = 0; k < VR_UW_LEN; ++k) {
        uint8_t b = kTrainBits[k];
        mr_push_bit(ctx->bit_buf, &ctx->bit_count, VR_MAX_BITS, b);
        mr_push_bit(ctx->bit_buf, &ctx->bit_count, VR_MAX_BITS, b);
    }
    modemcf_reset(ctx->modem);
}

/* ── Per-symbol-bearbetning ──────────────────────────────────────────────── */
static void process_sym(VdesRxCtx* ctx,
                        liquid_float_complex sym,
                        float pre_agc_energy,
                        double freq_hz, uint64_t unix_ms,
                        MrEmitFn emit_fn, void* user_data) {
    liquid_float_complex corrected, diff, mf_out;
    float mf_mag;

    /* Bärvågskorrigering via nco_crcf */
    nco_crcf_mix_down(ctx->pll, sym, &corrected);
    nco_crcf_step(ctx->pll);

    /* Energigrind — burst-SLUT-detektion */
    {
        int falling = mr_signal_gate_update(&ctx->gate, pre_agc_energy, 48u);
        if (falling && ctx->state == VR_BURST && ctx->bit_count >= VR_MIN_BITS) {
            VLOG("burst slut (energifall) bits=%u freq_err=%.1fHz\n",
                 ctx->bit_count, (double)ctx->last_freq_err_hz);
            emit_bits(ctx, freq_hz, unix_ms, emit_fn, user_data, "gate-close");
            reset_to_search(ctx);
            return;
        }
        if (falling && ctx->state == VR_BURST) {
            reset_to_search(ctx);
            return;
        }
    }

    if (ctx->lockout_syms > 0) { ctx->lockout_syms--; return; }

    /* Referenssymbol — ingen differential vid första symbolen */
    if (!ctx->have_prev) {
        ctx->prev_sym  = corrected;
        ctx->have_prev = 1;
        return;
    }

    /*
     * Differentialfasor normaliserad till enhetsmagnitud.
     * Normaliseringen är avgörande: AGC-transienter (t.ex. efter tystnad)
     * ger |d| >> 1 vilket annars blåser upp MF-utmatningens magnitud.
     * Fasen — som bär data och bärvågsavvikelse — bevaras exakt.
     */
    {
        liquid_float_complex d_raw = conjf(ctx->prev_sym) * corrected;
        float d_mag = cabsf(d_raw);
        ctx->prev_sym = corrected;
        if (d_mag > 1e-6f) {
            __real__ diff = crealf(d_raw) / d_mag;
            __imag__ diff = cimagf(d_raw) / d_mag;
        } else {
            __real__ diff = 1.0f; __imag__ diff = 0.0f;
        }
    }

    /* Lagra i ringbuffert (normaliserad) */
    ctx->diff_ring[ctx->diff_head] = diff;
    ctx->diff_head = (ctx->diff_head + 1u) % VR_UW_LEN;
    if (ctx->diff_count < VR_UW_LEN) ctx->diff_count++;

    /* Mata matchat filter */
    firfilt_cccf_push(ctx->uw_mf, diff);
    firfilt_cccf_execute(ctx->uw_mf, &mf_out);
    mf_mag = cabsf(mf_out);

    if (ctx->state == VR_SEARCH) {
        /* Rising-edge UW detection: trigger at the exact symbol where
         * diff_count first reaches VR_UW_LEN and |C| exceeds threshold.
         * At that moment the ring holds all 27 training symbols in order.
         * Triggering here (not on the falling edge) avoids a ~25% miss-rate
         * caused by the first data symbol having the same differential phase
         * as training[0], which would delay the falling edge by one symbol
         * and shift the emitted training sequence by 2 bits. */
        if (ctx->diff_count >= VR_UW_LEN && mf_mag > ctx->corr_threshold) {
            /* |C| ≈ N (amplitude-independent), arg(C) = δω (rad/symbol). */
            float delta_omega = cargf(mf_out);
            ctx->last_freq_err_hz = delta_omega
                                    * (float)ctx->symbol_rate_baud
                                    / (2.0f * (float)M_PI);
            ctx->bursts_detected++;

            VLOG("UW detekterat |C|=%.2f/%.0f δω=%.4f freq_err=%.1fHz agc_gain=%.2f\n",
                 (double)mf_mag, (double)VR_UW_LEN,
                 (double)delta_omega, (double)ctx->last_freq_err_hz,
                 (double)agc_crcf_get_gain(ctx->agc));

            /* EMA av δω-estimat: reducerar brus från payload-beroende SNR-bias.
             * Burst 1: initialisera EMA direkt. Burst 2+: hälften old, hälften new. */
            if (ctx->bursts_detected == 1) {
                ctx->freq_ema = delta_omega;
            } else {
                ctx->freq_ema = 0.75f * ctx->freq_ema + 0.25f * delta_omega;
            }
            VLOG("freq_ema=%.4f (%.1fHz)\n",
                 (double)ctx->freq_ema,
                 (double)(ctx->freq_ema * (float)ctx->symbol_rate_baud / (2.0f * (float)M_PI)));
            nco_crcf_set_frequency(ctx->pll, ctx->freq_ema);
            nco_crcf_set_phase(ctx->pll,    ctx->freq_ema);
            nco_crcf_pll_set_bandwidth(ctx->pll, VR_PLL_BW_BURST);

            symsync_crcf_set_lf_bw(ctx->symsync, VR_SYNC_BW_LOCKED);
            /* AGC stays locked at gain=1 — no unlock; see reset_to_search for rationale */

            /* Emit the known training bits directly — no ring-buffer needed. */
            emit_training_bits(ctx);

            ctx->state = VR_BURST;
            /* Do NOT fall through: current symbol IS training[26] (already
             * emitted above). The next call to process_sym receives LID[0]. */
            return;
        }
    }

    if (ctx->state == VR_BURST) {
        /* Dataväg A: hård demodulering via modemcf_demodulate().
         * Bitextraktion: b0 = MSB, b1 = LSB (VDES-konvention ITU-R M.2092-2). */
        unsigned int sym;
        modemcf_demodulate(ctx->modem, diff, &sym);

        /* PLL-feedback under burst: korrigerar kvarstående frekvensavvikelse
         * som den initiala δω-uppskattningen från MF kan ha missat. */
        nco_crcf_pll_step(ctx->pll, modemcf_get_demodulator_phase_error(ctx->modem));

        uint8_t b0 = (sym >> 1) & 1u;
        uint8_t b1 =  sym       & 1u;

        mr_push_bit(ctx->bit_buf, &ctx->bit_count, VR_MAX_BITS, b0);
        mr_push_bit(ctx->bit_buf, &ctx->bit_count, VR_MAX_BITS, b1);

        if (vdbg() && ctx->bit_count > 54u && (ctx->bit_count % 100u) == 0u)
            VLOG("tau bit=%u sym=%u: %.4f\n",
                 ctx->bit_count, (ctx->bit_count - 54u) / 2u,
                 (double)symsync_crcf_get_tau(ctx->symsync));

        if (ctx->bit_count >= VR_MAX_BITS) {
            VLOG("burst slut (buffert full) bits=%u\n", ctx->bit_count);
            emit_bits(ctx, freq_hz, unix_ms, emit_fn, user_data, "buf-full");
            ctx->lockout_syms = VR_LOCKOUT_SYMS;
            reset_to_search(ctx);
        }
    }
}

/* ── Plugin API ──────────────────────────────────────────────────────────── */
static const MrPluginMeta kMeta = {
    "vdes_burst_demod", "0.3.0", MR_PLUGIN_API_VERSION,
    "VDES VDE-TER: UW-korskorrelation + modemcf hard/soft bits (libliquid)",
    MR_PLUGIN_ROLE_DEMODULATOR
};

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

MrPluginCtx* mr_plugin_create(void) {
    VdesRxCtx* ctx = (VdesRxCtx*)calloc(1, sizeof(*ctx));
    if (!ctx) return NULL;
    ctx->symbol_rate_baud  = VR_DEFAULT_SYM_RATE;
    ctx->needs_reconfigure = 1;
    ctx->state             = VR_SEARCH;
    mr_iir_prefilter_init(&ctx->prefilter);
    mr_signal_gate_init(&ctx->gate, MR_GATE_SQUELCH_RATIO);
    return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
    VdesRxCtx* ctx = (VdesRxCtx*)raw;
    if (!ctx) return;
    teardown_dsp(ctx);
    mr_iir_prefilter_destroy(&ctx->prefilter);
    free(ctx);
}

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
    VdesRxCtx* ctx = (VdesRxCtx*)raw;
    if (!ctx || !key || !value) return 0;
    if (strcmp(key, "symbol_rate_baud") == 0) {
        int v = atoi(value);
        if (v >= 1000 && v <= 500000) {
            ctx->symbol_rate_baud = (uint32_t)v;
            ctx->needs_reconfigure = 1; return 1;
        }
        return 0;
    }
    if (strcmp(key, "squelch_db") == 0) {
        float db = (float)atof(value);
        ctx->gate.squelch_ratio = (db <= 0.0f) ? 0.0f : powf(10.0f, db / 10.0f);
        return 1;
    }
    if (strcmp(key, "corr_threshold") == 0) {
        float v = (float)atof(value);
        if (v > 0.0f && v <= (float)VR_UW_LEN) { ctx->corr_threshold = v; return 1; }
        return 0;
    }
    return 0;
}

void mr_plugin_process_bits(MrPluginCtx* ctx,
                             const uint8_t* bb, uint32_t bc,
                             double fhz, uint64_t ms,
                             const char* st, MrEmitFn ef, void* ud) {
    (void)ctx; (void)bb; (void)bc; (void)fhz; (void)ms;
    (void)st;  (void)ef; (void)ud;
}

void mr_plugin_process_iq(MrPluginCtx* raw,
                           const int16_t* iq, uint32_t num_pairs,
                           uint32_t sr, double freq_hz, uint64_t unix_ms,
                           MrEmitFn emit_fn, void* user_data) {
    VdesRxCtx* ctx = (VdesRxCtx*)raw;
    uint32_t i;
    unsigned int nr = 0u;

    if (!ctx || !iq || num_pairs == 0u) return;
    if (!configure(ctx, sr)) return;
    ctx->blocks_seen++;

    if (!ensure_cap(&ctx->in_buf, &ctx->in_cap, num_pairs)) return;

    /* int16 → komplex float + IIR-förfiltrering */
    for (i = 0u; i < num_pairs; ++i) {
        liquid_float_complex x;
        __real__ x = (float)iq[i * 2u]     / 32768.0f;
        __imag__ x = (float)iq[i * 2u + 1u] / 32768.0f;
        mr_iir_prefilter_execute(&ctx->prefilter, x, &ctx->in_buf[i]);
    }

    /* Resampla till VR_K × symbol_rate via msresamp_crcf */
    {
        float rate = (float)ctx->symbol_rate_baud * (float)VR_K / (float)sr;
        uint32_t need = (uint32_t)((float)num_pairs * rate + 64.0f);
        if (need < 128u) need = 128u;
        if (!ensure_cap(&ctx->resamp_out, &ctx->resamp_cap, need)) return;
        msresamp_crcf_execute(ctx->resamp, ctx->in_buf, num_pairs,
                               ctx->resamp_out, &nr);
    }

    /* AGC → symsync → per-symbol-bearbetning */
    for (i = 0u; i < nr; ++i) {
        liquid_float_complex agc_in  = ctx->resamp_out[i];
        liquid_float_complex agc_out;
        unsigned int         nsym   = 0u;
        float re = crealf(agc_in), im = cimagf(agc_in);
        float pre_energy = re * re + im * im;

        agc_crcf_execute(ctx->agc, agc_in, &agc_out);
        symsync_crcf_execute(ctx->symsync, &agc_out, 1u, ctx->sym_buf, &nsym);

        for (unsigned int s = 0u; s < nsym; ++s) {
            process_sym(ctx, ctx->sym_buf[s], pre_energy,
                        freq_hz, unix_ms, emit_fn, user_data);
        }
    }

    /* Periodisk diagnostik */
    if (vdbg() && ctx->blocks_seen % 200u == 0u) {
        VLOG("blocks=%llu bursts=%llu state=%s bits=%u\n",
             (unsigned long long)ctx->blocks_seen,
             (unsigned long long)ctx->bursts_detected,
             ctx->state == VR_SEARCH ? "SEARCH" : "BURST",
             ctx->bit_count);
    }
}
