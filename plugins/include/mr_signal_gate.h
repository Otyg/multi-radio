/**
 * mr_signal_gate.h — Energy-based signal gate for demodulator plugins.
 *
 * Estimates the noise floor with a slow EMA and tracks instantaneous signal
 * energy with a fast EMA.  The gate opens when the fast EMA exceeds the noise
 * floor by squelch_ratio, and stays open for hold_syms symbols after the
 * energy drops (preventing the end of a frame from being clipped).
 *
 * No libliquid dependency; plain C99.
 */

#pragma once

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Default tunables                                                     */
/* ------------------------------------------------------------------ */

#define MR_GATE_SQUELCH_RATIO  4.0f    /* gate opens at signal_energy > noise*ratio (6 dB) */
#define MR_GATE_HOLD_SYMS      48u     /* symbols to keep gate open after energy drops */
#define MR_GATE_NOISE_ALPHA    0.001f  /* noise floor EMA alpha (~1000 sym tau) */
#define MR_GATE_ENERGY_ALPHA   0.10f   /* signal energy EMA alpha (~10 sym tau) */
#define MR_GATE_NOISE_INIT     1e-4f   /* initial noise floor estimate */

/* ------------------------------------------------------------------ */
/* State struct                                                         */
/* ------------------------------------------------------------------ */

typedef struct {
    float    noise_floor;    /* slow EMA of per-sample energy in quiet periods */
    float    signal_energy;  /* fast EMA of current per-sample energy */
    float    squelch_ratio;  /* gate opens when signal_energy > noise_floor * ratio */
    uint32_t hold_syms;      /* symbols remaining in hold-off after energy drops */
    int      gate_open;      /* 1 = gate open (demodulate), 0 = gate closed */
} MrSignalGate;

/* ------------------------------------------------------------------ */
/* API                                                                  */
/* ------------------------------------------------------------------ */

static inline void mr_signal_gate_init(MrSignalGate* g, float squelch_ratio) {
    g->noise_floor   = MR_GATE_NOISE_INIT;
    g->signal_energy = 0.0f;
    g->squelch_ratio = (squelch_ratio > 0.0f) ? squelch_ratio : MR_GATE_SQUELCH_RATIO;
    g->hold_syms     = 0u;
    g->gate_open     = 0;
}

/* Reset energy state but keep tuning (call on reconfigure). */
static inline void mr_signal_gate_reset(MrSignalGate* g) {
    g->noise_floor   = MR_GATE_NOISE_INIT;
    g->signal_energy = 0.0f;
    g->hold_syms     = 0u;
    g->gate_open     = 0;
}

/**
 * Update gate state for one symbol group.
 *
 * per_sample_energy  average energy per sample in this symbol period
 *                    (= sum(I²+Q²) / k)
 * hold_syms_default  hold-off count when signal reappears; pass 0 to use the
 *                    MR_GATE_HOLD_SYMS default
 *
 * Returns 1 on the falling edge (gate just closed), 0 otherwise.
 * Check g->gate_open for the current state after the call.
 */
static inline int mr_signal_gate_update(MrSignalGate* g,
                                         float per_sample_energy,
                                         uint32_t hold_syms_default) {
    const uint32_t hold = hold_syms_default ? hold_syms_default : MR_GATE_HOLD_SYMS;

    /* Update noise floor only during quiet periods */
    if (per_sample_energy < g->noise_floor * g->squelch_ratio) {
        g->noise_floor = g->noise_floor * (1.0f - MR_GATE_NOISE_ALPHA)
                       + per_sample_energy * MR_GATE_NOISE_ALPHA;
    }
    if (g->noise_floor < 1e-12f) g->noise_floor = 1e-12f;

    /* Fast signal energy EMA */
    g->signal_energy = g->signal_energy * (1.0f - MR_GATE_ENERGY_ALPHA)
                     + per_sample_energy * MR_GATE_ENERGY_ALPHA;

    /* Threshold and hold-off */
    const int above = (g->squelch_ratio > 0.0f)
                    && (g->signal_energy > g->noise_floor * g->squelch_ratio);
    if (above) {
        g->hold_syms = hold;
    } else if (g->hold_syms > 0u) {
        g->hold_syms--;
    }

    const int prev    = g->gate_open;
    g->gate_open      = above | (g->hold_syms > 0u);
    return prev && !g->gate_open; /* 1 = falling edge */
}
