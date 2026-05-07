/**
 * mr_afc.h — Automatic Frequency Correction for FM-based demodulators.
 *
 * Estimates carrier frequency offset by averaging the FM discriminator output
 * over each symbol group (the mean of the instantaneous phase derivative equals
 * the frequency offset for a DC-balanced signal).  Corrects via a per-sample
 * NCO phase rotation applied before the demodulator.
 *
 * Requires libliquid for the complex sample type.  Uses only <math.h> otherwise.
 */

#pragma once

#if defined(MR_PLUGIN_HAS_LIQUID) && MR_PLUGIN_HAS_LIQUID
#include <liquid/liquid.h>
#endif

#include <math.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Default tunables                                                     */
/* ------------------------------------------------------------------ */

#define MR_AFC_ALPHA    0.005f  /* frequency error EMA alpha (~200 sym tau) */
#define MR_AFC_MAX_RAD  0.30f   /* max correction, rad/sample of resampled signal */

/* ------------------------------------------------------------------ */
/* State struct                                                         */
/* ------------------------------------------------------------------ */

typedef struct {
    float phase;     /* NCO phase accumulator, radians */
    float freq;      /* current frequency correction, rad/sample */
    float freq_est;  /* slow EMA of measured frequency error */
} MrAfc;

/* ------------------------------------------------------------------ */
/* API                                                                  */
/* ------------------------------------------------------------------ */

static inline void mr_afc_init(MrAfc* a) {
    a->phase    = 0.0f;
    a->freq     = 0.0f;
    a->freq_est = 0.0f;
}

static inline void mr_afc_reset(MrAfc* a) {
    a->phase = 0.0f;
    /* Preserve freq / freq_est so correction carries over between bursts */
}

#if defined(MR_PLUGIN_HAS_LIQUID) && MR_PLUGIN_HAS_LIQUID

/**
 * Apply current NCO correction to one complex sample (in-place safe).
 * Advances the internal phase by one step.
 */
static inline void mr_afc_correct(MrAfc* a,
                                   liquid_float_complex  in,
                                   liquid_float_complex* out) {
    const float cp = cosf(a->phase);
    const float sp = sinf(a->phase);
    const float re = __real__ in;
    const float im = __imag__ in;
    __real__ *out = re * cp + im * sp;
    __imag__ *out = im * cp - re * sp;
    a->phase += a->freq;
    if (a->phase >  3.14159265f) a->phase -= 6.28318530f;
    if (a->phase < -3.14159265f) a->phase += 6.28318530f;
}

/**
 * Update the frequency estimate from the FM discriminator over k samples.
 *
 * sym_buf  pointer to k contiguous liquid_float_complex samples
 * k        number of samples (samples per symbol)
 *
 * Only call when the gate is open (signal present).  The mean of the
 * instantaneous frequency (cross/|z|) across the symbol equals the
 * carrier frequency offset for a DC-balanced modulation like GMSK.
 */
static inline void mr_afc_update(MrAfc* a,
                                  const liquid_float_complex* sym_buf,
                                  int k) {
    float fm_sum = 0.0f;
    for (int s = 1; s < k; ++s) {
        const float re0 = __real__ sym_buf[s - 1];
        const float im0 = __imag__ sym_buf[s - 1];
        const float re1 = __real__ sym_buf[s];
        const float im1 = __imag__ sym_buf[s];
        const float cross = im1 * re0 - re1 * im0;
        const float dot   = re1 * re0 + im1 * im0;
        const float mag2  = cross * cross + dot * dot;
        fm_sum += (mag2 > 1e-12f) ? (cross / sqrtf(mag2)) : 0.0f;
    }
    const float fm_mean = fm_sum / (float)(k - 1);
    a->freq_est = a->freq_est * (1.0f - MR_AFC_ALPHA) + fm_mean * MR_AFC_ALPHA;
    a->freq = -a->freq_est;
    if (a->freq >  MR_AFC_MAX_RAD) a->freq =  MR_AFC_MAX_RAD;
    if (a->freq < -MR_AFC_MAX_RAD) a->freq = -MR_AFC_MAX_RAD;
}

#endif /* MR_PLUGIN_HAS_LIQUID */
