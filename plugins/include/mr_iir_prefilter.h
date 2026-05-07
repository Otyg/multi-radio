/**
 * mr_iir_prefilter.h — 4th-order Butterworth IIR channel-select pre-filter.
 *
 * Applies a low-pass filter to complex IQ samples before the resampler to
 * reduce out-of-band noise and adjacent-channel interference.
 *
 * Requires libliquid (MR_PLUGIN_HAS_LIQUID).
 */

#pragma once

#if defined(MR_PLUGIN_HAS_LIQUID) && MR_PLUGIN_HAS_LIQUID
#include <liquid/liquid.h>

/* ------------------------------------------------------------------ */
/* State struct                                                         */
/* ------------------------------------------------------------------ */

typedef struct {
    iirfilt_cccf filter; /* NULL if disabled (cutoff out of range) */
} MrIirPrefilter;

/* ------------------------------------------------------------------ */
/* API                                                                  */
/* ------------------------------------------------------------------ */

static inline void mr_iir_prefilter_init(MrIirPrefilter* pf) {
    pf->filter = NULL;
}

/**
 * Create (or recreate) the filter.
 *
 * cutoff_norm  normalized cutoff frequency in (0, 0.45).
 *              A typical value for GMSK/FSK is 2*baud_rate/sample_rate.
 *
 * Returns 1 on success, 0 if cutoff is out of range (filter stays NULL).
 */
static inline int mr_iir_prefilter_create(MrIirPrefilter* pf, float cutoff_norm) {
    if (pf->filter) {
        iirfilt_cccf_destroy(pf->filter);
        pf->filter = NULL;
    }
    if (cutoff_norm <= 0.0f || cutoff_norm >= 0.45f) return 0;
    pf->filter = iirfilt_cccf_create_prototype(
        LIQUID_IIRDES_BUTTER, LIQUID_IIRDES_LOWPASS, LIQUID_IIRDES_SOS,
        4,            /* order */
        cutoff_norm,  /* normalized cutoff */
        0.0f,         /* center frequency (unused for LP) */
        1.0f,         /* passband ripple (Butterworth: ignored) */
        60.0f);       /* stopband attenuation (Butterworth: ignored) */
    return pf->filter != NULL;
}

static inline void mr_iir_prefilter_destroy(MrIirPrefilter* pf) {
    if (pf->filter) {
        iirfilt_cccf_destroy(pf->filter);
        pf->filter = NULL;
    }
}

/**
 * Apply filter to one complex sample.
 * If the filter is NULL (disabled), output equals input.
 */
static inline void mr_iir_prefilter_execute(MrIirPrefilter* pf,
                                             liquid_float_complex  in,
                                             liquid_float_complex* out) {
    if (pf->filter) iirfilt_cccf_execute(pf->filter, in, out);
    else            *out = in;
}

#endif /* MR_PLUGIN_HAS_LIQUID */
