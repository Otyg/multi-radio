/**
 * libmodes_adsb_demod.c — ADS-B / Mode S demodulator plugin backed by libmodes
 *
 * Accepts int16_t SC16 IQ at 2.3–2.5 Msps (nominal 2.4 Msps) tuned to 1090 MHz.
 * Uses dump1090's proven demodulate2400() pipeline via libmodes.a.
 *
 * Emits signal_type "ADSB" with the raw message bytes as an uppercase hex string.
 * Metadata JSON: {"df":"<n>","icao":"<hex6>","correctedbits":"<n>","rssi":"<f>"}
 */

#include "mr_plugin_api.h"

#include <libmodes/libmodes.h>
#include <libmodes/demod_2400.h>
#include <libmodes/convert.h>
#include <libmodes/crc.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Leading overlap samples preserved between successive IQ blocks.
 * dump1090 uses 400; MODES_OS_PREAMBLE_SAMPLES (20) is the minimum. */
#define MAG_OVERLAP 400u

/* ------------------------------------------------------------------ */
/* Plugin context                                                        */
/* ------------------------------------------------------------------ */

/* State threaded through the libmodes callback during a single process_iq call. */
typedef struct {
    MrEmitFn emit_fn;
    void    *user_data;
    double   center_freq_hz;
    uint64_t unix_ms;
} emit_state_t;

typedef struct {
    struct modes_decoder    *decoder;
    struct converter_state  *conv_state;
    iq_convert_fn            converter;

    uint16_t                *mag_buf;    /* [MAG_OVERLAP | payload] */
    unsigned                 mag_buf_cap;

    uint32_t                 current_sr;
    emit_state_t            *emit_state; /* valid only during demodulate2400() */

    struct modes_demod_stats stats;
} adsb_ctx_t;

/* ------------------------------------------------------------------ */
/* libmodes message callback                                            */
/* ------------------------------------------------------------------ */

static void on_message(struct modesMessage *mm, void *userdata)
{
    adsb_ctx_t  *ctx = (adsb_ctx_t *)userdata;
    emit_state_t *es  = ctx->emit_state;
    if (!es || !es->emit_fn) return;

    int nbytes = mm->msgbits / 8;

    /* Hex-encode raw message bytes (max 14 bytes = 28 hex chars + NUL). */
    char hex[29];
    for (int i = 0; i < nbytes; i++)
        snprintf(hex + i * 2, 3, "%02X", mm->msg[i]);

    char meta[128];
    snprintf(meta, sizeof(meta),
             "{\"df\":\"%d\",\"icao\":\"%06X\","
             "\"correctedbits\":\"%d\",\"rssi\":\"%.4f\"}",
             mm->msgtype, mm->addr, mm->correctedbits, mm->signalLevel);

    es->emit_fn("ADSB", hex, es->center_freq_hz, es->unix_ms, meta, es->user_data);
}

/* ------------------------------------------------------------------ */
/* Converter initialisation                                             */
/* ------------------------------------------------------------------ */

static int reinit_converter(adsb_ctx_t *ctx, uint32_t sr)
{
    if (ctx->conv_state) {
        cleanup_converter(ctx->conv_state);
        ctx->conv_state = NULL;
        ctx->converter  = NULL;
    }
    ctx->converter = init_converter(INPUT_SC16, (double)sr, /*filter_dc=*/1,
                                    &ctx->conv_state);
    if (!ctx->converter) return 0;
    ctx->current_sr = sr;
    return 1;
}

/* ------------------------------------------------------------------ */
/* Mandatory plugin exports                                             */
/* ------------------------------------------------------------------ */

MrPluginCtx *mr_plugin_create(void)
{
    adsb_ctx_t *ctx = calloc(1, sizeof(adsb_ctx_t));
    if (!ctx) return NULL;

    modesChecksumInit(/*fixBits=*/2);

    struct modes_decoder_config cfg = {
        .nfix_crc    = 1,
        .fix_df      = 1,
        .enable_df24 = 0,
    };
    ctx->decoder = modes_decoder_create(&cfg);
    if (!ctx->decoder) { free(ctx); return NULL; }

    modes_decoder_set_callback(ctx->decoder, on_message, ctx);
    return ctx;
}

void mr_plugin_destroy(MrPluginCtx *ctx_)
{
    adsb_ctx_t *ctx = (adsb_ctx_t *)ctx_;
    if (!ctx) return;
    if (ctx->decoder)    modes_decoder_destroy(ctx->decoder);
    if (ctx->conv_state) cleanup_converter(ctx->conv_state);
    free(ctx->mag_buf);
    free(ctx);
}

static const MrPluginMeta s_meta = {
    .name        = "libmodes_adsb_demod",
    .version     = "1.0.0",
    .api_version = MR_PLUGIN_API_VERSION,
    .description = "ADS-B/Mode S demodulator via libmodes (dump1090)",
    .role        = MR_PLUGIN_ROLE_DEMODULATOR,
};

const MrPluginMeta *mr_plugin_get_meta(void) { return &s_meta; }

void mr_plugin_process_iq(MrPluginCtx      *ctx_,
                          const int16_t    *iq_samples,
                          uint32_t          num_pairs,
                          uint32_t          sample_rate_hz,
                          double            center_freq_hz,
                          uint64_t          unix_ms,
                          MrEmitFn          emit_fn,
                          void             *user_data)
{
    adsb_ctx_t *ctx = (adsb_ctx_t *)ctx_;

    if (sample_rate_hz < 2300000u || sample_rate_hz > 2500000u) return;
    if (num_pairs == 0) return;

    if (sample_rate_hz != ctx->current_sr || !ctx->converter)
        if (!reinit_converter(ctx, sample_rate_hz)) return;

    /* Ensure magnitude buffer is large enough for overlap + payload. */
    unsigned needed = MAG_OVERLAP + num_pairs;
    if (needed > ctx->mag_buf_cap) {
        uint16_t *nb = realloc(ctx->mag_buf, needed * sizeof(uint16_t));
        if (!nb) return;
        if (ctx->mag_buf_cap == 0)
            memset(nb, 0, MAG_OVERLAP * sizeof(uint16_t)); /* clear initial overlap */
        ctx->mag_buf     = nb;
        ctx->mag_buf_cap = needed;
    }

    /* Convert SC16 IQ → uint16_t magnitudes, placed after the overlap region. */
    double mean_level, mean_power;
    ctx->converter((void *)iq_samples,
                   ctx->mag_buf + MAG_OVERLAP,
                   num_pairs,
                   ctx->conv_state,
                   &mean_level,
                   &mean_power);

    struct mag_buf mbuf = {
        .data            = ctx->mag_buf,
        .totalLength     = needed,
        .validLength     = needed,
        .overlap         = MAG_OVERLAP,
        .sampleTimestamp = 0,
        .sysTimestamp    = unix_ms,
        .flags           = 0,
        .mean_level      = mean_level,
        .mean_power      = mean_power,
        .dropped         = 0,
        .next            = NULL,
    };

    emit_state_t es = {
        .emit_fn        = emit_fn,
        .user_data      = user_data,
        .center_freq_hz = center_freq_hz,
        .unix_ms        = unix_ms,
    };
    ctx->emit_state = &es;

    demodulate2400(ctx->decoder, &mbuf, &ctx->stats);

    ctx->emit_state = NULL;

    /*
     * Preserve the last MAG_OVERLAP samples of the payload as the leading
     * overlap for the next call.  If the payload is smaller than MAG_OVERLAP
     * we slide the existing overlap window instead.
     */
    if (num_pairs >= MAG_OVERLAP) {
        memcpy(ctx->mag_buf,
               ctx->mag_buf + num_pairs,
               MAG_OVERLAP * sizeof(uint16_t));
    } else {
        memmove(ctx->mag_buf,
                ctx->mag_buf + num_pairs,
                MAG_OVERLAP * sizeof(uint16_t));
    }
}

/* ------------------------------------------------------------------ */
/* Optional exports (stubs)                                             */
/* ------------------------------------------------------------------ */

void mr_plugin_process_bits(MrPluginCtx  *ctx,
                            const uint8_t *bit_bytes,
                            uint32_t       bit_count,
                            double         freq_hz,
                            uint64_t       unix_ms,
                            const char    *source_type,
                            MrEmitFn       emit_fn,
                            void          *user_data)
{
    (void)ctx; (void)bit_bytes; (void)bit_count;
    (void)freq_hz; (void)unix_ms; (void)source_type;
    (void)emit_fn; (void)user_data;
}

int mr_plugin_set_param(MrPluginCtx *ctx, const char *key, const char *value)
{
    (void)ctx; (void)key; (void)value;
    return 0;
}
