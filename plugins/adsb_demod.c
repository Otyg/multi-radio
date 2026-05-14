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
    adsb_ctx_t   *ctx = (adsb_ctx_t *)userdata;
    emit_state_t *es  = ctx->emit_state;
    if (!es || !es->emit_fn) return;

    int nbytes = mm->msgbits / 8;

    char hex[29];
    for (int i = 0; i < nbytes; i++)
        snprintf(hex + i * 2, 3, "%02X", mm->msg[i]);

    /* Build decoded key:value JSON — all fields governed by their _valid flag. */
    char  meta[2048];
    char *p   = meta;
    char *end = meta + sizeof(meta) - 2;
    int   first = 1;

#define KV(k, fmt, val) do { \
    int _n = snprintf(p, end - p, "%s\"" k "\":\"" fmt "\"", first ? "{" : ",", val); \
    if (_n > 0) { p += _n; first = 0; } \
} while (0)

    /* ---- Always present ---- */
    KV("df",            "%d",   mm->msgtype);
    KV("icao",          "%06X", mm->addr);
    KV("correctedbits", "%d",   mm->correctedbits);
    KV("rssi",          "%.4f", mm->signalLevel);
    KV("reliable",      "%d",   mm->reliable);

    { /* addrtype */
        const char *at;
        switch (mm->addrtype) {
            case ADDR_ADSB_ICAO:      at = "adsb_icao";      break;
            case ADDR_ADSB_ICAO_NT:   at = "adsb_icao_nt";   break;
            case ADDR_ADSR_ICAO:      at = "adsr_icao";      break;
            case ADDR_TISB_ICAO:      at = "tisb_icao";      break;
            case ADDR_ADSB_OTHER:     at = "adsb_other";     break;
            case ADDR_ADSR_OTHER:     at = "adsr_other";     break;
            case ADDR_TISB_TRACKFILE: at = "tisb_trackfile"; break;
            case ADDR_TISB_OTHER:     at = "tisb_other";     break;
            case ADDR_MODE_A:         at = "mode_a";         break;
            default:                  at = "unknown";        break;
        }
        KV("addrtype", "%s", at);
    }

    { /* source */
        const char *src;
        switch (mm->source) {
            case SOURCE_MODE_AC:        src = "mode_ac";        break;
            case SOURCE_MLAT:           src = "mlat";           break;
            case SOURCE_MODE_S:         src = "mode_s";         break;
            case SOURCE_MODE_S_CHECKED: src = "mode_s_checked"; break;
            case SOURCE_TISB:           src = "tisb";           break;
            case SOURCE_ADSR:           src = "adsr";           break;
            case SOURCE_ADSB:           src = "adsb";           break;
            default:                    src = "unknown";        break;
        }
        KV("source", "%s", src);
    }

    /* ---- Air/ground ---- */
    if (mm->airground != AG_INVALID) {
        const char *ag;
        switch (mm->airground) {
            case AG_GROUND:    ag = "ground";    break;
            case AG_AIRBORNE:  ag = "airborne";  break;
            case AG_UNCERTAIN: ag = "uncertain"; break;
            default:           ag = "unknown";   break;
        }
        KV("airground", "%s", ag);
    }

    /* ---- DF17/18 extended squitter ---- */
    if (mm->msgtype == 17 || mm->msgtype == 18) {
        KV("metype", "%u", mm->metype);
        KV("mesub",  "%u", mm->mesub);
        if (mm->category_valid)
            KV("category", "%02X", mm->category);

        /* Airborne velocity (ME type 19): IC, IFR, airspeed type, vrate source */
        if (mm->metype == 19) {
            KV("ic",  "%d", (mm->ME[1] >> 7) & 1);
            KV("ifr", "%d", (mm->ME[1] >> 6) & 1);
            if      (mm->ias_valid)       KV("airspeed_type", "%s", "ias");
            else if (mm->tas_valid)       KV("airspeed_type", "%s", "tas");
            if      (mm->baro_rate_valid) KV("vrate_src", "%s", "baro");
            else if (mm->geom_rate_valid) KV("vrate_src", "%s", "geom");
        }
    }

    /* ---- DF4/5/20/21 raw fields ---- */
    if (mm->msgtype == 4 || mm->msgtype == 5 ||
        mm->msgtype == 20 || mm->msgtype == 21) {
        KV("fs", "%u", mm->FS);
        KV("dr", "%u", mm->DR);
        KV("um", "%u", mm->UM);
    }

    /* DF11: capability and interrogator ID */
    if (mm->msgtype == 11) {
        KV("ca",  "%u", mm->CA);
        KV("iid", "%u", mm->IID);
    }

    /* ---- Identification ---- */
    if (mm->callsign_valid) {
        char cs[9];
        memcpy(cs, mm->callsign, 8); cs[8] = '\0';
        for (int i = 7; i >= 0 && cs[i] == ' '; --i) cs[i] = '\0';
        KV("callsign", "%s", cs);
    }
    if (mm->squawk_valid)    KV("squawk", "%04X", mm->squawk);
    if (mm->alert_valid)     KV("alert",  "%d",   mm->alert);
    if (mm->spi_valid)       KV("spi",    "%d",   mm->spi);

    if (mm->emergency_valid) {
        const char *em;
        switch (mm->emergency) {
            case EMERGENCY_NONE:      em = "none";      break;
            case EMERGENCY_GENERAL:   em = "general";   break;
            case EMERGENCY_LIFEGUARD: em = "lifeguard"; break;
            case EMERGENCY_MINFUEL:   em = "minfuel";   break;
            case EMERGENCY_NORDO:     em = "nordo";     break;
            case EMERGENCY_UNLAWFUL:  em = "unlawful";  break;
            case EMERGENCY_DOWNED:    em = "downed";    break;
            default:                  em = "reserved";  break;
        }
        KV("emergency", "%s", em);
    }

    /* ---- Altitude ---- */
    if (mm->altitude_baro_valid) KV("alt_baro",   "%d", mm->altitude_baro);
    if (mm->altitude_geom_valid) KV("alt_geom",   "%d", mm->altitude_geom);
    if (mm->geom_delta_valid)    KV("geom_delta", "%d", mm->geom_delta);

    /* ---- Velocity / heading ---- */
    if (mm->heading_valid) {
        KV("heading", "%.2f", (double)mm->heading);
        const char *ht;
        switch (mm->heading_type) {
            case HEADING_GROUND_TRACK:     ht = "ground_track";     break;
            case HEADING_TRUE:             ht = "true";             break;
            case HEADING_MAGNETIC:         ht = "magnetic";         break;
            case HEADING_MAGNETIC_OR_TRUE: ht = "magnetic_or_true"; break;
            case HEADING_TRACK_OR_HEADING: ht = "track_or_heading"; break;
            default:                       ht = "unknown";          break;
        }
        KV("heading_type", "%s", ht);
    }
    if (mm->track_rate_valid) KV("track_rate", "%.2f", (double)mm->track_rate);
    if (mm->roll_valid)       KV("roll",       "%.2f", (double)mm->roll);
    if (mm->gs_valid)         KV("gs",         "%.1f", (double)mm->gs.selected);
    if (mm->ias_valid)        KV("ias",        "%u",   mm->ias);
    if (mm->tas_valid)        KV("tas",        "%u",   mm->tas);
    if (mm->mach_valid)       KV("mach",       "%.3f", mm->mach);
    if (mm->baro_rate_valid)  KV("baro_rate",  "%d",   mm->baro_rate);
    if (mm->geom_rate_valid)  KV("geom_rate",  "%d",   mm->geom_rate);

    /* ---- Position ---- */
    if (mm->cpr_decoded) {
        KV("lat", "%.6f", mm->decoded_lat);
        KV("lon", "%.6f", mm->decoded_lon);
        KV("nic", "%u",   mm->decoded_nic);
        KV("rc",  "%u",   mm->decoded_rc);
    }

    /* ---- Navigation (type 29 / Comm-B BDS 4,0) ---- */
    if (mm->nav.fms_altitude_valid) KV("nav_fms_alt", "%d",   mm->nav.fms_altitude);
    if (mm->nav.mcp_altitude_valid) KV("nav_mcp_alt", "%d",   mm->nav.mcp_altitude);
    if (mm->nav.qnh_valid)          KV("nav_qnh",     "%.1f", (double)mm->nav.qnh);
    if (mm->nav.heading_valid)      KV("nav_heading", "%.2f", (double)mm->nav.heading);
    if (mm->nav.modes_valid)        KV("nav_modes",   "%u",   (unsigned)mm->nav.modes);

    /* ---- Accuracy ---- */
    if (mm->accuracy.nac_p_valid)   KV("nac_p",   "%u", mm->accuracy.nac_p);
    if (mm->accuracy.nac_v_valid)   KV("nac_v",   "%u", mm->accuracy.nac_v);
    if (mm->accuracy.nic_baro_valid) KV("nic_baro", "%u", mm->accuracy.nic_baro);
    if (mm->accuracy.nic_a_valid)   KV("nic_a",   "%u", mm->accuracy.nic_a);
    if (mm->accuracy.nic_b_valid)   KV("nic_b",   "%u", mm->accuracy.nic_b);
    if (mm->accuracy.nic_c_valid)   KV("nic_c",   "%u", mm->accuracy.nic_c);
    if (mm->accuracy.gva_valid)     KV("gva",     "%u", mm->accuracy.gva);
    if (mm->accuracy.sda_valid)     KV("sda",     "%u", mm->accuracy.sda);
    if (mm->accuracy.sil_type != SIL_INVALID) {
        KV("sil", "%u", mm->accuracy.sil);
        const char *st;
        switch (mm->accuracy.sil_type) {
            case SIL_PER_SAMPLE: st = "per_sample"; break;
            case SIL_PER_HOUR:   st = "per_hour";   break;
            default:             st = "unknown";    break;
        }
        KV("sil_type", "%s", st);
    }

    /* ---- Comm-B decoded format ---- */
    if (mm->commb_format != COMMB_UNKNOWN && mm->commb_format != COMMB_NOT_DECODED) {
        const char *cbf;
        switch (mm->commb_format) {
            case COMMB_AMBIGUOUS:         cbf = "ambiguous";         break;
            case COMMB_EMPTY_RESPONSE:    cbf = "empty_response";    break;
            case COMMB_DATALINK_CAPS:     cbf = "datalink_caps";     break;
            case COMMB_GICB_CAPS:         cbf = "gicb_caps";         break;
            case COMMB_AIRCRAFT_IDENT:    cbf = "aircraft_ident";    break;
            case COMMB_ACAS_RA:           cbf = "acas_ra";           break;
            case COMMB_VERTICAL_INTENT:   cbf = "vertical_intent";   break;
            case COMMB_TRACK_TURN:        cbf = "track_turn";        break;
            case COMMB_HEADING_SPEED:     cbf = "heading_speed";     break;
            case COMMB_MRAR:              cbf = "mrar";              break;
            case COMMB_AIRBORNE_POSITION: cbf = "airborne_position"; break;
            default:                      cbf = "other";             break;
        }
        KV("commb_format", "%s", cbf);
    }

    /* ---- MRAR weather (BDS 4,4 / 4,5) ---- */
    if (mm->wind_valid) {
        KV("wind_speed", "%.1f", (double)mm->wind_speed);
        KV("wind_dir",   "%.1f", (double)mm->wind_dir);
    }
    if (mm->temperature_valid) KV("temperature", "%.1f", (double)mm->temperature);
    if (mm->pressure_valid)    KV("pressure",    "%.1f", (double)mm->pressure);
    if (mm->humidity_valid)    KV("humidity",    "%.1f", (double)mm->humidity);
    if (mm->turbulence_valid) {
        const char *turb;
        switch (mm->turbulence) {
            case HAZARD_LIGHT:    turb = "light";    break;
            case HAZARD_MODERATE: turb = "moderate"; break;
            case HAZARD_SEVERE:   turb = "severe";   break;
            default:              turb = "nil";      break;
        }
        KV("turbulence", "%s", turb);
    }

#undef KV

    if (first) *p++ = '{';
    *p++ = '}';
    *p   = '\0';

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
    ctx->converter = init_converter(INPUT_SC16, (double)sr, /*filter_dc=*/0,
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
    .name        = "adsb_demod",
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
