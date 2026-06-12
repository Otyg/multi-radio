/**
 * asm_decoder.c — AIS Application-Specific Message (ASM) decoder
 *
 * Role: MR_PLUGIN_ROLE_POSTPROCESSING
 *
 * Decodes AIS message types 6, 8, 12, 14 and the international ASM
 * payloads defined in ITU-R M.1371-5 Annex 5 (DAC=001):
 *
 *   FI=11  Meteorological and Hydrographic Data (short)
 *   FI=13  Fairway Closed
 *   FI=16  Persons on Board
 *   FI=17  VTS-Generated / Synthesised AIS Safety-Related Message
 *   FI=21  Weather Observation from Ship
 *   FI=22  Area Notice (Broadcast)
 *   FI=26  Environmental
 *   FI=27  Route Information
 *   FI=28  Text Using 6-bit ASCII
 *   FI=29  Marine Traffic Signal
 *   FI=31  Meteorological and Hydrographic Data (long)
 *   FI=32  Tidal Window
 *
 * Can operate standalone (HDLC decode from raw bits) or receive pre-framed
 * bytes via the AIS_MSG8_RAW handoff from ais_decoder.
 *
 * Emitted signal types:
 *   AIS_MSG6       addressed binary message (raw hex if DAC≠001 or unknown FI)
 *   AIS_MSG8       broadcast binary message (same)
 *   AIS_MSG6_RAW   raw hex from ais_decoder (full frame including FCS)
 *   AIS_MSG8_RAW   raw hex from ais_decoder (full frame including FCS)
 *   AIS_MSG12_RAW  raw hex from ais_decoder (full frame including FCS)
 *   AIS_MSG12      addressed safety-related text
 *   AIS_MSG14      safety-related broadcast text
 *   AIS_ASM_*      per-FI decoded payloads (see list above)
 */

#include "mr_plugin_api.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── debug ────────────────────────────────────────────────────────────── */

static int asm_dbg(void) {
    static int v = -1;
    if (v < 0) { const char* e = getenv("MR_AIS_DEBUG"); v = (e && e[0] != '0'); }
    return v;
}
#define ALOG(...) do { if (asm_dbg()) fprintf(stderr, "[asm] " __VA_ARGS__); } while(0)

/* ── CRC-16-CCITT ─────────────────────────────────────────────────────── */

#define HDLC_CRC_INIT    0xFFFFu
#define HDLC_CRC_RESIDUE 0xF0B8u
#define HDLC_CRC_POLY    0x8408u
#define HDLC_MAX_FRAME   512u

static uint16_t crc16_byte(uint16_t crc, uint8_t byte) {
    int i;
    crc ^= (uint16_t)byte;
    for (i = 0; i < 8; ++i)
        crc = (crc & 1u) ? ((crc >> 1) ^ HDLC_CRC_POLY) : (crc >> 1);
    return crc;
}

static int hdlc_check_crc(const uint8_t* buf, uint32_t len) {
    uint16_t crc = HDLC_CRC_INIT;
    uint32_t i;
    for (i = 0; i < len; ++i) crc = crc16_byte(crc, buf[i]);
    return crc == HDLC_CRC_RESIDUE;
}

/* ── bit extraction (AIS LSB-first byte assembly → MSB-first fields) ──── */

static uint64_t ais_u(const uint8_t* d, uint32_t dbytes, int start, int len) {
    uint64_t r = 0; int i;
    for (i = 0; i < len; ++i) {
        int pos = start + i;
        if ((pos / 8) >= (int)dbytes) break;
        if (d[pos / 8] & (1u << (7 - (pos % 8))))
            r |= (1ULL << (len - 1 - i));
    }
    return r;
}

static int64_t ais_s(const uint8_t* d, uint32_t db, int start, int len) {
    uint64_t u = ais_u(d, db, start, len);
    if (u & (1ULL << (len - 1))) u |= ~((1ULL << len) - 1ULL);
    return (int64_t)u;
}

/* Decode n×6-bit AIS characters starting at start_bit into out (NUL-terminated). */
static void ais_str(const uint8_t* d, uint32_t db, int start, int n, char* out) {
    int i, end;
    for (i = 0; i < n; ++i) {
        uint64_t v = ais_u(d, db, start + i * 6, 6);
        out[i] = (char)(v < 32u ? v + 64u : v);
    }
    end = n;
    while (end > 0 && (out[end-1] == '@' || out[end-1] == ' ')) --end;
    out[end] = '\0';
}

/* Helper function to convert hex string to byte array */
static uint32_t hex_to_bytes(const char* hex_str, uint8_t* out_buf, uint32_t out_buf_cap) {
    uint32_t len = 0;
    if (!hex_str || !out_buf || out_buf_cap == 0) return 0;

    for (int i = 0; hex_str[i] && hex_str[i+1]; i += 2) {
        if (len >= out_buf_cap) break;
        char hex_byte[3];
        hex_byte[0] = hex_str[i];
        hex_byte[1] = hex_str[i+1];
        hex_byte[2] = '\0';
        out_buf[len++] = (uint8_t)strtol(hex_byte, NULL, 16);
    }
    return len;
}

/* Hex-encode `bytes` bytes starting at frame_buf[offset]. */
static void hex_encode(const uint8_t* buf, uint32_t offset, uint32_t bytes,
                       char* out, size_t out_sz) {
    uint32_t i;
    out[0] = '\0';
    for (i = 0; i < bytes && (i * 2 + 2) < out_sz; ++i)
        snprintf(out + i * 2, 3, "%02X", (unsigned)buf[offset + i]);
}

/* ── helpers ──────────────────────────────────────────────────────────── */

static const char* precip_name(int t) {
    switch (t) {
        case 0: return "Rain";
        case 1: return "Thunderstorm";
        case 2: return "Freezing rain";
        case 3: return "Mixed/ice";
        case 4: return "Snow";
        case 5: return "None";
        default: return "N/A";
    }
}

static const char* notice_type_name(int t) {
    /* ITU-R M.1371-5 Table 11.9 (partial) */
    switch (t) {
        case 0:  return "Caution Area: Marine mammals";
        case 1:  return "Caution Area: Migratory birds";
        case 2:  return "Caution Area: Turtle habitat";
        case 3:  return "Caution Area: Storm";
        case 4:  return "Caution Area: Special use";
        case 5:  return "Caution Area: Anchoring prohibited";
        case 6:  return "Caution Area: Restricted fishing";
        case 7:  return "Caution Area: Anchoring permitted";
        case 8:  return "Caution Area: Entry approval req.";
        case 9:  return "Caution Area: Entry prohibited";
        case 10: return "Caution Area: Diving";
        case 11: return "Caution Area: Underwater ops";
        case 12: return "Caution Area: Swimming";
        case 13: return "Caution Area: Wading";
        case 14: return "Caution Area: Water skiing";
        case 15: return "Caution Area: Tugs ops";
        case 16: return "Caution Area: Ferry crossing";
        case 17: return "Weather";
        case 18: return "Traffic restriction: Congestion";
        case 19: return "Traffic restriction: Limited";
        case 20: return "Anchorage";
        case 21: return "Work in progress";
        case 22: return "Object, non-default shape";
        case 23: return "Festivity/public event";
        case 24: return "Tidal window (IALA A)";
        case 25: return "Tidal window (IALA B)";
        case 26: return "Navigational hazard";
        case 27: return "Vessel aground";
        case 28: return "Vessel in distress";
        case 29: return "Medical emergency";
        case 30: return "Wreck";
        case 31: return "Drifting mine";
        case 32: return "Military exercise";
        case 33: return "Law enforcement";
        case 34: return "Cableway/pipeline";
        case 35: return "Seismic survey";
        case 36: return "Hydrographic survey";
        case 37: return "VTS in operation";
        case 57: return "Report from ship";
        case 58: return "Whale strike info";
        case 59: return "Vessel towing";
        case 60: return "Volcanic activity";
        case 61: return "Rescue/salvage ops";
        case 62: return "Research vessel";
        case 63: return "Minelaying ops";
        case 64: return "Entry prohibited - national reason";
        case 99: return "Cancel notice";
        default: return "Unknown";
    }
}

/* ── DAC=001 FI decoders ──────────────────────────────────────────────── */

/*
 * FI=11 and FI=21: Met/Hydro data (short, 168 app bits)
 * App data offset from msg8 bit 56 (msg6 bit 88).
 * Bit positions are relative to start of app data (after DAC+FI).
 */
static void decode_fi11_21(const uint8_t* d, uint32_t db, int app_off,
                            uint32_t mmsi, int fi, int msg_type,
                            MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    /* Wind */
    int   wspd = (int)ais_u(d, db, app_off,      7);   /* kn, 127=N/A */
    int   wgst = (int)ais_u(d, db, app_off + 7,  7);   /* kn, 127=N/A */
    int   wdir = (int)ais_u(d, db, app_off + 14, 9);   /* deg, 360=N/A */
    int   wgdr = (int)ais_u(d, db, app_off + 23, 9);   /* deg, 360=N/A */
    /* Air */
    int64_t atmp = ais_s(d, db, app_off + 32, 11);     /* 0.1°C, 601=N/A */
    int   rhum = (int)ais_u(d, db, app_off + 43, 7);   /* %, 101=N/A */
    int64_t dew = ais_s(d, db, app_off + 50, 10);      /* 0.1°C, 501=N/A */
    int   pres = (int)ais_u(d, db, app_off + 60, 9);   /* hPa-800, 511=N/A */
    int64_t ptrend = ais_s(d, db, app_off + 69, 4);    /* hPa/3h */
    /* Visibility */
    int   vis  = (int)ais_u(d, db, app_off + 73, 8);   /* 0.1 nm, 255=N/A */
    int   prec = (int)ais_u(d, db, app_off + 81, 3);   /* type */
    /* Sea */
    int   sal  = (int)ais_u(d, db, app_off + 84, 9);   /* 0.1‰, 511=N/A */
    int   ice  = (int)ais_u(d, db, app_off + 93, 2);   /* 0=no 1=yes 3=N/A */

    char kv[768], pay[256];
    const char* src = (fi == 21) ? "ship" : "shore";

    /* Simpler version without compound-literal trick for older compilers */
    char ws[12], wd[12], wg[12], at[12], rh[12], dp[12], pr[12], vi[12], sl[12];
    if (wspd==127) snprintf(ws,12,"N/A"); else snprintf(ws,12,"%d kn",wspd);
    if (wdir==360) snprintf(wd,12,"N/A"); else snprintf(wd,12,"%d°",wdir);
    if (wgst==127) snprintf(wg,12,"N/A"); else snprintf(wg,12,"%d kn",wgst);
    if (atmp==601) snprintf(at,12,"N/A"); else snprintf(at,12,"%.1f°C",(double)atmp/10.0);
    if (rhum==101) snprintf(rh,12,"N/A"); else snprintf(rh,12,"%d%%",rhum);
    if (dew ==501) snprintf(dp,12,"N/A"); else snprintf(dp,12,"%.1f°C",(double)dew/10.0);
    if (pres==511) snprintf(pr,12,"N/A"); else snprintf(pr,12,"%d hPa",pres+800);
    if (vis ==255) snprintf(vi,12,"N/A"); else snprintf(vi,12,"%.1f nm",(double)vis/10.0);
    if (sal ==511) snprintf(sl,12,"N/A"); else snprintf(sl,12,"%.1f‰",(double)sal/10.0);

    /* Re-build kv cleanly */
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_ASM_METHYDRO\",\"msg_type\":\"%d\",\"sender_mmsi\":\"%u\","
        "\"fi\":\"%d\",\"mmsi\":\"%u\",\"source\":\"%s\","
        "\"wind_speed_kn\":\"%s\",\"wind_dir_deg\":\"%s\","
        "\"wind_gust_kn\":\"%s\","
        "\"air_temp_c\":\"%s\",\"rel_humidity_pct\":\"%s\","
        "\"dew_point_c\":\"%s\",\"pressure_hpa\":\"%s\","
        "\"pressure_trend_hpa3h\":\"%lld\","
        "\"visibility_nm\":\"%s\","
        "\"precipitation\":\"%s\","
        "\"salinity_ppt\":\"%s\","
        "\"ice\":\"%s\"}",
        msg_type, mmsi, fi, mmsi, src,
        ws, wd, wg, at, rh, dp, pr, (long long)ptrend, vi,
        precip_name(prec), sl,
        ice == 3 ? "N/A" : (ice == 1 ? "Yes" : "No"));

    snprintf(pay, sizeof(pay),
        "Met/Hydro(%s) MMSI:%u Wind:%s@%s Gust:%s Temp:%s RH:%s Pres:%s Vis:%s Prec:%s",
        src, mmsi, ws, wd, wg, at, rh, pr, vi, precip_name(prec));

    emit("AIS_ASM_METHYDRO", pay, freq, ms, kv, ud);
}

/* FI=13: Fairway Closed */
static void decode_fi13(const uint8_t* d, uint32_t db, int app_off,
                         uint32_t mmsi, int msg_type,
                         MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    char reason[21];
    ais_str(d, db, app_off, 20, reason);  /* 20 chars × 6 bit */
    int from_day  = (int)ais_u(d, db, app_off + 120, 5);
    int from_hour = (int)ais_u(d, db, app_off + 125, 5);
    int from_min  = (int)ais_u(d, db, app_off + 130, 6);
    int to_day    = (int)ais_u(d, db, app_off + 136, 5);
    int to_hour   = (int)ais_u(d, db, app_off + 141, 5);
    int to_min    = (int)ais_u(d, db, app_off + 146, 6);

    char kv[512], pay[256];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_ASM_FAIRWAY_CLOSED\",\"msg_type\":\"%d\",\"sender_mmsi\":\"%u\","
        "\"mmsi\":\"%u\","
        "\"reason\":\"%s\","
        "\"from\":\"day%02d %02d:%02d\","
        "\"to\":\"day%02d %02d:%02d\"}",
        msg_type, mmsi, mmsi, reason, from_day, from_hour, from_min,
        to_day, to_hour, to_min);
    snprintf(pay, sizeof(pay),
        "Fairway Closed MMSI:%u Reason:%s From:day%02d %02d:%02d To:day%02d %02d:%02d",
        mmsi, reason, from_day, from_hour, from_min, to_day, to_hour, to_min);
    emit("AIS_ASM_FAIRWAY_CLOSED", pay, freq, ms, kv, ud);
}

/* FI=16: Number of Persons on Board (app bits: 13+3 spare = 16 bits) */
static void decode_fi16(const uint8_t* d, uint32_t db, int app_off,
                         uint32_t mmsi, int msg_type,
                         MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    int persons = (int)ais_u(d, db, app_off, 13);  /* 0-8190; 8191=N/A */
    char kv[256], pay[128];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_ASM_PERSONS_ON_BOARD\",\"msg_type\":\"%d\",\"sender_mmsi\":\"%u\","
        "\"mmsi\":\"%u\",\"persons\":\"%s\"}",
        msg_type, mmsi, mmsi, persons == 8191 ? "N/A" : (snprintf(pay,128,"%d",persons),pay));
    snprintf(pay, sizeof(pay),
        "Persons on Board MMSI:%u Count:%s",
        mmsi, persons == 8191 ? "N/A" : (snprintf(kv+200,50,"%d",persons),kv+200));
    emit("AIS_ASM_PERSONS_ON_BOARD", pay, freq, ms, kv, ud);
}

/* FI=17: VTS-Generated Safety-Related Text (6-bit ASCII, variable) */
static void decode_fi17(const uint8_t* d, uint32_t db, int app_off,
                         uint32_t mmsi, int msg_type,
                         MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    /* App data: link ID (10 bits) + text (6-bit ASCII, variable) */
    int link_id = (int)ais_u(d, db, app_off, 10);
    int app_bits = (int)((db - (uint32_t)(app_off / 8)) * 8) - app_off % 8;
    if (app_bits < 0) app_bits = 0;
    int n_chars = (app_bits - 10) / 6;
    if (n_chars < 0) n_chars = 0;
    if (n_chars > 63) n_chars = 63;
    char text[64] = "";
    if (n_chars > 0) ais_str(d, db, app_off + 10, n_chars, text);
    char kv[384], pay[256];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_ASM_VTS_TEXT\",\"msg_type\":\"%d\",\"sender_mmsi\":\"%u\","
        "\"mmsi\":\"%u\",\"link_id\":\"%d\",\"text\":\"%s\"}",
        msg_type, mmsi, mmsi, link_id, text);
    snprintf(pay, sizeof(pay), "VTS Text MMSI:%u [%d] %s", mmsi, link_id, text);
    emit("AIS_ASM_VTS_TEXT", pay, freq, ms, kv, ud);
}

/* FI=22: Area Notice Broadcast (variable length, sub-areas) */
static void decode_fi22(const uint8_t* d, uint32_t db, int app_off,
                         uint32_t mmsi, int msg_type,
                         MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    int link_id    = (int)ais_u(d, db, app_off,      10);
    int notice_t   = (int)ais_u(d, db, app_off + 10,  7);
    int month      = (int)ais_u(d, db, app_off + 17,  4);
    int day        = (int)ais_u(d, db, app_off + 21,  5);
    int hour       = (int)ais_u(d, db, app_off + 26,  5);
    int minute     = (int)ais_u(d, db, app_off + 31,  6);
    int duration   = (int)ais_u(d, db, app_off + 37, 18);  /* minutes */

    char kv[512], pay[256];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_ASM_AREA_NOTICE\",\"msg_type\":\"%d\",\"sender_mmsi\":\"%u\","
        "\"mmsi\":\"%u\","
        "\"link_id\":\"%d\","
        "\"notice_type\":\"%d\","
        "\"notice_name\":\"%s\","
        "\"utc\":\"M%02d D%02d %02d:%02d\","
        "\"duration_min\":\"%d\"}",
        msg_type, mmsi, mmsi, link_id, notice_t, notice_type_name(notice_t),
        month, day, hour, minute, duration);
    snprintf(pay, sizeof(pay),
        "Area Notice MMSI:%u [%d] %s UTC:M%02d D%02d %02d:%02d Dur:%dmin",
        mmsi, link_id, notice_type_name(notice_t),
        month, day, hour, minute, duration);
    emit("AIS_ASM_AREA_NOTICE", pay, freq, ms, kv, ud);
}

/* FI=26: Environmental (sensor report, first sensor only) */
static void decode_fi26(const uint8_t* d, uint32_t db, int app_off,
                         uint32_t mmsi, int msg_type,
                         MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    /* Report type (4 bits), report data (variable). Emit first report type. */
    int report_t = (int)ais_u(d, db, app_off, 4);
    static const char* rep_names[] = {
        "Spare", "Location", "Station ID", "Weather", "Air gap/draught",
        "Water level", "Current flow 2D", "Current flow 3D", "Horizontal flow",
        "Sea state", "Salinity", "Weather detail", "Air gap/draught",
        "Reserved", "Reserved", "Reserved"
    };
    char kv[384], pay[256];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_ASM_ENVIRONMENTAL\",\"msg_type\":\"%d\",\"sender_mmsi\":\"%u\","
        "\"mmsi\":\"%u\",\"report_type\":\"%d\",\"report_name\":\"%s\"}",
        msg_type, mmsi, mmsi, report_t,
        (report_t < 16) ? rep_names[report_t] : "Unknown");
    snprintf(pay, sizeof(pay),
        "Environmental MMSI:%u Report:%d(%s)",
        mmsi, report_t, (report_t < 16) ? rep_names[report_t] : "Unknown");
    emit("AIS_ASM_ENVIRONMENTAL", pay, freq, ms, kv, ud);
}

/* FI=27: Route Information */
static void decode_fi27(const uint8_t* d, uint32_t db, int app_off,
                         uint32_t mmsi, int msg_type,
                         MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    int link_id    = (int)ais_u(d, db, app_off,     10);
    int sender_cla = (int)ais_u(d, db, app_off + 10, 2);
    int rtype      = (int)ais_u(d, db, app_off + 12, 4);
    int month      = (int)ais_u(d, db, app_off + 16, 4);
    int day        = (int)ais_u(d, db, app_off + 20, 5);
    int hour       = (int)ais_u(d, db, app_off + 25, 5);
    int minute     = (int)ais_u(d, db, app_off + 30, 6);
    int duration   = (int)ais_u(d, db, app_off + 36, 18);
    int wp_count   = (int)ais_u(d, db, app_off + 54,  5);

    /* Decode first waypoint if available */
    char wp1[48] = "N/A";
    if (wp_count > 0) {
        int64_t lon = ais_s(d, db, app_off + 59, 28);
        int64_t lat = ais_s(d, db, app_off + 87, 27);
        if (lon != 0x8000000 && lat != 0x4000000)
            snprintf(wp1, sizeof(wp1), "%.6f,%.6f",
                     lat / 600000.0, lon / 600000.0);
    }

    static const char* rtypes[] = {
        "N/A","Mandatory","Recommended","Alternative","Recommended through ice"
    };
    char kv[512], pay[256];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_ASM_ROUTE\",\"msg_type\":\"%d\",\"sender_mmsi\":\"%u\","
        "\"mmsi\":\"%u\",\"link_id\":\"%d\","
        "\"route_type\":\"%s\","
        "\"utc\":\"M%02d D%02d %02d:%02d\","
        "\"duration_min\":\"%d\","
        "\"waypoints\":\"%d\","
        "\"first_wp\":\"%s\"}",
        msg_type, mmsi, mmsi, link_id,
        (rtype < 5) ? rtypes[rtype] : "Unknown",
        month, day, hour, minute, duration, wp_count, wp1);
    snprintf(pay, sizeof(pay),
        "Route MMSI:%u [%d] Type:%s WPs:%d First:%s UTC:M%02d D%02d %02d:%02d Dur:%dmin",
        mmsi, link_id,
        (rtype < 5) ? rtypes[rtype] : "Unknown",
        wp_count, wp1, month, day, hour, minute, duration);
    emit("AIS_ASM_ROUTE", pay, freq, ms, kv, ud);
}

/* FI=28: Text using 6-bit ASCII (variable, up to ~14 chars per slot) */
static void decode_fi28(const uint8_t* d, uint32_t db, int app_off,
                         uint32_t mmsi, int msg_type,
                         MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    /* bits 0-1: sentence sequence; bit 2: last; bits 3-5: spare */
    int seq    = (int)ais_u(d, db, app_off, 2);
    int is_last = (int)ais_u(d, db, app_off + 2, 1);

    /* Remaining bits are 6-bit ASCII text */
    int text_start = app_off + 6;  /* after 2+1+3 header bits */
    int avail_bits = (int)(db * 8u) - text_start;
    if (avail_bits < 0) avail_bits = 0;
    int n_chars = avail_bits / 6;
    if (n_chars > 63) n_chars = 63;
    char text[64] = "";
    if (n_chars > 0) ais_str(d, db, text_start, n_chars, text);

    char kv[384], pay[256];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_ASM_TEXT\",\"msg_type\":\"%d\",\"sender_mmsi\":\"%u\","
        "\"mmsi\":\"%u\",\"seq\":\"%d\",\"last\":\"%d\",\"text\":\"%s\"}",
        msg_type, mmsi, mmsi, seq, is_last, text);
    snprintf(pay, sizeof(pay), "ASM Text MMSI:%u [%d%s] \"%s\"",
             mmsi, seq, is_last ? "/" : "+", text);
    emit("AIS_ASM_TEXT", pay, freq, ms, kv, ud);
}

/* FI=29: Marine Traffic Signal */
static void decode_fi29(const uint8_t* d, uint32_t db, int app_off,
                         uint32_t mmsi, int msg_type,
                         MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    int link_id  = (int)ais_u(d, db, app_off,     10);
    char name[15]; ais_str(d, db, app_off + 10, 14, name);
    int64_t lon  = ais_s(d, db, app_off + 94, 28);
    int64_t lat  = ais_s(d, db, app_off +122, 27);
    int status   = (int)ais_u(d, db, app_off +149,  2);
    int signal   = (int)ais_u(d, db, app_off +151,  5);
    int hour     = (int)ais_u(d, db, app_off +156,  5);
    int minute   = (int)ais_u(d, db, app_off +161,  6);
    static const char* stat_names[] = {"N/A","In Use","Not in Use","N/A"};
    char kv[512], pay[256];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_ASM_TRAFFIC_SIGNAL\",\"msg_type\":\"%d\",\"sender_mmsi\":\"%u\","
        "\"mmsi\":\"%u\",\"link_id\":\"%d\",\"name\":\"%s\","
        "\"lat\":\"%.6f\",\"lon\":\"%.6f\","
        "\"status\":\"%s\",\"signal\":\"%d\","
        "\"utc\":\"%02d:%02d\"}",
        msg_type, mmsi, mmsi, link_id, name,
        lat / 600000.0, lon / 600000.0,
        stat_names[status & 3], signal, hour, minute);
    snprintf(pay, sizeof(pay),
        "Traffic Signal MMSI:%u [%d] %s Lat:%.5f Lon:%.5f Status:%s Signal:%d %02d:%02d",
        mmsi, link_id, name,
        lat / 600000.0, lon / 600000.0,
        stat_names[status & 3], signal, hour, minute);
    emit("AIS_ASM_TRAFFIC_SIGNAL", pay, freq, ms, kv, ud);
}

/* FI=31: Met/Hydro Data (long), 352 app bits */
static void decode_fi31(const uint8_t* d, uint32_t db, int app_off,
                         uint32_t mmsi, int msg_type,
                         MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    int64_t lon  = ais_s(d, db, app_off,      25);  /* 1/10 min */
    int64_t lat  = ais_s(d, db, app_off + 25, 24);  /* 1/10 min */
    int pos_acc  = (int)ais_u(d, db, app_off + 49,  1);
    int utcday   = (int)ais_u(d, db, app_off + 50,  5);
    int utchour  = (int)ais_u(d, db, app_off + 55,  5);
    int utcmin   = (int)ais_u(d, db, app_off + 60,  6);
    int wspd     = (int)ais_u(d, db, app_off + 66,  7);   /* kn */
    int wgst     = (int)ais_u(d, db, app_off + 73,  7);   /* kn */
    int wdir     = (int)ais_u(d, db, app_off + 80,  9);   /* deg */
    int wgdir    = (int)ais_u(d, db, app_off + 89,  9);   /* deg */
    int64_t atmp = ais_s(d, db, app_off + 98, 11);        /* 0.1°C */
    int rhum     = (int)ais_u(d, db, app_off +109,  7);   /* % */
    int64_t dew  = ais_s(d, db, app_off +116, 10);        /* 0.1°C */
    int pres     = (int)ais_u(d, db, app_off +126,  9);   /* +800 hPa */
    int64_t ptr  = ais_s(d, db, app_off +135,  4);        /* hPa/3h */
    int vis      = (int)ais_u(d, db, app_off +139,  8);   /* 0.1 nm */
    int wlvl     = (int)ais_u(d, db, app_off +147,  9);   /* 0.1 m */
    int swell_h  = (int)ais_u(d, db, app_off +156,  8);   /* 0.1 m */
    int swell_p  = (int)ais_u(d, db, app_off +164,  6);   /* s */
    int swell_d  = (int)ais_u(d, db, app_off +170,  9);   /* deg */
    int64_t stmp = ais_s(d, db, app_off +179, 10);        /* 0.1°C */
    int prec     = (int)ais_u(d, db, app_off +189,  3);
    int salinity = (int)ais_u(d, db, app_off +192,  9);   /* 0.1‰ */
    int ice      = (int)ais_u(d, db, app_off +201,  2);

    /* Cleaner kv without compound literals */
    char kv[1024], pay[384];
    char ws[16], wd[16], at[16], rh[16], dp[16], pr[16], vi[16], sw[16];
    char wgsts[16], wgdrs[16], swells[16], stmps[16], sals[16];
    if (wgst ==127) snprintf(wgsts,16,"N/A"); else snprintf(wgsts,16,"%dkn",wgst);
    if (wgdir==360) snprintf(wgdrs,16,"N/A"); else snprintf(wgdrs,16,"%d°",wgdir);
    if (stmp ==601) snprintf(stmps,16,"N/A"); else snprintf(stmps,16,"%.1f°C",(double)stmp/10.0);
    if (salinity==511) snprintf(sals,16,"N/A"); else snprintf(sals,16,"%.1f‰",(double)salinity/10.0);

    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_ASM_METHYDRO_LONG\",\"msg_type\":\"%d\",\"sender_mmsi\":\"%u\","
        "\"mmsi\":\"%u\","
        "\"lat\":\"%.6f\",\"lon\":\"%.6f\","
        "\"utc\":\"D%02d %02d:%02d\","
        "\"wind_speed_kn\":\"%s\",\"wind_dir_deg\":\"%s\","
        "\"wind_gust_kn\":\"%s\",\"gust_dir_deg\":\"%s\","
        "\"air_temp_c\":\"%s\",\"rel_humidity_pct\":\"%s\","
        "\"dew_point_c\":\"%s\",\"pressure_hpa\":\"%s\","
        "\"pressure_trend\":\"%lld\","
        "\"visibility_nm\":\"%s\","
        "\"swell_height_m\":\"%s\",\"swell_period_s\":\"%d\","
        "\"sea_temp_c\":\"%s\","
        "\"precipitation\":\"%s\","
        "\"salinity_ppt\":\"%s\","
        "\"ice\":\"%s\"}",
        msg_type, mmsi, mmsi,
        lat / 600000.0, lon / 600000.0,
        utcday, utchour, utcmin,
        ws, wd, wgsts, wgdrs, at, rh, dp, pr,
        (long long)ptr, vi, sw, swell_p, stmps,
        precip_name(prec), sals,
        ice == 3 ? "N/A" : (ice == 1 ? "Yes" : "No"));

    snprintf(pay, sizeof(pay),
        "Met/Hydro(long) MMSI:%u Pos:%.5f,%.5f D%02d %02d:%02d "
        "Wind:%s@%s Gust:%s Temp:%s RH:%s Pres:%s Vis:%s Swell:%s Prec:%s",
        mmsi, lat/600000.0, lon/600000.0, utcday, utchour, utcmin,
        ws, wd, wgsts, at, rh, pr, vi, sw, precip_name(prec));
    emit("AIS_ASM_METHYDRO_LONG", pay, freq, ms, kv, ud);
}

/* FI=32: Tidal Window */
static void decode_fi32(const uint8_t* d, uint32_t db, int app_off,
                         uint32_t mmsi, int msg_type,
                         MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    int month = (int)ais_u(d, db, app_off,      4);
    int day   = (int)ais_u(d, db, app_off +  4, 5);
    /* Up to 3 tidal window entries follow; decode first one */
    int64_t lat1 = ais_s(d, db, app_off +  9, 27);
    int64_t lon1 = ais_s(d, db, app_off + 36, 28);
    int from_h   = (int)ais_u(d, db, app_off + 64, 5);
    int from_m   = (int)ais_u(d, db, app_off + 69, 6);
    int to_h     = (int)ais_u(d, db, app_off + 75, 5);
    int to_m     = (int)ais_u(d, db, app_off + 80, 6);
    int cdir     = (int)ais_u(d, db, app_off + 86, 9);  /* deg */
    int cspd     = (int)ais_u(d, db, app_off + 95, 8);  /* 0.1 kn */

    char kv[512], pay[256];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_ASM_TIDAL_WINDOW\",\"msg_type\":\"%d\",\"sender_mmsi\":\"%u\","
        "\"mmsi\":\"%u\","
        "\"date\":\"M%02d D%02d\","
        "\"lat\":\"%.6f\",\"lon\":\"%.6f\","
        "\"from\":\"%02d:%02d\",\"to\":\"%02d:%02d\","
        "\"current_dir_deg\":\"%d\","
        "\"current_spd_kn\":\"%.1f\"}",
        msg_type, mmsi, mmsi, month, day,
        lat1 / 600000.0, lon1 / 600000.0,
        from_h, from_m, to_h, to_m,
        cdir, (double)cspd / 10.0);
    snprintf(pay, sizeof(pay),
        "Tidal Window MMSI:%u M%02d D%02d Pos:%.5f,%.5f From:%02d:%02d To:%02d:%02d Curr:%d°@%.1fkn",
        mmsi, month, day,
        lat1/600000.0, lon1/600000.0,
        from_h, from_m, to_h, to_m,
        cdir, (double)cspd/10.0);
    emit("AIS_ASM_TIDAL_WINDOW", pay, freq, ms, kv, ud);
}

/* ── DAC/FI dispatch ──────────────────────────────────────────────────── */

static void dispatch_dac001(const uint8_t* d, uint32_t db,
                             int app_off, int fi, uint32_t mmsi, int msg_type,
                             MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    switch (fi) {
    case 11: decode_fi11_21(d, db, app_off, mmsi, 11, msg_type, emit, ud, freq, ms); break;
    case 13: decode_fi13(d, db, app_off, mmsi, msg_type, emit, ud, freq, ms); break;
    case 16: decode_fi16(d, db, app_off, mmsi, msg_type, emit, ud, freq, ms); break;
    case 17: decode_fi17(d, db, app_off, mmsi, msg_type, emit, ud, freq, ms); break;
    case 21: decode_fi11_21(d, db, app_off, mmsi, 21, msg_type, emit, ud, freq, ms); break;
    case 22: decode_fi22(d, db, app_off, mmsi, msg_type, emit, ud, freq, ms); break;
    case 26: decode_fi26(d, db, app_off, mmsi, msg_type, emit, ud, freq, ms); break;
    case 27: decode_fi27(d, db, app_off, mmsi, msg_type, emit, ud, freq, ms); break;
    case 28: decode_fi28(d, db, app_off, mmsi, msg_type, emit, ud, freq, ms); break;
    case 29: decode_fi29(d, db, app_off, mmsi, msg_type, emit, ud, freq, ms); break;
    case 31: decode_fi31(d, db, app_off, mmsi, msg_type, emit, ud, freq, ms); break;
    case 32: decode_fi32(d, db, app_off, mmsi, msg_type, emit, ud, freq, ms); break;
    default: {
        /* Unknown FI: emit raw hex */
        uint32_t app_byte = (uint32_t)(app_off / 8);
        uint32_t app_bytes = (db > app_byte) ? (db - app_byte) : 0u;
        char hex[128]; hex_encode(d, app_byte, app_bytes < 60u ? app_bytes : 60u,
                                   hex, sizeof(hex));
        char kv[384], pay[256];
        snprintf(kv, sizeof(kv),
            "{\"signal_type\":\"AIS_ASM_UNKNOWN\",\"msg_type\":\"%d\",\"sender_mmsi\":\"%u\","
            "\"mmsi\":\"%u\",\"dac\":\"1\",\"fi\":\"%d\",\"hex\":\"%s\"}",
            msg_type, mmsi, mmsi, fi, hex);
        snprintf(pay, sizeof(pay),
            "ASM MMSI:%u DAC:1 FI:%d Data:%s", mmsi, fi, hex);
        emit("AIS_ASM_UNKNOWN", pay, freq, ms, kv, ud);
    } break;
    }
}

/* ── Message type decoders ────────────────────────────────────────────── */

/* Msg 6: Addressed Binary Message (app data at bit 88) */
static void decode_msg6(const uint8_t* frm, uint32_t flen,
                         MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    if (flen < 12) return;
    const uint32_t db = flen - 2u;  /* strip FCS */
    uint32_t src_mmsi = (uint32_t)ais_u(frm, db, 8, 30);
    uint32_t dst_mmsi = (uint32_t)ais_u(frm, db, 40, 30);
    int dac = (int)ais_u(frm, db, 72, 10);
    int fi  = (int)ais_u(frm, db, 82, 6);

    if (dac == 1) {
        dispatch_dac001(frm, db, 88, fi, src_mmsi, 6, emit, ud, freq, ms);
        return;
    }

    /* Unknown DAC: raw hex */
    char hex[128]; hex_encode(frm, 11, db > 11 ? (db-11 < 60u ? db-11 : 60u) : 0u, hex, sizeof(hex));
    char kv[384], pay[256];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_MSG6\",\"msg_type\":\"6\",\"sender_mmsi\":\"%u\","
        "\"src_mmsi\":\"%u\",\"dst_mmsi\":\"%u\","
        "\"dac\":\"%d\",\"fi\":\"%d\",\"hex\":\"%s\"}",
        src_mmsi, src_mmsi, dst_mmsi, dac, fi, hex);
    snprintf(pay, sizeof(pay),
        "MSG6 Src:%u Dst:%u DAC:%d FI:%d Data:%s",
        src_mmsi, dst_mmsi, dac, fi, hex);
    emit("AIS_MSG6", pay, freq, ms, kv, ud);
}

/* Msg 8: Binary Broadcast Message (app data at bit 56) */
static void decode_msg8(const uint8_t* frm, uint32_t flen,
                         MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    if (flen < 7) return;
    const uint32_t db = flen - 2u;
    uint32_t mmsi = (uint32_t)ais_u(frm, db, 8, 30);
    int dac = (int)ais_u(frm, db, 40, 10);
    int fi  = (int)ais_u(frm, db, 50,  6);

    if (dac == 1) {
        dispatch_dac001(frm, db, 56, fi, mmsi, 8, emit, ud, freq, ms);
        return;
    }

    char hex[128]; hex_encode(frm, 7, db > 7 ? (db-7 < 60u ? db-7 : 60u) : 0u, hex, sizeof(hex));
    char kv[384], pay[256];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_MSG8\","
        "\"mmsi\":\"%u\",\"dac\":\"%d\",\"fi\":\"%d\",\"hex\":\"%s\"}",
        mmsi, dac, fi, hex);
    snprintf(pay, sizeof(pay),
        "MSG8 MMSI:%u DAC:%d FI:%d Data:%s", mmsi, dac, fi, hex);
    emit("AIS_MSG8", pay, freq, ms, kv, ud);
}

/* Msg 12: Addressed Safety-Related Message (text at bit 72) */
static void decode_msg12(const uint8_t* frm, uint32_t flen,
                          MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    if (flen < 10) return;
    const uint32_t db = flen - 2u;
    uint32_t src = (uint32_t)ais_u(frm, db, 8,  30);
    uint32_t dst = (uint32_t)ais_u(frm, db, 40, 30);
    int n = (int)(db * 8u - 72u) / 6;
    if (n < 0) n = 0;
    if (n > 63) n = 63;
    char text[64] = "";
    if (n > 0) ais_str(frm, db, 72, n, text);
    char kv[384], pay[256];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_MSG12\",\"msg_type\":\"12\",\"sender_mmsi\":\"%u\","
        "\"src_mmsi\":\"%u\",\"dst_mmsi\":\"%u\",\"text\":\"%s\"}",
        src, src, dst, text);
    snprintf(pay, sizeof(pay), "Safety(addr) Src:%u Dst:%u \"%s\"", src, dst, text);
    emit("AIS_MSG12", pay, freq, ms, kv, ud);
}

/* Msg 14: Safety-Related Broadcast Message (text at bit 40) */
static void decode_msg14(const uint8_t* frm, uint32_t flen,
                          MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    if (flen < 6) return;
    const uint32_t db = flen - 2u;
    uint32_t mmsi = (uint32_t)ais_u(frm, db, 8, 30);
    int n = (int)(db * 8u - 40u) / 6;
    if (n < 0) n = 0;
    if (n > 63) n = 63;
    char text[64] = "";
    if (n > 0) ais_str(frm, db, 40, n, text);
    char kv[256], pay[256];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_MSG14\",\"msg_type\":\"14\",\"sender_mmsi\":\"%u\","
        "\"mmsi\":\"%u\",\"text\":\"%s\"}", mmsi, mmsi, text);
    snprintf(pay, sizeof(pay), "Safety(bcast) MMSI:%u \"%s\"", mmsi, text);
    emit("AIS_MSG14", pay, freq, ms, kv, ud);
}

/* ── Standard AIS message decoders (mirror of ais_decoder) ───────────── */

static void decode_position(const uint8_t* d, uint32_t db, int mt,
                             MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    uint32_t mmsi = (uint32_t)ais_u(d, db, 8, 30);
    int is_a   = (mt <= 3);
    int b_sog  = is_a ? 50 : 46;
    int b_lon  = is_a ? 61 : 57;
    int b_lat  = is_a ? 89 : 85;
    int b_cog  = is_a ? 116 : 112;
    int b_hdg  = is_a ? 128 : 124;
    int64_t rlon = ais_s(d, db, b_lon, 28);
    int64_t rlat = ais_s(d, db, b_lat, 27);
    double  sog  = (double)ais_u(d, db, b_sog, 10) / 10.0;
    double  cog  = (double)ais_u(d, db, b_cog, 12) / 10.0;
    int     hdg  = (int)ais_u(d, db, b_hdg, 9);
    char    slat[20], slon[20];
    if (rlat == 54600000LL) snprintf(slat, 20, "N/A");
    else snprintf(slat, 20, "%.6f", rlat / 600000.0);
    if (rlon == 108600000LL) snprintf(slon, 20, "N/A");
    else snprintf(slon, 20, "%.6f", rlon / 600000.0);
    char kv[384], pay[192];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_POS\",\"msg_type\":\"%d\","
        "\"mmsi\":\"%u\",\"lat\":\"%s\",\"lon\":\"%s\","
        "\"sog\":\"%.1f\",\"cog\":\"%.1f\",\"hdg\":\"%d\"}",
        mt, mmsi, slat, slon, sog, cog, hdg == 511 ? -1 : hdg);
    snprintf(pay, sizeof(pay), "MMSI:%u Lat:%s Lon:%s SOG:%.1fkn COG:%.1f°",
             mmsi, slat, slon, sog, cog);
    emit("AIS_POS", pay, freq, ms, kv, ud);
}

static void decode_base_station(const uint8_t* d, uint32_t db,
                                 MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    uint32_t mmsi = (uint32_t)ais_u(d, db, 8, 30);
    int year  = (int)ais_u(d, db, 38, 14);
    int month = (int)ais_u(d, db, 52,  4);
    int day   = (int)ais_u(d, db, 56,  5);
    int hour  = (int)ais_u(d, db, 61,  5);
    int min   = (int)ais_u(d, db, 66,  6);
    int64_t rlon = ais_s(d, db, 79,  28);
    int64_t rlat = ais_s(d, db, 107, 27);
    char slat[20], slon[20];
    if (rlat == 54600000LL) snprintf(slat, 20, "N/A");
    else snprintf(slat, 20, "%.6f", rlat / 600000.0);
    if (rlon == 108600000LL) snprintf(slon, 20, "N/A");
    else snprintf(slon, 20, "%.6f", rlon / 600000.0);
    char kv[384], pay[192];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_BSR\",\"msg_type\":\"4\","
        "\"mmsi\":\"%u\",\"utc\":\"%04d-%02d-%02dT%02d:%02d:00Z\","
        "\"lat\":\"%s\",\"lon\":\"%s\"}",
        mmsi, year, month, day, hour, min, slat, slon);
    snprintf(pay, sizeof(pay), "MMSI:%u %04d-%02d-%02dT%02d:%02dZ Lat:%s Lon:%s",
             mmsi, year, month, day, hour, min, slat, slon);
    emit("AIS_BSR", pay, freq, ms, kv, ud);
}

static void decode_voyage(const uint8_t* d, uint32_t db,
                           MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    uint32_t mmsi = (uint32_t)ais_u(d, db, 8, 30);
    char name[21], callsign[8], dest[21];
    ais_str(d, db, 112, 20, name);
    ais_str(d, db,  70,  7, callsign);
    ais_str(d, db, 302, 20, dest);
    char kv[384], pay[192];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_STAT\",\"msg_type\":\"5\","
        "\"mmsi\":\"%u\",\"name\":\"%s\",\"callsign\":\"%s\",\"dest\":\"%s\"}",
        mmsi, name, callsign, dest);
    snprintf(pay, sizeof(pay), "MMSI:%u Name:%s CS:%s Dest:%s",
             mmsi, name, callsign, dest);
    emit("AIS_STAT", pay, freq, ms, kv, ud);
}

static void decode_aton(const uint8_t* d, uint32_t db,
                         MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    uint32_t mmsi = (uint32_t)ais_u(d, db, 8, 30);
    int aton_type = (int)ais_u(d, db, 38, 5);
    char name[21]; ais_str(d, db, 43, 20, name);
    int64_t rlon = ais_s(d, db, 164, 28);
    int64_t rlat = ais_s(d, db, 192, 27);
    char slat[20], slon[20];
    if (rlat == 54600000LL) snprintf(slat, 20, "N/A");
    else snprintf(slat, 20, "%.6f", rlat / 600000.0);
    if (rlon == 108600000LL) snprintf(slon, 20, "N/A");
    else snprintf(slon, 20, "%.6f", rlon / 600000.0);
    char kv[384], pay[192];
    snprintf(kv, sizeof(kv),
        "{\"signal_type\":\"AIS_ATON\",\"msg_type\":\"21\","
        "\"mmsi\":\"%u\",\"name\":\"%s\",\"type\":\"%d\","
        "\"lat\":\"%s\",\"lon\":\"%s\"}",
        mmsi, name, aton_type, slat, slon);
    snprintf(pay, sizeof(pay), "MMSI:%u AtoN:%d %s Lat:%s Lon:%s",
             mmsi, aton_type, name, slat, slon);
    emit("AIS_ATON", pay, freq, ms, kv, ud);
}

static void decode_static24(const uint8_t* d, uint32_t db,
                              MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    uint32_t mmsi   = (uint32_t)ais_u(d, db, 8, 30);
    int      part   = (int)ais_u(d, db, 38, 2);
    char kv[256], pay[128];
    if (part == 0) {
        char name[21]; ais_str(d, db, 40, 20, name);
        snprintf(kv, sizeof(kv),
            "{\"signal_type\":\"AIS_STAT24\",\"msg_type\":\"24\","
            "\"part\":\"A\",\"mmsi\":\"%u\",\"name\":\"%s\"}", mmsi, name);
        snprintf(pay, sizeof(pay), "MMSI:%u Name:%s", mmsi, name);
    } else {
        char cs[8], vendor[8];
        int stype = (int)ais_u(d, db, 40, 8);
        ais_str(d, db, 48, 7, vendor);
        ais_str(d, db, 90, 7, cs);
        snprintf(kv, sizeof(kv),
            "{\"signal_type\":\"AIS_STAT24\",\"msg_type\":\"24\","
            "\"part\":\"B\",\"mmsi\":\"%u\",\"callsign\":\"%s\","
            "\"ship_type\":\"%d\",\"vendor\":\"%s\"}",
            mmsi, cs, stype, vendor);
        snprintf(pay, sizeof(pay), "MMSI:%u CS:%s Type:%d", mmsi, cs, stype);
    }
    emit("AIS_STAT24", pay, freq, ms, kv, ud);
}

/* ── Frame dispatch ───────────────────────────────────────────────────── */

static void dispatch_frame(const uint8_t* frm, uint32_t flen,
                            MrEmitFn emit, void* ud, double freq, uint64_t ms) {
    if (flen < 5) return;
    const uint32_t db = flen - 2u;
    const int msg_type = (int)ais_u(frm, db, 0, 6);
    switch (msg_type) {
        /* ASM-specific types */
        case 6:  decode_msg6 (frm, flen, emit, ud, freq, ms); return;
        case 8:  decode_msg8 (frm, flen, emit, ud, freq, ms); return;
        case 12: decode_msg12(frm, flen, emit, ud, freq, ms); return;
        case 14: decode_msg14(frm, flen, emit, ud, freq, ms); return;
        /* Standard AIS types — decoded identically to ais_decoder */
        case 1: case 2: case 3: case 18:
            if (db >= 21) decode_position(frm, db, msg_type, emit, ud, freq, ms);
            return;
        case 4:
            if (db >= 21) decode_base_station(frm, db, emit, ud, freq, ms);
            return;
        case 5:
            if (db >= 53) decode_voyage(frm, db, emit, ud, freq, ms);
            return;
        case 21:
            if (db >= 34) decode_aton(frm, db, emit, ud, freq, ms);
            return;
        case 24:
            if (db >= 20) decode_static24(frm, db, emit, ud, freq, ms);
            return;
        default: break;
    }
    /* Fallback: emit raw for any other valid frame so traffic is visible */
    {
        uint32_t mmsi = (db >= 5) ? (uint32_t)ais_u(frm, db, 8, 30) : 0u;
        char hex[128]; hex_encode(frm, 0, db < 60u ? db : 60u, hex, sizeof(hex));
        char kv[256], pay[128];
        snprintf(kv, sizeof(kv),
            "{\"signal_type\":\"AIS_OTHER\",\"msg_type\":\"%d\","
            "\"mmsi\":\"%u\",\"byte_count\":\"%u\"}",
            msg_type, mmsi, db);
        snprintf(pay, sizeof(pay), "MMSI:%u Type:%d", mmsi, msg_type);
        emit("AIS_OTHER", pay, freq, ms, kv, ud);
    }
}

/* ── HDLC framing state ───────────────────────────────────────────────── */

typedef struct {
    int      consecutive_ones;
    int      in_frame;
    uint8_t  cur_byte;
    int      bit_pos;
    uint8_t  frame_buf[HDLC_MAX_FRAME];
    uint32_t frame_len;
} AsmCtx;

/* ── Plugin API ───────────────────────────────────────────────────────── */

static const MrPluginMeta kMeta = {
    "asm_decoder",
    "2.0.0",
    MR_PLUGIN_API_VERSION,
    "AIS ASM decoder: msg 6/8/12/14 + DAC=001 international ASMs (M.1371-5)",
    MR_PLUGIN_ROLE_POSTPROCESSING
};

MrPluginCtx* mr_plugin_create(void) {
    return (MrPluginCtx*)calloc(1, sizeof(AsmCtx));
}
void mr_plugin_destroy(MrPluginCtx* raw) { free(raw); }
const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
    (void)raw; (void)key; (void)value; return 0;
}

void mr_plugin_process_bits(MrPluginCtx* raw,
                             const uint8_t* bit_bytes, uint32_t bit_count,
                             double freq_hz, uint64_t unix_ms,
                             const char* source_type,
                             MrEmitFn emit_fn, void* user_data) {
    AsmCtx* ctx;
    uint32_t i;

    if (!raw || !bit_bytes || bit_count == 0) return;
    ctx = (AsmCtx*)raw;

    /* Byte-aligned handoff from ais_decoder (AIS_MSGX_RAW) */
    if (source_type && (
        strcmp(source_type, "AIS_MSG6_RAW") == 0 ||
        strcmp(source_type, "AIS_MSG8_RAW") == 0 ||
        strcmp(source_type, "AIS_MSG12_RAW") == 0 ||
        strcmp(source_type, "AIS_MSG14_RAW") == 0)) {
        // bit_bytes is actually a hex string here
        uint8_t frame_buf[HDLC_MAX_FRAME];
        uint32_t frame_len = hex_to_bytes((const char*)bit_bytes, frame_buf, sizeof(frame_buf));

        if (frame_len >= 5u) { // Minimum frame length for any meaningful AIS message
            dispatch_frame(frame_buf, frame_len, emit_fn, user_data, freq_hz, unix_ms);
        } else {
            ALOG("Received AIS_MSGX_RAW with too few bytes: %u\n", frame_len);
        }
        return;
    }

    /* Standalone HDLC decoder */
    for (i = 0; i < bit_count; ++i) {
        const int bit = (bit_bytes[i / 8] >> (7 - (i % 8))) & 1;
        if (bit) {
            ++ctx->consecutive_ones;
            if (ctx->consecutive_ones > 6) {
                ctx->in_frame = 0; ctx->frame_len = 0;
                ctx->bit_pos  = 0; ctx->cur_byte  = 0;
            }
            if (ctx->in_frame && ctx->consecutive_ones <= 5) {
                ctx->cur_byte |= (uint8_t)(1u << ctx->bit_pos);
                if (++ctx->bit_pos == 8) {
                    if (ctx->frame_len < HDLC_MAX_FRAME)
                        ctx->frame_buf[ctx->frame_len++] = ctx->cur_byte;
                    ctx->cur_byte = 0; ctx->bit_pos = 0;
                }
            }
        } else {
            if (ctx->consecutive_ones == 5) {
                ctx->consecutive_ones = 0; continue;
            } else if (ctx->consecutive_ones == 6) {
                ctx->consecutive_ones = 0;
                if (ctx->in_frame && ctx->frame_len >= 5) {
                    if (hdlc_check_crc(ctx->frame_buf, ctx->frame_len)) {
                        dispatch_frame(ctx->frame_buf, ctx->frame_len,
                                       emit_fn, user_data, freq_hz, unix_ms);
                    } else {
                        ALOG("CRC fail %u bytes\n", ctx->frame_len);
                    }
                }
                ctx->in_frame = 1; ctx->frame_len = 0;
                ctx->bit_pos  = 0; ctx->cur_byte  = 0;
                continue;
            } else {
                ctx->consecutive_ones = 0;
            }
            if (ctx->in_frame) {
                if (++ctx->bit_pos == 8) {
                    if (ctx->frame_len < HDLC_MAX_FRAME)
                        ctx->frame_buf[ctx->frame_len++] = ctx->cur_byte;
                    ctx->cur_byte = 0; ctx->bit_pos = 0;
                }
            }
        }
    }
}

void mr_plugin_process_iq(MrPluginCtx* ctx,
                           const int16_t* iq, uint32_t num_pairs,
                           uint32_t sr, double freq_hz, uint64_t unix_ms,
                           MrEmitFn emit_fn, void* user_data) {
    (void)ctx; (void)iq; (void)num_pairs; (void)sr;
    (void)freq_hz; (void)unix_ms; (void)emit_fn; (void)user_data;
}
