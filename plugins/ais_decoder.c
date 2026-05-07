/**
 * ais_decoder.c — AIS (Automatic Identification System) decoder plugin
 *
 * Role: MR_PLUGIN_ROLE_POSTPROCESSING
 *
 * Performs HDLC frame extraction (CRC-16-CCITT, LSB-first byte assembly)
 * and decodes AIS messages per ITU-R M.1371-5.
 *
 * Supported message types:
 *   1, 2, 3  — Class A Position Report (168 bits)
 *   5        — Static and Voyage Related Data (426 bits)
 *   18       — Standard Class B CS Position Report (168 bits)
 *   24A/B    — Class B CS Static Data Report (160 / 168 bits)
 *   Other    — Emitted as AIS_OTHER with raw hex payload
 *
 * Emitted signal types:
 *   AIS_POS     payload: "MMSI:N Lat:D Lon:D SOG:N COG:N"
 *   AIS_STAT    payload: "MMSI:N Name:X CS:X Dest:X"
 *   AIS_STAT24  payload: "MMSI:N Name:X / CS:X"
 *   AIS_OTHER   payload: hex bytes
 */

#include "mr_plugin_api.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* CRC-16-CCITT (poly 0x8408 reflected, init 0xFFFF, residue 0xF0B8)  */
/* ------------------------------------------------------------------ */

#define HDLC_CRC_INIT    0xFFFFu
#define HDLC_CRC_RESIDUE 0xF0B8u
#define HDLC_CRC_POLY    0x8408u
#define HDLC_MAX_FRAME   512u   /* bytes; AIS longest is type 5 at ~54 bytes */

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
    for (i = 0; i < len; ++i)
        crc = crc16_byte(crc, buf[i]);
    return (crc == HDLC_CRC_RESIDUE);
}

/* ------------------------------------------------------------------ */
/* AIS bit extraction (LSB-first byte layout from HDLC assembly)       */
/* ------------------------------------------------------------------ */

/* Extract `len` bits starting at bit `start` (bit 0 = first received).
   First AIS bit is returned as MSB of the result (AIS fields are MSB-first). */
static uint64_t ais_bits(const uint8_t* data, uint32_t data_bytes,
                          int start, int len) {
    uint64_t r = 0;
    int i;
    for (i = 0; i < len; ++i) {
        int abs  = start + i;
        int bidx = abs / 8;
        int boff = 7 - (abs % 8);  /* MSB-first: first AIS bit is MSB of byte 0 */
        if (bidx < (int)data_bytes && (data[bidx] & (1u << boff)))
            r |= (1ULL << (len - 1 - i));
    }
    return r;
}

/* Sign-extend a two's-complement value of `len` bits. */
static int64_t ais_signed(const uint8_t* data, uint32_t data_bytes,
                           int start, int len) {
    uint64_t u = ais_bits(data, data_bytes, start, len);
    if (u & (1ULL << (len - 1)))
        u |= ~((1ULL << len) - 1ULL);
    return (int64_t)u;
}

/* Decode `num_chars` × 6-bit AIS characters starting at `start_bit`. */
static void ais_text(const uint8_t* data, uint32_t data_bytes,
                     int start_bit, int num_chars, char* out) {
    int i;
    for (i = 0; i < num_chars; ++i) {
        uint64_t v = ais_bits(data, data_bytes, start_bit + i * 6, 6);
        out[i] = (char)(v < 32u ? v + 64u : v);
    }
    /* Trim trailing spaces and @ (null in AIS text) */
    int end = num_chars;
    while (end > 0 && (out[end - 1] == ' ' || out[end - 1] == '@'))
        --end;
    out[end] = '\0';
}

/* ------------------------------------------------------------------ */
/* AIS message decoders                                                 */
/* ------------------------------------------------------------------ */

/* Types 1, 2, 3 (Class A position) and 18 (Class B position) */
static void decode_position(const uint8_t* d, uint32_t bytes,
                             int msg_type,
                             char* kv, size_t kv_sz,
                             char* payload, size_t pay_sz) {
    uint32_t mmsi = (uint32_t)ais_bits(d, bytes, 8, 30);
    int is_a  = (msg_type <= 3);
    int nav   = is_a ? (int)ais_bits(d, bytes, 38, 4) : -1;
    int b_sog = is_a ? 50 : 46;
    int b_acc = is_a ? 60 : 56;
    int b_lon = is_a ? 61 : 57;
    int b_lat = is_a ? 89 : 85;
    int b_cog = is_a ? 116 : 112;
    int b_hdg = is_a ? 128 : 124;

    int64_t raw_lon = ais_signed(d, bytes, b_lon, 28);
    int64_t raw_lat = ais_signed(d, bytes, b_lat, 27);
    double  sog     = (double)ais_bits(d, bytes, b_sog, 10) / 10.0;
    double  cog     = (double)ais_bits(d, bytes, b_cog, 12) / 10.0;
    int     hdg     = (int)ais_bits(d, bytes, b_hdg, 9);
    int     acc     = (int)ais_bits(d, bytes, b_acc, 1);

    /* Sentinels: 181*600000 = 108600000, 91*600000 = 54600000 */
    int lon_na = (raw_lon == 108600000LL);
    int lat_na = (raw_lat == 54600000LL);

    char slat[20], slon[20];
    if (lat_na) snprintf(slat, sizeof(slat), "N/A");
    else        snprintf(slat, sizeof(slat), "%.6f", raw_lat / 600000.0);
    if (lon_na) snprintf(slon, sizeof(slon), "N/A");
    else        snprintf(slon, sizeof(slon), "%.6f", raw_lon / 600000.0);

    snprintf(kv, kv_sz,
        "{\"signal_type\":\"AIS_POS\","
        "\"msg_type\":\"%d\","
        "\"mmsi\":\"%u\","
        "\"lat\":\"%s\","
        "\"lon\":\"%s\","
        "\"sog\":\"%.1f\","
        "\"cog\":\"%.1f\","
        "\"hdg\":\"%d\","
        "\"nav_status\":\"%d\","
        "\"pos_acc\":\"%d\"}",
        msg_type, mmsi, slat, slon,
        sog, cog, (hdg == 511 ? -1 : hdg), nav, acc);

    if (lat_na || lon_na)
        snprintf(payload, pay_sz, "MMSI:%u SOG:%.1fkn COG:%.1f° Pos:N/A",
                 mmsi, sog, cog);
    else
        snprintf(payload, pay_sz,
                 "MMSI:%u Lat:%s Lon:%s SOG:%.1fkn COG:%.1f°",
                 mmsi, slat, slon, sog, cog);
}

/* Type 5 (Static and Voyage Related Data, Class A) */
static void decode_voyage(const uint8_t* d, uint32_t bytes,
                           char* kv, size_t kv_sz,
                           char* payload, size_t pay_sz) {
    uint32_t mmsi = (uint32_t)ais_bits(d, bytes, 8, 30);
    char name[21], callsign[8], dest[21];
    ais_text(d, bytes, 112, 20, name);
    ais_text(d, bytes, 70,  7,  callsign);
    ais_text(d, bytes, 302, 20, dest);
    double draught = (double)ais_bits(d, bytes, 294, 8) / 10.0;
    int ship_type  = (int)ais_bits(d, bytes, 232, 8);

    snprintf(kv, kv_sz,
        "{\"signal_type\":\"AIS_STAT\","
        "\"msg_type\":\"5\","
        "\"mmsi\":\"%u\","
        "\"name\":\"%s\","
        "\"callsign\":\"%s\","
        "\"dest\":\"%s\","
        "\"draught\":\"%.1f\","
        "\"ship_type\":\"%d\"}",
        mmsi, name, callsign, dest, draught, ship_type);

    snprintf(payload, pay_sz,
             "MMSI:%u Name:%s CS:%s Dest:%s",
             mmsi, name, callsign, dest);
}

/* Type 24 (Class B CS Static Data Report) */
static void decode_static24(const uint8_t* d, uint32_t bytes,
                              char* kv, size_t kv_sz,
                              char* payload, size_t pay_sz) {
    uint32_t mmsi    = (uint32_t)ais_bits(d, bytes, 8, 30);
    int      part_no = (int)ais_bits(d, bytes, 38, 2);
    if (part_no == 0) {
        char name[21];
        ais_text(d, bytes, 40, 20, name);
        snprintf(kv, kv_sz,
            "{\"signal_type\":\"AIS_STAT24\","
            "\"msg_type\":\"24\","
            "\"part\":\"A\","
            "\"mmsi\":\"%u\","
            "\"name\":\"%s\"}",
            mmsi, name);
        snprintf(payload, pay_sz, "MMSI:%u Name:%s", mmsi, name);
    } else {
        char callsign[8], vendor[8];
        int ship_type = (int)ais_bits(d, bytes, 40, 8);
        ais_text(d, bytes, 48, 7, vendor);
        ais_text(d, bytes, 90, 7, callsign);
        snprintf(kv, kv_sz,
            "{\"signal_type\":\"AIS_STAT24\","
            "\"msg_type\":\"24\","
            "\"part\":\"B\","
            "\"mmsi\":\"%u\","
            "\"callsign\":\"%s\","
            "\"ship_type\":\"%d\","
            "\"vendor\":\"%s\"}",
            mmsi, callsign, ship_type, vendor);
        snprintf(payload, pay_sz, "MMSI:%u CS:%s Type:%d",
                 mmsi, callsign, ship_type);
    }
}

/* Type 8 (Binary Broadcast Message) */
static void decode_bbm(const uint8_t* d, uint32_t bytes,
                        char* kv, size_t kv_sz,
                        char* payload, size_t pay_sz) {
    uint32_t mmsi = (uint32_t)ais_bits(d, bytes, 8, 30);
    int      dac  = (int)ais_bits(d, bytes, 40, 10);
    int      fi   = (int)ais_bits(d, bytes, 50,  6);

    /* Application data begins at byte 7 (bit 56).
       With MSB-first storage, frame_buf[7..bytes-1] are the app data bytes. */
    const uint32_t app_off   = 7u;
    const uint32_t app_bytes = (bytes > app_off) ? (bytes - app_off) : 0u;

    /* Hex-encode app data (max AIS frame gives ≤14 bytes = 28 hex chars) */
    char app_hex[64] = "";
    if (app_bytes > 0) {
        uint32_t i;
        uint32_t lim = app_bytes < 28u ? app_bytes : 28u; /* safety cap */
        for (i = 0; i < lim; ++i)
            snprintf(app_hex + i * 2, 3, "%02X", (unsigned)d[app_off + i]);
    }

    snprintf(kv, kv_sz,
        "{\"signal_type\":\"AIS_BBM\","
        "\"msg_type\":\"8\","
        "\"mmsi\":\"%u\","
        "\"dac\":\"%d\","
        "\"fi\":\"%d\","
        "\"app_data_bytes\":\"%u\","
        "\"app_data\":\"%s\"}",
        mmsi, dac, fi, app_bytes, app_hex);

    snprintf(payload, pay_sz,
             "MMSI:%u DAC:%d FI:%d Data:%s",
             mmsi, dac, fi, app_hex[0] ? app_hex : "(none)");
}

/* Dispatch to per-type decoder and emit. */
static void decode_and_emit(const uint8_t* frame_buf, uint32_t frame_len,
                             double freq_hz, uint64_t unix_ms,
                             MrEmitFn emit_fn, void* user_data) {
    /* Minimum: 3 bytes data + 2 CRC = at least message type + MMSI + FCS */
    if (frame_len < 5) return;

    const uint32_t data_bytes = frame_len - 2; /* strip 2-byte FCS */
    const int msg_type = (int)ais_bits(frame_buf, data_bytes, 0, 6);

    char kv[512];
    char payload[256];

    switch (msg_type) {
    case 1: case 2: case 3: case 18:
        if (data_bytes < 21) return;  /* 168 bits = 21 bytes */
        decode_position(frame_buf, data_bytes, msg_type,
                        kv, sizeof(kv), payload, sizeof(payload));
        break;
    case 5:
        if (data_bytes < 53) return;  /* 426 bits = 53.25 bytes */
        decode_voyage(frame_buf, data_bytes,
                      kv, sizeof(kv), payload, sizeof(payload));
        break;
    case 8:
        if (data_bytes < 7) return;  /* 56 bits = 7 bytes minimum */
        decode_bbm(frame_buf, data_bytes,
                   kv, sizeof(kv), payload, sizeof(payload));
        break;
    case 24:
        if (data_bytes < 20) return;
        decode_static24(frame_buf, data_bytes,
                        kv, sizeof(kv), payload, sizeof(payload));
        break;
    default: {
        uint32_t mmsi = (data_bytes >= 5) ?
                        (uint32_t)ais_bits(frame_buf, data_bytes, 8, 30) : 0;
        uint32_t i;
        /* Build hex payload */
        char* hex = (char*)malloc(data_bytes * 2 + 1);
        if (!hex) return;
        for (i = 0; i < data_bytes; ++i)
            snprintf(hex + i * 2, 3, "%02X", (unsigned)frame_buf[i]);
        snprintf(kv, sizeof(kv),
                 "{\"signal_type\":\"AIS_OTHER\",\"msg_type\":\"%d\","
                 "\"mmsi\":\"%u\",\"byte_count\":\"%u\"}",
                 msg_type, mmsi, data_bytes);
        snprintf(payload, sizeof(payload), "MMSI:%u Type:%d", mmsi, msg_type);
        if (emit_fn) emit_fn("AIS_OTHER", hex, freq_hz, unix_ms, kv, user_data);
        free(hex);
        return;
    }
    }

    const char* sig = (msg_type == 5)  ? "AIS_STAT"
                    : (msg_type == 8)  ? "AIS_BBM"
                    : (msg_type == 24) ? "AIS_STAT24"
                    : "AIS_POS";
    if (emit_fn) emit_fn(sig, payload, freq_hz, unix_ms, kv, user_data);
}

/* ------------------------------------------------------------------ */
/* HDLC framing state                                                   */
/* ------------------------------------------------------------------ */

typedef struct {
    int      consecutive_ones;
    int      in_frame;
    uint8_t  cur_byte;
    int      bit_pos;
    uint8_t  frame_buf[HDLC_MAX_FRAME];
    uint32_t frame_len;
} AisCtx;

/* ------------------------------------------------------------------ */
/* Plugin API                                                           */
/* ------------------------------------------------------------------ */

static const MrPluginMeta kMeta = {
    "ais_decoder",
    "1.0.0",
    MR_PLUGIN_API_VERSION,
    "AIS decoder: HDLC framing + ITU-R M.1371-5 message parsing",
    MR_PLUGIN_ROLE_POSTPROCESSING
};

MrPluginCtx* mr_plugin_create(void) {
    return (MrPluginCtx*)calloc(1, sizeof(AisCtx));
}

void mr_plugin_destroy(MrPluginCtx* raw) { free(raw); }

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
    (void)raw; (void)key; (void)value;
    return 0;
}

void mr_plugin_process_bits(MrPluginCtx* raw,
                            const uint8_t* bit_bytes, uint32_t bit_count,
                            double freq_hz, uint64_t unix_ms,
                            const char* source_type,
                            MrEmitFn emit_fn, void* user_data) {
    (void)source_type;
    if (!raw || !bit_bytes || !bit_count) return;
    AisCtx* ctx = (AisCtx*)raw;

    uint32_t i;
    for (i = 0; i < bit_count; ++i) {
        const int bit = (bit_bytes[i / 8] >> (7 - (i % 8))) & 1;

        if (bit) {
            ++ctx->consecutive_ones;
            if (ctx->consecutive_ones > 6) {
                ctx->in_frame  = 0;
                ctx->frame_len = 0;
                ctx->bit_pos   = 0;
                ctx->cur_byte  = 0;
            }
            if (ctx->in_frame && ctx->consecutive_ones <= 5) {
                ctx->cur_byte |= (uint8_t)(1u << ctx->bit_pos);
                if (++ctx->bit_pos == 8) {
                    if (ctx->frame_len < HDLC_MAX_FRAME)
                        ctx->frame_buf[ctx->frame_len++] = ctx->cur_byte;
                    ctx->cur_byte = 0;
                    ctx->bit_pos  = 0;
                }
            }
        } else {
            if (ctx->consecutive_ones == 5) {
                ctx->consecutive_ones = 0;
                continue; /* bit stuffing — discard */
            } else if (ctx->consecutive_ones == 6) {
                ctx->consecutive_ones = 0;
                if (ctx->in_frame && ctx->frame_len >= 5 &&
                    hdlc_check_crc(ctx->frame_buf, ctx->frame_len)) {
                    decode_and_emit(ctx->frame_buf, ctx->frame_len,
                                    freq_hz, unix_ms, emit_fn, user_data);
                }
                ctx->in_frame  = 1;
                ctx->frame_len = 0;
                ctx->bit_pos   = 0;
                ctx->cur_byte  = 0;
                continue;
            } else {
                ctx->consecutive_ones = 0;
            }
            if (ctx->in_frame) {
                if (++ctx->bit_pos == 8) {
                    if (ctx->frame_len < HDLC_MAX_FRAME)
                        ctx->frame_buf[ctx->frame_len++] = ctx->cur_byte;
                    ctx->cur_byte = 0;
                    ctx->bit_pos  = 0;
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
