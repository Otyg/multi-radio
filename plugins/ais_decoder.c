/**
 * ais_decoder.c — AIS (Automatic Identification System) decoder plugin
 *
 * Role: MR_PLUGIN_ROLE_POSTPROCESSING
 *
 * Performs HDLC frame extraction (CRC-16-CCITT, LSB-first byte assembly)
 * and decodes AIS messages per ITU-R M.1371-5.
 *
 * Supported message types:
 *   6        — Addressed Binary Message (handed off to asm_decoder)
 *   1, 2, 3  — Class A Position Report (168 bits)
 *   4        — Base Station Report (168 bits)
 *   5        — Static and Voyage Related Data (426 bits)
 *   8        — Binary Broadcast Message (handed off to asm_decoder)
 *   9        — Standard SAR Aircraft Position Report (168 bits)
 *   12       — Addressed Safety-Related Message (handed off to asm_decoder)
 *   18       — Standard Class B CS Position Report (168 bits)
 *   19       — Class B CS Extended Position Report (312 bits, includes name)
 *   21       — Aid-to-Navigation Report (272+ bits, includes name)
 *   24A/B    — Class B CS Static Data Report (160 / 168 bits)
 *   Other    — Emitted as AIS_OTHER with raw hex payload
 *
 * Emitted signal types:
 *   AIS_POS     payload: "MMSI:N Lat:D Lon:D SOG:N COG:N"
 *   AIS_STAT    payload: "MMSI:N Name:X CS:X Dest:X"
 *   AIS_MSG6_RAW payload: hex bytes (full frame including FCS)
 *   AIS_MSG8_RAW payload: hex bytes (full frame including FCS)
 *   AIS_MSG12_RAW payload: hex bytes (full frame including FCS)
 *   AIS_STAT24  payload: "MMSI:N Name:X / CS:X"
 *   AIR-SAR     payload: "MMSI:N Lat:D Lon:D Alt:Nft SOG:N COG:N"
 *   AIS_OTHER   payload: hex bytes
 */

#include "mr_plugin_api.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int ais_dec_dbg(void) {
    static int v = -1;
    if (v < 0) { const char* e = getenv("MR_AIS_DEBUG"); v = (e && e[0] != '0') ? 1 : 0; }
    return v;
}
#define ALOG(...) do { if (ais_dec_dbg()) fprintf(stderr, "[ais_dec] " __VA_ARGS__); } while (0)

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

/* Type 9 (Standard SAR Aircraft Position Report) — 168 bits = 21 bytes */
static void decode_sar_position(const uint8_t* d, uint32_t bytes,
                                 char* kv, size_t kv_sz,
                                 char* payload, size_t pay_sz) {
    uint32_t mmsi    = (uint32_t)ais_bits(d, bytes, 8, 30);
    int      alt_m   = (int)ais_bits(d, bytes, 38, 12);  /* meters; 4095 = N/A */
    double   sog     = (double)ais_bits(d, bytes, 50, 10) / 10.0;
    int64_t  raw_lon = ais_signed(d, bytes, 61, 28);
    int64_t  raw_lat = ais_signed(d, bytes, 89, 27);
    double   cog     = (double)ais_bits(d, bytes, 116, 12) / 10.0;

    int lon_na = (raw_lon == 108600000LL);
    int lat_na = (raw_lat ==  54600000LL);
    char slat[20], slon[20];
    if (lat_na) snprintf(slat, sizeof(slat), "N/A");
    else        snprintf(slat, sizeof(slat), "%.6f", raw_lat / 600000.0);
    if (lon_na) snprintf(slon, sizeof(slon), "N/A");
    else        snprintf(slon, sizeof(slon), "%.6f", raw_lon / 600000.0);

    /* 4095 = not available → 0 ft; otherwise convert meters → feet */
    double alt_ft = (alt_m == 4095) ? 0.0 : alt_m * 3.28084;

    snprintf(kv, kv_sz,
        "{\"signal_type\":\"AIR-SAR\","
        "\"msg_type\":\"9\","
        "\"mmsi\":\"%u\","
        "\"lat\":\"%s\","
        "\"lon\":\"%s\","
        "\"sog\":\"%.1f\","
        "\"cog\":\"%.1f\","
        "\"alt_baro\":\"%.0f\"}",
        mmsi, slat, slon, sog, cog, alt_ft);

    if (lat_na || lon_na)
        snprintf(payload, pay_sz,
                 "MMSI:%u Alt:%.0fft SOG:%.1fkn Pos:N/A",
                 mmsi, alt_ft, sog);
    else
        snprintf(payload, pay_sz,
                 "MMSI:%u Lat:%s Lon:%s Alt:%.0fft SOG:%.1fkn COG:%.1f\xc2\xb0",
                 mmsi, slat, slon, alt_ft, sog, cog);
}

/* Type 4 (Base Station Report) — 168 bits */
static void decode_base_station(const uint8_t* d, uint32_t bytes,
                                  char* kv, size_t kv_sz,
                                  char* payload, size_t pay_sz) {
    uint32_t mmsi   = (uint32_t)ais_bits(d, bytes, 8, 30);
    int year        = (int)ais_bits(d, bytes, 38, 14);
    int month       = (int)ais_bits(d, bytes, 52,  4);
    int day         = (int)ais_bits(d, bytes, 56,  5);
    int hour        = (int)ais_bits(d, bytes, 61,  5);
    int minute      = (int)ais_bits(d, bytes, 66,  6);
    int second      = (int)ais_bits(d, bytes, 72,  6);
    int64_t raw_lon = ais_signed(d, bytes, 79,  28);
    int64_t raw_lat = ais_signed(d, bytes, 107, 27);

    int lon_na = (raw_lon == 108600000LL);
    int lat_na = (raw_lat ==  54600000LL);
    char slat[20], slon[20];
    if (lat_na) snprintf(slat, sizeof(slat), "N/A");
    else        snprintf(slat, sizeof(slat), "%.6f", raw_lat / 600000.0);
    if (lon_na) snprintf(slon, sizeof(slon), "N/A");
    else        snprintf(slon, sizeof(slon), "%.6f", raw_lon / 600000.0);

    snprintf(kv, kv_sz,
        "{\"signal_type\":\"AIS_BSR\","
        "\"msg_type\":\"4\","
        "\"mmsi\":\"%u\","
        "\"utc\":\"%04d-%02d-%02dT%02d:%02d:%02dZ\","
        "\"lat\":\"%s\","
        "\"lon\":\"%s\"}",
        mmsi, year, month, day, hour, minute, second, slat, slon);

    snprintf(payload, pay_sz,
             "MMSI:%u %04d-%02d-%02dT%02d:%02d:%02dZ Lat:%s Lon:%s",
             mmsi, year, month, day, hour, minute, second, slat, slon);
}

/* Type 21 (Aid-to-Navigation Report) — 272 bits minimum */
static void decode_aton(const uint8_t* d, uint32_t bytes,
                         char* kv, size_t kv_sz,
                         char* payload, size_t pay_sz) {
    uint32_t mmsi    = (uint32_t)ais_bits(d, bytes, 8, 30);
    int type_of_aton = (int)ais_bits(d, bytes, 38, 5);
    char name[21];
    ais_text(d, bytes, 43, 20, name);
    int64_t raw_lon = ais_signed(d, bytes, 164, 28);
    int64_t raw_lat = ais_signed(d, bytes, 192, 27);

    int lon_na = (raw_lon == 108600000LL);
    int lat_na = (raw_lat ==  54600000LL);
    char slat[20], slon[20];
    if (lat_na) snprintf(slat, sizeof(slat), "N/A");
    else        snprintf(slat, sizeof(slat), "%.6f", raw_lat / 600000.0);
    if (lon_na) snprintf(slon, sizeof(slon), "N/A");
    else        snprintf(slon, sizeof(slon), "%.6f", raw_lon / 600000.0);

    snprintf(kv, kv_sz,
        "{\"signal_type\":\"AIS_ATON\","
        "\"msg_type\":\"21\","
        "\"mmsi\":\"%u\","
        "\"name\":\"%s\","
        "\"type_of_aton\":\"%d\","
        "\"lat\":\"%s\","
        "\"lon\":\"%s\"}",
        mmsi, name, type_of_aton, slat, slon);

    snprintf(payload, pay_sz,
             "MMSI:%u Name:%s AtoN:%d Lat:%s Lon:%s",
             mmsi, name, type_of_aton, slat, slon);
}

/* Type 19 (Class B CS Extended Position Report) — 312 bits = 39 bytes */
static void decode_extended_position(const uint8_t* d, uint32_t bytes,
                                      char* kv, size_t kv_sz,
                                      char* payload, size_t pay_sz) {
    uint32_t mmsi   = (uint32_t)ais_bits(d, bytes, 8, 30);
    double   sog    = (double)ais_bits(d, bytes, 42, 10) / 10.0;
    int64_t  raw_lon = ais_signed(d, bytes, 53, 28);
    int64_t  raw_lat = ais_signed(d, bytes, 81, 27);
    double   cog    = (double)ais_bits(d, bytes, 108, 12) / 10.0;
    int      hdg    = (int)ais_bits(d, bytes, 120, 9);
    char     name[21];
    ais_text(d, bytes, 143, 20, name);
    int      ship_type = (int)ais_bits(d, bytes, 263, 8);

    int lon_na = (raw_lon == 108600000LL);
    int lat_na = (raw_lat ==  54600000LL);
    char slat[20], slon[20];
    if (lat_na) snprintf(slat, sizeof(slat), "N/A");
    else        snprintf(slat, sizeof(slat), "%.6f", raw_lat / 600000.0);
    if (lon_na) snprintf(slon, sizeof(slon), "N/A");
    else        snprintf(slon, sizeof(slon), "%.6f", raw_lon / 600000.0);

    snprintf(kv, kv_sz,
        "{\"signal_type\":\"AIS_POS\","
        "\"msg_type\":\"19\","
        "\"mmsi\":\"%u\","
        "\"lat\":\"%s\","
        "\"lon\":\"%s\","
        "\"sog\":\"%.1f\","
        "\"cog\":\"%.1f\","
        "\"hdg\":\"%d\","
        "\"name\":\"%s\","
        "\"ship_type\":\"%d\"}",
        mmsi, slat, slon, sog, cog,
        (hdg == 511 ? -1 : hdg), name, ship_type);

    if (lat_na || lon_na)
        snprintf(payload, pay_sz, "MMSI:%u Name:%s SOG:%.1fkn Pos:N/A",
                 mmsi, name, sog);
    else
        snprintf(payload, pay_sz,
                 "MMSI:%u Name:%s Lat:%s Lon:%s SOG:%.1fkn COG:%.1f°",
                 mmsi, name, slat, slon, sog, cog);
}

/* Generic handoff: emit raw AIS frame (+FCS) to asm_decoder. */
static void emit_msg_raw_for_asm(const uint8_t* frame_buf, uint32_t frame_len,
                                  double freq_hz, uint64_t unix_ms,
                                  MrEmitFn emit_fn, void* user_data,
                                  int msg_type, const char* signal_name) {
    const uint32_t data_bytes = (frame_len >= 2u) ? (frame_len - 2u) : 0u;
    const uint32_t mmsi = (data_bytes >= 5u)
                              ? (uint32_t)ais_bits(frame_buf, data_bytes, 8, 30)
                              : 0u;
    uint32_t i;
    char kv[256];
    char* hex_payload;
    if (!emit_fn || frame_len == 0u) return;

    hex_payload = (char*)malloc(frame_len * 2u + 1u);
    if (!hex_payload) return;
    for (i = 0; i < frame_len; ++i)
        snprintf(hex_payload + i * 2u, 3, "%02X", (unsigned)frame_buf[i]);

    snprintf(kv, sizeof(kv),
             "{\"signal_type\":\"%s\","
             "\"msg_type\":%d,"
             "\"mmsi\":\"%u\","
             "\"frame_bytes\":\"%u\","
             "\"hex\":\"%s\"}",
             signal_name, msg_type, mmsi, frame_len, hex_payload);
    emit_fn(signal_name, hex_payload, freq_hz, unix_ms, kv, user_data);
    free(hex_payload);
}

/* Type 8 handoff: emit raw AIS frame (+FCS) to asm_decoder. */
static void emit_msg8_raw_for_asm(const uint8_t* frame_buf, uint32_t frame_len,
                                   double freq_hz, uint64_t unix_ms,
                                   MrEmitFn emit_fn, void* user_data) {
    emit_msg_raw_for_asm(frame_buf, frame_len, freq_hz, unix_ms, emit_fn, user_data, 8, "AIS_MSG8_RAW");
}

/* Type 6 handoff: emit raw AIS frame (+FCS) to asm_decoder. */
static void emit_msg6_raw_for_asm(const uint8_t* frame_buf, uint32_t frame_len,
                                   double freq_hz, uint64_t unix_ms,
                                   MrEmitFn emit_fn, void* user_data) {
    emit_msg_raw_for_asm(frame_buf, frame_len, freq_hz, unix_ms, emit_fn, user_data, 6, "AIS_MSG6_RAW");
}

/* Type 12 handoff: emit raw AIS frame (+FCS) to asm_decoder. */
static void emit_msg12_raw_for_asm(const uint8_t* frame_buf, uint32_t frame_len,
                                    double freq_hz, uint64_t unix_ms,
                                    MrEmitFn emit_fn, void* user_data) {
    emit_msg_raw_for_asm(frame_buf, frame_len, freq_hz, unix_ms, emit_fn, user_data, 12, "AIS_MSG12_RAW");
}

/* Type 14 handoff: emit raw AIS frame (+FCS) to asm_decoder. */
static void emit_msg14_raw_for_asm(const uint8_t* frame_buf, uint32_t frame_len,
                                    double freq_hz, uint64_t unix_ms,
                                    MrEmitFn emit_fn, void* user_data) {
    emit_msg_raw_for_asm(frame_buf, frame_len, freq_hz, unix_ms, emit_fn, user_data, 14, "AIS_MSG14_RAW");
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
    case 9:
        if (data_bytes < 21) return;  /* 168 bits = 21 bytes */
        decode_sar_position(frame_buf, data_bytes,
                            kv, sizeof(kv), payload, sizeof(payload));
        break;
    case 4:
        if (data_bytes < 21) return;  /* 168 bits = 21 bytes */
        decode_base_station(frame_buf, data_bytes,
                            kv, sizeof(kv), payload, sizeof(payload));
        break;
    case 5:
        if (data_bytes < 53) return;  /* 426 bits = 53.25 bytes */
        decode_voyage(frame_buf, data_bytes,
                      kv, sizeof(kv), payload, sizeof(payload));
        break;
    case 6:
        if (data_bytes < 11) return; /* 88 bits = 11 bytes minimum for DAC/FI */
        emit_msg6_raw_for_asm(frame_buf, frame_len, freq_hz, unix_ms, emit_fn, user_data);
        return;
    case 8:
        if (data_bytes < 7) return;  /* 56 bits = 7 bytes minimum */
        emit_msg8_raw_for_asm(frame_buf, frame_len, freq_hz, unix_ms, emit_fn, user_data);
        return;
    case 12:
        if (data_bytes < 9) return; /* 72 bits = 9 bytes minimum for text */
        emit_msg12_raw_for_asm(frame_buf, frame_len, freq_hz, unix_ms, emit_fn, user_data);
        return;
    case 14:
        if (data_bytes < 5) return; /* 40 bits = 5 bytes minimum for text */
        emit_msg14_raw_for_asm(frame_buf, frame_len, freq_hz, unix_ms, emit_fn, user_data);
        return;
    case 21:
        if (data_bytes < 34) return;  /* 272 bits = 34 bytes */
        decode_aton(frame_buf, data_bytes,
                    kv, sizeof(kv), payload, sizeof(payload));
        break;
    case 19:
        if (data_bytes < 39) return;  /* 312 bits = 39 bytes */
        decode_extended_position(frame_buf, data_bytes,
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

    const char* sig = (msg_type == 4)  ? "AIS_BSR"
                    : (msg_type == 5)  ? "AIS_STAT"
                    : (msg_type == 9)  ? "AIR-SAR"
                    : (msg_type == 21) ? "AIS_ATON"
                    : (msg_type == 24) ? "AIS_STAT24"
                    : "AIS_POS";  /* covers 1,2,3,18,19 */
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
    ALOG("process_bits: %u bits  freq=%.3f MHz  src=%s\n",
         bit_count, freq_hz / 1e6, source_type ? source_type : "?");
    if (ais_dec_dbg()) {
        /* Print first 16 bytes as hex so we can see the raw bit pattern. */
        uint32_t print_bytes = bit_count / 8u;
        if (print_bytes > 16u) print_bytes = 16u;
        fprintf(stderr, "[ais_dec] hex: ");
        for (uint32_t b = 0; b < print_bytes; ++b)
            fprintf(stderr, "%02X ", bit_bytes[b]);
        fprintf(stderr, "\n");

        /* Find longest run of consecutive 1-bits.
           An HDLC flag (01111110) contains exactly 6 consecutive ones.
           If max_ones < 6 the bit stream is definitely wrong (inverted or noise).
           If max_ones == 6 and flags_count == 0 the flag boundary aligns wrong. */
        int max_ones = 0, cur_ones = 0, flags_count = 0;
        for (uint32_t bi = 0; bi < bit_count; ++bi) {
            const int b = (bit_bytes[bi / 8] >> (7 - (bi % 8))) & 1;
            if (b) {
                ++cur_ones;
                if (cur_ones > max_ones) max_ones = cur_ones;
            } else {
                if (cur_ones >= 6) ++flags_count;
                cur_ones = 0;
            }
        }
        fprintf(stderr, "[ais_dec] max-ones-run=%d  flag-candidates(>=6)=%d\n",
                max_ones, flags_count);
    }
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
                ctx->cur_byte |= (uint8_t)(1u << (7 - ctx->bit_pos));
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
                if (ctx->in_frame && ctx->frame_len >= 5) {
                    const int crc_ok = hdlc_check_crc(ctx->frame_buf, ctx->frame_len);
                    ALOG("frame: %u bytes  CRC %s  freq=%.3f MHz\n",
                         ctx->frame_len, crc_ok ? "OK" : "FAIL", freq_hz / 1e6);
                    if (crc_ok) {
                        decode_and_emit(ctx->frame_buf, ctx->frame_len,
                                        freq_hz, unix_ms, emit_fn, user_data);
                    }
                } else if (ctx->in_frame) {
                    ALOG("frame: too short (%u bytes) — discarded\n", ctx->frame_len);
                }
                ALOG("HDLC flag  freq=%.3f MHz\n", freq_hz / 1e6);
                ctx->in_frame  = 1;
                ctx->frame_len = 0;
                ctx->bit_pos   = 0;
                ctx->cur_byte  = 0;
                continue;
            } else {
                ctx->consecutive_ones = 0;
            }
            if (ctx->in_frame) {
                ctx->cur_byte &= ~(uint8_t)(1u << ctx->bit_pos); // Säkerställ nolla vid 0-bit
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
