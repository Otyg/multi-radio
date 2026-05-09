/**
 * asm_decoder.c — ASM (Application Specific Message) postprocessor
 *
 * Role: MR_PLUGIN_ROLE_POSTPROCESSING
 *
 * Expects NRZI-decoded HDLC bitstream (source_type typically NRZI_ASM_DATA),
 * extracts HDLC frames, validates CRC-16-CCITT, then decodes AIS message type 8
 * (Binary Broadcast Message / ASM envelope: DAC/FI + application data).
 */

#include "mr_plugin_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int asm_dbg(void) {
    static int v = -1;
    if (v < 0) {
        const char* e = getenv("MR_AIS_DEBUG");
        v = (e && e[0] != '0') ? 1 : 0;
    }
    return v;
}

#define ASM_LOG(...) do { if (asm_dbg()) fprintf(stderr, "[asm_dec] " __VA_ARGS__); } while (0)

#define HDLC_CRC_INIT    0xFFFFu
#define HDLC_CRC_RESIDUE 0xF0B8u
#define HDLC_CRC_POLY    0x8408u
#define HDLC_MAX_FRAME   512u

typedef struct {
    int      consecutive_ones;
    int      in_frame;
    uint8_t  cur_byte;
    int      bit_pos;
    uint8_t  frame_buf[HDLC_MAX_FRAME];
    uint32_t frame_len;
} AsmCtx;

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
    return crc == HDLC_CRC_RESIDUE;
}

static uint64_t ais_bits(const uint8_t* data, uint32_t data_bytes, int start, int len) {
    uint64_t r = 0;
    int i;
    for (i = 0; i < len; ++i) {
        const int abs = start + i;
        const int bidx = abs / 8;
        const int boff = 7 - (abs % 8);
        if (bidx < (int)data_bytes && (data[bidx] & (1u << boff))) {
            r |= (1ULL << (len - 1 - i));
        }
    }
    return r;
}

static void emit_asm(const uint8_t* frame_buf, uint32_t frame_len,
                     double freq_hz, uint64_t unix_ms,
                     MrEmitFn emit_fn, void* user_data) {
    if (!emit_fn || frame_len < 5) return;

    const uint32_t data_bytes = frame_len - 2;
    const int msg_type = (int)ais_bits(frame_buf, data_bytes, 0, 6);
    const uint32_t mmsi = (data_bytes >= 5) ? (uint32_t)ais_bits(frame_buf, data_bytes, 8, 30) : 0u;

    if (msg_type == 8 && data_bytes >= 7) {
        const int dac = (int)ais_bits(frame_buf, data_bytes, 40, 10);
        const int fi  = (int)ais_bits(frame_buf, data_bytes, 50, 6);
        const uint32_t app_off = 7u;
        const uint32_t app_bytes = (data_bytes > app_off) ? (data_bytes - app_off) : 0u;

        char app_hex[512];
        app_hex[0] = '\0';
        if (app_bytes > 0) {
            uint32_t i;
            const uint32_t lim = (app_bytes > 200u) ? 200u : app_bytes;
            for (i = 0; i < lim; ++i) {
                snprintf(app_hex + i * 2u, 3, "%02X", (unsigned)frame_buf[app_off + i]);
            }
        }

        char kv[640];
        snprintf(kv, sizeof(kv),
                 "{\"signal_type\":\"ASM_MSG\","
                 "\"msg_type\":\"8\","
                 "\"mmsi\":\"%u\","
                 "\"dac\":\"%d\","
                 "\"fi\":\"%d\","
                 "\"app_data_bytes\":\"%u\","
                 "\"app_data\":\"%s\"}",
                 mmsi, dac, fi, app_bytes, app_hex);

        char payload[320];
        snprintf(payload, sizeof(payload),
                 "MMSI:%u DAC:%d FI:%d Data:%s",
                 mmsi, dac, fi, app_hex[0] ? app_hex : "(none)");
        emit_fn("ASM_MSG", payload, freq_hz, unix_ms, kv, user_data);
        return;
    }

    {
        char kv[256];
        char payload[128];
        snprintf(kv, sizeof(kv),
                 "{\"signal_type\":\"ASM_OTHER\","
                 "\"msg_type\":\"%d\","
                 "\"mmsi\":\"%u\","
                 "\"byte_count\":\"%u\"}",
                 msg_type, mmsi, data_bytes);
        snprintf(payload, sizeof(payload), "MMSI:%u Type:%d", mmsi, msg_type);
        emit_fn("ASM_OTHER", payload, freq_hz, unix_ms, kv, user_data);
    }
}

static const MrPluginMeta kMeta = {
    "asm_decoder",
    "1.0.0",
    MR_PLUGIN_API_VERSION,
    "ASM decoder: HDLC framing + AIS type-8 (DAC/FI) extraction",
    MR_PLUGIN_ROLE_POSTPROCESSING
};

MrPluginCtx* mr_plugin_create(void) {
    return (MrPluginCtx*)calloc(1, sizeof(AsmCtx));
}

void mr_plugin_destroy(MrPluginCtx* raw) {
    free(raw);
}

const MrPluginMeta* mr_plugin_get_meta(void) {
    return &kMeta;
}

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
    (void)raw;
    (void)key;
    (void)value;
    return 0;
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

    ASM_LOG("process_bits: bits=%u src=%s freq=%.3f MHz\n",
            bit_count, source_type ? source_type : "?", freq_hz / 1e6);

    for (i = 0; i < bit_count; ++i) {
        const int bit = (bit_bytes[i / 8] >> (7 - (i % 8))) & 1;

        if (bit) {
            ++ctx->consecutive_ones;
            if (ctx->consecutive_ones > 6) {
                ctx->in_frame = 0;
                ctx->frame_len = 0;
                ctx->bit_pos = 0;
                ctx->cur_byte = 0;
            }
            if (ctx->in_frame && ctx->consecutive_ones <= 5) {
                ctx->cur_byte |= (uint8_t)(1u << ctx->bit_pos);
                if (++ctx->bit_pos == 8) {
                    if (ctx->frame_len < HDLC_MAX_FRAME) {
                        ctx->frame_buf[ctx->frame_len++] = ctx->cur_byte;
                    }
                    ctx->cur_byte = 0;
                    ctx->bit_pos = 0;
                }
            }
        } else {
            if (ctx->consecutive_ones == 5) {
                ctx->consecutive_ones = 0;
                continue;
            } else if (ctx->consecutive_ones == 6) {
                ctx->consecutive_ones = 0;
                if (ctx->in_frame && ctx->frame_len >= 5) {
                    const int crc_ok = hdlc_check_crc(ctx->frame_buf, ctx->frame_len);
                    if (crc_ok) {
                        emit_asm(ctx->frame_buf, ctx->frame_len,
                                 freq_hz, unix_ms, emit_fn, user_data);
                    }
                }
                ctx->in_frame = 1;
                ctx->frame_len = 0;
                ctx->bit_pos = 0;
                ctx->cur_byte = 0;
                continue;
            } else {
                ctx->consecutive_ones = 0;
            }

            if (ctx->in_frame) {
                if (++ctx->bit_pos == 8) {
                    if (ctx->frame_len < HDLC_MAX_FRAME) {
                        ctx->frame_buf[ctx->frame_len++] = ctx->cur_byte;
                    }
                    ctx->cur_byte = 0;
                    ctx->bit_pos = 0;
                }
            }
        }
    }
}

void mr_plugin_process_iq(MrPluginCtx* ctx,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
    (void)ctx;
    (void)iq;
    (void)num_pairs;
    (void)sr;
    (void)freq_hz;
    (void)unix_ms;
    (void)emit_fn;
    (void)user_data;
}
