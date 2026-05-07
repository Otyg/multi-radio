/**
 * hdlc_postproc.c — HDLC frame extractor postprocessing plugin
 *
 * Role: MR_PLUGIN_ROLE_POSTPROCESSING
 *
 * Reads a packed bit stream (MSB-first) from mr_plugin_process_bits,
 * performs bit-stuffing removal, extracts HDLC frames delimited by
 * flag bytes (0x7E = 01111110), validates CRC-16-CCITT and emits:
 *
 *   signal_type = "HDLC_FRAME"         (CRC OK)
 *   signal_type = "HDLC_FRAME_BAD_CRC" (CRC failed)
 *
 * Payload: hex string of frame content excluding the 2-byte FCS.
 * normalized_kv_json: {"byte_count":"N","crc_ok":"1"} or {"...":"0"}.
 *
 * State is maintained across calls because frames can span invocations.
 */

#include "mr_plugin_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* CRC-16-CCITT (poly 0x8408 reflected, init 0xFFFF)                  */
/* Valid residue when all frame bytes including FCS are fed: 0xF0B8   */
/* ------------------------------------------------------------------ */

#define HDLC_CRC_INIT    0xFFFFu
#define HDLC_CRC_RESIDUE 0xF0B8u
#define HDLC_CRC_POLY    0x8408u

static uint16_t crc16_ccitt_byte(uint16_t crc, uint8_t byte) {
    int i;
    crc ^= (uint16_t)byte;
    for (i = 0; i < 8; ++i) {
        if (crc & 1u)
            crc = (crc >> 1) ^ HDLC_CRC_POLY;
        else
            crc >>= 1;
    }
    return crc;
}

/* ------------------------------------------------------------------ */
/* Plugin context                                                       */
/* ------------------------------------------------------------------ */

/* Maximum frame size we will buffer (bytes, before FCS removal).
   Frames larger than this are discarded. */
#define HDLC_MAX_FRAME_BYTES 4096

typedef struct {
    /* Bit de-stuffing state */
    int  consecutive_ones;  /* how many consecutive 1-bits we have seen */

    /* Frame assembly */
    int     in_frame;       /* are we between two flags? */
    uint8_t cur_byte;       /* byte being assembled */
    int     bit_pos;        /* bits accumulated in cur_byte (0-7, MSB first) */

    /* Assembled frame bytes (including FCS at end) */
    uint8_t  frame_buf[HDLC_MAX_FRAME_BYTES];
    uint32_t frame_len;     /* bytes accumulated so far */
} HdlcCtx;

/* ------------------------------------------------------------------ */
/* Plugin API                                                           */
/* ------------------------------------------------------------------ */

static const MrPluginMeta kMeta = {
    "hdlc_postproc",
    "1.0.0",
    MR_PLUGIN_API_VERSION,
    "HDLC frame extractor with CRC-16-CCITT validation",
    MR_PLUGIN_ROLE_POSTPROCESSING
};

MrPluginCtx* mr_plugin_create(void) {
    HdlcCtx* ctx = (HdlcCtx*)calloc(1, sizeof(HdlcCtx));
    return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) { free(raw); }

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
    (void)raw; (void)key; (void)value;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Emit a completed frame                                               */
/* ------------------------------------------------------------------ */

static void emit_frame(HdlcCtx* ctx, double freq_hz, uint64_t unix_ms,
                       MrEmitFn emit_fn, void* user_data) {
    /* Need at least 4 bytes total (2 data + 2 FCS minimum meaningful frame) */
    if (ctx->frame_len < 4) return;

    /* Compute CRC over all frame bytes including the FCS field */
    uint16_t crc = HDLC_CRC_INIT;
    uint32_t i;
    for (i = 0; i < ctx->frame_len; ++i)
        crc = crc16_ccitt_byte(crc, ctx->frame_buf[i]);

    const int crc_ok = (crc == HDLC_CRC_RESIDUE);

    /* Payload = frame content excluding the 2-byte FCS */
    const uint32_t data_len = ctx->frame_len - 2;

    /* Build hex string */
    char* hex = (char*)malloc(data_len * 2 + 1);
    if (!hex) return;
    for (i = 0; i < data_len; ++i)
        snprintf(hex + i * 2, 3, "%02X", (unsigned)ctx->frame_buf[i]);

    /* Build normalized KV JSON */
    char kv[128];
    snprintf(kv, sizeof(kv),
             "{\"byte_count\":\"%u\",\"crc_ok\":\"%d\"}",
             (unsigned)data_len, crc_ok);

    const char* sig_type = crc_ok ? "HDLC_FRAME" : "HDLC_FRAME_BAD_CRC";

    if (emit_fn)
        emit_fn(sig_type, hex, freq_hz, unix_ms, kv, user_data);

    free(hex);
}

/* ------------------------------------------------------------------ */
/* mr_plugin_process_bits                                              */
/* ------------------------------------------------------------------ */

void mr_plugin_process_bits(MrPluginCtx* raw,
                            const uint8_t* bit_bytes, uint32_t bit_count,
                            double freq_hz, uint64_t unix_ms,
                            const char* source_type,
                            MrEmitFn emit_fn, void* user_data) {
    (void)source_type;
    if (!raw || !bit_bytes || bit_count == 0) return;
    HdlcCtx* ctx = (HdlcCtx*)raw;

    uint32_t i;
    for (i = 0; i < bit_count; ++i) {
        /* Extract bit: MSB first within each byte */
        const uint32_t byte_idx = i / 8;
        const uint32_t bit_idx  = 7u - (i % 8u);
        const int bit = (bit_bytes[byte_idx] >> bit_idx) & 1;

        if (bit == 1) {
            ctx->consecutive_ones++;

            /* Six consecutive 1s after a 0: flag sequence 01111110 */
            if (ctx->consecutive_ones == 6) {
                /* The *next* bit should be 0 to complete the flag.
                   We handle this after seeing the 0 — see below.
                   For now just note we have 6 ones. */
            } else if (ctx->consecutive_ones > 6) {
                /* More than 6 consecutive 1s: abort frame (invalid) */
                ctx->in_frame  = 0;
                ctx->frame_len = 0;
                ctx->bit_pos   = 0;
                ctx->cur_byte  = 0;
                /* keep consecutive_ones counting — next 0 may be a flag */
            }

            /* If we are inside a frame, accumulate this 1-bit (unless
               it is the stuffed bit that will be stripped — we strip
               on the following 0). */
            if (ctx->in_frame && ctx->consecutive_ones <= 5) {
                /* Normal data bit = 1 */
                ctx->cur_byte |= (uint8_t)(1u << (7 - ctx->bit_pos));
                ctx->bit_pos++;
                if (ctx->bit_pos == 8) {
                    if (ctx->frame_len < HDLC_MAX_FRAME_BYTES)
                        ctx->frame_buf[ctx->frame_len++] = ctx->cur_byte;
                    ctx->cur_byte = 0;
                    ctx->bit_pos  = 0;
                }
            }

        } else {
            /* bit == 0 */
            if (ctx->consecutive_ones == 5) {
                /* Bit stuffing: 5 ones followed by 0 — discard the 0 */
                ctx->consecutive_ones = 0;
                /* Do NOT add this bit to the frame */
                continue;
            } else if (ctx->consecutive_ones == 6) {
                /* Flag detected: 01111110 */
                ctx->consecutive_ones = 0;

                if (ctx->in_frame && ctx->frame_len >= 4) {
                    /* Close the current frame */
                    emit_frame(ctx, freq_hz, unix_ms, emit_fn, user_data);
                }

                /* Start a new frame */
                ctx->in_frame  = 1;
                ctx->frame_len = 0;
                ctx->bit_pos   = 0;
                ctx->cur_byte  = 0;
                continue;
            } else {
                ctx->consecutive_ones = 0;
            }

            /* Normal data bit = 0 */
            if (ctx->in_frame) {
                /* cur_byte bit already 0 from calloc/reset, just advance */
                ctx->bit_pos++;
                if (ctx->bit_pos == 8) {
                    if (ctx->frame_len < HDLC_MAX_FRAME_BYTES)
                        ctx->frame_buf[ctx->frame_len++] = ctx->cur_byte;
                    ctx->cur_byte = 0;
                    ctx->bit_pos  = 0;
                }
            }
        }
    }
}

/* mr_plugin_process_iq: no-op stub (postprocessing role never called for IQ) */
void mr_plugin_process_iq(MrPluginCtx* ctx,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
    (void)ctx; (void)iq; (void)num_pairs; (void)sr;
    (void)freq_hz; (void)unix_ms; (void)emit_fn; (void)user_data;
}
