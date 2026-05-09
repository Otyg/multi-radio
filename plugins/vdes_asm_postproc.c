/**
 * vdes_asm_postproc.c — EXPERIMENTAL VDES ASM postprocessor
 *
 * Role: MR_PLUGIN_ROLE_POSTPROCESSING
 *
 * Current behavior:
 *   - consumes sync-aligned candidate bursts from vdes_asm_decoder
 *   - exposes lightweight header diagnostics for iterative tuning
 */

#include "mr_plugin_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint64_t blocks_seen;
    uint64_t bits_seen;
    uint64_t candidates_seen;
} VdesAsmPostCtx;

static const MrPluginMeta kMeta = {
    "vdes_asm_postproc",
    "0.2.0",
    MR_PLUGIN_API_VERSION,
    "EXPERIMENTAL: VDES ASM postprocessor (candidate diagnostics)",
    MR_PLUGIN_ROLE_POSTPROCESSING
};

static uint32_t bits_be(const uint8_t* data, uint32_t bit_count,
                        uint32_t start, uint32_t len) {
    uint32_t r = 0u;
    for (uint32_t i = 0; i < len; ++i) {
        const uint32_t bpos = start + i;
        if (bpos >= bit_count) break;
        const uint8_t b = (data[bpos / 8u] >> (7u - (bpos % 8u))) & 1u;
        r = (r << 1u) | (uint32_t)b;
    }
    return r;
}

MrPluginCtx* mr_plugin_create(void) {
    return (MrPluginCtx*)calloc(1, sizeof(VdesAsmPostCtx));
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
    VdesAsmPostCtx* ctx = (VdesAsmPostCtx*)raw;
    char payload[160];
    char kv[384];

    if (!ctx || !emit_fn) return;

    ctx->blocks_seen++;
    ctx->bits_seen += bit_count;

    if (bit_bytes && bit_count >= 64u) {
        const uint32_t hdr0 = bits_be(bit_bytes, bit_count, 13u, 16u);
        const uint32_t hdr1 = bits_be(bit_bytes, bit_count, 29u, 16u);
        const uint32_t proto = bits_be(bit_bytes, bit_count, 45u, 8u);
        ctx->candidates_seen++;

        snprintf(payload, sizeof(payload),
                 "VDES ASM cand hdr0=0x%04X hdr1=0x%04X proto=0x%02X",
                 hdr0 & 0xFFFFu, hdr1 & 0xFFFFu, proto & 0xFFu);
        snprintf(kv, sizeof(kv),
                 "{\"signal_type\":\"VDES_ASM_DIAG\","
                 "\"source_type\":\"%s\","
                 "\"bit_count\":\"%u\","
                 "\"hdr0\":\"%u\","
                 "\"hdr1\":\"%u\","
                 "\"proto\":\"%u\","
                 "\"blocks_seen\":\"%llu\","
                 "\"candidates_seen\":\"%llu\"}",
                 source_type ? source_type : "", bit_count,
                 hdr0, hdr1, proto,
                 (unsigned long long)ctx->blocks_seen,
                 (unsigned long long)ctx->candidates_seen);
        emit_fn("VDES_ASM_DIAG", payload, freq_hz, unix_ms, kv, user_data);
        return;
    }

    snprintf(payload, sizeof(payload),
             "VDES ASM stream blocks=%llu bits=%llu",
             (unsigned long long)ctx->blocks_seen,
             (unsigned long long)ctx->bits_seen);
    snprintf(kv, sizeof(kv),
             "{\"signal_type\":\"VDES_ASM_DIAG\","
             "\"source_type\":\"%s\","
             "\"bit_count\":\"%u\","
             "\"blocks_seen\":\"%llu\","
             "\"bits_seen\":\"%llu\"}",
             source_type ? source_type : "", bit_count,
             (unsigned long long)ctx->blocks_seen,
             (unsigned long long)ctx->bits_seen);
    emit_fn("VDES_ASM_DIAG", payload, freq_hz, unix_ms, kv, user_data);
}

void mr_plugin_process_iq(MrPluginCtx* ctx,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
    (void)ctx; (void)iq; (void)num_pairs; (void)sr;
    (void)freq_hz; (void)unix_ms; (void)emit_fn; (void)user_data;
}
