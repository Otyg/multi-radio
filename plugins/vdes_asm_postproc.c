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
    uint64_t candidates_emitted;
    uint64_t candidates_suppressed;
    uint32_t diag_interval_blocks;
    int emit_unfiltered_candidates;
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

static uint32_t popcount32(uint32_t v) {
    uint32_t c = 0u;
    while (v) {
        c += (v & 1u);
        v >>= 1u;
    }
    return c;
}

MrPluginCtx* mr_plugin_create(void) {
    VdesAsmPostCtx* ctx = (VdesAsmPostCtx*)calloc(1, sizeof(VdesAsmPostCtx));
    if (!ctx) return NULL;
    ctx->diag_interval_blocks = 50u;
    ctx->emit_unfiltered_candidates = 0;
    return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) { free(raw); }

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
    VdesAsmPostCtx* ctx = (VdesAsmPostCtx*)raw;
    if (!ctx || !key || !value) return 0;
    if (strcmp(key, "diag_interval_blocks") == 0) {
        const int v = atoi(value);
        if (v >= 0 && v <= 10000) {
            ctx->diag_interval_blocks = (uint32_t)v;
            return 1;
        }
        return 0;
    }
    if (strcmp(key, "emit_unfiltered_candidates") == 0) {
        ctx->emit_unfiltered_candidates = (atoi(value) != 0) ? 1 : 0;
        return 1;
    }
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
        const uint32_t pop = popcount32(hdr0) + popcount32(hdr1) + popcount32(proto);
        const int plausible_proto = (proto != 0u && proto != 0xFFu);
        const int plausible_hdr = ((hdr0 | hdr1) != 0u);
        const int interesting = (plausible_proto && plausible_hdr && pop >= 3u);
        ctx->candidates_seen++;
        if (ctx->emit_unfiltered_candidates || interesting) {
            ctx->candidates_emitted++;
            snprintf(payload, sizeof(payload),
                     "VDES ASM cand hdr0=0x%04X hdr1=0x%04X proto=0x%02X pop=%u",
                     hdr0 & 0xFFFFu, hdr1 & 0xFFFFu, proto & 0xFFu, pop);
            snprintf(kv, sizeof(kv),
                     "{\"signal_type\":\"VDES_ASM_DIAG\","
                     "\"source_type\":\"%s\","
                     "\"diag_kind\":\"candidate\","
                     "\"candidate_interesting\":\"%d\","
                     "\"bit_count\":\"%u\","
                     "\"hdr0\":\"%u\","
                     "\"hdr1\":\"%u\","
                     "\"proto\":\"%u\","
                     "\"popcount\":\"%u\","
                     "\"blocks_seen\":\"%llu\","
                     "\"candidates_seen\":\"%llu\","
                     "\"candidates_emitted\":\"%llu\","
                     "\"candidates_suppressed\":\"%llu\"}",
                     source_type ? source_type : "", interesting ? 1 : 0, bit_count,
                     hdr0, hdr1, proto, pop,
                     (unsigned long long)ctx->blocks_seen,
                     (unsigned long long)ctx->candidates_seen,
                     (unsigned long long)ctx->candidates_emitted,
                     (unsigned long long)ctx->candidates_suppressed);
            emit_fn("VDES_ASM_DIAG", payload, freq_hz, unix_ms, kv, user_data);
        } else {
            ctx->candidates_suppressed++;
            if (ctx->diag_interval_blocks > 0u &&
                (ctx->blocks_seen % ctx->diag_interval_blocks) == 0u) {
                snprintf(payload, sizeof(payload),
                         "VDES ASM suppressed noisy candidates=%llu (last hdr0=0x%04X hdr1=0x%04X proto=0x%02X pop=%u)",
                         (unsigned long long)ctx->candidates_suppressed,
                         hdr0 & 0xFFFFu, hdr1 & 0xFFFFu, proto & 0xFFu, pop);
                snprintf(kv, sizeof(kv),
                         "{\"signal_type\":\"VDES_ASM_DIAG\","
                         "\"source_type\":\"%s\","
                         "\"diag_kind\":\"summary\","
                         "\"bit_count\":\"%u\","
                         "\"blocks_seen\":\"%llu\","
                         "\"candidates_seen\":\"%llu\","
                         "\"candidates_emitted\":\"%llu\","
                         "\"candidates_suppressed\":\"%llu\"}",
                         source_type ? source_type : "", bit_count,
                         (unsigned long long)ctx->blocks_seen,
                         (unsigned long long)ctx->candidates_seen,
                         (unsigned long long)ctx->candidates_emitted,
                         (unsigned long long)ctx->candidates_suppressed);
                emit_fn("VDES_ASM_DIAG", payload, freq_hz, unix_ms, kv, user_data);
            }
        }
        return;
    }

    if (ctx->diag_interval_blocks > 0u &&
        (ctx->blocks_seen % ctx->diag_interval_blocks) == 0u) {
        snprintf(payload, sizeof(payload),
                 "VDES ASM stream blocks=%llu bits=%llu",
                 (unsigned long long)ctx->blocks_seen,
                 (unsigned long long)ctx->bits_seen);
        snprintf(kv, sizeof(kv),
                 "{\"signal_type\":\"VDES_ASM_DIAG\","
                 "\"source_type\":\"%s\","
                 "\"diag_kind\":\"stream\","
                 "\"bit_count\":\"%u\","
                 "\"blocks_seen\":\"%llu\","
                 "\"bits_seen\":\"%llu\","
                 "\"candidates_seen\":\"%llu\","
                 "\"candidates_emitted\":\"%llu\","
                 "\"candidates_suppressed\":\"%llu\"}",
                 source_type ? source_type : "", bit_count,
                 (unsigned long long)ctx->blocks_seen,
                 (unsigned long long)ctx->bits_seen,
                 (unsigned long long)ctx->candidates_seen,
                 (unsigned long long)ctx->candidates_emitted,
                 (unsigned long long)ctx->candidates_suppressed);
        emit_fn("VDES_ASM_DIAG", payload, freq_hz, unix_ms, kv, user_data);
    }
}

void mr_plugin_process_iq(MrPluginCtx* ctx,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
    (void)ctx; (void)iq; (void)num_pairs; (void)sr;
    (void)freq_hz; (void)unix_ms; (void)emit_fn; (void)user_data;
}
