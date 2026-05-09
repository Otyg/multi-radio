/**
 * vdes_asm_decoder.c — EXPERIMENTAL VDES ASM link decoder
 *
 * Role: MR_PLUGIN_ROLE_DECODER
 *
 * First implementation step:
 *   - scans demodulated bitstream for candidate 13-bit sync (Barker-like)
 *   - emits sync-aligned candidate bursts for downstream analysis
 *
 * TODO:
 *   - standardized burst framing
 *   - deinterleaving / descrambling
 *   - FEC decode
 */

#include "mr_plugin_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VDES_CAND_DEFAULT_BITS 1024u
#define VDES_CAND_MIN_BITS       96u

typedef struct {
    uint32_t candidate_bits;
    uint64_t blocks_seen;
    uint64_t candidates_emitted;
} VdesAsmDecCtx;

static const MrPluginMeta kMeta = {
    "vdes_asm_decoder",
    "0.2.0",
    MR_PLUGIN_API_VERSION,
    "EXPERIMENTAL: VDES ASM decoder (sync candidate extraction)",
    MR_PLUGIN_ROLE_DECODER
};

static int get_bit(const uint8_t* bits, uint32_t idx) {
    return (bits[idx / 8u] >> (7u - (idx % 8u))) & 1u;
}

static void set_bit(uint8_t* bits, uint32_t idx, int v) {
    const uint8_t mask = (uint8_t)(1u << (7u - (idx % 8u)));
    if (v) bits[idx / 8u] |= mask;
    else   bits[idx / 8u] &= (uint8_t)~mask;
}

static int match_sync13(const uint8_t* bits, uint32_t bit_count, uint32_t start, int inverted) {
    static const uint8_t kSync[13] = {1,1,1,1,1,0,0,1,1,0,1,0,1};
    uint32_t i;
    if (start + 13u > bit_count) return 0;
    for (i = 0; i < 13u; ++i) {
        int b = get_bit(bits, start + i);
        int s = kSync[i];
        if (inverted) s = !s;
        if (b != s) return 0;
    }
    return 1;
}

static void emit_candidate(VdesAsmDecCtx* ctx,
                           const uint8_t* bit_bytes, uint32_t bit_count,
                           uint32_t start, int inv,
                           double freq_hz, uint64_t unix_ms,
                           MrEmitFn emit_fn, void* user_data,
                           const char* source_type) {
    uint32_t avail_bits;
    uint32_t use_bits;
    uint32_t out_bytes;
    uint8_t* out;
    char* hex;
    char kv[320];

    if (!emit_fn || start >= bit_count) return;

    avail_bits = bit_count - start;
    use_bits = avail_bits < ctx->candidate_bits ? avail_bits : ctx->candidate_bits;
    if (use_bits < VDES_CAND_MIN_BITS) return;

    out_bytes = (use_bits + 7u) / 8u;
    out = (uint8_t*)calloc(out_bytes, 1u);
    if (!out) return;

    for (uint32_t i = 0; i < use_bits; ++i) {
        set_bit(out, i, get_bit(bit_bytes, start + i));
    }

    hex = (char*)malloc((size_t)out_bytes * 2u + 1u);
    if (!hex) {
        free(out);
        return;
    }
    for (uint32_t i = 0; i < out_bytes; ++i) snprintf(hex + i * 2u, 3, "%02X", (unsigned)out[i]);

    snprintf(kv, sizeof(kv),
             "{\"signal_type\":\"VDES_ASM_L2\","
             "\"source_type\":\"%s\","
             "\"decoder_scope\":\"VDES_ASM_SYNC_CANDIDATE\","
             "\"sync_len\":\"13\","
             "\"sync_inverted\":\"%d\","
             "\"sync_offset_bits\":\"%u\","
             "\"candidate_bits\":\"%u\","
             "\"blocks_seen\":\"%llu\","
             "\"candidates_emitted\":\"%llu\"}",
             source_type ? source_type : "", inv ? 1 : 0, start, use_bits,
             (unsigned long long)ctx->blocks_seen,
             (unsigned long long)ctx->candidates_emitted);

    emit_fn("VDES_ASM_L2", hex, freq_hz, unix_ms, kv, user_data);

    ctx->candidates_emitted++;
    free(hex);
    free(out);
}

MrPluginCtx* mr_plugin_create(void) {
    VdesAsmDecCtx* ctx = (VdesAsmDecCtx*)calloc(1, sizeof(VdesAsmDecCtx));
    if (!ctx) return NULL;
    ctx->candidate_bits = VDES_CAND_DEFAULT_BITS;
    return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) { free(raw); }

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
    VdesAsmDecCtx* ctx = (VdesAsmDecCtx*)raw;
    if (!ctx || !key || !value) return 0;
    if (strcmp(key, "candidate_bits") == 0) {
        const int v = atoi(value);
        if (v >= (int)VDES_CAND_MIN_BITS && v <= 4096) ctx->candidate_bits = (uint32_t)v;
        return 1;
    }
    return 0;
}

void mr_plugin_process_bits(MrPluginCtx* raw,
                            const uint8_t* bit_bytes, uint32_t bit_count,
                            double freq_hz, uint64_t unix_ms,
                            const char* source_type,
                            MrEmitFn emit_fn, void* user_data) {
    VdesAsmDecCtx* ctx = (VdesAsmDecCtx*)raw;
    int emitted = 0;

    if (!ctx || !bit_bytes || bit_count < 13u || !emit_fn) return;
    ctx->blocks_seen++;

    for (uint32_t i = 0; i + 13u <= bit_count; ++i) {
        if (match_sync13(bit_bytes, bit_count, i, 0)) {
            emit_candidate(ctx, bit_bytes, bit_count, i, 0,
                           freq_hz, unix_ms, emit_fn, user_data, source_type);
            emitted = 1;
            i += 12u;
            continue;
        }
        if (match_sync13(bit_bytes, bit_count, i, 1)) {
            emit_candidate(ctx, bit_bytes, bit_count, i, 1,
                           freq_hz, unix_ms, emit_fn, user_data, source_type);
            emitted = 1;
            i += 12u;
            continue;
        }
    }

    if (!emitted && (ctx->blocks_seen % 50u) == 0u) {
        char kv[224];
        snprintf(kv, sizeof(kv),
                 "{\"signal_type\":\"VDES_ASM_L2\","
                 "\"source_type\":\"%s\","
                 "\"decoder_scope\":\"VDES_ASM_SYNC_CANDIDATE\","
                 "\"sync_found\":\"0\","
                 "\"bit_count\":\"%u\","
                 "\"blocks_seen\":\"%llu\"}",
                 source_type ? source_type : "", bit_count,
                 (unsigned long long)ctx->blocks_seen);
        emit_fn("VDES_ASM_L2", "", freq_hz, unix_ms, kv, user_data);
    }
}

void mr_plugin_process_iq(MrPluginCtx* ctx,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
    (void)ctx; (void)iq; (void)num_pairs; (void)sr;
    (void)freq_hz; (void)unix_ms; (void)emit_fn; (void)user_data;
}
