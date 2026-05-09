/**
 * vdes_asm_decoder.c — EXPERIMENTAL VDES ASM link decoder
 *
 * Role: MR_PLUGIN_ROLE_DECODER
 *
 * This revision adds a stateful sync search across block boundaries and
 * candidate extraction based on configurable sync patterns.
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

#define VDES_CAND_DEFAULT_BITS  1056u
#define VDES_CAND_MIN_BITS        96u
#define VDES_MAX_STREAM_BITS   16384u
#define VDES_TAIL_KEEP_BITS      512u

typedef struct {
    uint32_t candidate_bits;
    uint32_t sync_errors_max;
    uint32_t diag_interval_blocks;
    uint8_t  sync_bits[64];
    uint8_t  sync_len;

    uint8_t* stream;
    uint32_t stream_cap_bits;
    uint32_t stream_bits;

    uint64_t blocks_seen;
    uint64_t candidates_emitted;
} VdesAsmDecCtx;

static const MrPluginMeta kMeta = {
    "vdes_asm_decoder",
    "0.3.0",
    MR_PLUGIN_API_VERSION,
    "EXPERIMENTAL: VDES ASM decoder (stateful sync candidate extraction)",
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

static int ensure_stream_capacity(VdesAsmDecCtx* ctx, uint32_t need_bits) {
    uint32_t need_bytes;
    uint32_t cap_bytes;
    uint8_t* nb;

    if (need_bits <= ctx->stream_cap_bits) return 1;
    if (need_bits > VDES_MAX_STREAM_BITS) return 0;

    need_bytes = (need_bits + 7u) / 8u;
    cap_bytes = (ctx->stream_cap_bits + 7u) / 8u;
    if (need_bytes < cap_bytes * 2u) need_bytes = cap_bytes * 2u;
    if (need_bytes < 512u) need_bytes = 512u;

    nb = (uint8_t*)realloc(ctx->stream, need_bytes);
    if (!nb) return 0;
    if (need_bytes > cap_bytes) {
        memset(nb + cap_bytes, 0, need_bytes - cap_bytes);
    }
    ctx->stream = nb;
    ctx->stream_cap_bits = need_bytes * 8u;
    return 1;
}

static void set_default_sync(VdesAsmDecCtx* ctx) {
    /* Default: 28-bit ASM-TER style double Barker sync candidate. */
    static const uint8_t kSync28[28] = {
        1,
        1,1,1,1,1,0,0,1,1,0,1,0,1,
        0,
        0,0,0,0,0,1,1,0,0,1,0,1,0
    };
    memcpy(ctx->sync_bits, kSync28, sizeof(kSync28));
    ctx->sync_len = 28u;
}

static int parse_sync_bits(const char* value, uint8_t* out, uint8_t* out_len) {
    uint8_t n = 0u;
    if (!value || !out || !out_len) return 0;
    while (*value) {
        if (*value == '0' || *value == '1') {
            if (n >= 64u) return 0;
            out[n++] = (uint8_t)(*value - '0');
        }
        value++;
    }
    if (n < 8u) return 0;
    *out_len = n;
    return 1;
}

static int sync_errors(const uint8_t* bits, uint32_t bit_count, uint32_t start,
                       const uint8_t* sync, uint8_t sync_len, int inverted) {
    int err = 0;
    for (uint32_t i = 0; i < sync_len; ++i) {
        int b;
        int s;
        if (start + i >= bit_count) return 9999;
        b = get_bit(bits, start + i);
        s = sync[i] ? 1 : 0;
        if (inverted) s = !s;
        if (b != s) err++;
    }
    return err;
}

static void emit_candidate(VdesAsmDecCtx* ctx,
                           const uint8_t* bits, uint32_t bit_count,
                           uint32_t start, int inv, int err,
                           double freq_hz, uint64_t unix_ms,
                           MrEmitFn emit_fn, void* user_data,
                           const char* source_type) {
    uint32_t avail_bits;
    uint32_t use_bits;
    uint32_t out_bytes;
    uint8_t* out;
    char* hex;
    char kv[384];

    if (!emit_fn || start >= bit_count) return;

    avail_bits = bit_count - start;
    use_bits = avail_bits < ctx->candidate_bits ? avail_bits : ctx->candidate_bits;
    if (use_bits < VDES_CAND_MIN_BITS) return;

    out_bytes = (use_bits + 7u) / 8u;
    out = (uint8_t*)calloc(out_bytes, 1u);
    if (!out) return;

    for (uint32_t i = 0; i < use_bits; ++i) {
        set_bit(out, i, get_bit(bits, start + i));
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
             "\"sync_len\":\"%u\","
             "\"sync_inverted\":\"%d\","
             "\"sync_errors\":\"%d\","
             "\"sync_offset_bits\":\"%u\","
             "\"candidate_bits\":\"%u\","
             "\"blocks_seen\":\"%llu\","
             "\"candidates_emitted\":\"%llu\"}",
             source_type ? source_type : "",
             (unsigned)ctx->sync_len,
             inv ? 1 : 0,
             err,
             start,
             use_bits,
             (unsigned long long)ctx->blocks_seen,
             (unsigned long long)ctx->candidates_emitted);

    emit_fn("VDES_ASM_L2", hex, freq_hz, unix_ms, kv, user_data);

    ctx->candidates_emitted++;
    free(hex);
    free(out);
}

static void append_bits(VdesAsmDecCtx* ctx, const uint8_t* in, uint32_t in_bits) {
    uint32_t i;
    if (!ctx || !in || in_bits == 0u) return;
    if (!ensure_stream_capacity(ctx, ctx->stream_bits + in_bits)) return;

    for (i = 0; i < in_bits; ++i) {
        set_bit(ctx->stream, ctx->stream_bits + i, get_bit(in, i));
    }
    ctx->stream_bits += in_bits;
}

static void keep_stream_tail(VdesAsmDecCtx* ctx) {
    uint32_t keep = (ctx->sync_len > VDES_TAIL_KEEP_BITS) ? ctx->sync_len : VDES_TAIL_KEEP_BITS;
    uint32_t i;
    if (ctx->stream_bits <= keep) return;

    for (i = 0; i < keep; ++i) {
        set_bit(ctx->stream, i, get_bit(ctx->stream, ctx->stream_bits - keep + i));
    }
    ctx->stream_bits = keep;
}

MrPluginCtx* mr_plugin_create(void) {
    VdesAsmDecCtx* ctx = (VdesAsmDecCtx*)calloc(1, sizeof(VdesAsmDecCtx));
    if (!ctx) return NULL;

    ctx->candidate_bits = VDES_CAND_DEFAULT_BITS;
    ctx->sync_errors_max = 1u;
    ctx->diag_interval_blocks = 50u;
    set_default_sync(ctx);
    return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
    VdesAsmDecCtx* ctx = (VdesAsmDecCtx*)raw;
    if (!ctx) return;
    free(ctx->stream);
    free(ctx);
}

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
    VdesAsmDecCtx* ctx = (VdesAsmDecCtx*)raw;
    if (!ctx || !key || !value) return 0;

    if (strcmp(key, "candidate_bits") == 0) {
        const int v = atoi(value);
        if (v >= (int)VDES_CAND_MIN_BITS && v <= 4096) ctx->candidate_bits = (uint32_t)v;
        return 1;
    }
    if (strcmp(key, "sync_errors_max") == 0) {
        const int v = atoi(value);
        if (v >= 0 && v <= 8) ctx->sync_errors_max = (uint32_t)v;
        return 1;
    }
    if (strcmp(key, "diag_interval_blocks") == 0) {
        const int v = atoi(value);
        if (v >= 0 && v <= 10000) {
            ctx->diag_interval_blocks = (uint32_t)v;
            return 1;
        }
        return 0;
    }
    if (strcmp(key, "sync_bits") == 0) {
        uint8_t tmp[64];
        uint8_t n = 0u;
        if (parse_sync_bits(value, tmp, &n)) {
            memcpy(ctx->sync_bits, tmp, n);
            ctx->sync_len = n;
            return 1;
        }
        return 0;
    }
    return 0;
}

void mr_plugin_process_bits(MrPluginCtx* raw,
                            const uint8_t* bit_bytes, uint32_t bit_count,
                            double freq_hz, uint64_t unix_ms,
                            const char* source_type,
                            MrEmitFn emit_fn, void* user_data) {
    VdesAsmDecCtx* ctx = (VdesAsmDecCtx*)raw;
    uint32_t i;
    int emitted = 0;

    if (!ctx || !bit_bytes || bit_count == 0u || !emit_fn || ctx->sync_len < 8u) return;
    ctx->blocks_seen++;

    append_bits(ctx, bit_bytes, bit_count);

    if (ctx->stream_bits >= ctx->sync_len) {
        for (i = 0; i + ctx->sync_len <= ctx->stream_bits; ++i) {
            int e0 = sync_errors(ctx->stream, ctx->stream_bits, i, ctx->sync_bits, ctx->sync_len, 0);
            int e1 = sync_errors(ctx->stream, ctx->stream_bits, i, ctx->sync_bits, ctx->sync_len, 1);

            if (e0 <= (int)ctx->sync_errors_max) {
                emit_candidate(ctx, ctx->stream, ctx->stream_bits,
                               i, 0, e0,
                               freq_hz, unix_ms, emit_fn, user_data, source_type);
                emitted = 1;
                i += (ctx->sync_len > 1u) ? (ctx->sync_len - 1u) : 0u;
                continue;
            }
            if (e1 <= (int)ctx->sync_errors_max) {
                emit_candidate(ctx, ctx->stream, ctx->stream_bits,
                               i, 1, e1,
                               freq_hz, unix_ms, emit_fn, user_data, source_type);
                emitted = 1;
                i += (ctx->sync_len > 1u) ? (ctx->sync_len - 1u) : 0u;
                continue;
            }
        }
    }

    if (!emitted &&
        ctx->diag_interval_blocks > 0u &&
        (ctx->blocks_seen % ctx->diag_interval_blocks) == 0u) {
        char payload[192];
        char kv[256];
        snprintf(payload, sizeof(payload),
                 "No sync (stream=%u bits, block=%u bits, sync_len=%u, max_err=%u)",
                 ctx->stream_bits, bit_count,
                 (unsigned)ctx->sync_len, (unsigned)ctx->sync_errors_max);
        snprintf(kv, sizeof(kv),
                 "{\"signal_type\":\"VDES_ASM_DIAG\","
                 "\"source_type\":\"%s\","
                 "\"decoder_scope\":\"VDES_ASM_SYNC_CANDIDATE\","
                 "\"sync_found\":\"0\","
                 "\"sync_len\":\"%u\","
                 "\"sync_errors_max\":\"%u\","
                 "\"stream_bits\":\"%u\","
                 "\"bit_count\":\"%u\","
                 "\"blocks_seen\":\"%llu\"}",
                 source_type ? source_type : "",
                 (unsigned)ctx->sync_len,
                 (unsigned)ctx->sync_errors_max,
                 ctx->stream_bits,
                 bit_count,
                 (unsigned long long)ctx->blocks_seen);
        emit_fn("VDES_ASM_DIAG", payload, freq_hz, unix_ms, kv, user_data);
    }

    keep_stream_tail(ctx);
}

void mr_plugin_process_iq(MrPluginCtx* ctx,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
    (void)ctx; (void)iq; (void)num_pairs; (void)sr;
    (void)freq_hz; (void)unix_ms; (void)emit_fn; (void)user_data;
}
