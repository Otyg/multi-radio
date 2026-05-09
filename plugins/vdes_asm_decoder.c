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

#define VDES_MAX_SYNC_BITS       64u
#define VDES_CAND_DEFAULT_BITS  1056u
#define VDES_CAND_MIN_BITS        96u
#define VDES_MAX_STREAM_BITS   16384u
#define VDES_TAIL_KEEP_BITS      512u
#define VDES_AUTOTUNE_INTERVAL_DEFAULT 32u
#define VDES_AUTOTUNE_STRIDE_DEFAULT    2u
#define VDES_AUTOTUNE_SCAN_BITS      4096u
#define VDES_AUTOTUNE_ERR_CAP           4u
#define VDES_AUTOTUNE_MIN_SCORE         8u

typedef struct {
    uint32_t candidate_bits;
    uint32_t sync_errors_max;
    uint32_t diag_interval_blocks;
    uint32_t sync_autotune_enabled;
    uint32_t sync_autotune_interval_blocks;
    uint32_t sync_autotune_stride_bits;
    uint32_t sync_autotune_pattern_lock;
    uint32_t sync_autotune_error_lock;
    uint8_t  sync_bits[VDES_MAX_SYNC_BITS];
    uint8_t  sync_len;

    uint8_t* stream;
    uint32_t stream_cap_bits;
    uint32_t stream_bits;

    uint64_t blocks_seen;
    uint64_t candidates_emitted;
    uint64_t autotune_updates;
} VdesAsmDecCtx;

typedef struct {
    const char* name;
    const uint8_t* bits;
    uint8_t len;
} SyncPatternDef;

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

/* 28-bit ASM style sync candidate used as default/reference. */
static const uint8_t kSync28[28] = {
    1,
    1,1,1,1,1,0,0,1,1,0,1,0,1,
    0,
    0,0,0,0,0,1,1,0,0,1,0,1,0
};

/* 13-bit Barker (binary form) and derivatives are used by autotune candidates. */
static const uint8_t kBarker13[13] = {
    1,1,1,1,1,0,0,1,1,0,1,0,1
};

static void reverse_bits_seq(uint8_t* dst, const uint8_t* src, uint8_t len) {
    uint8_t i;
    for (i = 0; i < len; ++i) dst[i] = src[len - 1u - i];
}

static void set_sync_pattern(VdesAsmDecCtx* ctx, const uint8_t* bits, uint8_t len) {
    if (!ctx || !bits || len < 8u || len > VDES_MAX_SYNC_BITS) return;
    memcpy(ctx->sync_bits, bits, len);
    ctx->sync_len = len;
}

static void set_default_sync(VdesAsmDecCtx* ctx) {
    set_sync_pattern(ctx, kSync28, (uint8_t)sizeof(kSync28));
}

static uint32_t score_hits(const uint32_t* hits, uint32_t cap) {
    uint32_t s = 0u;
    if (!hits) return 0u;
    for (uint32_t e = 0; e <= cap; ++e) {
        uint32_t w = 1u;
        if (e == 0u) w = 16u;
        else if (e == 1u) w = 8u;
        else if (e == 2u) w = 4u;
        else if (e == 3u) w = 2u;
        s += hits[e] * w;
    }
    return s;
}

static uint32_t choose_error_limit(const uint32_t* hits, uint32_t cap, uint32_t offsets) {
    uint32_t cum = 0u;
    uint32_t min_hits;

    if (!hits || cap == 0u) return 0u;
    min_hits = offsets / 320u;
    if (min_hits < 1u) min_hits = 1u;

    for (uint32_t e = 0; e <= cap; ++e) {
        cum += hits[e];
        if (cum >= min_hits) return e;
    }
    return cap;
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

static void maybe_autotune_sync(VdesAsmDecCtx* ctx,
                                double freq_hz, uint64_t unix_ms,
                                const char* source_type,
                                MrEmitFn emit_fn, void* user_data) {
    uint8_t rev_sync28[sizeof(kSync28)];
    uint8_t rev_b13[sizeof(kBarker13)];
    uint8_t b13x2[26];
    const SyncPatternDef patterns[] = {
        {"sync28",     kSync28,      (uint8_t)sizeof(kSync28)},
        {"sync28_rev", rev_sync28,   (uint8_t)sizeof(rev_sync28)},
        {"b13",        kBarker13,    (uint8_t)sizeof(kBarker13)},
        {"b13_rev",    rev_b13,      (uint8_t)sizeof(rev_b13)},
        {"b13_x2",     b13x2,        (uint8_t)sizeof(b13x2)}
    };
    enum { kNumPatterns = (int)(sizeof(patterns) / sizeof(patterns[0])) };
    uint32_t hits[kNumPatterns][VDES_AUTOTUNE_ERR_CAP + 1u];
    uint32_t offsets[kNumPatterns];
    uint32_t scores[kNumPatterns];
    uint32_t desired_err[kNumPatterns];
    uint32_t start_bit;
    uint32_t scan_bits;
    uint32_t best_score = 0u;
    int best_idx = -1;
    int current_idx = -1;

    if (!ctx || !ctx->sync_autotune_enabled || ctx->stream_bits < 128u) return;
    if (ctx->sync_autotune_interval_blocks == 0u) return;
    if ((ctx->blocks_seen % ctx->sync_autotune_interval_blocks) != 0u) return;

    reverse_bits_seq(rev_sync28, kSync28, (uint8_t)sizeof(kSync28));
    reverse_bits_seq(rev_b13, kBarker13, (uint8_t)sizeof(kBarker13));
    memcpy(b13x2, kBarker13, sizeof(kBarker13));
    memcpy(b13x2 + sizeof(kBarker13), kBarker13, sizeof(kBarker13));

    memset(hits, 0, sizeof(hits));
    memset(offsets, 0, sizeof(offsets));
    memset(scores, 0, sizeof(scores));
    memset(desired_err, 0, sizeof(desired_err));

    scan_bits = ctx->stream_bits;
    if (scan_bits > VDES_AUTOTUNE_SCAN_BITS) scan_bits = VDES_AUTOTUNE_SCAN_BITS;
    start_bit = ctx->stream_bits - scan_bits;

    for (int p = 0; p < kNumPatterns; ++p) {
        const uint8_t len = patterns[p].len;
        if (len < 8u || scan_bits <= len) continue;

        for (uint32_t rel = 0u; rel + len <= scan_bits; rel += ctx->sync_autotune_stride_bits) {
            const uint32_t pos = start_bit + rel;
            const int e0 = sync_errors(ctx->stream, ctx->stream_bits, pos,
                                       patterns[p].bits, len, 0);
            const int e1 = sync_errors(ctx->stream, ctx->stream_bits, pos,
                                       patterns[p].bits, len, 1);
            const int em = (e0 < e1) ? e0 : e1;
            if (em >= 0 && em <= (int)VDES_AUTOTUNE_ERR_CAP) {
                hits[p][(uint32_t)em]++;
            }
            offsets[p]++;
        }

        scores[p] = score_hits(hits[p], VDES_AUTOTUNE_ERR_CAP);
        desired_err[p] = choose_error_limit(hits[p], VDES_AUTOTUNE_ERR_CAP, offsets[p]);

        if (len == ctx->sync_len && memcmp(patterns[p].bits, ctx->sync_bits, len) == 0) {
            current_idx = p;
        }
        if (scores[p] > best_score) {
            best_score = scores[p];
            best_idx = p;
        }
    }

    if (best_idx < 0 || best_score < VDES_AUTOTUNE_MIN_SCORE) return;

    if (!ctx->sync_autotune_pattern_lock) {
        uint32_t cur_score = 0u;
        if (current_idx >= 0) cur_score = scores[current_idx];
        if (current_idx < 0 || best_idx != current_idx) {
            if (cur_score == 0u || best_score * 100u >= cur_score * 120u) {
                set_sync_pattern(ctx, patterns[best_idx].bits, patterns[best_idx].len);
                ctx->autotune_updates++;
                if (emit_fn) {
                    char payload[192];
                    char kv[320];
                    snprintf(payload, sizeof(payload),
                             "Autotune sync_bits -> %s (%u bits, score=%u)",
                             patterns[best_idx].name, (unsigned)patterns[best_idx].len, best_score);
                    snprintf(kv, sizeof(kv),
                             "{\"signal_type\":\"VDES_ASM_DIAG\","
                             "\"source_type\":\"%s\","
                             "\"decoder_scope\":\"VDES_ASM_AUTOTUNE\","
                             "\"update\":\"sync_bits\","
                             "\"sync_name\":\"%s\","
                             "\"sync_len\":\"%u\","
                             "\"score\":\"%u\","
                             "\"autotune_updates\":\"%llu\"}",
                             source_type ? source_type : "",
                             patterns[best_idx].name,
                             (unsigned)patterns[best_idx].len,
                             best_score,
                             (unsigned long long)ctx->autotune_updates);
                    emit_fn("VDES_ASM_DIAG", payload, freq_hz, unix_ms, kv, user_data);
                }
            }
        }
    }

    if (!ctx->sync_autotune_error_lock) {
        uint32_t target_err = desired_err[best_idx];
        if (target_err > 8u) target_err = 8u;
        if (target_err > ctx->sync_errors_max + 1u) {
            ctx->sync_errors_max++;
        } else if (ctx->sync_errors_max > target_err + 1u) {
            ctx->sync_errors_max--;
        } else {
            ctx->sync_errors_max = target_err;
        }
    }
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
    ctx->sync_autotune_enabled = 1u;
    ctx->sync_autotune_interval_blocks = VDES_AUTOTUNE_INTERVAL_DEFAULT;
    ctx->sync_autotune_stride_bits = VDES_AUTOTUNE_STRIDE_DEFAULT;
    ctx->sync_autotune_pattern_lock = 0u;
    ctx->sync_autotune_error_lock = 0u;
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
    if (strcmp(key, "sync_autotune") == 0) {
        ctx->sync_autotune_enabled = (atoi(value) != 0) ? 1u : 0u;
        return 1;
    }
    if (strcmp(key, "sync_autotune_interval_blocks") == 0) {
        const int v = atoi(value);
        if (v >= 1 && v <= 2000) {
            ctx->sync_autotune_interval_blocks = (uint32_t)v;
            return 1;
        }
        return 0;
    }
    if (strcmp(key, "sync_autotune_stride_bits") == 0) {
        const int v = atoi(value);
        if (v >= 1 && v <= 64) {
            ctx->sync_autotune_stride_bits = (uint32_t)v;
            return 1;
        }
        return 0;
    }
    if (strcmp(key, "sync_autotune_pattern_lock") == 0) {
        ctx->sync_autotune_pattern_lock = (atoi(value) != 0) ? 1u : 0u;
        return 1;
    }
    if (strcmp(key, "sync_autotune_error_lock") == 0) {
        ctx->sync_autotune_error_lock = (atoi(value) != 0) ? 1u : 0u;
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
    maybe_autotune_sync(ctx, freq_hz, unix_ms, source_type, emit_fn, user_data);

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
