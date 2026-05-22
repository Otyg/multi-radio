/**
 * vdes_burst_decoder.c — VDES ASM burst detector / candidate extractor
 *
 * Role: MR_PLUGIN_ROLE_DECODER
 * Pipeline: vdes_asm_demod --[VDES_ASM_DATA]--> vdes_burst_decoder
 *
 * Receives demodulated pi/4-DQPSK bits and hunts for VDES ASM bursts:
 *
 *   1. Scans a streaming bit buffer for sync word matches.
 *   2. On a hit, emits VDES_DIAG_SYNC (offset + error count) and
 *      VDES_ASM_CANDIDATE (hex-packed bit window).
 *   3. Optionally dumps bit windows to files for offline analysis.
 *
 * Both normal and bit-inverted matches are tried (phase ambiguity).
 * After each hit the scan advances by candidate_bits to dedup overlaps.
 */

#include "mr_plugin_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── tunables ─────────────────────────────────────────────────────────── */

#define BD_STREAM_MAX_BITS   16384u
#define BD_TAIL_KEEP_BITS     2048u  /* keep at tail after trim */
#define BD_CANDIDATE_DEFAULT   480u  /* bits to extract after sync (default) */
#define BD_CANDIDATE_MIN       128u
#define BD_CANDIDATE_MAX      2048u
#define BD_SYNC_ERRORS_DEFAULT   2u  /* max bit errors in sync word */
#define BD_SYNC_MAX_LEN         64u
#define BD_DIAG_INTERVAL        50u
#define BD_DUMP_PATH_MAX       256u

/* ── built-in sync patterns ───────────────────────────────────────────── */

/*
 * Search for the Link ID directly in the bit stream.
 *
 * Because the signal gate opens at an arbitrary point during the VDES training
 * sequence, the demodulator's first emitted bit is NOT necessarily at a fixed
 * training-sequence position.  The Link ID (32 bits, §A2-1.2.3.4) always
 * follows immediately after the training sequence, so searching for it works
 * regardless of gate timing.
 *
 * Two variants per Link ID are tried:
 *   raw  = Reed-Muller (32,6) codeword from Table 3 (ITU-R M.2092-2)
 *   ota  = codeword XOR scramble mask 0xC2E28E4F (§A2-1.2.3.4)
 *
 * The decoder emits the candidate starting FROM the sync hit position, so the
 * first 32 bits of every candidate are the (possibly inverted) Link ID.
 *
 * False-positive rate per 32-bit pattern at 4 errors:
 *   C(32,4)*2/2^32 ≈ 39 ppm; 8 patterns total → ~310 ppm per position.
 *   Over a 5000-bit burst: ~1.5 false hits — confirmed/rejected by CRC in Python.
 */

/* Raw Reed-Muller codewords (Table 3) */
static const uint8_t kLID5r[32]  = {1,1,0,1,0,1,0,1, 1,1,1,0,1,1,0,1, 0,1,1,1,1,1,1,0, 1,0,1,1,1,1,1,1};
static const uint8_t kLID11r[32] = {1,1,1,0,1,1,0,1, 0,0,1,0,1,1,1,0, 1,1,0,0,0,0,1,0, 0,1,1,1,1,1,0,0};
static const uint8_t kLID17r[32] = {1,0,0,0,0,1,1,1, 0,0,1,1,0,1,1,1, 0,0,1,0,0,1,0,0, 1,1,1,0,0,1,0,1};
static const uint8_t kLID19r[32] = {1,0,0,0,1,1,1,1, 0,1,0,0,1,0,0,0, 0,0,1,0,0,1,0,0, 0,0,0,1,1,0,1,0};

/* OTA = codeword XOR 0xC2E28E4F (§A2-1.2.3.4 scramble mask) */
static const uint8_t kLID5o[32]  = {0,0,0,1,0,1,1,1, 0,0,0,0,1,1,1,1, 1,1,1,1,0,0,0,0, 1,1,1,1,0,0,0,0};
static const uint8_t kLID11o[32] = {0,0,1,0,1,1,1,1, 1,1,0,0,1,1,0,0, 0,1,0,0,1,1,0,0, 0,0,1,1,0,0,1,1};
static const uint8_t kLID17o[32] = {0,1,0,0,0,1,0,1, 1,1,0,1,0,1,0,1, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0};
static const uint8_t kLID19o[32] = {0,1,0,0,1,1,0,1, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 0,1,0,1,0,1,0,1};

typedef struct {
    const char*    name;
    const uint8_t* bits;
    uint8_t        len;
    uint32_t       max_errors; /* pattern-specific ceiling; 0 = exact match only */
} SyncDef;

static const SyncDef kBuiltinPatterns[] = {
    { "lid5o",  kLID5o,  32u, 4u },
    { "lid11o", kLID11o, 32u, 4u },
    { "lid17o", kLID17o, 32u, 4u },
    { "lid19o", kLID19o, 32u, 4u },
    { "lid5r",  kLID5r,  32u, 4u },
    { "lid11r", kLID11r, 32u, 4u },
    { "lid17r", kLID17r, 32u, 4u },
    { "lid19r", kLID19r, 32u, 4u },
};
#define BD_NUM_BUILTIN 8u

/* ── context ──────────────────────────────────────────────────────────── */

typedef struct {
    /* config */
    uint32_t candidate_bits;
    uint32_t sync_errors_max;
    uint32_t diag_interval_blocks;
    int      dump_enabled;
    char     dump_dir[BD_DUMP_PATH_MAX];

    /* optional user-supplied sync pattern (replaces all built-ins when set) */
    uint8_t  user_sync[BD_SYNC_MAX_LEN];
    uint8_t  user_sync_len;
    int      user_sync_set;

    /* streaming state */
    uint8_t* stream;
    uint32_t stream_cap;   /* allocated bits */
    uint32_t stream_bits;  /* valid bits */
    uint32_t scan_pos;     /* first unscanned position */

    /* stats */
    uint64_t blocks_seen;
    uint64_t sync_hits;
    uint64_t candidates_emitted;
    uint64_t dump_seq;
} VdesBurstDecCtx;

static const MrPluginMeta kMeta = {
    "vdes_burst_decoder",
    "0.1.0",
    MR_PLUGIN_API_VERSION,
    "VDES ASM burst detector: sync search + candidate extraction",
    MR_PLUGIN_ROLE_DECODER
};

/* ── bit helpers ──────────────────────────────────────────────────────── */

static int bd_get_bit(const uint8_t* buf, uint32_t idx) {
    return (buf[idx >> 3u] >> (7u - (idx & 7u))) & 1;
}

static void bd_set_bit(uint8_t* buf, uint32_t idx, int v) {
    const uint8_t mask = (uint8_t)(1u << (7u - (idx & 7u)));
    if (v) buf[idx >> 3u] |=  mask;
    else   buf[idx >> 3u] &= (uint8_t)~mask;
}

/* ── stream buffer ────────────────────────────────────────────────────── */

static int ensure_stream_cap(VdesBurstDecCtx* ctx, uint32_t need_bits) {
    uint32_t need_bytes, old_bytes;
    uint8_t* nb;
    if (need_bits <= ctx->stream_cap) return 1;
    if (need_bits > BD_STREAM_MAX_BITS) return 0;
    old_bytes  = (ctx->stream_cap + 7u) / 8u;
    need_bytes = (need_bits + 7u) / 8u;
    if (need_bytes < old_bytes * 2u) need_bytes = old_bytes * 2u;
    if (need_bytes < 512u) need_bytes = 512u;
    nb = (uint8_t*)realloc(ctx->stream, need_bytes);
    if (!nb) return 0;
    if (need_bytes > old_bytes) memset(nb + old_bytes, 0, need_bytes - old_bytes);
    ctx->stream = nb;
    ctx->stream_cap = need_bytes * 8u;
    return 1;
}

static void append_bits(VdesBurstDecCtx* ctx, const uint8_t* in, uint32_t n) {
    uint32_t i;
    if (!in || n == 0u || !ensure_stream_cap(ctx, ctx->stream_bits + n)) return;
    for (i = 0; i < n; ++i)
        bd_set_bit(ctx->stream, ctx->stream_bits + i, bd_get_bit(in, i));
    ctx->stream_bits += n;
}

/*
 * Discard old bits, keeping the last BD_TAIL_KEEP_BITS.
 * Returns number of bits trimmed (used to adjust scan_pos).
 */
static uint32_t keep_stream_tail(VdesBurstDecCtx* ctx) {
    uint32_t keep, trim, i;
    keep = BD_TAIL_KEEP_BITS;
    if (ctx->stream_bits <= keep) return 0u;
    trim = ctx->stream_bits - keep;
    for (i = 0; i < keep; ++i)
        bd_set_bit(ctx->stream, i, bd_get_bit(ctx->stream, trim + i));
    ctx->stream_bits = keep;
    return trim;
}

/* ── sync matching ────────────────────────────────────────────────────── */

/*
 * Count bit errors for 'sync' vs stream starting at 'pos'.
 * Returns BD_SYNC_MAX_LEN+1 when the window extends past stream end.
 */
static uint32_t count_sync_errors(const uint8_t* stream, uint32_t stream_bits,
                                   uint32_t pos,
                                   const uint8_t* sync, uint8_t sync_len,
                                   int inverted) {
    uint32_t err = 0u, i;
    if ((uint32_t)sync_len == 0u || pos + (uint32_t)sync_len > stream_bits)
        return BD_SYNC_MAX_LEN + 1u;
    for (i = 0u; i < (uint32_t)sync_len; ++i) {
        int b = bd_get_bit(stream, pos + i);
        int s = sync[i] ? 1 : 0;
        if (inverted) s ^= 1;
        if (b != s) {
            if (++err > BD_SYNC_MAX_LEN) return err;
        }
    }
    return err;
}

/* ── emit ─────────────────────────────────────────────────────────────── */

static void emit_diag_sync(VdesBurstDecCtx* ctx,
                            double freq_hz, uint64_t unix_ms,
                            MrEmitFn emit_fn, void* user_data,
                            const char* source_type,
                            const char* pattern_name,
                            uint32_t sync_offset, uint32_t sync_errors,
                            int inverted, uint32_t cand_bits) {
    char payload[192];
    char kv[448];
    snprintf(payload, sizeof(payload),
             "VDES_DIAG_SYNC pattern=%s offset=%u err=%u inv=%d cand=%u",
             pattern_name, sync_offset, sync_errors, inverted, cand_bits);
    snprintf(kv, sizeof(kv),
             "{\"signal_type\":\"VDES_DIAG_SYNC\","
             "\"source_type\":\"%s\","
             "\"pattern\":\"%s\","
             "\"sync_offset_bits\":\"%u\","
             "\"sync_errors\":\"%u\","
             "\"sync_inverted\":\"%d\","
             "\"candidate_bits\":\"%u\","
             "\"sync_hits\":\"%llu\","
             "\"candidates_emitted\":\"%llu\","
             "\"blocks_seen\":\"%llu\"}",
             source_type ? source_type : "",
             pattern_name,
             sync_offset, sync_errors, inverted ? 1 : 0, cand_bits,
             (unsigned long long)ctx->sync_hits,
             (unsigned long long)ctx->candidates_emitted,
             (unsigned long long)ctx->blocks_seen);
    emit_fn("VDES_DIAG_SYNC", payload, freq_hz, unix_ms, kv, user_data);
}

static void emit_candidate_bits(VdesBurstDecCtx* ctx,
                                 double freq_hz, uint64_t unix_ms,
                                 MrEmitFn emit_fn, void* user_data,
                                 const char* source_type,
                                 const char* pattern_name,
                                 uint32_t sync_offset, uint32_t sync_errors, int inverted,
                                 uint32_t cand_start) {
    uint32_t avail, use_bits, out_bytes, i;
    uint8_t* out;
    char* hex;
    char kv[448];

    if (cand_start >= ctx->stream_bits) return;
    avail    = ctx->stream_bits - cand_start;
    use_bits = avail < ctx->candidate_bits ? avail : ctx->candidate_bits;
    if (use_bits < BD_CANDIDATE_MIN) return;

    out_bytes = (use_bits + 7u) / 8u;
    out = (uint8_t*)calloc(out_bytes, 1u);
    if (!out) return;
    for (i = 0u; i < use_bits; ++i)
        bd_set_bit(out, i, bd_get_bit(ctx->stream, cand_start + i));

    hex = (char*)malloc((size_t)out_bytes * 2u + 1u);
    if (!hex) { free(out); return; }
    for (i = 0u; i < out_bytes; ++i)
        snprintf(hex + i * 2u, 3u, "%02X", (unsigned)out[i]);

    snprintf(kv, sizeof(kv),
             "{\"signal_type\":\"VDES_ASM_CANDIDATE\","
             "\"source_type\":\"%s\","
             "\"pattern\":\"%s\","
             "\"sync_offset_bits\":\"%u\","
             "\"sync_errors\":\"%u\","
             "\"sync_inverted\":\"%d\","
             "\"candidate_bits\":\"%u\","
             "\"candidates_emitted\":\"%llu\"}",
             source_type ? source_type : "",
             pattern_name,
             sync_offset, sync_errors, inverted ? 1 : 0, use_bits,
             (unsigned long long)ctx->candidates_emitted);

    emit_fn("VDES_ASM_CANDIDATE", hex, freq_hz, unix_ms, kv, user_data);
    ctx->candidates_emitted++;

    if (ctx->dump_enabled && ctx->dump_dir[0]) {
        char path[BD_DUMP_PATH_MAX + 80];
        FILE* fp;
        snprintf(path, sizeof(path), "%s/vdes_cand_%06llu_off%u_err%u.bin",
                 ctx->dump_dir,
                 (unsigned long long)ctx->dump_seq,
                 sync_offset, sync_errors);
        fp = fopen(path, "wb");
        if (fp) {
            fwrite(out, 1u, out_bytes, fp);
            fclose(fp);
            ctx->dump_seq++;
        }
    }

    free(hex);
    free(out);
}

/* ── plugin API ───────────────────────────────────────────────────────── */

MrPluginCtx* mr_plugin_create(void) {
    VdesBurstDecCtx* ctx = (VdesBurstDecCtx*)calloc(1, sizeof(*ctx));
    if (!ctx) return NULL;
    ctx->candidate_bits     = BD_CANDIDATE_DEFAULT;
    ctx->sync_errors_max    = BD_SYNC_ERRORS_DEFAULT;
    ctx->diag_interval_blocks = BD_DIAG_INTERVAL;
    return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
    VdesBurstDecCtx* ctx = (VdesBurstDecCtx*)raw;
    if (!ctx) return;
    free(ctx->stream);
    free(ctx);
}

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

static int parse_sync_bits(const char* value, uint8_t* out, uint8_t* out_len) {
    uint8_t n = 0u;
    if (!value || !out || !out_len) return 0;
    while (*value) {
        if (*value == '0' || *value == '1') {
            if (n >= BD_SYNC_MAX_LEN) return 0;
            out[n++] = (uint8_t)(*value - '0');
        }
        value++;
    }
    if (n < 8u) return 0;
    *out_len = n;
    return 1;
}

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
    VdesBurstDecCtx* ctx = (VdesBurstDecCtx*)raw;
    if (!ctx || !key || !value) return 0;

    if (strcmp(key, "candidate_bits") == 0) {
        const int v = atoi(value);
        if (v >= (int)BD_CANDIDATE_MIN && v <= (int)BD_CANDIDATE_MAX) {
            ctx->candidate_bits = (uint32_t)v;
            return 1;
        }
        return 0;
    }
    if (strcmp(key, "sync_errors_max") == 0) {
        const int v = atoi(value);
        if (v >= 0 && v <= 8) { ctx->sync_errors_max = (uint32_t)v; return 1; }
        return 0;
    }
    if (strcmp(key, "diag_interval_blocks") == 0) {
        const int v = atoi(value);
        if (v >= 0) { ctx->diag_interval_blocks = (uint32_t)v; return 1; }
        return 0;
    }
    if (strcmp(key, "sync_bits") == 0) {
        uint8_t tmp[BD_SYNC_MAX_LEN]; uint8_t n = 0u;
        if (parse_sync_bits(value, tmp, &n)) {
            memcpy(ctx->user_sync, tmp, n);
            ctx->user_sync_len = n;
            ctx->user_sync_set = 1;
            return 1;
        }
        return 0;
    }
    if (strcmp(key, "dump_dir") == 0) {
        strncpy(ctx->dump_dir, value, BD_DUMP_PATH_MAX - 1u);
        ctx->dump_dir[BD_DUMP_PATH_MAX - 1u] = '\0';
        ctx->dump_enabled = (ctx->dump_dir[0] != '\0');
        return 1;
    }
    if (strcmp(key, "dump_enabled") == 0) {
        ctx->dump_enabled = (atoi(value) != 0) ? 1 : 0;
        return 1;
    }
    return 0;
}

void mr_plugin_process_bits(MrPluginCtx* raw,
                             const uint8_t* bit_bytes, uint32_t bit_count,
                             double freq_hz, uint64_t unix_ms,
                             const char* source_type,
                             MrEmitFn emit_fn, void* user_data) {
    VdesBurstDecCtx* ctx = (VdesBurstDecCtx*)raw;
    uint32_t i, trim, backup;
    int found_any = 0;

    if (!ctx || !bit_bytes || bit_count == 0u || !emit_fn) return;
    ctx->blocks_seen++;

    append_bits(ctx, bit_bytes, bit_count);

    /* Scan from last position to end of stream. */
    i = ctx->scan_pos;
    while (i < ctx->stream_bits) {
        uint32_t p;

        if (ctx->user_sync_set) {
            /* Only try the user-specified pattern. */
            uint32_t e0 = count_sync_errors(ctx->stream, ctx->stream_bits,
                                             i, ctx->user_sync, ctx->user_sync_len, 0);
            uint32_t e1 = count_sync_errors(ctx->stream, ctx->stream_bits,
                                             i, ctx->user_sync, ctx->user_sync_len, 1);
            if (e0 <= ctx->sync_errors_max || e1 <= ctx->sync_errors_max) {
                int inv = (e1 < e0) ? 1 : 0;
                uint32_t err = inv ? e1 : e0;
                ctx->sync_hits++;
                found_any = 1;
                emit_diag_sync(ctx, freq_hz, unix_ms, emit_fn, user_data,
                               source_type, "user",
                               i, err, inv, ctx->candidate_bits);
                emit_candidate_bits(ctx, freq_hz, unix_ms, emit_fn, user_data,
                                    source_type, "user",
                                    i, err, inv,
                                    i);
                i += ctx->candidate_bits;
                continue;
            }
            i++;
            continue;
        }

        /* Try all built-in patterns, each with its own error ceiling. */
        {
            int hit = 0;
            for (p = 0u; p < BD_NUM_BUILTIN && !hit; ++p) {
                uint8_t  slen  = kBuiltinPatterns[p].len;
                uint32_t plim  = kBuiltinPatterns[p].max_errors;
                /* Global sync_errors_max can only tighten, never loosen. */
                if (ctx->sync_errors_max < plim) plim = ctx->sync_errors_max;
                uint32_t e0 = count_sync_errors(ctx->stream, ctx->stream_bits,
                                                 i, kBuiltinPatterns[p].bits, slen, 0);
                uint32_t e1 = count_sync_errors(ctx->stream, ctx->stream_bits,
                                                 i, kBuiltinPatterns[p].bits, slen, 1);
                if (e0 <= plim || e1 <= plim) {
                    int inv = (e1 < e0) ? 1 : 0;
                    uint32_t err = inv ? e1 : e0;
                    ctx->sync_hits++;
                    found_any = 1;
                    hit = 1;
                    emit_diag_sync(ctx, freq_hz, unix_ms, emit_fn, user_data,
                                   source_type, kBuiltinPatterns[p].name,
                                   i, err, inv, ctx->candidate_bits);
                    /* Candidate starts AT the sync (Link ID position), not after it,
                     * so the Python decoder sees the Link ID as bits[0:32]. */
                    emit_candidate_bits(ctx, freq_hz, unix_ms, emit_fn, user_data,
                                        source_type, kBuiltinPatterns[p].name,
                                        i, err, inv,
                                        i);
                    i += ctx->candidate_bits;
                }
            }
            if (!hit) i++;
        }
    }
    ctx->scan_pos = i;

    /* Periodic no-sync diagnostic. */
    if (!found_any &&
        ctx->diag_interval_blocks > 0u &&
        (ctx->blocks_seen % ctx->diag_interval_blocks) == 0u) {
        char payload[192];
        char kv[320];
        snprintf(payload, sizeof(payload),
                 "VDES_DIAG_SYNC no-sync stream=%u bits block=%u bits hits=%llu",
                 ctx->stream_bits, bit_count,
                 (unsigned long long)ctx->sync_hits);
        snprintf(kv, sizeof(kv),
                 "{\"signal_type\":\"VDES_DIAG_SYNC\","
                 "\"source_type\":\"%s\","
                 "\"sync_found\":\"0\","
                 "\"stream_bits\":\"%u\","
                 "\"bit_count\":\"%u\","
                 "\"sync_hits\":\"%llu\","
                 "\"blocks_seen\":\"%llu\"}",
                 source_type ? source_type : "",
                 ctx->stream_bits, bit_count,
                 (unsigned long long)ctx->sync_hits,
                 (unsigned long long)ctx->blocks_seen);
        emit_fn("VDES_DIAG_SYNC", payload, freq_hz, unix_ms, kv, user_data);
    }

    trim = keep_stream_tail(ctx);
    if (trim > 0u) {
        backup = BD_SYNC_MAX_LEN;
        ctx->scan_pos = (ctx->scan_pos >= trim + backup)
                        ? (ctx->scan_pos - trim - backup)
                        : 0u;
    }
}

void mr_plugin_process_iq(MrPluginCtx* ctx,
                           const int16_t* iq, uint32_t num_pairs,
                           uint32_t sr, double freq_hz, uint64_t unix_ms,
                           MrEmitFn emit_fn, void* user_data) {
    (void)ctx; (void)iq; (void)num_pairs; (void)sr;
    (void)freq_hz; (void)unix_ms; (void)emit_fn; (void)user_data;
}
