/**
 * vdes_asm_postproc.c — EXPERIMENTAL VDES ASM postprocessor sketch
 *
 * Role: MR_PLUGIN_ROLE_POSTPROCESSING
 *
 * Current behavior:
 *   Emits lightweight diagnostics for the VDES ASM scaffold path.
 *
 * TODO:
 *   - parse ASM link-layer message units after proper decoder implementation
 *   - map DAC/FI/message semantics where applicable for VDES ASM payloads
 */

#include "mr_plugin_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint64_t blocks_seen;
    uint64_t bits_seen;
} VdesAsmPostCtx;

static const MrPluginMeta kMeta = {
    "vdes_asm_postproc",
    "0.1.0",
    MR_PLUGIN_API_VERSION,
    "EXPERIMENTAL: VDES ASM postprocessor scaffold",
    MR_PLUGIN_ROLE_POSTPROCESSING
};

MrPluginCtx* mr_plugin_create(void) {
    return (MrPluginCtx*)calloc(1, sizeof(VdesAsmPostCtx));
}

void mr_plugin_destroy(MrPluginCtx* raw) {
    free(raw);
}

const MrPluginMeta* mr_plugin_get_meta(void) {
    return &kMeta;
}

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
    char payload[128];
    char kv[256];
    (void)bit_bytes;
    if (!ctx || !emit_fn) return;

    ctx->blocks_seen++;
    ctx->bits_seen += bit_count;

    snprintf(payload, sizeof(payload),
             "VDES ASM scaffold: blocks=%llu bits=%llu",
             (unsigned long long)ctx->blocks_seen,
             (unsigned long long)ctx->bits_seen);
    snprintf(kv, sizeof(kv),
             "{\"signal_type\":\"VDES_ASM_TBD\","
             "\"source_type\":\"%s\","
             "\"bit_count\":\"%u\","
             "\"blocks_seen\":\"%llu\","
             "\"bits_seen\":\"%llu\"}",
             source_type ? source_type : "",
             bit_count,
             (unsigned long long)ctx->blocks_seen,
             (unsigned long long)ctx->bits_seen);
    emit_fn("VDES_ASM_TBD", payload, freq_hz, unix_ms, kv, user_data);
}

void mr_plugin_process_iq(MrPluginCtx* ctx,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
    (void)ctx; (void)iq; (void)num_pairs; (void)sr;
    (void)freq_hz; (void)unix_ms; (void)emit_fn; (void)user_data;
}
