/**
 * vdes_asm_decoder.c — EXPERIMENTAL VDES ASM link decoder sketch
 *
 * Role: MR_PLUGIN_ROLE_DECODER
 *
 * Current behavior:
 *   - Accepts provisional bitstream from vdes_asm_demod (VDES_ASM_DATA)
 *   - Passes bits onward with explicit "sketch/no-FEC" metadata
 *
 * TODO:
 *   - ASM burst/frame synchronization
 *   - deinterleaving, descrambling
 *   - FEC decoding per VDES ASM profile
 */

#include "mr_plugin_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const MrPluginMeta kMeta = {
    "vdes_asm_decoder",
    "0.1.0",
    MR_PLUGIN_API_VERSION,
    "EXPERIMENTAL: VDES ASM link decoder scaffold (sync/FEC TBD)",
    MR_PLUGIN_ROLE_DECODER
};

MrPluginCtx* mr_plugin_create(void) { return (MrPluginCtx*)calloc(1, 1); }
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
    uint32_t i;
    const uint32_t byte_count = (bit_count + 7u) / 8u;
    char* hex;
    char kv[256];
    (void)raw;
    if (!bit_bytes || bit_count == 0u || !emit_fn) return;

    hex = (char*)malloc((size_t)byte_count * 2u + 1u);
    if (!hex) return;
    for (i = 0; i < byte_count; ++i) snprintf(hex + i * 2u, 3, "%02X", (unsigned)bit_bytes[i]);

    snprintf(kv, sizeof(kv),
             "{\"signal_type\":\"VDES_ASM_L2\","
             "\"source_type\":\"%s\","
             "\"bit_count\":\"%u\","
             "\"decoder_scope\":\"VDES_ASM_SKETCH\","
             "\"fec\":\"not_implemented\"}",
             source_type ? source_type : "", bit_count);
    emit_fn("VDES_ASM_L2", hex, freq_hz, unix_ms, kv, user_data);
    free(hex);
}

void mr_plugin_process_iq(MrPluginCtx* ctx,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
    (void)ctx; (void)iq; (void)num_pairs; (void)sr;
    (void)freq_hz; (void)unix_ms; (void)emit_fn; (void)user_data;
}
