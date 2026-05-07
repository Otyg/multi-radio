/**
 * nrzi_decoder.c — NRZI (Non-Return-to-Zero Inverted) decoder plugin
 *
 * Role: MR_PLUGIN_ROLE_DECODER
 *
 * NRZI encoding: a bit transition represents a '1', no transition a '0'
 * (or inverted, depending on convention — selectable via mr_plugin_set_param).
 *
 * Receives packed bit bytes via mr_plugin_process_bits and emits the
 * differentially-decoded result as signal_type "NRZI_DATA".
 *
 * Environment / runtime params:
 *   MR_NRZI_INVERT   "1" to invert the convention (transition='0'), default 0
 */

#include "mr_plugin_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  int invert;      /* 0: transition=1 (standard), 1: transition=0 */
  int last_bit;    /* last input bit seen, for differential decode */
} NrziCtx;

/* ------------------------------------------------------------------ */
/* Plugin API                                                           */
/* ------------------------------------------------------------------ */

static const MrPluginMeta kMeta = {
  "nrzi_decoder",
  "1.0.0",
  MR_PLUGIN_API_VERSION,
  "NRZI differential decoder (transition → bit)",
  MR_PLUGIN_ROLE_DECODER
};

MrPluginCtx* mr_plugin_create(void) {
  NrziCtx* ctx = (NrziCtx*)calloc(1, sizeof(NrziCtx));
  if (!ctx) return NULL;
  const char* inv = getenv("MR_NRZI_INVERT");
  ctx->invert   = (inv && atoi(inv)) ? 1 : 0;
  ctx->last_bit = 0;
  return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) { free(raw); }

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
  if (!raw || !key || !value) return 0;
  NrziCtx* ctx = (NrziCtx*)raw;
  if (strcmp(key, "invert") == 0) { ctx->invert = atoi(value) ? 1 : 0; return 1; }
  return 0;
}

void mr_plugin_process_bits(MrPluginCtx* raw,
                            const uint8_t* bit_bytes, uint32_t bit_count,
                            double freq_hz, uint64_t unix_ms,
                            const char* source_type,
                            MrEmitFn emit_fn, void* user_data) {
  if (!raw || !bit_bytes || bit_count == 0) return;
  NrziCtx* ctx = (NrziCtx*)raw;

  const uint32_t byte_count = (bit_count + 7) / 8;
  uint8_t* out = (uint8_t*)calloc(byte_count, 1);
  if (!out) return;

  for (uint32_t i = 0; i < bit_count; ++i) {
    const uint32_t bi = i / 8;
    const uint32_t mask = 1u << (7 - (i % 8));
    const int cur = (bit_bytes[bi] & mask) ? 1 : 0;
    /* NRZI decode: output bit = transition XOR invert */
    const int transition = cur ^ ctx->last_bit;
    const int decoded = ctx->invert ? !transition : transition;
    ctx->last_bit = cur;
    if (decoded) out[bi] |=  (uint8_t)mask;
    else         out[bi] &= (uint8_t)~mask;
  }

  /* Build hex payload */
  char* hex = (char*)malloc(byte_count * 2 + 1);
  if (!hex) { free(out); return; }
  for (uint32_t i = 0; i < byte_count; ++i)
    snprintf(hex + i * 2, 3, "%02X", (unsigned)out[i]);
  free(out);

  char kv[128];
  snprintf(kv, sizeof(kv),
           "{\"source_type\":\"%s\",\"invert\":\"%d\",\"bit_count\":\"%u\"}",
           source_type ? source_type : "", ctx->invert, bit_count);

  if (emit_fn) emit_fn("NRZI_DATA", hex, freq_hz, unix_ms, kv, user_data);
  free(hex);
}

/* mr_plugin_process_iq is not used for decoder role but must be linkable */
void mr_plugin_process_iq(MrPluginCtx* ctx,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
  (void)ctx; (void)iq; (void)num_pairs; (void)sr;
  (void)freq_hz; (void)unix_ms; (void)emit_fn; (void)user_data;
}
