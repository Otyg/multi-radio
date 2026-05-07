/**
 * mr_bit_buf.h — Packed bit-buffer helpers for demodulator plugins.
 *
 * Bits are stored MSB-first within each byte: the first received bit goes
 * into bit 7 of byte 0, the next into bit 6, and so on.  This matches the
 * convention used by gmsk_demod and fsk_demod when feeding downstream
 * decoder plugins via mr_plugin_process_bits.
 *
 * No libliquid dependency; plain C99.
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* Core helpers                                                         */
/* ------------------------------------------------------------------ */

/**
 * Pack one bit into buf at position *count (0 = first received bit).
 * Does nothing if *count >= max_bits.
 */
static inline void mr_push_bit(uint8_t* buf, uint32_t* count,
                                uint32_t max_bits, unsigned int bit) {
    if (*count >= max_bits) return;
    const uint32_t byte_idx = *count / 8u;
    const uint32_t bit_idx  = 7u - (*count % 8u);
    if (bit) buf[byte_idx] |=  (uint8_t)(1u << bit_idx);
    else     buf[byte_idx] &= (uint8_t)~(1u << bit_idx);
    ++(*count);
}

/**
 * Emit accumulated bits as a hex-encoded signal via emit_fn, then reset the
 * buffer.  signal_type and kv_json are passed verbatim to emit_fn.
 *
 * Does nothing (but still resets) if *bit_count < min_bits.
 *
 * Returns 1 if emit_fn was called, 0 otherwise.
 */
/**
 * Emit accumulated bits and reset the buffer.
 *
 * buf_bytes  total allocated size of buf in bytes (used for memset)
 */
static inline int mr_emit_bits(uint8_t* buf, uint32_t* bit_count,
                                uint32_t min_bits, uint32_t buf_bytes,
                                const char* signal_type, const char* kv_json,
                                double freq_hz, uint64_t unix_ms,
                                void (*emit_fn)(const char*, const char*, double,
                                                uint64_t, const char*, void*),
                                void* user_data) {
    int emitted = 0;
    if (*bit_count >= min_bits) {
        const uint32_t byte_count = *bit_count / 8u;
        char* hex = (char*)malloc(byte_count * 2u + 1u);
        if (hex) {
            for (uint32_t i = 0; i < byte_count; ++i)
                snprintf(hex + i * 2, 3, "%02X", (unsigned)buf[i]);
            if (emit_fn) emit_fn(signal_type, hex, freq_hz, unix_ms, kv_json, user_data);
            free(hex);
            emitted = 1;
        }
    }
    *bit_count = 0;
    memset(buf, 0, buf_bytes);
    return emitted;
}
