/**
 * mr_plugin_api.h — Multi-Radio stable C plugin ABI (version 1)
 *
 * Each plugin .so must export the four functions declared at the bottom of
 * this file.  The host loads them via dlopen/dlsym and calls them without
 * ever knowing about the plugin's internal state.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Version                                                              */
/* ------------------------------------------------------------------ */

#define MR_PLUGIN_API_VERSION 1

/* ------------------------------------------------------------------ */
/* Plugin roles                                                         */
/* ------------------------------------------------------------------ */

typedef enum {
  MR_PLUGIN_ROLE_DEMODULATOR    = 1,  /* IQ → raw bits / symbols       */
  MR_PLUGIN_ROLE_DECODER        = 2,  /* raw bits → protocol data       */
  MR_PLUGIN_ROLE_POSTPROCESSING = 3,  /* transforms decoded messages     */
} MrPluginRole;

/* ------------------------------------------------------------------ */
/* Emit callback                                                        */
/* ------------------------------------------------------------------ */

/**
 * MrEmitFn — function pointer the host passes to mr_plugin_process_iq.
 *
 * The plugin calls this whenever it has decoded a complete message.
 *
 * @param signal_type         NUL-terminated string, e.g. "FSK_DATA".
 * @param payload             NUL-terminated hex string of the decoded bytes.
 * @param frequency_hz        Centre frequency of the channel (Hz).
 * @param unix_ms             Timestamp of the IQ block (ms since Unix epoch).
 * @param normalized_kv_json  NUL-terminated JSON object with extra fields,
 *                            e.g. {"baud_rate":"4800","bit_count":"8"}.
 *                            May be NULL.
 * @param user_data           Opaque pointer supplied by the host.
 */
typedef void (*MrEmitFn)(const char* signal_type,
                         const char* payload,
                         double      frequency_hz,
                         uint64_t    unix_ms,
                         const char* normalized_kv_json,
                         void*       user_data);

/* ------------------------------------------------------------------ */
/* Plugin metadata                                                      */
/* ------------------------------------------------------------------ */

typedef struct MrPluginMeta {
  const char*  name;         /**< Short identifier, e.g. "fsk_demod"   */
  const char*  version;      /**< SemVer string, e.g. "1.0.0"          */
  uint32_t     api_version;  /**< Must equal MR_PLUGIN_API_VERSION      */
  const char*  description;  /**< Human-readable one-liner              */
  MrPluginRole role;         /**< Functional role of this plugin        */
} MrPluginMeta;

/* ------------------------------------------------------------------ */
/* Opaque plugin context                                                */
/* ------------------------------------------------------------------ */

typedef void MrPluginCtx;

/* ------------------------------------------------------------------ */
/* Mandatory exports — every plugin .so must provide all four          */
/* ------------------------------------------------------------------ */

/**
 * mr_plugin_create — allocate and initialise plugin state.
 * Returns NULL on failure.
 */
MrPluginCtx* mr_plugin_create(void);

/**
 * mr_plugin_destroy — release all resources allocated by mr_plugin_create.
 */
void mr_plugin_destroy(MrPluginCtx* ctx);

/**
 * mr_plugin_get_meta — return a pointer to static plugin metadata.
 * The returned pointer must remain valid for the lifetime of the .so.
 */
const MrPluginMeta* mr_plugin_get_meta(void);

/**
 * mr_plugin_process_bits — decode a packed bit stream (OPTIONAL export).
 *
 * Called by the host on DECODER plugins after a demodulator has emitted bits.
 *
 * @param ctx               Opaque context.
 * @param bit_bytes         Packed bits, MSB first (bit 7 of byte 0 = first bit).
 * @param bit_count         Total number of valid bits in bit_bytes.
 * @param freq_hz           Centre frequency (Hz).
 * @param unix_ms           Timestamp (ms since Unix epoch).
 * @param source_type       Signal type string from the upstream demodulator,
 *                          e.g. "GMSK_DATA".
 * @param emit_fn / user_data  Same emit convention as mr_plugin_process_iq.
 */
void mr_plugin_process_bits(MrPluginCtx* ctx,
                            const uint8_t* bit_bytes,
                            uint32_t       bit_count,
                            double         freq_hz,
                            uint64_t       unix_ms,
                            const char*    source_type,
                            MrEmitFn       emit_fn,
                            void*          user_data);

/**
 * mr_plugin_set_param — update a runtime parameter (OPTIONAL export).
 *
 * The host calls this when configuration changes, before the next
 * mr_plugin_process_iq call.  Plugins that do not export this symbol
 * are simply not reconfigured at runtime.
 *
 * @param ctx    Opaque context.
 * @param key    NUL-terminated parameter name, e.g. "baud_rate".
 * @param value  NUL-terminated string value, e.g. "9600".
 * @return       1 if the parameter was accepted, 0 if unknown.
 */
int mr_plugin_set_param(MrPluginCtx* ctx, const char* key, const char* value);

/**
 * mr_plugin_process_iq — demodulate one IQ block.
 *
 * @param ctx             Opaque context returned by mr_plugin_create.
 * @param iq_samples      Interleaved int16 I/Q samples (I0,Q0,I1,Q1,...).
 * @param num_pairs       Number of I/Q pairs (total elements = num_pairs*2).
 * @param sample_rate_hz  Sample rate of iq_samples (Hz).
 * @param center_freq_hz  Tuned centre frequency (Hz).
 * @param unix_ms         Block timestamp (ms since Unix epoch).
 * @param emit_fn         Callback to invoke for each decoded message.
 * @param user_data       Forwarded verbatim to emit_fn.
 */
void mr_plugin_process_iq(MrPluginCtx* ctx,
                          const int16_t* iq_samples,
                          uint32_t       num_pairs,
                          uint32_t       sample_rate_hz,
                          double         center_freq_hz,
                          uint64_t       unix_ms,
                          MrEmitFn       emit_fn,
                          void*          user_data);

#ifdef __cplusplus
}
#endif
