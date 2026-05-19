/**
 * vdes_iq_recorder.c - null demodulator for VDES/ASM IQ capture
 *
 * Role: MR_PLUGIN_ROLE_DEMODULATOR
 *
 * This plugin intentionally performs no demodulation and emits no decoded
 * messages. It records the incoming interleaved int16 IQ stream to .iq16 files
 * and writes a JSON sidecar with capture metadata.
 *
 * Recording is controlled explicitly via set_param:
 *   recording_start  open a new file immediately (on next IQ block)
 *   recording_stop   flush and close the current file
 *
 * Environment (read once at create time):
 *   MR_IQ_RECORD_DIR        output directory, default: recordings/vdes
 *   MR_IQ_RECORD_PREFIX     filename prefix, default: vdes
 *   MR_IQ_RECORD_ENABLED    0 disables start, default: enabled
 *   MR_IQ_RECORD_MAX_BYTES  safety limit: close (no restart) when reached,
 *                           default: 0 (disabled)
 */

#include "mr_plugin_api.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#if defined(_WIN32)
#include <direct.h>
#define MR_MKDIR(path) _mkdir(path)
#else
#define MR_MKDIR(path) mkdir(path, 0775)
#endif

#define MR_IQ_REC_DEFAULT_DIR "recordings/"
#define MR_IQ_REC_DEFAULT_PREFIX "raw"
#define MR_IQ_REC_DEFAULT_MAX_BYTES 0ull
#define MR_IQ_REC_PATH_MAX 1024u
#define MR_IQ_REC_TOKEN_MAX 96u

typedef struct {
  int enabled;
  int recording_requested;
  int write_failed;

  char output_dir[MR_IQ_REC_PATH_MAX];
  char prefix[MR_IQ_REC_TOKEN_MAX];
  uint64_t max_bytes;

  FILE* iq_file;
  char iq_path[MR_IQ_REC_PATH_MAX];
  char json_path[MR_IQ_REC_PATH_MAX];

  uint32_t sample_rate_hz;
  double center_freq_hz;
  uint64_t first_unix_ms;
  uint64_t last_unix_ms;
  uint64_t block_count;
  uint64_t iq_pair_count;
  uint64_t byte_count;
  uint64_t file_index;
} IqRecorderCtx;

static const MrPluginMeta kMeta = {
  "iq_recorder",
  "0.1.0",
  MR_PLUGIN_API_VERSION,
  "Null demodulator: records IQ to .iq16 with JSON metadata",
  MR_PLUGIN_ROLE_DEMODULATOR
};

static int env_disabled(const char* value) {
  return value != NULL && value[0] != '\0' &&
         (strcmp(value, "0") == 0 || strcmp(value, "false") == 0 ||
          strcmp(value, "FALSE") == 0 || strcmp(value, "off") == 0 ||
          strcmp(value, "OFF") == 0);
}

static uint64_t parse_u64_env(const char* key, uint64_t fallback) {
  const char* value = getenv(key);
  char* end = NULL;
  unsigned long long parsed;
  if (!value || !value[0]) return fallback;
  errno = 0;
  parsed = strtoull(value, &end, 10);
  if (errno != 0 || end == value) return fallback;
  return (uint64_t)parsed;
}

static void copy_string(char* dst, size_t dst_len, const char* src, const char* fallback) {
  const char* value = (src && src[0]) ? src : fallback;
  if (!dst || dst_len == 0) return;
  snprintf(dst, dst_len, "%s", value ? value : "");
}

static void sanitize_token(char* s) {
  if (!s) return;
  for (char* p = s; *p; ++p) {
    const int ok = ((*p >= 'a' && *p <= 'z') ||
                    (*p >= 'A' && *p <= 'Z') ||
                    (*p >= '0' && *p <= '9') ||
                    *p == '-' || *p == '_');
    if (!ok) *p = '_';
  }
}

static int mkdir_p(const char* path) {
  char tmp[MR_IQ_REC_PATH_MAX];
  size_t len;

  if (!path || !path[0]) return 0;
  snprintf(tmp, sizeof(tmp), "%s", path);
  len = strlen(tmp);
  if (len == 0) return 0;
  if (tmp[len - 1] == '/') tmp[len - 1] = '\0';

  for (char* p = tmp + 1; *p; ++p) {
    if (*p != '/') continue;
    *p = '\0';
    if (MR_MKDIR(tmp) != 0 && errno != EEXIST) return 0;
    *p = '/';
  }
  if (MR_MKDIR(tmp) != 0 && errno != EEXIST) return 0;
  return 1;
}

static void format_utc(uint64_t unix_ms, char* out, size_t out_len) {
  time_t seconds = (time_t)(unix_ms / 1000ull);
  struct tm tm_utc;
  memset(&tm_utc, 0, sizeof(tm_utc));
#if defined(_WIN32)
  gmtime_s(&tm_utc, &seconds);
#else
  gmtime_r(&seconds, &tm_utc);
#endif
  strftime(out, out_len, "%Y%m%dT%H%M%SZ", &tm_utc);
}

static void write_json(IqRecorderCtx* ctx, int closed) {
  FILE* f;
  char started[32];
  char updated[32];

  if (!ctx || !ctx->json_path[0]) return;
  format_utc(ctx->first_unix_ms, started, sizeof(started));
  format_utc(ctx->last_unix_ms ? ctx->last_unix_ms : ctx->first_unix_ms,
             updated, sizeof(updated));

  f = fopen(ctx->json_path, "wb");
  if (!f) return;
  fprintf(f,
          "{\n"
          "  \"format\": \"s16le_interleaved_iq\",\n"
          "  \"component_type\": \"int16\",\n"
          "  \"byte_order\": \"little_endian\",\n"
          "  \"layout\": \"I0,Q0,I1,Q1,...\",\n"
          "  \"source_plugin\": \"iq_recorder\",\n"
          "  \"recording_complete\": %s,\n"
          "  \"started_utc\": \"%s\",\n"
          "  \"updated_utc\": \"%s\",\n"
          "  \"first_unix_ms\": %llu,\n"
          "  \"last_unix_ms\": %llu,\n"
          "  \"center_frequency_hz\": %.3f,\n"
          "  \"sample_rate_hz\": %u,\n"
          "  \"block_count\": %llu,\n"
          "  \"iq_pair_count\": %llu,\n"
          "  \"byte_count\": %llu,\n"
          "  \"iq_file\": \"%s\",\n"
          "  \"json_file\": \"%s\",\n"
          "  \"notes\": \"Raw receiver IQ captured without demodulation.\"\n"
          "}\n",
          closed ? "true" : "false",
          started,
          updated,
          (unsigned long long)ctx->first_unix_ms,
          (unsigned long long)ctx->last_unix_ms,
          ctx->center_freq_hz,
          ctx->sample_rate_hz,
          (unsigned long long)ctx->block_count,
          (unsigned long long)ctx->iq_pair_count,
          (unsigned long long)ctx->byte_count,
          ctx->iq_path,
          ctx->json_path);
  fclose(f);
}

static void close_recording(IqRecorderCtx* ctx) {
  if (!ctx) return;
  if (ctx->iq_file) {
    fflush(ctx->iq_file);
    fclose(ctx->iq_file);
    ctx->iq_file = NULL;
  }
  if (ctx->json_path[0]) write_json(ctx, 1);
  ctx->iq_path[0] = '\0';
  ctx->json_path[0] = '\0';
  ctx->sample_rate_hz = 0;
  ctx->center_freq_hz = 0.0;
  ctx->first_unix_ms = 0;
  ctx->last_unix_ms = 0;
  ctx->block_count = 0;
  ctx->iq_pair_count = 0;
  ctx->byte_count = 0;
}

static int start_recording(IqRecorderCtx* ctx,
                           uint32_t sample_rate_hz,
                           double center_freq_hz,
                           uint64_t unix_ms) {
  char ts[32];
  unsigned long long freq_hz;

  if (!ctx || !ctx->enabled || ctx->write_failed) return 0;
  close_recording(ctx);
  if (!mkdir_p(ctx->output_dir)) {
    ctx->write_failed = 1;
    return 0;
  }

  if (unix_ms == 0) unix_ms = (uint64_t)time(NULL) * 1000ull;
  format_utc(unix_ms, ts, sizeof(ts));
  freq_hz = (center_freq_hz > 0.0) ? (unsigned long long)(center_freq_hz + 0.5) : 0ull;

  snprintf(ctx->iq_path, sizeof(ctx->iq_path), "%s/%s_%s_%llu_%u_%llu.iq16",
           ctx->output_dir, ctx->prefix, ts, freq_hz, sample_rate_hz,
           (unsigned long long)ctx->file_index);
  snprintf(ctx->json_path, sizeof(ctx->json_path), "%s/%s_%s_%llu_%u_%llu.json",
           ctx->output_dir, ctx->prefix, ts, freq_hz, sample_rate_hz,
           (unsigned long long)ctx->file_index);
  ctx->file_index++;

  ctx->iq_file = fopen(ctx->iq_path, "wb");
  if (!ctx->iq_file) {
    ctx->write_failed = 1;
    ctx->iq_path[0] = '\0';
    ctx->json_path[0] = '\0';
    return 0;
  }

  ctx->sample_rate_hz = sample_rate_hz;
  ctx->center_freq_hz = center_freq_hz;
  ctx->first_unix_ms = unix_ms;
  ctx->last_unix_ms = unix_ms;
  ctx->block_count = 0;
  ctx->iq_pair_count = 0;
  ctx->byte_count = 0;
  write_json(ctx, 0);
  return 1;
}

MrPluginCtx* mr_plugin_create(void) {
  IqRecorderCtx* ctx = (IqRecorderCtx*)calloc(1, sizeof(IqRecorderCtx));
  const char* dir;
  const char* prefix;
  if (!ctx) return NULL;

  ctx->enabled = !env_disabled(getenv("MR_IQ_RECORD_ENABLED"));
  dir = getenv("MR_IQ_RECORD_DIR");
  prefix = getenv("MR_IQ_RECORD_PREFIX");
  copy_string(ctx->output_dir, sizeof(ctx->output_dir), dir, MR_IQ_REC_DEFAULT_DIR);
  copy_string(ctx->prefix, sizeof(ctx->prefix), prefix, MR_IQ_REC_DEFAULT_PREFIX);
  sanitize_token(ctx->prefix);
  ctx->max_bytes = parse_u64_env("MR_IQ_RECORD_MAX_BYTES", MR_IQ_REC_DEFAULT_MAX_BYTES);
  return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
  IqRecorderCtx* ctx = (IqRecorderCtx*)raw;
  if (!ctx) return;
  close_recording(ctx);
  free(ctx);
}

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
  IqRecorderCtx* ctx = (IqRecorderCtx*)raw;
  if (!ctx || !key || !value) return 0;

  if (strcmp(key, "recording_start") == 0) {
    if (ctx->enabled) {
      ctx->recording_requested = 1;
      ctx->write_failed = 0;
    }
    return 1;
  }
  if (strcmp(key, "recording_stop") == 0) {
    close_recording(ctx);
    ctx->recording_requested = 0;
    return 1;
  }
  if (strcmp(key, "recording_enabled") == 0) {
    const int enabled = !env_disabled(value);
    if (!enabled) {
      close_recording(ctx);
      ctx->recording_requested = 0;
    }
    ctx->enabled = enabled;
    return 1;
  }
  if (strcmp(key, "recording_dir") == 0) {
    close_recording(ctx);
    copy_string(ctx->output_dir, sizeof(ctx->output_dir), value, MR_IQ_REC_DEFAULT_DIR);
    ctx->write_failed = 0;
    return 1;
  }
  if (strcmp(key, "recording_prefix") == 0) {
    close_recording(ctx);
    copy_string(ctx->prefix, sizeof(ctx->prefix), value, MR_IQ_REC_DEFAULT_PREFIX);
    sanitize_token(ctx->prefix);
    return 1;
  }
  if (strcmp(key, "recording_max_bytes") == 0) {
    char* end = NULL;
    unsigned long long parsed = strtoull(value, &end, 10);
    if (end != value) ctx->max_bytes = (uint64_t)parsed;
    return 1;
  }
  return 0;
}

void mr_plugin_process_bits(MrPluginCtx* ctx,
                            const uint8_t* bit_bytes, uint32_t bit_count,
                            double freq_hz, uint64_t unix_ms,
                            const char* source_type,
                            MrEmitFn emit_fn, void* user_data) {
  (void)ctx;
  (void)bit_bytes;
  (void)bit_count;
  (void)freq_hz;
  (void)unix_ms;
  (void)source_type;
  (void)emit_fn;
  (void)user_data;
}

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sample_rate_hz, double center_freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
  IqRecorderCtx* ctx = (IqRecorderCtx*)raw;
  const size_t component_count = (size_t)num_pairs * 2u;
  const size_t byte_count = component_count * sizeof(int16_t);
  size_t written;

  (void)emit_fn;
  (void)user_data;

  if (!ctx || ctx->write_failed || !iq || num_pairs == 0) return;
  if (sample_rate_hz == 0) sample_rate_hz = 2048000u;

  if (!ctx->iq_file) {
    if (!ctx->recording_requested) return;
    if (!start_recording(ctx, sample_rate_hz, center_freq_hz, unix_ms)) return;
    ctx->recording_requested = 0;
  }

  if (ctx->max_bytes > 0 && ctx->byte_count >= ctx->max_bytes) {
    close_recording(ctx);
    return;
  }

  written = fwrite(iq, sizeof(int16_t), component_count, ctx->iq_file);
  if (written != component_count) {
    ctx->write_failed = 1;
    close_recording(ctx);
    return;
  }

  ctx->last_unix_ms = unix_ms;
  ctx->block_count++;
  ctx->iq_pair_count += num_pairs;
  ctx->byte_count += (uint64_t)byte_count;
  write_json(ctx, 0);
}
