/**
 * dsc_decoder.c - Initial DSC bitstream decoder (word-level).
 *
 * Role: MR_PLUGIN_ROLE_DECODER
 *
 * Consumes packed bits (typically from FSK_DATA) and decodes DSC 10-bit words:
 *   - 7 data bits + 3 zero-count check bits
 *   - alignment search over 10 possible bit offsets
 *
 * Emits:
 *   signal_type "DSC"
 *   payload: hex bytes of decoded 7-bit symbols (one symbol per byte)
 *   normalized fields: alignment/validity/metrics/symbols
 */

#include "mr_plugin_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DSC_DEC_MAX_WORDS             256u
#define DSC_DEC_MIN_WORDS             6u
#define DSC_DEC_DEFAULT_VALID_RATIO_PCT 60u

typedef struct {
  int reverse_data_bits;   /* 1: transmitted bit order is reversed in 7-bit symbol */
  uint32_t min_words;
  uint32_t min_valid_ratio_pct;
  int drop_duplicates;

  uint64_t metric_blocks;
  uint64_t metric_candidates;
  uint64_t metric_duplicates;
  uint64_t metric_emitted;
  uint64_t metric_frames_parsed;
  uint64_t metric_ecc_ok;
  uint64_t metric_ecc_fail;

  char last_payload_hex[DSC_DEC_MAX_WORDS * 2u + 1u];
  uint64_t last_unix_ms;
} DscDecCtx;

static const MrPluginMeta kMeta = {
    "dsc_decoder",
    "0.1.0",
    MR_PLUGIN_API_VERSION,
    "DSC decoder: 10-bit word alignment + zero-count check",
    MR_PLUGIN_ROLE_DECODER};

static int get_bit(const uint8_t* bytes, uint32_t idx) {
  return (bytes[idx >> 3u] >> (7u - (idx & 7u))) & 1u;
}

static const char* format_label(uint8_t sym) {
  switch (sym) {
    case 112: return "distress";
    case 116: return "all_ships";
    case 120: return "individual";
    case 114: return "group_common_interest";
    case 102: return "group_geographic";
    case 123: return "semi_auto_auto_service";
    default:  return "unknown";
  }
}

static const char* category_label(uint8_t sym) {
  switch (sym) {
    case 112: return "distress";
    case 110: return "urgency";
    case 108: return "safety";
    case 106: return "ship_business";
    case 100: return "routine";
    default:  return "unknown";
  }
}

static const char* eos_label(uint8_t sym) {
  switch (sym) {
    case 117: return "ack_request";
    case 122: return "ack";
    case 127: return "end_of_sequence";
    default:  return "unknown";
  }
}

static const char* distress_nature_label(uint8_t sym) {
  switch (sym) {
    case 100: return "fire_explosion";
    case 101: return "flooding";
    case 102: return "collision";
    case 103: return "grounding";
    case 104: return "listing_capsize_risk";
    case 105: return "sinking";
    case 106: return "disabled_adrift";
    case 107: return "undesignated";
    case 108: return "abandoning_ship";
    case 109: return "piracy_armed_robbery";
    case 110: return "man_overboard";
    case 112: return "epirb_emission";
    default:  return "unknown";
  }
}

static const char* telecommand1_label(uint8_t sym) {
  switch (sym) {
    case 100: return "f3e_g3e_all_modes_tp";
    case 101: return "f3e_g3e_duplex_tp";
    case 102: return "polling";
    case 104: return "unable_to_comply";
    case 105: return "end_of_call";
    case 106: return "data";
    case 109: return "j3e_tp";
    case 110: return "distress_ack";
    case 112: return "distress_relay";
    case 113: return "f1b_j2b_tty_fec";
    case 115: return "f1b_j2b_tty_arq";
    case 118: return "test";
    case 121: return "ship_position_update";
    case 126: return "no_information";
    default:  return "unknown";
  }
}

static uint8_t decode_symbol7(const uint8_t* bytes, uint32_t bit0, int reverse_data_bits) {
  uint8_t v = 0u;
  for (uint32_t i = 0u; i < 7u; ++i) {
    const uint32_t src = reverse_data_bits ? (bit0 + (6u - i)) : (bit0 + i);
    v = (uint8_t)((v << 1u) | (uint8_t)get_bit(bytes, src));
  }
  return v;
}

static uint8_t zero_count_check(const uint8_t* bytes, uint32_t bit0) {
  uint8_t z = 0u;
  for (uint32_t i = 0u; i < 7u; ++i) {
    if (get_bit(bytes, bit0 + i) == 0) ++z;
  }
  return z;
}

static uint8_t parse_check_bits(const uint8_t* bytes, uint32_t bit0) {
  uint8_t v = 0u;
  for (uint32_t i = 0u; i < 3u; ++i) {
    v = (uint8_t)((v << 1u) | (uint8_t)get_bit(bytes, bit0 + 7u + i));
  }
  return v;
}

static void bytes_to_hex(const uint8_t* data, uint32_t len, char* out_hex, size_t out_cap) {
  if (!out_hex || out_cap == 0u) return;
  if (!data || len == 0u) {
    out_hex[0] = '\0';
    return;
  }
  if (out_cap < (size_t)len * 2u + 1u) {
    out_hex[0] = '\0';
    return;
  }
  for (uint32_t i = 0u; i < len; ++i) snprintf(out_hex + i * 2u, 3u, "%02X", (unsigned)data[i]);
}

static void symbols_to_csv(const uint8_t* symbols, uint32_t n, char* out, size_t out_cap) {
  if (!out || out_cap == 0u) return;
  out[0] = '\0';
  if (!symbols || n == 0u) return;
  for (uint32_t i = 0u; i < n; ++i) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%s%u", (i == 0u ? "" : ","), (unsigned)symbols[i]);
    strncat(out, tmp, out_cap - strlen(out) - 1u);
  }
}

static int mmsi_from_symbol_pairs(const uint8_t* symbols, uint32_t n, char* out, size_t out_cap) {
  if (!symbols || n < 5u || !out || out_cap < 11u) return 0;
  for (uint32_t i = 0u; i < 5u; ++i) {
    if (symbols[i] > 99u) return 0;
  }
  snprintf(out, out_cap, "%02u%02u%02u%02u%02u",
           (unsigned)symbols[0],
           (unsigned)symbols[1],
           (unsigned)symbols[2],
           (unsigned)symbols[3],
           (unsigned)symbols[4]);
  return 1;
}

static int parse_utc_hhmm(const uint8_t* symbols, uint32_t n, char* out, size_t out_cap) {
  if (!symbols || n < 2u || !out || out_cap < 6u) return 0;
  if (symbols[0] > 99u || symbols[1] > 99u) return 0;
  snprintf(out, out_cap, "%02u:%02u", (unsigned)symbols[0], (unsigned)symbols[1]);
  return 1;
}

MrPluginCtx* mr_plugin_create(void) {
  DscDecCtx* ctx = (DscDecCtx*)calloc(1, sizeof(DscDecCtx));
  if (!ctx) return NULL;
  ctx->reverse_data_bits = 1;
  ctx->min_words = DSC_DEC_MIN_WORDS;
  ctx->min_valid_ratio_pct = DSC_DEC_DEFAULT_VALID_RATIO_PCT;
  ctx->drop_duplicates = 1;
  return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) { free(raw); }

const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
  DscDecCtx* ctx = (DscDecCtx*)raw;
  if (!ctx || !key || !value) return 0;
  if (strcmp(key, "reverse_data_bits") == 0) {
    ctx->reverse_data_bits = atoi(value) ? 1 : 0;
    return 1;
  }
  if (strcmp(key, "dsc_min_words") == 0) {
    const int v = atoi(value);
    if (v > 0) ctx->min_words = (uint32_t)v;
    return 1;
  }
  if (strcmp(key, "dsc_min_valid_ratio_pct") == 0) {
    const int v = atoi(value);
    if (v > 0 && v <= 100) ctx->min_valid_ratio_pct = (uint32_t)v;
    return 1;
  }
  if (strcmp(key, "dsc_drop_duplicates") == 0) {
    ctx->drop_duplicates = atoi(value) ? 1 : 0;
    return 1;
  }
  return 0;
}

void mr_plugin_process_bits(MrPluginCtx* raw,
                            const uint8_t* bit_bytes, uint32_t bit_count,
                            double freq_hz, uint64_t unix_ms,
                            const char* source_type,
                            MrEmitFn emit_fn, void* user_data) {
  DscDecCtx* ctx = (DscDecCtx*)raw;
  if (!ctx || !bit_bytes || bit_count < 20u) return;

  ctx->metric_blocks++;
  ctx->metric_candidates++;

  uint32_t best_off = 0u;
  uint32_t best_valid = 0u;
  uint32_t best_total = 0u;

  for (uint32_t off = 0u; off < 10u; ++off) {
    const uint32_t words_total = (bit_count > off) ? ((bit_count - off) / 10u) : 0u;
    if (words_total == 0u) continue;
    const uint32_t words_eval = words_total > DSC_DEC_MAX_WORDS ? DSC_DEC_MAX_WORDS : words_total;
    uint32_t valid = 0u;
    for (uint32_t w = 0u; w < words_eval; ++w) {
      const uint32_t bit0 = off + w * 10u;
      const uint8_t z = zero_count_check(bit_bytes, bit0);
      const uint8_t c = parse_check_bits(bit_bytes, bit0);
      if (z == c) ++valid;
    }
    if (valid > best_valid || (valid == best_valid && words_eval > best_total)) {
      best_valid = valid;
      best_total = words_eval;
      best_off = off;
    }
  }

  if (best_total < ctx->min_words) return;
  if (best_valid * 100u < ctx->min_valid_ratio_pct * best_total) return;

  uint32_t words_total = best_total;
  uint8_t symbols[DSC_DEC_MAX_WORDS];
  uint32_t words_valid = 0u;
  uint32_t words_invalid = 0u;
  uint32_t emit_symbols = 0u;

  for (uint32_t w = 0u; w < words_total; ++w) {
    const uint32_t bit0 = best_off + w * 10u;
    const uint8_t z = zero_count_check(bit_bytes, bit0);
    const uint8_t c = parse_check_bits(bit_bytes, bit0);
    const int ok = (z == c);
    if (ok) {
      symbols[emit_symbols++] = decode_symbol7(bit_bytes, bit0, ctx->reverse_data_bits);
      words_valid++;
    } else {
      words_invalid++;
    }
  }

  if (emit_symbols == 0u) return;

  char payload_hex[DSC_DEC_MAX_WORDS * 2u + 1u];
  payload_hex[0] = '\0';
  bytes_to_hex(symbols, emit_symbols, payload_hex, sizeof(payload_hex));
  if (payload_hex[0] == '\0') return;

  if (ctx->drop_duplicates &&
      ctx->last_payload_hex[0] != '\0' &&
      strcmp(payload_hex, ctx->last_payload_hex) == 0 &&
      unix_ms >= ctx->last_unix_ms && (unix_ms - ctx->last_unix_ms) < 2500u) {
    ctx->metric_duplicates++;
    return;
  }

  strncpy(ctx->last_payload_hex, payload_hex, sizeof(ctx->last_payload_hex) - 1u);
  ctx->last_payload_hex[sizeof(ctx->last_payload_hex) - 1u] = '\0';
  ctx->last_unix_ms = unix_ms;

  {
    char symbol_csv[DSC_DEC_MAX_WORDS * 5u + 1u];
    char address_csv[64];
    char selfid_csv[64];
    char address_mmsi[16];
    char selfid_mmsi[16];
    char msg1_csv[128];
    char distress_pos_csv[64];
    char distress_utc[8];
    const uint8_t fmt_sym = symbols[0];
    const int has_format_repeat = (emit_symbols >= 2u && symbols[0] == symbols[1]);
    const uint32_t addr_start = has_format_repeat ? 2u : 1u;
    uint8_t cat_sym = 255u;
    uint8_t tc1_sym = 255u;
    uint8_t tc2_sym = 255u;
    uint8_t distress_nature_sym = 255u;
    uint8_t eos_sym = 255u;
    int eos_idx = -1;
    int ecc_idx = -1;
    uint8_t ecc_sym = 255u;
    int has_address = 0;
    int has_selfid = 0;
    int has_address_mmsi = 0;
    int has_selfid_mmsi = 0;
    int has_distress_utc = 0;
    int has_distress_pos = 0;
    int msg_start_index = -1;
    int has_tc1 = 0;
    int has_tc2 = 0;
    const int is_distress = (fmt_sym == 112u);
    const int is_all_ships = (fmt_sym == 116u);
    const int is_individual = (fmt_sym == 120u);

    symbols_to_csv(symbols, emit_symbols, symbol_csv, sizeof(symbol_csv));
    address_csv[0] = '\0';
    selfid_csv[0] = '\0';
    address_mmsi[0] = '\0';
    selfid_mmsi[0] = '\0';
    msg1_csv[0] = '\0';
    distress_pos_csv[0] = '\0';
    distress_utc[0] = '\0';

    for (uint32_t i = 0u; i < emit_symbols; ++i) {
      const uint8_t v = symbols[emit_symbols - 1u - i];
      if (v == 117u || v == 122u || v == 127u) {
        eos_idx = (int)(emit_symbols - 1u - i);
        eos_sym = v;
        break;
      }
    }
    if (eos_idx > 0) {
      ecc_idx = eos_idx - 1;
      ecc_sym = symbols[(uint32_t)ecc_idx];
      ctx->metric_ecc_ok++;
    } else {
      ctx->metric_ecc_fail++;
    }

    if (is_distress) {
      if (emit_symbols >= addr_start + 5u) {
        const uint8_t* sid = symbols + addr_start;
        has_selfid = 1;
        symbols_to_csv(sid, 5u, selfid_csv, sizeof(selfid_csv));
        has_selfid_mmsi = mmsi_from_symbol_pairs(sid, 5u, selfid_mmsi, sizeof(selfid_mmsi));
      }
      if (emit_symbols > addr_start + 5u) distress_nature_sym = symbols[addr_start + 5u];
      if (emit_symbols >= addr_start + 11u) {
        const uint8_t* pos = symbols + addr_start + 6u;
        has_distress_pos = 1;
        symbols_to_csv(pos, 5u, distress_pos_csv, sizeof(distress_pos_csv));
      }
      if (emit_symbols >= addr_start + 13u) {
        const uint8_t* utc = symbols + addr_start + 11u;
        has_distress_utc = parse_utc_hhmm(utc, 2u, distress_utc, sizeof(distress_utc));
      }
      if (emit_symbols > addr_start + 13u) {
        tc1_sym = symbols[addr_start + 13u];
        has_tc1 = 1;
        msg_start_index = (int)(addr_start + 14u);
      }
      if (msg_start_index >= 0 && eos_idx > msg_start_index) {
        symbols_to_csv(symbols + (uint32_t)msg_start_index,
                       (uint32_t)(eos_idx - msg_start_index),
                       msg1_csv, sizeof(msg1_csv));
      }
    } else {
      if (emit_symbols >= addr_start + 5u) {
        const uint8_t* addr = symbols + addr_start;
        has_address = 1;
        symbols_to_csv(addr, 5u, address_csv, sizeof(address_csv));
        has_address_mmsi = mmsi_from_symbol_pairs(addr, 5u, address_mmsi, sizeof(address_mmsi));
      }
      if (emit_symbols > addr_start + 5u) cat_sym = symbols[addr_start + 5u];
      if (emit_symbols >= addr_start + 11u) {
        const uint8_t* sid = symbols + addr_start + 6u;
        has_selfid = 1;
        symbols_to_csv(sid, 5u, selfid_csv, sizeof(selfid_csv));
        has_selfid_mmsi = mmsi_from_symbol_pairs(sid, 5u, selfid_mmsi, sizeof(selfid_mmsi));
      }
      if (emit_symbols > addr_start + 11u) {
        tc1_sym = symbols[addr_start + 11u];
        has_tc1 = 1;
      }
      if (emit_symbols > addr_start + 12u) {
        tc2_sym = symbols[addr_start + 12u];
        has_tc2 = 1;
      }
      msg_start_index = has_tc2 ? (int)(addr_start + 13u) : (has_tc1 ? (int)(addr_start + 12u) : -1);
      if (msg_start_index >= 0 && eos_idx > msg_start_index) {
        symbols_to_csv(symbols + (uint32_t)msg_start_index,
                       (uint32_t)(eos_idx - msg_start_index),
                       msg1_csv, sizeof(msg1_csv));
      }
    }

    int eos_tail_repeats = 0;
    if (eos_idx >= 0) {
      for (int i = eos_idx; i < (int)emit_symbols; ++i) {
        if (symbols[(uint32_t)i] == eos_sym) eos_tail_repeats++;
        else break;
      }
    }

    char kv[8192];
    snprintf(
        kv, sizeof(kv),
        "{\"source_type\":\"%s\","
        "\"word_offset\":\"%u\","
        "\"words_total\":\"%u\","
        "\"words_valid\":\"%u\","
        "\"words_invalid\":\"%u\","
        "\"symbols_decoded\":\"%u\","
        "\"symbols_csv\":\"%s\","
        "\"format_specifier\":\"%u\","
        "\"format_label\":\"%s\","
        "\"message_class\":\"%s\","
        "\"parse_status\":\"%s\","
        "\"has_format_repeat\":\"%d\","
        "\"address_symbols\":\"%s\","
        "\"address_mmsi\":\"%s\","
        "\"category_symbol\":\"%u\","
        "\"category_label\":\"%s\","
        "\"self_id_symbols\":\"%s\","
        "\"self_id_mmsi\":\"%s\","
        "\"eos_symbol\":\"%u\","
        "\"eos_label\":\"%s\","
        "\"eos_index\":\"%d\","
        "\"eos_tail_repeats\":\"%d\","
        "\"ecc_symbol\":\"%u\","
        "\"ecc_index\":\"%d\","
        "\"telecommand1_symbol\":\"%u\","
        "\"telecommand1_label\":\"%s\","
        "\"telecommand2_symbol\":\"%u\","
        "\"message_symbols\":\"%s\","
        "\"message_start_index\":\"%d\","
        "\"distress_nature_symbol\":\"%u\","
        "\"distress_nature_label\":\"%s\","
        "\"distress_position_symbols\":\"%s\","
        "\"distress_utc\":\"%s\","
        "\"sequence_status\":\"%s\","
        "\"reverse_data_bits\":\"%d\","
        "\"metric_blocks\":\"%llu\","
        "\"metric_candidates\":\"%llu\","
        "\"metric_duplicates\":\"%llu\","
        "\"metric_emitted\":\"%llu\","
        "\"metric_frames_parsed\":\"%llu\","
        "\"metric_ecc_ok\":\"%llu\","
        "\"metric_ecc_fail\":\"%llu\"}",
        source_type ? source_type : "",
        best_off,
        words_total,
        words_valid,
        words_invalid,
        emit_symbols,
        symbol_csv,
        (unsigned)fmt_sym,
        format_label(fmt_sym),
        is_distress ? "distress" : (is_all_ships ? "all_ships" : (is_individual ? "individual" : "selective_other")),
        (eos_idx >= 0 && has_format_repeat) ? "ok" : ((eos_idx >= 0) ? "partial" : "weak"),
        has_format_repeat,
        has_address ? address_csv : "",
        has_address_mmsi ? address_mmsi : "",
        (unsigned)(cat_sym <= 127u ? cat_sym : 0u),
        (cat_sym <= 127u) ? category_label(cat_sym) : "unknown",
        has_selfid ? selfid_csv : "",
        has_selfid_mmsi ? selfid_mmsi : "",
        (unsigned)(eos_sym <= 127u ? eos_sym : 0u),
        (eos_sym <= 127u) ? eos_label(eos_sym) : "unknown",
        eos_idx,
        eos_tail_repeats,
        (unsigned)(ecc_sym <= 127u ? ecc_sym : 0u),
        ecc_idx,
        (unsigned)(has_tc1 ? tc1_sym : 0u),
        has_tc1 ? telecommand1_label(tc1_sym) : "unknown",
        (unsigned)(has_tc2 ? tc2_sym : 0u),
        msg1_csv,
        msg_start_index,
        (unsigned)(distress_nature_sym <= 127u ? distress_nature_sym : 0u),
        (distress_nature_sym <= 127u) ? distress_nature_label(distress_nature_sym) : "unknown",
        has_distress_pos ? distress_pos_csv : "",
        has_distress_utc ? distress_utc : "",
        (eos_idx >= 0) ? "ok" : "partial",
        ctx->reverse_data_bits,
        (unsigned long long)ctx->metric_blocks,
        (unsigned long long)ctx->metric_candidates,
        (unsigned long long)ctx->metric_duplicates,
        (unsigned long long)(ctx->metric_emitted + 1u),
        (unsigned long long)(ctx->metric_frames_parsed + 1u),
        (unsigned long long)ctx->metric_ecc_ok,
        (unsigned long long)ctx->metric_ecc_fail);

    ctx->metric_emitted++;
    ctx->metric_frames_parsed++;
    if (emit_fn) emit_fn("DSC", payload_hex, freq_hz, unix_ms, kv, user_data);
  }
}

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sample_rate_hz,
                          double center_freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
  (void)raw;
  (void)iq;
  (void)num_pairs;
  (void)sample_rate_hz;
  (void)center_freq_hz;
  (void)unix_ms;
  (void)emit_fn;
  (void)user_data;
}
