/**
 * adsb_demod.c — ADS-B / Mode S demodulator
 *
 * Kräver IQ vid 2 Msps (eller 2048 ksps) inställt på 1090 MHz.
 *
 * Algoritm:
 *   1. Beräkna magnituden (I²+Q²) per sampel.
 *   2. Sök efter Mode S-preambel (8 µs = 16 samplar vid 2 Msps):
 *        HÖG: sampelpositioner 0, 2, 7, 9
 *        LÅG: sampelpositioner 1, 3–6, 8, 10–15
 *   3. PPM-avkoda bitar (hög+låg=1, låg+hög=0, 1 µs = 2 samplar/bit).
 *   4. Validera CRC-24.
 *   5. Emittera godkänd ram som "ADSB" med råhex som payload.
 *
 * Normaliserade fält: df, icao, bits
 */

#include "mr_plugin_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------ */
/* Mode S-konstanter                                                    */
/* ------------------------------------------------------------------ */

#define PREAMBLE_SAMPS  16u   /* 8 µs × 2 samplar/µs */
#define BIT_SAMPS        2u   /* 1 µs × 2 samplar/µs */
#define SHORT_BITS      56u
#define LONG_BITS      112u
#define LONG_SAMPS     (LONG_BITS * BIT_SAMPS)          /* 224 */
#define FRAME_SAMPS    (PREAMBLE_SAMPS + LONG_SAMPS)    /* 240 */

#define MODES_POLY 0xFFF409u  /* CRC-24-generatorpolynom för Mode S */

/* ------------------------------------------------------------------ */
/* Kontext                                                              */
/* ------------------------------------------------------------------ */

typedef struct {
  uint32_t* mag;
  uint32_t  mag_cap;
  uint32_t  mag_len;
  uint64_t  preamble_count;
  uint64_t  crc_ok_count;
  uint64_t  crc_ok_corrected_count;
  uint64_t  crc_fail_count;
  uint64_t  df17_ok_count;
  uint64_t  df17_fail_count;
  uint64_t  last_stats_ms;
} AdsbCtx;

/* ------------------------------------------------------------------ */
/* CRC-24                                                               */
/* ------------------------------------------------------------------ */

static uint32_t crc24(const uint8_t* data, uint32_t n) {
  uint32_t crc = 0;
  for (uint32_t i = 0; i < n; ++i) {
    crc ^= (uint32_t)data[i] << 16;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x800000u) ? (crc << 1) ^ MODES_POLY : crc << 1;
      crc &= 0xFFFFFFu;
    }
  }
  return crc;
}

/* ------------------------------------------------------------------ */
/* Preambeldetektion                                                    */
/* ------------------------------------------------------------------ */

/* HIGH vid 0,2,7,9 — LÅG vid 1,3–6,8 och guard-intervall 10–15 */
static int preamble_ok(const uint32_t* m) {
  /* Lokal kontrast: varje hög-puls måste överstiga sina direkta grannar × 2 */
  if (m[0] < m[1] * 2u) return 0;
  if (m[2] < m[1] * 2u || m[2] < m[3] * 2u) return 0;
  if (m[7] < m[6] * 2u || m[7] < m[8] * 2u) return 0;
  if (m[9] < m[8] * 2u || m[9] < m[10] * 2u) return 0;

  /* Pulslikhet: de fyra hög-pulserna ska vara inom 3:1 i effekt */
  uint32_t hi_min = m[0], hi_max = m[0];
  if (m[2] < hi_min) hi_min = m[2]; else if (m[2] > hi_max) hi_max = m[2];
  if (m[7] < hi_min) hi_min = m[7]; else if (m[7] > hi_max) hi_max = m[7];
  if (m[9] < hi_min) hi_min = m[9]; else if (m[9] > hi_max) hi_max = m[9];
  if (hi_min * 3u < hi_max) return 0;

  /* Summa-SNR: summan av 4 höga ≥ summan av 12 låga inkl. guard-intervall */
  uint32_t h = m[0] + m[2] + m[7] + m[9];
  uint32_t l = m[1] + m[3] + m[4] + m[5] + m[6] + m[8]
             + m[10] + m[11] + m[12] + m[13] + m[14] + m[15];
  return h >= l;
}

/* ------------------------------------------------------------------ */
/* Bitavkodning (PPM)                                                   */
/* ------------------------------------------------------------------ */

static int decode_bit(uint32_t s0, uint32_t s1) {
  return (s0 > s1) ? 1 : 0;
}

/* ------------------------------------------------------------------ */
/* Ettbitars-felkorrigering                                             */
/* ------------------------------------------------------------------ */

/* Provar att vända en bit i taget; returnerar bitindex (0-based) om en
 * enda bit-vändning ger korrekt CRC-24, annars -1. */
static int try_fix_single_bit(uint8_t* frame, uint32_t n_bytes) {
  for (uint32_t bit = 0; bit < n_bytes * 8u; ++bit) {
    frame[bit / 8u] ^= (uint8_t)(0x80u >> (bit % 8u));
    uint32_t calc = crc24(frame, n_bytes - 3u);
    uint32_t recv = ((uint32_t)frame[n_bytes - 3u] << 16) |
                    ((uint32_t)frame[n_bytes - 2u] <<  8) |
                     (uint32_t)frame[n_bytes - 1u];
    if (calc == recv) return (int)bit;
    frame[bit / 8u] ^= (uint8_t)(0x80u >> (bit % 8u)); /* återställ */
  }
  return -1;
}

/* ------------------------------------------------------------------ */
/* ICAO 24-bitars adress → land (ICAO Annex 10-tilldelningar)          */
/* ------------------------------------------------------------------ */

typedef struct { uint32_t lo; uint32_t hi; const char* cc; } IcaoBlock;

/* Sorterade block; binärsökning i icao_country() */
static const IcaoBlock kIcaoBlocks[] = {
  {0x004000,0x0043FF,"MZ"},{0x006000,0x006FFF,"ZA"},{0x008000,0x00FFFF,"ZA"},
  {0x010000,0x017FFF,"EG"},{0x018000,0x01FFFF,"LY"},{0x020000,0x027FFF,"MA"},
  {0x028000,0x02FFFF,"TN"},{0x030000,0x0303FF,"BW"},{0x032000,0x032FFF,"BI"},
  {0x034000,0x034FFF,"CM"},{0x038000,0x03FFFF,"CG"},
  {0x040000,0x043FFF,"DZ"},
  {0x060000,0x067FFF,"KE"},{0x068000,0x06FFFF,"NG"},
  {0x100000,0x1FFFFF,"RU"},
  {0x201000,0x2013FF,"NA"},{0x202000,0x2023FF,"ER"},
  {0x300000,0x33FFFF,"IT"},{0x340000,0x37FFFF,"ES"},
  {0x380000,0x3BFFFF,"FR"},{0x3C0000,0x3FFFFF,"DE"},
  {0x400000,0x43FFFF,"GB"},
  {0x440000,0x447FFF,"AT"},{0x448000,0x44FFFF,"BE"},
  {0x450000,0x457FFF,"BG"},{0x458000,0x45FFFF,"DK"},
  {0x460000,0x467FFF,"FI"},{0x468000,0x46FFFF,"GR"},
  {0x470000,0x477FFF,"HU"},{0x478000,0x47FFFF,"IE"},
  {0x480000,0x487FFF,"IT"},{0x488000,0x48FFFF,"LU"},
  {0x490000,0x497FFF,"MT"},{0x498000,0x49FFFF,"MC"},
  {0x4A0000,0x4A7FFF,"NL"},{0x4A8000,0x4AFFFF,"NO"},
  {0x4B0000,0x4B7FFF,"PL"},{0x4B8000,0x4BFFFF,"PT"},
  {0x4C0000,0x4C7FFF,"RO"},{0x4C8000,0x4CFFFF,"CZ"},
  {0x4D0000,0x4D7FFF,"SE"},{0x4D8000,0x4DFFFF,"CH"},
  {0x4E0000,0x4E7FFF,"TR"},{0x4E8000,0x4EFFFF,"RS"},
  {0x500000,0x5003FF,"UA"},{0x501000,0x5013FF,"BY"},
  {0x504000,0x504FFF,"SK"},{0x508000,0x50FFFF,"UA"},
  {0x510000,0x5103FF,"KZ"},{0x511000,0x5113FF,"GE"},
  {0x600000,0x6003FF,"AM"},{0x600800,0x6008FF,"AZ"},
  {0x601000,0x6013FF,"KG"},{0x601800,0x6018FF,"TJ"},
  {0x680000,0x6803FF,"LV"},{0x681000,0x6813FF,"LT"},
  {0x682000,0x6823FF,"EE"},
  {0x700000,0x7003FF,"BA"},{0x701000,0x7013FF,"MK"},
  {0x702000,0x7023FF,"ME"},{0x710000,0x717FFF,"SA"},
  {0x720000,0x72FFFF,"IL"},{0x730000,0x737FFF,"IR"},
  {0x740000,0x747FFF,"PK"},{0x748000,0x74FFFF,"AF"},
  {0x750000,0x757FFF,"KW"},{0x758000,0x75FFFF,"AE"},
  {0x760000,0x767FFF,"IQ"},{0x768000,0x76FFFF,"QA"},
  {0x770000,0x777FFF,"BH"},{0x778000,0x77FFFF,"OM"},
  {0x780000,0x7BFFFF,"CN"},{0x7C0000,0x7FFFFF,"AU"},
  {0x800000,0x83FFFF,"IN"},{0x840000,0x87FFFF,"JP"},
  {0x880000,0x887FFF,"TH"},{0x888000,0x88FFFF,"VN"},
  {0x890000,0x890FFF,"SG"},{0x895000,0x8953FF,"MY"},
  {0x898000,0x898FFF,"PH"},{0x8A0000,0x8AFFFF,"KR"},
  {0x900000,0x9003FF,"NZ"},
  {0xA00000,0xAFFFFF,"US"},
  {0xB00000,0xB03FFF,"MX"},
  {0xC00000,0xC3FFFF,"CA"},
  {0xC80000,0xC83FFF,"AR"},{0xC84000,0xC87FFF,"BO"},
  {0xC88000,0xC8BFFF,"BR"},{0xC8C000,0xC8FFFF,"CL"},
  {0xC90000,0xC9001F,"CO"},
  {0xE80000,0xE80FFF,"LK"},{0xE84000,0xE84FFF,"KH"},
  {0xE8C000,0xE8CFFF,"BD"},
};

#define ICAO_NBLOCKS ((uint32_t)(sizeof(kIcaoBlocks)/sizeof(kIcaoBlocks[0])))

static const char* icao_country(uint32_t icao) {
  uint32_t lo = 0, hi = ICAO_NBLOCKS - 1u;
  while (lo <= hi) {
    uint32_t mid = (lo + hi) >> 1u;
    if      (icao < kIcaoBlocks[mid].lo) { if (mid == 0) break; hi = mid - 1u; }
    else if (icao > kIcaoBlocks[mid].hi) lo = mid + 1u;
    else return kIcaoBlocks[mid].cc;
  }
  return "??";
}

/* ------------------------------------------------------------------ */
/* ME-fältavkodning + ramemission                                       */
/* ------------------------------------------------------------------ */

/* ADS-B 6-bitars teckentabell (ICAO Annex 10) */
static const char kAdsbCS[64] =
    " ABCDEFGHIJKLMNOPQRSTUVWXYZ     "   /* index  0-31 */
    "                0123456789      ";  /* index 32-63 */

/* Extrahera len bitar med start (MSB-first) ur ME-fält */
static uint32_t me_bits(const uint8_t* me, uint32_t start, uint32_t len) {
  uint32_t val = 0;
  for (uint32_t i = 0; i < len; ++i) {
    uint32_t idx = start + i;
    val = (val << 1u) | ((me[idx >> 3u] >> (7u - (idx & 7u))) & 1u);
  }
  return val;
}

/* AC13-fält (DF4/DF20): bits 15-27 i 56/112-bitarsram */
static uint32_t extract_ac13(const uint8_t* f) {
  return ((f[1] & 0x01u) << 12) | ((uint32_t)f[2] << 4) | ((f[3] >> 4) & 0x0Fu);
}

/* Avkoda 13-bitars höjdkod (returnerar 1 vid lyckad avkodning) */
static int decode_ac13(uint32_t ac, int* alt_ft) {
  if ((ac >> 6u) & 1u) return 0;         /* M-bit = metrisk, ej hanterat */
  if (!((ac >> 4u) & 1u)) return 0;      /* Q-bit = 0 → Gillham, ej hanterat */
  int n = (int)(((ac & 0x1F80u) >> 2u) | ((ac & 0x0020u) >> 1u) | (ac & 0x000Fu));
  *alt_ft = n * 25 - 1200;
  return 1;
}

/* Avkoda 13-bitars squawk (Mode C identitetskod) → 4 oktala siffror */
static uint32_t decode_squawk(uint32_t id) {
  uint32_t c1=(id>>12)&1, a1=(id>>11)&1, c2=(id>>10)&1, a2=(id>>9)&1;
  uint32_t c4=(id>>8)&1,  a4=(id>>7)&1,  b1=(id>>6)&1,  d1=(id>>5)&1;
  uint32_t b2=(id>>4)&1,  d2=(id>>3)&1,  b4=(id>>2)&1,  d4=(id>>1)&1;
  return (a4*4+a2*2+a1)*1000 + (b4*4+b2*2+b1)*100
       + (c4*4+c2*2+c1)*10   + (d4*4+d2*2+d1);
}

static const char* df_name(uint32_t df) {
  switch (df) {
    case  0: return "ACAS";
    case  4: return "SurveillanceAlt";
    case  5: return "SurveillanceId";
    case 11: return "AllCall";
    case 16: return "ACAS-Long";
    case 17: return "ADS-B";
    case 18: return "ADS-B/NT";
    case 19: return "ADS-B/Mil";
    case 20: return "CommB-Alt";
    case 21: return "CommB-Id";
    case 24: return "CommD";
    default: return "Mode-S";
  }
}

static const char* tc_type(uint32_t tc) {
  if (tc == 0)              return "no-info";
  if (tc >= 5u && tc <= 8u) return "surface-pos";
  if (tc == 28u)            return "aircraft-status";
  if (tc == 29u)            return "target-state";
  if (tc == 31u)            return "op-status";
  return "reserved";
}

/* emit_frame bygger:
 *   payload  — läsbar text som syns i host-utskriften
 *   kv JSON  — maskinavläsbar, inkl. råhex och avkodade fält              */
static void emit_frame(const uint8_t* frame, uint32_t n_bytes,
                       double freq_hz, uint64_t unix_ms,
                       MrEmitFn emit_fn, void* user_data) {
  /* Råhex — läggs i kv */
  char* raw = (char*)malloc(n_bytes * 2u + 1u);
  if (!raw) return;
  for (uint32_t i = 0; i < n_bytes; ++i)
    snprintf(raw + i * 2u, 3, "%02X", (unsigned)frame[i]);

  uint32_t df   = (frame[0] >> 3) & 0x1Fu;
  uint32_t icao = ((uint32_t)frame[1] << 16) |
                  ((uint32_t)frame[2] <<  8) |
                   (uint32_t)frame[3];

  char text[256];   /* läsbar payload — visas av hosten */
  char kv[512];     /* JSON — maskinavläsbar             */

  const char* cc = icao_country(icao);
  int kpos = snprintf(kv, sizeof(kv),
                      "{\"df\":\"%u\",\"icao\":\"%06X\",\"country\":\"%s\","
                      "\"bits\":\"%u\",\"raw\":\"%s\"",
                      df, icao, cc, n_bytes * 8u, raw);
  int tpos = snprintf(text, sizeof(text), "[%s] ", cc);

  if ((df == 17u || df == 18u) && n_bytes == 14u) {
    const uint8_t* me = frame + 4;
    uint32_t tc = me_bits(me, 0u, 5u);
    kpos += snprintf(kv + kpos, sizeof(kv) - (size_t)kpos, ",\"tc\":\"%u\"", tc);

    if (tc >= 1u && tc <= 4u) {
      /* ---- Identifiering (anropssignal) ---- */
      char cs[9];
      for (int i = 0; i < 8; ++i)
        cs[i] = kAdsbCS[me_bits(me, 8u + (uint32_t)i * 6u, 6u) & 0x3Fu];
      int end = 7;
      while (end > 0 && cs[end] == ' ') --end;
      cs[end + 1] = '\0';
      kpos += snprintf(kv + kpos, sizeof(kv) - (size_t)kpos,
                       ",\"callsign\":\"%s\"", cs);
      tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                       "Anrop: %s", cs);

    } else if (tc >= 9u && tc <= 18u) {
      /* ---- Luftläge (barometrisk höjd) ---- */
      uint32_t alt12 = me_bits(me, 8u, 12u);
      uint32_t F     = me_bits(me, 21u, 1u);
      uint32_t lat17 = me_bits(me, 22u, 17u);
      uint32_t lon17 = me_bits(me, 39u, 17u);
      tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos, "Luftläge:");
      if ((alt12 >> 4u) & 1u) {
        int32_t N   = (int32_t)(((alt12 & 0xFE0u) >> 1u) | (alt12 & 0xFu));
        int32_t alt = N * 25 - 1000;
        kpos += snprintf(kv + kpos, sizeof(kv) - (size_t)kpos,
                         ",\"alt_ft\":\"%d\"", alt);
        tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                         " %d ft,", alt);
      }
      kpos += snprintf(kv + kpos, sizeof(kv) - (size_t)kpos,
                       ",\"cpr_odd\":\"%u\",\"cpr_lat\":\"%u\",\"cpr_lon\":\"%u\"",
                       F, lat17, lon17);
      tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                       " CPR %s lat %u lon %u",
                       F ? "udda" : "j\xc3\xa4mn", lat17, lon17);

    } else if (tc >= 20u && tc <= 22u) {
      /* ---- Luftläge (GNSS-höjd) ---- */
      uint32_t F     = me_bits(me, 21u, 1u);
      uint32_t lat17 = me_bits(me, 22u, 17u);
      uint32_t lon17 = me_bits(me, 39u, 17u);
      kpos += snprintf(kv + kpos, sizeof(kv) - (size_t)kpos,
                       ",\"cpr_odd\":\"%u\",\"cpr_lat\":\"%u\",\"cpr_lon\":\"%u\"",
                       F, lat17, lon17);
      tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                       "Luftläge(GNSS): CPR %s lat %u lon %u",
                       F ? "udda" : "j\xc3\xa4mn", lat17, lon17);

    } else if (tc == 19u) {
      /* ---- Hastighet ---- */
      uint32_t st = me_bits(me, 5u, 3u);
      if (st == 1u || st == 2u) {
        uint32_t dew   = me_bits(me, 13u, 1u);
        uint32_t vew_r = me_bits(me, 14u, 10u);
        uint32_t dns   = me_bits(me, 24u, 1u);
        uint32_t vns_r = me_bits(me, 25u, 10u);
        uint32_t vrsgn = me_bits(me, 36u, 1u);
        uint32_t vr_r  = me_bits(me, 37u, 9u);
        double   mult  = (st == 2u) ? 4.0 : 1.0;
        tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos, "Hastighet:");
        if (vew_r > 0u && vns_r > 0u) {
          double vew = (double)(vew_r - 1u) * mult * (dew ? -1.0 :  1.0);
          double vns = (double)(vns_r - 1u) * mult * (dns ? -1.0 :  1.0);
          double spd = sqrt(vew * vew + vns * vns);
          double hdg = atan2(vew, vns) * (180.0 / M_PI);
          if (hdg < 0.0) hdg += 360.0;
          kpos += snprintf(kv + kpos, sizeof(kv) - (size_t)kpos,
                           ",\"spd_kt\":\"%.0f\",\"hdg_deg\":\"%.1f\"", spd, hdg);
          tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                           " %.0f kt, kurs %.0f\xc2\xb0", spd, hdg);
        }
        if (vr_r > 0u) {
          int32_t vrate = (int32_t)(vr_r - 1u) * 64 * (vrsgn ? -1 : 1);
          kpos += snprintf(kv + kpos, sizeof(kv) - (size_t)kpos,
                           ",\"vrate_fpm\":\"%d\"", vrate);
          tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                           ", stig %+d fpm", vrate);
        }
      } else if (st == 3u || st == 4u) {
        uint32_t hdg_ok = me_bits(me, 13u, 1u);
        uint32_t hdg_r  = me_bits(me, 14u, 10u);
        uint32_t is_tas = me_bits(me, 24u, 1u);
        uint32_t as_r   = me_bits(me, 25u, 10u);
        uint32_t vrsgn  = me_bits(me, 36u, 1u);
        uint32_t vr_r   = me_bits(me, 37u, 9u);
        double   mult   = (st == 4u) ? 4.0 : 1.0;
        tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos, "Hastighet:");
        if (hdg_ok) {
          double hdg = hdg_r * (360.0 / 1024.0);
          kpos += snprintf(kv + kpos, sizeof(kv) - (size_t)kpos,
                           ",\"hdg_deg\":\"%.1f\"", hdg);
          tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                           " kurs %.0f\xc2\xb0,", hdg);
        }
        if (as_r > 0u) {
          int spd = (int)((as_r - 1u) * mult);
          kpos += snprintf(kv + kpos, sizeof(kv) - (size_t)kpos,
                           ",\"%s\":\"%d\"", is_tas ? "tas_kt" : "ias_kt", spd);
          tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                           " %s %d kt", is_tas ? "TAS" : "IAS", spd);
        }
        if (vr_r > 0u) {
          int32_t vrate = (int32_t)(vr_r - 1u) * 64 * (vrsgn ? -1 : 1);
          kpos += snprintf(kv + kpos, sizeof(kv) - (size_t)kpos,
                           ",\"vrate_fpm\":\"%d\"", vrate);
          tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                           ", stig %+d fpm", vrate);
        }
      }
    } else {
      /* TC utan specifik avkodning */
      tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                       "TC%u (%s)", tc, tc_type(tc));
    }
  } else {
    /* Ej DF17/18 — avkoda standardformat, visa "DF-n okänt" för övriga */
    switch (df) {

      case 0:  /* Short Air-Air Surveillance (ACAS) */
        tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos, "ACAS");
        break;

      case 4:   /* Surveillance Altitude Reply */
      case 20: { /* Comm-B Altitude Reply */
        uint32_t ac = extract_ac13(frame);
        int alt; int ok = decode_ac13(ac, &alt);
        if (ok)
          tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                           "%s: %d ft",
                           df == 4 ? "H\xc3\xb6jdssvar" : "CommB-h\xc3\xb6jd", alt);
        else
          tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                           "%s (h\xc3\xb6jd ej avkodbar)",
                           df == 4 ? "H\xc3\xb6jdssvar" : "CommB-h\xc3\xb6jd");
        break;
      }

      case 5:   /* Surveillance Identity Reply */
      case 21: { /* Comm-B Identity Reply */
        uint32_t id = extract_ac13(frame);  /* ID13 på samma position som AC13 */
        uint32_t sq = decode_squawk(id);
        tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                         "%s: squawk %04u",
                         df == 5 ? "Identitetssvar" : "CommB-identitet", sq);
        break;
      }

      case 11: { /* All-Call Reply */
        static const char* ca_desc[] = {
          "nivå 1", "reserverad", "reserverad", "reserverad",
          "ACAS (mark)", "ACAS (luft)", "ACAS (ok\xc3\xa4nd)", "full Mode-S"
        };
        uint32_t ca = frame[0] & 0x07u;
        tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                         "Rollkall: CA=%u (%s)", ca, ca_desc[ca]);
        break;
      }

      case 16: /* Long Air-Air Surveillance (ACAS) */
        tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos, "ACAS-Long");
        break;

      case 19: /* Military Extended Squitter */
        tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos, "ADS-B/Milit\xc3\xa4r");
        break;

      case 24: /* Comm-D Extended Length Message */
        tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos, "CommD (ELM)");
        break;

      default:
        tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos,
                         "Mode-S (DF%u, reserverad/ok\xc3\xa4nd)", df);
        break;
    }
  }

  snprintf(kv + kpos, sizeof(kv) - (size_t)kpos, "}");
  emit_fn("ADSB", text, freq_hz, unix_ms, kv, user_data);
  free(raw);
}

/* ================================================================== */
/* Plugin-gränssnitt                                                    */
/* ================================================================== */

MrPluginCtx* mr_plugin_create(void) {
  AdsbCtx* ctx = (AdsbCtx*)calloc(1, sizeof(AdsbCtx));
  if (!ctx) return NULL;
  ctx->mag_cap = FRAME_SAMPS + 65536u;
  ctx->mag = (uint32_t*)malloc(ctx->mag_cap * sizeof(uint32_t));
  if (!ctx->mag) { free(ctx); return NULL; }
  return (MrPluginCtx*)ctx;
}

void mr_plugin_destroy(MrPluginCtx* raw) {
  if (!raw) return;
  AdsbCtx* ctx = (AdsbCtx*)raw;
  free(ctx->mag);
  free(ctx);
}

static const MrPluginMeta kMeta = {
  "adsb_demod", "1.0.0", MR_PLUGIN_API_VERSION,
  "ADS-B / Mode S-demodulator (2 Msps, CRC-24)",
  MR_PLUGIN_ROLE_DEMODULATOR
};
const MrPluginMeta* mr_plugin_get_meta(void) { return &kMeta; }

int mr_plugin_set_param(MrPluginCtx* raw, const char* key, const char* value) {
  (void)raw; (void)key; (void)value;
  return 0;
}

void mr_plugin_process_iq(MrPluginCtx* raw,
                          const int16_t* iq, uint32_t num_pairs,
                          uint32_t sr, double freq_hz, uint64_t unix_ms,
                          MrEmitFn emit_fn, void* user_data) {
  if (!raw || !iq || !num_pairs || !emit_fn) return;
  AdsbCtx* ctx = (AdsbCtx*)raw;

  /* Kräver ~2 Msps (acceptera 1.8–2.2 Msps) */
  if (sr < 1800000u || sr > 2200000u) return;

  /* Utöka buffert vid behov */
  const uint32_t needed = ctx->mag_len + num_pairs;
  if (needed > ctx->mag_cap) {
    uint32_t new_cap = needed + 65536u;
    uint32_t* nb = (uint32_t*)realloc(ctx->mag, new_cap * sizeof(uint32_t));
    if (!nb) { ctx->mag_len = 0u; return; }
    ctx->mag = nb;
    ctx->mag_cap = new_cap;
  }

  /* IQ → magnitud (I²+Q²), undviker heltalsspill via int32_t */
  for (uint32_t n = 0; n < num_pairs; ++n) {
    int32_t i = (int32_t)iq[n * 2u];
    int32_t q = (int32_t)iq[n * 2u + 1u];
    ctx->mag[ctx->mag_len++] = (uint32_t)(i * i) + (uint32_t)(q * q);
  }

  /* Samplar per bit ur faktisk sample rate */
  float spb = (float)sr * 1e-6f;

  /* Vid sr > 2 Msps sträcker sig puls 1 (0–0.5 µs) över sample 1 (t=0.488 µs < 0.5 µs),
   * vilket gör att preamble_ok matchar 1 sampel sent: p = P_true + 1.
   * chip_off kompenserar för att datapekaren (p + PREAMBLE_SAMPS) hamnar
   * (PREAMBLE_SAMPS + det_off) − 8×spb samplar efter den verkliga datastarten. */
  uint32_t det_off  = (sr > 2000000u) ? 1u : 0u;
  float    chip_off = (float)(PREAMBLE_SAMPS + det_off) - 8.0f * spb;

  /* Chip-sampling: s0 = första sampel i chip 1, s1 = första sampel i chip 2.
   * Använder ceil(base) = floor(base + 0.99999) för att undvika <math.h>. */
#define CHIP_S0(b) ({ float _b = (float)(b) * spb - chip_off; \
                      (_b > 0.0f) ? (uint32_t)(_b + 0.99999f) : 0u; })
#define CHIP_S1(b) ({ float _b = (float)(b) * spb - chip_off + spb * 0.5f; \
                      (_b > 0.0f) ? (uint32_t)(_b + 0.99999f) : 0u; })

  uint32_t min_frame = PREAMBLE_SAMPS + (uint32_t)((float)LONG_BITS * spb) + 2u;

  /* Sök efter preamblar och avkoda ramar */
  uint32_t p = 0;
  while (p + min_frame <= ctx->mag_len) {
    if (!preamble_ok(ctx->mag + p)) { ++p; continue; }
    ++ctx->preamble_count;

    const uint32_t* data = ctx->mag + p + PREAMBLE_SAMPS;

    /* DF-fält (5 bitar) */
    uint32_t df5 = 0;
    for (int b = 0; b < 5; ++b)
      df5 = (df5 << 1) | (uint32_t)decode_bit(data[CHIP_S0(b)], data[CHIP_S1(b)]);

    uint32_t n_bits  = (df5 >= 16u) ? LONG_BITS : SHORT_BITS;
    uint32_t n_samps = PREAMBLE_SAMPS + (uint32_t)((float)n_bits * spb) + 2u;

    /* Vänta om vi inte har tillräckligt med samplar för hela ramen */
    if (p + n_samps > ctx->mag_len) break;

    /* Avkoda alla bitar */
    uint8_t frame[LONG_BITS / 8u];
    memset(frame, 0, sizeof(frame));
    for (uint32_t bit = 0; bit < n_bits; ++bit) {
      if (decode_bit(data[CHIP_S0(bit)], data[CHIP_S1(bit)]))
        frame[bit / 8u] |= (uint8_t)(0x80u >> (bit % 8u));
    }

    uint32_t n_bytes = n_bits / 8u;

    /* CRC-24-validering */
    uint32_t crc_calc = crc24(frame, n_bytes - 3u);
    uint32_t crc_fram = ((uint32_t)frame[n_bytes - 3u] << 16) |
                        ((uint32_t)frame[n_bytes - 2u] <<  8) |
                         (uint32_t)frame[n_bytes - 1u];

    uint32_t df = (frame[0] >> 3) & 0x1Fu;
    if (crc_calc == crc_fram) {
      ++ctx->crc_ok_count;
      if (df == 17u) ++ctx->df17_ok_count;
      emit_frame(frame, n_bytes, freq_hz, unix_ms, emit_fn, user_data);
      p += n_samps - 2u;
    } else {
      /* Försök rätta ett enskilt bitfel */
      int fixed_bit = try_fix_single_bit(frame, n_bytes);
      if (fixed_bit >= 0) {
        ++ctx->crc_ok_corrected_count;
        ++ctx->crc_ok_count;
        df = (frame[0] >> 3) & 0x1Fu;
        if (df == 17u) ++ctx->df17_ok_count;
        emit_frame(frame, n_bytes, freq_hz, unix_ms, emit_fn, user_data);
        p += n_samps - 2u;
      } else {
        if (df == 17u) {
          if (ctx->df17_fail_count < 5u) {
            fprintf(stderr, "[ADS-B DBG] DF17 #%llu sr=%u spb=%.3f: ",
                    (unsigned long long)ctx->df17_fail_count + 1u, sr, (double)spb);
            for (uint32_t x = 0; x < n_bytes; ++x)
              fprintf(stderr, "%02X", frame[x]);
            fprintf(stderr, "  calc=%06X rx=%06X\n", crc_calc, crc_fram);
          }
          ++ctx->df17_fail_count;
        }
        ++ctx->crc_fail_count;
        ++p;
      }
    }
  }

  /* Skriv ut statistik var 10:e sekund */
  if (unix_ms - ctx->last_stats_ms >= 10000u) {
    ctx->last_stats_ms = unix_ms;
    fprintf(stderr,
            "[ADS-B] preamble=%llu  crc_ok=%llu(+%llu korr)  crc_fail=%llu"
            "  df17_ok=%llu  df17_fail=%llu\n",
            (unsigned long long)ctx->preamble_count,
            (unsigned long long)ctx->crc_ok_count,
            (unsigned long long)ctx->crc_ok_corrected_count,
            (unsigned long long)ctx->crc_fail_count,
            (unsigned long long)ctx->df17_ok_count,
            (unsigned long long)ctx->df17_fail_count);
  }

  /* Behåll obearbetade samplar för nästa block */
  uint32_t keep = (p < ctx->mag_len) ? (ctx->mag_len - p) : 0u;
  if (p > 0u && keep > 0u)
    memmove(ctx->mag, ctx->mag + p, keep * sizeof(uint32_t));
  ctx->mag_len = keep;
}
