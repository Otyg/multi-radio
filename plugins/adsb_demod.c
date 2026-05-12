/**
 * adsb_demod.c — ADS-B / Mode S demodulator
 *
 * Anpassad från dump1090 (https://github.com/antirez/dump1090)
 * för kompatibilitet med multi-radio plugin-struktur.
 *
 * Kräver IQ vid 2 Msps (eller 2048 ksps) inställt på 1090 MHz.
 *
 * Algoritm:
 *   1. Beräkna magnituden (I²+Q²) per sampel.
 *   2. Sök efter Mode S-preambel med fas-korrigering.
 *   3. PPM-avkoda bitar med felkorrigering.
 *   4. Validera CRC-24 med felkorrigeringstabell.
 *   5. Avkoda meddelandetyper (DF17, etc.).
 *   6. Emittera godkänd ram som "ADSB" med råhex som payload.
 *
 * Normaliserade fält: df, icao, bits, crc_ok, errorbit
 */

#include "mr_plugin_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------ */
/* Mode S-konstanter från dump1090                                     */
/* ------------------------------------------------------------------ */

#define MODES_DEFAULT_RATE         2000000
#define MODES_PREAMBLE_US 8       /* microseconds */
#define MODES_LONG_MSG_BITS 112
#define MODES_SHORT_MSG_BITS 56
#define PREAMBLE_SAMPS  16u      /* 8 µs × 2 samplar/µs */
#define BIT_SAMPS        2u      /* 1 µs × 2 samplar/µs */
#define SHORT_BITS      56u
#define LONG_BITS      112u
#define LONG_SAMPS     (LONG_BITS * BIT_SAMPS)
#define FRAME_SAMPS    (PREAMBLE_SAMPS + LONG_SAMPS)
#define MODES_FULL_LEN (MODES_PREAMBLE_US+MODES_LONG_MSG_BITS)
#define MODES_LONG_MSG_BYTES (112/8)
#define MODES_SHORT_MSG_BYTES (56/8)
#define MODES_MAX_BITERRORS 2     /* Maximum correctable bit errors. */

/* Number of entries in error syndrome table:
 * Single bit errors: 107 (bits 5-111)
 * Two-bit errors: 107*106/2 = 5671
 * Total: 5778 */
#define NERRORINFO 5778
#define MODES_ICAO_CACHE_LEN 1024
#define MODES_ICAO_CACHE_TTL 60

/* ------------------------------------------------------------------ */
/* Datastrukturer från dump1090                                       */
/* ------------------------------------------------------------------ */

/* Structure for error syndrome lookup table entries. */
struct errorinfo {
    uint32_t syndrome;          /* CRC syndrome for this error pattern. */
    int bits;                   /* Number of bit errors (1 or 2). */
    int pos[MODES_MAX_BITERRORS]; /* Bit positions to correct. */
};

/* Error syndrome lookup table, sorted by syndrome for binary search. */
static struct errorinfo bitErrorTable[NERRORINFO];

/* The struct we use to store information about a decoded message. */
struct modesMessage {
    /* Generic fields */
    unsigned char msg[MODES_LONG_MSG_BYTES]; /* Binary message. */
    int msgbits;                /* Number of bits in message */
    int msgtype;                /* Downlink format # */
    int crcok;                  /* True if CRC was valid */
    uint32_t crc;               /* Message CRC */
    int errorbit;               /* Bit corrected. -1 if no bit corrected. */
    int aa1, aa2, aa3;          /* ICAO Address bytes 1 2 and 3 */
    int phase_corrected;        /* True if phase correction was applied. */

    /* DF 17, 18 */
    int metype;                 /* Extended squitter message type. */
    int mesub;                  /* Extended squitter message subtype. */
    int fflag;                  /* 1 = Odd, 0 = Even CPR message. */
    int raw_latitude;           /* Non decoded latitude */
    int raw_longitude;          /* Non decoded longitude */
    char flight[9];             /* 8 chars flight number. */
    int ew_dir;                 /* 0 = East, 1 = West. */
    int ew_velocity;            /* E/W velocity. */
    int ns_dir;                 /* 0 = North, 1 = South. */
    int ns_velocity;            /* N/S velocity. */
    int vert_rate_source;       /* Vertical rate source. */
    int vert_rate_sign;         /* Vertical rate sign. */
    int vert_rate;              /* Vertical rate. */
    int velocity;               /* Computed from EW and NS velocity. */

    /* DF4, DF5, DF20, DF21 */
    int fs;                     /* Flight status for DF4,5,20,21 */
    int dr;                     /* Request extraction of downlink request. */
    int um;                     /* Request extraction of downlink request. */
    int identity;               /* 13 bits identity (Squawk). */

    /* Fields used by multiple message types. */
    int altitude, unit;
};

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
  uint64_t  crc_ok_ap_count;
  uint64_t  crc_fail_count;
  uint64_t  df17_ok_count;
  uint64_t  df17_fail_count;
  uint64_t  df18_ok_count;
  uint64_t  df18_fail_count;
  uint64_t  last_stats_ms;
  uint32_t icao_cache[MODES_ICAO_CACHE_LEN * 2];
  int      error_info_initialized;
} AdsbCtx;

/* ------------------------------------------------------------------ */
/* CRC-24 från dump1090                                                 */
/* ------------------------------------------------------------------ */

/* Parity table for MODE S Messages.
 * The table contains 112 elements, every element corresponds to a bit set
 * in the message, starting from the first bit of actual data after the
 * preamble.
 *
 * For messages of 112 bit, the whole table is used.
 * For messages of 56 bits only the last 56 elements are used.
 */
static uint32_t modes_checksum_table[112] = {
0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178,
0x2c38bc, 0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14,
0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449,
0x939020, 0x49c810, 0x24e408, 0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22,
0x3f6d11, 0xe04c8c, 0x702646, 0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7,
0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4, 0x2bfd53, 0xea04ad, 0x8af852,
0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441, 0xf91024, 0x7c8812,
0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80, 0x0706c0, 0x038360,
0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000
};

/* Compute the CRC syndrome of a Mode S message. */
static uint32_t modesChecksum(unsigned char *msg, int bits) {
    uint32_t crc = 0;
    int offset = (bits == 112) ? 0 : (112-56);
    int j;

    for(j = 0; j < bits-24; j++) {
        int byte = j/8;
        int bit = j%8;
        int bitmask = 1 << (7-bit);
        if (msg[byte] & bitmask)
            crc ^= modes_checksum_table[j+offset];
    }

    uint32_t rem = ((uint32_t)msg[(bits/8)-3] << 16) |
          ((uint32_t)msg[(bits/8)-2] << 8) |
           (uint32_t)msg[(bits/8)-1];
    return (crc ^ rem) & 0x00FFFFFF;
}

/* Given the Downlink Format (DF) of the message, return the message length in bits. */
static int modesMessageLenByType(int type) {
    if (type == 16 || type == 17 || type == 18 || type == 19 ||
        type == 20 || type == 21)
        return MODES_LONG_MSG_BITS;
    else
        return MODES_SHORT_MSG_BITS;
}

/* Comparison function for qsort/bsearch on errorinfo by syndrome. */
static int cmpErrorInfo(const void *a, const void *b) {
    const struct errorinfo *ea = (const struct errorinfo *)a;
    const struct errorinfo *eb = (const struct errorinfo *)b;
    if (ea->syndrome < eb->syndrome) return -1;
    if (ea->syndrome > eb->syndrome) return 1;
    return 0;
}

static uint32_t crc24(const uint8_t* data, uint32_t n);

static uint32_t modesComputeCRC(const uint8_t *msg, int bits) {
    return crc24(msg, (bits/8) - 3);
}

static uint32_t ICAOCacheHashAddress(uint32_t a) {
    a = ((a >> 16) ^ a) * 0x45d9f3b;
    a = ((a >> 16) ^ a) * 0x45d9f3b;
    a = ((a >> 16) ^ a);
    return a & (MODES_ICAO_CACHE_LEN - 1u);
}

static void addRecentlySeenICAOAddr(AdsbCtx *ctx, uint32_t addr) {
    uint32_t h = ICAOCacheHashAddress(addr);
    ctx->icao_cache[h*2]   = addr;
    ctx->icao_cache[h*2+1] = (uint32_t)time(NULL);
}

static int ICAOAddressWasRecentlySeen(AdsbCtx *ctx, uint32_t addr) {
    uint32_t h = ICAOCacheHashAddress(addr);
    uint32_t a = ctx->icao_cache[h*2];
    uint32_t t = ctx->icao_cache[h*2+1];
    return a && a == addr && time(NULL) - t <= MODES_ICAO_CACHE_TTL;
}

static int bruteForceAP(AdsbCtx *ctx, uint8_t *frame, uint32_t n_bits) {
    uint32_t addr;
    uint32_t crc = modesComputeCRC(frame, n_bits);
    uint32_t lastbyte = (n_bits/8) - 1u;
    uint8_t aux[MODES_LONG_MSG_BYTES];

    memcpy(aux, frame, n_bits/8);
    aux[lastbyte]   ^= crc & 0xffu;
    aux[lastbyte-1] ^= (crc >> 8) & 0xffu;
    aux[lastbyte-2] ^= (crc >> 16) & 0xffu;

    addr = ((uint32_t)aux[lastbyte-2] << 16) |
           ((uint32_t)aux[lastbyte-1] << 8) |
            (uint32_t)aux[lastbyte];

    return ICAOAddressWasRecentlySeen(ctx, addr);
}

/* Initialize the error correction syndrome table. */
static void modesInitErrorInfo(void) {
    unsigned char msg[MODES_LONG_MSG_BYTES];
    int i, j, n;
    uint32_t crc;

    n = 0;
    memset(bitErrorTable, 0, sizeof(bitErrorTable));
    memset(msg, 0, MODES_LONG_MSG_BYTES);

    for (i = 5; i < MODES_LONG_MSG_BITS; i++) {
        int bytepos0 = (i >> 3);
        int mask0 = 1 << (7 - (i & 7));
        msg[bytepos0] ^= mask0;
        crc = modesChecksum(msg, MODES_LONG_MSG_BITS);

        bitErrorTable[n].syndrome = crc;
        bitErrorTable[n].bits = 1;
        bitErrorTable[n].pos[0] = i;
        bitErrorTable[n].pos[1] = -1;
        n++;

        for (j = i + 1; j < MODES_LONG_MSG_BITS; j++) {
            int bytepos1 = (j >> 3);
            int mask1 = 1 << (7 - (j & 7));
            msg[bytepos1] ^= mask1;
            crc = modesChecksum(msg, MODES_LONG_MSG_BITS);

            if (n >= NERRORINFO) break;

            bitErrorTable[n].syndrome = crc;
            bitErrorTable[n].bits = 2;
            bitErrorTable[n].pos[0] = i;
            bitErrorTable[n].pos[1] = j;
            n++;

            msg[bytepos1] ^= mask1;
        }
        msg[bytepos0] ^= mask0;
    }

    qsort(bitErrorTable, NERRORINFO, sizeof(struct errorinfo), cmpErrorInfo);
}

/* Fix bit errors using the syndrome table. */
static int fixBitErrors(unsigned char *msg, int bits, int maxfix, int *fixedbits) {
    struct errorinfo *pei;
    struct errorinfo ei;
    int bitpos, offset, i, res;

    memset(&ei, 0, sizeof(struct errorinfo));
    ei.syndrome = modesChecksum(msg, bits);

    pei = bsearch(&ei, bitErrorTable, NERRORINFO,
                  sizeof(struct errorinfo), cmpErrorInfo);
    if (pei == NULL) return 0;

    if (pei->bits > maxfix) return 0;

    offset = MODES_LONG_MSG_BITS - bits;
    for (i = 0; i < pei->bits; i++) {
        bitpos = pei->pos[i] - offset;
        if ((bitpos < 0) || (bitpos >= bits)) return 0;
    }

    res = 0;
    for (i = 0; i < pei->bits; i++) {
        bitpos = pei->pos[i] - offset;
        msg[bitpos >> 3] ^= (1 << (7 - (bitpos & 7)));
        if (fixedbits) {
            fixedbits[res++] = bitpos;
        } else {
            res++;
        }
    }
    return res;
}

/* ------------------------------------------------------------------ */
/* Preamble detection and decoding helpers                            */
/* ------------------------------------------------------------------ */

static uint32_t crc24(const uint8_t* data, uint32_t n) {
    uint32_t crc = 0;
    for (uint32_t i = 0; i < n; ++i) {
        crc ^= (uint32_t)data[i] << 16;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x800000u) ? (crc << 1) ^ 0xFFF409u : crc << 1;
            crc &= 0xFFFFFFu;
        }
    }
    return crc;
}

static int preamble_ok(const uint32_t* m) {
    if (m[0] < m[1] * 2u) return 0;
    if (m[2] < m[1] * 2u || m[2] < m[3] * 2u) return 0;
    if (m[7] < m[6] * 2u || m[7] < m[8] * 2u) return 0;
    if (m[9] < m[8] * 2u || m[9] < m[10] * 2u) return 0;

    uint32_t hi_min = m[0], hi_max = m[0];
    if (m[2] < hi_min) hi_min = m[2]; else if (m[2] > hi_max) hi_max = m[2];
    if (m[7] < hi_min) hi_min = m[7]; else if (m[7] > hi_max) hi_max = m[7];
    if (m[9] < hi_min) hi_min = m[9]; else if (m[9] > hi_max) hi_max = m[9];
    if (hi_min * 3u < hi_max) return 0;

    uint32_t h = m[0] + m[2] + m[7] + m[9];
    uint32_t l = m[1] + m[3] + m[4] + m[5] + m[6] + m[8] +
                 m[10] + m[11] + m[12] + m[13] + m[14] + m[15];
    return h >= l;
}

static uint32_t sample_index(float base, float spb, float chip_off) {
    float x = base * spb - chip_off;
    return (x > 0.0f) ? (uint32_t)(x + 0.99999f) : 0u;
}

/* ------------------------------------------------------------------ */
/* Bitavkodning (PPM)                                                   */
/* ------------------------------------------------------------------ */

static int decode_bit(uint32_t s0, uint32_t s1) {
  return (s0 > s1) ? 1 : 0;
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
                       MrEmitFn emit_fn, void* user_data,
                       int corrected, int ap_recovered) {
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
  if (corrected > 0) {
    kpos += snprintf(kv + kpos, sizeof(kv) - (size_t)kpos,
                     ",\"corrected_bits\":%d", corrected);
  }
  if (ap_recovered) {
    kpos += snprintf(kv + kpos, sizeof(kv) - (size_t)kpos,
                     ",\"ap_recovered\":1");
  }
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
    if (df == 18u) {
      tpos += snprintf(text + tpos, sizeof(text) - (size_t)tpos, " [DF18]");
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
  ctx->error_info_initialized = 0;

  // Debug: notify plugin initialization
  const char* debug_env = getenv("MR_PLUGIN_DEBUG");
  if (debug_env && debug_env[0] != '0') {
    fprintf(stderr, "[adsb_demod] Plugin initialized (sample rate 1.8-2.2 Msps)\n");
  }

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

  /* Initialize error correction table if not done */
  if (!ctx->error_info_initialized) {
    modesInitErrorInfo();
    ctx->error_info_initialized = 1;
  }

  /* Kräver ~2 Msps (acceptera 1.8–2.2 Msps) */
  if (sr < 1800000u || sr > 2200000u) {
    const char* debug_env = getenv("MR_PLUGIN_DEBUG");
    if (debug_env && debug_env[0] != '0') {
      fprintf(stderr, "[adsb_demod] WARNING: Invalid sample rate %u Hz (expected 1.8-2.2 Msps)\n", sr);
    }
    return;
  }

  /* Utöka buffert vid behov */
  const uint32_t needed = ctx->mag_len + num_pairs;
  if (needed > ctx->mag_cap) {
    uint32_t new_cap = needed + 65536u;
    uint32_t* nb = (uint32_t*)realloc(ctx->mag, new_cap * sizeof(uint32_t));
    if (!nb) { ctx->mag_len = 0u; return; }
    ctx->mag = nb;
    ctx->mag_cap = new_cap;
  }

  /* IQ → magnitud (I²+Q²) */
  for (uint32_t n = 0; n < num_pairs; ++n) {
    int32_t i = (int32_t)iq[n * 2u];
    int32_t q = (int32_t)iq[n * 2u + 1u];
    ctx->mag[ctx->mag_len++] = (uint32_t)(i * i) + (uint32_t)(q * q);
  }

  float spb = (float)sr * 1e-6f;
  uint32_t det_off = (sr > 2000000u) ? 1u : 0u;
  float chip_off = (float)(PREAMBLE_SAMPS + det_off) - 8.0f * spb;
  uint32_t min_frame = PREAMBLE_SAMPS + (uint32_t)((float)LONG_BITS * spb) + 2u;

  uint32_t p = 0;
  while (p + min_frame <= ctx->mag_len) {
    if (!preamble_ok(ctx->mag + p)) {
      ++p;
      continue;
    }

    ++ctx->preamble_count;
    const uint32_t* data = ctx->mag + p + PREAMBLE_SAMPS;

    uint32_t df5 = 0;
    for (int b = 0; b < 5; ++b) {
      uint32_t s0 = data[sample_index((float)b, spb, chip_off)];
      uint32_t s1 = data[sample_index((float)b, spb, chip_off + spb * 0.5f)];
      df5 = (df5 << 1u) | (uint32_t)decode_bit(s0, s1);
    }

    uint32_t n_bits = (df5 >= 16u) ? LONG_BITS : SHORT_BITS;
    uint32_t n_samps = PREAMBLE_SAMPS + (uint32_t)((float)n_bits * spb) + 2u;
    if (p + n_samps > ctx->mag_len) break;

    uint8_t frame[MODES_LONG_MSG_BYTES];
    memset(frame, 0, sizeof(frame));
    for (uint32_t bit = 0; bit < n_bits; ++bit) {
      uint32_t s0 = data[sample_index((float)bit, spb, chip_off)];
      uint32_t s1 = data[sample_index((float)bit, spb, chip_off + spb * 0.5f)];
      if (decode_bit(s0, s1))
        frame[bit / 8u] |= (uint8_t)(0x80u >> (bit % 8u));
    }

    uint32_t n_bytes = n_bits / 8u;
    uint32_t crc_calc = crc24(frame, n_bytes - 3u);
    uint32_t crc_fram = ((uint32_t)frame[n_bytes - 3u] << 16) |
                        ((uint32_t)frame[n_bytes - 2u] <<  8) |
                         (uint32_t)frame[n_bytes - 1u];

    int corrected = 0;
    int ap_recovered = 0;
    if (crc_calc != crc_fram) {
      int fixed = fixBitErrors(frame, n_bits, MODES_MAX_BITERRORS, NULL);
      if (fixed > 0) {
        corrected = fixed;
        crc_calc = crc24(frame, n_bytes - 3u);
      } else {
        uint32_t df = (frame[0] >> 3) & 0x1Fu;
        if (df == 0u || df == 4u || df == 5u || df == 16u ||
            df == 20u || df == 21u || df == 24u) {
          if (bruteForceAP(ctx, frame, n_bits)) {
            ap_recovered = 1;
            corrected = -1;
          }
        }
      }
    }

    uint32_t df = (frame[0] >> 3) & 0x1Fu;
    if (crc_calc == crc_fram || ap_recovered) {
      ctx->crc_ok_count++;
      if (corrected > 0) ctx->crc_ok_corrected_count += (uint64_t)corrected;
      if (ap_recovered) ctx->crc_ok_ap_count++;
      if (df == 17u) ctx->df17_ok_count++;

      uint32_t icao = ((uint32_t)frame[1] << 16) |
                      ((uint32_t)frame[2] <<  8) |
                       (uint32_t)frame[3];
      if (df == 17u || df == 18u) {
        addRecentlySeenICAOAddr(ctx, icao);
      }

      const char* debug_env = getenv("MR_PLUGIN_DEBUG");
      if (debug_env && debug_env[0] != '0') {
        fprintf(stderr, "[adsb_demod] OK: DF=%u ICAO=%06X bytes=%u", df, icao, n_bytes);
        if (corrected > 0) fprintf(stderr, " corrected=%d", corrected);
        if (ap_recovered) fprintf(stderr, " ap_recovered=1");
        if (df == 17u && ap_recovered) fprintf(stderr, " [DF17 AP RECOVERED]");
        if (df == 18u) fprintf(stderr, " [DF18]");
        fprintf(stderr, " hex=");
        for (uint32_t x = 0; x < n_bytes; ++x)
          fprintf(stderr, "%02X", frame[x]);
        fprintf(stderr, "\n");
      }

      emit_frame(frame, n_bytes, freq_hz, unix_ms, emit_fn, user_data, corrected, ap_recovered);
      p += n_samps - 2u;
    } else {
      if (df == 17u || df == 18u) {
        if ((df == 17u && ctx->df17_fail_count < 5u) ||
            (df == 18u && ctx->df18_fail_count < 5u)) {
          fprintf(stderr, "[ADS-B DBG] DF%u #%llu sr=%u spb=%.3f: ",
                  df,
                  (unsigned long long)((df == 17u) ? ctx->df17_fail_count : ctx->df18_fail_count) + 1u,
                  sr, (double)spb);
          for (uint32_t x = 0; x < n_bytes; ++x)
            fprintf(stderr, "%02X", frame[x]);
          fprintf(stderr, "  calc=%06X rx=%06X\n", crc_calc, crc_fram);
        }
        if (df == 17u) ++ctx->df17_fail_count;
        if (df == 18u) ++ctx->df18_fail_count;
      }
      ++ctx->crc_fail_count;
      ++p;
    }
  }

  uint32_t keep = ctx->mag_len - p;
  if (keep > 0u) {
    memmove(ctx->mag, ctx->mag + p, keep * sizeof(uint32_t));
  }
  ctx->mag_len = keep;

  if (unix_ms - ctx->last_stats_ms >= 10000u) {
    ctx->last_stats_ms = unix_ms;
    fprintf(stderr,
            "[ADS-B] preamble=%llu  crc_ok=%llu(+%llu korr +%llu ap)  crc_fail=%llu"
            "  df17_ok=%llu  df17_fail=%llu  df18_ok=%llu  df18_fail=%llu\n",
            (unsigned long long)ctx->preamble_count,
            (unsigned long long)ctx->crc_ok_count,
            (unsigned long long)ctx->crc_ok_corrected_count,
            (unsigned long long)ctx->crc_ok_ap_count,
            (unsigned long long)ctx->crc_fail_count,
            (unsigned long long)ctx->df17_ok_count,
            (unsigned long long)ctx->df17_fail_count,
            (unsigned long long)ctx->df18_ok_count,
            (unsigned long long)ctx->df18_fail_count);
  }
}
