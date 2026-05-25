/**
 * vdes_synth_liq.c — VDES VDE-TER syntetisk signalgenerator via libliquid
 *
 * Genererar en .iq16-fil med pi/4-DQPSK-modulerade VDES-burstar.
 * Interpolationsfiltret skapas med firinterp_crcf_create_prototype(RRC, k, m, β)
 * med EXAKT samma parametrar som symsync_crcf i vdes_burst_demod.c, vilket
 * garanterar att TX_RRC × RX_RRC = RC och noll ISI vid samplingstidpunkterna.
 *
 * Användning:
 *   vdes_synth_liq --out FILE.iq16 [OPTIONS]
 *
 * Options:
 *   --rate HZ          Utdata-samplingsfrekvens (standard 2048000)
 *   --sym-rate BAUD    Symbolhastighet (standard 76800)
 *   --freq HZ          Centerfrekvens (standard 161862500, bara metadatamärkning)
 *   --offset HZ        Bärvågsoffset i bas-bandet (standard -50000)
 *   --snr-db DB        Signal-brusförhållande (standard 20)
 *   --n-bursts N       Antal burstar (standard 3)
 *   --burst-gap-ms MS  Tystnad mellan burstar ms (standard 50)
 *
 * Ramstruktur (ITU-R M.2092-2, LID 11, ingen FEC):
 *   [27 träningssymboler] [16 LID-symboler] [200 payload-symboler (400 bitar)]
 *
 * Varje symbol är ett pi/4-QPSK-dibit; utdata-bitar är direkt jämförbara
 * med vdes_decode.py efter avkodning.
 */

#include <liquid/liquid.h>

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ── Parametrar (måste matcha vdes_burst_demod.c) ──────────────────────── */
#define BD_K           4u       /* sampler/symbol i symsync                */
#define BD_SYMSYNC_M   5u       /* filter half-length                       */
#define BD_SYMSYNC_BETA 0.35f   /* RRC roll-off                             */

/* ── Träningssekvens (ITU-R M.2092-2, Tabell 1) ────────────────────────── */
static const uint8_t kTrain[27] = {
    1,1,1,1,1,1,0,0, 1,1,0,1,0,1,0,0, 0,0,0,1,1,0,0,1, 0,1,0
};

/* ── Link ID 11 OTA (RM-kodord XOR mask 0xC2E28E4F) ────────────────────── */
static const uint8_t kLID11_ota[32] = {
    0,0,1,0,1,1,1,1, 1,1,0,0,1,1,0,0, 0,1,0,0,1,1,0,0, 0,0,1,1,0,0,1,1
};

/* ── LFSR-scrambler (ITU-R M.2092-2 §A2-1.2.6) ─────────────────────────── */
static uint8_t lfsr_state[15] = {1,0,0,1,0,1,0,0,0,0,0,0,0,0,0};

static uint8_t lfsr_next(void) {
    uint8_t fb = lfsr_state[14] ^ lfsr_state[13];
    memmove(lfsr_state + 1, lfsr_state, 14);
    lfsr_state[0] = fb;
    return fb;
}

static void lfsr_reset(void) {
    const uint8_t init[15] = {1,0,0,1,0,1,0,0,0,0,0,0,0,0,0};
    memcpy(lfsr_state, init, 15);
}

/* ── CRC-32/MPEG-2 ──────────────────────────────────────────────────────── */
static uint32_t crc32_mpeg(const uint8_t* bits, uint32_t n) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < n; ++i) {
        if (((crc >> 31) ^ bits[i]) & 1)
            crc = ((crc << 1) ^ 0x04C11DB7u) & 0xFFFFFFFFu;
        else
            crc = (crc << 1) & 0xFFFFFFFFu;
    }
    return crc;
}

/* ── pi/4-DQPSK differential phase lookup ──────────────────────────────── */
static float dibit_phase(uint8_t b0, uint8_t b1) {
    if (!b0 && !b1) return  (float)( M_PI / 4.0);
    if (!b0 &&  b1) return  (float)(3*M_PI / 4.0);
    if ( b0 &&  b1) return  (float)(-3*M_PI / 4.0);
    /* b0=1, b1=0 */
    return  (float)(-M_PI / 4.0);
}

/* ── AWGN ────────────────────────────────────────────────────────────────── */
static float randn(void) {
    /* Box-Muller */
    float u, v, s;
    do {
        u = 2.0f * (float)rand() / (float)RAND_MAX - 1.0f;
        v = 2.0f * (float)rand() / (float)RAND_MAX - 1.0f;
        s = u*u + v*v;
    } while (s >= 1.0f || s < 1e-10f);
    return u * sqrtf(-2.0f * logf(s) / s);
}

/* ── Bygg VDES ASM-burst ─────────────────────────────────────────────────── */
/* bits: kTrain (27) + LID11_ota (32) + scrambled payload (400) = 459 "bitar"
 * Notera: träningsbitar är "single-bit per symbol" (training bit → dibit (b,b)),
 * LID/payload är "2 bitar per symbol" (normalt pi/4-QPSK).
 * Funktionen returnerar antalet symboler och fyller phi[]-arrayen. */
static uint32_t build_burst_phases(float* phi, uint32_t phi_cap,
                                   const uint8_t* payload_bits,
                                   uint32_t n_payload_bits) {
    uint32_t nsym = 0;
    float theta = 0.0f;

    /* Träning: 27 symboler (varje träningsbit → dibit (b,b)) */
    for (uint32_t i = 0; i < 27 && nsym < phi_cap; ++i) {
        theta += dibit_phase(kTrain[i], kTrain[i]);
        phi[nsym++] = theta;
    }

    /* LID: 16 symboler */
    for (uint32_t i = 0; i < 32 && nsym < phi_cap; i += 2) {
        theta += dibit_phase(kLID11_ota[i], kLID11_ota[i+1]);
        phi[nsym++] = theta;
    }

    /* Payload: n_payload_bits/2 symboler */
    for (uint32_t i = 0; i < n_payload_bits && nsym < phi_cap; i += 2) {
        theta += dibit_phase(payload_bits[i], payload_bits[i+1]);
        phi[nsym++] = theta;
    }

    return nsym;
}

/* ── Huvud ───────────────────────────────────────────────────────────────── */

int main(int argc, char** argv) {
    const char* out_path    = "vdes_test.iq16";
    uint32_t    sample_rate = 2048000u;
    uint32_t    sym_rate    = 76800u;
    int         freq_hz     = 161862500;
    int         offset_hz   = -50000;
    float       snr_db      = 20.0f;
    uint32_t    n_bursts    = 3u;
    uint32_t    gap_ms      = 50u;

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--out")       && i+1 < argc) out_path    = argv[++i];
        else if (!strcmp(argv[i], "--rate")     && i+1 < argc) sample_rate = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--sym-rate") && i+1 < argc) sym_rate    = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--freq")     && i+1 < argc) freq_hz     = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--offset")   && i+1 < argc) offset_hz   = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--snr-db")   && i+1 < argc) snr_db      = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--n-bursts") && i+1 < argc) n_bursts    = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--burst-gap-ms") && i+1 < argc) gap_ms  = (uint32_t)atoi(argv[++i]);
    }

    FILE* fp = fopen(out_path, "wb");
    if (!fp) { fprintf(stderr, "Kan inte öppna: %s\n", out_path); return 1; }

    fprintf(stderr, "vdes_synth_liq: %s  rate=%u sym=%u offset=%d snr=%.1fdB n=%u\n",
            out_path, sample_rate, sym_rate, offset_hz, (double)snr_db, n_bursts);

    /* Skapa interpolationsfilter (EXAKT samma RRC som symsync_crcf) */
    firinterp_crcf interp = firinterp_crcf_create_prototype(
        LIQUID_FIRFILT_RRC, BD_K, BD_SYMSYNC_M, BD_SYMSYNC_BETA, 0);

    /* Resampler från BD_K*sym_rate till sample_rate */
    float resamp_rate = (float)sample_rate / (float)(BD_K * sym_rate);
    msresamp_crcf resamp = msresamp_crcf_create(resamp_rate, 60.0f);

    /* NCO för bärvågsoffset */
    nco_crcf nco = nco_crcf_create(LIQUID_NCO);
    nco_crcf_set_frequency(nco,
        2.0f * (float)M_PI * (float)offset_hz / (float)sample_rate);

    /* Beräkna bruseffekt */
    float noise_std = powf(10.0f, -snr_db / 20.0f);

    /* Payload: 368 bit data (slumpad) + 32 bit CRC, LFSR-scramblad = 400 bit */
    const uint32_t N_DATA = 368u;
    const uint32_t N_PAYLOAD = 400u;
    uint8_t data_bits[400];  /* data + CRC */
    uint8_t scrambled[400];

    /* Symbol-/IQ-buffertar */
    liquid_float_complex sym_buf[1];
    liquid_float_complex interp_out[BD_K];
    uint32_t resamp_out_max = (uint32_t)(BD_K * 4u + 32u);
    liquid_float_complex* resamp_out =
        (liquid_float_complex*)malloc(resamp_out_max * sizeof(liquid_float_complex));

    float phase_buf[27 + 32 + 200 + 16];
    const uint32_t PHASE_CAP = sizeof(phase_buf) / sizeof(phase_buf[0]);

    uint32_t gap_samples     = (uint32_t)((float)gap_ms / 1000.0f * (float)sample_rate);
    uint32_t lead_in_samples = (uint32_t)(0.05f * (float)sample_rate); /* 50 ms tystnad */

    /* ── Skriv inledande tystnad ─────────────────────────────────────────── */
    {
        int16_t zero[2] = {0, 0};
        for (uint32_t s = 0; s < lead_in_samples; ++s) fwrite(zero, 4, 1, fp);
    }

    srand(42);

    for (uint32_t burst = 0; burst < n_bursts; ++burst) {
        /* Slumpad nyttolast */
        for (uint32_t i = 0; i < N_DATA; ++i) data_bits[i] = rand() & 1;

        /* CRC-32 på data */
        uint32_t crc = crc32_mpeg(data_bits, N_DATA);
        for (uint32_t i = 0; i < 32; ++i)
            data_bits[N_DATA + i] = (crc >> (31 - i)) & 1;

        /* LFSR-scrambling */
        lfsr_reset();
        for (uint32_t i = 0; i < N_PAYLOAD; ++i)
            scrambled[i] = data_bits[i] ^ lfsr_next();

        fprintf(stderr, "  burst %u CRC=0x%08X\n", burst, crc);

        /* Bygg symbolfaser */
        uint32_t n_syms = build_burst_phases(phase_buf, PHASE_CAP,
                                              scrambled, N_PAYLOAD);

        /* Flytta fram filtret med BD_SYMSYNC_M nollsymboler så att
         * filter-ramp-up sker under tystnad, inte under träningssekvensen. */
        {
            liquid_float_complex z = {0};
            liquid_float_complex flush[BD_K];
            for (uint32_t pre = 0; pre < BD_SYMSYNC_M; ++pre) {
                firinterp_crcf_execute(interp, z, flush);
                for (uint32_t k = 0; k < BD_K; ++k) {
                    unsigned int nr = 0u;
                    msresamp_crcf_execute(resamp, &flush[k], 1, resamp_out, &nr);
                    for (unsigned int r = 0; r < nr; ++r) {
                        liquid_float_complex mixed;
                        nco_crcf_mix_up(nco, resamp_out[r], &mixed);
                        nco_crcf_step(nco);
                        int16_t i16 = (int16_t)(crealf(mixed) * 30000.0f);
                        int16_t q16 = (int16_t)(cimagf(mixed) * 30000.0f);
                        fwrite(&i16, 2, 1, fp);
                        fwrite(&q16, 2, 1, fp);
                    }
                }
            }
        }

        /* Modulera och skriv */
        for (uint32_t s = 0; s < n_syms; ++s) {
            __real__ sym_buf[0] = cosf(phase_buf[s]);
            __imag__ sym_buf[0] = sinf(phase_buf[s]);

            firinterp_crcf_execute(interp, sym_buf[0], interp_out);

            for (uint32_t k = 0; k < BD_K; ++k) {
                /* AWGN */
                __real__ interp_out[k] += noise_std * randn();
                __imag__ interp_out[k] += noise_std * randn();

                /* Resampler */
                unsigned int nr = 0u;
                if ((uint32_t)(BD_K + 32u) > resamp_out_max) {
                    resamp_out_max = BD_K + 64u;
                    resamp_out = (liquid_float_complex*)
                        realloc(resamp_out, resamp_out_max * sizeof(*resamp_out));
                }
                msresamp_crcf_execute(resamp, &interp_out[k], 1, resamp_out, &nr);

                for (unsigned int r = 0; r < nr; ++r) {
                    liquid_float_complex mixed;
                    nco_crcf_mix_up(nco, resamp_out[r], &mixed);
                    nco_crcf_step(nco);

                    int16_t i16 = (int16_t)(crealf(mixed) * 30000.0f);
                    int16_t q16 = (int16_t)(cimagf(mixed) * 30000.0f);
                    fwrite(&i16, 2, 1, fp);
                    fwrite(&q16, 2, 1, fp);
                }
            }
        }

        /* Tystnad efter burst */
        if (burst + 1 < n_bursts) {
            int16_t zero[2] = {0, 0};
            for (uint32_t s = 0; s < gap_samples; ++s) {
                /* AWGN i tystnaden för realistisk gate-detektering */
                float ni = noise_std * randn() * 0.1f;
                float nq = noise_std * randn() * 0.1f;
                int16_t i16 = (int16_t)(ni * 30000.0f);
                int16_t q16 = (int16_t)(nq * 30000.0f);
                fwrite(&i16, 2, 1, fp);
                fwrite(&q16, 2, 1, fp);
            }
        }
    }

    /* Avslutande tystnad */
    {
        int16_t zero[2] = {0, 0};
        uint32_t trail = (uint32_t)(0.05f * (float)sample_rate);
        for (uint32_t s = 0; s < trail; ++s) fwrite(zero, 4, 1, fp);
    }

    firinterp_crcf_destroy(interp);
    msresamp_crcf_destroy(resamp);
    nco_crcf_destroy(nco);
    free(resamp_out);
    fclose(fp);

    fprintf(stderr, "Färdig: %s\n", out_path);
    return 0;
}
