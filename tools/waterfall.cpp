#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <complex>
#include <algorithm>
#include <string>
#include <iomanip>
#include <numeric>
#include <cstdint>
// FFTW3-biblioteket
#include <fftw3.h>

// Aktivera implementeringen av STB Image Write
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Struktur för RGB-färg
struct RGB {
    uint8_t r, g, b;
};

// Funktion för att skapa ett Rainbow-färgschema baserat på ett värde mellan 0.0 och 1.0
RGB get_rainbow_color(float value) {
    value = std::clamp(value, 0.0f, 1.0f);
    
    // Invertera så att hög energi blir röd (1.0) och låg blir blå (0.0)
    float h = (1.0f - value) * 240.0f; // 240 är blått, 0 är rött i HSV
    
    float x = (1.0f - std::abs(std::fmod(h / 60.0f, 2.0f) - 1.0f));
    float r = 0, g = 0, b = 0;

    if (h >= 0 && h < 60)        { r = 1.0f; g = x;    b = 0;    }
    else if (h >= 60 && h < 120) { r = x;    g = 1.0f; b = 0;    }
    else if (h >= 120 && h < 180){ r = 0;    g = 1.0f; b = x;    }
    else if (h >= 180 && h < 240){ r = 0;    g = x;    b = 1.0f; }
    
    return { static_cast<uint8_t>(r * 255), static_cast<uint8_t>(g * 255), static_cast<uint8_t>(b * 255) };
}

int main(int argc, char* argv[]) {
    if (argc < 8) {
        std::cerr << "Användning: " << argv[0] << " <input_iq16_fil> <output_png> <fft_storlek> <center_freq_hz> <sample_rate_hz> <kanal_bredd_hz> <squelch_db>\n";
        std::cerr << "Exempel: " << argv[0] << " gqrx_capture.iq16 sdr_waterfall.png 2048 433920000 2048000 25000 12.0\n";
        return 1;
    }

    std::string input_path = argv[1];
    std::string output_path = argv[2];
    size_t fft_size = std::stoul(argv[3]);
    double center_freq = std::stod(argv[4]);
    double sample_rate = std::stod(argv[5]);
    double channel_width = std::stod(argv[6]);
    float squelch_threshold_db = std::stof(argv[7]); // Hur många dB över bruset krävs?

    // Kontrollera FFT-storleken
    if ((fft_size & (fft_size - 1)) != 0 || fft_size == 0) {
        std::cerr << "Fel: FFT-storleken måste vara en potens av 2.\n";
        return 1;
    }

    std::ifstream file(input_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "Kunde inte öppna filen: " << input_path << "\n";
        return 1;
    }

    std::streamsize file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    size_t total_samples = file_size / sizeof(int16_t) / 2;
    size_t num_rows = total_samples / fft_size;

    if (num_rows == 0) {
        std::cerr << "Filen är för liten för den valda FFT-storleken.\n";
        return 1;
    }

    std::cout << "Analyserar " << num_rows << " rader med FFTW3...\n";

    // Förbered Hanning-fönster
    std::vector<float> window(fft_size);
    for (size_t i = 0; i < fft_size; ++i) {
        window[i] = 0.5f * (1.0f - std::cos(2.0f * M_PI * i / (fft_size - 1)));
    }

    // Allokera minne för FFTW3
    fftwf_complex* fftw_in = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * fft_size);
    fftwf_complex* fftw_out = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * fft_size);
    fftwf_plan plan = fftwf_plan_dft_1d(fft_size, fftw_in, fftw_out, FFTW_FORWARD, FFTW_MEASURE);

    // Grid för att spara alla dB-värden
    std::vector<std::vector<float>> magnitude_grid(num_rows, std::vector<float>(fft_size));
    
    double total_db_sum = 0.0;
    size_t total_bins = num_rows * fft_size;

    std::vector<int16_t> iq_buffer(fft_size * 2);

    for (size_t r = 0; r < num_rows; ++r) {
        file.read(reinterpret_cast<char*>(iq_buffer.data()), fft_size * 2 * sizeof(int16_t));

        for (size_t i = 0; i < fft_size; ++i) {
            fftw_in[i][0] = static_cast<float>(iq_buffer[2 * i]) * window[i];     
            fftw_in[i][1] = static_cast<float>(iq_buffer[2 * i + 1]) * window[i]; 
        }

        fftwf_execute(plan);

        for (size_t i = 0; i < fft_size; ++i) {
            size_t shifted_index = (i + fft_size / 2) % fft_size;
            
            float real = fftw_out[i][0];
            float imag = fftw_out[i][1];
            float mag_sq = real * real + imag * imag;
            
            float db = 10.0f * std::log10(mag_sq + 1e-10f);

            magnitude_grid[r][shifted_index] = db;
            total_db_sum += db; // Samla ihop för att räkna ut genomsnittligt brus
        }
    }

    fftwf_destroy_plan(plan);
    fftwf_free(fftw_in);
    fftwf_free(fftw_out);
    file.close();

    // Beräkna den genomsnittliga brusmattan (noise floor)
    float avg_noise_floor = static_cast<float>(total_db_sum / total_bins);
    float dynamic_squelch_level = avg_noise_floor + squelch_threshold_db;

    std::cout << "Genomsnittlig brusmatta: " << avg_noise_floor << " dB\n";
    std::cout << "Squelch-tröskel satt till: " << dynamic_squelch_level << " dB\n";

    // Hitta maxvärdet över tröskeln för att skala färgerna snyggt
    float max_mag = dynamic_squelch_level;
    for (size_t r = 0; r < num_rows; ++r) {
        for (size_t c = 0; c < fft_size; ++c) {
            if (magnitude_grid[r][c] > max_mag) {
                max_mag = magnitude_grid[r][c];
            }
        }
    }
    
    float range = max_mag - dynamic_squelch_level;
    if (range <= 0) range = 1.0f;

    // Generera PNG-bilden
    int scale_height = 40;
    int img_width = static_cast<int>(fft_size);
    int img_height = static_cast<int>(num_rows) + scale_height;
    std::vector<uint8_t> image_data(img_width * img_height * 3, 0);

    // 1. Rita vattenfallet med brusreducering (Squelch)
    for (size_t r = 0; r < num_rows; ++r) {
        for (size_t c = 0; c < fft_size; ++c) {
            float current_db = magnitude_grid[r][c];
            RGB color;

            if (current_db < dynamic_squelch_level) {
                // Allt under tröskeln blir "tyst" mörkblå bakgrund (bruset dämpas helt)
                color = { 5, 5, 25 }; 
            } else {
                // Signaler över tröskeln mappas till Rainbow-skalan
                float norm = (current_db - dynamic_squelch_level) / range;
                color = get_rainbow_color(norm);
            }

            size_t pixel_idx = ((r + scale_height) * img_width + c) * 3;
            image_data[pixel_idx]     = color.r;
            image_data[pixel_idx + 1] = color.g;
            image_data[pixel_idx + 2] = color.b;
        }
    }

    // 2. Rita frekvenskanaler och linjer
    double hz_per_pixel = sample_rate / fft_size;
    double start_freq = center_freq - (sample_rate / 2.0);

    for (size_t c = 0; c < fft_size; ++c) {
        double current_freq = start_freq + (c * hz_per_pixel);
        
        double dist_to_channel = std::fmod(std::abs(current_freq - center_freq), channel_width);
        if (dist_to_channel > channel_width / 2.0) {
            dist_to_channel = channel_width - dist_to_channel;
        }

        if (dist_to_channel < (hz_per_pixel * 0.5)) {
            for (int y = 15; y < scale_height; ++y) {
                size_t idx = (y * img_width + c) * 3;
                image_data[idx] = 255; image_data[idx+1] = 255; image_data[idx+2] = 255;
            }

            for (size_t r = 0; r < num_rows; r += 6) { 
                size_t idx = ((r + scale_height) * img_width + c) * 3;
                // Gör rutnätet lite subtilt så det inte krockar med signalerna
                image_data[idx] = 80; image_data[idx+1] = 80; image_data[idx+2] = 80;
            }
        }

        if (std::abs(current_freq - center_freq) < (hz_per_pixel * 0.5)) {
            for (int y = 0; y < scale_height; ++y) {
                size_t idx = (y * img_width + c) * 3;
                image_data[idx] = 255; image_data[idx+1] = 0; image_data[idx+2] = 0;
            }
        }
    }

    std::cout << "Sparar PNG-fil till: " << output_path << "...\n";
    if (stbi_write_png(output_path.c_str(), img_width, img_height, 3, image_data.data(), img_width * 3)) {
        std::cout << "Klart! Bildstorlek: " << img_width << "x" << img_height << " pixlar.\n";
    } else {
        std::cerr << "Kunde inte skriva PNG-fil.\n";
        return 1;
    }

    return 0;
}
