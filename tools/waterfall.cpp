#include <cstdint>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <complex>
#include <algorithm>
#include <string>
#include <iomanip>
#include <numeric>

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

// Enkel bitmap för siffror 0-9 (5x3 pixlar) för tidsskalan
const uint8_t FONT[10][5] = {
    {0x7, 0x5, 0x5, 0x5, 0x7}, // 0
    {0x2, 0x2, 0x2, 0x2, 0x2}, // 1
    {0x7, 0x1, 0x7, 0x4, 0x7}, // 2
    {0x7, 0x1, 0x7, 0x1, 0x7}, // 3
    {0x5, 0x5, 0x7, 0x1, 0x1}, // 4
    {0x7, 0x4, 0x7, 0x1, 0x7}, // 5
    {0x7, 0x4, 0x7, 0x5, 0x7}, // 6
    {0x7, 0x1, 0x1, 0x1, 0x1}, // 7
    {0x7, 0x5, 0x7, 0x5, 0x7}, // 8
    {0x7, 0x5, 0x7, 0x1, 0x7}  // 9
};

// Funktion för att rita ett tecken (siffra eller 's' / '.')
void draw_char(std::vector<uint8_t>& img, int w, int x, int y, char c, RGB color) {
    if (x + 3 >= w) return;
    int digit = c - '0';
    if (digit >= 0 && digit <= 9) {
        for (int row = 0; row < 5; ++row) {
            for (int col = 0; col < 3; ++col) {
                if ((FONT[digit][row] >> (2 - col)) & 1) {
                    size_t idx = ((y + row) * w + (x + col)) * 3;
                    if (idx + 2 < img.size()) {
                        img[idx] = color.r; img[idx+1] = color.g; img[idx+2] = color.b;
                    }
                }
            }
        }
    } else if (c == 's') { // Enkelt 's' för sekunder
        int s_bmp[5] = {0x7, 0x4, 0x7, 0x1, 0x7};
        for (int row = 0; row < 5; ++row) {
            for (int col = 0; col < 3; ++col) {
                if ((s_bmp[row] >> (2 - col)) & 1) {
                    size_t idx = ((y + row) * w + (x + col)) * 3;
                    img[idx] = color.r; img[idx+1] = color.g; img[idx+2] = color.b;
                }
            }
        }
    } else if (c == '.') { // Punkt
        size_t idx = ((y + 4) * w + (x + 1)) * 3;
        img[idx] = color.r; img[idx+1] = color.g; img[idx+2] = color.b;
    }
}

// Rita en sträng på skalan
void draw_string(std::vector<uint8_t>& img, int w, int x, int y, const std::string& str, RGB color) {
    int cur_x = x;
    for (char c : str) {
        draw_char(img, w, cur_x, y, c, color);
        cur_x += 4; // 3 pixlar bredd + 1 pixel mellanrum
    }
}

RGB get_rainbow_color(float value) {
    value = std::clamp(value, 0.0f, 1.0f);
    float h = (1.0f - value) * 240.0f;
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
        std::cerr << "Användning: " << argv[0] << " <iq_fil> <ut_png> <fft_storlek> <center_hz> <rate_hz> <kanal_hz> <squelch_db> [start_sek] [slut_sek]\n";
        return 1;
    }

    std::string input_path = argv[1];
    std::string output_path = argv[2];
    size_t fft_size = std::stoul(argv[3]);
    double center_freq = std::stod(argv[4]);
    double sample_rate = std::stod(argv[5]);
    double channel_width = std::stod(argv[6]);
    float squelch_threshold_db = std::stof(argv[7]);

    double start_sec = (argc >= 9) ? std::stod(argv[8]) : 0.0;
    double end_sec = (argc >= 10) ? std::stod(argv[9]) : -1.0; // -1 betyder till filens slut

    if ((fft_size & (fft_size - 1)) != 0 || fft_size == 0) {
        std::cerr << "Fel: FFT-storleken måste vara en potens av 2.\n";
        return 1;
    }

    std::ifstream file(input_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "Kunde inte öppna filen.\n";
        return 1;
    }

    std::streamsize file_size = file.tellg();
    size_t total_samples = file_size / sizeof(int16_t) / 2;
    double total_duration = static_cast<double>(total_samples) / sample_rate;

    // Bestäm tidsfönster
    if (end_sec < 0.0 || end_sec > total_duration) {
        end_sec = total_duration;
    }
    if (start_sec < 0.0) start_sec = 0.0;
    if (start_sec >= end_sec) {
        std::cerr << "Fel: Starttid måste vara mindre än sluttid.\n";
        return 1;
    }

    // Räkna ut vilka rader/FFT-block som ska läsas
    double time_per_fft = static_cast<double>(fft_size) / sample_rate;
    size_t start_row = static_cast<size_t>(start_sec / time_per_fft);
    size_t end_row = static_cast<size_t>(end_sec / time_per_fft);
    size_t num_rows = end_row - start_row;

    if (num_rows == 0) {
        std::cerr << "Det valda tidsintervallet innehåller inga FFT-block.\n";
        return 1;
    }

    std::cout << "Filens totala längd: " << std::fixed << std::setprecision(2) << total_duration << " s\n";
    std::cout << "Renderar från " << start_sec << " s till " << end_sec << " s (" << num_rows << " rader)...\n";

    // Flytta filpekaren till startpositionen
    size_t bytes_per_row = fft_size * 2 * sizeof(int16_t);
    file.seekg(start_row * bytes_per_row, std::ios::beg);

    // Förbered fönsterfunktion och FFTW3
    std::vector<float> window(fft_size);
    for (size_t i = 0; i < fft_size; ++i) window[i] = 0.5f * (1.0f - std::cos(2.0f * M_PI * i / (fft_size - 1)));

    fftwf_complex* fftw_in = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * fft_size);
    fftwf_complex* fftw_out = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * fft_size);
    fftwf_plan plan = fftwf_plan_dft_1d(fft_size, fftw_in, fftw_out, FFTW_FORWARD, FFTW_MEASURE);

    std::vector<std::vector<float>> magnitude_grid(num_rows, std::vector<float>(fft_size));
    double total_db_sum = 0.0;

    std::vector<int16_t> iq_buffer(fft_size * 2);

    for (size_t r = 0; r < num_rows; ++r) {
        file.read(reinterpret_cast<char*>(iq_buffer.data()), bytes_per_row);
        if (!file) { num_rows = r; break; } // Om filen tar slut i förtid

        for (size_t i = 0; i < fft_size; ++i) {
            fftw_in[i][0] = static_cast<float>(iq_buffer[2 * i]) * window[i];     
            fftw_in[i][1] = static_cast<float>(iq_buffer[2 * i + 1]) * window[i]; 
        }

        fftwf_execute(plan);

        for (size_t i = 0; i < fft_size; ++i) {
            size_t shifted_index = (i + fft_size / 2) % fft_size;
            float mag_sq = fftw_out[i][0]*fftw_out[i][0] + fftw_out[i][1]*fftw_out[i][1];
            float db = 10.0f * std::log10(mag_sq + 1e-10f);
            magnitude_grid[r][shifted_index] = db;
            total_db_sum += db;
        }
    }

    fftwf_destroy_plan(plan); fftwf_free(fftw_in); fftwf_free(fftw_out); file.close();

    float avg_noise_floor = static_cast<float>(total_db_sum / (num_rows * fft_size));
    float dynamic_squelch_level = avg_noise_floor + squelch_threshold_db;

    float max_mag = dynamic_squelch_level;
    for (size_t r = 0; r < num_rows; ++r)
        for (size_t c = 0; c < fft_size; ++c)
            if (magnitude_grid[r][c] > max_mag) max_mag = magnitude_grid[r][c];
    
    float range = max_mag - dynamic_squelch_level;
    if (range <= 0) range = 1.0f;

    // Dimensioner för den färdiga bilden (Läger till 60 pixlar till höger för tidsskalan)
    int scale_height = 40;
    int time_scale_width = 60; 
    int img_width = static_cast<int>(fft_size) + time_scale_width;
    int img_height = static_cast<int>(num_rows) + scale_height;
    std::vector<uint8_t> image_data(img_width * img_height * 3, 15); // Mörkgrå bas för marginaler

    // 1. Rita vattenfallet
    for (size_t r = 0; r < num_rows; ++r) {
        for (size_t c = 0; c < fft_size; ++c) {
            float current_db = magnitude_grid[r][c];
            RGB color = (current_db < dynamic_squelch_level) ? RGB{5, 5, 25} : get_rainbow_color((current_db - dynamic_squelch_level) / range);

            size_t pixel_idx = ((r + scale_height) * img_width + c) * 3;
            image_data[pixel_idx] = color.r; image_data[pixel_idx+1] = color.g; image_data[pixel_idx+2] = color.b;
        }
    }

    // 2. Rita frekvenskanaler (X-axel)
    double hz_per_pixel = sample_rate / fft_size;
    double start_freq = center_freq - (sample_rate / 2.0);

    for (size_t c = 0; c < fft_size; ++c) {
        double current_freq = start_freq + (c * hz_per_pixel);
        double dist_to_channel = std::fmod(std::abs(current_freq - center_freq), channel_width);
        if (dist_to_channel > channel_width / 2.0) dist_to_channel = channel_width - dist_to_channel;

        if (dist_to_channel < (hz_per_pixel * 0.5)) {
            for (int y = 15; y < scale_height; ++y) {
                size_t idx = (y * img_width + c) * 3;
                image_data[idx] = 255; image_data[idx+1] = 255; image_data[idx+2] = 255;
            }
            for (size_t r = 0; r < num_rows; r += 6) { 
                size_t idx = ((r + scale_height) * img_width + c) * 3;
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

    // 3. Rita tidsskalan (Y-axel på högerkanten)
    int label_spacing = std::max(1, static_cast<int>(num_rows / 10)); // Skapa ca 10 tidsstämplar jämnt fördelat
    for (size_t r = 0; r < num_rows; r += label_spacing) {
        double current_time = start_sec + (r * time_per_fft);
        
        // Formatera sträng, t.ex. "14.5s"
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << current_time << "s";
        
        int text_x = static_cast<int>(fft_size) + 8;
        int text_y = static_cast<int>(r) + scale_height - 2; // -2 centrerar texten vertikalt mot strecket

        draw_string(image_data, img_width, text_x, text_y, ss.str(), {200, 200, 200});

        // Rita ett litet vitt markeringsstreck vid tidpunkten
        for (int x = static_cast<int>(fft_size); x < static_cast<int>(fft_size) + 5; ++x) {
            size_t idx = ((r + scale_height) * img_width + x) * 3;
            image_data[idx] = 255; image_data[idx+1] = 255; image_data[idx+2] = 255;
        }
    }

    std::cout << "Sparar till: " << output_path << "...\n";
    stbi_write_png(output_path.c_str(), img_width, img_height, 3, image_data.data(), img_width * 3);
    std::cout << "Klart! Genererade en bild på " << img_width << "x" << img_height << " px.\n";

    return 0;
}
