#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <vector>

#include <QColor>

namespace iq_analyzer {

constexpr int    kDefaultFftSize = 4096;
constexpr double kPi        = 3.14159265358979323846;
constexpr double kNormScale = 1.0 / 32768.0;

inline void FftRadix2InPlace(std::vector<std::complex<double>>& data) {
    const size_t n = data.size();
    if (n < 2U) return;
    for (size_t i = 1U, j = 0U; i < n; ++i) {
        size_t bit = n >> 1U;
        while (j & bit) { j ^= bit; bit >>= 1U; }
        j ^= bit;
        if (i < j) std::swap(data[i], data[j]);
    }
    for (size_t len = 2U; len <= n; len <<= 1U) {
        const double angle = -2.0 * kPi / static_cast<double>(len);
        const std::complex<double> wlen(std::cos(angle), std::sin(angle));
        for (size_t i = 0U; i < n; i += len) {
            std::complex<double> w(1.0, 0.0);
            const size_t half = len >> 1U;
            for (size_t j2 = 0U; j2 < half; ++j2) {
                const std::complex<double> u = data[i + j2];
                const std::complex<double> v = data[i + j2 + half] * w;
                data[i + j2]        = u + v;
                data[i + j2 + half] = u - v;
                w *= wlen;
            }
        }
    }
}

inline double HannWindow(size_t i, size_t n) {
    return 0.5 * (1.0 - std::cos(2.0 * kPi * static_cast<double>(i) /
                                  static_cast<double>(n - 1U)));
}

// 256-entry precomputed rainbow LUT: black→blue→cyan→green→yellow→orange→red→white.
inline QRgb WaterfallColor(double v) {
    static const std::array<QRgb, 256> kLut = []() {
        struct Stop { float pos, r, g, b; };
        constexpr Stop kStops[] = {
            {0.000f,   0,   0,  64},  // dark blue — LUT[0] ≠ black so threshold is visible
            {0.125f,   0,   0, 255},
            {0.250f,   0, 128, 255},
            {0.375f,   0, 255, 255},
            {0.500f,   0, 255,   0},
            {0.625f, 255, 255,   0},
            {0.750f, 255, 128,   0},
            {0.875f, 255,   0,   0},
            {1.000f, 255, 255, 255},
        };
        constexpr int kN = static_cast<int>(sizeof(kStops) / sizeof(kStops[0]));
        std::array<QRgb, 256> lut{};
        for (int idx = 0; idx < 256; ++idx) {
            const float fv = static_cast<float>(idx) / 255.f;
            int i = 0;
            while (i < kN - 2 && fv > kStops[i + 1].pos) ++i;
            const float span = kStops[i + 1].pos - kStops[i].pos;
            const float t = (span > 0.f) ? (fv - kStops[i].pos) / span : 0.f;
            lut[static_cast<size_t>(idx)] = qRgb(
                static_cast<int>(kStops[i].r + t * (kStops[i+1].r - kStops[i].r)),
                static_cast<int>(kStops[i].g + t * (kStops[i+1].g - kStops[i].g)),
                static_cast<int>(kStops[i].b + t * (kStops[i+1].b - kStops[i].b)));
        }
        return lut;
    }();
    const int idx = static_cast<int>(std::clamp(v, 0.0, 1.0) * 255.0 + 0.5);
    return kLut[static_cast<size_t>(idx)];
}

}  // namespace iq_analyzer
