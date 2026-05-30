#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

#include <QColor>

namespace iq_analyzer {

constexpr int    kFftSize   = 4096;
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

// Black → blue → cyan → yellow → red waterfall colormap.
inline QRgb WaterfallColor(double v) {
    v = std::clamp(v, 0.0, 1.0);
    struct Stop { float r, g, b; };
    static constexpr Stop kStops[5] = {
        {0.f, 0.f, 0.f},
        {0.f, 0.f, 1.f},
        {0.f, 1.f, 1.f},
        {1.f, 1.f, 0.f},
        {1.f, 0.f, 0.f},
    };
    const double pos = v * 4.0;
    const int i = std::min(static_cast<int>(pos), 3);
    const double t = pos - i;
    const auto& lo = kStops[i];
    const auto& hi = kStops[i + 1];
    return qRgb(
        static_cast<int>((lo.r + t * (hi.r - lo.r)) * 255.0),
        static_cast<int>((lo.g + t * (hi.g - lo.g)) * 255.0),
        static_cast<int>((lo.b + t * (hi.b - lo.b)) * 255.0));
}

}  // namespace iq_analyzer
