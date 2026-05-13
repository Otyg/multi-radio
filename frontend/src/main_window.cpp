#include "main_window.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <limits>
#include <optional>
#include <sstream>

#include <QByteArray>
#include <QAbstractButton>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QEvent>
#include <QFontDatabase>
#include <QFontMetrics>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QIODevice>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSettings>
#include <QMenu>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSplitter>
#include <QStackedWidget>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#if MR_HAS_QT_MULTIMEDIA
#include <QAudioDevice>
#include <QAudioFormat>
#include <QAudioSink>
#include <QMediaDevices>
#endif

namespace multi_radio {

namespace {

constexpr int kFixedModeTabIndex = 0;
constexpr int kScanRangeModeTabIndex = 1;
constexpr int kScanListModeTabIndex = 2;
constexpr int kAirMarineModeTabIndex = 3;
constexpr int kGlobalSettingsTabIndex = 4;
constexpr double kDefaultScanListSquelchDb = -67.5;
constexpr double kScanSpectrumFloorDb   = -90.0;
constexpr double kScanSpectrumCeilingDb = -20.0;
constexpr int kAudioPrefillMs = 180;
constexpr int kAudioPrefillMaxWaitMs = 450;
constexpr int kAudioMinStartupMs = 40;
constexpr int kAudioFrontendStatsIntervalMs = 1000;
constexpr int kAudioSinkBufferMs = 1200;
constexpr int kAudioPendingMaxMs = 1500;
constexpr int kAudioDrainIntervalMs = 10;
constexpr int kAudioBytesPerSample = 2;
constexpr int kAudioGapFillLowWaterMs = 40;
constexpr double kAudioGapFillMaxMs = 40.0;

int DefaultBandwidthHzForModulation(v1::Modulation modulation) {
  switch (modulation) {
    case v1::MODULATION_AM:   return 10000;
    case v1::MODULATION_WFM:  return 180000;
    case v1::MODULATION_FSK:  return 12500;
    case v1::MODULATION_GMSK: return 12500;
    case v1::MODULATION_VDES_ASM: return 25000;
    case v1::MODULATION_PPM:  return 500000;
    case v1::MODULATION_ADSB:     return 2000000;
    case v1::MODULATION_AIS_DUAL: return 200000;
    case v1::MODULATION_NFM:
    case v1::MODULATION_UNSPECIFIED:
    default:                  return 12500;
  }
}

QString ModulationLabel(v1::Modulation modulation) {
  switch (modulation) {
    case v1::MODULATION_AM:   return "AM";
    case v1::MODULATION_WFM:  return "WFM";
    case v1::MODULATION_FSK:  return "FSK";
    case v1::MODULATION_GMSK: return "GMSK";
    case v1::MODULATION_VDES_ASM: return "VDES ASM";
    case v1::MODULATION_PPM:  return "PPM";
    case v1::MODULATION_ADSB:     return "ADS-B";
    case v1::MODULATION_AIS_DUAL: return "AIS Dual";
    case v1::MODULATION_NFM:
    case v1::MODULATION_UNSPECIFIED:
    default:                  return "NFM";
  }
}

v1::Modulation ModulationFromText(const QString& text) {
  const QString upper = text.trimmed().toUpper();
  if (upper == "AM") {
    return v1::MODULATION_AM;
  }
  if (upper == "WFM") {
    return v1::MODULATION_WFM;
  }
  if (upper == "AIS DUAL" || upper == "AIS_DUAL" || upper == "AISDUAL") {
    return v1::MODULATION_AIS_DUAL;
  }
  if (upper == "VDES ASM" || upper == "VDES_ASM" || upper == "VDESASM" || upper == "VDES") {
    return v1::MODULATION_VDES_ASM;
  }
  return v1::MODULATION_NFM;
}

bool TryParseCsvModulation(const QString& text, v1::Modulation* out) {
  if (out == nullptr) {
    return false;
  }
  const QString upper = text.trimmed().toUpper();
  if (upper == "AM") {
    *out = v1::MODULATION_AM;
    return true;
  }
  if (upper == "WFM" || upper == "WIDEFM" || upper == "FM_BROADCAST") {
    *out = v1::MODULATION_WFM;
    return true;
  }
  if (upper == "NFM" || upper == "FM") {
    *out = v1::MODULATION_NFM;
    return true;
  }
  if (upper == "VDES" || upper == "VDES_ASM" || upper == "VDES ASM") {
    *out = v1::MODULATION_VDES_ASM;
    return true;
  }
  return false;
}

v1::Modulation FixedModulationFromCombo(const QComboBox* combo) {
  if (combo == nullptr) {
    return v1::MODULATION_WFM;
  }
  bool value_ok = false;
  const int modulation_value = combo->currentData().toInt(&value_ok);
  if (!value_ok) {
    return v1::MODULATION_WFM;
  }
  const auto modulation = static_cast<v1::Modulation>(modulation_value);
  if (modulation == v1::MODULATION_NFM      || modulation == v1::MODULATION_WFM  ||
      modulation == v1::MODULATION_AM       || modulation == v1::MODULATION_FSK  ||
      modulation == v1::MODULATION_GMSK     || modulation == v1::MODULATION_PPM  ||
      modulation == v1::MODULATION_ADSB     || modulation == v1::MODULATION_AIS_DUAL ||
      modulation == v1::MODULATION_VDES_ASM) {
    return modulation;
  }
  return v1::MODULATION_WFM;
}

bool ResampleMonoPcmS16Le(const QByteArray& input_pcm, int input_sample_rate_hz,
                          int output_sample_rate_hz, double* next_source_pos,
                          bool* has_prev_sample, int16_t* prev_sample,
                          QByteArray* output_pcm) {
  if (output_pcm == nullptr) {
    return false;
  }
  output_pcm->clear();
  if (next_source_pos == nullptr || has_prev_sample == nullptr || prev_sample == nullptr) {
    return false;
  }
  if (input_pcm.isEmpty() || input_sample_rate_hz <= 0 || output_sample_rate_hz <= 0) {
    return false;
  }
  const int input_byte_count = input_pcm.size() - (input_pcm.size() % kAudioBytesPerSample);
  if (input_byte_count <= 0) {
    return false;
  }
  if (input_sample_rate_hz == output_sample_rate_hz) {
    output_pcm->append(input_pcm.constData(), input_byte_count);
    return true;
  }

  const int input_sample_count = input_byte_count / kAudioBytesPerSample;
  if (input_sample_count <= 0) {
    return false;
  }

  std::vector<int16_t> input_samples(static_cast<size_t>(input_sample_count), 0);
  for (int sample_idx = 0; sample_idx < input_sample_count; ++sample_idx) {
    const int byte_idx = sample_idx * kAudioBytesPerSample;
    const uint16_t lo = static_cast<uint8_t>(input_pcm.at(byte_idx));
    const uint16_t hi = static_cast<uint8_t>(input_pcm.at(byte_idx + 1));
    input_samples[static_cast<size_t>(sample_idx)] =
        static_cast<int16_t>((hi << 8) | lo);
  }

  const double ratio = static_cast<double>(input_sample_rate_hz) /
                       static_cast<double>(output_sample_rate_hz);
  if (ratio <= 0.0 || !std::isfinite(ratio)) {
    return false;
  }

  const bool have_prev = *has_prev_sample;
  const double source_base_offset = have_prev ? 1.0 : 0.0;
  const double source_virtual_size = static_cast<double>(input_sample_count) + source_base_offset;
  if (source_virtual_size < 2.0) {
    output_pcm->append(input_pcm.constData(), input_byte_count);
    *prev_sample = input_samples.back();
    *has_prev_sample = true;
    *next_source_pos = 0.0;
    return true;
  }

  auto source_at = [&](size_t virtual_idx) -> int16_t {
    if (have_prev) {
      if (virtual_idx == 0) {
        return *prev_sample;
      }
      const size_t source_idx = std::min(virtual_idx - 1, input_samples.size() - 1);
      return input_samples[source_idx];
    }
    const size_t source_idx = std::min(virtual_idx, input_samples.size() - 1);
    return input_samples[source_idx];
  };

  const double estimated_output_samples = (source_virtual_size * static_cast<double>(output_sample_rate_hz)) /
                                          static_cast<double>(input_sample_rate_hz);
  std::vector<int16_t> output_samples;
  output_samples.reserve(std::max<size_t>(
      1, static_cast<size_t>(std::ceil(estimated_output_samples)) + 2U));

  const double step = std::max(1.0e-12, ratio);
  double pos = std::clamp(*next_source_pos, 0.0, step) + source_base_offset;
  for (; pos < source_virtual_size; pos += step) {
    const size_t index = static_cast<size_t>(pos);
    const size_t next_index = std::min(index + 1, static_cast<size_t>(source_virtual_size - 1.0));
    const double frac = pos - static_cast<double>(index);
    const double mixed =
        static_cast<double>(source_at(index)) +
        (static_cast<double>(source_at(next_index)) - static_cast<double>(source_at(index))) *
            frac;
    output_samples.push_back(static_cast<int16_t>(
        std::lrint(std::clamp(mixed, -32768.0, 32767.0))));
  }
  *next_source_pos = pos - source_virtual_size;
  if (!std::isfinite(*next_source_pos) || *next_source_pos < 0.0 || *next_source_pos >= step) {
    *next_source_pos = 0.0;
  }
  *prev_sample = input_samples.back();
  *has_prev_sample = true;
  if (output_samples.empty()) {
    output_samples.push_back(input_samples.back());
  }

  output_pcm->resize(static_cast<int>(output_samples.size() * kAudioBytesPerSample));
  for (size_t idx = 0; idx < output_samples.size(); ++idx) {
    const uint16_t sample_bits = static_cast<uint16_t>(output_samples[idx]);
    (*output_pcm)[static_cast<int>(idx * kAudioBytesPerSample)] =
        static_cast<char>(sample_bits & 0xFFU);
    (*output_pcm)[static_cast<int>(idx * kAudioBytesPerSample + 1)] =
        static_cast<char>((sample_bits >> 8) & 0xFFU);
  }
  return true;
}

QString TokenValue(const QString& message, const QString& key) {
  const QStringList tokens = message.split(' ', Qt::SkipEmptyParts);
  const QString prefix = key + "=";
  for (const QString& token : tokens) {
    if (token.startsWith(prefix)) {
      return token.mid(prefix.size());
    }
  }
  return {};
}

v1::RadioMode ModeFromTabIndex(int tab_index) {
  switch (tab_index) {
    case kFixedModeTabIndex:
      return v1::RADIO_MODE_FIXED;
    case kScanRangeModeTabIndex:
      return v1::RADIO_MODE_SCAN_RANGE;
    case kScanListModeTabIndex:
      return v1::RADIO_MODE_SCAN_LIST;
    case kAirMarineModeTabIndex:
      // RADAR_VIEW uses scan-list mode with its own channel set (radar_scan_list).
      return v1::RADIO_MODE_SCAN_LIST;
    default:
      return v1::RADIO_MODE_FIXED;
  }
}

QString ToLocalTime(quint64 unix_ms) {
  return QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(unix_ms)).toLocalTime().toString("HH:mm:ss");
}

bool ParseSeries(const QString& value, std::vector<double>* out) {
  if (out == nullptr) {
    return false;
  }
  out->clear();
  if (value.isEmpty()) {
    return false;
  }
  const QStringList tokens = value.split(',', Qt::SkipEmptyParts);
  out->reserve(tokens.size());
  for (const QString& token : tokens) {
    bool ok = false;
    const double parsed = token.toDouble(&ok);
    if (!ok) {
      return false;
    }
    out->push_back(parsed);
  }
  return !out->empty();
}

bool DecodeInt16LeBytes(const QByteArray& bytes, std::vector<int16_t>* out) {
  if (out == nullptr) {
    return false;
  }
  out->clear();
  if (bytes.isEmpty()) {
    return false;
  }
  const int byte_count = bytes.size() - (bytes.size() % 2);
  if (byte_count <= 0) {
    return false;
  }
  const int sample_count = byte_count / 2;
  out->resize(static_cast<size_t>(sample_count));
  for (int i = 0; i < sample_count; ++i) {
    const uint16_t lo = static_cast<uint8_t>(bytes.at(i * 2));
    const uint16_t hi = static_cast<uint8_t>(bytes.at(i * 2 + 1));
    (*out)[static_cast<size_t>(i)] = static_cast<int16_t>((hi << 8) | lo);
  }
  return true;
}

size_t LargestPowerOfTwoLeq(size_t value) {
  if (value < 2U) {
    return 0U;
  }
  size_t out = 1U;
  while ((out << 1U) <= value) {
    out <<= 1U;
  }
  return out;
}

void FftRadix2InPlace(std::vector<std::complex<double>>* data) {
  if (data == nullptr) {
    return;
  }
  const size_t n = data->size();
  if (n < 2U) {
    return;
  }

  // Bit-reversal permutation
  for (size_t i = 1U, j = 0U; i < n; ++i) {
    size_t bit = n >> 1U;
    while (j & bit) {
      j ^= bit;
      bit >>= 1U;
    }
    j ^= bit;
    if (i < j) {
      std::swap((*data)[i], (*data)[j]);
    }
  }

  // Iterative radix-2 Cooley-Tukey
  constexpr double kPi = 3.14159265358979323846;
  for (size_t len = 2U; len <= n; len <<= 1U) {
    const double angle = -2.0 * kPi / static_cast<double>(len);
    const std::complex<double> w_len(std::cos(angle), std::sin(angle));
    for (size_t i = 0U; i < n; i += len) {
      std::complex<double> w(1.0, 0.0);
      const size_t half = len >> 1U;
      for (size_t j = 0U; j < half; ++j) {
        const std::complex<double> u = (*data)[i + j];
        const std::complex<double> v = (*data)[i + j + half] * w;
        (*data)[i + j] = u + v;
        (*data)[i + j + half] = u - v;
        w *= w_len;
      }
    }
  }
}


std::vector<double> ResampleVectorLinear(const std::vector<double>& source, size_t target_size) {
  if (target_size == 0U) {
    return {};
  }
  if (source.empty()) {
    return std::vector<double>(target_size, 0.0);
  }
  if (source.size() == target_size) {
    return source;
  }
  if (source.size() == 1U) {
    return std::vector<double>(target_size, source.front());
  }
  std::vector<double> out(target_size, 0.0);
  for (size_t i = 0; i < target_size; ++i) {
    const double t = (target_size <= 1U) ? 0.0 : static_cast<double>(i) / static_cast<double>(target_size - 1U);
    const double source_pos = t * static_cast<double>(source.size() - 1U);
    const size_t left = static_cast<size_t>(std::floor(source_pos));
    const size_t right = std::min(left + 1U, source.size() - 1U);
    const double frac = source_pos - static_cast<double>(left);
    out[i] = source[left] + (source[right] - source[left]) * frac;
  }
  return out;
}

// Fixed dBFS scale: 0.0 = floor_db, 1.0 = ceiling_db.
std::vector<double> BuildFixedScaleSpectrumFromComplex(
    const std::vector<std::complex<double>>& complex_samples, int spectrum_bins,
    double floor_db, double ceiling_db) {
  if (complex_samples.size() < 16 || spectrum_bins <= 0) {
    return {};
  }
  const size_t requested_bins = static_cast<size_t>(std::max(32, spectrum_bins));
  const size_t desired_fft = std::max<size_t>(64U, requested_bins * 2U);
  const size_t available = complex_samples.size();
  const size_t fft_size = LargestPowerOfTwoLeq(std::min(available, desired_fft));
  if (fft_size < 64U) {
    return {};
  }
  std::vector<std::complex<double>> fft_in(fft_size);
  const size_t start = available - fft_size;
  constexpr double kPi = 3.14159265358979323846;
  for (size_t i = 0; i < fft_size; ++i) {
    const double phase = (2.0 * kPi * static_cast<double>(i)) / static_cast<double>(fft_size - 1U);
    const double w = 0.5 * (1.0 - std::cos(phase));
    fft_in[i] = complex_samples[start + i] * w;
  }
  FftRadix2InPlace(&fft_in);

  const size_t half = fft_size / 2U;
  const double span = std::max(1.0, ceiling_db - floor_db);
  std::vector<double> out(fft_size, 0.0);
  for (size_t i = 0; i < fft_size; ++i) {
    const size_t idx = (i + half) % fft_size;
    const double magnitude = std::abs(fft_in[idx]) / static_cast<double>(fft_size);
    const double db = 20.0 * std::log10(std::max(1.0e-12, magnitude));
    out[i] = std::clamp((db - floor_db) / span, 0.0, 1.0);
  }
  if (out.size() != requested_bins) {
    out = ResampleVectorLinear(out, requested_bins);
  }
  return out;
}

void BuildReceiverVisualizationFrame(const QByteArray& interleaved_iq_s16le, int spectrum_bins,
                                     bool apply_dc_suppression, std::vector<double>* waveform,
                                     std::vector<double>* spectrum, double* signal_level_db) {
  if (waveform == nullptr || spectrum == nullptr || signal_level_db == nullptr) {
    return;
  }
  waveform->clear();
  spectrum->clear();
  *signal_level_db = -120.0;

  std::vector<int16_t> iq_s16;
  if (!DecodeInt16LeBytes(interleaved_iq_s16le, &iq_s16) || iq_s16.size() < 16) {
    return;
  }
  if ((iq_s16.size() % 2U) != 0U) {
    iq_s16.pop_back();
  }
  const size_t iq_pairs = iq_s16.size() / 2U;
  if (iq_pairs < 8U) {
    return;
  }

  std::vector<std::complex<double>> complex_samples;
  complex_samples.reserve(iq_pairs);
  double i_sum = 0.0;
  double q_sum = 0.0;
  double power_sum = 0.0;
  for (size_t i = 0; i < iq_pairs; ++i) {
    const double i_norm = static_cast<double>(iq_s16[i * 2U]) / 32768.0;
    const double q_norm = static_cast<double>(iq_s16[i * 2U + 1U]) / 32768.0;
    if (apply_dc_suppression) {
      i_sum += i_norm;
      q_sum += q_norm;
      complex_samples.emplace_back(0.0, 0.0);
    } else {
      complex_samples.emplace_back(i_norm, q_norm);
    }
    power_sum += (i_norm * i_norm) + (q_norm * q_norm);
    waveform->push_back(std::clamp(0.5 + (0.5 * i_norm), 0.0, 1.0));
  }

  if (apply_dc_suppression) {
    const double inv_count = 1.0 / static_cast<double>(iq_pairs);
    const double i_mean = i_sum * inv_count;
    const double q_mean = q_sum * inv_count;
    for (size_t i = 0; i < iq_pairs; ++i) {
      const double i_norm = static_cast<double>(iq_s16[i * 2U]) / 32768.0 - i_mean;
      const double q_norm = static_cast<double>(iq_s16[i * 2U + 1U]) / 32768.0 - q_mean;
      complex_samples[i] = std::complex<double>(i_norm, q_norm);
    }
  }

  const double rms = std::sqrt(power_sum / static_cast<double>(iq_pairs));
  *signal_level_db = std::clamp(20.0 * std::log10(std::max(1.0e-9, rms)), -120.0, 0.0);
  *spectrum = BuildFixedScaleSpectrumFromComplex(complex_samples, spectrum_bins,
                                                 kScanSpectrumFloorDb, kScanSpectrumCeilingDb);
}

void BuildDemodVisualizationFrame(const QByteArray& pcm_s16le, int spectrum_bins, std::vector<double>* waveform,
                                  std::vector<double>* spectrum) {
  if (waveform == nullptr || spectrum == nullptr) {
    return;
  }
  waveform->clear();
  spectrum->clear();

  std::vector<int16_t> pcm;
  if (!DecodeInt16LeBytes(pcm_s16le, &pcm) || pcm.size() < 16) {
    return;
  }

  std::vector<std::complex<double>> complex_samples;
  complex_samples.reserve(pcm.size());
  for (const int16_t sample : pcm) {
    const double normalized = static_cast<double>(sample) / 32768.0;
    waveform->push_back(std::clamp(0.5 + (0.5 * normalized), 0.0, 1.0));
    complex_samples.emplace_back(normalized, 0.0);
  }
  *spectrum = BuildFixedScaleSpectrumFromComplex(
      complex_samples, spectrum_bins, kScanSpectrumFloorDb, kScanSpectrumCeilingDb);
}

bool ParseVisualizationFrameEvent(const QString& message, double* peak_hz, double* peak_strength,
                                  std::vector<double>* waveform, std::vector<double>* spectrum,
                                  SignalVisualizationWidget::SpectrumSource* source,
                                  double* frame_frequency_start_hz,
                                  double* frame_frequency_end_hz) {
  if (peak_hz == nullptr || peak_strength == nullptr || waveform == nullptr || spectrum == nullptr) {
    return false;
  }
  if (source == nullptr || frame_frequency_start_hz == nullptr || frame_frequency_end_hz == nullptr) {
    return false;
  }
  if (!message.startsWith("VIZ_FRAME ")) {
    return false;
  }

  const QStringList tokens = message.split(' ', Qt::SkipEmptyParts);
  if (tokens.size() < 5) {
    return false;
  }

  bool peak_hz_ok = false;
  bool peak_strength_ok = false;
  bool waveform_ok = false;
  bool spectrum_ok = false;
  bool frequency_start_ok = false;
  bool frequency_end_ok = false;
  double parsed_peak_hz = 0.0;
  double parsed_peak_strength = 0.0;
  double parsed_frequency_start_hz = 0.0;
  double parsed_frequency_end_hz = 20000.0;
  std::vector<double> parsed_waveform;
  std::vector<double> parsed_spectrum;
  SignalVisualizationWidget::SpectrumSource parsed_source =
      SignalVisualizationWidget::SpectrumSource::kDemodulated;
  for (int i = 1; i < tokens.size(); ++i) {
    const QString token = tokens[i];
    if (token.startsWith("peak_hz=")) {
      parsed_peak_hz = token.mid(8).toDouble(&peak_hz_ok);
    } else if (token.startsWith("peak_strength=")) {
      parsed_peak_strength = token.mid(14).toDouble(&peak_strength_ok);
    } else if (token.startsWith("source=")) {
      const QString source_value = token.mid(7).trimmed().toLower();
      if (source_value == "receiver") {
        parsed_source = SignalVisualizationWidget::SpectrumSource::kReceiverInput;
      } else {
        parsed_source = SignalVisualizationWidget::SpectrumSource::kDemodulated;
      }
    } else if (token.startsWith("start_hz=")) {
      parsed_frequency_start_hz = token.mid(9).toDouble(&frequency_start_ok);
    } else if (token.startsWith("end_hz=")) {
      parsed_frequency_end_hz = token.mid(7).toDouble(&frequency_end_ok);
    } else if (token.startsWith("waveform=")) {
      waveform_ok = ParseSeries(token.mid(9), &parsed_waveform);
    } else if (token.startsWith("spectrum=")) {
      spectrum_ok = ParseSeries(token.mid(9), &parsed_spectrum);
    }
  }

  if (!peak_hz_ok || !peak_strength_ok || !waveform_ok || !spectrum_ok) {
    return false;
  }
  *peak_hz = parsed_peak_hz;
  *peak_strength = std::clamp(parsed_peak_strength, 0.0, 1.0);
  *waveform = std::move(parsed_waveform);
  *spectrum = std::move(parsed_spectrum);
  *source = parsed_source;
  if (frequency_start_ok && frequency_end_ok && parsed_frequency_end_hz > parsed_frequency_start_hz) {
    *frame_frequency_start_hz = parsed_frequency_start_hz;
    *frame_frequency_end_hz = parsed_frequency_end_hz;
  } else {
    *frame_frequency_start_hz = 0.0;
    *frame_frequency_end_hz = 20000.0;
  }
  return true;
}

bool EnvFlagEnabled(const char* key) {
  const char* value = std::getenv(key);
  if (value == nullptr) {
    return false;
  }
  const QString normalized = QString::fromUtf8(value).trimmed().toLower();
  return normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on";
}

bool IsWslEnvironment() {
  return std::getenv("WSL_DISTRO_NAME") != nullptr || std::getenv("WSL_INTEROP") != nullptr;
}

struct ParsedAisNmeaSentence {
  std::vector<int> bits;
  QString channel;
};

int DecodeAisSixBitValue(QChar c) {
  const int ascii = c.unicode();
  if (ascii < 48 || ascii > 119) {
    return -1;
  }
  int value = ascii - 48;
  if (value > 40) {
    value -= 8;
  }
  if (value < 0 || value > 63) {
    return -1;
  }
  return value;
}

bool ReadUnsignedBits(const std::vector<int>& bits, int start, int length, uint32_t* out) {
  if (out == nullptr || start < 0 || length <= 0 || length > 32) {
    return false;
  }
  const size_t begin = static_cast<size_t>(start);
  const size_t end = begin + static_cast<size_t>(length);
  if (end > bits.size()) {
    return false;
  }
  uint32_t value = 0;
  for (size_t idx = begin; idx < end; ++idx) {
    value = (value << 1U) | static_cast<uint32_t>(bits[idx] != 0 ? 1 : 0);
  }
  *out = value;
  return true;
}

bool ReadSignedBits(const std::vector<int>& bits, int start, int length, int32_t* out) {
  if (out == nullptr || length <= 0 || length >= 32) {
    return false;
  }
  uint32_t raw = 0;
  if (!ReadUnsignedBits(bits, start, length, &raw)) {
    return false;
  }
  const uint32_t sign_bit = 1U << static_cast<uint32_t>(length - 1);
  if ((raw & sign_bit) == 0U) {
    *out = static_cast<int32_t>(raw);
    return true;
  }
  const uint32_t mask = (1U << static_cast<uint32_t>(length)) - 1U;
  *out = static_cast<int32_t>(raw | ~mask);
  return true;
}

QString DecodeAisSixBitText(const std::vector<int>& bits, int start, int length) {
  if (start < 0 || length <= 0 || (length % 6) != 0) {
    return {};
  }
  if (static_cast<size_t>(start + length) > bits.size()) {
    return {};
  }
  static const char* kAisCharset =
      "@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_ !\"#$%&'()*+,-./0123456789:;<=>?";
  QString out;
  out.reserve(length / 6);
  for (int i = 0; i < length; i += 6) {
    uint32_t value = 0;
    if (!ReadUnsignedBits(bits, start + i, 6, &value)) {
      return {};
    }
    out.append(QChar::fromLatin1(kAisCharset[value]));
  }
  out.replace('@', ' ');
  return out.trimmed();
}

std::optional<double> DecodeLonDeg(const std::vector<int>& bits, int start, int length) {
  int32_t raw = 0;
  if (!ReadSignedBits(bits, start, length, &raw)) {
    return std::nullopt;
  }
  const double degrees = static_cast<double>(raw) / 600000.0;
  if (degrees < -180.0 || degrees > 180.0) {
    return std::nullopt;
  }
  return degrees;
}

std::optional<double> DecodeLatDeg(const std::vector<int>& bits, int start, int length) {
  int32_t raw = 0;
  if (!ReadSignedBits(bits, start, length, &raw)) {
    return std::nullopt;
  }
  const double degrees = static_cast<double>(raw) / 600000.0;
  if (degrees < -90.0 || degrees > 90.0) {
    return std::nullopt;
  }
  return degrees;
}

std::optional<double> DecodeTenths(const std::vector<int>& bits, int start, int length,
                                   uint32_t unavailable) {
  uint32_t raw = 0;
  if (!ReadUnsignedBits(bits, start, length, &raw) || raw == unavailable) {
    return std::nullopt;
  }
  return static_cast<double>(raw) / 10.0;
}

std::optional<int> DecodeUnsignedMaybe(const std::vector<int>& bits, int start, int length,
                                       uint32_t unavailable) {
  uint32_t raw = 0;
  if (!ReadUnsignedBits(bits, start, length, &raw) || raw == unavailable) {
    return std::nullopt;
  }
  return static_cast<int>(raw);
}

bool ParseAisNmeaSentence(const QString& sentence, ParsedAisNmeaSentence* out) {
  if (out == nullptr) {
    return false;
  }
  const QString trimmed = sentence.trimmed();
  if (trimmed.isEmpty()) {
    return false;
  }

  const int star = trimmed.indexOf('*');
  QString body = (star >= 0) ? trimmed.left(star) : trimmed;
  if (body.startsWith('!') || body.startsWith('$')) {
    body = body.mid(1);
  }
  const QStringList parts = body.split(',');
  if (parts.size() < 7) {
    return false;
  }
  if (!(parts[0].endsWith("VDM") || parts[0].endsWith("VDO"))) {
    return false;
  }
  bool fragments_ok = false;
  const int fragments = parts[1].toInt(&fragments_ok);
  bool fragment_no_ok = false;
  const int fragment_no = parts[2].toInt(&fragment_no_ok);
  if (!fragments_ok || !fragment_no_ok || fragments <= 0 || fragment_no <= 0 || fragments != 1 ||
      fragment_no != 1) {
    return false;
  }

  bool fill_ok = false;
  const int fill_bits = parts[6].toInt(&fill_ok);
  if (!fill_ok || fill_bits < 0 || fill_bits > 5) {
    return false;
  }
  const QString sixbit_payload = parts[5];
  if (sixbit_payload.isEmpty()) {
    return false;
  }

  std::vector<int> bits;
  bits.reserve(static_cast<size_t>(sixbit_payload.size()) * 6U);
  for (QChar c : sixbit_payload) {
    const int value = DecodeAisSixBitValue(c);
    if (value < 0) {
      return false;
    }
    for (int bit = 5; bit >= 0; --bit) {
      bits.push_back((value >> bit) & 1);
    }
  }
  if (fill_bits > static_cast<int>(bits.size())) {
    return false;
  }
  if (fill_bits > 0) {
    bits.resize(bits.size() - static_cast<size_t>(fill_bits));
  }

  out->bits = std::move(bits);
  out->channel = parts[4].trimmed();
  return !out->bits.empty();
}

QString NavigationStatusToString(uint32_t status) {
  switch (status) {
    case 0:
      return "under_way";
    case 1:
      return "at_anchor";
    case 2:
      return "not_under_command";
    case 3:
      return "restricted_maneuverability";
    case 4:
      return "constrained_by_draught";
    case 5:
      return "moored";
    case 6:
      return "aground";
    case 7:
      return "fishing";
    case 8:
      return "under_way_sailing";
    case 15:
      return "not_defined";
    default:
      return QString("status_%1").arg(status);
  }
}

QString BuildAisDecodedSummary(const QString& sentence, const QVariantMap& fields) {
  ParsedAisNmeaSentence parsed;
  if (!ParseAisNmeaSentence(sentence, &parsed)) {
    return {};
  }

  auto field_text = [&fields](const QString& key) -> QString {
    if (!fields.contains(key)) {
      return {};
    }
    return fields.value(key).toString();
  };

  uint32_t message_type = 0;
  bool type_ok = false;
  const QString msg_type_text = field_text("msg_type");
  if (!msg_type_text.isEmpty()) {
    message_type = msg_type_text.toUInt(&type_ok);
  }
  if (!type_ok && !ReadUnsignedBits(parsed.bits, 0, 6, &message_type)) {
    return {};
  }

  uint32_t mmsi = 0;
  bool mmsi_ok = false;
  const QString mmsi_text = field_text("mmsi");
  if (!mmsi_text.isEmpty()) {
    mmsi = mmsi_text.toUInt(&mmsi_ok);
  }
  if (!mmsi_ok) {
    ReadUnsignedBits(parsed.bits, 8, 30, &mmsi);
  }

  QStringList summary_parts;
  summary_parts << QString("type=%1").arg(message_type);
  if (mmsi > 0) {
    summary_parts << QString("mmsi=%1").arg(mmsi);
  }
  if (!parsed.channel.isEmpty()) {
    summary_parts << QString("radio_ch=%1").arg(parsed.channel);
  }

  auto append_double = [&summary_parts](const QString& key, const std::optional<double>& value,
                                        int precision) {
    if (!value.has_value()) {
      return;
    }
    summary_parts << QString("%1=%2").arg(key).arg(QString::number(*value, 'f', precision));
  };
  auto append_int = [&summary_parts](const QString& key, const std::optional<int>& value) {
    if (!value.has_value()) {
      return;
    }
    summary_parts << QString("%1=%2").arg(key).arg(*value);
  };

  if (message_type >= 1 && message_type <= 3) {
    uint32_t nav_status = 0;
    if (ReadUnsignedBits(parsed.bits, 38, 4, &nav_status)) {
      summary_parts << QString("nav=%1").arg(NavigationStatusToString(nav_status));
    }
    append_double("sog_kn", DecodeTenths(parsed.bits, 50, 10, 1023), 1);
    append_double("lon", DecodeLonDeg(parsed.bits, 61, 28), 5);
    append_double("lat", DecodeLatDeg(parsed.bits, 89, 27), 5);
    append_double("cog_deg", DecodeTenths(parsed.bits, 116, 12, 3600), 1);
    append_int("hdg_deg", DecodeUnsignedMaybe(parsed.bits, 128, 9, 511));
  } else if (message_type == 5) {
    uint32_t imo = 0;
    if (ReadUnsignedBits(parsed.bits, 40, 30, &imo) && imo > 0) {
      summary_parts << QString("imo=%1").arg(imo);
    }
    const QString call_sign = DecodeAisSixBitText(parsed.bits, 70, 42);
    if (!call_sign.isEmpty()) {
      summary_parts << QString("callsign=%1").arg(call_sign);
    }
    const QString vessel_name = DecodeAisSixBitText(parsed.bits, 112, 120);
    if (!vessel_name.isEmpty()) {
      summary_parts << QString("name=%1").arg(vessel_name);
    }
    uint32_t ship_type = 0;
    if (ReadUnsignedBits(parsed.bits, 232, 8, &ship_type) && ship_type > 0) {
      summary_parts << QString("ship_type=%1").arg(ship_type);
    }
    uint32_t to_bow = 0;
    uint32_t to_stern = 0;
    uint32_t to_port = 0;
    uint32_t to_starboard = 0;
    if (ReadUnsignedBits(parsed.bits, 240, 9, &to_bow) &&
        ReadUnsignedBits(parsed.bits, 249, 9, &to_stern) &&
        ReadUnsignedBits(parsed.bits, 258, 6, &to_port) &&
        ReadUnsignedBits(parsed.bits, 264, 6, &to_starboard)) {
      summary_parts << QString("dim_m=%1/%2/%3/%4").arg(to_bow).arg(to_stern).arg(to_port).arg(to_starboard);
    }
    const QString destination = DecodeAisSixBitText(parsed.bits, 302, 120);
    if (!destination.isEmpty()) {
      summary_parts << QString("dest=%1").arg(destination);
    }
  } else if (message_type == 18) {
    append_double("sog_kn", DecodeTenths(parsed.bits, 46, 10, 1023), 1);
    append_double("lon", DecodeLonDeg(parsed.bits, 57, 28), 5);
    append_double("lat", DecodeLatDeg(parsed.bits, 85, 27), 5);
    append_double("cog_deg", DecodeTenths(parsed.bits, 112, 12, 3600), 1);
    append_int("hdg_deg", DecodeUnsignedMaybe(parsed.bits, 124, 9, 511));
  } else if (message_type == 19) {
    append_double("sog_kn", DecodeTenths(parsed.bits, 46, 10, 1023), 1);
    append_double("lon", DecodeLonDeg(parsed.bits, 57, 28), 5);
    append_double("lat", DecodeLatDeg(parsed.bits, 85, 27), 5);
    append_double("cog_deg", DecodeTenths(parsed.bits, 112, 12, 3600), 1);
    append_int("hdg_deg", DecodeUnsignedMaybe(parsed.bits, 124, 9, 511));
    const QString vessel_name = DecodeAisSixBitText(parsed.bits, 143, 120);
    if (!vessel_name.isEmpty()) {
      summary_parts << QString("name=%1").arg(vessel_name);
    }
    uint32_t ship_type = 0;
    if (ReadUnsignedBits(parsed.bits, 263, 8, &ship_type) && ship_type > 0) {
      summary_parts << QString("ship_type=%1").arg(ship_type);
    }
  } else if (message_type == 24) {
    uint32_t part_no = 0;
    if (ReadUnsignedBits(parsed.bits, 38, 2, &part_no)) {
      summary_parts << QString("part=%1").arg(part_no);
      if (part_no == 0) {
        const QString vessel_name = DecodeAisSixBitText(parsed.bits, 40, 120);
        if (!vessel_name.isEmpty()) {
          summary_parts << QString("name=%1").arg(vessel_name);
        }
      } else if (part_no == 1) {
        uint32_t ship_type = 0;
        if (ReadUnsignedBits(parsed.bits, 40, 8, &ship_type) && ship_type > 0) {
          summary_parts << QString("ship_type=%1").arg(ship_type);
        }
        const QString vendor = DecodeAisSixBitText(parsed.bits, 48, 42);
        if (!vendor.isEmpty()) {
          summary_parts << QString("vendor=%1").arg(vendor);
        }
        const QString call_sign = DecodeAisSixBitText(parsed.bits, 90, 42);
        if (!call_sign.isEmpty()) {
          summary_parts << QString("callsign=%1").arg(call_sign);
        }
      }
    }
  }

  return summary_parts.join(" ");
}

QString BuildDscDecodedSummary(const QVariantMap& fields) {
  auto field_text = [&fields](const QString& key) -> QString {
    if (!fields.contains(key)) {
      return {};
    }
    return fields.value(key).toString();
  };

  const QString kind = field_text("kind");
  if (kind.isEmpty() || kind == "candidate") {
    return {};
  }

  QStringList summary_parts;
  const QString format = field_text("format_label");
  const QString category = field_text("category_label");
  const QString address = field_text("address_digits");
  const QString self_id = field_text("self_id_digits");
  const QString tc1 = field_text("telecommand_1");
  const QString tc2 = field_text("telecommand_2");
  const QString eos = field_text("eos_label");
  const QString validity = field_text("validity");

  if (!format.isEmpty()) {
    summary_parts << QString("fmt=%1").arg(format);
  }
  if (!category.isEmpty()) {
    summary_parts << QString("cat=%1").arg(category);
  }
  if (!address.isEmpty()) {
    summary_parts << QString("to=%1").arg(address);
  }
  if (!self_id.isEmpty()) {
    summary_parts << QString("from=%1").arg(self_id);
  }
  if (!tc1.isEmpty()) {
    summary_parts << QString("tc1=%1").arg(tc1);
  }
  if (!tc2.isEmpty()) {
    summary_parts << QString("tc2=%1").arg(tc2);
  }
  if (!eos.isEmpty()) {
    summary_parts << QString("eos=%1").arg(eos);
  }
  if (!validity.isEmpty()) {
    summary_parts << QString("validity=%1").arg(validity);
  }

  return summary_parts.join(" ");
}

bool ParseDoubleField(const QVariantMap& fields, const QString& key, double* out) {
  if (out == nullptr) return false;
  if (!fields.contains(key)) return false;
  bool ok = false;
  const double v = fields.value(key).toString().toDouble(&ok);
  if (!ok) return false;
  *out = v;
  return true;
}

QString FieldString(const QVariantMap& fields, const QString& key) {
  return fields.contains(key) ? fields.value(key).toString() : QString();
}

std::vector<RadarFixedObject> LoadFixedObjectsFromSettings() {
  QSettings settings("multi-radio", "multi-radio-client");
  settings.beginGroup("radar_view");
  const QString json = settings.value("fixed_objects_json", "[]").toString();
  settings.endGroup();

  std::vector<RadarFixedObject> out;
  const auto doc = QJsonDocument::fromJson(json.toUtf8());
  if (!doc.isArray()) return out;
  const QJsonArray arr = doc.array();
  out.reserve((size_t)arr.size());
  for (const auto& v : arr) {
    if (!v.isObject()) continue;
    const QJsonObject o = v.toObject();
    RadarFixedObject fo;
    fo.id = o.value("id").toString();
    fo.name = o.value("name").toString();
    fo.lat = o.value("lat").toDouble();
    fo.lon = o.value("lon").toDouble();
    if (fo.id.isEmpty()) fo.id = QString("%1,%2").arg(fo.lat, 0, 'f', 6).arg(fo.lon, 0, 'f', 6);
    out.push_back(fo);
  }
  return out;
}

void SaveFixedObjectsToSettings(const QString& json) {
  QSettings settings("multi-radio", "multi-radio-client");
  settings.beginGroup("radar_view");
  settings.setValue("fixed_objects_json", json);
  settings.endGroup();
}

QString LoadNameAlias(const QString& key) {
  QSettings settings("multi-radio", "multi-radio-client");
  settings.beginGroup("radar_names");
  const QString v = settings.value(key, "").toString();
  settings.endGroup();
  return v;
}

void SaveNameAlias(const QString& key, const QString& value) {
  QSettings settings("multi-radio", "multi-radio-client");
  settings.beginGroup("radar_names");
  settings.setValue(key, value);
  settings.endGroup();
}

QString ActiveScanListSettingsGroup(const QTabWidget* mode_tabs) {
  if (mode_tabs == nullptr) return "scan_list";
  const int idx = mode_tabs->currentIndex();
  return (idx == kAirMarineModeTabIndex) ? "radar_scan_list" : "scan_list";
}

QString PreferNameOrCsOrAlias(const QVariantMap& fields, const QString& id) {
  const QString name = FieldString(fields, "name").trimmed();
  if (!name.isEmpty()) return name;
  const QString cs = FieldString(fields, "call_sign").trimmed();
  if (!cs.isEmpty()) return cs;
  return LoadNameAlias(id).trimmed();
}

}  // namespace

MainWindow::MainWindow(std::string grpc_target, std::string token, QWidget* parent)
    : QMainWindow(parent), client_(std::make_unique<GrpcClient>(std::move(grpc_target), std::move(token), this)) {
  setWindowTitle("Multi-Radio Client");
  resize(1300, 780);

  auto* central = new QWidget(this);
  auto* root_layout = new QVBoxLayout(central);

  auto* top_layout = new QHBoxLayout();

  auto* control_group = new QGroupBox("Receiver Control", central);
  auto* control_layout = new QFormLayout(control_group);

  receiver_combo_ = new QComboBox(control_group);
  receiver_combo_->setVisible(false);  // single-receiver UX for now

  fixed_frequency_edit_ = new QLineEdit("162.025", control_group);
  range_start_edit_ = new QLineEdit("156", control_group);
  range_end_edit_ = new QLineEdit("163", control_group);
  range_fft_size_combo_ = new QComboBox(control_group);
  for (int s : {64, 128, 256, 512, 1024, 2048, 4096}) {
    range_fft_size_combo_->addItem(QString::number(s), QVariant(s));
  }
  range_fft_size_combo_->setCurrentIndex(
      range_fft_size_combo_->findData(QVariant(1024)));

  dwell_ms_spin_ = new QSpinBox(control_group);
  dwell_ms_spin_->setRange(100, 10000);
  dwell_ms_spin_->setValue(500);
  sample_rate_spin_ = new QSpinBox(control_group);
  sample_rate_spin_->setRange(225000, 3200000);
  sample_rate_spin_->setSingleStep(1000);
  sample_rate_spin_->setValue(2048000);
  sample_rate_spin_->setSuffix(" Hz");
  channel_bandwidth_spin_ = new QSpinBox(control_group);
  channel_bandwidth_spin_->setRange(0, 500000);
  channel_bandwidth_spin_->setSingleStep(1000);
  channel_bandwidth_spin_->setValue(30000);
  channel_bandwidth_spin_->setSuffix(" Hz");
  channel_bandwidth_spin_->setSpecialValueText("Off");
  hardware_bandwidth_spin_ = new QSpinBox(control_group);
  hardware_bandwidth_spin_->setRange(0, 3200000);
  hardware_bandwidth_spin_->setSingleStep(1000);
  hardware_bandwidth_spin_->setValue(0);
  hardware_bandwidth_spin_->setSuffix(" Hz");
  hardware_bandwidth_spin_->setSpecialValueText("Auto");
  dc_blocker_checkbox_ = new QCheckBox("Enabled", control_group);
  dc_blocker_checkbox_->setChecked(false);
  dc_blocker_cutoff_spin_ = new QSpinBox(control_group);
  dc_blocker_cutoff_spin_->setRange(1, 5000);
  dc_blocker_cutoff_spin_->setSingleStep(10);
  dc_blocker_cutoff_spin_->setValue(30);
  dc_blocker_cutoff_spin_->setSuffix(" Hz");
  dc_blocker_cutoff_spin_->setEnabled(false);
  center_notch_checkbox_ = new QCheckBox("Enabled", control_group);
  center_notch_checkbox_->setChecked(false);
  center_notch_width_spin_ = new QSpinBox(control_group);
  center_notch_width_spin_->setRange(100, 200000);
  center_notch_width_spin_->setSingleStep(100);
  center_notch_width_spin_->setValue(2000);
  center_notch_width_spin_->setSuffix(" Hz");
  center_notch_width_spin_->setEnabled(false);
  lo_offset_checkbox_ = new QCheckBox("Enabled", control_group);
  lo_offset_checkbox_->setChecked(false);
  lo_offset_spin_ = new QSpinBox(control_group);
  lo_offset_spin_->setRange(-500000, 500000);
  lo_offset_spin_->setSingleStep(100);
  lo_offset_spin_->setValue(0);
  lo_offset_spin_->setSuffix(" Hz");
  lo_offset_spin_->setEnabled(false);

  mode_tabs_ = new QTabWidget(control_group);
  mode_tabs_->setUsesScrollButtons(false);
  mode_tabs_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  control_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  auto* fixed_tab = new QWidget(mode_tabs_);
  auto* fixed_layout = new QFormLayout(fixed_tab);
  fixed_modulation_combo_ = new QComboBox(fixed_tab);
  fixed_modulation_combo_->addItem("NFM",  QVariant::fromValue<int>(v1::MODULATION_NFM));
  fixed_modulation_combo_->addItem("WFM",  QVariant::fromValue<int>(v1::MODULATION_WFM));
  fixed_modulation_combo_->addItem("AM",   QVariant::fromValue<int>(v1::MODULATION_AM));
  fixed_modulation_combo_->addItem("FSK",  QVariant::fromValue<int>(v1::MODULATION_FSK));
  fixed_modulation_combo_->addItem("GMSK", QVariant::fromValue<int>(v1::MODULATION_GMSK));
  fixed_modulation_combo_->addItem("VDES ASM", QVariant::fromValue<int>(v1::MODULATION_VDES_ASM));
  fixed_modulation_combo_->addItem("PPM",   QVariant::fromValue<int>(v1::MODULATION_PPM));
  fixed_modulation_combo_->addItem("ADS-B",    QVariant::fromValue<int>(v1::MODULATION_ADSB));
  fixed_modulation_combo_->addItem("AIS Dual", QVariant::fromValue<int>(v1::MODULATION_AIS_DUAL));
  fixed_modulation_combo_->setCurrentIndex(
      fixed_modulation_combo_->findData(QVariant::fromValue<int>(v1::MODULATION_WFM)));
  fixed_channel_bandwidth_spin_ = new QSpinBox(fixed_tab);
  fixed_channel_bandwidth_spin_->setRange(0, 500000);
  fixed_channel_bandwidth_spin_->setSingleStep(1000);
  fixed_channel_bandwidth_spin_->setValue(30000);
  fixed_channel_bandwidth_spin_->setSuffix(" Hz");
  fixed_channel_bandwidth_spin_->setSpecialValueText("Off");
  fixed_sample_rate_warning_label_ = new QLabel(fixed_tab);
  fixed_sample_rate_warning_label_->setText(
      "Varning: sample-rate bor vara 2048000 Hz for stabil GMSK/VDES mottagning.");
  fixed_sample_rate_warning_label_->setStyleSheet("color: #F57C00; font-weight: 600;");
  fixed_sample_rate_warning_label_->setVisible(false);
  fixed_audio_hpf300_checkbox_    = new QCheckBox("HP 300 Hz",    fixed_tab);
  fixed_audio_lpf3k5_checkbox_   = new QCheckBox("LP 3.5 kHz",  fixed_tab);
  fixed_audio_lpf4k5_checkbox_   = new QCheckBox("LP 4.5 kHz",  fixed_tab);
  fixed_audio_bpf_voice_checkbox_ = new QCheckBox("BP 300\xe2\x80\x933k Hz", fixed_tab);
  fixed_audio_hpf300_checkbox_->setToolTip("High-pass filter at 300 Hz \xe2\x80\x94 removes low-frequency hum and rumble");
  fixed_audio_lpf3k5_checkbox_->setToolTip("Low-pass filter at 3.5 kHz \xe2\x80\x94 aggressive double-pass, very steep rolloff (~96 dB/octave)");
  fixed_audio_lpf4k5_checkbox_->setToolTip("Low-pass filter at 4.5 kHz \xe2\x80\x94 order-8 Butterworth, cuts noise above wider speech band");
  fixed_audio_bpf_voice_checkbox_->setToolTip("Band-pass filter 300 Hz \xe2\x80\x93 3 kHz \xe2\x80\x94 pass-band optimised for voice communications");
  fixed_audio_rnnoise_checkbox_ = new QCheckBox("RNNoise", fixed_tab);
  fixed_audio_rnnoise_checkbox_->setToolTip("RNNoise neural network noise reduction");
  fixed_audio_rnnoise_strength_spin_ = new QSpinBox(fixed_tab);
  fixed_audio_rnnoise_strength_spin_->setRange(0, 100);
  fixed_audio_rnnoise_strength_spin_->setValue(100);
  fixed_audio_rnnoise_strength_spin_->setSuffix("%");
  fixed_audio_rnnoise_strength_spin_->setToolTip("Blend between original (0%) och fully denoised (100%)");
  auto* fixed_filter_row = new QHBoxLayout();
  fixed_filter_row->addWidget(fixed_audio_hpf300_checkbox_);
  fixed_filter_row->addWidget(fixed_audio_lpf3k5_checkbox_);
  fixed_filter_row->addWidget(fixed_audio_lpf4k5_checkbox_);
  fixed_filter_row->addWidget(fixed_audio_bpf_voice_checkbox_);
  fixed_filter_row->addWidget(fixed_audio_rnnoise_checkbox_);
  fixed_filter_row->addWidget(fixed_audio_rnnoise_strength_spin_);
  fixed_filter_row->addStretch(1);

  // GMSK parameter controls (shown only when GMSK modulation is selected).
  gmsk_params_widget_ = new QWidget(fixed_tab);
  auto* gmsk_row = new QHBoxLayout(gmsk_params_widget_);
  gmsk_row->setContentsMargins(0, 0, 0, 0);
  gmsk_baud_rate_spin_ = new QSpinBox(gmsk_params_widget_);
  gmsk_baud_rate_spin_->setRange(300, 1000000);
  gmsk_baud_rate_spin_->setSingleStep(100);
  gmsk_baud_rate_spin_->setValue(9600);
  gmsk_baud_rate_spin_->setSuffix(" bit/s");
  gmsk_bt_spin_ = new QDoubleSpinBox(gmsk_params_widget_);
  gmsk_bt_spin_->setRange(0.1, 1.0);
  gmsk_bt_spin_->setSingleStep(0.05);
  gmsk_bt_spin_->setDecimals(2);
  gmsk_bt_spin_->setValue(0.4);
  gmsk_bt_spin_->setToolTip("Bandwidth-Time product (BT). Typical: 0.3 (GSM), 0.4, 0.5");
  gmsk_mod_index_spin_ = new QDoubleSpinBox(gmsk_params_widget_);
  gmsk_mod_index_spin_->setRange(0.1, 2.0);
  gmsk_mod_index_spin_->setSingleStep(0.05);
  gmsk_mod_index_spin_->setDecimals(2);
  gmsk_mod_index_spin_->setValue(0.5);
  gmsk_mod_index_spin_->setToolTip("Modulation index h. Standard GMSK: 0.5");
  vdes_bit_rate_spin_ = new QSpinBox(gmsk_params_widget_);
  vdes_bit_rate_spin_->setRange(2400, 76800);
  vdes_bit_rate_spin_->setSingleStep(1200);
  vdes_bit_rate_spin_->setValue(28800);
  vdes_bit_rate_spin_->setSuffix(" bps");
  vdes_bit_rate_spin_->setToolTip("VDES ASM bitrate for vdes_asm_demod.");
  vdes_pll_bw_spin_ = new QDoubleSpinBox(gmsk_params_widget_);
  vdes_pll_bw_spin_->setRange(0.0001, 0.2);
  vdes_pll_bw_spin_->setSingleStep(0.001);
  vdes_pll_bw_spin_->setDecimals(4);
  vdes_pll_bw_spin_->setValue(0.01);
  vdes_pll_bw_spin_->setToolTip("VDES ASM PLL bandwidth (carrier tracking loop).");
  vdes_candidate_bits_spin_ = new QSpinBox(gmsk_params_widget_);
  vdes_candidate_bits_spin_->setRange(96, 4096);
  vdes_candidate_bits_spin_->setSingleStep(16);
  vdes_candidate_bits_spin_->setValue(1056);
  vdes_candidate_bits_spin_->setToolTip("VDES ASM candidate burst length in bits.");
  vdes_sync_errors_spin_ = new QSpinBox(gmsk_params_widget_);
  vdes_sync_errors_spin_->setRange(0, 8);
  vdes_sync_errors_spin_->setSingleStep(1);
  vdes_sync_errors_spin_->setValue(1);
  vdes_sync_errors_spin_->setToolTip("Allowed sync bit errors for candidate detection.");
  gmsk_row->addWidget(new QLabel("Baudrate:", gmsk_params_widget_));
  gmsk_row->addWidget(gmsk_baud_rate_spin_);
  gmsk_row->addSpacing(8);
  gmsk_row->addWidget(new QLabel("BT:", gmsk_params_widget_));
  gmsk_row->addWidget(gmsk_bt_spin_);
  gmsk_row->addSpacing(8);
  gmsk_row->addWidget(new QLabel("Mod.index:", gmsk_params_widget_));
  gmsk_row->addWidget(gmsk_mod_index_spin_);
  gmsk_row->addSpacing(12);
  gmsk_row->addWidget(new QLabel("Avkodare:", gmsk_params_widget_));
  gmsk_decoder_combo_ = new QComboBox(gmsk_params_widget_);
  gmsk_decoder_combo_->addItem("Ingen",              QVariant(QString("")));
  gmsk_decoder_combo_->addItem("NRZI",               QVariant(QString("nrzi_decoder:0")));
  gmsk_decoder_combo_->addItem("NRZ-S (AIS/HDLC)",  QVariant(QString("nrzi_decoder:1")));
  gmsk_decoder_combo_->addItem("VDES ASM (skiss)",  QVariant(QString("vdes_asm_decoder:0")));
  gmsk_decoder_combo_->setToolTip(
      "Avkodare att kedja efter GMSK.\n"
      "NRZI: transition=1 (NRZ-Mark).\n"
      "NRZ-S (AIS/HDLC): transition=0, no-transition=1 (ITU-R M.1371).\n"
      "VDES ASM (skiss): scaffold for separat VDES ASM-kedja.");
  gmsk_row->addWidget(gmsk_decoder_combo_);
  gmsk_row->addSpacing(12);
  gmsk_row->addWidget(new QLabel("Postprocessing:", gmsk_params_widget_));
  gmsk_postproc_combo_ = new QComboBox(gmsk_params_widget_);
  gmsk_postproc_combo_->addItem("Ingen postprocessing", QVariant(QString("")));
  gmsk_postproc_combo_->addItem("HDLC",                 QVariant(QString("hdlc_postproc")));
  gmsk_postproc_combo_->addItem("AIS (HDLC+M.1371)",    QVariant(QString("ais_decoder")));
  gmsk_postproc_combo_->addItem("AIS Msg8 (HDLC+DAC/FI)", QVariant(QString("asm_decoder")));
  gmsk_postproc_combo_->addItem("VDES ASM (skiss)",      QVariant(QString("vdes_asm_postproc")));
  gmsk_postproc_combo_->setToolTip(
      "Postprocessing att kedja efter avkodaren.\n"
      "AIS: avkodar HDLC-ramar som AIS-meddelanden (ITU-R M.1371-5).\n"
      "AIS Msg8: avkodar AIS typ 8 (DAC/FI) over AIS/HDLC.\n"
      "VDES ASM (skiss): separat experimentell kedja (ej full standard-implementation).\n"
      "Obs: AIS Msg8-varianten ar inte VDES ASM-fysiklagret.");
  gmsk_row->addWidget(gmsk_postproc_combo_);
  gmsk_row->addSpacing(12);
  gmsk_row->addWidget(new QLabel("VDES bps:", gmsk_params_widget_));
  gmsk_row->addWidget(vdes_bit_rate_spin_);
  gmsk_row->addSpacing(8);
  gmsk_row->addWidget(new QLabel("PLL BW:", gmsk_params_widget_));
  gmsk_row->addWidget(vdes_pll_bw_spin_);
  gmsk_row->addSpacing(8);
  gmsk_row->addWidget(new QLabel("Cand bits:", gmsk_params_widget_));
  gmsk_row->addWidget(vdes_candidate_bits_spin_);
  gmsk_row->addSpacing(8);
  gmsk_row->addWidget(new QLabel("Sync err:", gmsk_params_widget_));
  gmsk_row->addWidget(vdes_sync_errors_spin_);
  gmsk_row->addStretch(1);
  gmsk_params_widget_->setVisible(false);

  // PPM parameter controls (shown only when PPM modulation is selected).
  ppm_params_widget_ = new QWidget(fixed_tab);
  auto* ppm_row = new QHBoxLayout(ppm_params_widget_);
  ppm_row->setContentsMargins(0, 0, 0, 0);
  ppm_bit_duration_us_spin_ = new QSpinBox(ppm_params_widget_);
  ppm_bit_duration_us_spin_->setRange(1, 100000);
  ppm_bit_duration_us_spin_->setSingleStep(1);
  ppm_bit_duration_us_spin_->setValue(10);
  ppm_bit_duration_us_spin_->setSuffix(" \xc2\xb5s");
  ppm_bit_duration_us_spin_->setToolTip(
      "Bittid i mikrosekunder — bestämmer chirp-tidens längd.\n"
      "Varje bit delas i två lika halvperioder:\n"
      "  HÖG+LÅG = logisk 1,  LÅG+HÖG = logisk 0.\n"
      "Kräver minst 4 samplar per bit.");
  ppm_data_rate_mbit_spin_ = new QDoubleSpinBox(ppm_params_widget_);
  ppm_data_rate_mbit_spin_->setRange(0.001, 100.0);
  ppm_data_rate_mbit_spin_->setSingleStep(0.1);
  ppm_data_rate_mbit_spin_->setDecimals(3);
  ppm_data_rate_mbit_spin_->setValue(0.1);
  ppm_data_rate_mbit_spin_->setSuffix(" MBit/s");
  ppm_data_rate_mbit_spin_->setToolTip("Datatakt — sätter takten på sändningens bitström.");
  ppm_row->addWidget(new QLabel("Bittid:", ppm_params_widget_));
  ppm_row->addWidget(ppm_bit_duration_us_spin_);
  ppm_row->addSpacing(12);
  ppm_row->addWidget(new QLabel("Datatakt:", ppm_params_widget_));
  ppm_row->addWidget(ppm_data_rate_mbit_spin_);
  ppm_row->addStretch(1);
  ppm_params_widget_->setVisible(false);

  fixed_plugin_params_widget_ = new QWidget(fixed_tab);
  {
    auto* vbox = new QVBoxLayout(fixed_plugin_params_widget_);
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->setSpacing(2);
    vbox->addWidget(gmsk_params_widget_);
    vbox->addWidget(ppm_params_widget_);
  }
  fixed_plugin_params_widget_->setVisible(false);

  auto* fixed_plugin_settings_button = new QPushButton("Plugin settings...", fixed_tab);
  fixed_plugin_settings_button->setToolTip(
      "Öppna plugin-parametrar för GMSK/VDES/PPM utan att visa alla fält i huvudvyn.");

  fixed_layout->addRow("Fixed MHz", fixed_frequency_edit_);
  fixed_layout->addRow("Demod", fixed_modulation_combo_);
  fixed_layout->addRow("Kanalbandbredd", fixed_channel_bandwidth_spin_);
  fixed_layout->addRow("Plugin", fixed_plugin_settings_button);
  fixed_layout->addRow(QString(), fixed_sample_rate_warning_label_);
  fixed_layout->addRow("Ljudfilter", fixed_filter_row);

  fixed_hdlc_log_ = new QPlainTextEdit(fixed_tab);
  fixed_hdlc_log_->setReadOnly(true);
  fixed_hdlc_log_->setMaximumBlockCount(1000);
  fixed_hdlc_log_->setMinimumHeight(90);
  {
    QFont f = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    f.setPointSizeF(f.pointSizeF() * 0.85);
    fixed_hdlc_log_->setFont(f);
  }
  fixed_hdlc_log_->setPlaceholderText("HDLC-ramar (CRC OK) visas här");
  fixed_layout->addRow("HDLC", fixed_hdlc_log_);

  mode_tabs_->addTab(fixed_tab, "FIXED");

  // Create scan_range_viz_ here so it can live inside the SCAN_RANGE tab.
  scan_range_viz_ = new ScanRangeVisualizationWidget(central);
  scan_range_viz_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  auto* range_tab = new QWidget(mode_tabs_);
  auto* range_outer = new QVBoxLayout(range_tab);
  auto* range_row = new QHBoxLayout();
  range_row->addWidget(new QLabel("Start MHz", range_tab));
  range_row->addWidget(range_start_edit_);
  range_row->addWidget(new QLabel("End MHz", range_tab));
  range_row->addWidget(range_end_edit_);
  range_row->addWidget(new QLabel("FFT", range_tab));
  range_row->addWidget(range_fft_size_combo_);
  range_row->addStretch(1);
  range_outer->addLayout(range_row);

  range_noise_gate_checkbox_ = new QCheckBox("Göm brus under", range_tab);
  range_noise_gate_spin_ = new QDoubleSpinBox(range_tab);
  range_noise_gate_spin_->setRange(kScanSpectrumFloorDb, kScanSpectrumCeilingDb);
  range_noise_gate_spin_->setSingleStep(1.0);
  range_noise_gate_spin_->setDecimals(0);
  range_noise_gate_spin_->setValue(-30.0);
  range_noise_gate_spin_->setSuffix(" dBFS");
  range_noise_gate_spin_->setEnabled(false);
  range_db_ceiling_spin_ = new QDoubleSpinBox(range_tab);
  range_db_ceiling_spin_->setRange(kScanSpectrumFloorDb, kScanSpectrumCeilingDb);
  range_db_ceiling_spin_->setSingleStep(1.0);
  range_db_ceiling_spin_->setDecimals(0);
  range_db_ceiling_spin_->setValue(kScanSpectrumCeilingDb);
  range_db_ceiling_spin_->setSuffix(" dBFS");
  range_db_ceiling_spin_->setToolTip("Max-värde på dB-skalan i spektrogrammet");
  scan_range_channel_bw_spin_ = new QSpinBox(range_tab);
  scan_range_channel_bw_spin_->setRange(0, 500000);
  scan_range_channel_bw_spin_->setSingleStep(1000);
  scan_range_channel_bw_spin_->setValue(0);
  scan_range_channel_bw_spin_->setSuffix(" Hz");
  scan_range_channel_bw_spin_->setSpecialValueText("Av");
  scan_range_channel_bw_spin_->setToolTip("Kanalbredd för kanalmarkeringar i spektrum/vattenfall");

  auto* threshold_row = new QHBoxLayout();
  threshold_row->addWidget(range_noise_gate_checkbox_);
  threshold_row->addWidget(range_noise_gate_spin_);
  threshold_row->addSpacing(16);
  threshold_row->addWidget(new QLabel("Tak:", range_tab));
  threshold_row->addWidget(range_db_ceiling_spin_);
  threshold_row->addSpacing(16);
  threshold_row->addWidget(new QLabel("Kanalbredd:", range_tab));
  threshold_row->addWidget(scan_range_channel_bw_spin_);
  threshold_row->addStretch(1);
  range_outer->addLayout(threshold_row);

  range_outer->addWidget(scan_range_viz_);  // fills remaining tab space
  mode_tabs_->addTab(range_tab, "SCAN_RANGE");

  auto* list_tab = new QWidget(mode_tabs_);
  auto* list_layout = new QVBoxLayout(list_tab);
  auto* list_caption = new QLabel(
      "Klicka pa en kanalruta for att konfigurera label, frekvens, modulation, bandbredd, squelch och dwell.",
      list_tab);
  list_caption->setWordWrap(true);
  list_layout->addWidget(list_caption);
  auto* controls_row = new QHBoxLayout();
  scan_list_monitor_checkbox_ = new QCheckBox("Monitor mode", list_tab);
  scan_list_monitor_checkbox_->setChecked(false);
  scan_list_monitor_checkbox_->setToolTip("Hold scan hopping, treat all channels as open");
  auto* default_squelch_label = new QLabel("Default squelch:", list_tab);
  scan_list_default_squelch_spin_ = new QDoubleSpinBox(list_tab);
  scan_list_default_squelch_spin_->setDecimals(1);
  scan_list_default_squelch_spin_->setRange(-120.0, 0.0);
  scan_list_default_squelch_spin_->setSingleStep(1.0);
  scan_list_default_squelch_spin_->setSuffix(" dB");
  scan_list_default_squelch_spin_->setValue(kDefaultScanListSquelchDb);
  audio_hpf300_checkbox_ = new QCheckBox("HP 300 Hz", list_tab);
  audio_lpf3k5_checkbox_ = new QCheckBox("LP 3.5 kHz", list_tab);
  audio_lpf4k5_checkbox_ = new QCheckBox("LP 4.5 kHz", list_tab);
  audio_bpf_voice_checkbox_ = new QCheckBox("BP 300–3k Hz", list_tab);
  audio_hpf300_checkbox_->setToolTip("High-pass filter at 300 Hz — removes low-frequency hum and rumble");
  audio_lpf3k5_checkbox_->setToolTip("Low-pass filter at 3.5 kHz — aggressive double-pass, very steep rolloff (~96 dB/octave)");
  audio_lpf4k5_checkbox_->setToolTip("Low-pass filter at 4.5 kHz — order-8 Butterworth, cuts noise above wider speech band");
  audio_bpf_voice_checkbox_->setToolTip("Band-pass filter 300 Hz – 3 kHz — pass-band optimised for voice communications");
  controls_row->addWidget(scan_list_monitor_checkbox_);
  controls_row->addSpacing(12);
  controls_row->addWidget(default_squelch_label);
  controls_row->addWidget(scan_list_default_squelch_spin_);
  controls_row->addSpacing(12);
  controls_row->addWidget(audio_hpf300_checkbox_);
  controls_row->addWidget(audio_lpf3k5_checkbox_);
  controls_row->addWidget(audio_lpf4k5_checkbox_);
  controls_row->addWidget(audio_bpf_voice_checkbox_);
  controls_row->addSpacing(12);
  audio_rnnoise_checkbox_ = new QCheckBox("RNNoise", list_tab);
  audio_rnnoise_checkbox_->setToolTip("RNNoise neural network noise reduction");
  audio_rnnoise_strength_spin_ = new QSpinBox(list_tab);
  audio_rnnoise_strength_spin_->setRange(0, 100);
  audio_rnnoise_strength_spin_->setValue(100);
  audio_rnnoise_strength_spin_->setSuffix("%");
  audio_rnnoise_strength_spin_->setToolTip("Blend between original (0%) and fully denoised (100%)");
  controls_row->addWidget(audio_rnnoise_checkbox_);
  controls_row->addWidget(audio_rnnoise_strength_spin_);
  controls_row->addStretch(1);
  list_layout->addLayout(controls_row);

  auto* list_actions = new QHBoxLayout();
  auto* add_channel_button = new QPushButton("Add channel", list_tab);
  auto* import_csv_button = new QPushButton("Import CSV", list_tab);
  auto* auto_squelch_button = new QPushButton("Auto squelch", list_tab);
  auto* clear_channels_button = new QPushButton("Clear channels", list_tab);
  list_actions->addWidget(add_channel_button);
  list_actions->addWidget(import_csv_button);
  list_actions->addWidget(auto_squelch_button);
  list_actions->addWidget(clear_channels_button);
  list_actions->addStretch(1);
  list_layout->addLayout(list_actions);

  scan_list_grid_widget_ = new QWidget(list_tab);
  scan_list_grid_layout_ = new QGridLayout(scan_list_grid_widget_);
  scan_list_grid_layout_->setHorizontalSpacing(8);
  scan_list_grid_layout_->setVerticalSpacing(8);
  scan_list_scroll_area_ = new QScrollArea(list_tab);
  scan_list_scroll_area_->setWidgetResizable(true);
  scan_list_scroll_area_->setWidget(scan_list_grid_widget_);
  scan_list_scroll_area_->viewport()->installEventFilter(this);
  list_layout->addWidget(scan_list_scroll_area_);

  connect(add_channel_button, &QPushButton::clicked, this, &MainWindow::AddScanListChannel);
  connect(import_csv_button, &QPushButton::clicked, this, &MainWindow::ImportScanListCsv);
  connect(auto_squelch_button, &QPushButton::clicked, this, &MainWindow::StartAutoSquelchCalibration);
  connect(scan_list_monitor_checkbox_, &QCheckBox::toggled, this, [this](bool /*enabled*/) {
    SaveScanListConfigToSettingsGroup(ActiveScanListSettingsGroup(mode_tabs_));
    if (receiver_combo_->currentIndex() >= 0) {
      ApplyModeAndConfig();
    }
  });
  connect(scan_list_default_squelch_spin_,
          static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
          [this](double value) {
            SaveScanListConfigToSettingsGroup(ActiveScanListSettingsGroup(mode_tabs_));
            RefreshScanListChannelCards();
            if (receiver_combo_->currentIndex() >= 0) {
              const uint32_t receiver_id = static_cast<uint32_t>(receiver_combo_->currentData().toUInt());
              bool use_default_for_visual = true;
              if (active_scan_list_channel_index_ >= 0 &&
                  static_cast<size_t>(active_scan_list_channel_index_) < scan_list_channels_.size()) {
                use_default_for_visual =
                    scan_list_channels_[static_cast<size_t>(active_scan_list_channel_index_)]
                        .use_default_squelch;
              }
              if (use_default_for_visual) {
                signal_visualization_->SetReceiverSquelchThresholdDb(receiver_id, value);
              }
            }
          });
  connect(scan_list_default_squelch_spin_, &QDoubleSpinBox::editingFinished, this, [this]() {
    if (receiver_combo_->currentIndex() >= 0) {
      ApplyModeAndConfig();
    }
  });
  auto apply_on_toggle = [this](bool /*checked*/) {
    if (receiver_combo_->currentIndex() >= 0) {
      ApplyModeAndConfig();
    }
  };
  connect(audio_hpf300_checkbox_,    &QCheckBox::toggled, this, apply_on_toggle);
  connect(audio_lpf3k5_checkbox_,   &QCheckBox::toggled, this, apply_on_toggle);
  connect(audio_lpf4k5_checkbox_,   &QCheckBox::toggled, this, apply_on_toggle);
  connect(audio_bpf_voice_checkbox_, &QCheckBox::toggled, this, apply_on_toggle);
  auto apply_on_spin = [this](int) { if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig(); };
  auto apply_on_dspin = [this](double) { if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig(); };
  auto update_sample_rate_warning = [this]() {
    if (fixed_sample_rate_warning_label_ == nullptr || fixed_modulation_combo_ == nullptr ||
        sample_rate_spin_ == nullptr) {
      return;
    }
    bool ok = false;
    const v1::Modulation mod = static_cast<v1::Modulation>(
        fixed_modulation_combo_->currentData().toInt(&ok));
    if (!ok) return;
    const bool digital_like = (mod == v1::MODULATION_GMSK ||
                               mod == v1::MODULATION_VDES_ASM ||
                               mod == v1::MODULATION_AIS_DUAL ||
                               mod == v1::MODULATION_FSK);
    const bool warn = digital_like && (sample_rate_spin_->value() != 2048000);
    fixed_sample_rate_warning_label_->setVisible(warn);
  };
  auto update_vdes_controls = [this]() {
    const QString decoder = gmsk_decoder_combo_
        ? gmsk_decoder_combo_->currentData().toString() : QString();
    const QString postproc = gmsk_postproc_combo_
        ? gmsk_postproc_combo_->currentData().toString() : QString();
    const v1::Modulation mod = FixedModulationFromCombo(fixed_modulation_combo_);
    const bool use_vdes = (mod == v1::MODULATION_VDES_ASM) ||
                          decoder.startsWith("vdes_asm_decoder") ||
                          postproc == "vdes_asm_postproc";
    if (vdes_bit_rate_spin_ != nullptr) vdes_bit_rate_spin_->setEnabled(use_vdes);
    if (vdes_pll_bw_spin_ != nullptr) vdes_pll_bw_spin_->setEnabled(use_vdes);
    if (vdes_candidate_bits_spin_ != nullptr) vdes_candidate_bits_spin_->setEnabled(use_vdes);
    if (vdes_sync_errors_spin_ != nullptr) vdes_sync_errors_spin_->setEnabled(use_vdes);
  };
  connect(gmsk_baud_rate_spin_,  QOverload<int>::of(&QSpinBox::valueChanged),       this, apply_on_spin);
  connect(gmsk_bt_spin_,         QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, apply_on_dspin);
  connect(gmsk_mod_index_spin_,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, apply_on_dspin);
  connect(vdes_bit_rate_spin_,   QOverload<int>::of(&QSpinBox::valueChanged),       this, apply_on_spin);
  connect(vdes_pll_bw_spin_,     QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, apply_on_dspin);
  connect(vdes_candidate_bits_spin_, QOverload<int>::of(&QSpinBox::valueChanged),   this, apply_on_spin);
  connect(vdes_sync_errors_spin_, QOverload<int>::of(&QSpinBox::valueChanged),      this, apply_on_spin);
  auto update_channel_overlay = [this]() {
    const bool is_receiver =
        signal_visualization_->CurrentSpectrumSource() ==
        SignalVisualizationWidget::SpectrumSource::kReceiverInput;
    const int bw = is_receiver
        ? (fixed_channel_bandwidth_spin_ ? fixed_channel_bandwidth_spin_->value()
                                         : channel_bandwidth_spin_->value())
        : 0;
    signal_visualization_->SetChannelBandwidthHz(bw);
  };

  connect(fixed_channel_bandwidth_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [this, update_channel_overlay](int value) {
            if (fixed_bandwidth_sync_in_progress_) return;
            fixed_bandwidth_manual_override_ = (value != fixed_bandwidth_last_auto_hz_);
            {
              const QSignalBlocker blocker(channel_bandwidth_spin_);
              channel_bandwidth_spin_->setValue(value);
            }
            update_channel_overlay();
            if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig();
          });
  connect(gmsk_decoder_combo_,   QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this, update_vdes_controls](int) {
            update_vdes_controls();
            if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig();
          });
  connect(gmsk_postproc_combo_,  QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this, update_vdes_controls](int) {
            update_vdes_controls();
            if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig();
          });
  connect(ppm_bit_duration_us_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, apply_on_spin);
  connect(ppm_data_rate_mbit_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, apply_on_dspin);
  update_vdes_controls();
  connect(sample_rate_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [this, update_sample_rate_warning](int) {
            update_sample_rate_warning();
            if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig();
          });
  update_sample_rate_warning();
  connect(fixed_audio_hpf300_checkbox_,    &QCheckBox::toggled, this, apply_on_toggle);
  connect(fixed_audio_lpf3k5_checkbox_,   &QCheckBox::toggled, this, apply_on_toggle);
  connect(fixed_audio_lpf4k5_checkbox_,   &QCheckBox::toggled, this, apply_on_toggle);
  connect(fixed_audio_bpf_voice_checkbox_, &QCheckBox::toggled, this, apply_on_toggle);
  connect(audio_rnnoise_checkbox_,         &QCheckBox::toggled, this, apply_on_toggle);
  connect(fixed_audio_rnnoise_checkbox_,   &QCheckBox::toggled, this, apply_on_toggle);
  connect(audio_rnnoise_strength_spin_,
          QOverload<int>::of(&QSpinBox::valueChanged), this,
          [this](int) { if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig(); });
  connect(fixed_audio_rnnoise_strength_spin_,
          QOverload<int>::of(&QSpinBox::valueChanged), this,
          [this](int) { if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig(); });
  connect(clear_channels_button, &QPushButton::clicked, this, [this]() {
    if (QMessageBox::question(this, "Clear channels",
                              "Remove all scan-list channels?") != QMessageBox::Yes) {
      return;
    }
    auto_squelch_active_ = false;
    auto_squelch_restore_monitor_mode_ = false;
    scan_list_channels_.clear();
    active_scan_list_channel_index_ = -1;
    active_scan_list_channel_state_ = ScanListChannelState::kIdle;
    SaveScanListConfigToSettingsGroup(ActiveScanListSettingsGroup(mode_tabs_));
    RefreshScanListChannelCards();
    if (receiver_combo_->currentIndex() >= 0) {
      ApplyModeAndConfig();
    }
  });
  mode_tabs_->addTab(list_tab, "SCAN_LIST");

  auto* air_marine_tab = new QWidget(mode_tabs_);
  auto* air_marine_layout = new QVBoxLayout(air_marine_tab);
  air_marine_layout->setContentsMargins(0, 0, 0, 0);

  // Three-column live radar view: left panel, radar, visible objects.
  auto* air_marine_splitter = new QSplitter(Qt::Horizontal, air_marine_tab);

  auto* air_marine_controls = new QWidget(air_marine_splitter);
  auto* controls_outer = new QVBoxLayout(air_marine_controls);
  controls_outer->setContentsMargins(6, 6, 6, 6);
  controls_outer->setSpacing(10);

  constexpr int kRadarSidePanelWidth = 340;
  air_marine_controls->setMinimumWidth(kRadarSidePanelWidth);
  air_marine_controls->setMaximumWidth(kRadarSidePanelWidth);

  auto* radar_group = new QGroupBox("Radar", air_marine_controls);
  auto* radar_layout = new QFormLayout(radar_group);
  radar_layout->setContentsMargins(8, 8, 8, 8);
  controls_outer->addWidget(radar_group, 0);

  auto* radio_group = new QGroupBox("Radio", air_marine_controls);
  auto* radio_layout = new QVBoxLayout(radio_group);
  radio_layout->setContentsMargins(8, 8, 8, 8);
  controls_outer->addWidget(radio_group, 1);

  // Radar-view radio scanner (separate from SCAN_LIST).
  {
    auto* radar_actions = new QHBoxLayout();
    auto* radar_add_button = new QPushButton("Add", radio_group);
    auto* radar_import_button = new QPushButton("Import CSV", radio_group);
    auto* radar_clear_button = new QPushButton("Clear", radio_group);
    radar_actions->addWidget(radar_add_button);
    radar_actions->addWidget(radar_import_button);
    radar_actions->addWidget(radar_clear_button);
    radar_actions->addStretch(1);
    radio_layout->addLayout(radar_actions);

    radar_scan_list_grid_widget_ = new QWidget(radio_group);
    radar_scan_list_grid_layout_ = new QGridLayout(radar_scan_list_grid_widget_);
    radar_scan_list_grid_layout_->setHorizontalSpacing(4);
    radar_scan_list_grid_layout_->setVerticalSpacing(4);
    radar_scan_list_scroll_area_ = new QScrollArea(radio_group);
    radar_scan_list_scroll_area_->setWidgetResizable(true);
    radar_scan_list_scroll_area_->setWidget(radar_scan_list_grid_widget_);
    radio_layout->addWidget(radar_scan_list_scroll_area_, 1);

    LoadRadarScanListConfigFromSettings();
    RefreshRadarScanListChannelCards();

    connect(radar_add_button, &QPushButton::clicked, this, &MainWindow::AddRadarScanListChannel);
    connect(radar_import_button, &QPushButton::clicked, this, &MainWindow::ImportRadarScanListCsv);
    connect(radar_clear_button, &QPushButton::clicked, this, [this]() {
      radar_scan_list_channels_.clear();
      radar_active_scan_list_channel_index_ = -1;
      SaveRadarScanListConfigToSettings();
      RefreshRadarScanListChannelCards();
      if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig();
    });
  }

  signal_filter_combo_ = new QComboBox(radar_group);
  signal_filter_combo_->addItem("ALL");
  signal_filter_combo_->addItem("SIGNAL_TYPE_AIS");
  signal_filter_combo_->addItem("SIGNAL_TYPE_ADSB");
  signal_filter_combo_->addItem("SIGNAL_TYPE_DSC");
  receiver_filter_combo_ = new QComboBox(radar_group);
  receiver_filter_combo_->addItem("ALL", QVariant::fromValue(-1));
  minutes_filter_spin_ = new QSpinBox(radar_group);
  minutes_filter_spin_->setRange(1, 240);
  minutes_filter_spin_->setValue(30);
  radar_layout->addRow(new QLabel("Radar updates come from the active scan-list channels.", radar_group));
  radar_layout->addRow(new QLabel("Tip: add an AIS Dual channel (around 162 MHz) to feed the map.", radar_group));
  auto* signal_minutes_row = new QWidget(radar_group);
  auto* signal_minutes_layout = new QHBoxLayout(signal_minutes_row);
  signal_minutes_layout->setContentsMargins(0, 0, 0, 0);
  signal_minutes_layout->addWidget(signal_filter_combo_);
  signal_minutes_layout->addWidget(minutes_filter_spin_);
  radar_layout->addRow("Signal / Last minutes", signal_minutes_row);
  // Single-receiver assumption: keep receiver filter internal but don't expose it in UI.
  receiver_filter_combo_->setVisible(false);

  radar_widget_ = new RadarMapWidget(air_marine_splitter);
  radar_widget_->SetRangeKm(10.0);
  radar_widget_->SetFixedObjects(LoadFixedObjectsFromSettings());

  air_marine_splitter->addWidget(air_marine_controls);
  air_marine_splitter->addWidget(radar_widget_);
  visible_objects_widget_ = new VisibleObjectsWidget(air_marine_splitter);
  visible_objects_widget_->setMinimumWidth(kRadarSidePanelWidth);
  visible_objects_widget_->setMaximumWidth(kRadarSidePanelWidth);
  air_marine_splitter->addWidget(visible_objects_widget_);
  air_marine_splitter->setStretchFactor(0, 1);
  air_marine_splitter->setStretchFactor(1, 3);
  air_marine_splitter->setStretchFactor(2, 1);

  air_marine_layout->addWidget(air_marine_splitter);

  // Selection wiring.
  connect(radar_widget_, &RadarMapWidget::TargetSelected, this, [this](const QString& id) {
    if (visible_objects_widget_ != nullptr) visible_objects_widget_->SetSelectedTarget(id);
  });
  connect(visible_objects_widget_, &VisibleObjectsWidget::TargetActivated, this, [this](const QString& id) {
    if (radar_widget_ != nullptr) radar_widget_->SetSelectedTarget(id);
  });

  // Radar settings controls (persisted).
  {
    QSettings settings("multi-radio", "multi-radio-client");
    settings.beginGroup("radar_view");
    const bool show_labels = settings.value("show_labels", false).toBool();
    const bool show_fixed_names = settings.value("show_fixed_names", true).toBool();
    const bool hide_low_speed = settings.value("hide_low_speed", false).toBool();
    const double range_km = std::clamp(settings.value("range_km", 10.0).toDouble(), 0.2, 500.0);
    const double trail_s = std::clamp(settings.value("trail_seconds", 120.0).toDouble(), 5.0, 3600.0);
    const double center_lat = settings.value("center_lat", 0.0).toDouble();
    const double center_lon = settings.value("center_lon", 0.0).toDouble();
    settings.endGroup();

    radar_widget_->SetShowLabels(show_labels);
    radar_widget_->SetShowFixedNames(show_fixed_names);
    radar_widget_->SetHideLowSpeed(hide_low_speed);
    radar_widget_->SetRangeKm(range_km);
    radar_widget_->SetTrailWindowSeconds(trail_s);
    if (center_lat != 0.0 || center_lon != 0.0) {
      radar_widget_->SetCenter(center_lat, center_lon);
    }
    if (visible_objects_widget_ != nullptr) visible_objects_widget_->SetHideLowSpeed(hide_low_speed);

    auto* radar_settings_button = new QPushButton("Radar settings...", radar_group);
    radar_layout->addRow(radar_settings_button);
    connect(radar_settings_button, &QPushButton::clicked, this, [this]() {
      if (radar_widget_ == nullptr) return;

      QSettings s("multi-radio", "multi-radio-client");
      s.beginGroup("radar_view");
      const double cur_lat = s.value("center_lat", 0.0).toDouble();
      const double cur_lon = s.value("center_lon", 0.0).toDouble();
      const bool show_labels = s.value("show_labels", false).toBool();
      const bool show_fixed_names = s.value("show_fixed_names", true).toBool();
      const bool hide_low_speed = s.value("hide_low_speed", false).toBool();
      const double range_km = std::clamp(s.value("range_km", 10.0).toDouble(), 0.2, 500.0);
      const double trail_s = std::clamp(s.value("trail_seconds", 120.0).toDouble(), 5.0, 3600.0);
      const QString fixed_json = s.value("fixed_objects_json", "[]").toString();
      s.endGroup();

      QDialog dialog(this);
      dialog.setWindowTitle("Radar settings");
      dialog.setMinimumSize(560, 520);
      auto* outer = new QVBoxLayout(&dialog);
      auto* layout = new QFormLayout();
      auto* lat_spin = new QDoubleSpinBox(&dialog);
      lat_spin->setDecimals(7);
      lat_spin->setRange(-90.0, 90.0);
      lat_spin->setValue(cur_lat);
      auto* lon_spin = new QDoubleSpinBox(&dialog);
      lon_spin->setDecimals(7);
      lon_spin->setRange(-180.0, 180.0);
      lon_spin->setValue(cur_lon);
      auto* range_spin = new QDoubleSpinBox(&dialog);
      range_spin->setDecimals(1);
      range_spin->setRange(0.2, 500.0);
      range_spin->setSingleStep(0.5);
      range_spin->setValue(range_km);
      range_spin->setSuffix(" km");
      auto* trail_spin = new QDoubleSpinBox(&dialog);
      trail_spin->setDecimals(0);
      trail_spin->setRange(5.0, 3600.0);
      trail_spin->setSingleStep(5.0);
      trail_spin->setValue(trail_s);
      trail_spin->setSuffix(" s");

      auto* show_labels_cb = new QCheckBox("Show labels", &dialog);
      show_labels_cb->setChecked(show_labels);
      auto* show_fixed_names_cb = new QCheckBox("Show fixed names", &dialog);
      show_fixed_names_cb->setChecked(show_fixed_names);
      auto* hide_low_speed_cb = new QCheckBox("Hide low speed (<1 kn)", &dialog);
      hide_low_speed_cb->setChecked(hide_low_speed);

      auto* fixed_editor = new QPlainTextEdit(&dialog);
      fixed_editor->setPlainText(fixed_json);

      layout->addRow("Center latitude", lat_spin);
      layout->addRow("Center longitude", lon_spin);
      layout->addRow("Range", range_spin);
      layout->addRow("Trail window", trail_spin);
      layout->addRow(show_labels_cb);
      layout->addRow(show_fixed_names_cb);
      layout->addRow(hide_low_speed_cb);

      outer->addLayout(layout);
      outer->addWidget(new QLabel("Fixed objects (JSON):", &dialog));
      outer->addWidget(fixed_editor, 1);

      auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
      outer->addWidget(buttons);
      connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
      connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

      if (dialog.exec() != QDialog::Accepted) return;
      const double lat = lat_spin->value();
      const double lon = lon_spin->value();
      const double new_range_km = range_spin->value();
      const double new_trail_s = trail_spin->value();
      const bool new_show_labels = show_labels_cb->isChecked();
      const bool new_show_fixed_names = show_fixed_names_cb->isChecked();
      const bool new_hide_low_speed = hide_low_speed_cb->isChecked();
      const QString json = fixed_editor->toPlainText().trimmed();
      const auto doc = QJsonDocument::fromJson(json.toUtf8());
      if (!doc.isArray()) {
        QMessageBox::warning(this, "Invalid JSON", "Fixed objects must be a JSON array.");
        return;
      }

      QSettings out("multi-radio", "multi-radio-client");
      out.beginGroup("radar_view");
      out.setValue("center_lat", lat);
      out.setValue("center_lon", lon);
      out.setValue("range_km", new_range_km);
      out.setValue("trail_seconds", new_trail_s);
      out.setValue("show_labels", new_show_labels);
      out.setValue("show_fixed_names", new_show_fixed_names);
      out.setValue("hide_low_speed", new_hide_low_speed);
      out.setValue("fixed_objects_json", json);
      out.endGroup();
      radar_widget_->SetCenter(lat, lon);
      radar_widget_->SetRangeKm(new_range_km);
      radar_widget_->SetTrailWindowSeconds(new_trail_s);
      radar_widget_->SetShowLabels(new_show_labels);
      radar_widget_->SetShowFixedNames(new_show_fixed_names);
      radar_widget_->SetHideLowSpeed(new_hide_low_speed);
      radar_widget_->SetFixedObjects(LoadFixedObjectsFromSettings());
      if (visible_objects_widget_ != nullptr) visible_objects_widget_->SetHideLowSpeed(new_hide_low_speed);
    });
  }

  // TODO: add a dedicated radar radio-scanner instance here (separate from SCAN_LIST).

  mode_tabs_->addTab(air_marine_tab, "RADAR_VIEW");

  auto* global_tab = new QWidget(mode_tabs_);
  auto* global_layout = new QFormLayout(global_tab);
  global_layout->addRow("Default dwell ms", dwell_ms_spin_);
  global_layout->addRow("Sample rate", sample_rate_spin_);
  global_layout->addRow("Hardware bandwidth", hardware_bandwidth_spin_);
  global_layout->addRow("DC blocker", dc_blocker_checkbox_);
  global_layout->addRow("DC cutoff", dc_blocker_cutoff_spin_);
  global_layout->addRow("Center notch", center_notch_checkbox_);
  global_layout->addRow("Notch width", center_notch_width_spin_);
  global_layout->addRow("LO offset", lo_offset_checkbox_);
  global_layout->addRow("LO offset Hz", lo_offset_spin_);
  ppm_correction_spin_ = new QSpinBox(global_tab);
  ppm_correction_spin_->setRange(-200, 200);
  ppm_correction_spin_->setSingleStep(1);
  ppm_correction_spin_->setSuffix(" ppm");
  ppm_correction_spin_->setToolTip(
      "Frekvenskorrigering för RTL-SDR-kristallen.\n"
      "Positiva värden höjer mottagarfrekvensen; negativa sänker.\n"
      "Justera tills AIS-signalens bärfrekvens stämmer.");
  global_layout->addRow("PPM-korrigering", ppm_correction_spin_);
  spectrum_source_combo_ = new QComboBox(global_tab);
  spectrum_source_combo_->addItem("Demodulated", QVariant::fromValue(0));
  spectrum_source_combo_->addItem("Receiver spectrum", QVariant::fromValue(1));
  spectrum_source_combo_->setCurrentIndex(1);
  auto* visualization_settings_button = new QPushButton("Visualization settings...", global_tab);
  global_layout->addRow("Spectrum view", spectrum_source_combo_);
  global_layout->addRow(visualization_settings_button);
  mode_tabs_->addTab(global_tab, "GLOBAL");
  mode_tabs_->setCurrentIndex(kFixedModeTabIndex);
  last_tab_index_ = mode_tabs_->currentIndex();

  auto* button_row = new QWidget(control_group);
  auto* button_layout = new QHBoxLayout(button_row);
  button_layout->setContentsMargins(0, 0, 0, 0);
  auto* refresh_button = new QPushButton("Refresh", button_row);
  auto* start_button = new QPushButton("Start", button_row);
  auto* stop_button = new QPushButton("Stop", button_row);
  auto* apply_button = new QPushButton("Apply mode/radio settings", button_row);
  button_layout->addWidget(refresh_button);
  button_layout->addWidget(start_button);
  button_layout->addWidget(stop_button);
  button_layout->addWidget(apply_button);

  // Use full width for the active view.
  control_layout->addRow(mode_tabs_);
  control_layout->addRow(button_row);

  top_layout->addWidget(control_group, 1);

  signal_visualization_ = new SignalVisualizationWidget(central);
  signal_visualization_->SetSpectrumSource(SignalVisualizationWidget::SpectrumSource::kReceiverInput);
  if (fixed_channel_bandwidth_spin_ != nullptr)
    signal_visualization_->SetChannelBandwidthHz(fixed_channel_bandwidth_spin_->value());
  {
    QSettings settings("multi-radio", "multi-radio-client");
    settings.beginGroup("visualization");
    const bool auto_noise_reduction = settings.value("auto_noise_reduction", false).toBool();
    const bool noise_floor_filter_enabled =
        settings.value("noise_floor_filter_enabled", false).toBool();
    const double noise_floor_db =
        std::clamp(settings.value("noise_floor_db", -30.0).toDouble(), -120.0, 0.0);
    iq_visual_dc_suppression_enabled_ = settings.value("iq_dc_suppression", true).toBool();
    settings.endGroup();
    signal_visualization_->SetAutoNoiseReductionEnabled(auto_noise_reduction);
    signal_visualization_->SetNoiseFloorFilterEnabled(noise_floor_filter_enabled);
    signal_visualization_->SetNoiseFloorDb(noise_floor_db);
  }

  event_log_ = new QPlainTextEdit(central);
  event_log_->setReadOnly(true);

  root_layout->addLayout(top_layout, 1);
  root_layout->addWidget(signal_visualization_);
  root_layout->addWidget(event_log_);

  setCentralWidget(central);
  LoadScanListConfigFromSettings();
  RefreshScanListChannelCards();

  audio_output_disabled_ = EnvFlagEnabled("MR_DISABLE_AUDIO_OUTPUT");
  audio_output_disabled_by_env_ = audio_output_disabled_;
  if (audio_output_disabled_) {
    AppendLog("Audio output disabled by MR_DISABLE_AUDIO_OUTPUT");
  } else if (IsWslEnvironment() && !EnvFlagEnabled("MR_ENABLE_AUDIO_OUTPUT")) {
    AppendLog("WSL environment detected; attempting audio output (set MR_DISABLE_AUDIO_OUTPUT=1 to disable)");
  }
#if MR_HAS_QT_MULTIMEDIA
  audio_drain_timer_ = new QTimer(this);
  audio_drain_timer_->setInterval(kAudioDrainIntervalMs);
  connect(audio_drain_timer_, &QTimer::timeout, this, &MainWindow::DrainAudioOutputQueue);
  audio_drain_timer_->start();
#endif

  connect(refresh_button, &QPushButton::clicked, this, &MainWindow::RefreshReceivers);
  connect(start_button, &QPushButton::clicked, this, &MainWindow::StartSelectedReceiver);
  connect(stop_button, &QPushButton::clicked, this, &MainWindow::StopSelectedReceiver);
  connect(apply_button, &QPushButton::clicked, this, &MainWindow::ApplyModeAndConfig);
  connect(visualization_settings_button, &QPushButton::clicked, this,
          &MainWindow::OpenVisualizationSettingsDialog);

  connect(signal_filter_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
    if (decoded_table_ != nullptr) decoded_table_->setRowCount(0);
    for (const auto& row : all_rows_) {
      AddMessageRow(row);
    }
  });
  connect(receiver_filter_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
    if (decoded_table_ != nullptr) decoded_table_->setRowCount(0);
    for (const auto& row : all_rows_) {
      AddMessageRow(row);
    }
    if (signal_visualization_ != nullptr) {
      signal_visualization_->SetReceiverFilter(receiver_filter_combo_->currentData().toInt());
    }
  });
  connect(spectrum_source_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this, update_channel_overlay]() {
    const int selected = spectrum_source_combo_->currentData().toInt();
    const auto source =
        (selected == 1) ? SignalVisualizationWidget::SpectrumSource::kReceiverInput
                        : SignalVisualizationWidget::SpectrumSource::kDemodulated;
    signal_visualization_->SetSpectrumSource(source);
    update_channel_overlay();
    AppendLog(QString("Visualization spectrum source: %1")
                  .arg(source == SignalVisualizationWidget::SpectrumSource::kReceiverInput
                           ? "receiver"
                           : "demodulated"));
  });
  connect(minutes_filter_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this]() {
    decoded_table_->setRowCount(0);
    for (const auto& row : all_rows_) {
      AddMessageRow(row);
    }
  });
  connect(dwell_ms_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
    if (scan_range_viz_) scan_range_viz_->SetDwellMs(value);
  });
  connect(dwell_ms_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this]() {
    SaveScanListConfigToSettingsGroup(ActiveScanListSettingsGroup(mode_tabs_));
  });
  connect(channel_bandwidth_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this, update_channel_overlay](int value) {
    if (fixed_bandwidth_sync_in_progress_) {
      return;
    }
    fixed_bandwidth_manual_override_ = (value != fixed_bandwidth_last_auto_hz_);
    if (fixed_channel_bandwidth_spin_ != nullptr &&
        fixed_channel_bandwidth_spin_->value() != value) {
      const QSignalBlocker blocker(fixed_channel_bandwidth_spin_);
      fixed_channel_bandwidth_spin_->setValue(value);
    }
    update_channel_overlay();
  });
  connect(fixed_modulation_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this, update_vdes_controls]() {
    const v1::Modulation modulation = FixedModulationFromCombo(fixed_modulation_combo_);
    const bool is_gmsk_like = (modulation == v1::MODULATION_GMSK ||
                               modulation == v1::MODULATION_VDES_ASM);
    const bool is_ppm      = (modulation == v1::MODULATION_PPM);
    const bool is_ais_dual = (modulation == v1::MODULATION_AIS_DUAL);
    /* Keep parameter widgets hidden from the main Fixed layout. */
    if (gmsk_params_widget_ != nullptr) gmsk_params_widget_->setVisible(false);
    if (ppm_params_widget_ != nullptr) ppm_params_widget_->setVisible(false);
    if (fixed_plugin_params_widget_ != nullptr) fixed_plugin_params_widget_->setVisible(false);

    if (modulation == v1::MODULATION_VDES_ASM) {
      if (gmsk_decoder_combo_ != nullptr) {
        const QSignalBlocker blocker(gmsk_decoder_combo_);
        const int idx = gmsk_decoder_combo_->findData(QVariant(QString("vdes_asm_decoder:0")));
        if (idx >= 0) gmsk_decoder_combo_->setCurrentIndex(idx);
      }
      if (gmsk_postproc_combo_ != nullptr) {
        const QSignalBlocker blocker(gmsk_postproc_combo_);
        const int idx = gmsk_postproc_combo_->findData(QVariant(QString("vdes_asm_postproc")));
        if (idx >= 0) gmsk_postproc_combo_->setCurrentIndex(idx);
      }
    }
    update_vdes_controls();
    /* ADS-B kräver exakt 2.4 Msps (dump1090-kompatibel fas-avkodning) — lås samplerate-spinnern */
    if (sample_rate_spin_ != nullptr) {
      const bool is_adsb = (modulation == v1::MODULATION_ADSB);
      if (is_adsb) {
        const QSignalBlocker blocker(sample_rate_spin_);
        sample_rate_spin_->setValue(2400000);
      }
      /* AIS Dual rekommenderar minst 250 kSps för att täcka båda kanalerna (±25 kHz) */
      if (is_ais_dual && sample_rate_spin_->value() < 250000) {
        const QSignalBlocker blocker(sample_rate_spin_);
        sample_rate_spin_->setValue(250000);
      }
      sample_rate_spin_->setEnabled(!is_adsb);
    }
    const int suggested_bandwidth_hz = DefaultBandwidthHzForModulation(modulation);
    const int current_bandwidth_hz = fixed_channel_bandwidth_spin_ != nullptr
        ? fixed_channel_bandwidth_spin_->value()
        : channel_bandwidth_spin_->value();
    const bool can_auto_apply =
        !fixed_bandwidth_manual_override_ || current_bandwidth_hz == fixed_bandwidth_last_auto_hz_;

    fixed_bandwidth_last_auto_hz_ = suggested_bandwidth_hz;
    if (!can_auto_apply) {
      if (fixed_sample_rate_warning_label_ != nullptr && sample_rate_spin_ != nullptr) {
        const bool digital_like = (modulation == v1::MODULATION_GMSK ||
                                   modulation == v1::MODULATION_VDES_ASM ||
                                   modulation == v1::MODULATION_AIS_DUAL ||
                                   modulation == v1::MODULATION_FSK);
        fixed_sample_rate_warning_label_->setVisible(
            digital_like && (sample_rate_spin_->value() != 2048000));
      }
      AppendLog(QString("Fixed demod %1 selected; keeping manual bandwidth %2 Hz")
                    .arg(ModulationLabel(modulation))
                    .arg(current_bandwidth_hz));
      return;
    }

    fixed_bandwidth_sync_in_progress_ = true;
    if (fixed_channel_bandwidth_spin_ != nullptr) {
      fixed_channel_bandwidth_spin_->setValue(suggested_bandwidth_hz);
    }
    channel_bandwidth_spin_->setValue(suggested_bandwidth_hz);
    fixed_bandwidth_sync_in_progress_ = false;
    fixed_bandwidth_manual_override_ = false;
    if (fixed_sample_rate_warning_label_ != nullptr && sample_rate_spin_ != nullptr) {
      const bool digital_like = (modulation == v1::MODULATION_GMSK ||
                                 modulation == v1::MODULATION_VDES_ASM ||
                                 modulation == v1::MODULATION_AIS_DUAL ||
                                 modulation == v1::MODULATION_FSK);
      fixed_sample_rate_warning_label_->setVisible(
          digital_like && (sample_rate_spin_->value() != 2048000));
    }
    AppendLog(QString("Fixed demod %1 selected; bandwidth auto-set to %2 Hz")
                  .arg(ModulationLabel(modulation))
                  .arg(suggested_bandwidth_hz));
  });

  connect(fixed_plugin_settings_button, &QPushButton::clicked, this,
          [this, update_vdes_controls]() {
    QDialog dialog(this);
    dialog.setWindowTitle("Plugin settings");
    auto* layout = new QFormLayout(&dialog);

    auto* gmsk_baud = new QSpinBox(&dialog);
    gmsk_baud->setRange(300, 1000000);
    gmsk_baud->setSingleStep(100);
    gmsk_baud->setSuffix(" bit/s");
    gmsk_baud->setValue(gmsk_baud_rate_spin_ ? gmsk_baud_rate_spin_->value() : 9600);

    auto* gmsk_bt = new QDoubleSpinBox(&dialog);
    gmsk_bt->setRange(0.1, 1.0);
    gmsk_bt->setSingleStep(0.05);
    gmsk_bt->setDecimals(2);
    gmsk_bt->setValue(gmsk_bt_spin_ ? gmsk_bt_spin_->value() : 0.4);

    auto* gmsk_mod_idx = new QDoubleSpinBox(&dialog);
    gmsk_mod_idx->setRange(0.1, 2.0);
    gmsk_mod_idx->setSingleStep(0.05);
    gmsk_mod_idx->setDecimals(2);
    gmsk_mod_idx->setValue(gmsk_mod_index_spin_ ? gmsk_mod_index_spin_->value() : 0.5);

    auto* decoder_combo = new QComboBox(&dialog);
    auto* postproc_combo = new QComboBox(&dialog);
    if (gmsk_decoder_combo_ != nullptr) {
      for (int i = 0; i < gmsk_decoder_combo_->count(); ++i) {
        decoder_combo->addItem(gmsk_decoder_combo_->itemText(i), gmsk_decoder_combo_->itemData(i));
      }
      const int idx = decoder_combo->findData(gmsk_decoder_combo_->currentData());
      if (idx >= 0) decoder_combo->setCurrentIndex(idx);
    }
    if (gmsk_postproc_combo_ != nullptr) {
      for (int i = 0; i < gmsk_postproc_combo_->count(); ++i) {
        postproc_combo->addItem(gmsk_postproc_combo_->itemText(i), gmsk_postproc_combo_->itemData(i));
      }
      const int idx = postproc_combo->findData(gmsk_postproc_combo_->currentData());
      if (idx >= 0) postproc_combo->setCurrentIndex(idx);
    }

    auto* vdes_bps = new QSpinBox(&dialog);
    vdes_bps->setRange(2400, 76800);
    vdes_bps->setSingleStep(1200);
    vdes_bps->setSuffix(" bps");
    vdes_bps->setValue(vdes_bit_rate_spin_ ? vdes_bit_rate_spin_->value() : 19200);

    auto* vdes_pll = new QDoubleSpinBox(&dialog);
    vdes_pll->setRange(0.0001, 0.2);
    vdes_pll->setSingleStep(0.001);
    vdes_pll->setDecimals(4);
    vdes_pll->setValue(vdes_pll_bw_spin_ ? vdes_pll_bw_spin_->value() : 0.01);

    auto* vdes_cand = new QSpinBox(&dialog);
    vdes_cand->setRange(96, 4096);
    vdes_cand->setSingleStep(16);
    vdes_cand->setValue(vdes_candidate_bits_spin_ ? vdes_candidate_bits_spin_->value() : 1056);

    auto* vdes_sync_err = new QSpinBox(&dialog);
    vdes_sync_err->setRange(0, 8);
    vdes_sync_err->setSingleStep(1);
    vdes_sync_err->setValue(vdes_sync_errors_spin_ ? vdes_sync_errors_spin_->value() : 1);

    auto* adsb_agc_bw = new QDoubleSpinBox(&dialog);
    adsb_agc_bw->setRange(0.000001, 0.1);
    adsb_agc_bw->setSingleStep(0.000001);
    adsb_agc_bw->setDecimals(8);
    adsb_agc_bw->setValue(adsb_agc_bandwidth_ > 0.0 ? adsb_agc_bandwidth_ : 0.000005);
    adsb_agc_bw->setToolTip("ADS-B AGC bandwidth for libliquid (default 5e-6). Lower is slower.");

    auto* adsb_agc_target = new QDoubleSpinBox(&dialog);
    adsb_agc_target->setRange(0.1, 10.0);
    adsb_agc_target->setSingleStep(0.1);
    adsb_agc_target->setDecimals(2);
    adsb_agc_target->setValue(adsb_agc_target_level_ > 0.0 ? adsb_agc_target_level_ : 1.0);
    adsb_agc_target->setToolTip("ADS-B AGC target level for normalized IQ amplitude.");

    auto* ppm_bit_us = new QSpinBox(&dialog);
    ppm_bit_us->setRange(1, 100000);
    ppm_bit_us->setSingleStep(1);
    ppm_bit_us->setSuffix(" us");
    ppm_bit_us->setValue(ppm_bit_duration_us_spin_ ? ppm_bit_duration_us_spin_->value() : 10);

    auto* ppm_rate = new QDoubleSpinBox(&dialog);
    ppm_rate->setRange(0.001, 100.0);
    ppm_rate->setSingleStep(0.1);
    ppm_rate->setDecimals(3);
    ppm_rate->setSuffix(" MBit/s");
    ppm_rate->setValue(ppm_data_rate_mbit_spin_ ? ppm_data_rate_mbit_spin_->value() : 0.1);

    auto update_vdes_enabled = [this, decoder_combo, postproc_combo, vdes_bps, vdes_pll, vdes_cand, vdes_sync_err]() {
      const v1::Modulation mod = FixedModulationFromCombo(fixed_modulation_combo_);
      const bool use_vdes = (mod == v1::MODULATION_VDES_ASM) ||
                            decoder_combo->currentData().toString().startsWith("vdes_asm_decoder") ||
                            postproc_combo->currentData().toString() == "vdes_asm_postproc";
      vdes_bps->setEnabled(use_vdes);
      vdes_pll->setEnabled(use_vdes);
      vdes_cand->setEnabled(use_vdes);
      vdes_sync_err->setEnabled(use_vdes);
    };

    auto update_adsb_enabled = [this, adsb_agc_bw, adsb_agc_target]() {
      const v1::Modulation mod = FixedModulationFromCombo(fixed_modulation_combo_);
      const bool use_adsb = (mod == v1::MODULATION_ADSB);
      adsb_agc_bw->setEnabled(use_adsb);
      adsb_agc_target->setEnabled(use_adsb);
    };

    connect(decoder_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), &dialog,
            [update_vdes_enabled](int) { update_vdes_enabled(); });
    connect(postproc_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), &dialog,
            [update_vdes_enabled](int) { update_vdes_enabled(); });
    connect(fixed_modulation_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), &dialog,
            [update_vdes_enabled, update_adsb_enabled](int) {
              update_vdes_enabled();
              update_adsb_enabled();
            });
    update_vdes_enabled();
    update_adsb_enabled();

    layout->addRow("GMSK baudrate", gmsk_baud);
    layout->addRow("GMSK BT", gmsk_bt);
    layout->addRow("GMSK mod.index", gmsk_mod_idx);
    layout->addRow("Decoder", decoder_combo);
    layout->addRow("Postprocessing", postproc_combo);
    layout->addRow("VDES bitrate", vdes_bps);
    layout->addRow("VDES PLL BW", vdes_pll);
    layout->addRow("VDES candidate bits", vdes_cand);
    layout->addRow("VDES sync errors", vdes_sync_err);
    layout->addRow("ADS-B AGC BW", adsb_agc_bw);
    layout->addRow("ADS-B AGC target", adsb_agc_target);
    layout->addRow("PPM bit duration", ppm_bit_us);
    layout->addRow("PPM data rate", ppm_rate);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    layout->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted) return;

    if (gmsk_baud_rate_spin_ != nullptr) {
      const QSignalBlocker blocker(gmsk_baud_rate_spin_);
      gmsk_baud_rate_spin_->setValue(gmsk_baud->value());
    }
    if (gmsk_bt_spin_ != nullptr) {
      const QSignalBlocker blocker(gmsk_bt_spin_);
      gmsk_bt_spin_->setValue(gmsk_bt->value());
    }
    if (gmsk_mod_index_spin_ != nullptr) {
      const QSignalBlocker blocker(gmsk_mod_index_spin_);
      gmsk_mod_index_spin_->setValue(gmsk_mod_idx->value());
    }
    if (gmsk_decoder_combo_ != nullptr) {
      const QSignalBlocker blocker(gmsk_decoder_combo_);
      const int idx = gmsk_decoder_combo_->findData(decoder_combo->currentData());
      if (idx >= 0) gmsk_decoder_combo_->setCurrentIndex(idx);
    }
    if (gmsk_postproc_combo_ != nullptr) {
      const QSignalBlocker blocker(gmsk_postproc_combo_);
      const int idx = gmsk_postproc_combo_->findData(postproc_combo->currentData());
      if (idx >= 0) gmsk_postproc_combo_->setCurrentIndex(idx);
    }
    if (vdes_bit_rate_spin_ != nullptr) {
      const QSignalBlocker blocker(vdes_bit_rate_spin_);
      vdes_bit_rate_spin_->setValue(vdes_bps->value());
    }
    if (vdes_pll_bw_spin_ != nullptr) {
      const QSignalBlocker blocker(vdes_pll_bw_spin_);
      vdes_pll_bw_spin_->setValue(vdes_pll->value());
    }
    if (vdes_candidate_bits_spin_ != nullptr) {
      const QSignalBlocker blocker(vdes_candidate_bits_spin_);
      vdes_candidate_bits_spin_->setValue(vdes_cand->value());
    }
    if (vdes_sync_errors_spin_ != nullptr) {
      const QSignalBlocker blocker(vdes_sync_errors_spin_);
      vdes_sync_errors_spin_->setValue(vdes_sync_err->value());
    }
    adsb_agc_bandwidth_ = adsb_agc_bw->value();
    adsb_agc_target_level_ = adsb_agc_target->value();
    if (ppm_bit_duration_us_spin_ != nullptr) {
      const QSignalBlocker blocker(ppm_bit_duration_us_spin_);
      ppm_bit_duration_us_spin_->setValue(ppm_bit_us->value());
    }
    if (ppm_data_rate_mbit_spin_ != nullptr) {
      const QSignalBlocker blocker(ppm_data_rate_mbit_spin_);
      ppm_data_rate_mbit_spin_->setValue(ppm_rate->value());
    }

    update_vdes_controls();
    if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig();
  });
  connect(receiver_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
    auto_squelch_active_ = false;
    auto_squelch_restore_monitor_mode_ = false;
    active_scan_list_channel_index_ = -1;
    active_scan_list_channel_state_ = ScanListChannelState::kIdle;
    if (signal_visualization_ != nullptr) {
      signal_visualization_->SetChannelLabel(QString());
    }
    if (receiver_combo_->currentIndex() >= 0 && scan_list_default_squelch_spin_ != nullptr) {
      const uint32_t receiver_id = static_cast<uint32_t>(receiver_combo_->currentData().toUInt());
      signal_visualization_->SetReceiverSquelchThresholdDb(receiver_id,
                                                           scan_list_default_squelch_spin_->value());
    }
    RefreshScanListChannelCards();
  });
  connect(mode_tabs_, &QTabWidget::currentChanged, this, [this](int index) {
    const int previous_tab_index = last_tab_index_;
    last_tab_index_ = index;
    if (index >= kFixedModeTabIndex && index <= kAirMarineModeTabIndex) {
      last_mode_tab_index_ = index;
    }
    if (previous_tab_index < 0 || previous_tab_index == index) {
      return;
    }
    if (receiver_combo_->currentIndex() < 0) {
      return;
    }

    // No scan-list UI swapping: RADAR_VIEW has its own independent scanner.

    // Show/hide signal_visualization_ and configure scan range viz immediately,
    // before StopReceiver — these are UI-only and safe to do unconditionally.
    if (signal_visualization_ != nullptr) {
      // Hide spectrum/waterfall widgets in RADAR_VIEW.
      signal_visualization_->setVisible(index != kScanRangeModeTabIndex &&
                                        index != kAirMarineModeTabIndex);
    }
    if (index == kScanRangeModeTabIndex && scan_range_viz_ != nullptr) {
      const double start = range_start_edit_->text().toDouble() * 1e6;
      const double end   = range_end_edit_->text().toDouble() * 1e6;
      const double step = static_cast<double>(sample_rate_spin_ ? sample_rate_spin_->value() : 2048000) * 0.9;
      bool fft_ok = false;
      const int fft_val = range_fft_size_combo_
                              ? range_fft_size_combo_->currentData().toInt(&fft_ok)
                              : 1024;
      scan_range_viz_->Configure(start, end, step, fft_ok ? fft_val : 1024);
      if (dwell_ms_spin_) scan_range_viz_->SetDwellMs(dwell_ms_spin_->value());
    }

    const uint32_t receiver_id = static_cast<uint32_t>(receiver_combo_->currentData().toUInt());
    std::string error;
    if (!client_->StopReceiver(receiver_id, &error)) {
      QMessageBox::warning(this, "StopReceiver failed", QString::fromStdString(error));
      return;
    }
    AppendLog(QString("Tab switch stop requested for receiver %1").arg(receiver_id));
    if (signal_visualization_ != nullptr) {
      signal_visualization_->SetChannelLabel(QString());
    }
    RefreshReceivers();
    ApplyModeAndConfig();
  });
  connect(dc_blocker_checkbox_, &QCheckBox::toggled, this, [this](bool enabled) {
    dc_blocker_cutoff_spin_->setEnabled(enabled);
  });
  connect(center_notch_checkbox_, &QCheckBox::toggled, this, [this](bool enabled) {
    center_notch_width_spin_->setEnabled(enabled);
  });
  connect(lo_offset_checkbox_, &QCheckBox::toggled, this, [this](bool enabled) {
    lo_offset_spin_->setEnabled(enabled);
  });
  connect(ppm_correction_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [this](int ppm) {
            v1::HardwareConfig hw;
            hw.set_ppm_correction(ppm);
            std::string err;
            if (client_ && !client_->SetHardwareConfig(hw, &err))
              AppendLog(QString("PPM save failed: %1").arg(QString::fromStdString(err)));
            if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig();
          });

  auto update_scan_noise_gate = [this]() {
    if (scan_range_viz_ == nullptr) return;
    const bool enabled = range_noise_gate_checkbox_->isChecked();
    const double db = range_noise_gate_spin_->value();
    const double span = kScanSpectrumCeilingDb - kScanSpectrumFloorDb;
    const double normalized = std::clamp((db - kScanSpectrumFloorDb) / span, 0.0, 1.0);
    scan_range_viz_->SetNoiseGate(enabled, normalized);
  };
  connect(range_noise_gate_checkbox_, &QCheckBox::toggled, this, [this, update_scan_noise_gate](bool checked) {
    range_noise_gate_spin_->setEnabled(checked);
    update_scan_noise_gate();
  });
  connect(range_noise_gate_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [update_scan_noise_gate](double) { update_scan_noise_gate(); });
  connect(range_db_ceiling_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [this](double val) {
            if (scan_range_viz_ != nullptr) scan_range_viz_->SetDbCeiling(val);
          });
  connect(scan_range_channel_bw_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [this](int val) {
            if (scan_range_viz_ != nullptr) scan_range_viz_->SetChannelBandwidthHz(val);
          });
  connect(scan_range_viz_, &ScanRangeVisualizationWidget::RangeSelected, this,
          [this](double start_hz, double end_hz) {
            range_start_edit_->setText(QString::number(start_hz / 1e6, 'f', 3));
            range_end_edit_->setText(QString::number(end_hz / 1e6, 'f', 3));
          });

  connect(client_.get(), &GrpcClient::ReceiverEventReceived, this, &MainWindow::OnReceiverEvent,
          Qt::QueuedConnection);
  connect(client_.get(), &GrpcClient::DecodedMessageReceived, this, &MainWindow::OnDecodedMessage,
          Qt::QueuedConnection);
  connect(client_.get(), &GrpcClient::AudioFrameReceived, this, &MainWindow::OnAudioFrame,
          Qt::QueuedConnection);
  connect(client_.get(), &GrpcClient::IqFrameReceived, this, &MainWindow::OnIqFrame,
          Qt::QueuedConnection);
  connect(client_.get(), &GrpcClient::StreamError, this, &MainWindow::OnStreamError,
          Qt::QueuedConnection);

  fixed_bandwidth_last_auto_hz_ = DefaultBandwidthHzForModulation(FixedModulationFromCombo(fixed_modulation_combo_));
  fixed_bandwidth_manual_override_ = false;

  QTimer::singleShot(0, this, [this]() {
    RefreshReceivers();
    client_->StartStreaming();
  });
}

MainWindow::~MainWindow() { client_->StopStreaming(); }

bool MainWindow::eventFilter(QObject* watched, QEvent* event) {
  if (scan_list_scroll_area_ != nullptr && watched == scan_list_scroll_area_->viewport() &&
      event != nullptr && event->type() == QEvent::Resize) {
    RefreshScanListChannelCards();
  }
  return QMainWindow::eventFilter(watched, event);
}

void MainWindow::RefreshReceivers() {
  /* Load persisted hardware config from server and populate PPM spinbox. */
  {
    v1::HardwareConfig hw;
    std::string hw_err;
    if (client_->GetHardwareConfig(&hw, &hw_err) && ppm_correction_spin_) {
      QSignalBlocker blocker(ppm_correction_spin_);
      ppm_correction_spin_->setValue(hw.ppm_correction());
    }
  }

  std::vector<v1::ReceiverInfo> receivers;
  std::string error;
  if (!client_->ListReceivers(&receivers, &error)) {
    QMessageBox::warning(this, "ListReceivers failed", QString::fromStdString(error));
    return;
  }

  const uint32_t previous_receiver_id =
      (receiver_combo_->currentIndex() >= 0) ? static_cast<uint32_t>(receiver_combo_->currentData().toUInt()) : 0;
  const int previous_receiver_filter_id =
      (receiver_filter_combo_->currentIndex() >= 0) ? receiver_filter_combo_->currentData().toInt() : -1;
  receiver_combo_->clear();
  receiver_filter_combo_->clear();
  receiver_filter_combo_->addItem("ALL", QVariant::fromValue(-1));
  std::vector<uint32_t> receiver_ids;
  receiver_ids.reserve(receivers.size());

  for (const auto& receiver : receivers) {
    const QString label = QString("#%1 %2 (%3)")
                              .arg(receiver.receiver_id())
                              .arg(QString::fromStdString(receiver.serial()))
                              .arg(receiver.running() ? "running" : "stopped");
    receiver_ids.push_back(receiver.receiver_id());
    receiver_combo_->addItem(label, QVariant::fromValue<int>(receiver.receiver_id()));
    receiver_filter_combo_->addItem(QString("#%1").arg(receiver.receiver_id()),
                                    QVariant::fromValue<int>(receiver.receiver_id()));
  }

  if (previous_receiver_id != 0) {
    const int restore_index = receiver_combo_->findData(QVariant::fromValue<int>(previous_receiver_id));
    if (restore_index >= 0) {
      receiver_combo_->setCurrentIndex(restore_index);
    }
  }

  int receiver_filter_id_to_apply = -1;
  if (previous_receiver_filter_id >= 0) {
    const int restore_filter_index =
        receiver_filter_combo_->findData(QVariant::fromValue<int>(previous_receiver_filter_id));
    if (restore_filter_index >= 0) {
      receiver_filter_combo_->setCurrentIndex(restore_filter_index);
      receiver_filter_id_to_apply = previous_receiver_filter_id;
    }
  }

  if (receiver_combo_->currentIndex() >= 0) {
    const uint32_t selected_id = static_cast<uint32_t>(receiver_combo_->currentData().toUInt());
    int active_mode_tab_index = mode_tabs_ != nullptr ? mode_tabs_->currentIndex() : kFixedModeTabIndex;
    if (active_mode_tab_index == kGlobalSettingsTabIndex) {
      active_mode_tab_index = last_mode_tab_index_;
    }
    const bool radar_scan_list_active = (active_mode_tab_index == kAirMarineModeTabIndex);
    if (receiver_filter_id_to_apply < 0) {
      const int selected_filter_index = receiver_filter_combo_->findData(QVariant::fromValue<int>(selected_id));
      if (selected_filter_index >= 0) {
        receiver_filter_combo_->setCurrentIndex(selected_filter_index);
        receiver_filter_id_to_apply = static_cast<int>(selected_id);
      }
    }
    for (const auto& receiver : receivers) {
      if (receiver.receiver_id() != selected_id) {
        continue;
      }
      if (fixed_modulation_combo_ != nullptr) {
        v1::Modulation fixed_modulation = receiver.mode_config().fixed_modulation();
        if (fixed_modulation == v1::MODULATION_UNSPECIFIED) {
          fixed_modulation = v1::MODULATION_WFM;
        }
        if (fixed_modulation != v1::MODULATION_NFM  && fixed_modulation != v1::MODULATION_WFM  &&
            fixed_modulation != v1::MODULATION_AM   && fixed_modulation != v1::MODULATION_FSK  &&
            fixed_modulation != v1::MODULATION_GMSK && fixed_modulation != v1::MODULATION_PPM &&
            fixed_modulation != v1::MODULATION_ADSB && fixed_modulation != v1::MODULATION_AIS_DUAL &&
            fixed_modulation != v1::MODULATION_VDES_ASM) {
          fixed_modulation = v1::MODULATION_NFM;
        }
        {
          const QSignalBlocker blocker(fixed_modulation_combo_);
          const int modulation_index =
              fixed_modulation_combo_->findData(QVariant::fromValue<int>(static_cast<int>(fixed_modulation)));
          if (modulation_index >= 0) {
            fixed_modulation_combo_->setCurrentIndex(modulation_index);
          }
        }
        /* Synka samplerate-lås för ADS-B */
        if (sample_rate_spin_ != nullptr) {
          const bool is_adsb = (fixed_modulation == v1::MODULATION_ADSB);
          if (is_adsb) {
            const QSignalBlocker sr_blocker(sample_rate_spin_);
            sample_rate_spin_->setValue(2400000);
          }
          sample_rate_spin_->setEnabled(!is_adsb);
        }
        const int fixed_bandwidth_hz = receiver.mode_config().channel_bandwidth_hz() > 0
            ? static_cast<int>(receiver.mode_config().channel_bandwidth_hz())
            : DefaultBandwidthHzForModulation(fixed_modulation);
        fixed_bandwidth_last_auto_hz_ = fixed_bandwidth_hz;
        fixed_bandwidth_manual_override_ = false;
        if (receiver.mode() == v1::RADIO_MODE_FIXED && channel_bandwidth_spin_ != nullptr) {
          fixed_bandwidth_sync_in_progress_ = true;
          channel_bandwidth_spin_->setValue(fixed_bandwidth_hz);
          if (fixed_channel_bandwidth_spin_ != nullptr) {
            fixed_channel_bandwidth_spin_->setValue(fixed_bandwidth_hz);
          }
          fixed_bandwidth_sync_in_progress_ = false;
        }
        if (fixed_sample_rate_warning_label_ != nullptr && sample_rate_spin_ != nullptr) {
          const bool digital_like = (fixed_modulation == v1::MODULATION_GMSK ||
                                     fixed_modulation == v1::MODULATION_VDES_ASM ||
                                     fixed_modulation == v1::MODULATION_AIS_DUAL ||
                                     fixed_modulation == v1::MODULATION_FSK);
          fixed_sample_rate_warning_label_->setVisible(
              digital_like && (sample_rate_spin_->value() != 2048000));
        }
      }
      const double receiver_default_squelch_db =
          std::clamp(receiver.mode_config().scan_list_default_squelch_db(), -120.0, 0.0);
      if (gmsk_baud_rate_spin_ != nullptr) {
        const QSignalBlocker blocker(gmsk_baud_rate_spin_);
        gmsk_baud_rate_spin_->setValue(static_cast<int>(receiver.mode_config().gmsk_baud_rate()));
      }
      if (gmsk_bt_spin_ != nullptr) {
        const QSignalBlocker blocker(gmsk_bt_spin_);
        gmsk_bt_spin_->setValue(receiver.mode_config().gmsk_bt());
      }
      if (gmsk_mod_index_spin_ != nullptr) {
        const QSignalBlocker blocker(gmsk_mod_index_spin_);
        gmsk_mod_index_spin_->setValue(receiver.mode_config().gmsk_modulation_index());
      }
      if (gmsk_decoder_combo_ != nullptr) {
        const QString decoder_value = QString("%1:%2")
                                          .arg(QString::fromStdString(receiver.mode_config().gmsk_decoder()))
                                          .arg(receiver.mode_config().gmsk_nrzi_invert() ? 1 : 0);
        const int decoder_idx = gmsk_decoder_combo_->findData(QVariant(decoder_value));
        if (decoder_idx >= 0) {
          const QSignalBlocker blocker(gmsk_decoder_combo_);
          gmsk_decoder_combo_->setCurrentIndex(decoder_idx);
        }
      }
      if (gmsk_postproc_combo_ != nullptr) {
        const QString postproc_value = QString::fromStdString(receiver.mode_config().gmsk_postprocessor());
        const int postproc_idx = gmsk_postproc_combo_->findData(QVariant(postproc_value));
        if (postproc_idx >= 0) {
          const QSignalBlocker blocker(gmsk_postproc_combo_);
          gmsk_postproc_combo_->setCurrentIndex(postproc_idx);
        }
      }
      if (vdes_bit_rate_spin_ != nullptr) {
        const QSignalBlocker blocker(vdes_bit_rate_spin_);
        vdes_bit_rate_spin_->setValue(static_cast<int>(receiver.mode_config().vdes_asm_bit_rate_bps()));
      }
      if (vdes_pll_bw_spin_ != nullptr) {
        const QSignalBlocker blocker(vdes_pll_bw_spin_);
        vdes_pll_bw_spin_->setValue(receiver.mode_config().vdes_asm_pll_bw());
      }
      if (vdes_candidate_bits_spin_ != nullptr) {
        const QSignalBlocker blocker(vdes_candidate_bits_spin_);
        vdes_candidate_bits_spin_->setValue(
            static_cast<int>(receiver.mode_config().vdes_asm_candidate_bits()));
      }
      if (vdes_sync_errors_spin_ != nullptr) {
        const QSignalBlocker blocker(vdes_sync_errors_spin_);
        vdes_sync_errors_spin_->setValue(
            static_cast<int>(receiver.mode_config().vdes_asm_sync_errors_max()));
      }
      adsb_agc_bandwidth_ = receiver.mode_config().adsb_agc_bandwidth();
      adsb_agc_target_level_ = receiver.mode_config().adsb_agc_target_level();
      {
        v1::Modulation fixed_modulation = receiver.mode_config().fixed_modulation();
        if (fixed_modulation == v1::MODULATION_UNSPECIFIED) {
          fixed_modulation = v1::MODULATION_WFM;
        }
        const QString dec = QString::fromStdString(receiver.mode_config().gmsk_decoder());
        const QString pp = QString::fromStdString(receiver.mode_config().gmsk_postprocessor());
        const bool use_vdes = (fixed_modulation == v1::MODULATION_VDES_ASM ||
                               dec == "vdes_asm_decoder" || pp == "vdes_asm_postproc");
        if (vdes_bit_rate_spin_ != nullptr) vdes_bit_rate_spin_->setEnabled(use_vdes);
        if (vdes_pll_bw_spin_ != nullptr) vdes_pll_bw_spin_->setEnabled(use_vdes);
        if (vdes_candidate_bits_spin_ != nullptr) vdes_candidate_bits_spin_->setEnabled(use_vdes);
        if (vdes_sync_errors_spin_ != nullptr) vdes_sync_errors_spin_->setEnabled(use_vdes);
      }
      if (scan_list_default_squelch_spin_ != nullptr) {
        const QSignalBlocker blocker(scan_list_default_squelch_spin_);
        scan_list_default_squelch_spin_->setValue(receiver_default_squelch_db);
      }
      const auto& channels = receiver.mode_config().scan_list_channels();
      if (!channels.empty()) {
        std::vector<ScanListChannelConfig> updated_channels;
        updated_channels.reserve(static_cast<size_t>(channels.size()));
        for (int index = 0; index < channels.size(); ++index) {
          const auto& channel = channels.Get(index);
          ScanListChannelConfig updated;
          updated.label = QString::fromStdString(channel.label());
          updated.frequency_mhz = channel.frequency_hz() > 0.0 ? channel.frequency_hz() / 1000000.0 : 0.0;
          updated.modulation = channel.modulation();
          updated.bandwidth_hz = static_cast<int>(channel.channel_bandwidth_hz());
          const double channel_squelch_db = channel.squelch_threshold_db();
          bool use_default_squelch = channel.use_default_squelch();
          if (!use_default_squelch &&
              std::abs(channel_squelch_db - receiver_default_squelch_db) < 0.05) {
            // Legacy configs had no explicit "use default" flag. Treat same-value channels as default.
            use_default_squelch = true;
          }
          updated.use_default_squelch = use_default_squelch;
          updated.squelch_threshold_db = channel_squelch_db;
          if (updated.use_default_squelch) {
            updated.squelch_threshold_db = receiver_default_squelch_db;
          }
          updated.dwell_ms = static_cast<int>(channel.dwell_ms());
          updated.audio_gain_db = static_cast<double>(channel.audio_gain_db());
          updated_channels.push_back(std::move(updated));
        }
        if (radar_scan_list_active) {
          radar_scan_list_channels_ = std::move(updated_channels);
        } else {
          scan_list_channels_ = std::move(updated_channels);
        }
      }
      scan_list_monitor_checkbox_->setChecked(receiver.mode_config().scan_list_monitor_mode());
      if (scan_list_default_squelch_spin_ != nullptr) {
        signal_visualization_->SetReceiverSquelchThresholdDb(selected_id,
                                                             scan_list_default_squelch_spin_->value());
      }
      break;
    }
  }
  RefreshScanListChannelCards();
  RefreshRadarScanListChannelCards();

  signal_visualization_->SetKnownReceivers(receiver_ids);
  signal_visualization_->SetReceiverFilter(receiver_filter_id_to_apply);

  AppendLog(QString("Refreshed %1 receivers").arg(receivers.size()));
}

void MainWindow::StartSelectedReceiver() {
  uint32_t receiver_id = 0;
  if (!CurrentReceiverId(&receiver_id)) {
    return;
  }

  // Ensure the currently visible frontend settings are sent before hardware starts streaming.
  QString apply_error;
  if (!ApplyModeAndConfigForReceiver(receiver_id, &apply_error)) {
    QMessageBox::warning(this, "Apply mode/config failed", apply_error);
    return;
  }

  std::string error;
  if (!client_->StartReceiver(receiver_id, &error)) {
    QMessageBox::warning(this, "StartReceiver failed", QString::fromStdString(error));
    return;
  }
  if (signal_visualization_ != nullptr) {
    signal_visualization_->ResetPeakHold();
  }
  scan_channel_heat_.clear();
  AppendLog(QString("Start requested for receiver %1").arg(receiver_id));
  RefreshReceivers();
}

void MainWindow::StopSelectedReceiver() {
  uint32_t receiver_id = 0;
  if (!CurrentReceiverId(&receiver_id)) {
    return;
  }

  std::string error;
  if (!client_->StopReceiver(receiver_id, &error)) {
    QMessageBox::warning(this, "StopReceiver failed", QString::fromStdString(error));
    return;
  }
  AppendLog(QString("Stop requested for receiver %1").arg(receiver_id));
  if (signal_visualization_ != nullptr) {
    signal_visualization_->SetChannelLabel(QString());
  }
  RefreshReceivers();
}

void MainWindow::ApplyModeAndConfig() {
  uint32_t receiver_id = 0;
  if (!CurrentReceiverId(&receiver_id)) {
    return;
  }

  QString error_text;
  if (!ApplyModeAndConfigForReceiver(receiver_id, &error_text)) {
    QMessageBox::warning(this, "SetModeConfig failed", error_text);
    return;
  }

  const QString fixed_demod_label =
      (fixed_modulation_combo_ != nullptr)
          ? fixed_modulation_combo_->currentText().trimmed()
          : QString("unknown");
  int log_channel_bw = channel_bandwidth_spin_ ? channel_bandwidth_spin_->value() : 0;
  if (mode_tabs_->currentIndex() == kFixedModeTabIndex && fixed_channel_bandwidth_spin_ != nullptr) {
    log_channel_bw = fixed_channel_bandwidth_spin_->value();
  }
  AppendLog(QString("Applied mode/config to receiver %1 (fixed-demod=%2, sample-rate=%3 Hz, channel-bw=%4 Hz, hw-bw=%5 Hz, dc=%6@%7 Hz, notch=%8@%9 Hz, lo-offset=%10@%11 Hz)")
                .arg(receiver_id)
                .arg(fixed_demod_label)
                .arg(sample_rate_spin_->value())
                .arg(log_channel_bw)
                .arg(hardware_bandwidth_spin_->value())
                .arg(dc_blocker_checkbox_->isChecked() ? "on" : "off")
                .arg(dc_blocker_cutoff_spin_->value())
                .arg(center_notch_checkbox_->isChecked() ? "on" : "off")
                .arg(center_notch_width_spin_->value())
                .arg(lo_offset_checkbox_->isChecked() ? "on" : "off")
                .arg(lo_offset_spin_->value()));
}

bool MainWindow::ApplyModeAndConfigForReceiver(uint32_t receiver_id, QString* error_text) {
  if (error_text != nullptr) {
    error_text->clear();
  }

  int mode_tab_index = mode_tabs_->currentIndex();
  if (mode_tab_index == kGlobalSettingsTabIndex) {
    mode_tab_index = last_mode_tab_index_;
  }
  const v1::RadioMode mode = ModeFromTabIndex(mode_tab_index);
  const bool radar_scan_list_active = (mode_tab_index == kAirMarineModeTabIndex);

  std::string error;
  if (!client_->SetMode(receiver_id, mode, &error)) {
    if (error_text != nullptr) {
      *error_text = QString::fromStdString(error);
    }
    return false;
  }

  v1::ModeConfig config;
  config.set_fixed_frequency_hz(fixed_frequency_edit_->text().toDouble() * 1000000.0);
  v1::Modulation fixed_modulation = v1::MODULATION_WFM;
  if (fixed_modulation_combo_ != nullptr) {
    bool value_ok = false;
    const int modulation_value = fixed_modulation_combo_->currentData().toInt(&value_ok);
    if (value_ok) {
      const auto parsed = static_cast<v1::Modulation>(modulation_value);
      if (parsed == v1::MODULATION_NFM      || parsed == v1::MODULATION_WFM  ||
          parsed == v1::MODULATION_AM       || parsed == v1::MODULATION_FSK  ||
          parsed == v1::MODULATION_GMSK     || parsed == v1::MODULATION_PPM  ||
          parsed == v1::MODULATION_ADSB     || parsed == v1::MODULATION_AIS_DUAL ||
          parsed == v1::MODULATION_VDES_ASM) {
        fixed_modulation = parsed;
      }
    }
  }
  if (mode == v1::RADIO_MODE_AIR_MARINE_PLOT) {
    config.set_fixed_frequency_hz(162000000.0);
    fixed_modulation = v1::MODULATION_AIS_DUAL;
  }
  /* In scan-list mode, if any channel uses AIS Dual let that take precedence
     so the backend selects ais_dual_demod without requiring the user to also
     set the Fixed tab's modulation combo. */
  if (mode == v1::RADIO_MODE_SCAN_LIST && fixed_modulation != v1::MODULATION_AIS_DUAL) {
    const auto& channels = radar_scan_list_active ? radar_scan_list_channels_ : scan_list_channels_;
    for (const auto& ch : channels) {
      if (ch.modulation == v1::MODULATION_AIS_DUAL) {
        fixed_modulation = v1::MODULATION_AIS_DUAL;
        break;
      }
    }
  }
  config.set_fixed_modulation(fixed_modulation);
  config.set_range_start_hz(range_start_edit_->text().toDouble() * 1e6);
  config.set_range_end_hz(range_end_edit_->text().toDouble() * 1e6);
  config.set_range_step_hz(static_cast<double>(sample_rate_spin_->value()) * 0.9);
  config.set_dwell_ms(static_cast<uint32_t>(dwell_ms_spin_->value()));
  config.set_sample_rate_hz(static_cast<uint32_t>(sample_rate_spin_->value()));
  int channel_bandwidth_hz = channel_bandwidth_spin_->value();
  if (mode == v1::RADIO_MODE_FIXED && fixed_channel_bandwidth_spin_ != nullptr) {
    channel_bandwidth_hz = fixed_channel_bandwidth_spin_->value();
    if (channel_bandwidth_spin_ != nullptr && channel_bandwidth_spin_->value() != channel_bandwidth_hz) {
      fixed_bandwidth_sync_in_progress_ = true;
      channel_bandwidth_spin_->setValue(channel_bandwidth_hz);
      fixed_bandwidth_sync_in_progress_ = false;
    }
  }
  config.set_channel_bandwidth_hz(static_cast<uint32_t>(channel_bandwidth_hz));
  config.set_hardware_bandwidth_hz(static_cast<uint32_t>(hardware_bandwidth_spin_->value()));
  config.set_dc_blocker_enabled(dc_blocker_checkbox_->isChecked());
  config.set_dc_blocker_cutoff_hz(static_cast<uint32_t>(dc_blocker_cutoff_spin_->value()));
  config.set_center_notch_enabled(center_notch_checkbox_->isChecked());
  config.set_center_notch_width_hz(static_cast<uint32_t>(center_notch_width_spin_->value()));
  config.set_lo_offset_enabled(lo_offset_checkbox_->isChecked());
  config.set_lo_offset_hz(static_cast<int32_t>(lo_offset_spin_->value()));
  config.set_scan_list_monitor_mode(
      scan_list_monitor_checkbox_ != nullptr && scan_list_monitor_checkbox_->isChecked());
  const int locked_index = radar_scan_list_active ? radar_frozen_scan_channel_index_ : frozen_scan_channel_index_;
  config.set_scan_list_channel_locked(locked_index >= 0);
  config.set_gmsk_baud_rate(gmsk_baud_rate_spin_ ? static_cast<uint32_t>(gmsk_baud_rate_spin_->value()) : 9600u);
  config.set_gmsk_bt(gmsk_bt_spin_ ? static_cast<float>(gmsk_bt_spin_->value()) : 0.4f);
  config.set_gmsk_modulation_index(gmsk_mod_index_spin_ ? static_cast<float>(gmsk_mod_index_spin_->value()) : 0.5f);
  {
    const QString decoder_raw = gmsk_decoder_combo_
        ? gmsk_decoder_combo_->currentData().toString() : QString();
    const QStringList parts = decoder_raw.split(':');
    config.set_gmsk_decoder(parts.value(0).toStdString());
    config.set_gmsk_nrzi_invert(parts.value(1, "0").toInt() != 0);
  }
  config.set_gmsk_postprocessor(gmsk_postproc_combo_
      ? gmsk_postproc_combo_->currentData().toString().toStdString() : "");
  config.set_vdes_asm_bit_rate_bps(
      vdes_bit_rate_spin_ ? static_cast<uint32_t>(vdes_bit_rate_spin_->value()) : 19200u);
  config.set_vdes_asm_pll_bw(
      vdes_pll_bw_spin_ ? static_cast<float>(vdes_pll_bw_spin_->value()) : 0.01f);
  config.set_vdes_asm_candidate_bits(
      vdes_candidate_bits_spin_ ? static_cast<uint32_t>(vdes_candidate_bits_spin_->value()) : 1056u);
  config.set_vdes_asm_sync_errors_max(
      vdes_sync_errors_spin_ ? static_cast<uint32_t>(vdes_sync_errors_spin_->value()) : 1u);
  config.set_adsb_agc_bandwidth(static_cast<float>(adsb_agc_bandwidth_));
  config.set_adsb_agc_target_level(static_cast<float>(adsb_agc_target_level_));
  config.set_ppm_bit_duration_us(
      ppm_bit_duration_us_spin_ ? static_cast<uint32_t>(ppm_bit_duration_us_spin_->value()) : 10u);
  {
    const double mbit = ppm_data_rate_mbit_spin_ ? ppm_data_rate_mbit_spin_->value() : 0.1;
    config.set_ppm_data_rate_bps(static_cast<uint32_t>(mbit * 1e6));
  }
  config.set_ppm_correction(ppm_correction_spin_ ? ppm_correction_spin_->value() : 0);
  config.set_scan_list_locked_channel_index(static_cast<uint32_t>(std::max(0, locked_index)));
  const double default_squelch_db =
      (scan_list_default_squelch_spin_ != nullptr) ? scan_list_default_squelch_spin_->value()
                                                   : kDefaultScanListSquelchDb;
  config.set_scan_list_default_squelch_db(default_squelch_db);
  config.set_audio_hpf300_enabled(
      (audio_hpf300_checkbox_ != nullptr && audio_hpf300_checkbox_->isChecked()) ||
      (fixed_audio_hpf300_checkbox_ != nullptr && fixed_audio_hpf300_checkbox_->isChecked()));
  config.set_audio_lpf3k5_enabled(
      (audio_lpf3k5_checkbox_ != nullptr && audio_lpf3k5_checkbox_->isChecked()) ||
      (fixed_audio_lpf3k5_checkbox_ != nullptr && fixed_audio_lpf3k5_checkbox_->isChecked()));
  config.set_audio_lpf4k5_enabled(
      (audio_lpf4k5_checkbox_ != nullptr && audio_lpf4k5_checkbox_->isChecked()) ||
      (fixed_audio_lpf4k5_checkbox_ != nullptr && fixed_audio_lpf4k5_checkbox_->isChecked()));
  config.set_audio_bpf_voice_enabled(
      (audio_bpf_voice_checkbox_ != nullptr && audio_bpf_voice_checkbox_->isChecked()) ||
      (fixed_audio_bpf_voice_checkbox_ != nullptr && fixed_audio_bpf_voice_checkbox_->isChecked()));
  {
    const bool rn_list = audio_rnnoise_checkbox_ != nullptr && audio_rnnoise_checkbox_->isChecked();
    const bool rn_fixed = fixed_audio_rnnoise_checkbox_ != nullptr && fixed_audio_rnnoise_checkbox_->isChecked();
    config.set_rnnoise_enabled(rn_list || rn_fixed);
    // Strength: use the active mode's spinner, fall back to the other.
    const bool use_fixed = (mode == v1::RADIO_MODE_FIXED);
    const float strength = use_fixed
        ? (fixed_audio_rnnoise_strength_spin_ != nullptr
               ? static_cast<float>(fixed_audio_rnnoise_strength_spin_->value()) : 100.0f)
        : (audio_rnnoise_strength_spin_ != nullptr
               ? static_cast<float>(audio_rnnoise_strength_spin_->value()) : 100.0f);
    config.set_rnnoise_strength(strength);
  }

  config.clear_scan_list_channels();
  config.clear_frequency_list_hz();
  const auto& scan_channels = radar_scan_list_active ? radar_scan_list_channels_ : scan_list_channels_;
  for (const auto& channel : scan_channels) {
    auto* out = config.add_scan_list_channels();
    out->set_label(channel.label.toStdString());
    out->set_frequency_hz(channel.frequency_mhz * 1000000.0);
    out->set_modulation(channel.modulation);
    out->set_channel_bandwidth_hz(static_cast<uint32_t>(std::max(0, channel.bandwidth_hz)));
    out->set_use_default_squelch(channel.use_default_squelch);
    out->set_squelch_threshold_db(channel.use_default_squelch ? default_squelch_db
                                                               : channel.squelch_threshold_db);
    out->set_dwell_ms(static_cast<uint32_t>(std::max(0, channel.dwell_ms)));
    out->set_audio_gain_db(static_cast<float>(channel.audio_gain_db));
    if (channel.frequency_mhz > 0.0 && mode != v1::RADIO_MODE_SCAN_RANGE) {
      config.add_frequency_list_hz(channel.frequency_mhz * 1000000.0);
    }
  }

  if (mode == v1::RADIO_MODE_SCAN_RANGE) {
    const double start_hz = config.range_start_hz();
    const double end_hz   = config.range_end_hz();
    const double step_hz  = config.range_step_hz();
    if (step_hz > 0.0 && end_hz > start_hz) {
      for (double f = start_hz; f <= end_hz + step_hz * 0.01; f += step_hz) {
        config.add_frequency_list_hz(f);
      }
    }
  }

  if (!client_->SetModeConfig(receiver_id, config, &error)) {
    if (error_text != nullptr) {
      *error_text = QString::fromStdString(error);
    }
    return false;
  }
  if (mode == v1::RADIO_MODE_SCAN_RANGE && scan_range_viz_ != nullptr) {
    const double vis_start = config.range_start_hz();
    const double vis_end   = config.range_end_hz();
    const double vis_step  = config.range_step_hz();
    bool fft_ok = false;
    const int fft_val = range_fft_size_combo_
                            ? range_fft_size_combo_->currentData().toInt(&fft_ok)
                            : 1024;
    scan_range_viz_->Configure(vis_start, vis_end, vis_step, fft_ok ? fft_val : 1024);
    if (dwell_ms_spin_) scan_range_viz_->SetDwellMs(dwell_ms_spin_->value());
  }
  return true;
}

void MainWindow::OnReceiverEvent(uint32_t receiver_id, int event_kind, double tuned_frequency_hz,
                                 const QString& message, quint64 unix_ms) {
  if (message.startsWith("AUDIO_PCM16 ")) {
    if (IsSelectedReceiver(receiver_id)) {
      // Backward compatibility with older servers where audio is sent over event stream.
      HandleAudioPcmEvent(message);
    }
    return;
  }
  if (message.startsWith("SCAN_STATUS ")) {
    ApplyScanListStatusEvent(receiver_id, message);
    return;
  }
  if (event_kind == static_cast<int>(v1::EVENT_KIND_TUNE_HOP)) {
    return;
  }
  if (message.startsWith("SCAN_SQUELCH_OPEN ") || message.startsWith("SCAN_SQUELCH_CLOSE ")) {
    bool idx_ok = false;
    const int channel_index = TokenValue(message, "idx").toInt(&idx_ok);
    QString channel_label;
    if (IsSelectedReceiver(receiver_id) && idx_ok && channel_index >= 0 &&
        static_cast<size_t>(channel_index) < scan_list_channels_.size()) {
      channel_label = scan_list_channels_[static_cast<size_t>(channel_index)].label.trimmed();
    }
    if (channel_label.isEmpty()) {
      channel_label = TokenValue(message, "label");
      channel_label.replace('_', ' ');
      if (channel_label.isEmpty()) {
        channel_label = idx_ok ? QString("Kanal %1").arg(channel_index + 1) : QString("Kanal ?");
      }
    }
    bool signal_ok = false;
    const double signal_db = TokenValue(message, "signal_db").toDouble(&signal_ok);
    bool threshold_ok = false;
    const double threshold_db = TokenValue(message, "threshold_db").toDouble(&threshold_ok);
    bool open_ms_ok = false;
    const qint64 open_ms = TokenValue(message, "open_ms").toLongLong(&open_ms_ok);

    if (message.startsWith("SCAN_SQUELCH_OPEN ")) {
      if (IsSelectedReceiver(receiver_id)) {
        audio_stream_seq_valid_ = false;
        audio_stream_sample_index_valid_ = false;
        audio_stream_last_sequence_ = 0;
        audio_stream_next_sample_index_ = 0;
      }
      AppendLog(QString("[%1] RX%2 SCAN squelch OPEN ch=%3 idx=%4 signal=%5 dB threshold=%6 dB")
                    .arg(ToLocalTime(unix_ms))
                    .arg(receiver_id)
                    .arg(channel_label)
                    .arg(idx_ok ? channel_index : -1)
                    .arg(signal_ok ? QString::number(signal_db, 'f', 1) : "?")
                    .arg(threshold_ok ? QString::number(threshold_db, 'f', 1) : "?"));
    } else {
      AppendLog(QString("[%1] RX%2 SCAN squelch CLOSE ch=%3 idx=%4 open=%5 ms signal=%6 dB threshold=%7 dB")
                    .arg(ToLocalTime(unix_ms))
                    .arg(receiver_id)
                    .arg(channel_label)
                    .arg(idx_ok ? channel_index : -1)
                    .arg(open_ms_ok ? open_ms : -1)
                    .arg(signal_ok ? QString::number(signal_db, 'f', 1) : "?")
                    .arg(threshold_ok ? QString::number(threshold_db, 'f', 1) : "?"));
    }
    return;
  }
  if (message.startsWith("IQ_STATS ")) {
    if (!IsSelectedReceiver(receiver_id)) {
      return;
    }
    bool cfg_sr_ok = false;
    const int configured_sample_rate_hz = TokenValue(message, "cfg_sr").toInt(&cfg_sr_ok);
    bool meas_sr_ok = false;
    const double measured_sample_rate_hz = TokenValue(message, "meas_sr").toDouble(&meas_sr_ok);
    bool block_sr_ok = false;
    const int block_sample_rate_hz = TokenValue(message, "block_sr").toInt(&block_sr_ok);
    bool tuned_hz_ok = false;
    const double tuned_hz = TokenValue(message, "tuned_hz").toDouble(&tuned_hz_ok);
    bool center_hz_ok = false;
    const double center_hz = TokenValue(message, "center_hz").toDouble(&center_hz_ok);
    bool level_ok = false;
    const double level_dbfs = TokenValue(message, "level_dbfs").toDouble(&level_ok);
    bool psd_peak_db_ok = false;
    const double psd_peak_db = TokenValue(message, "psd_peak_db").toDouble(&psd_peak_db_ok);
    bool psd_floor_db_ok = false;
    const double psd_floor_db = TokenValue(message, "psd_floor_db").toDouble(&psd_floor_db_ok);
    bool clip_pct_ok = false;
    const double clip_pct = TokenValue(message, "clip_pct").toDouble(&clip_pct_ok);
    bool clip_ok = false;
    const int clip_flag = TokenValue(message, "clip").toInt(&clip_ok);
    bool snr_ok = false;
    const double snr_db = TokenValue(message, "snr_db").toDouble(&snr_ok);
    bool psd_peak_hz_ok = false;
    const double psd_peak_offset_hz = TokenValue(message, "psd_peak_offset_hz").toDouble(&psd_peak_hz_ok);
    bool quality_ok = false;
    const double quality_score = TokenValue(message, "quality_score").toDouble(&quality_ok);
    bool raw_signal_ok_ok = false;
    const int raw_signal_ok_flag = TokenValue(message, "signal_ok_raw").toInt(&raw_signal_ok_ok);
    bool signal_ok_ok = false;
    const int signal_ok_flag = TokenValue(message, "signal_ok").toInt(&signal_ok_ok);
    bool status_sr_ok = false;
    const int status_sr = TokenValue(message, "ok_sr").toInt(&status_sr_ok);
    bool status_level_ok = false;
    const int status_level = TokenValue(message, "ok_level").toInt(&status_level_ok);
    bool status_clip_ok = false;
    const int status_clip = TokenValue(message, "ok_clip").toInt(&status_clip_ok);
    bool status_snr_ok = false;
    const int status_snr = TokenValue(message, "ok_snr").toInt(&status_snr_ok);
    bool status_stable_ok = false;
    const int status_stable = TokenValue(message, "ok_stable").toInt(&status_stable_ok);
    const QString raw_status = TokenValue(message, "raw_status");
    const QString signal_status = TokenValue(message, "signal_status");

    signal_visualization_->SetReceiverSignalLevelDb(receiver_id, level_ok ? level_dbfs : -120.0);
    signal_visualization_->SetReceiverIqHealth(
        receiver_id, psd_peak_db_ok ? psd_peak_db : -120.0, psd_floor_db_ok ? psd_floor_db : -120.0,
        snr_ok ? snr_db : 0.0, psd_peak_hz_ok ? psd_peak_offset_hz : 0.0, quality_ok,
        quality_ok ? quality_score : 0.0, signal_ok_ok, signal_ok_ok && signal_ok_flag != 0);

    return;
  }
  if (message.startsWith("AUDIO_STATS ")) {
    if (!IsSelectedReceiver(receiver_id)) {
      return;
    }
    audio_backend_stats_last_seen_at_ms_ = QDateTime::currentMSecsSinceEpoch();
    bool idx_ok = false;
    const int channel_index = TokenValue(message, "idx").toInt(&idx_ok);
    QString channel_label;
    if (idx_ok && channel_index >= 0 && static_cast<size_t>(channel_index) < scan_list_channels_.size()) {
      channel_label = scan_list_channels_[static_cast<size_t>(channel_index)].label.trimmed();
    }
    if (channel_label.isEmpty()) {
      channel_label = TokenValue(message, "label");
      channel_label.replace('_', ' ');
      if (channel_label.isEmpty()) {
        channel_label = idx_ok ? QString("Kanal %1").arg(channel_index + 1) : QString("Kanal ?");
      }
    }

    const QString modulation = TokenValue(message, "mod");
    bool sr_ok = false;
    const int sample_rate_hz = TokenValue(message, "sr").toInt(&sr_ok);
    bool cfg_sr_ok = false;
    const int configured_sample_rate_hz = TokenValue(message, "cfg_sr").toInt(&cfg_sr_ok);
    bool iq_sr_ok = false;
    const int iq_sample_rate_hz = TokenValue(message, "iq_sr").toInt(&iq_sr_ok);
    bool iq_est_sr_ok = false;
    const double iq_estimated_sample_rate_hz = TokenValue(message, "iq_est_sr").toDouble(&iq_est_sr_ok);
    bool win_ms_ok = false;
    const qint64 window_ms = TokenValue(message, "win_ms").toLongLong(&win_ms_ok);
    bool gen_hz_ok = false;
    const double generated_hz = TokenValue(message, "gen_hz").toDouble(&gen_hz_ok);
    bool pub_hz_ok = false;
    const double published_hz = TokenValue(message, "pub_hz").toDouble(&pub_hz_ok);
    bool gate_ok = false;
    const int gate = TokenValue(message, "gate").toInt(&gate_ok);
    bool squelch_ok = false;
    const int squelch = TokenValue(message, "squelch").toInt(&squelch_ok);
    bool signal_ok = false;
    const double signal_db = TokenValue(message, "signal_db").toDouble(&signal_ok);
    bool blocks_ok = false;
    const qint64 blocks = TokenValue(message, "blocks").toLongLong(&blocks_ok);
    bool open_blocks_ok = false;
    const qint64 open_blocks = TokenValue(message, "gate_open_blocks").toLongLong(&open_blocks_ok);
    bool demod_ok_blocks_ok = false;
    const qint64 demod_ok_blocks = TokenValue(message, "demod_ok").toLongLong(&demod_ok_blocks_ok);
    bool demod_empty_ok = false;
    const qint64 demod_empty_blocks = TokenValue(message, "demod_empty").toLongLong(&demod_empty_ok);
    bool gen_ok = false;
    const qint64 generated_samples = TokenValue(message, "gen_samples").toLongLong(&gen_ok);
    bool pub_frames_ok = false;
    const qint64 published_frames = TokenValue(message, "pub_frames").toLongLong(&pub_frames_ok);
    bool pub_samples_ok = false;
    const qint64 published_samples = TokenValue(message, "pub_samples").toLongLong(&pub_samples_ok);
    bool pending_ok = false;
    const qint64 pending_samples = TokenValue(message, "pending_samples").toLongLong(&pending_ok);
    bool conceal_ok = false;
    const qint64 conceal_samples = TokenValue(message, "conceal_samples").toLongLong(&conceal_ok);
    bool clears_ok = false;
    const qint64 clears = TokenValue(message, "clears").toLongLong(&clears_ok);
    bool flush_frames_ok = false;
    const qint64 flush_frames = TokenValue(message, "flush_frames").toLongLong(&flush_frames_ok);
    bool flush_samples_ok = false;
    const qint64 flush_samples = TokenValue(message, "flush_samples").toLongLong(&flush_samples_ok);

    return;
  }

  double peak_hz = 0.0;
  double peak_strength = 0.0;
  double frame_frequency_start_hz = 0.0;
  double frame_frequency_end_hz = 20000.0;
  std::vector<double> waveform;
  std::vector<double> spectrum;
  SignalVisualizationWidget::SpectrumSource source =
      SignalVisualizationWidget::SpectrumSource::kDemodulated;
  if (ParseVisualizationFrameEvent(message, &peak_hz, &peak_strength, &waveform, &spectrum, &source,
                                   &frame_frequency_start_hz, &frame_frequency_end_hz)) {
    signal_visualization_->PushVisualizationFrame(receiver_id, waveform, spectrum, peak_hz, peak_strength,
                                                  source, frame_frequency_start_hz,
                                                  frame_frequency_end_hz);
    return;
  }

  AppendLog(QString("[%1] RX%2 kind=%3 f=%4 %5")
                .arg(ToLocalTime(unix_ms))
                .arg(receiver_id)
                .arg(event_kind)
                .arg(tuned_frequency_hz, 0, 'f', 0)
                .arg(message));
}

void MainWindow::OnIqFrame(uint32_t receiver_id, int sample_rate_hz, const QByteArray& interleaved_iq_s16le,
                           quint64 unix_ms, double tuned_frequency_hz, quint64 sequence,
                           quint64 sample_index) {
  Q_UNUSED(unix_ms);
  Q_UNUSED(sequence);
  Q_UNUSED(sample_index);
  if (!iq_frame_seen_) {
    iq_frame_seen_ = true;
    AppendLog(QString("IQ stream active: RX%1 sr=%2 Hz tuned=%3 Hz")
                  .arg(receiver_id)
                  .arg(sample_rate_hz)
                  .arg(tuned_frequency_hz, 0, 'f', 0));
  }
  if (!IsSelectedReceiver(receiver_id)) {
    return;
  }

  const bool is_scan_range = (mode_tabs_ != nullptr &&
                               mode_tabs_->currentIndex() == kScanRangeModeTabIndex);
  const bool is_fixed_mode = (mode_tabs_ != nullptr &&
                              mode_tabs_->currentIndex() == kFixedModeTabIndex);

  const double half_rate_hz = std::max(1.0, static_cast<double>(sample_rate_hz) * 0.5);
  double frame_frequency_start_hz = tuned_frequency_hz - half_rate_hz;
  double frame_frequency_end_hz = tuned_frequency_hz + half_rate_hz;
  if (is_fixed_mode && fixed_channel_bandwidth_spin_ != nullptr &&
      fixed_channel_bandwidth_spin_->value() > 0) {
    const double half_bw_hz = std::max(1.0, static_cast<double>(fixed_channel_bandwidth_spin_->value()) * 0.5);
    frame_frequency_start_hz = tuned_frequency_hz - half_bw_hz;
    frame_frequency_end_hz = tuned_frequency_hz + half_bw_hz;
  }

  if (is_scan_range && scan_range_viz_ != nullptr) {
    // Use a fixed absolute dB scale so noise stays dark and real signals stand out.
    // Per-frame normalization (used by the normal viz) makes noise fill the full color
    // range which is inappropriate for a multi-frequency sweep waterfall.
    bool fft_ok = false;
    const int fft_val = range_fft_size_combo_
                            ? range_fft_size_combo_->currentData().toInt(&fft_ok)
                            : 0;
    const int scan_bins = fft_ok ? std::max(32, fft_val / 2) : 512;

    std::vector<int16_t> iq_s16;
    if (!DecodeInt16LeBytes(interleaved_iq_s16le, &iq_s16) || iq_s16.size() < 32) {
      return;
    }
    if ((iq_s16.size() % 2U) != 0U) iq_s16.pop_back();
    const size_t iq_pairs = iq_s16.size() / 2U;
    double i_sum = 0.0, q_sum = 0.0;
    std::vector<std::complex<double>> cx;
    cx.reserve(iq_pairs);
    for (size_t i = 0; i < iq_pairs; ++i) {
      const double iv = static_cast<double>(iq_s16[i * 2U]) / 32768.0;
      const double qv = static_cast<double>(iq_s16[i * 2U + 1U]) / 32768.0;
      i_sum += iv; q_sum += qv;
      cx.emplace_back(iv, qv);
    }
    if (iq_visual_dc_suppression_enabled_) {
      const double im = i_sum / static_cast<double>(iq_pairs);
      const double qm = q_sum / static_cast<double>(iq_pairs);
      for (auto& s : cx) s -= std::complex<double>(im, qm);
    }
    const std::vector<double> spectrum =
        BuildFixedScaleSpectrumFromComplex(cx, scan_bins,
                                           kScanSpectrumFloorDb, kScanSpectrumCeilingDb);
    if (spectrum.empty()) return;
    scan_range_viz_->PushSpectrum(spectrum, frame_frequency_start_hz,
                                   frame_frequency_end_hz, tuned_frequency_hz);
    return;
  }

  bool fft_ok = false;
  const int fft_val = 0;
  const int spectrum_bins = signal_visualization_->FftSize() / 2;

  std::vector<double> waveform;
  std::vector<double> spectrum;
  double signal_level_db = -120.0;
  BuildReceiverVisualizationFrame(interleaved_iq_s16le, spectrum_bins,
                                  iq_visual_dc_suppression_enabled_, &waveform, &spectrum,
                                  &signal_level_db);
  if (spectrum.empty()) {
    return;
  }

  signal_visualization_->SetReceiverSignalLevelDb(receiver_id, signal_level_db);
  signal_visualization_->PushVisualizationFrame(
      receiver_id, waveform, spectrum, 0.0, 0.0,
      SignalVisualizationWidget::SpectrumSource::kReceiverInput, frame_frequency_start_hz,
      frame_frequency_end_hz);
}

void MainWindow::OnAudioFrame(uint32_t receiver_id, int sample_rate_hz, const QByteArray& pcm_s16le,
                              quint64 unix_ms, double tuned_frequency_hz, quint64 sequence,
                              quint64 sample_index) {
  Q_UNUSED(unix_ms);
  Q_UNUSED(tuned_frequency_hz);
  if (!IsSelectedReceiver(receiver_id)) {
    ++audio_frontend_filtered_frames_;
    MaybeEmitFrontendAudioStats();
    return;
  }

  const int frame_bytes = pcm_s16le.size();
  const int frame_samples = frame_bytes / kAudioBytesPerSample;
  const int low_water_bytes = (sample_rate_hz > 0)
                                  ? std::max(1024, (sample_rate_hz * kAudioBytesPerSample *
                                                    kAudioGapFillLowWaterMs) /
                                                       1000)
                                  : 1024;
  const bool queue_near_underrun = audio_pending_pcm_.size() <= low_water_bytes;

  if (audio_stream_seq_valid_) {
    if (sequence != audio_stream_last_sequence_ + 1) {
      ++audio_frontend_missing_frame_events_;
      audio_stream_sample_index_valid_ = false;
    }
  }

  // Primary gap detection path: sample-index continuity from backend.
  if (sample_rate_hz > 0 && frame_samples > 0) {
    if (audio_stream_sample_index_valid_) {
      if (sample_index > audio_stream_next_sample_index_) {
        const quint64 missing_samples = sample_index - audio_stream_next_sample_index_;
        if (missing_samples > 0) {
          ++audio_frontend_missing_frame_events_;
          const quint64 missing_bytes = missing_samples * static_cast<quint64>(kAudioBytesPerSample);
          audio_frontend_missing_sample_bytes_ += missing_bytes;
          if (queue_near_underrun) {
            const quint64 max_fill_samples = static_cast<quint64>(std::llround(
                (kAudioGapFillMaxMs * static_cast<double>(sample_rate_hz)) / 1000.0));
            const quint64 fill_samples = std::min(missing_samples, max_fill_samples);
            const int silence_bytes = static_cast<int>(
                std::min<quint64>(fill_samples * static_cast<quint64>(kAudioBytesPerSample),
                                  static_cast<quint64>(std::numeric_limits<int>::max())));
            if (silence_bytes > 0) {
              QByteArray silence_pcm(silence_bytes, '\0');
              audio_frontend_gap_fill_bytes_ += static_cast<quint64>(silence_bytes);
              HandleAudioPcmFrame(sample_rate_hz, silence_pcm);
            }
          }
        }
      } else if (sample_index < audio_stream_next_sample_index_) {
        // Stream reset or out-of-order frame; resync expected cursor.
        audio_stream_sample_index_valid_ = false;
      }
    }
    audio_stream_next_sample_index_ = sample_index + static_cast<quint64>(frame_samples);
    audio_stream_sample_index_valid_ = true;
  } else {
    audio_stream_sample_index_valid_ = false;
  }
  audio_stream_last_sequence_ = sequence;
  audio_stream_seq_valid_ = true;

  ++audio_frontend_rx_frames_;
  audio_frontend_rx_bytes_ += static_cast<quint64>(pcm_s16le.size());
  audio_frontend_last_rx_sample_rate_hz_ = sample_rate_hz;

  std::vector<double> waveform;
  std::vector<double> spectrum;
  BuildDemodVisualizationFrame(pcm_s16le, signal_visualization_->FftSize() / 2, &waveform, &spectrum);
  if (!spectrum.empty()) {
    double nyquist_hz = std::max(1.0, static_cast<double>(sample_rate_hz) * 0.5);
    const bool is_fixed_mode = (mode_tabs_ != nullptr &&
                                mode_tabs_->currentIndex() == kFixedModeTabIndex);
    if (is_fixed_mode && fixed_channel_bandwidth_spin_ != nullptr &&
        fixed_channel_bandwidth_spin_->value() > 0) {
      nyquist_hz = std::min(nyquist_hz,
                            std::max(1.0, static_cast<double>(fixed_channel_bandwidth_spin_->value())));
    }
    signal_visualization_->PushVisualizationFrame(
        receiver_id, waveform, spectrum, 0.0, 1.0, SignalVisualizationWidget::SpectrumSource::kDemodulated,
        0.0, nyquist_hz);
  }

  HandleAudioPcmFrame(sample_rate_hz, pcm_s16le);
  MaybeEmitFrontendAudioStats();
}

void MainWindow::OnDecodedMessage(uint32_t receiver_id, const QString& signal_type, double frequency_hz,
                                  const QString& payload, const QVariantMap& fields, quint64 unix_ms) {
  MessageRow row;
  row.timestamp = QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(unix_ms)).toLocalTime();
  row.receiver_id = receiver_id;
  row.signal_type = signal_type;
  row.frequency_hz = frequency_hz;
  row.payload = payload;
  if (signal_type == "SIGNAL_TYPE_AIS") {
    row.decoded_summary = BuildAisDecodedSummary(payload, fields);
  } else if (signal_type == "SIGNAL_TYPE_DSC") {
    row.decoded_summary = BuildDscDecodedSummary(fields);
  }
  row.mmsi = FieldString(fields, "mmsi");
  row.lat = FieldString(fields, "lat");
  if (row.lat.isEmpty()) row.lat = FieldString(fields, "latitude");
  row.lon = FieldString(fields, "lon");
  if (row.lon.isEmpty()) row.lon = FieldString(fields, "longitude");
  row.sog = FieldString(fields, "sog");
  row.cog = FieldString(fields, "cog");
  const QStringList excluded_fields = {
      "mmsi", "lat", "lon", "latitude", "longitude", "sog", "cog"};
  QStringList other_parts;
  if (!row.payload.isEmpty()) {
    other_parts.append(QString("payload=%1").arg(row.payload));
  }
  if (!row.decoded_summary.isEmpty()) {
    other_parts.append(QString("decoded=%1").arg(row.decoded_summary));
  }
  for (auto it = fields.cbegin(); it != fields.cend(); ++it) {
    const QString key = it.key();
    if (excluded_fields.contains(key)) {
      continue;
    }
    const QString value = it.value().toString();
    if (value.isEmpty()) {
      continue;
    }
    other_parts.append(QString("%1=%2").arg(key, value));
  }
  row.other = other_parts.join(' ');

  // Log plugin-decoded digital frames (FSK, GMSK, NRZI, etc.)
  const QString plugin_type = fields.value("signal_type").toString();
  if (plugin_type == "FSK_DATA" || plugin_type == "GMSK_DATA" ||
      plugin_type == "GMSK_ASM_DATA" || plugin_type == "VDES_ASM_DATA" ||
      plugin_type == "NRZI_DATA" || plugin_type == "NRZI_ASM_DATA") {
    QString extra;
    if (plugin_type == "GMSK_DATA" || plugin_type == "GMSK_ASM_DATA") {
      extra = QString(" BT=%1").arg(fields.value("bt").toString());
      const QString branch = fields.value("branch").toString();
      if (!branch.isEmpty()) {
        extra += QString(" branch=%1").arg(branch);
      }
    } else if (plugin_type == "VDES_ASM_DATA") {
      extra = QString(" mode=%1").arg(fields.value("modulation").toString());
    } else if (plugin_type == "FSK_DATA") {
      extra = QString(" dev=%1Hz").arg(fields.value("deviation_hz").toString());
    } else if (plugin_type == "NRZI_DATA" || plugin_type == "NRZI_ASM_DATA") {
      const QString src = fields.value("source_type").toString();
      extra = src.isEmpty() ? QString() : QString(" src=%1").arg(src);
    }
    const QString baud = fields.value("baud_rate").toString();
    const QString bits = fields.value("bit_count").toString();
    const QString baud_part = baud.isEmpty() ? QString() : QString(" baud=%1").arg(baud);
    AppendLog(QString("[%1] RX%2 %3 f=%4Hz%5%6 bits=%7: %8")
                  .arg(row.timestamp.toString("HH:mm:ss"))
                  .arg(receiver_id)
                  .arg(plugin_type)
                  .arg(frequency_hz, 0, 'f', 0)
                  .arg(baud_part)
                  .arg(extra)
                  .arg(bits)
                  .arg(payload));
    return;
  }

  if (plugin_type == "PPM_DATA") {
    const QString bits    = fields.value("bit_count").toString();
    const QString bit_us  = fields.value("bit_duration_us").toString();
    const QString bit_part = bit_us.isEmpty() ? QString() : QString(" bit=%1\xc2\xb5s").arg(bit_us);
    AppendLog(QString("[%1] RX%2 PPM_DATA f=%3Hz%4 bits=%5: %6")
                  .arg(row.timestamp.toString("HH:mm:ss"))
                  .arg(receiver_id)
                  .arg(frequency_hz, 0, 'f', 0)
                  .arg(bit_part)
                  .arg(bits)
                  .arg(payload));
    if (fixed_hdlc_log_ != nullptr) {
      fixed_hdlc_log_->appendPlainText(
          QString("[%1] PPM%2 %3")
              .arg(row.timestamp.toString("HH:mm:ss"))
              .arg(bit_part)
              .arg(payload));
    }
    return;
  }

  // Live radar targets (AIS / ADS-B with position).
  if (radar_widget_ != nullptr) {
    // Update cached labels even when a message doesn't carry position.
    if (plugin_type.startsWith("AIS_")) {
      const QString mmsi = FieldString(fields, "mmsi").trimmed();
      if (!mmsi.isEmpty()) {
        const QString label = PreferNameOrCsOrAlias(fields, mmsi);
        if (!label.isEmpty()) {
          SaveNameAlias(mmsi, label);
          radar_widget_->UpdateTargetLabel(mmsi, label);
          if (visible_objects_widget_ != nullptr) {
            visible_objects_widget_->UpdateTargetLabel(mmsi, label);
            // Ensure the table reflects newly learned names even before we have a position fix.
            RadarTargetUpdate stub;
            stub.id = mmsi;
            stub.kind = RadarTargetKind::kVessel;
            stub.label = label;
            stub.lat = std::numeric_limits<double>::quiet_NaN();
            stub.lon = std::numeric_limits<double>::quiet_NaN();
            stub.unix_ms = static_cast<std::uint64_t>(unix_ms);
            visible_objects_widget_->UpsertTarget(stub);
          }
        }
      }
    } else if (plugin_type == "ADSB") {
      const QString icao = FieldString(fields, "icao").trimmed();
      if (!icao.isEmpty()) {
        const QString label = PreferNameOrCsOrAlias(fields, icao);
        if (!label.isEmpty()) {
          SaveNameAlias(icao, label);
          radar_widget_->UpdateTargetLabel(icao, label);
          if (visible_objects_widget_ != nullptr) {
            visible_objects_widget_->UpdateTargetLabel(icao, label);
            RadarTargetUpdate stub;
            stub.id = icao;
            stub.kind = RadarTargetKind::kAircraft;
            stub.label = label;
            stub.lat = std::numeric_limits<double>::quiet_NaN();
            stub.lon = std::numeric_limits<double>::quiet_NaN();
            stub.unix_ms = static_cast<std::uint64_t>(unix_ms);
            visible_objects_widget_->UpsertTarget(stub);
          }
        }
      }
    }

    double lat = 0.0;
    double lon = 0.0;
    const bool has_lat = ParseDoubleField(fields, "lat", &lat) || ParseDoubleField(fields, "latitude", &lat);
    const bool has_lon = ParseDoubleField(fields, "lon", &lon) || ParseDoubleField(fields, "longitude", &lon);
    if (has_lat && has_lon) {
      RadarTargetUpdate t;
      t.lat = lat;
      t.lon = lon;
      t.unix_ms = static_cast<std::uint64_t>(unix_ms);
      ParseDoubleField(fields, "sog", &t.sog);
      ParseDoubleField(fields, "cog", &t.cog);

      if (plugin_type.startsWith("AIS_")) {
        const QString mmsi = FieldString(fields, "mmsi");
        t.id = mmsi.isEmpty() ? QString("AIS@%1,%2").arg(lat, 0, 'f', 5).arg(lon, 0, 'f', 5) : mmsi;
        t.kind = RadarTargetKind::kVessel;
        t.label = PreferNameOrCsOrAlias(fields, t.id);
        if (!t.label.isEmpty()) SaveNameAlias(t.id, t.label);
        if (t.label.isEmpty()) t.label = t.id;
      } else if (plugin_type == "ADSB") {
        const QString icao = FieldString(fields, "icao");
        t.id = icao.isEmpty() ? QString("ADSB@%1,%2").arg(lat, 0, 'f', 5).arg(lon, 0, 'f', 5) : icao;
        t.kind = RadarTargetKind::kAircraft;
        t.label = PreferNameOrCsOrAlias(fields, t.id);
        if (!t.label.isEmpty()) SaveNameAlias(t.id, t.label);
        if (t.label.isEmpty()) t.label = t.id;
      } else {
        t.id = QString("%1@%2").arg(plugin_type).arg(receiver_id);
        t.kind = RadarTargetKind::kUnknown;
        t.label = t.id;
      }

      radar_widget_->UpsertTarget(t);
      if (visible_objects_widget_ != nullptr) visible_objects_widget_->UpsertTarget(t);
    }
  }

  // ADS-B / Mode S frame
  if (plugin_type == "ADSB") {
    const QString ts   = row.timestamp.toString("HH:mm:ss");
    const QString df   = fields.value("df").toString();
    const QString icao = fields.value("icao").toString();
    const QString bits = fields.value("bits").toString();
    AppendLog(QString("[%1] RX%2 ADS-B f=%3Hz DF=%4 ICAO=%5 bits=%6: %7")
                  .arg(ts)
                  .arg(receiver_id)
                  .arg(frequency_hz, 0, 'f', 0)
                  .arg(df)
                  .arg(icao)
                  .arg(bits)
                  .arg(payload));
    if (fixed_hdlc_log_ != nullptr)
      fixed_hdlc_log_->appendPlainText(
          QString("[%1] DF%2 %3 %4").arg(ts).arg(df).arg(icao).arg(payload));
    return;
  }

  // AIS decoded message — append to the fixed-channel HDLC log and main log
  if (plugin_type == "AIS_POS"  || plugin_type == "AIS_STAT" ||
      plugin_type == "AIS_STAT24" || plugin_type == "AIS_BBM"  ||
      plugin_type == "AIS_BSR"  || plugin_type == "AIS_ATON"  ||
      plugin_type == "AIS_OTHER") {
    const QString ts    = row.timestamp.toString("HH:mm:ss");
    const QString mmsi  = fields.value("mmsi").toString();
    const QString mtype = fields.value("msg_type").toString();
    if (fixed_hdlc_log_ != nullptr)
      fixed_hdlc_log_->appendPlainText(
          QString("[%1] T%2 %3").arg(ts).arg(mtype).arg(payload));
    AppendLog(QString("[%1] RX%2 %3 f=%4Hz: %5")
                  .arg(ts)
                  .arg(receiver_id)
                  .arg(plugin_type)
                  .arg(frequency_hz, 0, 'f', 0)
                  .arg(payload));
    all_rows_.push_back(row);
    AddMessageRow(row);
    return;
  }

  if (plugin_type == "AIS_MSG8" || plugin_type == "AIS_MSG8_OTHER" ||
      plugin_type == "ASM_MSG" || plugin_type == "ASM_OTHER") {
    const QString ts    = row.timestamp.toString("HH:mm:ss");
    const QString mtype = fields.value("msg_type").toString();
    if (fixed_hdlc_log_ != nullptr) {
      fixed_hdlc_log_->appendPlainText(
          QString("[%1] AIS-MSG8 T%2 %3").arg(ts).arg(mtype).arg(payload));
    }
    AppendLog(QString("[%1] RX%2 %3 f=%4Hz: %5")
                  .arg(ts)
                  .arg(receiver_id)
                  .arg(plugin_type)
                  .arg(frequency_hz, 0, 'f', 0)
                  .arg(payload));
    all_rows_.push_back(row);
    AddMessageRow(row);
    return;
  }

  if (plugin_type == "VDES_ASM_L2" || plugin_type == "VDES_ASM_TBD" ||
      plugin_type == "VDES_ASM_DIAG") {
    const QString ts = row.timestamp.toString("HH:mm:ss");
    AppendLog(QString("[%1] RX%2 %3 f=%4Hz: %5")
                  .arg(ts)
                  .arg(receiver_id)
                  .arg(plugin_type)
                  .arg(frequency_hz, 0, 'f', 0)
                  .arg(payload));
    return;
  }

  // HDLC frame with CRC OK — append to the fixed-channel HDLC log
  if (plugin_type == "HDLC_FRAME") {
    if (fixed_hdlc_log_ != nullptr) {
      const QString bc = fields.value("byte_count").toString();
      fixed_hdlc_log_->appendPlainText(
          QString("[%1] %2 B: %3")
              .arg(row.timestamp.toString("HH:mm:ss"))
              .arg(bc)
              .arg(payload));
    }
    return;
  }

  // Log HDLC periodic statistics
  if (plugin_type == "HDLC_STATS") {
    const quint64 total = fields.value("total").toULongLong();
    const quint64 ok    = fields.value("ok").toULongLong();
    const quint64 fail  = fields.value("fail").toULongLong();
    const int pct = (total > 0) ? static_cast<int>(ok * 100u / total) : 0;
    AppendLog(QString("[%1] RX%2 HDLC f=%3Hz: %4 tot, %5 ok, %6 fail (%7% ok)")
                  .arg(row.timestamp.toString("HH:mm:ss"))
                  .arg(receiver_id)
                  .arg(frequency_hz, 0, 'f', 0)
                  .arg(total)
                  .arg(ok)
                  .arg(fail)
                  .arg(pct));
    return;
  }

  auto field_text = [&fields](const QString& key) -> QString {
    if (!fields.contains(key)) {
      return {};
    }
    return fields.value(key).toString();
  };

  const QString kind = field_text("kind");
  if (kind == "candidate") {
    return;
  }
  if (signal_type == "SIGNAL_TYPE_AIS" && kind == "metric") {
    const QString channel = field_text("channel");
    auto parse_u64 = [&field_text](const QString& key, quint64 fallback = 0) -> quint64 {
      bool ok = false;
      const quint64 value = field_text(key).toULongLong(&ok);
      return ok ? value : fallback;
    };
    const QString channel_key = channel.isEmpty() ? QString("AIS?") : channel;
    const quint64 crc_ok_total =
        parse_u64("metric_crc_ok_channel", parse_u64("metric_crc_ok", 0));
    const quint64 crc_fail_total =
        parse_u64("metric_crc_fail_channel", parse_u64("metric_crc_fail", 0));
    const quint64 emitted_total = parse_u64("metric_emitted", 0);
    const quint64 interval_ms = 30000;

    AisCrcSummaryState& summary = ais_crc_summary_by_channel_[channel_key];
    if (summary.last_log_unix_ms == 0 || unix_ms <= summary.last_log_unix_ms ||
        (unix_ms - summary.last_log_unix_ms) >= interval_ms) {
      const quint64 ok_delta =
          (crc_ok_total >= summary.last_crc_ok) ? (crc_ok_total - summary.last_crc_ok) : crc_ok_total;
      const quint64 fail_delta = (crc_fail_total >= summary.last_crc_fail)
                                     ? (crc_fail_total - summary.last_crc_fail)
                                     : crc_fail_total;
      const quint64 emitted_delta = (emitted_total >= summary.last_emitted)
                                        ? (emitted_total - summary.last_emitted)
                                        : emitted_total;
      const double ratio = (crc_ok_total + crc_fail_total) == 0
                               ? 0.0
                               : static_cast<double>(crc_ok_total) /
                                     static_cast<double>(crc_ok_total + crc_fail_total);

      AppendLog(QString("[%1] RX%2 AIS CRC summary ch=%3 path=%4 ok=%5 (+%6) fail=%7 (+%8) emitted=%9 (+%10) ok_ratio=%11")
                    .arg(ToLocalTime(unix_ms))
                    .arg(receiver_id)
                    .arg(channel_key)
                    .arg(field_text("metric_decode_path"))
                    .arg(crc_ok_total)
                    .arg(ok_delta)
                    .arg(crc_fail_total)
                    .arg(fail_delta)
                    .arg(emitted_total)
                    .arg(emitted_delta)
                    .arg(ratio, 0, 'f', 4));

      summary.last_log_unix_ms = unix_ms;
      summary.last_crc_ok = crc_ok_total;
      summary.last_crc_fail = crc_fail_total;
      summary.last_emitted = emitted_total;
    }
    return;
  }

  QString summary = QString("[%1] RX%2 %3 f=%4")
                        .arg(ToLocalTime(unix_ms))
                        .arg(receiver_id)
                        .arg(signal_type)
                        .arg(frequency_hz, 0, 'f', 0);
  const QString mmsi = field_text("mmsi");
  const QString msg_type = field_text("msg_type");
  const QString channel = field_text("channel");
  if (!mmsi.isEmpty()) {
    summary += QString(" mmsi=%1").arg(mmsi);
  }
  if (!msg_type.isEmpty()) {
    summary += QString(" type=%1").arg(msg_type);
  }
  if (!channel.isEmpty()) {
    summary += QString(" ch=%1").arg(channel);
  }
  if (!row.decoded_summary.isEmpty()) {
    summary += QString(" decoded=%1").arg(row.decoded_summary);
  }
  summary += QString(" payload=%1").arg(payload.left(96));
  AppendLog(summary);

  const QString metric_blocks = field_text("metric_blocks");
  if (!metric_blocks.isEmpty() && signal_type == "SIGNAL_TYPE_AIS") {
    AppendLog(QString("AIS metrics: blocks=%1 flags=%2 candidates=%3 crc_ok=%4 crc_fail=%5 dup=%6 emitted=%7 demod_ready=%8")
                  .arg(metric_blocks)
                  .arg(field_text("metric_flags"))
                  .arg(field_text("metric_candidates"))
                  .arg(field_text("metric_crc_ok"))
                  .arg(field_text("metric_crc_fail"))
                  .arg(field_text("metric_duplicates"))
                  .arg(field_text("metric_emitted"))
                  .arg(field_text("metric_demod_ready")));
  } else if (!metric_blocks.isEmpty() && signal_type == "SIGNAL_TYPE_DSC") {
    AppendLog(QString("DSC metrics: blocks=%1 candidates=%2 dup=%3 emitted=%4 frames=%5 ecc_ok=%6 ecc_fail=%7")
                  .arg(metric_blocks)
                  .arg(field_text("metric_candidates"))
                  .arg(field_text("metric_duplicates"))
                  .arg(field_text("metric_emitted"))
                  .arg(field_text("metric_frames_parsed"))
                  .arg(field_text("metric_ecc_ok"))
                  .arg(field_text("metric_ecc_fail")));
  }

  all_rows_.push_back(row);
  AddMessageRow(row);
}

bool MainWindow::IsSelectedReceiver(uint32_t receiver_id) const {
  if (receiver_combo_->currentIndex() < 0) {
    return false;
  }
  return static_cast<uint32_t>(receiver_combo_->currentData().toUInt()) == receiver_id;
}

QString MainWindow::ScanListChannelCardText(int index) const {
  if (index < 0 || static_cast<size_t>(index) >= scan_list_channels_.size()) {
    return {};
  }
  const auto& channel = scan_list_channels_[static_cast<size_t>(index)];
  const QString label = channel.label.trimmed().isEmpty()
                            ? QString("Kanal %1").arg(index + 1)
                            : channel.label.trimmed();
  const QString freq_text = channel.frequency_mhz > 0.0
                                ? QString("%1 MHz").arg(channel.frequency_mhz, 0, 'f', 3)
                                : QString("Frekvens ej satt");
  return QString("%1\n%2").arg(label).arg(freq_text);
}

QString MainWindow::ScanListChannelCardStyle(int index) const {
  constexpr auto kBase =
      "QPushButton { text-align: left; padding: 6px 10px; border-radius: 6px; ";
  if (frozen_scan_channel_index_ == index) {
    return QString(kBase) +
           "border: 2px solid #00BCD4; background: #0B1018; color: #80DEEA; }";
  }
  if (active_scan_list_channel_index_ == index &&
      active_scan_list_channel_state_ == ScanListChannelState::kSquelchOpen) {
    return QString(kBase) +
           "border: 2px solid #2E7D32; background: #0B1018; color: #5CDB95; }";
  }

  // Compute heat for all non-squelch-open states (permanent — no time decay).
  double heat = 0.0;
  if (index >= 0 && static_cast<size_t>(index) < scan_channel_heat_.size()) {
    heat = scan_channel_heat_[static_cast<size_t>(index)].value;
  }
  const double t = std::sqrt(std::clamp(heat, 0.0, 1.0));  // sqrt for fast initial response

  const auto lerp = [](int a, int b, double f) {
    return static_cast<int>(a + f * (b - a));
  };
  const auto toHex = [](int r, int g, int b) {
    return QString("#%1%2%3").arg(r, 2, 16, QChar('0'))
                             .arg(g, 2, 16, QChar('0'))
                             .arg(b, 2, 16, QChar('0'));
  };

  // Text: #163803 (22,56,3) → #5CDB95 (92,219,149)
  const QString text_color = toHex(lerp(22,92,t), lerp(56,219,t), lerp(3,149,t));
  // Background: #0B1018 (11,16,24) → #0D2012 (13,32,18) subtle green tint
  const QString bg_color = toHex(lerp(11,13,t), lerp(16,32,t), lerp(24,18,t));
  // Border: #1E2A38 → #2E7D32, always 2px when heat > 0
  const QString border_color = toHex(lerp(30,46,t), lerp(42,125,t), lerp(56,50,t));
  const int border_px = t > 0.01 ? 2 : 1;

  if (active_scan_list_channel_index_ == index &&
      active_scan_list_channel_state_ == ScanListChannelState::kSquelchClosed) {
    return QString(kBase) +
           QString("border: 2px solid #EF6C00; background: %1; color: %2; }")
               .arg(bg_color).arg(text_color);
  }
  return QString(kBase) +
         QString("border: %1px solid %2; background: %3; color: %4; }")
             .arg(border_px).arg(border_color).arg(bg_color).arg(text_color);
}

void MainWindow::RefreshScanListChannelCards() {
  if (scan_list_grid_layout_ == nullptr || scan_list_grid_widget_ == nullptr) {
    return;
  }

  while (scan_list_channel_buttons_.size() > scan_list_channels_.size()) {
    QPushButton* button = scan_list_channel_buttons_.back();
    scan_list_channel_buttons_.pop_back();
    scan_list_grid_layout_->removeWidget(button);
    button->deleteLater();
  }

  while (scan_list_channel_buttons_.size() < scan_list_channels_.size()) {
    auto* channel_button = new QPushButton(scan_list_grid_widget_);
    channel_button->setMinimumHeight(56);
    channel_button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    channel_button->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(channel_button, &QPushButton::clicked, this, [this, channel_button]() {
      const auto it = std::find(scan_list_channel_buttons_.begin(), scan_list_channel_buttons_.end(),
                                channel_button);
      if (it == scan_list_channel_buttons_.end()) {
        return;
      }
      const int index = static_cast<int>(std::distance(scan_list_channel_buttons_.begin(), it));
      ConfigureScanListChannel(index);
    });
    connect(channel_button, &QPushButton::customContextMenuRequested, this,
            [this, channel_button](const QPoint& pos) {
              const auto it = std::find(scan_list_channel_buttons_.begin(),
                                        scan_list_channel_buttons_.end(), channel_button);
              if (it == scan_list_channel_buttons_.end()) return;
              const int index =
                  static_cast<int>(std::distance(scan_list_channel_buttons_.begin(), it));
              QMenu menu(this);
              if (frozen_scan_channel_index_ == index) {
                menu.addAction("Lås upp scanner", [this]() {
                  frozen_scan_channel_index_ = -1;
                  RefreshScanListChannelCards();
                  if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig();
                });
              } else {
                menu.addAction("Frys scanner på denna kanal", [this, index]() {
                  frozen_scan_channel_index_ = index;
                  RefreshScanListChannelCards();
                  if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig();
                });
              }
              menu.exec(channel_button->mapToGlobal(pos));
            });
    scan_list_channel_buttons_.push_back(channel_button);
  }

  int max_text_width = 0;
  // Phase 1: update text + style (always); font metrics only when layout needs rebuilding.
  for (size_t idx = 0; idx < scan_list_channel_buttons_.size(); ++idx) {
    QPushButton* button = scan_list_channel_buttons_[idx];
    if (button == nullptr) continue;
    const int index = static_cast<int>(idx);
    const QString text = ScanListChannelCardText(index);
    button->setText(text);
    button->setStyleSheet(ScanListChannelCardStyle(index));
    // Font metrics are expensive; use cached button width on normal refreshes.
    if (scan_list_last_button_width_ == 0) {
      const QFontMetrics metrics(button->font());
      for (const QString& line : text.split('\n')) {
        max_text_width = std::max(max_text_width, metrics.horizontalAdvance(line));
      }
    }
  }

  // Phase 2: compute layout geometry.
  const int button_width = std::max(140, max_text_width + 24);
  int columns = 1;
  if (scan_list_scroll_area_ != nullptr && scan_list_scroll_area_->viewport() != nullptr) {
    int left = 0, top = 0, right = 0, bottom = 0;
    scan_list_grid_layout_->getContentsMargins(&left, &top, &right, &bottom);
    int spacing = scan_list_grid_layout_->horizontalSpacing();
    if (spacing < 0) spacing = 8;
    const int available_width =
        std::max(0, scan_list_scroll_area_->viewport()->width() - left - right);
    columns = std::max(1, (available_width + spacing) / (button_width + spacing));
  }

  // Phase 3: rebuild grid layout only when structure changed.
  const bool layout_changed = (columns != scan_list_last_columns_) ||
                              (button_width != scan_list_last_button_width_);
  if (layout_changed) {
    scan_list_last_columns_ = columns;
    scan_list_last_button_width_ = button_width;
  }
  if (layout_changed) {
  for (size_t idx = 0; idx < scan_list_channel_buttons_.size(); ++idx) {
    QPushButton* button = scan_list_channel_buttons_[idx];
    if (button == nullptr) continue;
    button->setFixedWidth(button_width);
    const int index = static_cast<int>(idx);
    scan_list_grid_layout_->addWidget(button, index / columns, index % columns,
                                      Qt::AlignLeft | Qt::AlignTop);
  }
  } // end if (layout_changed)
}

void MainWindow::RefreshRadarScanListChannelCards() {
  if (radar_scan_list_grid_layout_ == nullptr || radar_scan_list_grid_widget_ == nullptr) return;

  while (radar_scan_list_channel_buttons_.size() > radar_scan_list_channels_.size()) {
    QPushButton* button = radar_scan_list_channel_buttons_.back();
    radar_scan_list_channel_buttons_.pop_back();
    radar_scan_list_grid_layout_->removeWidget(button);
    button->deleteLater();
  }
  while (radar_scan_list_channel_buttons_.size() < radar_scan_list_channels_.size()) {
    auto* channel_button = new QPushButton(radar_scan_list_grid_widget_);
    channel_button->setMinimumHeight(56);
    channel_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    channel_button->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(channel_button, &QPushButton::clicked, this, [this, channel_button]() {
      const auto it = std::find(radar_scan_list_channel_buttons_.begin(),
                                radar_scan_list_channel_buttons_.end(), channel_button);
      if (it == radar_scan_list_channel_buttons_.end()) return;
      const int index = static_cast<int>(std::distance(radar_scan_list_channel_buttons_.begin(), it));
      ConfigureRadarScanListChannel(index);
    });
    connect(channel_button, &QPushButton::customContextMenuRequested, this,
            [this, channel_button](const QPoint& pos) {
              const auto it = std::find(radar_scan_list_channel_buttons_.begin(),
                                        radar_scan_list_channel_buttons_.end(), channel_button);
              if (it == radar_scan_list_channel_buttons_.end()) return;
              const int index =
                  static_cast<int>(std::distance(radar_scan_list_channel_buttons_.begin(), it));
              QMenu menu(this);
              if (radar_frozen_scan_channel_index_ == index) {
                menu.addAction("Unfreeze scanner", [this]() {
                  radar_frozen_scan_channel_index_ = -1;
                  RefreshRadarScanListChannelCards();
                  if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig();
                });
              } else {
                menu.addAction("Freeze scanner on this channel", [this, index]() {
                  radar_frozen_scan_channel_index_ = index;
                  RefreshRadarScanListChannelCards();
                  if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig();
                });
              }
              menu.addSeparator();
              menu.addAction("Remove channel", [this, index]() { RemoveRadarScanListChannel(index); });
              menu.exec(channel_button->mapToGlobal(pos));
            });
    radar_scan_list_channel_buttons_.push_back(channel_button);
  }

  for (size_t idx = 0; idx < radar_scan_list_channel_buttons_.size(); ++idx) {
    QPushButton* button = radar_scan_list_channel_buttons_[idx];
    if (button == nullptr) continue;
    const int index = static_cast<int>(idx);
    const auto& ch = radar_scan_list_channels_[static_cast<size_t>(index)];
    const QString label = ch.label.trimmed().isEmpty() ? QString("Kanal %1").arg(index + 1) : ch.label.trimmed();
    const QString freq = ch.frequency_mhz > 0.0 ? QString("%1 MHz").arg(ch.frequency_mhz, 0, 'f', 3) : "Frekvens ej satt";
    button->setText(QString("%1\n%2").arg(label).arg(freq));
    const bool frozen = (radar_frozen_scan_channel_index_ == index);
    const QString border = frozen ? "border:2px solid #00BCD4;" : "border:1px solid #0f4a0f;";
    button->setStyleSheet(QString("QPushButton { text-align:left; padding:4px 8px; border-radius:6px; %1 "
                                  "background:#001000; color:#9be89b; }")
                              .arg(border));
  }

  // Two-column layout.
  for (int i = radar_scan_list_grid_layout_->count() - 1; i >= 0; --i) {
    auto* item = radar_scan_list_grid_layout_->itemAt(i);
    if (item && item->widget()) {
      radar_scan_list_grid_layout_->removeWidget(item->widget());
    }
  }
  constexpr int kColumns = 2;
  for (size_t idx = 0; idx < radar_scan_list_channel_buttons_.size(); ++idx) {
    const int row = static_cast<int>(idx) / kColumns;
    const int col = static_cast<int>(idx) % kColumns;
    radar_scan_list_grid_layout_->addWidget(radar_scan_list_channel_buttons_[idx], row, col, 1, 1);
  }
}

void MainWindow::ConfigureScanListChannel(int index) {
  if (index < 0 || static_cast<size_t>(index) >= scan_list_channels_.size()) {
    return;
  }
  ScanListChannelConfig channel = scan_list_channels_[static_cast<size_t>(index)];

  QDialog dialog(this);
  dialog.setWindowTitle(QString("Konfigurera kanal %1").arg(index + 1));
  auto* layout = new QFormLayout(&dialog);

  auto* label_edit = new QLineEdit(channel.label, &dialog);
  auto* frequency_spin = new QDoubleSpinBox(&dialog);
  frequency_spin->setDecimals(3);
  frequency_spin->setRange(0.0, 6000.0);
  frequency_spin->setSingleStep(0.025);
  frequency_spin->setSuffix(" MHz");
  frequency_spin->setValue(channel.frequency_mhz);

  auto* modulation_combo = new QComboBox(&dialog);
  modulation_combo->addItem("AM");
  modulation_combo->addItem("WFM");
  modulation_combo->addItem("NFM");
  modulation_combo->addItem("AIS Dual");
  modulation_combo->addItem("VDES ASM");
  modulation_combo->setCurrentText(ModulationLabel(channel.modulation));

  auto* bandwidth_spin = new QSpinBox(&dialog);
  bandwidth_spin->setRange(2000, 500000);
  bandwidth_spin->setSingleStep(1000);
  bandwidth_spin->setSuffix(" Hz");
  const int channel_bandwidth =
      channel.bandwidth_hz > 0 ? channel.bandwidth_hz : DefaultBandwidthHzForModulation(channel.modulation);
  bandwidth_spin->setValue(channel_bandwidth);

  auto* squelch_spin = new QDoubleSpinBox(&dialog);
  squelch_spin->setDecimals(1);
  squelch_spin->setRange(-120.0, 0.0);
  squelch_spin->setSingleStep(1.0);
  squelch_spin->setSuffix(" dB");
  squelch_spin->setValue(channel.squelch_threshold_db);
  auto* use_default_squelch_checkbox = new QCheckBox("Use default squelch", &dialog);
  use_default_squelch_checkbox->setChecked(channel.use_default_squelch);
  if (channel.use_default_squelch && scan_list_default_squelch_spin_ != nullptr) {
    squelch_spin->setValue(scan_list_default_squelch_spin_->value());
  }
  squelch_spin->setEnabled(!channel.use_default_squelch);

  auto* dwell_spin = new QSpinBox(&dialog);
  dwell_spin->setRange(0, 60000);
  dwell_spin->setSingleStep(100);
  dwell_spin->setSpecialValueText("Use default");
  dwell_spin->setSuffix(" ms");
  dwell_spin->setValue(channel.dwell_ms);

  auto* gain_spin = new QDoubleSpinBox(&dialog);
  gain_spin->setDecimals(1);
  gain_spin->setRange(-20.0, 40.0);
  gain_spin->setSingleStep(1.0);
  gain_spin->setSuffix(" dB");
  gain_spin->setValue(channel.audio_gain_db);
  gain_spin->setToolTip("Audio gain applied after squelch and filters. 0 dB = no change.");

  connect(modulation_combo, &QComboBox::currentTextChanged, this, [bandwidth_spin](const QString& text) {
    bandwidth_spin->setValue(DefaultBandwidthHzForModulation(ModulationFromText(text)));
  });
  connect(use_default_squelch_checkbox, &QCheckBox::toggled, this, [this, squelch_spin](bool checked) {
    squelch_spin->setEnabled(!checked);
    if (checked && scan_list_default_squelch_spin_ != nullptr) {
      squelch_spin->setValue(scan_list_default_squelch_spin_->value());
    }
  });

  layout->addRow("Label", label_edit);
  layout->addRow("Frekvens", frequency_spin);
  layout->addRow("Modulation", modulation_combo);
  layout->addRow("Bandbredd", bandwidth_spin);
  layout->addRow(use_default_squelch_checkbox);
  layout->addRow("Squelch threshold", squelch_spin);
  layout->addRow("Kanal dwell", dwell_spin);
  layout->addRow("Ljud gain", gain_spin);

  auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  auto* delete_button = buttons->addButton("Delete", QDialogButtonBox::DestructiveRole);
  layout->addRow(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  connect(delete_button, &QPushButton::clicked, &dialog, [&dialog]() { dialog.done(1001); });

  const int dialog_result = dialog.exec();
  if (dialog_result == 1001) {
    RemoveScanListChannel(index);
    return;
  }
  if (dialog_result != QDialog::Accepted) {
    return;
  }

  channel.label = label_edit->text().trimmed();
  channel.frequency_mhz = frequency_spin->value();
  channel.modulation = ModulationFromText(modulation_combo->currentText());
  channel.bandwidth_hz = bandwidth_spin->value();
  channel.use_default_squelch = use_default_squelch_checkbox->isChecked();
  if (channel.use_default_squelch && scan_list_default_squelch_spin_ != nullptr) {
    channel.squelch_threshold_db = scan_list_default_squelch_spin_->value();
  } else {
    channel.squelch_threshold_db = squelch_spin->value();
  }
  channel.dwell_ms = dwell_spin->value();
  channel.audio_gain_db = gain_spin->value();
  scan_list_channels_[static_cast<size_t>(index)] = channel;
  SaveScanListConfigToSettingsGroup(ActiveScanListSettingsGroup(mode_tabs_));
  RefreshScanListChannelCards();

  if (receiver_combo_->currentIndex() >= 0) {
    ApplyModeAndConfig();
  }
}

void MainWindow::ConfigureRadarScanListChannel(int index) {
  if (index < 0 || static_cast<size_t>(index) >= radar_scan_list_channels_.size()) return;
  ScanListChannelConfig channel = radar_scan_list_channels_[static_cast<size_t>(index)];

  QDialog dialog(this);
  dialog.setWindowTitle(QString("Radar scanner channel %1").arg(index + 1));
  auto* layout = new QFormLayout(&dialog);

  auto* label_edit = new QLineEdit(channel.label, &dialog);
  auto* freq_spin = new QDoubleSpinBox(&dialog);
  freq_spin->setDecimals(3);
  freq_spin->setRange(0.0, 6000.0);
  freq_spin->setValue(channel.frequency_mhz);
  freq_spin->setSuffix(" MHz");

  auto* modulation_combo = new QComboBox(&dialog);
  modulation_combo->addItem("NFM");
  modulation_combo->addItem("FSK");
  modulation_combo->addItem("GMSK");
  modulation_combo->addItem("VDES ASM");
  modulation_combo->addItem("AIS Dual");
  modulation_combo->addItem("ADS-B");
  modulation_combo->setCurrentText(ModulationLabel(channel.modulation));

  auto* bandwidth_spin = new QSpinBox(&dialog);
  bandwidth_spin->setRange(0, 5000000);
  bandwidth_spin->setSingleStep(1000);
  bandwidth_spin->setValue(channel.bandwidth_hz);
  bandwidth_spin->setSuffix(" Hz");

  layout->addRow("Label", label_edit);
  layout->addRow("Frequency", freq_spin);
  layout->addRow("Modulation", modulation_combo);
  layout->addRow("Bandwidth", bandwidth_spin);

  auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  layout->addRow(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

  if (dialog.exec() != QDialog::Accepted) return;

  channel.label = label_edit->text().trimmed();
  channel.frequency_mhz = freq_spin->value();
  channel.modulation = ModulationFromText(modulation_combo->currentText());
  channel.bandwidth_hz = bandwidth_spin->value();
  if (channel.bandwidth_hz <= 0) channel.bandwidth_hz = DefaultBandwidthHzForModulation(channel.modulation);

  radar_scan_list_channels_[static_cast<size_t>(index)] = channel;
  SaveRadarScanListConfigToSettings();
  RefreshRadarScanListChannelCards();
  if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig();
}

void MainWindow::AddScanListChannel() {
  ScanListChannelConfig channel;
  channel.label = QString("Kanal %1").arg(scan_list_channels_.size() + 1);
  channel.modulation = v1::MODULATION_NFM;
  channel.bandwidth_hz = DefaultBandwidthHzForModulation(channel.modulation);
  channel.use_default_squelch = true;
  if (scan_list_default_squelch_spin_ != nullptr) {
    channel.squelch_threshold_db = scan_list_default_squelch_spin_->value();
  }
  scan_list_channels_.push_back(channel);
  SaveScanListConfigToSettingsGroup(ActiveScanListSettingsGroup(mode_tabs_));
  RefreshScanListChannelCards();
}

void MainWindow::AddRadarScanListChannel() {
  ScanListChannelConfig channel;
  channel.label = QString("Kanal %1").arg(radar_scan_list_channels_.size() + 1);
  channel.modulation = v1::MODULATION_NFM;
  channel.bandwidth_hz = DefaultBandwidthHzForModulation(channel.modulation);
  radar_scan_list_channels_.push_back(channel);
  SaveRadarScanListConfigToSettings();
  RefreshRadarScanListChannelCards();
}

void MainWindow::RemoveScanListChannel(int index) {
  if (index < 0 || static_cast<size_t>(index) >= scan_list_channels_.size()) {
    return;
  }
  scan_list_channels_.erase(scan_list_channels_.begin() + index);
  if (active_scan_list_channel_index_ == index) {
    active_scan_list_channel_index_ = -1;
    active_scan_list_channel_state_ = ScanListChannelState::kIdle;
  } else if (active_scan_list_channel_index_ > index) {
    --active_scan_list_channel_index_;
  }
  if (frozen_scan_channel_index_ == index) {
    frozen_scan_channel_index_ = -1;
  } else if (frozen_scan_channel_index_ > index) {
    --frozen_scan_channel_index_;
  }
  SaveScanListConfigToSettingsGroup(ActiveScanListSettingsGroup(mode_tabs_));
  RefreshScanListChannelCards();
  if (receiver_combo_->currentIndex() >= 0) {
    ApplyModeAndConfig();
  }
}

void MainWindow::RemoveRadarScanListChannel(int index) {
  if (index < 0 || static_cast<size_t>(index) >= radar_scan_list_channels_.size()) return;
  radar_scan_list_channels_.erase(radar_scan_list_channels_.begin() + index);
  if (radar_active_scan_list_channel_index_ == index) radar_active_scan_list_channel_index_ = -1;
  SaveRadarScanListConfigToSettings();
  RefreshRadarScanListChannelCards();
  if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig();
}

void MainWindow::ImportScanListCsv() {
  const QString path = QFileDialog::getOpenFileName(
      this, "Import scan-list CSV", QString(), "CSV files (*.csv);;All files (*)");
  if (path.isEmpty()) {
    return;
  }

  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    QMessageBox::warning(this, "CSV import", QString("Could not open %1").arg(path));
    return;
  }

  QTextStream stream(&file);
  const double default_squelch_db =
      (scan_list_default_squelch_spin_ != nullptr) ? scan_list_default_squelch_spin_->value()
                                                   : kDefaultScanListSquelchDb;
  std::vector<ScanListChannelConfig> imported;
  QStringList errors;
  int line_number = 0;
  while (!stream.atEnd()) {
    const QString raw_line = stream.readLine();
    ++line_number;
    const QString line = raw_line.trimmed();
    if (line.isEmpty() || line.startsWith('#')) {
      continue;
    }

    const QStringList columns = line.split(';', Qt::KeepEmptyParts);
    if (columns.size() < 3) {
      errors.push_back(QString("Line %1: expected format MHz;modulation;label").arg(line_number));
      continue;
    }

    bool frequency_ok = false;
    const double frequency_mhz = columns[0].trimmed().toDouble(&frequency_ok);
    if (!frequency_ok || frequency_mhz <= 0.0) {
      errors.push_back(QString("Line %1: invalid frequency '%2'").arg(line_number).arg(columns[0].trimmed()));
      continue;
    }

    v1::Modulation modulation = v1::MODULATION_NFM;
    if (!TryParseCsvModulation(columns[1], &modulation)) {
      errors.push_back(QString("Line %1: invalid modulation '%2'").arg(line_number).arg(columns[1].trimmed()));
      continue;
    }

    ScanListChannelConfig channel;
    channel.frequency_mhz = frequency_mhz;
    channel.modulation = modulation;
    channel.label = columns.mid(2).join(";").trimmed();
    if (channel.label.isEmpty()) {
      channel.label = QString("CSV %1").arg(imported.size() + 1);
    }
    channel.bandwidth_hz = DefaultBandwidthHzForModulation(channel.modulation);
    channel.squelch_threshold_db = default_squelch_db;
    channel.dwell_ms = 0;
    channel.use_default_squelch = true;
    imported.push_back(std::move(channel));
  }

  if (imported.empty()) {
    const QString details = errors.isEmpty() ? "No channels found in file."
                                             : errors.mid(0, 10).join("\n");
    QMessageBox::warning(this, "CSV import", details);
    return;
  }

  QMessageBox choice(this);
  choice.setWindowTitle("CSV import");
  choice.setText(QString("Imported %1 channels from CSV.").arg(imported.size()));
  choice.setInformativeText("Do you want to replace existing channels or append to them?");
  auto* replace_button = choice.addButton("Replace", QMessageBox::AcceptRole);
  auto* append_button = choice.addButton("Append", QMessageBox::ActionRole);
  choice.addButton(QMessageBox::Cancel);
  choice.exec();
  const QAbstractButton* clicked = choice.clickedButton();
  if (clicked == nullptr || clicked == choice.button(QMessageBox::Cancel)) {
    return;
  }

  if (clicked == replace_button) {
    scan_list_channels_ = std::move(imported);
  } else if (clicked == append_button) {
    scan_list_channels_.insert(scan_list_channels_.end(), imported.begin(), imported.end());
  } else {
    return;
  }

  SaveScanListConfigToSettingsGroup(ActiveScanListSettingsGroup(mode_tabs_));
  RefreshScanListChannelCards();
  if (receiver_combo_->currentIndex() >= 0) {
    ApplyModeAndConfig();
  }

  if (!errors.isEmpty()) {
    AppendLog(QString("CSV import warnings:\n%1").arg(errors.mid(0, 10).join("\n")));
  }
}

void MainWindow::ImportRadarScanListCsv() {
  const QString path = QFileDialog::getOpenFileName(
      this, "Import radar scan-list CSV", QString(), "CSV files (*.csv);;All files (*)");
  if (path.isEmpty()) return;
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    QMessageBox::warning(this, "CSV import", QString("Could not open %1").arg(path));
    return;
  }
  QTextStream stream(&file);
  std::vector<ScanListChannelConfig> imported;
  QStringList errors;
  int line_number = 0;
  while (!stream.atEnd()) {
    const QString raw_line = stream.readLine();
    ++line_number;
    const QString line = raw_line.trimmed();
    if (line.isEmpty() || line.startsWith('#')) continue;
    const QStringList columns = line.split(';', Qt::KeepEmptyParts);
    if (columns.size() < 3) {
      errors.push_back(QString("Line %1: expected format MHz;modulation;label").arg(line_number));
      continue;
    }
    bool frequency_ok = false;
    const double frequency_mhz = columns[0].trimmed().toDouble(&frequency_ok);
    if (!frequency_ok || frequency_mhz <= 0.0) {
      errors.push_back(QString("Line %1: invalid frequency '%2'").arg(line_number).arg(columns[0].trimmed()));
      continue;
    }
    v1::Modulation modulation = v1::MODULATION_NFM;
    if (!TryParseCsvModulation(columns[1], &modulation)) {
      errors.push_back(QString("Line %1: invalid modulation '%2'").arg(line_number).arg(columns[1].trimmed()));
      continue;
    }
    ScanListChannelConfig channel;
    channel.frequency_mhz = frequency_mhz;
    channel.modulation = modulation;
    channel.label = columns.mid(2).join(";").trimmed();
    channel.bandwidth_hz = DefaultBandwidthHzForModulation(channel.modulation);
    imported.push_back(std::move(channel));
  }
  if (imported.empty()) {
    const QString details = errors.isEmpty() ? "No channels found in file." : errors.mid(0, 10).join("\n");
    QMessageBox::warning(this, "CSV import", details);
    return;
  }
  radar_scan_list_channels_ = std::move(imported);
  SaveRadarScanListConfigToSettings();
  RefreshRadarScanListChannelCards();
  if (receiver_combo_->currentIndex() >= 0) ApplyModeAndConfig();
}

void MainWindow::ApplyScanListStatusEvent(uint32_t receiver_id, const QString& message) {
  bool signal_ok = false;
  const double signal_db = TokenValue(message, "signal_db").toDouble(&signal_ok);
  const QString state_early = TokenValue(message, "state").trimmed().toLower();
  if (state_early == "open") {
    if (signal_ok) signal_visualization_->SetReceiverSignalLevelDb(receiver_id, signal_db);
  } else {
    signal_visualization_->ClearReceiverSignalLevelDb(receiver_id);
  }

  bool threshold_ok = false;
  const double threshold_db = TokenValue(message, "threshold_db").toDouble(&threshold_ok);
  if (threshold_ok) {
    signal_visualization_->SetReceiverSquelchThresholdDb(receiver_id, threshold_db);
  }
  if (!IsSelectedReceiver(receiver_id)) {
    return;
  }
  bool idx_ok = false;
  const int index = TokenValue(message, "idx").toInt(&idx_ok);
  if (!idx_ok || index < 0 || static_cast<size_t>(index) >= scan_list_channels_.size()) {
    return;
  }
  bool monitor_ok = false;
  const int monitor = TokenValue(message, "monitor").toInt(&monitor_ok);
  if (auto_squelch_active_ && signal_ok && receiver_id == auto_squelch_receiver_id_ && monitor_ok &&
      monitor == 1) {
    if (auto_squelch_has_last_channel_ && index < auto_squelch_last_channel_index_) {
      ++auto_squelch_completed_loops_;
      AppendLog(QString("Auto squelch progress: loop %1/%2")
                    .arg(auto_squelch_completed_loops_)
                    .arg(auto_squelch_required_loops_));
    }
    auto_squelch_has_last_channel_ = true;
    auto_squelch_last_channel_index_ = index;
    auto_squelch_signal_sum_db_ += signal_db;
    ++auto_squelch_sample_count_;
    if (auto_squelch_completed_loops_ >= auto_squelch_required_loops_) {
      CompleteAutoSquelchCalibration();
    }
  }
  const QString state = TokenValue(message, "state").trimmed().toLower();
  if (state == "open") {
    constexpr double kBump = 0.125;
    if (static_cast<size_t>(index) >= scan_channel_heat_.size()) {
      scan_channel_heat_.resize(static_cast<size_t>(index) + 1);
    }
    auto& h = scan_channel_heat_[static_cast<size_t>(index)];
    h.value = std::min(1.0, h.value + kBump);
  }
  active_scan_list_channel_index_ = index;
  if (state == "open") {
    active_scan_list_channel_state_ = ScanListChannelState::kSquelchOpen;
  } else {
    active_scan_list_channel_state_ = ScanListChannelState::kSquelchClosed;
  }
  RefreshScanListChannelCards();

  // Update the waveform channel label so the user knows which channel's audio is playing.
  if (signal_visualization_ != nullptr) {
    const auto& ch = scan_list_channels_[static_cast<size_t>(index)];
    QString name = ch.label.trimmed();
    if (name.isEmpty()) {
      name = QString("Kanal %1").arg(index + 1);
    }
    Q_UNUSED(name);
  }
}

void MainWindow::StartAutoSquelchCalibration() {
  if (auto_squelch_active_) {
    auto_squelch_active_ = false;
    auto_squelch_restore_monitor_mode_ = false;
    AppendLog("Auto squelch canceled");
    return;
  }
  if (scan_list_channels_.empty()) {
    QMessageBox::information(this, "Auto squelch", "No scan-list channels configured.");
    return;
  }
  if (receiver_combo_->currentIndex() < 0) {
    QMessageBox::information(this, "Auto squelch", "Select a receiver first.");
    return;
  }

  auto_squelch_receiver_id_ = static_cast<uint32_t>(receiver_combo_->currentData().toUInt());
  auto_squelch_required_loops_ = 4;
  auto_squelch_completed_loops_ = 0;
  auto_squelch_last_channel_index_ = -1;
  auto_squelch_has_last_channel_ = false;
  auto_squelch_signal_sum_db_ = 0.0;
  auto_squelch_sample_count_ = 0;
  auto_squelch_active_ = true;

  if (mode_tabs_->currentIndex() != kScanListModeTabIndex) {
    mode_tabs_->setCurrentIndex(kScanListModeTabIndex);
  }
  auto_squelch_restore_monitor_mode_ =
      (scan_list_monitor_checkbox_ != nullptr && !scan_list_monitor_checkbox_->isChecked());
  if (scan_list_monitor_checkbox_ != nullptr && !scan_list_monitor_checkbox_->isChecked()) {
    scan_list_monitor_checkbox_->setChecked(true);
  }
  ApplyModeAndConfig();
  AppendLog(QString("Auto squelch started: sampling %1 loops in monitor mode")
                .arg(auto_squelch_required_loops_));
}

void MainWindow::CompleteAutoSquelchCalibration() {
  if (!auto_squelch_active_) {
    return;
  }
  auto_squelch_active_ = false;
  if (auto_squelch_sample_count_ <= 0 || scan_list_default_squelch_spin_ == nullptr) {
    auto_squelch_restore_monitor_mode_ = false;
    AppendLog("Auto squelch failed: no signal samples collected");
    return;
  }

  const double mean_db = auto_squelch_signal_sum_db_ / static_cast<double>(auto_squelch_sample_count_);
  const double calibrated_default_db = std::clamp(mean_db + 3.0, -120.0, 0.0);
  scan_list_default_squelch_spin_->setValue(calibrated_default_db);

  if (auto_squelch_restore_monitor_mode_ && scan_list_monitor_checkbox_ != nullptr) {
    scan_list_monitor_checkbox_->setChecked(false);
  }
  auto_squelch_restore_monitor_mode_ = false;
  ApplyModeAndConfig();

  AppendLog(QString("Auto squelch complete: mean=%1 dB, default set to %2 dB (%3 samples)")
                .arg(mean_db, 0, 'f', 1)
                .arg(calibrated_default_db, 0, 'f', 1)
                .arg(auto_squelch_sample_count_));
}

void MainWindow::HandleAudioPcmFrame(int sample_rate_hz, const QByteArray& pcm) {
#if MR_HAS_QT_MULTIMEDIA
  if (sample_rate_hz <= 1000 || sample_rate_hz > 192000 || pcm.isEmpty()) {
    return;
  }
  EnsureAudioOutputInitialized(sample_rate_hz);
  if (audio_output_disabled_ || audio_output_device_ == nullptr) {
    return;
  }
  QByteArray playback_pcm;
  int playback_sample_rate_hz = sample_rate_hz;
  if (audio_output_sample_rate_hz_ > 0 && audio_output_sample_rate_hz_ != sample_rate_hz) {
    if (!ResampleMonoPcmS16Le(pcm, sample_rate_hz, audio_output_sample_rate_hz_,
                              &audio_resample_next_source_pos_, &audio_resample_has_prev_sample_,
                              &audio_resample_prev_sample_, &playback_pcm)) {
      return;
    }
    playback_sample_rate_hz = audio_output_sample_rate_hz_;
  } else {
    audio_resample_next_source_pos_ = 0.0;
    audio_resample_has_prev_sample_ = false;
    audio_resample_prev_sample_ = 0;
    playback_pcm = pcm;
  }
  if (playback_pcm.isEmpty()) {
    return;
  }
  if (audio_pending_pcm_.isEmpty()) {
    audio_prefill_started_at_ms_ = QDateTime::currentMSecsSinceEpoch();
  }
  audio_pending_pcm_.append(playback_pcm);
  const int max_buffered_audio_bytes =
      std::max(4096, (playback_sample_rate_hz * kAudioBytesPerSample * kAudioPendingMaxMs) / 1000);
  if (audio_pending_pcm_.size() > max_buffered_audio_bytes) {
    int dropped = audio_pending_pcm_.size() - max_buffered_audio_bytes;
    dropped -= dropped % kAudioBytesPerSample;
    if (dropped > 0) {
      audio_pending_pcm_.remove(0, dropped);
      audio_frontend_overrun_dropped_bytes_ += static_cast<quint64>(dropped);
    }
    if (!audio_queue_overrun_logged_) {
      AppendLog(QString("Audio queue overrun: dropped %1 bytes").arg(dropped));
      audio_queue_overrun_logged_ = true;
    }
  } else {
    audio_queue_overrun_logged_ = false;
  }
  if (!audio_prefill_complete_) {
    const int prefill_bytes =
        std::max(4096, (playback_sample_rate_hz * kAudioBytesPerSample * kAudioPrefillMs) / 1000);
    const int min_start_bytes =
        std::max(1024, (playback_sample_rate_hz * kAudioBytesPerSample * kAudioMinStartupMs) / 1000);
    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    const bool prefill_timed_out = audio_prefill_started_at_ms_ > 0 &&
                                   now_ms >= audio_prefill_started_at_ms_ + kAudioPrefillMaxWaitMs;
    if (audio_pending_pcm_.size() < prefill_bytes &&
        (!prefill_timed_out || audio_pending_pcm_.size() < min_start_bytes)) {
      ++audio_frontend_prefill_wait_events_;
      MaybeEmitFrontendAudioStats();
      return;
    }
    audio_prefill_complete_ = true;
    ++audio_frontend_prefill_complete_events_;
  }
  DrainAudioOutputQueue();
  MaybeEmitFrontendAudioStats();
#else
  Q_UNUSED(sample_rate_hz);
  Q_UNUSED(pcm);
#endif
}

void MainWindow::HandleAudioPcmEvent(const QString& message) {
#if MR_HAS_QT_MULTIMEDIA
  bool sr_ok = false;
  const int sample_rate_hz = TokenValue(message, "sr").toInt(&sr_ok);
  if (!sr_ok || sample_rate_hz <= 1000 || sample_rate_hz > 192000) {
    return;
  }
  const QString data_b64 = TokenValue(message, "data");
  if (data_b64.isEmpty()) {
    return;
  }
  const QByteArray pcm = QByteArray::fromBase64(data_b64.toUtf8());
  if (pcm.isEmpty()) {
    return;
  }
  HandleAudioPcmFrame(sample_rate_hz, pcm);
#else
  Q_UNUSED(message);
#endif
}

void MainWindow::EnsureAudioOutputInitialized(int sample_rate_hz) {
#if MR_HAS_QT_MULTIMEDIA
  if (audio_output_disabled_ && audio_output_disabled_by_env_) {
    return;
  }
  // Recover from transient init failures; only env flag should hard-disable audio.
  audio_output_disabled_ = false;
  const QAudioDevice default_output = QMediaDevices::defaultAudioOutput();
  if (default_output.isNull()) {
    AppendLog("Audio output unavailable: no default output device (will retry)");
    return;
  }

  QAudioFormat audio_format;
  audio_format.setChannelCount(1);
  audio_format.setSampleFormat(QAudioFormat::Int16);

  int selected_rate_hz = 0;
  const QAudioFormat preferred = default_output.preferredFormat();
  if (preferred.sampleRate() > 0) {
    QAudioFormat preferred_mono_s16 = preferred;
    preferred_mono_s16.setChannelCount(1);
    preferred_mono_s16.setSampleFormat(QAudioFormat::Int16);
    if (default_output.isFormatSupported(preferred_mono_s16)) {
      selected_rate_hz = preferred_mono_s16.sampleRate();
    }
  }
  if (selected_rate_hz <= 0) {
    const int candidates[] = {48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, sample_rate_hz};
    int selected_distance = std::numeric_limits<int>::max();
    for (const int candidate_rate_hz : candidates) {
      if (candidate_rate_hz <= 0) {
        continue;
      }
      QAudioFormat candidate;
      candidate.setChannelCount(1);
      candidate.setSampleFormat(QAudioFormat::Int16);
      candidate.setSampleRate(candidate_rate_hz);
      if (!default_output.isFormatSupported(candidate)) {
        continue;
      }
      const int distance = std::abs(candidate_rate_hz - sample_rate_hz);
      if (distance < selected_distance || selected_rate_hz <= 0) {
        selected_distance = distance;
        selected_rate_hz = candidate_rate_hz;
      }
    }
  }
  if (selected_rate_hz <= 0) {
    AppendLog(QString("Audio output unavailable: no supported mono s16le sample rate (requested %1 Hz, preferred %2 Hz)")
                  .arg(sample_rate_hz)
                  .arg(preferred.sampleRate()));
    return;
  }
  audio_format.setSampleRate(selected_rate_hz);
  if (audio_sink_ != nullptr && audio_output_sample_rate_hz_ != selected_rate_hz) {
    audio_sink_->stop();
    audio_sink_->deleteLater();
    audio_sink_ = nullptr;
    audio_output_device_ = nullptr;
    audio_pending_pcm_.clear();
    audio_resample_next_source_pos_ = 0.0;
    audio_resample_has_prev_sample_ = false;
    audio_resample_prev_sample_ = 0;
    audio_prefill_complete_ = false;
    audio_prefill_started_at_ms_ = 0;
    audio_output_sample_rate_hz_ = 0;
  }
  if (audio_output_device_ != nullptr && audio_sink_ != nullptr &&
      audio_output_sample_rate_hz_ == selected_rate_hz) {
    return;
  }
  if (selected_rate_hz != sample_rate_hz) {
    AppendLog(QString("Audio output using %1 Hz (input %2 Hz; client resampling enabled)")
                  .arg(selected_rate_hz)
                  .arg(sample_rate_hz));
  }
  audio_sink_ = new QAudioSink(default_output, audio_format, this);
  audio_sink_->setBufferSize(
      std::max(4096, (audio_format.sampleRate() * kAudioBytesPerSample * kAudioSinkBufferMs) / 1000));
  audio_output_device_ = audio_sink_->start();
  if (audio_output_device_ == nullptr) {
    audio_sink_->deleteLater();
    audio_sink_ = nullptr;
    audio_pending_pcm_.clear();
    audio_resample_next_source_pos_ = 0.0;
    audio_resample_has_prev_sample_ = false;
    audio_resample_prev_sample_ = 0;
    audio_prefill_complete_ = false;
    audio_prefill_started_at_ms_ = 0;
    audio_output_sample_rate_hz_ = 0;
    AppendLog(QString("Audio output unavailable: sink start failed for %1 Hz (will retry)")
                  .arg(audio_format.sampleRate()));
    return;
  }
  audio_prefill_complete_ = false;
  audio_prefill_started_at_ms_ = 0;
  audio_resample_next_source_pos_ = 0.0;
  audio_resample_has_prev_sample_ = false;
  audio_resample_prev_sample_ = 0;
  audio_output_sample_rate_hz_ = audio_format.sampleRate();
#endif
}

void MainWindow::DrainAudioOutputQueue() {
#if MR_HAS_QT_MULTIMEDIA
  if (audio_output_disabled_ || audio_output_device_ == nullptr || !audio_prefill_complete_ ||
      audio_pending_pcm_.isEmpty()) {
    MaybeEmitFrontendAudioStats();
    return;
  }
  while (!audio_pending_pcm_.isEmpty()) {
    qint64 bytes_to_write = static_cast<qint64>(audio_pending_pcm_.size());
    if (audio_sink_ != nullptr) {
      const qint64 bytes_free = audio_sink_->bytesFree();
      if (bytes_free <= 0) {
        ++audio_frontend_write_blocked_events_;
        MaybeEmitFrontendAudioStats();
        return;
      }
      bytes_to_write = std::min(bytes_to_write, bytes_free);
    }
    bytes_to_write -= (bytes_to_write % kAudioBytesPerSample);
    if (bytes_to_write <= 0) {
      ++audio_frontend_write_blocked_events_;
      MaybeEmitFrontendAudioStats();
      return;
    }
    const qint64 written = audio_output_device_->write(audio_pending_pcm_.constData(), bytes_to_write);
    if (written <= 0) {
      ++audio_frontend_write_blocked_events_;
      MaybeEmitFrontendAudioStats();
      return;
    }
    const qint64 aligned_written = written - (written % kAudioBytesPerSample);
    if (aligned_written <= 0) {
      ++audio_frontend_write_blocked_events_;
      MaybeEmitFrontendAudioStats();
      return;
    }
    ++audio_frontend_write_calls_;
    audio_frontend_written_bytes_ += static_cast<quint64>(aligned_written);
    audio_pending_pcm_.remove(0, static_cast<int>(aligned_written));
  }
  MaybeEmitFrontendAudioStats();
#endif
}

void MainWindow::MaybeEmitFrontendAudioStats() {
  const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
  if (audio_frontend_stats_window_started_at_ms_ <= 0) {
    audio_frontend_stats_window_started_at_ms_ = now_ms;
  }
  if (now_ms < audio_frontend_stats_window_started_at_ms_ + kAudioFrontendStatsIntervalMs) {
    return;
  }

  const qint64 pending_bytes = static_cast<qint64>(audio_pending_pcm_.size());
  const int backend_seen_recent =
      (audio_backend_stats_last_seen_at_ms_ > 0 &&
       now_ms <= audio_backend_stats_last_seen_at_ms_ + (kAudioFrontendStatsIntervalMs * 2))
          ? 1
          : 0;
  const bool has_audio_activity = audio_frontend_rx_frames_ > 0 || audio_frontend_rx_bytes_ > 0 ||
                                  audio_frontend_filtered_frames_ > 0 ||
                                  audio_frontend_prefill_wait_events_ > 0 ||
                                  audio_frontend_prefill_complete_events_ > 0 ||
                                  pending_bytes > 0 || audio_frontend_write_calls_ > 0 ||
                                  audio_frontend_written_bytes_ > 0 ||
                                  audio_frontend_write_blocked_events_ > 0 ||
                                  audio_frontend_overrun_dropped_bytes_ > 0 ||
                                  audio_frontend_gap_fill_bytes_ > 0 ||
                                  audio_frontend_missing_frame_events_ > 0 ||
                                  audio_frontend_missing_sample_bytes_ > 0;
  if (backend_seen_recent == 0 && !has_audio_activity) {
    audio_frontend_stats_window_started_at_ms_ = now_ms;
    return;
  }

  audio_frontend_rx_frames_ = 0;
  audio_frontend_rx_bytes_ = 0;
  audio_frontend_filtered_frames_ = 0;
  audio_frontend_prefill_wait_events_ = 0;
  audio_frontend_prefill_complete_events_ = 0;
  audio_frontend_write_calls_ = 0;
  audio_frontend_written_bytes_ = 0;
  audio_frontend_write_blocked_events_ = 0;
  audio_frontend_overrun_dropped_bytes_ = 0;
  audio_frontend_gap_fill_bytes_ = 0;
  audio_frontend_missing_frame_events_ = 0;
  audio_frontend_missing_sample_bytes_ = 0;
  audio_frontend_stats_window_started_at_ms_ = now_ms;
}

void MainWindow::LoadScanListConfigFromSettingsGroup(const QString& group) {
  QSettings settings("multi-radio", "multi-radio-client");
  settings.beginGroup(group);
  const int saved_default_dwell =
      settings.value("default_dwell_ms", dwell_ms_spin_->value()).toInt();
  if (saved_default_dwell > 0) {
    dwell_ms_spin_->setValue(saved_default_dwell);
  }
  const double default_squelch_db = std::clamp(
      settings.value("default_squelch_db", kDefaultScanListSquelchDb).toDouble(), -120.0, 0.0);
  if (scan_list_default_squelch_spin_ != nullptr) {
    const QSignalBlocker blocker(scan_list_default_squelch_spin_);
    scan_list_default_squelch_spin_->setValue(default_squelch_db);
  }
  if (scan_list_monitor_checkbox_ != nullptr) {
    const QSignalBlocker blocker(scan_list_monitor_checkbox_);
    scan_list_monitor_checkbox_->setChecked(settings.value("monitor_mode", false).toBool());
  }

  int channel_count = settings.value("count", -1).toInt();
  if (channel_count < 0) {
    int max_index = -1;
    const QStringList groups = settings.childGroups();
    for (const QString& group : groups) {
      if (!group.startsWith("channel_")) {
        continue;
      }
      bool ok = false;
      const int idx = group.mid(8).toInt(&ok);
      if (ok && idx > max_index) {
        max_index = idx;
      }
    }
    channel_count = max_index + 1;
  }
  if (channel_count <= 0) {
    // Seed defaults for the radar view so Start immediately provides AIS updates.
    if (group == "radar_scan_list") {
      channel_count = 3;
    } else {
      channel_count = 5;
    }
  }

  scan_list_channels_.clear();
  scan_list_channels_.reserve(static_cast<size_t>(channel_count));
  bool migrated_legacy_squelch_format = false;
  for (int index = 0; index < channel_count; ++index) {
    ScanListChannelConfig channel;
    channel.squelch_threshold_db = default_squelch_db;
    settings.beginGroup(QString("channel_%1").arg(index));
    channel.label = settings.value("label", channel.label).toString();
    channel.frequency_mhz =
        settings.value("frequency_mhz", channel.frequency_mhz).toDouble();
    channel.bandwidth_hz = settings.value("bandwidth_hz", channel.bandwidth_hz).toInt();
    const bool has_use_default_flag = settings.contains("use_default_squelch");
    const bool has_saved_squelch = settings.contains("squelch_threshold_db");
    if (has_use_default_flag) {
      channel.use_default_squelch = settings.value("use_default_squelch").toBool();
    } else if (!has_saved_squelch) {
      channel.use_default_squelch = true;
      migrated_legacy_squelch_format = true;
    } else {
      const double saved_squelch =
          settings.value("squelch_threshold_db", channel.squelch_threshold_db).toDouble();
      channel.use_default_squelch = std::abs(saved_squelch - default_squelch_db) < 0.05;
      if (channel.use_default_squelch) {
        migrated_legacy_squelch_format = true;
      }
    }
    if (has_saved_squelch) {
      channel.squelch_threshold_db =
          settings.value("squelch_threshold_db", channel.squelch_threshold_db).toDouble();
    }
    if (channel.use_default_squelch) {
      channel.squelch_threshold_db = default_squelch_db;
    }
    channel.dwell_ms = settings.value("dwell_ms", channel.dwell_ms).toInt();
    channel.audio_gain_db = settings.value("audio_gain_db", 0.0).toDouble();

    const int modulation = settings.value("modulation", static_cast<int>(channel.modulation)).toInt();
    switch (modulation) {
      case v1::MODULATION_AIS_DUAL:
        channel.modulation = v1::MODULATION_AIS_DUAL;
        break;
      case v1::MODULATION_VDES_ASM:
        channel.modulation = v1::MODULATION_VDES_ASM;
        break;
      case v1::MODULATION_ADSB:
        channel.modulation = v1::MODULATION_ADSB;
        break;
      case v1::MODULATION_PPM:
        channel.modulation = v1::MODULATION_PPM;
        break;
      case v1::MODULATION_GMSK:
        channel.modulation = v1::MODULATION_GMSK;
        break;
      case v1::MODULATION_FSK:
        channel.modulation = v1::MODULATION_FSK;
        break;
      case v1::MODULATION_AM:
        channel.modulation = v1::MODULATION_AM;
        break;
      case v1::MODULATION_WFM:
        channel.modulation = v1::MODULATION_WFM;
        break;
      case v1::MODULATION_NFM:
      default:
        channel.modulation = v1::MODULATION_NFM;
        break;
    }
    scan_list_channels_.push_back(std::move(channel));
    settings.endGroup();
  }
  settings.endGroup();

  // If radar_scan_list is empty/missing (first run), seed sensible defaults.
  if (group == "radar_scan_list") {
    bool any_freq = false;
    for (const auto& ch : scan_list_channels_) {
      if (ch.frequency_mhz > 0.0) { any_freq = true; break; }
    }
    if (!any_freq) {
      scan_list_channels_.clear();
      scan_list_channels_.reserve(3);

      ScanListChannelConfig ais;
      ais.label = "AIS Dual";
      ais.frequency_mhz = 162.000;
      ais.modulation = v1::MODULATION_AIS_DUAL;
      ais.bandwidth_hz = DefaultBandwidthHzForModulation(ais.modulation);
      ais.use_default_squelch = true;
      ais.squelch_threshold_db = default_squelch_db;
      scan_list_channels_.push_back(ais);

      ScanListChannelConfig dsc;
      dsc.label = "DSC Ch 70";
      dsc.frequency_mhz = 156.525;
      dsc.modulation = v1::MODULATION_FSK;
      dsc.bandwidth_hz = DefaultBandwidthHzForModulation(dsc.modulation);
      dsc.use_default_squelch = true;
      dsc.squelch_threshold_db = default_squelch_db;
      scan_list_channels_.push_back(dsc);

      ScanListChannelConfig adsb;
      adsb.label = "ADS-B";
      adsb.frequency_mhz = 1090.000;
      adsb.modulation = v1::MODULATION_ADSB;
      adsb.bandwidth_hz = DefaultBandwidthHzForModulation(adsb.modulation);
      adsb.use_default_squelch = true;
      adsb.squelch_threshold_db = default_squelch_db;
      scan_list_channels_.push_back(adsb);

      SaveScanListConfigToSettingsGroup(group);
      AppendLog("Seeded radar scan-list defaults (AIS Dual, DSC, ADS-B)");
    }
  }

  if (migrated_legacy_squelch_format) {
    SaveScanListConfigToSettingsGroup(group);
    AppendLog("Migrated legacy scan-list squelch settings to default-aware format");
  }
}

void MainWindow::SaveScanListConfigToSettingsGroup(const QString& group) const {
  QSettings settings("multi-radio", "multi-radio-client");
  settings.beginGroup(group);
  settings.remove("");
  settings.setValue("default_dwell_ms", dwell_ms_spin_->value());
  settings.setValue("default_squelch_db", scan_list_default_squelch_spin_ != nullptr
                                             ? scan_list_default_squelch_spin_->value()
                                             : kDefaultScanListSquelchDb);
  settings.setValue("monitor_mode",
                    scan_list_monitor_checkbox_ != nullptr && scan_list_monitor_checkbox_->isChecked());
  settings.setValue("count", static_cast<int>(scan_list_channels_.size()));
  for (size_t index = 0; index < scan_list_channels_.size(); ++index) {
    const ScanListChannelConfig& channel = scan_list_channels_[index];
    settings.beginGroup(QString("channel_%1").arg(static_cast<qulonglong>(index)));
    settings.setValue("label", channel.label);
    settings.setValue("frequency_mhz", channel.frequency_mhz);
    settings.setValue("modulation", static_cast<int>(channel.modulation));
    settings.setValue("bandwidth_hz", channel.bandwidth_hz);
    settings.setValue("use_default_squelch", channel.use_default_squelch);
    if (channel.use_default_squelch) {
      settings.remove("squelch_threshold_db");
    } else {
      settings.setValue("squelch_threshold_db", channel.squelch_threshold_db);
    }
    settings.setValue("dwell_ms", channel.dwell_ms);
    settings.setValue("audio_gain_db", channel.audio_gain_db);
    settings.endGroup();
  }
  settings.endGroup();
}

void MainWindow::LoadScanListConfigFromSettings() {
  LoadScanListConfigFromSettingsGroup("scan_list");
}

void MainWindow::SaveScanListConfigToSettings() const {
  SaveScanListConfigToSettingsGroup("scan_list");
}

void MainWindow::LoadRadarScanListConfigFromSettings() {
  QSettings settings("multi-radio", "multi-radio-client");
  settings.beginGroup("radar_scan_list");
  int channel_count = settings.value("count", 0).toInt();
  if (channel_count <= 0) channel_count = 3;

  radar_scan_list_channels_.clear();
  radar_scan_list_channels_.reserve(static_cast<size_t>(channel_count));
  for (int index = 0; index < channel_count; ++index) {
    ScanListChannelConfig channel;
    settings.beginGroup(QString("channel_%1").arg(index));
    channel.label = settings.value("label", channel.label).toString();
    channel.frequency_mhz = settings.value("frequency_mhz", 0.0).toDouble();
    channel.bandwidth_hz = settings.value("bandwidth_hz", 0).toInt();
    channel.dwell_ms = settings.value("dwell_ms", 0).toInt();
    channel.use_default_squelch = settings.value("use_default_squelch", true).toBool();
    channel.squelch_threshold_db = settings.value("squelch_threshold_db", kDefaultScanListSquelchDb).toDouble();
    channel.audio_gain_db = settings.value("audio_gain_db", 0.0).toDouble();
    const int modulation = settings.value("modulation", static_cast<int>(channel.modulation)).toInt();
    channel.modulation = static_cast<v1::Modulation>(modulation);
    settings.endGroup();
    radar_scan_list_channels_.push_back(std::move(channel));
  }
  settings.endGroup();

  // Seed defaults if empty/unconfigured.
  bool any_freq = false;
  for (const auto& ch : radar_scan_list_channels_) {
    if (ch.frequency_mhz > 0.0) { any_freq = true; break; }
  }
  if (!any_freq) {
    radar_scan_list_channels_.clear();
    radar_scan_list_channels_.reserve(3);
    ScanListChannelConfig ais;
    ais.label = "AIS Dual";
    ais.frequency_mhz = 162.000;
    ais.modulation = v1::MODULATION_AIS_DUAL;
    ais.bandwidth_hz = DefaultBandwidthHzForModulation(ais.modulation);
    radar_scan_list_channels_.push_back(ais);
    ScanListChannelConfig dsc;
    dsc.label = "DSC Ch 70";
    dsc.frequency_mhz = 156.525;
    dsc.modulation = v1::MODULATION_FSK;
    dsc.bandwidth_hz = DefaultBandwidthHzForModulation(dsc.modulation);
    radar_scan_list_channels_.push_back(dsc);
    ScanListChannelConfig adsb;
    adsb.label = "ADS-B";
    adsb.frequency_mhz = 1090.000;
    adsb.modulation = v1::MODULATION_ADSB;
    adsb.bandwidth_hz = DefaultBandwidthHzForModulation(adsb.modulation);
    radar_scan_list_channels_.push_back(adsb);
    SaveRadarScanListConfigToSettings();
  }
}

void MainWindow::SaveRadarScanListConfigToSettings() const {
  QSettings settings("multi-radio", "multi-radio-client");
  settings.beginGroup("radar_scan_list");
  settings.remove("");
  settings.setValue("count", static_cast<int>(radar_scan_list_channels_.size()));
  for (size_t index = 0; index < radar_scan_list_channels_.size(); ++index) {
    const ScanListChannelConfig& channel = radar_scan_list_channels_[index];
    settings.beginGroup(QString("channel_%1").arg(static_cast<qulonglong>(index)));
    settings.setValue("label", channel.label);
    settings.setValue("frequency_mhz", channel.frequency_mhz);
    settings.setValue("modulation", static_cast<int>(channel.modulation));
    settings.setValue("bandwidth_hz", channel.bandwidth_hz);
    settings.setValue("dwell_ms", channel.dwell_ms);
    settings.setValue("use_default_squelch", channel.use_default_squelch);
    settings.setValue("squelch_threshold_db", channel.squelch_threshold_db);
    settings.setValue("audio_gain_db", channel.audio_gain_db);
    settings.endGroup();
  }
  settings.endGroup();
}

void MainWindow::OnStreamError(const QString& error) {
  AppendLog(QString("Stream error: %1").arg(error));
  if (error.contains("IQ stream unavailable", Qt::CaseInsensitive)) {
    if (spectrum_source_combo_ != nullptr && spectrum_source_combo_->currentData().toInt() == 1) {
      const QSignalBlocker blocker(spectrum_source_combo_);
      spectrum_source_combo_->setCurrentIndex(0);
      signal_visualization_->SetSpectrumSource(SignalVisualizationWidget::SpectrumSource::kDemodulated);
      AppendLog("Switched visualization source to Demodulated because IQ stream is unavailable.");
    }
    if (!iq_stream_unavailable_notified_) {
      iq_stream_unavailable_notified_ = true;
      QMessageBox::information(
          this, "IQ stream unavailable",
          "Servern exponerar inte IQ-stream.\n"
          "Visualisering fallbackar till demodulerad signal.\n"
          "Starta om servern byggd med senaste kod för rå IQ-visualisering.");
    }
  }
}

bool MainWindow::CurrentReceiverId(uint32_t* receiver_id) const {
  if (receiver_combo_->currentIndex() < 0) {
    QMessageBox::warning(const_cast<MainWindow*>(this), "No receiver", "Select a receiver first");
    return false;
  }
  if (receiver_id != nullptr) {
    *receiver_id = static_cast<uint32_t>(receiver_combo_->currentData().toInt());
  }
  return true;
}

void MainWindow::AppendLog(const QString& line) {
  event_log_->appendPlainText(line);
}

void MainWindow::OpenVisualizationSettingsDialog() {
  QDialog dialog(this);
  dialog.setWindowTitle("Visualization settings");

  auto* layout = new QFormLayout(&dialog);
  auto* fft_combo = new QComboBox(&dialog);
  const int fft_sizes[] = {64, 128, 256, 512, 1024, 2048, 4096};
  for (const int fft_size : fft_sizes) {
    fft_combo->addItem(QString::number(fft_size), QVariant::fromValue(fft_size));
  }
  const int current_fft = signal_visualization_->FftSize();
  const int fft_index = fft_combo->findData(QVariant::fromValue(current_fft));
  if (fft_index >= 0) {
    fft_combo->setCurrentIndex(fft_index);
  }

  auto* start_hz_spin = new QDoubleSpinBox(&dialog);
  start_hz_spin->setDecimals(0);
  start_hz_spin->setRange(0.0, 6000000000.0);
  start_hz_spin->setSingleStep(25000.0);
  start_hz_spin->setSuffix(" Hz");
  start_hz_spin->setValue(signal_visualization_->FrequencyStartHz());

  auto* end_hz_spin = new QDoubleSpinBox(&dialog);
  end_hz_spin->setDecimals(0);
  end_hz_spin->setRange(0.0, 6000000000.0);
  end_hz_spin->setSingleStep(25000.0);
  end_hz_spin->setSuffix(" Hz");
  end_hz_spin->setValue(signal_visualization_->FrequencyEndHz());

  auto* auto_noise_checkbox = new QCheckBox("Waterfall: hide bins at/below mean", &dialog);
  auto_noise_checkbox->setChecked(signal_visualization_->AutoNoiseReductionEnabled());
  auto* noise_floor_checkbox = new QCheckBox("Filter bins below noise floor", &dialog);
  noise_floor_checkbox->setChecked(signal_visualization_->NoiseFloorFilterEnabled());
  auto* noise_floor_spin = new QDoubleSpinBox(&dialog);
  noise_floor_spin->setDecimals(1);
  noise_floor_spin->setRange(-120.0, 0.0);
  noise_floor_spin->setSingleStep(1.0);
  noise_floor_spin->setSuffix(" dBFS");
  noise_floor_spin->setValue(signal_visualization_->NoiseFloorDb());
  noise_floor_spin->setEnabled(noise_floor_checkbox->isChecked());
  auto* iq_dc_suppress_checkbox = new QCheckBox("Receiver IQ: suppress DC in spectrum", &dialog);
  iq_dc_suppress_checkbox->setChecked(iq_visual_dc_suppression_enabled_);

  layout->addRow("FFT size", fft_combo);
  layout->addRow("Frequency start", start_hz_spin);
  layout->addRow("Frequency end", end_hz_spin);
  layout->addRow("Auto noise reduction", auto_noise_checkbox);
  layout->addRow("Noise floor filter", noise_floor_checkbox);
  layout->addRow("Noise floor level", noise_floor_spin);
  layout->addRow("IQ DC suppression", iq_dc_suppress_checkbox);
  connect(noise_floor_checkbox, &QCheckBox::toggled, noise_floor_spin, &QWidget::setEnabled);

  auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  layout->addRow(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

  if (dialog.exec() != QDialog::Accepted) {
    return;
  }

  const int fft_size = fft_combo->currentData().toInt();
  const double start_hz = start_hz_spin->value();
  const double end_hz = end_hz_spin->value();
  const bool auto_noise_reduction = auto_noise_checkbox->isChecked();
  const bool noise_floor_filter_enabled = noise_floor_checkbox->isChecked();
  const double noise_floor_db = noise_floor_spin->value();
  const bool iq_dc_suppression = iq_dc_suppress_checkbox->isChecked();
  iq_visual_dc_suppression_enabled_ = iq_dc_suppression;
  {
    QSettings settings("multi-radio", "multi-radio-client");
    settings.beginGroup("visualization");
    settings.setValue("auto_noise_reduction", auto_noise_reduction);
    settings.setValue("noise_floor_filter_enabled", noise_floor_filter_enabled);
    settings.setValue("noise_floor_db", noise_floor_db);
    settings.setValue("iq_dc_suppression", iq_visual_dc_suppression_enabled_);
    settings.endGroup();
  }
  signal_visualization_->SetVisualizationSettings(fft_size, start_hz, end_hz);
  signal_visualization_->SetAutoNoiseReductionEnabled(auto_noise_reduction);
  signal_visualization_->SetNoiseFloorFilterEnabled(noise_floor_filter_enabled);
  signal_visualization_->SetNoiseFloorDb(noise_floor_db);
  AppendLog(QString("Updated visualization settings: FFT=%1, range=%2-%3 Hz, waterfall-noise-filter=%4, noise-floor=%5 (%6 dBFS), iq-dc-suppress=%7")
                .arg(fft_size)
                .arg(start_hz, 0, 'f', 0)
                .arg(end_hz, 0, 'f', 0)
                .arg(auto_noise_reduction ? "on" : "off")
                .arg(noise_floor_filter_enabled ? "on" : "off")
                .arg(noise_floor_db, 0, 'f', 1)
                .arg(iq_dc_suppression ? "on" : "off"));
}

void MainWindow::AddMessageRow(const MessageRow& row) {
  if (decoded_table_ == nullptr) return;
  if (!PassesFilter(row)) {
    return;
  }

  const int current = decoded_table_->rowCount();
  decoded_table_->insertRow(current);
  decoded_table_->setItem(current, 0, new QTableWidgetItem(row.timestamp.toString("HH:mm:ss")));
  decoded_table_->setItem(current, 1, new QTableWidgetItem(row.mmsi));
  decoded_table_->setItem(current, 2, new QTableWidgetItem(row.lat));
  decoded_table_->setItem(current, 3, new QTableWidgetItem(row.lon));
  decoded_table_->setItem(current, 4, new QTableWidgetItem(row.sog));
  decoded_table_->setItem(current, 5, new QTableWidgetItem(row.cog));
  decoded_table_->setItem(current, 6, new QTableWidgetItem(row.other));
}

bool MainWindow::PassesFilter(const MessageRow& row) const {
  const QString signal_filter = signal_filter_combo_->currentText();
  if (signal_filter != "ALL" && row.signal_type != signal_filter) {
    return false;
  }

  if (receiver_filter_combo_ != nullptr && receiver_filter_combo_->currentIndex() >= 0) {
    const int receiver_filter = receiver_filter_combo_->currentData().toInt();
    if (receiver_filter >= 0 && static_cast<int>(row.receiver_id) != receiver_filter) {
      return false;
    }
  }

  const int minutes = minutes_filter_spin_->value();
  const QDateTime cutoff = QDateTime::currentDateTime().addSecs(-minutes * 60);
  return row.timestamp >= cutoff;
}

}  // namespace multi_radio
