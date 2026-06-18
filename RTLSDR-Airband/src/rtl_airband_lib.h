/*
 * rtl_airband_lib.h
 * Embeddable runtime API for RTLSDR-Airband
 */

#ifndef _RTL_AIRBAND_LIB_H
#define _RTL_AIRBAND_LIB_H 1

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace rtl_airband {

enum class Modulation {
    Am,
    Nfm,
    Iq,
};

struct FrequencyConfig {
    int frequency = 0;
    std::string label;
    Modulation modulation = Modulation::Am;
    int squelch_threshold_dbfs = 0;
    float squelch_snr_threshold = -1.0f;
    float notch_freq = 0.0f;
    float notch_q = 10.0f;
    float ctcss_freq = 0.0f;
    int bandwidth = 0;
    float ampfactor = 1.0f;
};

struct ChannelConfig {
    int highpass = 100;
    int lowpass = 2500;
    uint8_t afc = 0;
    int tau = 0;
    bool emit_audio = true;
    bool emit_iq = false;
    std::vector<FrequencyConfig> frequencies;
};

struct RtlSdrDeviceConfig {
    int index = -1;
    std::string serial;
    float gain_db = 0.0f;
    int correction = 0;
    int buffers = 10;
};

struct DeviceConfig {
    int sample_rate = 2560000;
    int center_frequency = 0;
    bool scan_mode = false;
    int tau = 0;
    RtlSdrDeviceConfig rtlsdr;
    std::vector<ChannelConfig> channels;
};

struct RuntimeOptions {
    bool use_localtime = false;
    bool multiple_demod_threads = false;
    bool multiple_output_threads = false;
    bool log_to_stderr = true;
    bool enable_tui = false;
    bool use_quadrature_demod = false;
    int shout_metadata_delay = 3;
    size_t fft_size_log = 9;
};

struct AudioBatch {
    int device_index = -1;
    int channel_index = -1;
    int frequency = 0;
    bool signal_active = false;
    bool stereo = false;
    const float* left = nullptr;
    const float* right = nullptr;
    size_t sample_count = 0;
};

struct IqBatch {
    int device_index = -1;
    int channel_index = -1;
    int frequency = 0;
    bool signal_active = false;
    const float* interleaved_iq = nullptr;
    size_t complex_sample_count = 0;
};

struct RawIqBatch {
    int device_index = -1;
    int center_frequency = 0;
    int sample_rate = 0;
    const int16_t* interleaved_iq_s16 = nullptr;
    size_t complex_sample_count = 0;
};

struct ScanState {
    int device_index = -1;
    int channel_index = -1;
    int frequency_index = -1;
    int frequency = 0;
    std::string label;
};

struct Callbacks {
    std::function<void(const AudioBatch&)> on_audio;
    std::function<void(const IqBatch&)> on_iq;
    std::function<void(const RawIqBatch&)> on_raw_iq;
    std::function<void(const ScanState&)> on_scan;
};

class Session {
   public:
    Session();
    ~Session();

    bool Configure(const std::vector<DeviceConfig>& devices, const RuntimeOptions& options, const Callbacks& callbacks, std::string* error);
    bool Start(std::string* error);
    bool Stop(std::string* error);

    bool UpdateDeviceCorrection(int device_index, int correction, std::string* error);
    bool SetScanFrequencyIndex(int device_index, int frequency_index, std::string* error);
    bool UpdateScanFrequency(int device_index, int frequency_index, const FrequencyConfig& config, std::string* error);
    bool ReplaceScanFrequencies(int device_index, const std::vector<FrequencyConfig>& frequencies, std::string* error);

    bool Running() const;

   private:
    bool configured_ = false;
};

}  // namespace rtl_airband

#endif /* _RTL_AIRBAND_LIB_H */
