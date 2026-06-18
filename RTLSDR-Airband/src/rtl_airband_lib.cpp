#include "rtl_airband_lib.h"

#include <algorithm>
#include <pthread.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "input-rtlsdr.h"
#include "rtl_airband.h"

namespace {

struct EmbeddedRuntimeState {
    rtl_airband::Callbacks callbacks;
    std::vector<rtl_airband::DeviceConfig> devices;
    std::vector<freq_t*> retired_scan_frequency_lists;
    bool configured = false;
};

EmbeddedRuntimeState g_embedded_state;

freq_t* MakeFrequencyList(int n) {
    if (n <= 0) {
        return nullptr;
    }
    auto* freqlist = static_cast<freq_t*>(XCALLOC(static_cast<size_t>(n), sizeof(freq_t)));
    for (int i = 0; i < n; ++i) {
        freqlist[i].label = nullptr;
        freqlist[i].ampfactor = 1.0f;
        freqlist[i].active_counter = 0;
        freqlist[i].modulation = MOD_AM;
    }
    return freqlist;
}

modulations ToAirbandModulation(rtl_airband::Modulation modulation) {
    switch (modulation) {
        case rtl_airband::Modulation::Iq:
            return MOD_RAWIQ;
        case rtl_airband::Modulation::Nfm:
#ifdef NFM
            return MOD_NFM;
#else
            return MOD_AM;
#endif
        case rtl_airband::Modulation::Am:
        default:
            return MOD_AM;
    }
}

bool ConfigureRtlSdrInput(const rtl_airband::DeviceConfig& config, input_t* input, std::string* error) {
    if (input == nullptr || input->dev_data == nullptr) {
        if (error) *error = "invalid RTLSDR input";
        return false;
    }
    auto* dev_data = static_cast<rtlsdr_dev_data_t*>(input->dev_data);
    dev_data->index = config.rtlsdr.index;
    if (!config.rtlsdr.serial.empty()) {
        dev_data->serial = strdup(config.rtlsdr.serial.c_str());
    }
    dev_data->gain = static_cast<int>(std::lround(config.rtlsdr.gain_db * 10.0f));
    dev_data->correction = config.rtlsdr.correction;
    dev_data->bufcnt = config.rtlsdr.buffers > 0 ? config.rtlsdr.buffers : RTLSDR_DEFAULT_LIBUSB_BUFFER_COUNT;
    input->sample_rate = config.sample_rate;
    input->centerfreq = config.center_frequency;
    if (dev_data->index < 0 && dev_data->serial == nullptr) {
        if (error) *error = "RTLSDR device requires index or serial";
        return false;
    }
    return true;
}

void ConfigureFrequency(freq_t* dest, const rtl_airband::FrequencyConfig& source) {
    if (dest->label != nullptr) {
        std::free(dest->label);
        dest->label = nullptr;
    }
    dest->frequency = source.frequency;
    dest->label = source.label.empty() ? nullptr : strdup(source.label.c_str());
    dest->modulation = ToAirbandModulation(source.modulation);
    dest->ampfactor = source.ampfactor;
    dest->active_counter = 0;
    if (source.squelch_threshold_dbfs < 0) {
        dest->squelch.set_squelch_level_threshold(dBFS_to_level(static_cast<float>(source.squelch_threshold_dbfs)));
    } else if (source.squelch_threshold_dbfs == 0) {
        dest->squelch.set_squelch_level_threshold(0.0f);
    }
    if (source.squelch_snr_threshold >= 0.0f) {
        dest->squelch.set_squelch_snr_threshold(source.squelch_snr_threshold);
    }
    if (source.notch_freq > 0.0f) {
        const float q = source.notch_q > 0.0f ? source.notch_q : 10.0f;
        dest->notch_filter = NotchFilter(source.notch_freq, WAVE_RATE, q);
    }
    if (source.ctcss_freq > 0.0f) {
        dest->squelch.set_ctcss_freq(source.ctcss_freq, WAVE_RATE);
    }
    if (source.bandwidth > 0) {
        dest->lowpass_filter = LowpassFilter(static_cast<float>(source.bandwidth) / 2.0f, WAVE_RATE);
    }
}

void ConfigureChannel(device_t* device, channel_t* channel, const rtl_airband::ChannelConfig& config, int channel_index) {
    for (int k = 0; k < AGC_EXTRA; ++k) {
        channel->wavein[k] = 20.0f;
        channel->waveout[k] = 0.5f;
    }
    channel->axcindicate = NO_SIGNAL;
    channel->mode = MM_MONO;
    channel->freq_count = static_cast<int>(config.frequencies.size());
    channel->freq_idx = 0;
    channel->highpass = config.highpass;
    channel->lowpass = config.lowpass;
    channel->afc = config.afc;
    channel->output_count = 0;
    channel->outputs = nullptr;
    channel->emit_audio_callback = config.emit_audio;
    channel->emit_iq_callback = config.emit_iq;
    channel->needs_raw_iq = config.emit_iq ? 1 : 0;
    channel->has_iq_outputs = config.emit_iq ? 1 : 0;
#ifdef NFM
    channel->pr = 0.0f;
    channel->pj = 0.0f;
    channel->prev_waveout = 0.5f;
    channel->alpha = config.tau == 0 ? device->alpha : exp(-1.0f / (WAVE_RATE * 1e-6f * config.tau));
#endif
    channel->freqlist = MakeFrequencyList(channel->freq_count);
    for (int freq_index = 0; freq_index < channel->freq_count; ++freq_index) {
        ConfigureFrequency(&channel->freqlist[freq_index], config.frequencies[freq_index]);
        if (config.frequencies[static_cast<size_t>(freq_index)].modulation == rtl_airband::Modulation::Iq) {
            device->emit_raw_iq_callback = true;
        }
#ifdef NFM
        if (channel->freqlist[freq_index].modulation == MOD_NFM) {
            channel->needs_raw_iq = 1;
        }
#endif
        if (channel->freqlist[freq_index].modulation == MOD_RAWIQ) {
            channel->needs_raw_iq = 1;
        }
    }

    if (device->mode == R_SCAN) {
        device->input->centerfreq = channel->freqlist[0].frequency + 20 * (double)(device->input->sample_rate / fft_size);
    }

    device->base_bins[channel_index] = device->bins[channel_index] =
        (size_t)ceil((channel->freqlist[0].frequency + device->input->sample_rate - device->input->centerfreq) /
                         (double)(device->input->sample_rate / fft_size) -
                     1.0) %
        fft_size;

    if (channel->needs_raw_iq) {
        double dm_dphi = static_cast<double>(channel->freqlist[0].frequency - device->input->centerfreq);
        double decimation_factor = static_cast<double>(device->input->sample_rate) / static_cast<double>(WAVE_RATE);
        double dm_dphi_correction = static_cast<double>(WAVE_RATE) / 2.0;
        dm_dphi_correction *= (decimation_factor - round(decimation_factor));
        dm_dphi_correction *= static_cast<double>(channel->freqlist[0].frequency - device->input->centerfreq) /
                              (static_cast<double>(device->input->sample_rate) / 2.0);
        dm_dphi -= dm_dphi_correction;
        dm_dphi /= static_cast<double>(WAVE_RATE);
        dm_dphi -= trunc(dm_dphi);
        dm_dphi *= 256.0 * 65536.0;
        channel->dm_dphi = static_cast<uint32_t>(static_cast<int>(dm_dphi));
        channel->dm_phi = 0;
    }
}

bool ConfigureDevices(const std::vector<rtl_airband::DeviceConfig>& configs, std::string* error) {
    device_count = static_cast<int>(configs.size());
    devices = static_cast<device_t*>(XCALLOC(device_count, sizeof(device_t)));
    for (int i = 0; i < device_count; ++i) {
        const auto& config = configs[static_cast<size_t>(i)];
        device_t* device = devices + i;
        device->mode = config.scan_mode ? R_SCAN : R_MULTICHANNEL;
        device->input = input_new("rtlsdr");
        if (device->input == nullptr) {
            if (error) *error = "failed to create RTLSDR input";
            return false;
        }
        if (!ConfigureRtlSdrInput(config, device->input, error)) {
            return false;
        }
#ifdef NFM
        device->alpha = config.tau == 0 ? alpha : exp(-1.0f / (WAVE_RATE * 1e-6f * config.tau));
#endif
        size_t fft_batch_len =
            FFT_BATCH * (2 * device->input->bytes_per_sample * (size_t)ceil((double)device->input->sample_rate / (double)WAVE_RATE));
        device->input->buf_size = MIN_BUF_SIZE;
        if (device->input->buf_size % fft_batch_len != 0) {
            device->input->buf_size += fft_batch_len - device->input->buf_size % fft_batch_len;
        }
        device->input->buffer = static_cast<unsigned char*>(
            XCALLOC(sizeof(unsigned char), device->input->buf_size + 2 * device->input->bytes_per_sample * fft_size));
        device->input->bufs = device->input->bufe = 0;
        device->input->overflow_count = 0;
        device->output_overrun_count = 0;
        device->waveend = device->waveavail = device->row = device->tq_head = device->tq_tail = 0;
        device->last_frequency = -1;
        device->emit_raw_iq_callback = false;
        device->raw_iq_callback = nullptr;
        device->raw_iq_callback_capacity_complex = 0;
        device->raw_iq_callback_fill_complex = 0;
        pthread_mutex_init(&device->tag_queue_lock, NULL);

        if (config.channels.empty()) {
            if (error) *error = "device must have at least one channel";
            return false;
        }
        device->channels = static_cast<channel_t*>(XCALLOC(config.channels.size(), sizeof(channel_t)));
        device->bins = static_cast<size_t*>(XCALLOC(config.channels.size(), sizeof(size_t)));
        device->base_bins = static_cast<size_t*>(XCALLOC(config.channels.size(), sizeof(size_t)));
        device->channel_count = static_cast<int>(config.channels.size());
        if (device->mode == R_SCAN && device->channel_count != 1) {
            if (error) *error = "scan mode requires exactly one channel";
            return false;
        }
        for (int channel_index = 0; channel_index < device->channel_count; ++channel_index) {
            const auto& channel_config = config.channels[static_cast<size_t>(channel_index)];
            if (channel_config.frequencies.empty()) {
                if (error) *error = "channel must define at least one frequency";
                return false;
            }
            ConfigureChannel(device, &device->channels[channel_index], channel_config, channel_index);
        }
        if (device->emit_raw_iq_callback) {
            const size_t raw_complex_per_window =
                static_cast<size_t>(WAVE_BATCH) *
                static_cast<size_t>(std::round(static_cast<double>(device->input->sample_rate) / static_cast<double>(WAVE_RATE)));
            device->raw_iq_callback_capacity_complex = std::max<size_t>(1, raw_complex_per_window);
            device->raw_iq_callback = static_cast<int16_t*>(
                XCALLOC(device->raw_iq_callback_capacity_complex * 2, sizeof(int16_t)));
        }
    }
    return true;
}

bool IsValidDeviceIndex(int device_index) {
    return device_index >= 0 && device_index < device_count;
}

bool UpdateRtlSdrCorrection(device_t* device, int correction, std::string* error) {
    if (device == nullptr || device->input == nullptr || device->input->dev_data == nullptr) {
        if (error) *error = "invalid device";
        return false;
    }
    auto* dev_data = static_cast<rtlsdr_dev_data_t*>(device->input->dev_data);
    if (dev_data->dev == nullptr) {
        dev_data->correction = correction;
        return true;
    }
    const int rc = rtlsdr_set_freq_correction(dev_data->dev, correction);
    if (rc < 0 && rc != -2) {
        if (error) *error = "failed to update RTLSDR frequency correction";
        return false;
    }
    dev_data->correction = correction;
    return true;
}

bool RetuneScanChannel(int device_index, channel_t* channel, device_t* device, std::string* error) {
    if (channel == nullptr || device == nullptr) {
        if (error) *error = "invalid scan channel";
        return false;
    }
    if (channel->freq_idx < 0 || channel->freq_idx >= channel->freq_count) {
        if (error) *error = "active scan frequency index out of range";
        return false;
    }
    const int center_freq = channel->freqlist[channel->freq_idx].frequency +
                            20 * (double)(device->input->sample_rate / fft_size);
    if (device->input->state == INPUT_RUNNING && input_set_centerfreq(device->input, center_freq) != 0) {
        if (error) *error = "failed to retune input";
        return false;
    }
    rtl_airband_emit_scan_callback(device_index, 0, channel);
    return true;
}

}  // namespace

bool rtl_airband_has_audio_callback() {
    return static_cast<bool>(g_embedded_state.callbacks.on_audio);
}

bool rtl_airband_has_iq_callback() {
    return static_cast<bool>(g_embedded_state.callbacks.on_iq);
}

bool rtl_airband_has_raw_iq_callback() {
    return static_cast<bool>(g_embedded_state.callbacks.on_raw_iq);
}

void rtl_airband_emit_audio_callback(int device_index, int channel_index, channel_t* channel) {
    if (!g_embedded_state.callbacks.on_audio || channel == nullptr || !channel->emit_audio_callback) {
        return;
    }
    rtl_airband::AudioBatch batch;
    batch.device_index = device_index;
    batch.channel_index = channel_index;
    batch.frequency = channel->freqlist[channel->freq_idx].frequency;
    batch.signal_active = channel->axcindicate != NO_SIGNAL;
    batch.stereo = channel->mode == MM_STEREO;
    batch.left = channel->waveout;
    batch.right = batch.stereo ? channel->waveout_r : nullptr;
    batch.sample_count = WAVE_BATCH;
    g_embedded_state.callbacks.on_audio(batch);
}

void rtl_airband_emit_iq_callback(int device_index, int channel_index, channel_t* channel) {
    if (!g_embedded_state.callbacks.on_iq || channel == nullptr || !channel->emit_iq_callback) {
        return;
    }
    rtl_airband::IqBatch batch;
    batch.device_index = device_index;
    batch.channel_index = channel_index;
    batch.frequency = channel->freqlist[channel->freq_idx].frequency;
    batch.signal_active = channel->axcindicate != NO_SIGNAL;
    batch.interleaved_iq = channel->iq_callback;
    batch.complex_sample_count = WAVE_BATCH;
    g_embedded_state.callbacks.on_iq(batch);
}

void rtl_airband_emit_raw_iq_callback(int device_index, device_t* device) {
    if (!g_embedded_state.callbacks.on_raw_iq || device == nullptr || !device->emit_raw_iq_callback ||
        device->raw_iq_callback == nullptr || device->raw_iq_callback_fill_complex == 0) {
        return;
    }
    rtl_airband::RawIqBatch batch;
    batch.device_index = device_index;
    batch.center_frequency = device->input ? device->input->centerfreq : 0;
    batch.sample_rate = device->input ? device->input->sample_rate : 0;
    batch.interleaved_iq_s16 = device->raw_iq_callback;
    batch.complex_sample_count = device->raw_iq_callback_fill_complex;
    g_embedded_state.callbacks.on_raw_iq(batch);
    device->raw_iq_callback_fill_complex = 0;
}

void rtl_airband_emit_scan_callback(int device_index, int channel_index, channel_t* channel) {
    if (!g_embedded_state.callbacks.on_scan || channel == nullptr) {
        return;
    }
    rtl_airband::ScanState state;
    state.device_index = device_index;
    state.channel_index = channel_index;
    state.frequency_index = channel->freq_idx;
    state.frequency = channel->freqlist[channel->freq_idx].frequency;
    if (channel->freqlist[channel->freq_idx].label != nullptr) {
        state.label = channel->freqlist[channel->freq_idx].label;
    }
    g_embedded_state.callbacks.on_scan(state);
}

namespace rtl_airband {

Session::Session() = default;

Session::~Session() {
    std::string error;
    Stop(&error);
}

bool Session::Configure(const std::vector<DeviceConfig>& devices_config, const RuntimeOptions& options, const Callbacks& callbacks,
                        std::string* error) {
    if (rtl_airband_runtime_running()) {
        if (error) *error = "runtime already running";
        return false;
    }
    rtl_airband_runtime_reset_globals();
    rtl_airband_runtime_set_log_to_stderr(options.log_to_stderr);
    rtl_airband_runtime_set_tui(options.enable_tui);
    rtl_airband_runtime_set_fft_size_log(options.fft_size_log);
    rtl_airband_runtime_set_shout_metadata_delay(options.shout_metadata_delay);
    rtl_airband_runtime_set_multiple_demod_threads(options.multiple_demod_threads);
    rtl_airband_runtime_set_multiple_output_threads(options.multiple_output_threads);
    rtl_airband_runtime_set_use_localtime(options.use_localtime);
    rtl_airband_runtime_set_quadrature_demod(options.use_quadrature_demod);

    g_embedded_state.callbacks = callbacks;
    g_embedded_state.devices = devices_config;
    if (!ConfigureDevices(devices_config, error)) {
        return false;
    }
    g_embedded_state.configured = true;
    configured_ = true;
    return true;
}

bool Session::Start(std::string* error) {
    if (!configured_ || !g_embedded_state.configured) {
        if (error) *error = "session not configured";
        return false;
    }
    return rtl_airband_runtime_start(true, error);
}

bool Session::Stop(std::string* error) {
    if (!rtl_airband_runtime_running()) {
        if (error) error->clear();
        return true;
    }
    return rtl_airband_runtime_stop(error);
}

bool Session::UpdateDeviceCorrection(int device_index, int correction, std::string* error) {
    if (!IsValidDeviceIndex(device_index)) {
        if (error) *error = "invalid device index";
        return false;
    }
    device_t* device = devices + device_index;
    if (!UpdateRtlSdrCorrection(device, correction, error)) {
        return false;
    }
    if (static_cast<size_t>(device_index) < g_embedded_state.devices.size()) {
        g_embedded_state.devices[static_cast<size_t>(device_index)].rtlsdr.correction = correction;
    }
    if (error) {
        error->clear();
    }
    return true;
}

bool Session::SetScanFrequencyIndex(int device_index, int frequency_index, std::string* error) {
    if (!IsValidDeviceIndex(device_index)) {
        if (error) *error = "invalid device index";
        return false;
    }
    device_t* device = devices + device_index;
    if (device->mode != R_SCAN || device->channel_count < 1) {
        if (error) *error = "device is not in scan mode";
        return false;
    }
    channel_t* channel = &device->channels[0];
    if (frequency_index < 0 || frequency_index >= channel->freq_count) {
        if (error) *error = "invalid frequency index";
        return false;
    }
    channel->freq_idx = frequency_index;
    return RetuneScanChannel(device_index, channel, device, error);
}

bool Session::UpdateScanFrequency(int device_index, int frequency_index, const FrequencyConfig& config, std::string* error) {
    if (!IsValidDeviceIndex(device_index)) {
        if (error) *error = "invalid device index";
        return false;
    }
    device_t* device = devices + device_index;
    if (device->mode != R_SCAN || device->channel_count < 1) {
        if (error) *error = "device is not in scan mode";
        return false;
    }
    channel_t* channel = &device->channels[0];
    if (frequency_index < 0 || frequency_index >= channel->freq_count) {
        if (error) *error = "invalid frequency index";
        return false;
    }
    ConfigureFrequency(&channel->freqlist[frequency_index], config);
    if (channel->freq_idx == frequency_index) {
        return SetScanFrequencyIndex(device_index, frequency_index, error);
    }
    return true;
}

bool Session::ReplaceScanFrequencies(int device_index, const std::vector<FrequencyConfig>& frequencies, std::string* error) {
    if (!IsValidDeviceIndex(device_index)) {
        if (error) *error = "invalid device index";
        return false;
    }
    if (frequencies.empty()) {
        if (error) *error = "scan list must contain at least one frequency";
        return false;
    }

    device_t* device = devices + device_index;
    if (device->mode != R_SCAN || device->channel_count < 1) {
        if (error) *error = "device is not in scan mode";
        return false;
    }

    channel_t* channel = &device->channels[0];
    freq_t* replacement = MakeFrequencyList(static_cast<int>(frequencies.size()));
    if (replacement == nullptr) {
        if (error) *error = "failed to allocate replacement scan list";
        return false;
    }
    for (size_t i = 0; i < frequencies.size(); ++i) {
        ConfigureFrequency(&replacement[i], frequencies[i]);
    }

    const int old_active_index = channel->freq_idx;
    if (channel->freqlist != nullptr) {
        g_embedded_state.retired_scan_frequency_lists.push_back(channel->freqlist);
    }
    channel->freqlist = replacement;
    channel->freq_count = static_cast<int>(frequencies.size());
    channel->freq_idx = std::clamp(old_active_index, 0, channel->freq_count - 1);

    if (static_cast<size_t>(device_index) < g_embedded_state.devices.size() &&
        !g_embedded_state.devices[static_cast<size_t>(device_index)].channels.empty()) {
        auto& stored_channel = g_embedded_state.devices[static_cast<size_t>(device_index)].channels[0];
        stored_channel.frequencies = frequencies;
    }

    return RetuneScanChannel(device_index, channel, device, error);
}

bool Session::Running() const {
    return rtl_airband_runtime_running();
}

}  // namespace rtl_airband
