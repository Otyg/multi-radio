#include "multi_radio/analog_demod_backend.hpp"

#include <cstdlib>

#include "multi_radio/am_demod.hpp"
#include "multi_radio/fm_demod.hpp"

namespace multi_radio {

namespace {

class LiquidAnalogDemodBackend final : public IAnalogDemodBackend {
 public:
  std::string Name() const override { return "liquid"; }

  bool Supports(Modulation modulation) const override {
    if (modulation == Modulation::kAm) {
      return AmDemodulator::Available();
    }
    if (modulation == Modulation::kNfm || modulation == Modulation::kWfm) {
      return FmDemodulator::Available();
    }
    return false;
  }

  std::string UnavailableReason(Modulation modulation) const override {
    if (modulation == Modulation::kAm) {
      return "AM demod unavailable: backend built without libliquid";
    }
    if (modulation == Modulation::kNfm || modulation == Modulation::kWfm) {
      return "FM demod unavailable: backend built without libliquid";
    }
    return "analog backend does not support this modulation";
  }

  void Reset() override {
    fm_demod_.Reset();
    fm_configured_ = false;
    am_demod_.Reset();
    am_configured_ = false;
  }

  bool Process(const AnalogDemodRequest& request, AnalogDemodResult* result,
               std::string* error) override {
    if (result == nullptr) {
      if (error != nullptr) {
        *error = "analog demod result is null";
      }
      return false;
    }
    result->pcm_s16le.clear();
    result->channel_rssi_db = -120.0f;
    if (request.block == nullptr) {
      if (error != nullptr) {
        *error = "analog demod input block is null";
      }
      return false;
    }
    if (!Supports(request.modulation)) {
      if (error != nullptr) {
        *error = UnavailableReason(request.modulation);
      }
      return false;
    }

    if (request.modulation == Modulation::kAm) {
      fm_demod_.Reset();
      fm_configured_ = false;

      const bool reconfigure = !am_configured_ ||
                               am_input_sr_hz_ != request.input_sample_rate_hz ||
                               am_audio_sr_hz_ != request.audio_sample_rate_hz ||
                               am_channel_bw_hz_ != request.channel_bandwidth_hz;
      if (reconfigure) {
        if (!am_demod_.Configure(request.input_sample_rate_hz, request.audio_sample_rate_hz,
                                 request.channel_bandwidth_hz, error)) {
          am_configured_ = false;
          return false;
        }
        am_configured_ = true;
        am_input_sr_hz_ = request.input_sample_rate_hz;
        am_audio_sr_hz_ = request.audio_sample_rate_hz;
        am_channel_bw_hz_ = request.channel_bandwidth_hz;
      }

      AmDemodProcessStats stats;
      if (!am_demod_.ProcessIq(request.block->interleaved_iq, &result->pcm_s16le, &stats, error)) {
        return false;
      }
      result->channel_rssi_db = stats.channel_rssi_db;
      return true;
    }

    am_demod_.Reset();
    am_configured_ = false;

    const bool reconfigure = !fm_configured_ ||
                             fm_input_sr_hz_ != request.input_sample_rate_hz ||
                             fm_audio_sr_hz_ != request.audio_sample_rate_hz ||
                             fm_channel_bw_hz_ != request.channel_bandwidth_hz ||
                             fm_modulation_ != request.modulation;
    if (reconfigure) {
      if (!fm_demod_.Configure(request.input_sample_rate_hz, request.audio_sample_rate_hz,
                               request.modulation, request.channel_bandwidth_hz, error)) {
        fm_configured_ = false;
        return false;
      }
      fm_configured_ = true;
      fm_input_sr_hz_ = request.input_sample_rate_hz;
      fm_audio_sr_hz_ = request.audio_sample_rate_hz;
      fm_channel_bw_hz_ = request.channel_bandwidth_hz;
      fm_modulation_ = request.modulation;
    }

    FmDemodProcessStats stats;
    if (!fm_demod_.ProcessIq(request.block->interleaved_iq, &result->pcm_s16le, &stats, error)) {
      return false;
    }
    result->channel_rssi_db = stats.channel_rssi_db;
    return true;
  }

 private:
  FmDemodulator fm_demod_;
  bool fm_configured_ = false;
  uint32_t fm_input_sr_hz_ = 0;
  uint32_t fm_audio_sr_hz_ = 0;
  uint32_t fm_channel_bw_hz_ = 0;
  Modulation fm_modulation_ = Modulation::kNfm;

  AmDemodulator am_demod_;
  bool am_configured_ = false;
  uint32_t am_input_sr_hz_ = 0;
  uint32_t am_audio_sr_hz_ = 0;
  uint32_t am_channel_bw_hz_ = 0;
};

}  // namespace

std::unique_ptr<IAnalogDemodBackend> CreateAnalogDemodBackend(std::string* selected_backend_name,
                                                              std::string* selection_warning) {
  const char* requested = std::getenv("MR_ANALOG_DEMOD_BACKEND");
  const std::string requested_name = (requested != nullptr && *requested != '\0')
                                         ? std::string(requested)
                                         : std::string("liquid");

  if (selection_warning != nullptr) {
    selection_warning->clear();
  }
  if (selected_backend_name != nullptr) {
    selected_backend_name->clear();
  }

  if (requested_name != "liquid") {
    if (selection_warning != nullptr) {
      *selection_warning =
          "MR_ANALOG_DEMOD_BACKEND=" + requested_name +
          " is not available in-process; falling back to liquid";
    }
  }

  auto backend = std::make_unique<LiquidAnalogDemodBackend>();
  if (selected_backend_name != nullptr) {
    *selected_backend_name = backend->Name();
  }
  return backend;
}

}  // namespace multi_radio
