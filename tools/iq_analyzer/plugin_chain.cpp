#include "plugin_chain.hpp"

#include <cstdlib>
#include <cstring>
#include <vector>

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

namespace iq_analyzer {

// ---------------------------------------------------------------------------
// PluginRegistry
// ---------------------------------------------------------------------------

PluginRegistry& PluginRegistry::instance() {
    static PluginRegistry reg;
    return reg;
}

QString PluginRegistry::defaultPluginDir() {
    // Try $MR_PLUGIN_DIR first.
    const char* env = std::getenv("MR_PLUGIN_DIR");
    if (env && *env) return QString::fromLocal8Bit(env);

    // Derive from the binary location.
    const QString bin = QCoreApplication::applicationDirPath();
    for (const QString& rel : {"../../plugins", "../plugins", "plugins"}) {
        const QString candidate = QDir::cleanPath(bin + "/" + rel);
        if (QDir(candidate).exists()) return candidate;
    }
    return {};
}

void PluginRegistry::scan(const QString& dir) {
    if (dir.isEmpty()) return;
    QDir d(dir, "*.so", QDir::Name, QDir::Files | QDir::Readable);
    for (const QFileInfo& fi : d.entryInfoList()) {
        const QString path = fi.absoluteFilePath();
        // Skip already-loaded paths.
        bool already = false;
        for (const PluginHandle* h : plugins_)
            if (h->path == path) { already = true; break; }
        if (already) continue;

        auto* lib = new QLibrary(path, this);
        if (!lib->load()) { delete lib; continue; }

        auto get_meta = reinterpret_cast<PluginHandle::GetMetaFn>(
            lib->resolve("mr_plugin_get_meta"));
        if (!get_meta) { lib->unload(); delete lib; continue; }

        const MrPluginMeta* meta = get_meta();
        if (!meta || meta->api_version != MR_PLUGIN_API_VERSION) {
            lib->unload(); delete lib; continue;
        }

        auto* h = new PluginHandle;
        h->lib         = lib;
        h->path        = path;
        h->name        = QString::fromUtf8(meta->name       ? meta->name        : "");
        h->description = QString::fromUtf8(meta->description? meta->description : "");
        h->role        = meta->role;

        h->create      = reinterpret_cast<PluginHandle::CreateFn>     (lib->resolve("mr_plugin_create"));
        h->destroy     = reinterpret_cast<PluginHandle::DestroyFn>    (lib->resolve("mr_plugin_destroy"));
        h->get_meta    = get_meta;
        h->process_iq  = reinterpret_cast<PluginHandle::ProcessIqFn>  (lib->resolve("mr_plugin_process_iq"));
        h->process_bits= reinterpret_cast<PluginHandle::ProcessBitsFn>(lib->resolve("mr_plugin_process_bits"));
        h->set_param   = reinterpret_cast<PluginHandle::SetParamFn>   (lib->resolve("mr_plugin_set_param"));

        plugins_.append(h);
    }
}

QVector<const PluginHandle*> PluginRegistry::byRole(MrPluginRole role) const {
    QVector<const PluginHandle*> out;
    for (const PluginHandle* h : plugins_)
        if (h->role == role) out.append(h);
    return out;
}

// ---------------------------------------------------------------------------
// Plugin chain helpers (static emit callbacks)
// ---------------------------------------------------------------------------

namespace {

// Extract bit_count from the kv_json produced by demodulators.
uint32_t extractBitCount(const char* kv_json, uint32_t default_count) {
    if (!kv_json) return default_count;
    const char* p = std::strstr(kv_json, "\"bit_count\":\"");
    if (!p) return default_count;
    p += 13;
    return static_cast<uint32_t>(std::atol(p));
}

// Convert hex string (e.g. "DEADBEEF") to packed bytes.
std::vector<uint8_t> hexToBytes(const char* hex) {
    if (!hex) return {};
    const size_t len = std::strlen(hex);
    std::vector<uint8_t> out(len / 2);
    for (size_t i = 0; i < out.size(); ++i) {
        char buf[3] = {hex[i*2], hex[i*2+1], '\0'};
        out[i] = static_cast<uint8_t>(std::strtol(buf, nullptr, 16));
    }
    return out;
}

// Context forwarded through the emit callbacks.
struct ChainCtx {
    const PluginHandle*    decoder_plugin   = nullptr;
    MrPluginCtx*           decoder_ctx      = nullptr;
    const PluginHandle*    postproc_plugin  = nullptr;
    MrPluginCtx*           postproc_ctx     = nullptr;
    QVector<ChainMessage>* output           = nullptr;
};

// Emit from POSTPROCESSING (or last stage) → collect.
static void postproc_emit(const char* signal_type, const char* payload,
                           double freq_hz, uint64_t /*unix_ms*/,
                           const char* kv_json, void* user_data) {
    auto* ctx = static_cast<ChainCtx*>(user_data);
    ctx->output->append({
        QString::fromUtf8(signal_type ? signal_type : ""),
        QString::fromUtf8(payload     ? payload     : ""),
        QString::fromUtf8(kv_json     ? kv_json     : ""),
        freq_hz, 2
    });
}

// Emit from DECODER → route to postproc or collect.
static void decoder_emit(const char* signal_type, const char* payload,
                          double freq_hz, uint64_t unix_ms,
                          const char* kv_json, void* user_data) {
    auto* ctx = static_cast<ChainCtx*>(user_data);
    if (ctx->postproc_plugin && ctx->postproc_ctx &&
        ctx->postproc_plugin->process_bits) {
        const auto bytes = hexToBytes(payload);
        const uint32_t bit_count =
            extractBitCount(kv_json, static_cast<uint32_t>(bytes.size() * 8));
        ctx->postproc_plugin->process_bits(
            ctx->postproc_ctx, bytes.data(), bit_count,
            freq_hz, unix_ms, signal_type, postproc_emit, ctx);
    } else {
        ctx->output->append({
            QString::fromUtf8(signal_type ? signal_type : ""),
            QString::fromUtf8(payload     ? payload     : ""),
            QString::fromUtf8(kv_json     ? kv_json     : ""),
            freq_hz, 1
        });
    }
}

// Emit from DEMODULATOR → route to decoder or collect.
static void demod_emit(const char* signal_type, const char* payload,
                        double freq_hz, uint64_t unix_ms,
                        const char* kv_json, void* user_data) {
    auto* ctx = static_cast<ChainCtx*>(user_data);
    if (ctx->decoder_plugin && ctx->decoder_ctx &&
        ctx->decoder_plugin->process_bits) {
        const auto bytes = hexToBytes(payload);
        const uint32_t bit_count =
            extractBitCount(kv_json, static_cast<uint32_t>(bytes.size() * 8));
        ctx->decoder_plugin->process_bits(
            ctx->decoder_ctx, bytes.data(), bit_count,
            freq_hz, unix_ms, signal_type, decoder_emit, ctx);
    } else {
        ctx->output->append({
            QString::fromUtf8(signal_type ? signal_type : ""),
            QString::fromUtf8(payload     ? payload     : ""),
            QString::fromUtf8(kv_json     ? kv_json     : ""),
            freq_hz, 0
        });
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// PluginChainRunner::run
// ---------------------------------------------------------------------------

QVector<ChainMessage> PluginChainRunner::run(const int16_t* iq_samples,
                                              size_t         n_pairs,
                                              uint32_t       sample_rate_hz,
                                              double         center_freq_hz) const {
    QVector<ChainMessage> results;
    if (!demod_ || !demod_->process_iq || !demod_->create) return results;

    MrPluginCtx* demod_ctx = demod_->create();
    if (!demod_ctx) return results;

    MrPluginCtx* decoder_ctx  = (decoder_  && decoder_->create)  ? decoder_->create()  : nullptr;
    MrPluginCtx* postproc_ctx = (postproc_ && postproc_->create) ? postproc_->create() : nullptr;

    ChainCtx chain{
        .decoder_plugin  = decoder_,
        .decoder_ctx     = decoder_ctx,
        .postproc_plugin = postproc_,
        .postproc_ctx    = postproc_ctx,
        .output          = &results,
    };

    // Feed IQ in kBlockSize chunks.
    for (size_t offset = 0; offset < n_pairs; offset += kBlockSize) {
        const uint32_t block =
            static_cast<uint32_t>(std::min(static_cast<size_t>(kBlockSize), n_pairs - offset));
        demod_->process_iq(demod_ctx,
                            iq_samples + offset * 2, block,
                            sample_rate_hz, center_freq_hz,
                            0 /* unix_ms — unknown for file playback */,
                            demod_emit, &chain);
    }

    if (demod_ctx  && demod_->destroy)    demod_->destroy(demod_ctx);
    if (decoder_ctx && decoder_->destroy) decoder_->destroy(decoder_ctx);
    if (postproc_ctx && postproc_->destroy) postproc_->destroy(postproc_ctx);

    return results;
}

}  // namespace iq_analyzer
