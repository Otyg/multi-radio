#pragma once

#include <QLibrary>
#include <QObject>
#include <QString>
#include <QVector>

// Include the plugin ABI (relative from the plugins/include directory added via CMake).
#include "mr_plugin_api.h"

namespace iq_analyzer {

// ---------------------------------------------------------------------------
// PluginHandle — loaded .so with resolved function pointers and metadata.
// ---------------------------------------------------------------------------
struct PluginHandle {
    QString      path;
    QString      name;
    QString      description;
    MrPluginRole role = MR_PLUGIN_ROLE_DEMODULATOR;

    using CreateFn      = MrPluginCtx* (*)();
    using DestroyFn     = void (*)(MrPluginCtx*);
    using GetMetaFn     = const MrPluginMeta* (*)();
    using ProcessIqFn   = void (*)(MrPluginCtx*, const int16_t*, uint32_t,
                                   uint32_t, double, uint64_t, MrEmitFn, void*);
    using ProcessBitsFn = void (*)(MrPluginCtx*, const uint8_t*, uint32_t,
                                   double, uint64_t, const char*, MrEmitFn, void*);
    using SetParamFn    = int  (*)(MrPluginCtx*, const char*, const char*);

    CreateFn      create      = nullptr;
    DestroyFn     destroy     = nullptr;
    GetMetaFn     get_meta    = nullptr;
    ProcessIqFn   process_iq  = nullptr;
    ProcessBitsFn process_bits= nullptr;
    SetParamFn    set_param   = nullptr;

    QLibrary*     lib         = nullptr;
};

// ---------------------------------------------------------------------------
// PluginRegistry — singleton; discovers .so files and caches their metadata.
// ---------------------------------------------------------------------------
class PluginRegistry : public QObject {
    Q_OBJECT
public:
    static PluginRegistry& instance();

    // Scan a directory for *.so files and load their metadata.
    // May be called multiple times to add more directories.
    void scan(const QString& dir);

    const QVector<PluginHandle*>& all() const { return plugins_; }
    QVector<const PluginHandle*> byRole(MrPluginRole role) const;

    // Best-effort: find plugins dir relative to the running binary.
    static QString defaultPluginDir();

private:
    explicit PluginRegistry(QObject* parent = nullptr) : QObject(parent) {}
    QVector<PluginHandle*> plugins_;
};

// ---------------------------------------------------------------------------
// ChainMessage — one decoded message emitted anywhere in the plugin chain.
// ---------------------------------------------------------------------------
struct ChainMessage {
    QString signal_type;
    QString payload;
    QString kv_json;
    double  freq_hz  = 0.0;
    int     stage    = 0;  // 0=demod, 1=decoder, 2=postproc
};

// ---------------------------------------------------------------------------
// PluginChainRunner — executes DEMODULATOR → DECODER → POSTPROCESSING.
// ---------------------------------------------------------------------------
class PluginChainRunner {
public:
    void setDemodPlugin   (const PluginHandle* p) { demod_    = p; }
    void setDecoderPlugin (const PluginHandle* p) { decoder_  = p; }
    void setPostprocPlugin(const PluginHandle* p) { postproc_ = p; }

    // Run the chain over the IQ block. Returns all collected messages.
    QVector<ChainMessage> run(const int16_t* iq_samples,
                               size_t         n_pairs,
                               uint32_t       sample_rate_hz,
                               double         center_freq_hz) const;

private:
    static constexpr uint32_t kBlockSize = 4096;

    const PluginHandle* demod_   = nullptr;
    const PluginHandle* decoder_ = nullptr;
    const PluginHandle* postproc_= nullptr;
};

}  // namespace iq_analyzer
