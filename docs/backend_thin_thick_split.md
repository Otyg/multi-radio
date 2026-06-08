# Thin backend / thick backend split

## Goal
Separate the SDR-facing "thin" backend from the DSP-heavy "thick" backend so the first part can run on the SDR host and the second part can run on a stronger machine.

## Proposed split

### Thin backend (SDR host)
Responsibilities:
- Open and control the SDR dongle
- Set frequency, gain, sample rate, hardware bandwidth, DC blocker, notch, LO offset, PPM correction
- Handle squelch thresholds and scan-list hopping at a low level
- Publish raw IQ stream to the thick backend
- Optionally publish minimal audio/metadata for local monitoring

Keep this part as small and deterministic as possible.

### Thick backend (DSP host)
Responsibilities:
- Demodulate IQ into audio or symbols
- Run plugin decoders (AIS/ADS-B/DSC/VDES/etc.)
- Decode messages, publish events, store state
- Render radar/target tracking and higher-level UI logic
- Run heavy DSP algorithms and filtering

## Existing boundary that already fits this model
The current gRPC API already exposes the right transport split:
- SetMode / SetModeConfig: control tuning and mode
- StreamIqFrames: raw IQ transport from thin to thick backend
- StreamAudioFrames / StreamReceiverEvents / StreamDecodedMessages: output paths for the thick backend

This means we can use the current proto surface as the first version of the thin/thick contract.

## Recommended implementation phases

### Phase 1 — Make the SDR side the real thin backend
- Keep the current backend as the default implementation
- Treat `receiver_worker` + `StreamIqFrames` as the thin-host path
- Make sure all SDR-only settings stay in this layer

### Phase 2 — Add an optional remote DSP backend
- Introduce a small remote worker that consumes `StreamIqFrames`
- It applies demodulation/decoding and publishes decoded results back through the existing event streams
- The UI can continue to talk to the same control service

### Phase 3 — Move heavy DSP out of the SDR host
- Move plugin demod/decoder stages to the thick backend
- Keep only the minimal SDR control path on the thin host
- Add explicit mode flags so the thick backend can choose whether to run locally or remotely

## Suggested config model
Add an optional setting such as:
- `backend_mode = local | remote`
- `remote_dsp_host = host:port`
- `iq_transport = grpc_stream | udp_raw`

Default behavior should remain unchanged (`local`) so current users are not affected.

## Suggested safety rules
- The thin backend must stay operational even if the thick backend is unavailable
- The thick backend must be able to reconnect to IQ streams without losing mode configuration
- The control plane must remain the single source of truth for mode/frequency/gain

## Why this is a good first step
The current codebase already has a natural split point:
- SDR and tuning logic live in the receiver/worker layer
- DSP and decoding live in the plugin and demod path

That makes the split mostly an architectural refactor and transport decision, not a rewrite of the whole system.
