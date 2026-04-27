# Multi-Radio MVP

Multi-radio client/server application for RTL-SDR with C++20 backend and Qt6 frontend.

## Implemented MVP capabilities

- gRPC client/server API (`proto/radio.proto`) with token auth via `authorization: Bearer <token>`.
- Radio server modes:
  - `FIXED`
  - `SCAN_RANGE`
  - `SCAN_LIST`
  - `AIR_MARINE_PLOT` (fixed dwell `5s`, scan hops between `162.000 MHz` for dual-channelized AIS1/AIS2 and CH70 `156.525 MHz` for DSC)
- Multiple simultaneous receivers (one worker thread per receiver).
- In-process plugin system (`.so`) with C ABI + API version check.
- Event + decoded message telemetry streaming.
- In-memory event bus and rotating JSONL log persistence.
- Qt6 client (receiver control + live table/log with filters by signal/receiver/time).
- Visualization source toggle in client (`Spectrum view`):
  - `Demodulated` (default)
  - `Receiver spectrum` (shows tuned center frequency in the middle with span `center +/- channel_bandwidth`)
- Optional RF cleanup/tuning controls in client (default off):
  - `DC blocker` with configurable cutoff
  - `Center notch` with configurable notch width
  - `LO offset` with configurable offset (applies in fixed and scanning modes)
- DSC plugin now includes an experimental receive/decode path (using current tuned frequency)
  with BFSK bit recovery plus first-pass DSC frame parsing (format/address/category/telecommand/EOS/ECC signals).

## Repository structure

- `proto/`: gRPC + protobuf contract.
- `backend/`: server runtime, mode engine, plugin host, device layer.
- `frontend/`: Qt6 desktop client.
- `tests/`: unit/integration tests and plugin loader failure fixtures.

## Build

## Dependency bootstrap (Ubuntu/Debian)

```bash
./scripts/bootstrap_deps.sh -y
```

Useful variants:
- Server-only environment (skip Qt): `./scripts/bootstrap_deps.sh --headless -y`
- No RTL-SDR packages: `./scripts/bootstrap_deps.sh --skip-rtl -y`
- Preview only: `./scripts/bootstrap_deps.sh --dry-run`

## Build

```bash
cmake -S . -B build \
  -DMR_BUILD_SERVER=ON \
  -DMR_BUILD_FRONTEND=ON \
  -DMR_BUILD_TESTS=ON \
  -DMR_ENABLE_RTLSDR=ON
cmake --build build -j
```

If dependencies are missing, relevant targets are skipped with warnings:
- Missing Protobuf/gRPC: server/frontend skipped.
- Missing Qt6: frontend skipped.
- Missing `librtlsdr`: mock device backend used.

## Run server

```bash
MR_BIND_ADDRESS=0.0.0.0:50051 \
MR_AUTH_TOKEN=multi-radio-dev-token \
MR_PLUGIN_DIR=./build/plugins \
MR_LOG_DIR=./logs \
./build/backend/multi_radio_server
```

## Run client

Create `client.ini` (or point to another file path) with:

```ini
grpc_target=192.168.128.82:50051
auth_token=multi-radio-dev-token
```

Then start the client:

```bash
./build/frontend/multi_radio_client
```

Optional overrides:
- Custom config path with env: `MR_CLIENT_CONFIG=/path/to/client.ini ./build/frontend/multi_radio_client`
- Custom config path with argument: `./build/frontend/multi_radio_client --config /path/to/client.ini`
- Legacy env fallback still works:

```bash
MR_GRPC_TARGET=192.168.128.82:50051 \
MR_AUTH_TOKEN=multi-radio-dev-token \
./build/frontend/multi_radio_client
```

### Scan-list CSV import

`SCAN_LIST` is no longer capped to 5 channels. You can import channels from CSV in the client with:

`<frequency MHz>;<modulation>;<label>`

Examples:

```text
156.800;NFM;VHF 16
118.300;AM;ATIS
98.500;WFM;Broadcast FM
```

Supported modulation values in import: `AM`, `NFM`, `FM`, `WFM`.

`SCAN_LIST` also supports `Monitor mode` in the client. In this mode, each channel is treated as
open for monitoring/audio while frequency hopping continues strictly on dwell timing (no squelch-hold on a channel).

If the client crashes on startup around audio backend initialization, temporarily disable audio output:

```bash
MR_DISABLE_AUDIO_OUTPUT=1 ./build/frontend/multi_radio_client
```

## Tests

```bash
ctest --test-dir build --output-on-failure
```

Test coverage includes:
- Scan scheduler behavior.
- Mode state transitions (worker start/stop + mode/config updates).
- Plugin loader success/failure paths.
- Multi-receiver integration behavior.
- Bearer-token validation helper.

## Plugin ABI

See `backend/include/multi_radio/plugin_api.h`.

A plugin must export:
- `multi_radio_get_plugin_descriptor()`.

Descriptor includes:
- `plugin_name`, `plugin_version`, `api_version`, `supported_signals_csv`.
- Hooks: `init`, `process_iq`, `flush`, `shutdown`.
