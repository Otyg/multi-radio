# Multi-Radio (API-Only Backend)

Multi-radio client/server application with C++20 backend and Qt6 frontend.

The backend is now intentionally minimal and keeps only the API contract used by the frontend.
It no longer depends on RTL-SDR, hardware tuning paths, DSP decode pipeline, or dynamic plugin loading.

## Current scope

- gRPC client/server API (`proto/radio.proto`) with bearer-token auth.
- Receiver control API (`list/get/start/stop/set mode/set mode config`).
- Telemetry streams:
  - receiver events
  - decoded messages (reserved channel, currently backend-generated none)
  - audio frames
- API-only virtual receiver runtime (synthetic audio + status/events for integration).
- Qt6 frontend compatibility preserved.

## Repository structure

- `proto/`: gRPC + protobuf contract.
- `backend/`: API server runtime and minimal receiver/event/audio engine.
- `docs/backend_thin_thick_split.md`: proposed thin-backend / thick-DSP split architecture.
- `frontend/`: Qt6 desktop client.
- `tests/`: core + API behavior tests.

## Dependency bootstrap (Ubuntu/Debian)

```bash
./scripts/bootstrap_deps.sh -y
```

Server-only environment (skip Qt):

```bash
./scripts/bootstrap_deps.sh --headless -y
```

## Build

```bash
cmake -S . -B build \
  -DMR_BUILD_SERVER=ON \
  -DMR_BUILD_FRONTEND=ON \
  -DMR_BUILD_TESTS=ON
cmake --build build -j
```

## Run server

```bash
MR_BIND_ADDRESS=0.0.0.0:50051 \
MR_AUTH_TOKEN=multi-radio-dev-token \
MR_LOG_DIR=./logs \
./build/backend/multi_radio_server
```

## Run Qt client

Create `client.ini`:

```ini
grpc_target=127.0.0.1:50051
auth_token=multi-radio-dev-token
```

Run:

```bash
./build/frontend/multi_radio_client
```

Optional config path:

```bash
MR_CLIENT_CONFIG=/path/to/client.ini ./build/frontend/multi_radio_client
```

## Run TUI client

The ncurses MVP reuses the same `client.ini` and consumes radar snapshots over gRPC.
It renders the radar pane through `gnuplot` in terminal text mode, so `gnuplot`
must be installed on the machine where you run it.

Build with TUI enabled:

```bash
cmake -S . -B build \
  -DMR_BUILD_SERVER=ON \
  -DMR_BUILD_FRONTEND=OFF \
  -DMR_BUILD_TUI=ON \
  -DMR_BUILD_TESTS=OFF
cmake --build build --target multi_radio_tui -j
```

Run:

```bash
./build/frontend/multi_radio_tui
```

Optional center override:

```bash
./build/frontend/multi_radio_tui --center-lat 57.7 --center-lon 11.9
```

## VDES/ASM IQ Recording

Selecting `VDES ASM` uses the `vdes_iq_recorder` null-demodulator. It does not
emit decoded messages; it writes raw interleaved int16 IQ plus a JSON sidecar.

```bash
MR_IQ_RECORD_DIR=./recordings/vdes \
MR_IQ_RECORD_PREFIX=vdes \
MR_IQ_RECORD_MAX_BYTES=268435456 \
./build/backend/multi_radio_server
```

Output files are named like:

```text
recordings/vdes/vdes_20260519T120000Z_161950000_2048000_0.iq16
recordings/vdes/vdes_20260519T120000Z_161950000_2048000_0.json
```

Replay a recording through the first-pass VDES PHY diagnostic plugin:

```bash
./build/tools/vdes_replay \
  --iq recordings/vdes/vdes_20260519T120000Z_161950000_2048000_0.iq16 \
  --rate 2048000 \
  --freq 161950000 \
  --diag-blocks 1 \
  --jsonl
```

## Windows frontend build from WSL

```bash
VCPKG_ROOT=~/vcpkg ./scripts/build_windows_frontend.sh
```

Artifact:
- `build-win-frontend/frontend/multi_radio_client.exe`

## Raspberry Pi 2 cross-build with Docker

The repository now includes `docker/raspberry_pi2_armhf/Dockerfile` for an
armhf cross-build environment targeting Raspberry Pi 2.

Build the image:

```bash
docker build -t multi-radio-rpi2 -f docker/raspberry_pi2_armhf/Dockerfile .
```

Run the cross-build:

```bash
docker run --rm \
  -v "$PWD":/src \
  -v "$PWD/out/rpi2":/out \
  -v /home/maves/projects/dump1090:/deps/dump1090 \
  multi-radio-rpi2
```

The container:

- configures and builds in an isolated container-only build directory
- installs armhf protobuf/gRPC/liquid/FFTW/RTL-SDR dependencies through Debian multiarch
- cross-builds `rnnoise` into `/opt/rpi2-prefix`
- builds `dump1090`/`libmodes` from `/deps/dump1090` with `make library` and `make library-install`
- installs `libmodes` into the armhf prefix under `/opt/rpi2-prefix/usr/local`

Artifacts are copied to:

- `out/rpi2/build`
- `out/rpi2/prefix`

## Tests

```bash
ctest --test-dir build --output-on-failure
```
