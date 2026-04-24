#!/usr/bin/env bash
set -euo pipefail

DRY_RUN=0
HEADLESS=0
SKIP_RTL=0
ASSUME_YES=0

usage() {
  cat <<'USAGE'
Usage: scripts/bootstrap_deps.sh [options]

Install build/runtime dependencies for Multi-Radio MVP on Ubuntu/Debian.

Options:
  --headless      Skip Qt6 packages (server-only environments)
  --skip-rtl      Skip RTL-SDR packages
  --dry-run       Print commands without executing
  -y, --yes       Non-interactive apt install (-y)
  -h, --help      Show this help
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)
      HEADLESS=1
      shift
      ;;
    --skip-rtl)
      SKIP_RTL=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -y|--yes)
      ASSUME_YES=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -r /etc/os-release ]]; then
  echo "Cannot detect distribution: /etc/os-release missing" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
DISTRO_ID="${ID:-}"
if [[ "$DISTRO_ID" != "ubuntu" && "$DISTRO_ID" != "debian" ]]; then
  echo "Unsupported distro '$DISTRO_ID'. This bootstrap supports Ubuntu/Debian only." >&2
  exit 1
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "apt-get not found. Cannot continue." >&2
  exit 1
fi

SUDO=()
if [[ "${EUID}" -ne 0 ]]; then
  if command -v sudo >/dev/null 2>&1; then
    SUDO=(sudo)
  else
    echo "This script needs root privileges (run as root or install sudo)." >&2
    exit 1
  fi
fi

run() {
  echo "+ $*"
  if [[ "$DRY_RUN" -eq 0 ]]; then
    "$@"
  fi
}

common_pkgs=(
  build-essential
  cmake
  ninja-build
  pkg-config
  git
  curl
  ca-certificates
)

proto_pkgs=(
  protobuf-compiler
  libprotobuf-dev
  protobuf-compiler-grpc
  libgrpc++-dev
)

qt_pkgs=(
  qt6-base-dev
  qt6-base-dev-tools
  qt6-tools-dev-tools
)

rtl_pkgs=(
  librtlsdr-dev
  rtl-sdr
)

all_pkgs=("${common_pkgs[@]}" "${proto_pkgs[@]}")
if [[ "$HEADLESS" -eq 0 ]]; then
  all_pkgs+=("${qt_pkgs[@]}")
fi
if [[ "$SKIP_RTL" -eq 0 ]]; then
  all_pkgs+=("${rtl_pkgs[@]}")
fi

APT_FLAGS=()
if [[ "$ASSUME_YES" -eq 1 ]]; then
  APT_FLAGS=(-y)
fi

echo "Detected distro: ${PRETTY_NAME:-$DISTRO_ID}"
echo "Headless mode: $HEADLESS"
echo "Skip RTL-SDR: $SKIP_RTL"
echo "Dry run: $DRY_RUN"

echo "Installing packages:"
printf '  - %s\n' "${all_pkgs[@]}"

run "${SUDO[@]}" apt-get update
run "${SUDO[@]}" apt-get install "${APT_FLAGS[@]}" "${all_pkgs[@]}"

echo
cat <<'NEXT'
Bootstrap complete.

Suggested next steps:
  cmake -S . -B build -DMR_BUILD_SERVER=ON -DMR_BUILD_FRONTEND=ON -DMR_BUILD_TESTS=ON -DMR_ENABLE_RTLSDR=ON
  cmake --build build -j
  ctest --test-dir build --output-on-failure
NEXT
