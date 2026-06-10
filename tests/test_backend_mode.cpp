#include <cassert>

#include "multi_radio/backend_mode.hpp"

int main() {
  using multi_radio::BackendMode;
  using multi_radio::IsImplementedBackendMode;
  using multi_radio::IsRemoteBackendMode;
  using multi_radio::IsValidIqTransport;
  using multi_radio::ParseBackendMode;

  assert(ParseBackendMode("local") == BackendMode::kLocal);
  assert(ParseBackendMode("REMOTE") == BackendMode::kRemote);
  assert(ParseBackendMode("thick") == BackendMode::kRemote);
  assert(!IsRemoteBackendMode(BackendMode::kLocal));
  assert(IsRemoteBackendMode(BackendMode::kRemote));
  assert(IsImplementedBackendMode(BackendMode::kLocal));
  assert(IsImplementedBackendMode(BackendMode::kRemote));
  assert(IsValidIqTransport("grpc_stream"));
  assert(IsValidIqTransport("udp_raw"));
  assert(!IsValidIqTransport("tcp_raw"));

  return 0;
}
