#include "multi_radio/auth.hpp"

#include <grpcpp/server_context.h>

namespace multi_radio::auth {

bool ValidateBearerTokenValue(const std::string& authorization_header,
                              const std::string& expected_token) {
  if (expected_token.empty()) {
    return true;
  }
  return authorization_header == ("Bearer " + expected_token);
}

bool ValidateBearerToken(const grpc::ServerContext& context, const std::string& expected_token) {
  if (expected_token.empty()) {
    return true;
  }

  const auto it = context.client_metadata().find("authorization");
  if (it == context.client_metadata().end()) {
    return false;
  }

  const std::string value(it->second.data(), it->second.length());
  return ValidateBearerTokenValue(value, expected_token);
}

}  // namespace multi_radio::auth
