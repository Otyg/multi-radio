#pragma once

#include <string>

namespace grpc {
class ServerContext;
}

namespace multi_radio::auth {

bool ValidateBearerTokenValue(const std::string& authorization_header,
                              const std::string& expected_token);
bool ValidateBearerToken(const grpc::ServerContext& context, const std::string& expected_token);

}  // namespace multi_radio::auth
