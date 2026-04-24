#pragma once

#include <string>
#include <grpcpp/server_context.h>

namespace multi_radio::auth {

bool ValidateBearerTokenValue(const std::string& authorization_header,
                              const std::string& expected_token);
bool ValidateBearerToken(const grpc::ServerContext& context, const std::string& expected_token);

}  // namespace multi_radio::auth
