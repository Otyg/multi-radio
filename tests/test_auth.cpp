#include <cassert>

#include "multi_radio/auth.hpp"

int main() {
  using multi_radio::auth::ValidateBearerTokenValue;

  assert(ValidateBearerTokenValue("Bearer abc", "abc"));
  assert(!ValidateBearerTokenValue("abc", "abc"));
  assert(!ValidateBearerTokenValue("Bearer wrong", "abc"));
  assert(ValidateBearerTokenValue("anything", ""));

  return 0;
}
