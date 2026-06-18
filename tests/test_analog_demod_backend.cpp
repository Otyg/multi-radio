#include <cassert>
#include <cstdlib>
#include <string>

#include "multi_radio/analog_demod_backend.hpp"

int main() {
  using namespace multi_radio;

  unsetenv("MR_ANALOG_DEMOD_BACKEND");
  std::string backend_name;
  std::string warning;
  auto backend = CreateAnalogDemodBackend(&backend_name, &warning);
  assert(backend != nullptr);
  assert(backend_name == "liquid");
  assert(warning.empty());
  assert(backend->Name() == "liquid");

  setenv("MR_ANALOG_DEMOD_BACKEND", "rtl_airband", 1);
  backend = CreateAnalogDemodBackend(&backend_name, &warning);
  assert(backend != nullptr);
  assert(backend_name == "liquid");
  assert(!warning.empty());
  assert(warning.find("rtl_airband") != std::string::npos);

  unsetenv("MR_ANALOG_DEMOD_BACKEND");
  return 0;
}
