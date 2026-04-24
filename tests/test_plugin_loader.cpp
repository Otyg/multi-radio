#include <cassert>
#include <iostream>
#include <string>

#include "multi_radio/plugin_host.hpp"

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: test_plugin_loader <mode> <plugin_dir>\n";
    return 1;
  }

  const std::string mode = argv[1];
  const std::string dir = argv[2];

  multi_radio::PluginHost host(dir);
  std::string error;
  const bool loaded = host.LoadAll(&error);

  if (mode == "good") {
    assert(loaded);
    assert(host.ListPlugins().size() == 1);
    return 0;
  }

  assert(!loaded);
  if (mode == "bad_api") {
    assert(error.find("API mismatch") != std::string::npos);
  } else if (mode == "init_fail") {
    assert(error.find("init failed") != std::string::npos);
  } else if (mode == "missing_symbol") {
    assert(error.find("missing required symbol") != std::string::npos);
  } else {
    std::cerr << "unknown mode: " << mode << "\n";
    return 1;
  }

  return 0;
}
