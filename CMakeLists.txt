cmake_minimum_required(VERSION 3.20)
project(multi_radio VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MR_BUILD_SERVER "Build the thick DSP backend" ON)
option(MR_BUILD_RADIO "Build the thin SDR backend" OFF)
option(MR_BUILD_FRONTEND "Build Qt frontend" ON)
option(MR_BUILD_TUI "Build ncurses frontend" ON)
option(MR_BUILD_TESTS "Build tests" ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_subdirectory(proto)
add_subdirectory(backend)
if(MR_BUILD_SERVER)
  add_subdirectory(plugins)
  add_subdirectory(tools)
endif()

if(MR_BUILD_FRONTEND OR MR_BUILD_TUI)
  add_subdirectory(frontend)
endif()

add_subdirectory(tools/iq_analyzer)

if(MR_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
