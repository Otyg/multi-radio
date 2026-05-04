find_package(Threads REQUIRED)
find_package(PkgConfig QUIET)

set(MR_BUILD_GIT_COMMIT "unknown")
execute_process(
  COMMAND git -C ${CMAKE_SOURCE_DIR} rev-parse --short=12 HEAD
  OUTPUT_VARIABLE MR_BUILD_GIT_COMMIT_RAW
  RESULT_VARIABLE MR_BUILD_GIT_COMMIT_RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(MR_BUILD_GIT_COMMIT_RESULT EQUAL 0 AND NOT MR_BUILD_GIT_COMMIT_RAW STREQUAL "")
  set(MR_BUILD_GIT_COMMIT "${MR_BUILD_GIT_COMMIT_RAW}")
endif()

set(MR_RTLSDR_LINK_TARGET "")
if(PkgConfig_FOUND)
  pkg_check_modules(RTLSDR QUIET IMPORTED_TARGET librtlsdr)
  if(RTLSDR_FOUND)
    set(MR_RTLSDR_LINK_TARGET PkgConfig::RTLSDR)
  else()
    pkg_check_modules(RTLSDR_ALT QUIET IMPORTED_TARGET rtl-sdr)
    if(RTLSDR_ALT_FOUND)
      set(MR_RTLSDR_LINK_TARGET PkgConfig::RTLSDR_ALT)
    endif()
  endif()
endif()

set(MR_HAS_LIQUID 0)
set(MR_LIQUID_LINK_TARGET "")
if(PkgConfig_FOUND)
  pkg_check_modules(LIQUID QUIET IMPORTED_TARGET liquid)
  if(LIQUID_FOUND)
    set(MR_HAS_LIQUID 1)
    set(MR_LIQUID_LINK_TARGET PkgConfig::LIQUID)
  endif()
endif()

if(NOT MR_HAS_LIQUID)
  unset(MR_LIQUID_INCLUDE_DIR)
  unset(MR_LIQUID_INCLUDE_DIR CACHE)
  unset(MR_LIQUID_LIBRARY)
  unset(MR_LIQUID_LIBRARY CACHE)
  find_path(MR_LIQUID_INCLUDE_DIR liquid/liquid.h
    PATHS /usr/local/include /usr/include
  )
  find_library(MR_LIQUID_LIBRARY NAMES liquid
    PATHS /usr/local/lib /usr/lib /usr/lib64
  )
  if(MR_LIQUID_INCLUDE_DIR AND MR_LIQUID_LIBRARY)
    set(MR_HAS_LIQUID 1)
  endif()
endif()

add_library(multi_radio_core
  src/types.cpp
  src/event_bus.cpp
  src/jsonl_logger.cpp
  src/plugin_host.cpp
  src/fm_demod.cpp
  src/rtl_sdr_device.cpp
  src/receiver_worker.cpp
  src/receiver_manager.cpp
)

target_include_directories(multi_radio_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(multi_radio_core
  PUBLIC
    Threads::Threads
)

if(MR_RTLSDR_LINK_TARGET)
  target_link_libraries(multi_radio_core PRIVATE ${MR_RTLSDR_LINK_TARGET})
  target_compile_definitions(multi_radio_core PRIVATE MR_HAS_RTLSDR=1)
endif()

if(MR_HAS_LIQUID)
  target_compile_definitions(multi_radio_core PRIVATE MR_HAS_LIQUID=1)
  if(MR_LIQUID_LINK_TARGET)
    target_link_libraries(multi_radio_core PRIVATE ${MR_LIQUID_LINK_TARGET})
  else()
    target_include_directories(multi_radio_core PRIVATE ${MR_LIQUID_INCLUDE_DIR})
    target_link_libraries(multi_radio_core PRIVATE ${MR_LIQUID_LIBRARY})
  endif()
endif()

add_library(multi_radio::core ALIAS multi_radio_core)

if(MR_BUILD_SERVER)
  if(MR_PROTO_AVAILABLE)
    add_library(multi_radio_server_lib
      src/auth.cpp
      src/grpc_services.cpp
    )

    target_include_directories(multi_radio_server_lib
      PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(multi_radio_server_lib
      PUBLIC
        multi_radio::core
        multi_radio::proto
    )

    add_executable(multi_radio_server src/server_main.cpp)
    target_compile_definitions(multi_radio_server PRIVATE MR_BUILD_GIT_COMMIT="${MR_BUILD_GIT_COMMIT}")
    target_link_libraries(multi_radio_server PRIVATE multi_radio_server_lib)
  else()
    message(WARNING "Skipping server build: proto/gRPC unavailable.")
  endif()
endif()
