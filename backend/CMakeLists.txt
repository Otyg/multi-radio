find_package(Threads REQUIRED)

if(MR_ENABLE_RTLSDR)
  find_library(RTLSDR_LIBRARY NAMES rtlsdr)
  find_path(RTLSDR_INCLUDE_DIR rtl-sdr.h)
  if(RTLSDR_LIBRARY AND RTLSDR_INCLUDE_DIR)
    set(MR_HAS_RTLSDR TRUE)
  else()
    set(MR_HAS_RTLSDR FALSE)
    message(WARNING "librtlsdr not found. Falling back to mock radio device backend.")
  endif()
else()
  set(MR_HAS_RTLSDR FALSE)
endif()

add_library(multi_radio_core
  src/types.cpp
  src/scan_scheduler.cpp
  src/event_bus.cpp
  src/jsonl_logger.cpp
  src/plugin_host.cpp
  src/radio_device_factory.cpp
  src/radio_device_mock.cpp
  src/radio_device_rtlsdr.cpp
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
    dl
)

if(MR_HAS_RTLSDR)
  target_compile_definitions(multi_radio_core PRIVATE MULTI_RADIO_WITH_RTLSDR)
  target_include_directories(multi_radio_core PRIVATE ${RTLSDR_INCLUDE_DIR})
  target_link_libraries(multi_radio_core PRIVATE ${RTLSDR_LIBRARY})
endif()

add_library(multi_radio::core ALIAS multi_radio_core)

add_library(ais_wrapper_plugin SHARED plugins/ais_plugin.cpp)
add_library(adsb_wrapper_plugin SHARED plugins/adsb_plugin.cpp)
add_library(dsc_wrapper_plugin SHARED plugins/dsc_plugin.cpp)

foreach(plugin_target IN ITEMS ais_wrapper_plugin adsb_wrapper_plugin dsc_wrapper_plugin)
  target_include_directories(${plugin_target}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  set_target_properties(${plugin_target}
    PROPERTIES
      PREFIX ""
      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins"
  )
endforeach()

if(MR_BUILD_SERVER)
  if(MR_PROTO_AVAILABLE)
    add_library(multi_radio_server_lib
      src/auth.cpp
      src/server_app.cpp
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
    target_link_libraries(multi_radio_server PRIVATE multi_radio_server_lib)
  else()
    message(WARNING "Skipping server build: proto/gRPC unavailable.")
  endif()
endif()
