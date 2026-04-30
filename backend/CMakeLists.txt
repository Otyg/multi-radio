find_package(Threads REQUIRED)

add_library(multi_radio_core
  src/types.cpp
  src/event_bus.cpp
  src/jsonl_logger.cpp
  src/plugin_host.cpp
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
    target_link_libraries(multi_radio_server PRIVATE multi_radio_server_lib)
  else()
    message(WARNING "Skipping server build: proto/gRPC unavailable.")
  endif()
endif()
