set(MR_PROTO_AVAILABLE OFF CACHE BOOL "Whether protobuf/gRPC codegen is available" FORCE)

find_package(Protobuf QUIET)
find_package(gRPC QUIET)

if(Protobuf_FOUND AND gRPC_FOUND)
  set(MR_PROTO_AVAILABLE ON CACHE BOOL "Whether protobuf/gRPC codegen is available" FORCE)

  set(PROTO_FILE "${CMAKE_CURRENT_SOURCE_DIR}/radio.proto")
  set(PROTO_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
  file(MAKE_DIRECTORY "${PROTO_OUT_DIR}")

  if(TARGET gRPC::grpc_cpp_plugin)
    set(GRPC_CPP_PLUGIN_EXECUTABLE "$<TARGET_FILE:gRPC::grpc_cpp_plugin>")
  else()
    find_program(GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
  endif()
  if(NOT GRPC_CPP_PLUGIN_EXECUTABLE)
    message(WARNING "gRPC cpp plugin not found; disabling proto target.")
    set(MR_PROTO_AVAILABLE OFF CACHE BOOL "Whether protobuf/gRPC codegen is available" FORCE)
    add_library(multi_radio_proto INTERFACE)
    add_library(multi_radio::proto ALIAS multi_radio_proto)
    return()
  endif()

  add_custom_command(
    OUTPUT
      "${PROTO_OUT_DIR}/radio.pb.cc"
      "${PROTO_OUT_DIR}/radio.pb.h"
      "${PROTO_OUT_DIR}/radio.grpc.pb.cc"
      "${PROTO_OUT_DIR}/radio.grpc.pb.h"
    COMMAND protobuf::protoc
    ARGS
      --proto_path="${CMAKE_CURRENT_SOURCE_DIR}"
      --cpp_out="${PROTO_OUT_DIR}"
      --grpc_out="${PROTO_OUT_DIR}"
      --plugin=protoc-gen-grpc="${GRPC_CPP_PLUGIN_EXECUTABLE}"
      "${PROTO_FILE}"
    DEPENDS "${PROTO_FILE}"
    VERBATIM
  )

  add_library(multi_radio_proto
    "${PROTO_OUT_DIR}/radio.pb.cc"
    "${PROTO_OUT_DIR}/radio.grpc.pb.cc"
  )

  target_include_directories(multi_radio_proto
    PUBLIC
      "${PROTO_OUT_DIR}"
  )

  target_link_libraries(multi_radio_proto
    PUBLIC
      protobuf::libprotobuf
      gRPC::grpc++
  )

  add_library(multi_radio::proto ALIAS multi_radio_proto)
else()
  message(WARNING "Protobuf/gRPC not found. gRPC server/frontend targets that depend on proto will be skipped.")
  add_library(multi_radio_proto INTERFACE)
  add_library(multi_radio::proto ALIAS multi_radio_proto)
endif()
