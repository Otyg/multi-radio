set(MR_PROTO_AVAILABLE OFF CACHE BOOL "Whether protobuf/gRPC codegen is available" FORCE)

find_package(Protobuf QUIET)
find_package(gRPC CONFIG QUIET)

set(MR_GRPC_LINK_TARGET "")
if(gRPC_FOUND AND TARGET gRPC::grpc++)
  set(MR_GRPC_LINK_TARGET gRPC::grpc++)
elseif(NOT CMAKE_CROSSCOMPILING)
  # pkg-config fallback is useful for native Linux builds but is unsafe during
  # cross-compilation (it may pick host libraries instead of target libraries).
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(GRPCPP QUIET IMPORTED_TARGET grpc++)
    if(GRPCPP_FOUND)
      set(MR_GRPC_LINK_TARGET PkgConfig::GRPCPP)
    endif()
  endif()
endif()

if(Protobuf_FOUND AND MR_GRPC_LINK_TARGET)
  set(MR_PROTO_AVAILABLE ON CACHE BOOL "Whether protobuf/gRPC codegen is available" FORCE)

  set(PROTO_FILE "${CMAKE_CURRENT_SOURCE_DIR}/radio.proto")
  set(PROTO_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
  file(MAKE_DIRECTORY "${PROTO_OUT_DIR}")

  # .proto code generation runs at build-time on the host machine.
  # Always resolve host tools from PATH (do not use target executables).
  find_program(MR_PROTOC_EXECUTABLE protoc)
  if(NOT MR_PROTOC_EXECUTABLE)
    message(WARNING "protoc not found in PATH; disabling proto target.")
    set(MR_PROTO_AVAILABLE OFF CACHE BOOL "Whether protobuf/gRPC codegen is available" FORCE)
    add_library(multi_radio_proto INTERFACE)
    add_library(multi_radio::proto ALIAS multi_radio_proto)
    return()
  endif()

  find_program(MR_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
  if(NOT MR_GRPC_CPP_PLUGIN_EXECUTABLE)
    message(WARNING "grpc_cpp_plugin not found in PATH; disabling proto target.")
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
    COMMAND "${MR_PROTOC_EXECUTABLE}"
    ARGS
      --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
      --cpp_out=${PROTO_OUT_DIR}
      --grpc_out=${PROTO_OUT_DIR}
      --plugin=protoc-gen-grpc=${MR_GRPC_CPP_PLUGIN_EXECUTABLE}
      ${PROTO_FILE}
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
      ${MR_GRPC_LINK_TARGET}
  )

  add_library(multi_radio::proto ALIAS multi_radio_proto)
else()
  message(WARNING "Protobuf/gRPC target libraries not found. gRPC server/frontend targets that depend on proto will be skipped.")
  add_library(multi_radio_proto INTERFACE)
  add_library(multi_radio::proto ALIAS multi_radio_proto)
endif()
