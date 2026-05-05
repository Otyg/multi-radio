if(NOT MR_PROTO_AVAILABLE)
  message(WARNING "Skipping frontend: proto/gRPC unavailable.")
  return()
endif()

find_package(Qt6 COMPONENTS Widgets Core QUIET)
if(NOT Qt6_FOUND)
  message(WARNING "Skipping frontend: Qt6 not found.")
  return()
endif()
find_package(Qt6 COMPONENTS Multimedia QUIET)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

add_executable(multi_radio_client
  src/main.cpp
  src/main_window.cpp
  src/grpc_client.cpp
  src/signal_visualization_widget.cpp
  src/scan_range_visualization_widget.cpp
  include/main_window.hpp
  include/grpc_client.hpp
  include/signal_visualization_widget.hpp
  include/scan_range_visualization_widget.hpp
)

target_include_directories(multi_radio_client
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(multi_radio_client
  PRIVATE
    Qt6::Widgets
    Qt6::Core
    multi_radio::proto
)

if(TARGET Qt6::Multimedia)
  target_link_libraries(multi_radio_client PRIVATE Qt6::Multimedia)
  target_compile_definitions(multi_radio_client PRIVATE MR_HAS_QT_MULTIMEDIA=1)
else()
  target_compile_definitions(multi_radio_client PRIVATE MR_HAS_QT_MULTIMEDIA=0)
  message(WARNING "Qt6 Multimedia not found: scan-list audio playback disabled.")
endif()
