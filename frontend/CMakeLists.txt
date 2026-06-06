if(NOT MR_PROTO_AVAILABLE)
  message(WARNING "Skipping frontend targets: proto/gRPC unavailable.")
  return()
endif()

find_package(Qt6 COMPONENTS Core QUIET)
if(NOT Qt6_FOUND)
  message(WARNING "Skipping frontend targets: Qt6 Core not found.")
  return()
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

add_library(multi_radio_frontend_common
  src/grpc_client.cpp
  include/grpc_client.hpp
  include/radar_types.hpp
)

target_include_directories(multi_radio_frontend_common
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(multi_radio_frontend_common
  PUBLIC
    Qt6::Core
    multi_radio::proto
)

set_target_properties(multi_radio_frontend_common PROPERTIES AUTOMOC ON)

if(MR_BUILD_TUI)
  find_package(Curses QUIET)
  if(CURSES_FOUND)
    add_executable(multi_radio_tui
      src/tui_main.cpp
      src/tui_radar_renderer.cpp
      include/tui_radar_renderer.hpp
      include/tui_radar_types.hpp
    )

    target_include_directories(multi_radio_tui
      PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CURSES_INCLUDE_DIRS}
    )

    target_link_libraries(multi_radio_tui
      PRIVATE
        multi_radio_frontend_common
        ${CURSES_LIBRARIES}
        sqlite3
    )

    set_target_properties(multi_radio_tui PROPERTIES AUTOMOC ON)
  else()
    message(WARNING "Skipping ncurses frontend: Curses not found.")
  endif()
endif()

if(NOT MR_BUILD_FRONTEND)
  return()
endif()

find_package(Qt6 COMPONENTS Widgets Network Concurrent)
message(STATUS "DBG Qt6_FOUND=${Qt6_FOUND}")
message(STATUS "DBG Qt6Widgets_FOUND=${Qt6Widgets_FOUND}")
message(STATUS "DBG Qt6Gui_FOUND=${Qt6Gui_FOUND}")
message(STATUS "DBG Qt6DBus_FOUND=${Qt6DBus_FOUND}")
message(STATUS "DBG Qt6Network_FOUND=${Qt6Network_FOUND}")
message(STATUS "DBG Qt6Concurrent_FOUND=${Qt6Concurrent_FOUND}")
if(TARGET Qt6::Widgets)
  message(STATUS "DBG TARGET Qt6::Widgets exists")
else()
  message(STATUS "DBG TARGET Qt6::Widgets missing")
endif()
if(NOT Qt6Widgets_FOUND OR NOT Qt6Network_FOUND OR NOT Qt6Concurrent_FOUND)
  message(WARNING "Skipping Qt frontend: Qt6 Widgets/Network/Concurrent not found.")
  return()
endif()
find_package(Qt6 COMPONENTS Multimedia QUIET)

add_executable(multi_radio_client
  src/main.cpp
  src/main_window.cpp
  src/signal_visualization_widget.cpp
  src/scan_range_visualization_widget.cpp
  src/radar_map_widget.cpp
  src/visible_objects_widget.cpp
  src/coastline_cache.cpp
  src/coastline_loader.cpp
  include/main_window.hpp
  include/signal_visualization_widget.hpp
  include/scan_range_visualization_widget.hpp
  include/radar_map_widget.hpp
  include/visible_objects_widget.hpp
  include/coastline_cache.hpp
  include/coastline_loader.hpp
)

target_include_directories(multi_radio_client
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(multi_radio_client
  PRIVATE
    Qt6::Widgets
    Qt6::Network
    Qt6::Concurrent
    multi_radio_frontend_common
    sqlite3
)

if(TARGET Qt6::Multimedia)
  target_link_libraries(multi_radio_client PRIVATE Qt6::Multimedia)
  target_compile_definitions(multi_radio_client PRIVATE MR_HAS_QT_MULTIMEDIA=1)
else()
  target_compile_definitions(multi_radio_client PRIVATE MR_HAS_QT_MULTIMEDIA=0)
  message(WARNING "Qt6 Multimedia not found: scan-list audio playback disabled.")
endif()
