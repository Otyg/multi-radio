find_package(Qt6 COMPONENTS Widgets Core QUIET)
if(NOT Qt6_FOUND)
  message(WARNING "Skipping iq_analyzer: Qt6 not found.")
  return()
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

add_executable(iq_analyzer
  main.cpp
  iq_analyzer_window.cpp
  iq_analyzer_window.hpp
  waterfall_dialog.cpp
  waterfall_dialog.hpp
  dsp_utils.hpp
)

target_include_directories(iq_analyzer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(iq_analyzer PRIVATE Qt6::Widgets Qt6::Core)
