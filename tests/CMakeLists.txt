add_executable(test_scan_scheduler test_scan_scheduler.cpp)
target_link_libraries(test_scan_scheduler PRIVATE multi_radio::core)
add_test(NAME test_scan_scheduler COMMAND test_scan_scheduler)

add_executable(test_mode_state_machine test_mode_state_machine.cpp)
target_link_libraries(test_mode_state_machine PRIVATE multi_radio::core)
add_test(NAME test_mode_state_machine COMMAND test_mode_state_machine)

add_executable(test_event_bus test_event_bus.cpp)
target_link_libraries(test_event_bus PRIVATE multi_radio::core)
add_test(NAME test_event_bus COMMAND test_event_bus)

add_executable(test_multi_receiver_integration test_multi_receiver_integration.cpp)
target_link_libraries(test_multi_receiver_integration PRIVATE multi_radio::core)
add_test(NAME test_multi_receiver_integration COMMAND test_multi_receiver_integration)

if(TARGET multi_radio_server_lib)
  add_executable(test_auth test_auth.cpp)
  target_link_libraries(test_auth PRIVATE multi_radio_server_lib)
  add_test(NAME test_auth COMMAND test_auth)
endif()

add_library(good_plugin SHARED plugins/good_plugin.cpp)
target_include_directories(good_plugin PRIVATE ${PROJECT_SOURCE_DIR}/backend/include)
set_target_properties(good_plugin PROPERTIES PREFIX "" LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests/good_plugins")

add_library(bad_api_plugin SHARED plugins/bad_api_plugin.cpp)
target_include_directories(bad_api_plugin PRIVATE ${PROJECT_SOURCE_DIR}/backend/include)
set_target_properties(bad_api_plugin PROPERTIES PREFIX "" LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests/bad_api_plugins")

add_library(init_fail_plugin SHARED plugins/init_fail_plugin.cpp)
target_include_directories(init_fail_plugin PRIVATE ${PROJECT_SOURCE_DIR}/backend/include)
set_target_properties(init_fail_plugin PROPERTIES PREFIX "" LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests/init_fail_plugins")

add_library(missing_symbol_plugin SHARED plugins/missing_symbol_plugin.cpp)
target_include_directories(missing_symbol_plugin PRIVATE ${PROJECT_SOURCE_DIR}/backend/include)
set_target_properties(missing_symbol_plugin PROPERTIES PREFIX "" LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests/missing_symbol_plugins")

add_executable(test_plugin_loader test_plugin_loader.cpp)
target_link_libraries(test_plugin_loader PRIVATE multi_radio::core)
add_dependencies(test_plugin_loader good_plugin bad_api_plugin init_fail_plugin missing_symbol_plugin)

add_test(NAME test_plugin_loader_good COMMAND test_plugin_loader good "${CMAKE_BINARY_DIR}/tests/good_plugins")
add_test(NAME test_plugin_loader_bad_api COMMAND test_plugin_loader bad_api "${CMAKE_BINARY_DIR}/tests/bad_api_plugins")
add_test(NAME test_plugin_loader_init_fail COMMAND test_plugin_loader init_fail "${CMAKE_BINARY_DIR}/tests/init_fail_plugins")
add_test(NAME test_plugin_loader_missing_symbol COMMAND test_plugin_loader missing_symbol "${CMAKE_BINARY_DIR}/tests/missing_symbol_plugins")
