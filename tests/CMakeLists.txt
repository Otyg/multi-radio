add_executable(test_mode_state_machine test_mode_state_machine.cpp)
target_link_libraries(test_mode_state_machine PRIVATE multi_radio::core)
add_test(NAME test_mode_state_machine COMMAND test_mode_state_machine)

add_executable(test_event_bus test_event_bus.cpp)
target_link_libraries(test_event_bus PRIVATE multi_radio::core)
add_test(NAME test_event_bus COMMAND test_event_bus)

add_executable(test_multi_receiver_integration test_multi_receiver_integration.cpp)
target_link_libraries(test_multi_receiver_integration PRIVATE multi_radio::core)
add_test(NAME test_multi_receiver_integration COMMAND test_multi_receiver_integration)

add_executable(test_backend_mode test_backend_mode.cpp)
target_link_libraries(test_backend_mode PRIVATE multi_radio::core)
add_test(NAME test_backend_mode COMMAND test_backend_mode)

add_executable(test_backend_config test_backend_config.cpp)
target_link_libraries(test_backend_config PRIVATE multi_radio::core)
add_test(NAME test_backend_config COMMAND test_backend_config)

if(TARGET multi_radio_server_lib)
  add_executable(test_auth test_auth.cpp)
  target_link_libraries(test_auth PRIVATE multi_radio_server_lib)
  add_test(NAME test_auth COMMAND test_auth)
endif()
