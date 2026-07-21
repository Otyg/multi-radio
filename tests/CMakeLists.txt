set(CMAKE_AUTOMOC ON)

find_package(Qt6 COMPONENTS Core Network REQUIRED)

add_executable(test_mode_state_machine test_mode_state_machine.cpp)
target_link_libraries(test_mode_state_machine PRIVATE multi_radio::core)
add_test(NAME test_mode_state_machine COMMAND test_mode_state_machine)

add_executable(test_event_bus test_event_bus.cpp)
target_link_libraries(test_event_bus PRIVATE multi_radio::core)
add_test(NAME test_event_bus COMMAND test_event_bus)

add_executable(test_multi_receiver_integration test_multi_receiver_integration.cpp)
target_link_libraries(test_multi_receiver_integration PRIVATE multi_radio::core)
add_test(NAME test_multi_receiver_integration COMMAND test_multi_receiver_integration)

add_executable(test_weather_worker
  test_weather_worker.cpp
  ${PROJECT_SOURCE_DIR}/frontend/src/weather_worker.cpp
  ${PROJECT_SOURCE_DIR}/frontend/include/weather_worker.hpp
)
target_include_directories(test_weather_worker PRIVATE ${PROJECT_SOURCE_DIR}/frontend/include)
target_link_libraries(test_weather_worker PRIVATE Qt6::Core Qt6::Network)
add_test(NAME test_weather_worker COMMAND test_weather_worker)

if(TARGET multi_radio_server_lib)
  add_executable(test_auth test_auth.cpp)
  target_link_libraries(test_auth PRIVATE multi_radio_server_lib)
  add_test(NAME test_auth COMMAND test_auth)
endif()
