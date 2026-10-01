cmake_minimum_required(VERSION 3.16)
project(mc02_host_tests C)

include(CTest)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(
  vision_protocol_test
  vision_protocol_test.c
  ../../modules/master_machine/vision_protocol.c
  ../../modules/master_machine/vision_control.c)
target_include_directories(vision_protocol_test PRIVATE ../../modules/master_machine)
if(UNIX)
  target_link_libraries(vision_protocol_test PRIVATE m)
endif()
add_test(NAME vision_protocol_and_control_safety COMMAND vision_protocol_test)
