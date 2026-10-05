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

add_executable(
  bsp_safety_test
  bsp_safety_test.c
  ../../bsp/safety/bsp_safety.c)
target_include_directories(bsp_safety_test BEFORE PRIVATE fakes)
target_include_directories(bsp_safety_test PRIVATE ../../bsp/safety)
target_compile_definitions(bsp_safety_test PRIVATE MC02_SAFETY_TRACE_TEST)
add_test(NAME production_emergency_output_shutdown COMMAND bsp_safety_test)

add_executable(
  fatal_error_entry_test
  fatal_error_entry_test.c
  ../../bsp/safety/bsp_safety.c
  ../../bsp/safety/bsp_safety_error.c)
target_include_directories(fatal_error_entry_test BEFORE PRIVATE fakes)
target_include_directories(fatal_error_entry_test PRIVATE ../../bsp/safety)
target_compile_definitions(fatal_error_entry_test PRIVATE MC02_HOST_TEST)
add_test(NAME production_fatal_error_entry_shutdown COMMAND fatal_error_entry_test)

add_executable(
  bsp_crash_frame_test
  bsp_crash_frame_test.c
  ../../bsp/crash/bsp_crash_frame.c)
target_include_directories(bsp_crash_frame_test PRIVATE ../../bsp/crash)
add_test(NAME crash_frame_range_and_stacking_errors COMMAND bsp_crash_frame_test)

add_executable(
  crash_guard_test
  crash_guard_test.c
  ../../bsp/crash/bsp_crash_guard.c)
target_include_directories(crash_guard_test BEFORE PRIVATE fakes)
target_include_directories(crash_guard_test PRIVATE ../../bsp/crash)
add_test(NAME crash_reentry_immediate_reset COMMAND crash_guard_test)

add_executable(
  task_monitor_shutdown_test
  task_monitor_shutdown_test.c
  ../../modules/task_monitor/task_monitor.c
  ../../bsp/safety/bsp_safety.c)
target_include_directories(task_monitor_shutdown_test BEFORE PRIVATE fakes)
target_include_directories(task_monitor_shutdown_test PRIVATE ../../modules/task_monitor ../../bsp/safety)
add_test(NAME task_monitor_fault_shutdown_order COMMAND task_monitor_shutdown_test)

add_executable(
  watchdog_debug_safety_test
  watchdog_debug_safety_test.c
  ../../bsp/watchdog/bsp_watchdog.c)
target_include_directories(watchdog_debug_safety_test BEFORE PRIVATE watchdog_fakes fakes)
target_include_directories(watchdog_debug_safety_test PRIVATE ../../bsp/watchdog ../../bsp/dwt)
add_test(NAME watchdog_debug_freeze_and_bounded_wait COMMAND watchdog_debug_safety_test)
set_tests_properties(watchdog_debug_freeze_and_bounded_wait PROPERTIES TIMEOUT 2)
