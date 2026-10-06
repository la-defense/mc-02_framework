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

add_executable(bmi088_temperature_test bmi088_temperature_test.c)
target_include_directories(bmi088_temperature_test PRIVATE ../../modules/BMI088)
add_test(NAME bmi088_temperature_signed_decode COMMAND bmi088_temperature_test)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang" AND UNIX AND NOT APPLE)
  find_package(Threads REQUIRED)
  add_executable(
    hal_tick_suspend_test
    hal_tick_suspend_test.c
    ../../Core/Src/stm32h7xx_hal_timebase_tim.c)
  target_include_directories(hal_tick_suspend_test BEFORE PRIVATE hal_timebase_fakes)
  target_include_directories(hal_tick_suspend_test PRIVATE ../../Core/Inc)
  target_compile_options(hal_tick_suspend_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(hal_tick_suspend_test PRIVATE -Wl,--gc-sections)
  add_test(NAME production_hal_tick_suspend_and_spi_wait_lease COMMAND hal_tick_suspend_test)

  add_executable(
    bsp_spi_status_test
    bsp_spi_status_test.c
    ../../bsp/spi/bsp_spi.c)
  target_include_directories(bsp_spi_status_test BEFORE PRIVATE bsp_spi_fakes)
  target_include_directories(bsp_spi_status_test PRIVATE ../../bsp/spi)
  target_compile_options(bsp_spi_status_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(bsp_spi_status_test PRIVATE -Wl,--gc-sections)
  target_link_libraries(bsp_spi_status_test PRIVATE Threads::Threads)
  add_test(NAME production_spi_status_and_bus_ownership COMMAND bsp_spi_status_test)

  add_executable(
    imu_heater_sample_test
    imu_heater_sample_test.c
    ../../modules/imu/imu_heater.c)
  target_include_directories(imu_heater_sample_test BEFORE PRIVATE imu_heater_fakes)
  target_include_directories(imu_heater_sample_test PRIVATE ../../modules/imu ../../modules/BMI088)
  target_compile_definitions(imu_heater_sample_test PRIVATE MC02_HEATER_ENABLED=1)
  target_link_libraries(imu_heater_sample_test PRIVATE m)
  add_test(NAME production_heater_rejects_stale_imu_sample COMMAND imu_heater_sample_test)

  add_executable(
    param_operation_test
    param_operation_test.c
    ../../bsp/param/bsp_param.c)
  target_include_directories(param_operation_test BEFORE PRIVATE param_operation_fakes)
  target_include_directories(param_operation_test PRIVATE ../../bsp/param)
  target_compile_options(param_operation_test PRIVATE -fno-pie)
  target_link_options(param_operation_test PRIVATE -no-pie)
  add_test(NAME production_parameter_flash_is_guarded COMMAND param_operation_test)

  add_executable(
    bmi088_calibration_temperature_test
    bmi088_calibration_temperature_test.c
    bmi088_host_fixture.c
    ../../modules/BMI088/bmi088.c)
  target_include_directories(
    bmi088_calibration_temperature_test
    BEFORE PRIVATE bmi088_fakes)
  target_include_directories(
    bmi088_calibration_temperature_test
    PRIVATE ../../modules/BMI088)
  target_compile_options(bmi088_calibration_temperature_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(bmi088_calibration_temperature_test PRIVATE -Wl,--gc-sections)
  target_link_libraries(bmi088_calibration_temperature_test PRIVATE m)
  add_test(NAME bmi088_calibration_preserves_celsius COMMAND bmi088_calibration_temperature_test)

  add_executable(
    bmi088_sample_status_test
    bmi088_sample_status_test.c
    bmi088_host_fixture.c
    ../../modules/BMI088/bmi088.c)
  target_include_directories(bmi088_sample_status_test BEFORE PRIVATE bmi088_fakes)
  target_include_directories(bmi088_sample_status_test PRIVATE . ../../modules/BMI088)
  target_compile_options(bmi088_sample_status_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(bmi088_sample_status_test PRIVATE -Wl,--gc-sections)
  target_link_libraries(bmi088_sample_status_test PRIVATE m)
  add_test(NAME bmi088_partial_and_non_finite_samples_are_not_published COMMAND bmi088_sample_status_test)
endif()
