cmake_minimum_required(VERSION 3.16)
project(mc02_host_tests C)

if(UNIX AND NOT APPLE)
  add_executable(lcd_spi_timing_test lcd_spi_timing_test.c ../../Core/Src/spi.c ../../modules/lcd/lcd.c)
  target_include_directories(lcd_spi_timing_test BEFORE PRIVATE lcd_spi_fakes)
  target_include_directories(lcd_spi_timing_test PRIVATE ../../modules/lcd)
  target_compile_options(lcd_spi_timing_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(lcd_spi_timing_test PRIVATE -Wl,--gc-sections)
  add_test(NAME production_lcd_rising_edge_timing COMMAND lcd_spi_timing_test)
  add_test(NAME production_lcd_select_clock_boundary COMMAND lcd_spi_timing_test --select-boundary)
  foreach(profile IN ITEMS bench_safe robot_disabled robot_enabled)
    set(target tim_profile_${profile})
    add_executable(${target} tim_profile_test.c ../../Core/Src/tim.c)
    target_include_directories(${target} BEFORE PRIVATE tim_profile_fakes)
    target_compile_options(${target} PRIVATE -ffunction-sections -fdata-sections)
    target_link_options(${target} PRIVATE -Wl,--gc-sections)
    if(profile STREQUAL "robot_enabled")
      target_compile_definitions(${target} PRIVATE MC02_PROFILE_ROBOT=1 MC02_HEATER_ENABLED=1)
    elseif(profile STREQUAL "bench_safe")
      target_compile_definitions(${target} PRIVATE MC02_PROFILE_BENCH_SAFE=1 MC02_HEATER_ENABLED=0)
    else()
      target_compile_definitions(${target} PRIVATE MC02_PROFILE_ROBOT=1 MC02_HEATER_ENABLED=0)
    endif()
  endforeach()
endif()

include(CTest)
add_executable(robot_safety_test robot_safety_test.c ../../modules/safety/robot_safety.c)
target_include_directories(robot_safety_test BEFORE PRIVATE robot_cmd_fakes)
target_include_directories(robot_safety_test PRIVATE ../../application ../../modules/imu
  ../../modules/master_machine ../../modules/safety ../../bsp/usart)
add_test(NAME production_safety_reported_faults COMMAND robot_safety_test)
add_test(NAME production_safety_fault_recovery_priority COMMAND robot_safety_test recovery)
add_executable(remote_control_test remote_control_test.c ../../bsp/usart/bsp_usart.c
  ../../modules/remote/remote_safety.c
  ../../modules/remote/remote_control.c ../../modules/daemon/daemon.c ../../modules/safety/robot_safety.c)
target_include_directories(remote_control_test BEFORE PRIVATE remote_fakes robot_cmd_fakes)
target_include_directories(remote_control_test PRIVATE ../../bsp/usart ../../modules/remote
  ../../modules/daemon ../../modules/safety ../../modules/imu ../../modules/master_machine ../../application)
add_test(NAME production_remote_startup_offline COMMAND remote_control_test)
add_test(NAME production_remote_complete_frame COMMAND remote_control_test length)
add_test(NAME production_remote_fields_and_lease COMMAND remote_control_test fields)
add_test(NAME production_remote_queue_and_key_history COMMAND remote_control_test queue-keys)
add_test(NAME production_remote_shared_gestures COMMAND remote_control_test gestures)
add_test(NAME production_remote_keyboard_ad_mapping COMMAND remote_control_test keyboard-map)
add_executable(usart_snapshot_test usart_snapshot_test.c ../../bsp/usart/bsp_usart.c)
target_include_directories(usart_snapshot_test BEFORE PRIVATE remote_fakes)
target_include_directories(usart_snapshot_test PRIVATE ../../bsp/usart)
add_test(NAME production_uart_immutable_dispatch COMMAND usart_snapshot_test)
add_executable(remote_bench_test remote_control_test.c ../../bsp/usart/bsp_usart.c
  ../../modules/remote/remote_control.c ../../modules/remote/remote_safety.c
  ../../modules/remote/remote_bench.c ../../modules/daemon/daemon.c ../../modules/safety/robot_safety.c)
target_include_directories(remote_bench_test BEFORE PRIVATE remote_fakes robot_cmd_fakes)
target_include_directories(remote_bench_test PRIVATE ../../bsp/usart ../../modules/remote
  ../../modules/daemon ../../modules/safety ../../modules/imu ../../modules/master_machine ../../application)
target_compile_definitions(remote_bench_test PRIVATE MC02_PROFILE_BENCH_SAFE=1 MC02_BENCH_RC_DIAGNOSTICS=1)
add_test(NAME production_remote_bench_never_ready COMMAND remote_bench_test)
add_executable(remote_command_integration_test remote_command_integration_test.c
  ../../bsp/usart/bsp_usart.c ../../modules/remote/remote_control.c ../../modules/remote/remote_safety.c
  ../../modules/daemon/daemon.c ../../modules/safety/robot_safety.c ../../application/cmd/robot_cmd.c
  ../../modules/message_center/message_center.c ../../modules/master_machine/vision_control.c)
target_include_directories(remote_command_integration_test BEFORE PRIVATE remote_fakes robot_cmd_fakes)
target_include_directories(remote_command_integration_test PRIVATE ../../application ../../application/cmd
  ../../modules
  ../../application/config ../../modules/remote ../../modules/daemon ../../modules/safety ../../bsp/usart
  ../../modules/imu ../../modules/master_machine ../../modules/message_center)
if(UNIX)
  target_link_libraries(remote_command_integration_test PRIVATE m)
endif()
add_test(NAME production_remote_to_command_safety COMMAND remote_command_integration_test)
add_executable(vision_limits_test vision_limits_test.c ../../modules/master_machine/vision_control.c)
target_include_directories(vision_limits_test PRIVATE ../../modules/master_machine)
if(UNIX)
  target_link_libraries(vision_limits_test PRIVATE m)
endif()
add_test(NAME vision_mechanical_limits_and_conversion COMMAND vision_limits_test)
add_executable(robot_cmd_task_test robot_cmd_task_test.c ../../application/cmd/robot_cmd.c
  ../../modules/remote/remote_safety.c
  ../../modules/safety/robot_safety.c ../../modules/message_center/message_center.c
  ../../modules/master_machine/vision_control.c)
target_include_directories(robot_cmd_task_test BEFORE PRIVATE robot_cmd_fakes)
target_include_directories(robot_cmd_task_test PRIVATE ../../application ../../application/cmd ../../application/config
  ../../modules ../../modules/remote ../../modules/imu ../../modules/master_machine ../../modules/message_center
  ../../modules/safety ../../bsp/usart)
if(UNIX)
  target_link_libraries(robot_cmd_task_test PRIVATE m)
endif()
add_test(NAME production_command_task_source_revocation COMMAND robot_cmd_task_test)
add_test(NAME production_command_task_fault_revocation COMMAND robot_cmd_task_test faults)
add_test(NAME production_shoot_manual_source_switches COMMAND robot_cmd_task_test shoot-switches)
add_test(NAME production_remote_gesture_tick_zero COMMAND robot_cmd_task_test gesture-zero)
add_executable(robot_cmd_vision_test robot_cmd_task_test.c ../../application/cmd/robot_cmd.c
  ../../modules/remote/remote_safety.c
  ../../modules/safety/robot_safety.c ../../modules/message_center/message_center.c
  ../../modules/master_machine/vision_control.c)
target_include_directories(robot_cmd_vision_test BEFORE PRIVATE robot_cmd_fakes)
target_include_directories(robot_cmd_vision_test PRIVATE ../../application ../../application/cmd ../../application/config
  ../../modules ../../modules/remote ../../modules/imu ../../modules/master_machine ../../modules/message_center
  ../../modules/safety ../../bsp/usart)
# These mechanical values belong to this simulation executable only.
target_compile_definitions(robot_cmd_vision_test PRIVATE VISION_LIMITS_CONFIGURED=1
  VISION_YAW_CONSTRAINT=VISION_YAW_BOUNDED VISION_YAW_MIN_DEG=-180.0f VISION_YAW_MAX_DEG=180.0f
  PITCH_MIN_ANGLE=-20.0f PITCH_MAX_ANGLE=30.0f VISION_PITCH_ZERO_OFFSET_DEG=5.0f)
if(UNIX)
  target_link_libraries(robot_cmd_vision_test PRIVATE m)
endif()
add_test(NAME production_command_task_vision_handoff COMMAND robot_cmd_vision_test)
add_test(NAME production_command_task_vision_fault_recovery COMMAND robot_cmd_vision_test faults)
add_test(NAME production_shoot_vision_source_switches COMMAND robot_cmd_vision_test shoot-switches)
if(UNIX AND NOT APPLE)
  foreach(profile IN ITEMS bench_safe robot)
    set(target motor_profile_${profile})
    add_executable(${target} motor_profile_test.c ../../modules/motor/motor_task.c)
    target_include_directories(${target} BEFORE PRIVATE motor_profile_fakes)
    target_include_directories(${target} PRIVATE ../../modules/motor)
    if(profile STREQUAL "bench_safe")
      target_compile_definitions(${target} PRIVATE MC02_PROFILE_BENCH_SAFE=1)
    else()
      target_compile_definitions(${target} PRIVATE MC02_PROFILE_ROBOT=1)
    endif()
    add_test(NAME production_motor_task_profile_${profile} COMMAND ${target})
  endforeach()
endif()
if(UNIX AND NOT APPLE)
  foreach(profile IN ITEMS bench_safe robot_disabled robot_enabled)
    add_test(NAME production_tim_pin_profile_${profile} COMMAND tim_profile_${profile})
  endforeach()
endif()
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

  foreach(heater_profile IN ITEMS enabled disabled)
    set(heater_recovery_target imu_heater_fault_recovery_${heater_profile})
    add_executable(
      ${heater_recovery_target}
      imu_heater_fault_recovery_test.c
      ../../modules/imu/imu_heater.c
      ../../modules/algorithm/controller.c)
    target_include_directories(${heater_recovery_target} BEFORE PRIVATE imu_heater_recovery_fakes)
    target_include_directories(
      ${heater_recovery_target}
      PRIVATE ../../modules/imu ../../modules/BMI088 ../../modules/algorithm)
    if(heater_profile STREQUAL "enabled")
      target_compile_definitions(${heater_recovery_target} PRIVATE MC02_HEATER_ENABLED=1)
    else()
      target_compile_definitions(${heater_recovery_target} PRIVATE MC02_HEATER_ENABLED=0)
    endif()
    target_link_libraries(${heater_recovery_target} PRIVATE m)
    foreach(recovery_scenario IN ITEMS
        recovery invalid-temperature imu-age-5 imu-age-6 sequence adc-invalid adc-age-21
        sensor-timeout sensor-timeout-wrap heat-timeout heat-timeout-wrap heat-timeout-at-target
        heat-timeout-restarts-after-zero force-off-reinit long-operation post-operation-sample
        initialized-state)
      add_test(
        NAME production_heater_${heater_profile}_${recovery_scenario}
        COMMAND ${heater_recovery_target} ${recovery_scenario})
    endforeach()
  endforeach()

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
    bsp_adc_snapshot_test
    bsp_adc_snapshot_test.c
    ../../bsp/adc/bsp_adc.c)
  target_include_directories(bsp_adc_snapshot_test BEFORE PRIVATE bsp_adc_fakes)
  target_include_directories(bsp_adc_snapshot_test PRIVATE ../../bsp/adc)
  add_test(NAME production_adc_snapshot_age_and_error COMMAND bsp_adc_snapshot_test)

  add_executable(
    bsp_adc_invalid_config_test
    bsp_adc_snapshot_test.c
    ../../bsp/adc/bsp_adc.c)
  target_include_directories(bsp_adc_invalid_config_test BEFORE PRIVATE bsp_adc_fakes)
  target_include_directories(bsp_adc_invalid_config_test PRIVATE ../../bsp/adc)
  target_compile_definitions(bsp_adc_invalid_config_test PRIVATE
    BSP_ADC_EXPECT_INVALID_CONFIG
    BSP_ADC_VREF_MV=0u)
  add_test(NAME production_adc_rejects_invalid_voltage_config COMMAND bsp_adc_invalid_config_test)

  add_executable(
    lcd_key_input_test
    lcd_key_input_test.c
    ../../modules/lcd/lcd_key_input.c)
  target_include_directories(lcd_key_input_test PRIVATE ../../modules/lcd)
  add_test(NAME production_lcd_invalid_sample_cancels_key COMMAND lcd_key_input_test)

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
    bmi088_init_failure_test
    bmi088_init_failure_test.c
    bmi088_host_fixture.c
    ../../modules/BMI088/bmi088.c)
  target_include_directories(bmi088_init_failure_test BEFORE PRIVATE bmi088_fakes)
  target_include_directories(bmi088_init_failure_test PRIVATE . ../../modules/BMI088)
  target_compile_options(bmi088_init_failure_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(bmi088_init_failure_test PRIVATE -Wl,--gc-sections)
  target_link_libraries(bmi088_init_failure_test PRIVATE m)
  add_test(NAME bmi088_startup_failure_diagnostics COMMAND bmi088_init_failure_test)

  add_executable(
    bmi088_post_reset_dummy_read_test
    bmi088_post_reset_dummy_read_test.c
    bmi088_host_fixture.c
    ../../modules/BMI088/bmi088.c)
  target_include_directories(bmi088_post_reset_dummy_read_test BEFORE PRIVATE bmi088_fakes)
  target_include_directories(bmi088_post_reset_dummy_read_test PRIVATE . ../../modules/BMI088)
  target_compile_options(bmi088_post_reset_dummy_read_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(bmi088_post_reset_dummy_read_test PRIVATE -Wl,--gc-sections)
  target_link_libraries(bmi088_post_reset_dummy_read_test PRIVATE m)
  add_test(NAME bmi088_post_reset_dummy_read COMMAND bmi088_post_reset_dummy_read_test)

  add_executable(
    bmi088_init_write_error_test
    bmi088_init_write_error_test.c
    bmi088_host_fixture.c
    ../../modules/BMI088/bmi088.c)
  target_include_directories(bmi088_init_write_error_test BEFORE PRIVATE bmi088_fakes)
  target_include_directories(bmi088_init_write_error_test PRIVATE . ../../modules/BMI088)
  target_compile_options(bmi088_init_write_error_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(bmi088_init_write_error_test PRIVATE -Wl,--gc-sections)
  target_link_libraries(bmi088_init_write_error_test PRIVATE m)
  add_test(NAME bmi088_init_spi_failures_abort_attempt COMMAND bmi088_init_write_error_test)

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
