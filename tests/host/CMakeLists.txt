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
# PID 本身只依赖标准 C；Linux 用真实线程检查不同实例同时计算。
add_executable(pid_test pid_test.c ../../modules/algorithm/controller.c ../../modules/motor/motor_pid.c)
target_include_directories(pid_test PRIVATE ../../modules/algorithm ../../modules/motor)
if(UNIX)
  target_link_libraries(pid_test PRIVATE m)
  find_package(Threads REQUIRED)
  target_link_libraries(pid_test PRIVATE Threads::Threads)
  target_compile_definitions(pid_test PRIVATE PID_TEST_THREADS=1)
endif()
foreach(scenario IN ITEMS numeric improvements failures blocked instances timing)
  add_test(NAME production_pid_${scenario} COMMAND pid_test ${scenario})
  set_tests_properties(production_pid_${scenario} PROPERTIES TIMEOUT 15)
endforeach()
add_executable(motor_pid_integration_test motor_pid_integration_test.c
  ../../modules/motor/DJImotor/dji_motor.c ../../modules/motor/motor_pid.c
  ../../modules/algorithm/controller.c)
target_include_directories(motor_pid_integration_test BEFORE PRIVATE motor_pid_fakes)
target_include_directories(motor_pid_integration_test PRIVATE ../../modules/motor
  ../../modules/motor/DJImotor ../../modules/algorithm ../../modules/daemon)
if(UNIX)
  target_link_libraries(motor_pid_integration_test PRIVATE m)
endif()
add_test(NAME production_motor_pid_cascade_and_failure COMMAND motor_pid_integration_test)
add_executable(host_test_check_test test_check_test.c)
target_include_directories(host_test_check_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME host_test_check_positive_control COMMAND host_test_check_test)
add_test(NAME host_test_check_negative_control
  COMMAND ${CMAKE_COMMAND}
    -DTEST_PROGRAM=$<TARGET_FILE:host_test_check_test>
    -P ${CMAKE_CURRENT_SOURCE_DIR}/expect_check_failure.cmake)

add_executable(quaternion_ekf_inv_sqrt_test quaternion_ekf_inv_sqrt_test.c
  quaternion_ekf_fakes/arm_math.c
  ../../modules/algorithm/QuaternionEKF.c
  ../../modules/algorithm/kalman_filter.c)
target_include_directories(quaternion_ekf_inv_sqrt_test BEFORE PRIVATE kalman_fakes)
target_include_directories(quaternion_ekf_inv_sqrt_test PRIVATE ../../modules/algorithm)
if(UNIX)
  target_compile_options(quaternion_ekf_inv_sqrt_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(quaternion_ekf_inv_sqrt_test PRIVATE -Wl,--gc-sections)
  target_link_libraries(quaternion_ekf_inv_sqrt_test PRIVATE m)
endif()
add_test(NAME production_quaternion_inverse_sqrt_accuracy COMMAND quaternion_ekf_inv_sqrt_test)

add_executable(message_freshness_test message_freshness_test.c ../../modules/message_center/message_center.c)
target_include_directories(message_freshness_test BEFORE PRIVATE init_resource_fakes)
target_include_directories(message_freshness_test PRIVATE ../../modules/message_center)
add_test(NAME message_freshness_age_and_snapshot_consistency COMMAND message_freshness_test)
add_executable(can_comm_freshness_test can_comm_freshness_test.c ../../modules/can_comm/can_comm.c)
target_include_directories(can_comm_freshness_test BEFORE PRIVATE can_comm_fakes)
target_include_directories(can_comm_freshness_test PRIVATE ../../modules/can_comm ../../modules/message_center)
if(UNIX)
  target_compile_options(can_comm_freshness_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(can_comm_freshness_test PRIVATE -Wl,--gc-sections)
endif()
add_test(NAME can_comm_crc_and_snapshot_expiry COMMAND can_comm_freshness_test)
add_executable(init_resource_test init_resource_test.c
  ../../modules/daemon/daemon.c ../../modules/message_center/message_center.c)
target_include_directories(init_resource_test BEFORE PRIVATE init_resource_fakes)
target_include_directories(init_resource_test PRIVATE ../../bsp/dwt ../../modules/daemon
  ../../modules/message_center ../../modules/alarm)
if(UNIX AND NOT APPLE)
  target_compile_options(init_resource_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(init_resource_test PRIVATE -Wl,--gc-sections
    -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free)
endif()
add_test(NAME production_init_resource_failures COMMAND init_resource_test)
add_executable(init_diagnostic_test init_diagnostic_test.c ../../application/init_diagnostic.c)
target_include_directories(init_diagnostic_test BEFORE PRIVATE init_diagnostic_fakes)
target_include_directories(init_diagnostic_test PRIVATE ../../application)
add_test(NAME production_init_diagnostic_preserves_first_failure COMMAND init_diagnostic_test)
add_executable(kalman_filter_init_test kalman_filter_init_test.c ../../modules/algorithm/kalman_filter.c)
target_include_directories(kalman_filter_init_test BEFORE PRIVATE kalman_fakes)
target_include_directories(kalman_filter_init_test PRIVATE ../../modules/algorithm)
if(UNIX AND NOT APPLE)
  target_compile_options(kalman_filter_init_test PRIVATE -ffunction-sections -fdata-sections -fno-pie)
  target_link_options(kalman_filter_init_test PRIVATE -Wl,--gc-sections -no-pie
    -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free)
endif()
add_test(NAME production_kalman_allocation_failure_rolls_back COMMAND kalman_filter_init_test)
add_executable(robot_safety_test robot_safety_test.c ../../modules/safety/robot_safety.c)
target_include_directories(robot_safety_test BEFORE PRIVATE robot_cmd_fakes)
target_include_directories(robot_safety_test PRIVATE ../../application ../../modules/imu
  ../../modules/master_machine ../../modules/safety ../../bsp/usart)
add_test(NAME production_safety_reported_faults COMMAND robot_safety_test)
add_test(NAME production_safety_fault_recovery_priority COMMAND robot_safety_test recovery)
add_test(NAME production_safety_init_failure_sticky COMMAND robot_safety_test init-failure)
add_executable(remote_control_test remote_control_test.c ../../bsp/usart/bsp_usart.c
  ../../modules/remote/remote_safety.c
  ../../modules/remote/remote_control.c ../../modules/daemon/daemon.c ../../modules/safety/robot_safety.c)
target_include_directories(remote_control_test BEFORE PRIVATE remote_fakes robot_cmd_fakes)
target_include_directories(remote_control_test PRIVATE ../../bsp/usart ../../modules/remote
  ../../modules/daemon ../../modules/safety ../../modules/imu ../../modules/master_machine ../../modules/message_center ../../application)
add_test(NAME production_remote_startup_offline COMMAND remote_control_test)
add_test(NAME production_remote_complete_frame COMMAND remote_control_test length)
add_test(NAME production_remote_fields_and_lease COMMAND remote_control_test fields)
add_test(NAME production_remote_queue_and_key_history COMMAND remote_control_test queue-keys)
add_test(NAME production_remote_shared_gestures COMMAND remote_control_test gestures)
add_test(NAME production_remote_keyboard_ad_mapping COMMAND remote_control_test keyboard-map)
add_test(NAME production_remote_mouse_z_wire_bits COMMAND remote_control_test mouse-z)
add_test(NAME production_pc_zero_heartbeat_denied COMMAND remote_control_test pc-startup)
add_test(NAME production_pc_released_handshake COMMAND remote_control_test pc-handshake)
add_test(NAME production_pc_lease_age_and_tick_wrap COMMAND remote_control_test pc-age)
add_test(NAME production_pc_sequence_and_invalid_frames COMMAND remote_control_test pc-boundaries)
add_executable(usart_snapshot_test usart_snapshot_test.c ../../bsp/usart/bsp_usart.c)
target_include_directories(usart_snapshot_test BEFORE PRIVATE remote_fakes)
target_include_directories(usart_snapshot_test PRIVATE ../../bsp/usart ../../modules/message_center)
if(UNIX AND NOT APPLE)
  target_link_options(usart_snapshot_test PRIVATE -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free)
endif()
add_test(NAME production_uart_immutable_dispatch COMMAND usart_snapshot_test)
add_executable(vcp_queue_freshness_test vcp_queue_freshness_test.c ../../bsp/usb/bsp_usb.c)
target_include_directories(vcp_queue_freshness_test BEFORE PRIVATE vcp_fakes)
target_include_directories(vcp_queue_freshness_test PRIVATE ../../bsp/usb ../../modules/message_center)
add_test(NAME production_vcp_immutable_dispatch_and_expiry COMMAND vcp_queue_freshness_test)
add_executable(remote_bench_test remote_control_test.c ../../bsp/usart/bsp_usart.c
  ../../modules/remote/remote_control.c ../../modules/remote/remote_safety.c
  ../../modules/remote/remote_bench.c ../../modules/daemon/daemon.c ../../modules/safety/robot_safety.c)
target_include_directories(remote_bench_test BEFORE PRIVATE remote_fakes robot_cmd_fakes)
target_include_directories(remote_bench_test PRIVATE ../../bsp/usart ../../modules/remote
  ../../modules/daemon ../../modules/safety ../../modules/imu ../../modules/master_machine ../../modules/message_center ../../application)
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
add_test(NAME production_pc_command_expiry_and_remote_handoff COMMAND remote_command_integration_test pc-command)
add_executable(pc_vision_integration_test remote_command_integration_test.c
  ../../bsp/usart/bsp_usart.c ../../modules/remote/remote_control.c ../../modules/remote/remote_safety.c
  ../../modules/daemon/daemon.c ../../modules/safety/robot_safety.c ../../application/cmd/robot_cmd.c
  ../../modules/message_center/message_center.c ../../modules/master_machine/vision_control.c)
target_include_directories(pc_vision_integration_test BEFORE PRIVATE remote_fakes robot_cmd_fakes)
target_include_directories(pc_vision_integration_test PRIVATE ../../application ../../application/cmd
  ../../modules ../../application/config ../../modules/remote ../../modules/daemon ../../modules/safety
  ../../bsp/usart ../../modules/imu ../../modules/master_machine ../../modules/message_center)
target_compile_definitions(pc_vision_integration_test PRIVATE VISION_LIMITS_CONFIGURED=1
  VISION_YAW_CONSTRAINT=VISION_YAW_BOUNDED VISION_YAW_MIN_DEG=-180.0f VISION_YAW_MAX_DEG=180.0f
  PITCH_MIN_ANGLE=-20.0f PITCH_MAX_ANGLE=30.0f VISION_PITCH_ZERO_OFFSET_DEG=5.0f)
if(UNIX)
  target_link_libraries(pc_vision_integration_test PRIVATE m)
endif()
add_test(NAME production_pc_expiry_preserves_vision COMMAND pc_vision_integration_test pc-vision)
add_executable(vision_limits_test vision_limits_test.c ../../modules/master_machine/vision_control.c)
target_include_directories(vision_limits_test PRIVATE ../../modules/master_machine)
if(UNIX)
  target_link_libraries(vision_limits_test PRIVATE m)
endif()
add_test(NAME vision_mechanical_limits_and_conversion COMMAND vision_limits_test)
add_executable(chassis_motion_test chassis_motion_test.c ../../application/chassis/chassis_motion.c)
target_include_directories(chassis_motion_test BEFORE PRIVATE chassis_fakes)
target_include_directories(chassis_motion_test PRIVATE ../../application/chassis)
if(UNIX)
  target_link_libraries(chassis_motion_test PRIVATE m)
endif()
add_test(NAME production_chassis_motion_limits COMMAND chassis_motion_test)
foreach(chassis_limits_profile IN ITEMS configured unconfigured)
  set(target chassis_task_${chassis_limits_profile}_test)
  add_executable(${target} chassis_task_test.c
    ../../application/chassis/chassis.c ../../application/chassis/chassis_motion.c
    ../../modules/message_center/message_center.c)
  target_include_directories(${target} BEFORE PRIVATE chassis_fakes)
  target_include_directories(${target} PRIVATE ../../application ../../application/chassis
    ../../modules ../../modules/message_center ../../modules/imu ../../modules/master_machine)
  if(UNIX)
    target_link_libraries(${target} PRIVATE m)
  endif()
  if(chassis_limits_profile STREQUAL "configured")
    target_compile_definitions(${target} PRIVATE CHASSIS_TASK_EXPECT_CONFIGURED=1
      CHASSIS_MOTION_LIMITS_CONFIGURED=1 CHASSIS_MAX_WZ_DEG_S=10.0f
      CHASSIS_ROTATE_WZ_DEG_S=8.0f CHASSIS_MAX_MOTOR_SPEED_DEG_S=10000.0f)
  else()
    target_compile_definitions(${target} PRIVATE CHASSIS_TASK_EXPECT_CONFIGURED=0)
  endif()
  target_compile_options(${target} PRIVATE -ffunction-sections -fdata-sections)
  if(UNIX)
    target_link_options(${target} PRIVATE -Wl,--gc-sections)
  endif()
  add_test(NAME production_chassis_task_${chassis_limits_profile} COMMAND ${target})
endforeach()
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
add_executable(robot_cmd_dual_test robot_cmd_task_test.c ../../application/cmd/robot_cmd.c
  ../../modules/remote/remote_safety.c ../../modules/safety/robot_safety.c
  ../../modules/message_center/message_center.c ../../modules/master_machine/vision_control.c)
target_include_directories(robot_cmd_dual_test BEFORE PRIVATE robot_cmd_fakes)
target_include_directories(robot_cmd_dual_test PRIVATE ../../application ../../application/cmd ../../application/config
  ../../modules ../../modules/remote ../../modules/imu ../../modules/master_machine ../../modules/message_center
  ../../modules/safety ../../bsp/usart ../../modules/can_comm)
target_compile_definitions(robot_cmd_dual_test PRIVATE GIMBAL_BOARD=1)
if(UNIX)
  target_link_libraries(robot_cmd_dual_test PRIVATE m)
endif()
add_test(NAME production_dual_board_command_snapshot_path COMMAND robot_cmd_dual_test dual-board)
add_executable(shoot_freshness_test shoot_freshness_test.c
  ../../application/shoot/shoot.c ../../modules/message_center/message_center.c)
target_include_directories(shoot_freshness_test BEFORE PRIVATE shoot_fakes init_resource_fakes)
target_include_directories(shoot_freshness_test PRIVATE ../../application/shoot ../../modules/message_center)
add_test(NAME production_shoot_init_and_expired_command_rearm COMMAND shoot_freshness_test)
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
    ../../modules/imu/imu_heater.c ../../modules/algorithm/controller.c)
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
        pid-timing recovery invalid-temperature imu-age-5 imu-age-6 sequence adc-invalid adc-age-21
        sensor-timeout sensor-timeout-wrap heat-timeout heat-timeout-wrap heat-timeout-at-target
        heat-timeout-restarts-after-zero force-off-reinit long-operation post-operation-sample
        initialized-state)
      add_test(
        NAME production_heater_${heater_profile}_${recovery_scenario}
        COMMAND ${heater_recovery_target} ${recovery_scenario})
    endforeach()
  endforeach()

  # 注入 PID 初始化拒绝，检查失败发生在 PWM 启动之前。
  add_executable(imu_heater_pid_init_failure_test imu_heater_pid_init_failure_test.c
    ../../modules/imu/imu_heater.c ../../modules/algorithm/controller.c)
  target_include_directories(imu_heater_pid_init_failure_test BEFORE PRIVATE imu_heater_recovery_fakes)
  target_include_directories(imu_heater_pid_init_failure_test PRIVATE
    ../../modules/imu ../../modules/BMI088 ../../modules/algorithm)
  target_compile_definitions(imu_heater_pid_init_failure_test PRIVATE MC02_HEATER_ENABLED=1)
  target_link_libraries(imu_heater_pid_init_failure_test PRIVATE m)
  target_link_options(imu_heater_pid_init_failure_test PRIVATE -Wl,--wrap=PIDInit)
  add_test(NAME production_heater_pid_init_failure COMMAND imu_heater_pid_init_failure_test)

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
