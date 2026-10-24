# 主机行为回归入口：CMake 配置后构建，再用 CTest 执行；硬件操作不在此处运行。
cmake_minimum_required(VERSION 3.16)
project(mc02_host_tests C)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(../../cmake/mc02_format_diagnostics.cmake)
mc02_check_format_diagnostics()
# 生产纯数值模块供独立测试和INS／BMI088／HT04调用方共同链接。
add_library(mc02_numeric STATIC ../../modules/algorithm/math_utils.c
  ../../modules/algorithm/vector3.c ../../modules/algorithm/sample_filter.c)
target_include_directories(mc02_numeric PUBLIC ../../modules/algorithm)
target_compile_features(mc02_numeric PUBLIC c_std_11)
target_compile_options(mc02_numeric PRIVATE -fno-fast-math)
if(UNIX)
  target_link_libraries(mc02_numeric PUBLIC m)
endif()
include(../../cmake/mc02_dsp.cmake)
mc02_add_dsp(mc02_dsp_host TRUE)
add_library(mc02_kf STATIC ../../modules/algorithm/kalman_filter.c ../../modules/algorithm/dsp_matrix.c)
target_include_directories(mc02_kf PUBLIC ../../modules/algorithm)
target_link_libraries(mc02_kf PUBLIC mc02_dsp_host)
if(UNIX)
  target_link_libraries(mc02_kf PUBLIC m)
endif()

# 生产 CRC 仅依赖标准 C；协议与参数测试共同使用同一份实现。
add_library(mc02_software_crc STATIC ../../modules/algorithm/crc.c)
target_include_directories(mc02_software_crc PUBLIC ../../modules/algorithm)
add_executable(crc_test crc_test.c)
target_include_directories(crc_test PRIVATE ../../bsp/crash)
if(UNIX)
  find_package(Threads REQUIRED)
  target_link_libraries(crc_test PRIVATE Threads::Threads)
  target_compile_definitions(crc_test PRIVATE CRC_TEST_THREADS=1)
endif()


if(UNIX AND NOT APPLE)
  add_executable(usb_diagnostic_test usb_backend_test.c ../../bsp/usb/usb_transport.c
    ../../bsp/usb/usb_transport_port_stm32.c ../../bsp/usb/usb_diagnostic.c
    ../../USB_DEVICE/App/usbd_cdc_if.c)
  target_include_directories(usb_diagnostic_test BEFORE PRIVATE usb_backend_fakes)
  target_include_directories(usb_diagnostic_test PRIVATE ../../bsp/usb ../../USB_DEVICE/App)
  target_compile_definitions(usb_diagnostic_test PRIVATE MC02_BENCH_USB_DIAGNOSTICS=1
    MC02_PROFILE_BENCH_SAFE=1 MC02_HEATER_ENABLED=0)
  target_compile_options(usb_diagnostic_test PRIVATE -Wall -Wextra -Werror)
  add_test(NAME production_usb_diagnostic_finite_recovery COMMAND usb_diagnostic_test diagnostic)
  set_tests_properties(production_usb_diagnostic_finite_recovery PROPERTIES TIMEOUT 10)

  add_executable(usb_backend_test usb_backend_test.c ../../bsp/usb/usb_transport.c
    ../../bsp/usb/usb_transport_port_stm32.c ../../USB_DEVICE/App/usbd_cdc_if.c)
  target_include_directories(usb_backend_test BEFORE PRIVATE usb_backend_fakes)
  target_include_directories(usb_backend_test PRIVATE ../../bsp/usb ../../USB_DEVICE/App)
  target_compile_options(usb_backend_test PRIVATE -Wall -Wextra -Werror)
  foreach(scenario completion fifo-failure stop rearm-error)
    add_test(NAME production_usb_backend_${scenario} COMMAND usb_backend_test ${scenario})
    set_tests_properties(production_usb_backend_${scenario} PROPERTIES TIMEOUT 10)
  endforeach()

  add_executable(vision_sp_stream_test vision_sp_stream_test.c
    ../../modules/master_machine/vision_protocol.c)
  target_include_directories(vision_sp_stream_test PRIVATE ../../modules/master_machine)
  target_link_libraries(vision_sp_stream_test PRIVATE mc02_software_crc)
  add_test(NAME production_vision_sp_stream_blocks COMMAND vision_sp_stream_test)
  set_tests_properties(production_vision_sp_stream_blocks PROPERTIES TIMEOUT 10)

  add_executable(vision_protocol_v2_test vision_protocol_v2_test.c
    ../../modules/master_machine/vision_protocol_v2.c)
  target_include_directories(vision_protocol_v2_test PRIVATE
    ../../modules/master_machine ../../modules/algorithm)
  target_link_libraries(vision_protocol_v2_test PRIVATE mc02_software_crc m)
  target_compile_options(vision_protocol_v2_test PRIVATE -Wall -Wextra -Werror -fno-fast-math)
  add_test(NAME production_vision_v2_vectors_and_streaming COMMAND vision_protocol_v2_test)
  set_tests_properties(production_vision_v2_vectors_and_streaming PROPERTIES TIMEOUT 10)

  # MasterProcess、消息中心和V2编解码共同运行；仅替换USB、Daemon与毫秒时钟。
  add_executable(vision_v2_link_test vision_v2_link_test.c
    ../../modules/master_machine/master_process.c
    ../../modules/master_machine/vision_protocol.c
    ../../modules/master_machine/vision_protocol_v2.c
    ../../modules/time_sync/time_mapping.c
    ../../application/topics/robot_topics.c
    ../../modules/message_center/message_center.c)
  target_include_directories(vision_v2_link_test BEFORE PRIVATE
    vision_v2_link_fakes crc_protocol_fakes)
  target_include_directories(vision_v2_link_test PRIVATE
    ../../bsp/usb ../../bsp/usart ../../application ../../application/topics ../../modules/master_machine
    ../../modules/message_center ../../modules/referee ../../modules/daemon ../../modules/algorithm
    ../../modules/time_sync)
  target_compile_definitions(vision_v2_link_test PRIVATE VISION_USE_VCP=1
    MC02_VISION_PROTOCOL_V2=1)
  target_compile_options(vision_v2_link_test PRIVATE -Wall -Wextra -Werror
    -fno-fast-math -ffunction-sections -fdata-sections)
  target_link_options(vision_v2_link_test PRIVATE -Wl,--gc-sections)
  target_link_libraries(vision_v2_link_test PRIVATE mc02_software_crc m)
  add_test(NAME production_v2_master_receive_topics_and_reports COMMAND vision_v2_link_test)
  set_tests_properties(production_v2_master_receive_topics_and_reports PROPERTIES TIMEOUT 10)

  # USB核心直接使用生产实现；端点模型与线程只在独立测试进程中存在。
  add_executable(usb_transport_test usb_transport_test.c ../../bsp/usb/usb_transport.c)
  target_include_directories(usb_transport_test PRIVATE ../../bsp/usb)
  target_link_libraries(usb_transport_test PRIVATE Threads::Threads)
  target_compile_options(usb_transport_test PRIVATE -Wall -Wextra -Werror)
  target_link_options(usb_transport_test PRIVATE -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free)
  foreach(scenario arguments queue time-generation sending timeout stop-failure start-error exhaustion retry
      late-completion cancel-resume transfer-error init-failure enumeration wrap transmit-only receive-only stable-gap threads)
    add_test(NAME production_usb_transport_${scenario} COMMAND usb_transport_test ${scenario})
    set_tests_properties(production_usb_transport_${scenario} PROPERTIES TIMEOUT 10)
  endforeach()

  add_executable(lcd_diag_state_test lcd_diag_state_test.c)
  target_include_directories(lcd_diag_state_test BEFORE PRIVATE lcd_spi_fakes)
  target_include_directories(lcd_diag_state_test PRIVATE ../../modules/lcd)
  add_test(NAME production_lcd_diagnostic_failure_releases_ui COMMAND lcd_diag_state_test)
  set_tests_properties(production_lcd_diagnostic_failure_releases_ui PROPERTIES TIMEOUT 10)

  add_executable(lcd_spi_timing_test lcd_spi_timing_test.c ../../Core/Src/spi.c ../../modules/lcd/lcd.c)
  target_include_directories(lcd_spi_timing_test BEFORE PRIVATE lcd_spi_fakes)
  target_include_directories(lcd_spi_timing_test PRIVATE ../../modules/lcd)
  target_compile_options(lcd_spi_timing_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(lcd_spi_timing_test PRIVATE -Wl,--gc-sections)
  add_test(NAME production_lcd_rising_edge_timing COMMAND lcd_spi_timing_test)
  add_test(NAME production_lcd_select_clock_boundary COMMAND lcd_spi_timing_test --select-boundary)
  add_executable(lcd_bounds_test lcd_bounds_test.c ../../modules/lcd/lcd.c
    ../../modules/lcd/lcd_buf.c ../../modules/lcd/lcd_font_cn.c ../../modules/lcd/lcd_ui.c
    ../../modules/lcd/lcd_referee_rows.c)
  target_include_directories(lcd_bounds_test BEFORE PRIVATE lcd_spi_fakes)
  target_include_directories(lcd_bounds_test BEFORE PRIVATE lcd_ui_fakes)
  target_include_directories(lcd_bounds_test PRIVATE ../../modules/lcd ../../Core/Inc ../../application
    ../../modules/safety ../../modules/algorithm ../../modules/imu ../../modules/remote
    ../../modules/master_machine ../../modules/referee ../../modules/task_monitor ../../modules/BMI088
    ../../modules/message_center ../../bsp/adc ../../bsp/watchdog ../../bsp/can ../../bsp/crash ../../bsp/usart
    ../../Drivers/STM32H7xx_HAL_Driver/Inc ../../Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ../../Drivers/CMSIS/Include ../../Middlewares/Third_Party/CMSIS-DSP-1.7.0/Include
    ../../Middlewares/Third_Party/FreeRTOS/Source/include
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS)
  target_compile_options(lcd_bounds_test PRIVATE -Wall -Wextra -Werror -fno-fast-math)
  target_compile_options(lcd_bounds_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(lcd_bounds_test PRIVATE -Wl,--gc-sections)
  target_link_libraries(lcd_bounds_test PRIVATE m)
  foreach(scenario boundaries invalid numbers send-failure deadline-stop timeout-context ui-status ui-last-send-failure ui-cache-recovery)
    add_test(NAME production_lcd_${scenario} COMMAND lcd_bounds_test ${scenario})
    set_tests_properties(production_lcd_${scenario} PROPERTIES TIMEOUT 10)
  endforeach()
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

add_executable(time_mapping_test time_mapping_test.c
  ../../modules/time_sync/time_mapping.c)
target_include_directories(time_mapping_test PRIVATE ../../modules/time_sync)
target_compile_options(time_mapping_test PRIVATE -Wall -Wextra -Werror -fno-fast-math)
add_test(NAME production_time_mapping_wrap_expiry_and_recovery COMMAND time_mapping_test)
set_tests_properties(production_time_mapping_wrap_expiry_and_recovery PROPERTIES TIMEOUT 10)
# 三位小数入口独立于日志硬件；Python Decimal提供不同实现的数值参考。
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_executable(
  bsp_adc_snapshot_test
  bsp_adc_snapshot_test.c
  ../../bsp/adc/bsp_adc.c)
target_include_directories(bsp_adc_snapshot_test BEFORE PRIVATE bsp_adc_fakes)
target_include_directories(bsp_adc_snapshot_test PRIVATE ../../bsp/adc)
add_test(NAME production_adc_snapshot_age_error_and_diagnostics COMMAND bsp_adc_snapshot_test)
set_tests_properties(production_adc_snapshot_age_error_and_diagnostics PROPERTIES TIMEOUT 10)

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
set_tests_properties(production_adc_rejects_invalid_voltage_config PROPERTIES TIMEOUT 10)

add_test(NAME production_adc_vin_measurement_tool
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../tools/test_adc_vin_measurement.py)
set_tests_properties(production_adc_vin_measurement_tool PROPERTIES TIMEOUT 10)

add_executable(float_to_string_test float_to_string_test.c ../../bsp/log/float_to_string.c)
target_include_directories(float_to_string_test PRIVATE ../../bsp/log)
target_compile_features(float_to_string_test PRIVATE c_std_11)
target_compile_options(float_to_string_test PRIVATE -Wall -Wextra -Werror -fno-fast-math)
if(UNIX)
  find_package(Threads REQUIRED)
  target_link_libraries(float_to_string_test PRIVATE m Threads::Threads)
  target_compile_definitions(float_to_string_test PRIVATE FLOAT2STR_TEST_THREADS=1)
endif()
foreach(scenario IN ITEMS values failures instances)
  add_test(NAME production_float_to_string_${scenario} COMMAND float_to_string_test ${scenario})
  set_tests_properties(production_float_to_string_${scenario} PROPERTIES TIMEOUT 10)
endforeach()
add_test(NAME production_float_to_string_decimal_reference
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/float_to_string_reference.py
    $<TARGET_FILE:float_to_string_test>)
set_tests_properties(production_float_to_string_decimal_reference PROPERTIES TIMEOUT 10)

# 两个独立调用源文件共同链接生产启动实现，防止头文件再次带入函数体。
add_executable(bsp_init_link_test bsp_init_link_test.c bsp_init_caller_a.c
  bsp_init_caller_b.c ../../bsp/bsp_init.c)
target_include_directories(bsp_init_link_test BEFORE PRIVATE bsp_init_fakes)
target_include_directories(bsp_init_link_test PRIVATE ../../bsp)
target_compile_features(bsp_init_link_test PRIVATE c_std_11)
target_compile_options(bsp_init_link_test PRIVATE -Wall -Wextra -Werror)
foreach(caller IN ITEMS a b)
  add_test(NAME production_bsp_init_link_${caller} COMMAND bsp_init_link_test ${caller})
  set_tests_properties(production_bsp_init_link_${caller} PROPERTIES TIMEOUT 10)
endforeach()
# 生产 LK、PID 和 Daemon 共同链接；仅替换 CAN 与硬件时基，检查资源回滚及三总线路由。
add_executable(lk_motor_registration_test lk_motor_registration_test.c
  ../../modules/motor/LKmotor/LK9025.c ../../modules/motor/motor_feedback.c
  ../../modules/motor/motor_pid.c
  ../../modules/algorithm/controller.c ../../modules/daemon/daemon.c)
target_include_directories(lk_motor_registration_test BEFORE PRIVATE lk_motor_fakes)
target_include_directories(lk_motor_registration_test PRIVATE ../../modules/motor/LKmotor
  ../../modules/motor ../../modules ../../modules/algorithm ../../modules/daemon)
target_compile_options(lk_motor_registration_test PRIVATE -Wall -Wextra -Werror)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(lk_motor_registration_test PRIVATE -fno-builtin-malloc -fno-builtin-free)
endif()
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR
   (CMAKE_C_COMPILER_ID MATCHES "Clang" AND NOT APPLE))
  target_compile_definitions(lk_motor_registration_test PRIVATE LK_TEST_ALLOC_WRAP=1)
  target_link_options(lk_motor_registration_test PRIVATE -Wl,--wrap=malloc -Wl,--wrap=free)
endif()
foreach(scenario IN ITEMS registration can_failure sparse routing)
  add_test(NAME production_lk_motor_${scenario} COMMAND lk_motor_registration_test ${scenario})
  set_tests_properties(production_lk_motor_${scenario} PROPERTIES TIMEOUT 10)
endforeach()

# 这组回归链接生产 BSP CAN 注册与发送实现，仅替换 HAL、时间和日志。
add_executable(lk_motor_bsp_can_test lk_motor_bsp_can_test.c
  ../../modules/motor/LKmotor/LK9025.c ../../modules/motor/motor_feedback.c
  ../../modules/motor/motor_pid.c
  ../../modules/motor/DJImotor/dji_motor.c
  ../../modules/algorithm/controller.c ../../modules/daemon/daemon.c ../../bsp/can/bsp_can.c)
target_include_directories(lk_motor_bsp_can_test BEFORE PRIVATE ../../bsp/can lk_motor_can_fakes
  lk_motor_fakes . ../../modules/motor/LKmotor ../../modules/motor ../../modules
  ../../modules/motor/DJImotor ../../modules/algorithm ../../modules/daemon ../../modules/message_center)
target_compile_options(lk_motor_bsp_can_test PRIVATE -Wall -Wextra -Werror
  -Wno-type-limits -ffunction-sections -fdata-sections -fno-builtin-malloc -fno-builtin-free)
target_link_libraries(lk_motor_bsp_can_test PRIVATE mc02_numeric)
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR
   (CMAKE_C_COMPILER_ID MATCHES "Clang" AND NOT APPLE))
  target_compile_definitions(lk_motor_bsp_can_test PRIVATE LK_TEST_ALLOC_WRAP=1)
  target_link_options(lk_motor_bsp_can_test PRIVATE -Wl,--wrap=malloc -Wl,--wrap=free -Wl,--gc-sections)
endif()
add_test(NAME production_lk_motor_bsp_can_rollback COMMAND lk_motor_bsp_can_test rollback)
add_test(NAME production_lk_motor_bsp_can_routing COMMAND lk_motor_bsp_can_test routing)
add_test(NAME production_lk_motor_bsp_can_closed_loops COMMAND lk_motor_bsp_can_test closed_loops)
add_test(NAME production_lk_motor_bsp_can_feedforward COMMAND lk_motor_bsp_can_test feedforward)
add_test(NAME production_lk_motor_bsp_can_feedback_failure COMMAND lk_motor_bsp_can_test feedback_failure)
add_test(NAME production_lk_motor_bsp_can_feedforward_overflow COMMAND lk_motor_bsp_can_test feedforward_overflow)
foreach(direction_case IN ITEMS angle speed current open invalid)
  add_test(NAME production_lk_motor_bsp_can_direction_${direction_case}
    COMMAND lk_motor_bsp_can_test direction-${direction_case})
  set_tests_properties(production_lk_motor_bsp_can_direction_${direction_case} PROPERTIES TIMEOUT 10)
endforeach()
set_tests_properties(production_lk_motor_bsp_can_rollback production_lk_motor_bsp_can_routing
  production_lk_motor_bsp_can_closed_loops production_lk_motor_bsp_can_feedforward
  production_lk_motor_bsp_can_feedback_failure production_lk_motor_bsp_can_feedforward_overflow
  PROPERTIES TIMEOUT 10)
add_test(NAME production_can_full_queue_burst COMMAND lk_motor_bsp_can_test rx-burst)
set_tests_properties(production_can_full_queue_burst PROPERTIES TIMEOUT 10)
add_test(NAME production_dji_parser_full_queue_burst COMMAND lk_motor_bsp_can_test dji-rx-burst)
set_tests_properties(production_dji_parser_full_queue_burst PROPERTIES TIMEOUT 10)

# DM 测试链接生产电机、FDCAN 注册和 Daemon；测试仅替换 HAL、RTOS、时间与日志。
add_executable(dm_motor_test dm_motor_test.c ../../modules/motor/DMmotor/dmmotor.c
  ../../bsp/can/bsp_can.c ../../modules/daemon/daemon.c)
target_include_directories(dm_motor_test BEFORE PRIVATE dm_motor_fakes lk_motor_can_fakes .)
target_include_directories(dm_motor_test PRIVATE ../../modules/motor/DMmotor ../../modules/motor
  ../../modules/algorithm ../../modules/daemon ../../modules/alarm ../../bsp/can
  ../../modules/message_center ../../modules)
# 生产 CAN 中已有一个无符号值与0比较的告警；只在此集成目标内屏蔽该既有告警。
target_compile_options(dm_motor_test PRIVATE -Wall -Wextra -Werror -Wno-type-limits
  -ffunction-sections -fdata-sections -fno-builtin-malloc -fno-builtin-free)
target_link_options(dm_motor_test PRIVATE -Wl,--gc-sections)
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR
   (CMAKE_C_COMPILER_ID MATCHES "Clang" AND NOT APPLE))
  target_compile_definitions(dm_motor_test PRIVATE DM_TEST_ALLOC_WRAP=1)
  target_link_options(dm_motor_test PRIVATE -Wl,--wrap=malloc -Wl,--wrap=free)
endif()
if(UNIX)
  find_package(Threads REQUIRED)
  target_compile_definitions(dm_motor_test PRIVATE DM_TEST_THREADS=1)
  target_link_libraries(dm_motor_test PRIVATE Threads::Threads)
endif()
foreach(scenario IN ITEMS init config rollback frames reverse edges commands instances quarantine submit-stop resources)
  add_test(NAME production_dm_motor_${scenario} COMMAND dm_motor_test ${scenario})
  set_tests_properties(production_dm_motor_${scenario} PROPERTIES TIMEOUT 10)
endforeach()
if(UNIX)
  add_test(NAME production_dm_motor_threads COMMAND dm_motor_test threads)
  set_tests_properties(production_dm_motor_threads PROPERTIES TIMEOUT 10)
endif()

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

# 标量、向量、平均和多历史分别报告，全部在Release运行且限制执行时间。
add_executable(math_utils_test math_utils_test.c)
target_link_libraries(math_utils_test PRIVATE mc02_numeric)
target_compile_options(math_utils_test PRIVATE -Wall -Wextra -Werror)
if(UNIX)
  find_package(Threads REQUIRED)
  target_link_libraries(math_utils_test PRIVATE Threads::Threads)
  target_compile_definitions(math_utils_test PRIVATE MATH_TEST_THREADS=1)
endif()
if(UNIX AND NOT APPLE)
  target_compile_definitions(math_utils_test PRIVATE MATH_TEST_ALLOC_WRAP=1)
  target_link_options(math_utils_test PRIVATE -Wl,--wrap=malloc -Wl,--wrap=calloc
    -Wl,--wrap=realloc -Wl,--wrap=free)
endif()
foreach(scenario IN ITEMS scalars vectors average instances)
  add_test(NAME production_numeric_${scenario} COMMAND math_utils_test ${scenario})
  set_tests_properties(production_numeric_${scenario} PROPERTIES TIMEOUT 10)
endforeach()
# 四类生产滤波共享公开入口；参考、失败、并发及频率响应分别报告。
add_executable(sample_filter_test sample_filter_test.c)
target_link_libraries(sample_filter_test PRIVATE mc02_numeric)
target_compile_options(sample_filter_test PRIVATE -Wall -Wextra -Werror)
if(UNIX)
  target_link_libraries(sample_filter_test PRIVATE Threads::Threads)
  target_compile_definitions(sample_filter_test PRIVATE FILTER_TEST_THREADS=1)
endif()
if(UNIX AND NOT APPLE)
  target_compile_definitions(sample_filter_test PRIVATE FILTER_TEST_ALLOC_WRAP=1)
  target_link_options(sample_filter_test PRIVATE -Wl,--wrap=malloc -Wl,--wrap=calloc
    -Wl,--wrap=realloc -Wl,--wrap=free)
endif()
foreach(scenario IN ITEMS reference failures instances response)
  add_test(NAME production_sample_filter_${scenario} COMMAND sample_filter_test ${scenario})
  set_tests_properties(production_sample_filter_${scenario} PROPERTIES TIMEOUT 15)
endforeach()
add_executable(motor_pid_integration_test motor_pid_integration_test.c
  ../../modules/motor/DJImotor/dji_motor.c ../../modules/motor/motor_feedback.c
  ../../modules/motor/motor_pid.c
  ../../modules/algorithm/controller.c)
target_include_directories(motor_pid_integration_test BEFORE PRIVATE motor_pid_fakes)
target_include_directories(motor_pid_integration_test PRIVATE ../../modules/motor
  ../../modules/motor/DJImotor ../../modules/algorithm ../../modules/daemon)
if(UNIX)
  target_link_libraries(motor_pid_integration_test PRIVATE m)
endif()
target_link_libraries(motor_pid_integration_test PRIVATE mc02_numeric)
add_test(NAME production_motor_pid_cascade_and_failure COMMAND motor_pid_integration_test)
add_executable(motor_feedback_filter_test motor_feedback_filter_test.c
  ../../modules/motor/DJImotor/dji_motor.c ../../modules/motor/motor_pid.c
  ../../modules/motor/motor_feedback.c
  ../../modules/algorithm/controller.c ../../modules/algorithm/sample_filter.c
  ../../modules/algorithm/math_utils.c)
target_include_directories(motor_feedback_filter_test BEFORE PRIVATE motor_pid_fakes)
target_include_directories(motor_feedback_filter_test PRIVATE ../../modules/motor
  ../../modules/motor/DJImotor ../../modules/algorithm ../../modules/daemon)
if(UNIX)
  target_link_libraries(motor_feedback_filter_test PRIVATE m)
endif()
foreach(mode IN ITEMS legacy time-constant bypass gap missing multi-instance bounds channels config)
  add_test(NAME production_dji_feedback_filter_${mode}
    COMMAND motor_feedback_filter_test ${mode})
  set_tests_properties(production_dji_feedback_filter_${mode} PROPERTIES TIMEOUT 10)
endforeach()
add_executable(motor_feedback_test motor_feedback_test.c ../../modules/motor/motor_feedback.c)
target_include_directories(motor_feedback_test PRIVATE ../../modules/motor)
target_include_directories(motor_feedback_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(motor_feedback_test PRIVATE -Wall -Wextra -Werror)
if(UNIX)
  target_link_libraries(motor_feedback_test PRIVATE m)
endif()
foreach(scenario IN ITEMS conversion capabilities signal)
  add_test(NAME production_motor_feedback_${scenario} COMMAND motor_feedback_test ${scenario})
  set_tests_properties(production_motor_feedback_${scenario} PROPERTIES TIMEOUT 10)
endforeach()
add_executable(host_test_check_test test_check_test.c)
target_include_directories(host_test_check_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME host_test_check_positive_control COMMAND host_test_check_test)
add_test(NAME host_test_check_negative_control
  COMMAND ${CMAKE_COMMAND}
    -DTEST_PROGRAM=$<TARGET_FILE:host_test_check_test>
    -P ${CMAKE_CURRENT_SOURCE_DIR}/expect_check_failure.cmake)

add_executable(quaternion_ekf_inv_sqrt_test quaternion_ekf_inv_sqrt_test.c
  ../../modules/algorithm/QuaternionEKF.c
  )
target_link_libraries(quaternion_ekf_inv_sqrt_test PRIVATE mc02_kf)
target_include_directories(quaternion_ekf_inv_sqrt_test PRIVATE ../../modules/algorithm)
if(UNIX)
  target_compile_options(quaternion_ekf_inv_sqrt_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(quaternion_ekf_inv_sqrt_test PRIVATE -Wl,--gc-sections)
  target_link_libraries(quaternion_ekf_inv_sqrt_test PRIVATE m)
endif()
add_test(NAME production_quaternion_inverse_sqrt_accuracy COMMAND quaternion_ekf_inv_sqrt_test)

add_executable(attitude_filter_test attitude_filter_test.c
  ../../modules/algorithm/attitude_filter.c
  ../../modules/algorithm/QuaternionEKF.c
  ../../Middlewares/Third_Party/Fusion/FusionAhrs.c)
target_include_directories(attitude_filter_test PRIVATE
  ../../modules/algorithm
  ../../Middlewares/Third_Party/Fusion)
target_compile_definitions(attitude_filter_test PRIVATE MC02_ATTITUDE_FILTER_ENABLE_BOTH_BACKENDS=1)
target_link_libraries(attitude_filter_test PRIVATE mc02_kf m)
target_compile_options(attitude_filter_test PRIVATE -Wall -Wextra -Werror -fno-fast-math)
add_test(NAME production_attitude_backends_quality_prediction_and_isolation COMMAND attitude_filter_test)
set_tests_properties(production_attitude_backends_quality_prediction_and_isolation PROPERTIES TIMEOUT 10)

add_executable(message_freshness_test message_freshness_test.c ../../modules/message_center/message_center.c)
target_include_directories(message_freshness_test BEFORE PRIVATE init_resource_fakes)
target_include_directories(message_freshness_test PRIVATE ../../modules/message_center)
add_test(NAME message_freshness_age_and_snapshot_consistency COMMAND message_freshness_test)
set_tests_properties(message_freshness_age_and_snapshot_consistency PROPERTIES TIMEOUT 10)
add_executable(message_center_capacity_test message_center_capacity_test.c
  ../../modules/message_center/message_center.c)
target_include_directories(message_center_capacity_test BEFORE PRIVATE init_resource_fakes)
target_include_directories(message_center_capacity_test PRIVATE ../../modules/message_center)
add_test(NAME production_message_center_payload_storage_capacity COMMAND message_center_capacity_test)
set_tests_properties(production_message_center_payload_storage_capacity PROPERTIES TIMEOUT 10)
if(UNIX AND NOT APPLE)
  find_package(Threads REQUIRED)
  add_executable(message_center_thread_test message_center_thread_test.c
    ../../modules/message_center/message_center.c)
  target_include_directories(message_center_thread_test BEFORE PRIVATE message_center_thread_fakes)
  target_include_directories(message_center_thread_test PRIVATE ../../modules/message_center)
  target_link_libraries(message_center_thread_test PRIVATE Threads::Threads)
  target_compile_options(message_center_thread_test PRIVATE -Wall -Wextra -Werror)
  add_test(NAME production_message_center_cross_thread_snapshot COMMAND message_center_thread_test)
  set_tests_properties(production_message_center_cross_thread_snapshot PROPERTIES TIMEOUT 10)

  add_executable(dwt_timeline_thread_test dwt_timeline_thread_test.c
    ../../bsp/dwt/bsp_dwt.c host_critical_model.c)
  target_include_directories(dwt_timeline_thread_test BEFORE PRIVATE dwt_thread_fakes)
  target_include_directories(dwt_timeline_thread_test PRIVATE ../../bsp/dwt)
  target_link_libraries(dwt_timeline_thread_test PRIVATE Threads::Threads m)
  target_compile_options(dwt_timeline_thread_test PRIVATE -Wall -Wextra -Werror -fno-fast-math)
  add_test(NAME production_dwt_timeline_atomic_snapshot COMMAND dwt_timeline_thread_test)
  set_tests_properties(production_dwt_timeline_atomic_snapshot PROPERTIES TIMEOUT 10)
endif()
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
add_executable(iic_registration_test iic_registration_test.c
  ../../bsp/iic/bsp_iic.c ../../modules/TFminiPlus/tfminiplus.c
  ../../modules/ist8310/ist8310.c)
target_include_directories(iic_registration_test BEFORE PRIVATE iic_registration_fakes)
target_include_directories(iic_registration_test PRIVATE ../../bsp/iic ../../bsp/dwt
  ../../modules/TFminiPlus ../../modules/ist8310)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(iic_registration_test PRIVATE -ffunction-sections -fdata-sections
    -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-free)
endif()
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR
   (CMAKE_C_COMPILER_ID MATCHES "Clang" AND NOT APPLE))
  target_link_options(iic_registration_test PRIVATE -Wl,--gc-sections
    -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free)
endif()
add_test(NAME production_iic_registration_contract COMMAND iic_registration_test register)
add_test(NAME production_iic_registration_capacity COMMAND iic_registration_test capacity)
add_test(NAME production_iic_callers_handle_registration_failure COMMAND iic_registration_test callers)
set_tests_properties(production_iic_registration_contract production_iic_registration_capacity
  production_iic_callers_handle_registration_failure PROPERTIES TIMEOUT 10)
add_executable(init_diagnostic_test init_diagnostic_test.c ../../application/init_diagnostic.c)
target_include_directories(init_diagnostic_test BEFORE PRIVATE init_diagnostic_fakes)
target_include_directories(init_diagnostic_test PRIVATE ../../application)
add_test(NAME production_init_diagnostic_preserves_first_failure COMMAND init_diagnostic_test)
# 旧分配器测试替换为调用者工作区、无分配及失败原值测试。
add_executable(kalman_filter_init_test kalman_filter_init_test.c)
target_link_libraries(kalman_filter_init_test PRIVATE mc02_kf)
add_test(NAME production_kalman_workspace_and_results COMMAND kalman_filter_init_test)
add_executable(dsp_numeric_test dsp_numeric_test.c)
target_link_libraries(dsp_numeric_test PRIVATE mc02_kf)
add_test(NAME production_dsp_matrix_and_trigonometry COMMAND dsp_numeric_test)
add_executable(ins_filter_test ins_filter_test.c ../../modules/imu/ins_task.c
  ../../modules/algorithm/attitude_filter.c ../../modules/algorithm/QuaternionEKF.c
  ../../Middlewares/Third_Party/Fusion/FusionAhrs.c)
target_include_directories(ins_filter_test BEFORE PRIVATE ins_filter_fakes)
target_include_directories(ins_filter_test PRIVATE ../../modules/imu ../../modules/BMI088
  ../../Middlewares/Third_Party/Fusion)
target_compile_definitions(ins_filter_test PRIVATE MC02_VISION_PROTOCOL_V2=1
  MC02_ATTITUDE_FILTER_ENABLE_BOTH_BACKENDS=1 MC02_IMU_BACKEND_FUSION=0)
target_link_libraries(ins_filter_test PRIVATE mc02_kf mc02_numeric)
add_test(NAME production_ins_filter_validity_and_recovery COMMAND ins_filter_test)
add_test(NAME production_ins_numeric_init_failure COMMAND ins_filter_test --math-failure)
foreach(direction IN ITEMS tilt inverted zero short)
  add_test(NAME production_ins_numeric_initial_${direction} COMMAND ins_filter_test --initial-${direction})
  set_tests_properties(production_ins_numeric_initial_${direction} PROPERTIES TIMEOUT 10)
endforeach()
set_tests_properties(production_ins_numeric_init_failure PROPERTIES TIMEOUT 10)
add_executable(ins_filter_fusion_test ins_filter_test.c ../../modules/imu/ins_task.c
  ../../modules/algorithm/attitude_filter.c ../../modules/algorithm/QuaternionEKF.c
  ../../Middlewares/Third_Party/Fusion/FusionAhrs.c)
target_include_directories(ins_filter_fusion_test BEFORE PRIVATE ins_filter_fakes)
target_include_directories(ins_filter_fusion_test PRIVATE ../../modules/imu ../../modules/BMI088
  ../../Middlewares/Third_Party/Fusion)
target_compile_definitions(ins_filter_fusion_test PRIVATE MC02_VISION_PROTOCOL_V2=1
  MC02_ATTITUDE_FILTER_ENABLE_BOTH_BACKENDS=1 MC02_IMU_BACKEND_FUSION=1)
target_link_libraries(ins_filter_fusion_test PRIVATE mc02_kf mc02_numeric)
add_test(NAME production_ins_fusion_backend_validity_and_recovery COMMAND ins_filter_fusion_test)
add_test(NAME production_ins_fusion_backend_numeric_init_failure COMMAND ins_filter_fusion_test --math-failure)
foreach(direction IN ITEMS tilt inverted zero short)
  add_test(NAME production_ins_fusion_backend_initial_${direction} COMMAND ins_filter_fusion_test --initial-${direction})
  set_tests_properties(production_ins_fusion_backend_initial_${direction} PROPERTIES TIMEOUT 10)
endforeach()
set_tests_properties(production_ins_fusion_backend_validity_and_recovery
  production_ins_fusion_backend_numeric_init_failure PROPERTIES TIMEOUT 10)
if(UNIX AND NOT APPLE)
  find_package(Threads REQUIRED)
  target_compile_definitions(ins_filter_test PRIVATE INS_TEST_INIT_WRAP=1)
  target_link_options(ins_filter_test PRIVATE -Wl,--wrap=AttitudeFilterInit)
  add_test(NAME production_ins_filter_init_failure COMMAND ins_filter_test --init-failure)
  target_compile_definitions(ins_filter_fusion_test PRIVATE INS_TEST_INIT_WRAP=1)
  target_link_options(ins_filter_fusion_test PRIVATE -Wl,--wrap=AttitudeFilterInit)
  add_test(NAME production_ins_fusion_backend_init_failure COMMAND ins_filter_fusion_test --init-failure)
  add_executable(kf_matrix_failure_test kf_matrix_failure_test.c)
  target_link_libraries(kf_matrix_failure_test PRIVATE mc02_kf)
  target_link_options(kf_matrix_failure_test PRIVATE -Wl,--wrap=arm_mat_mult_f32
    -Wl,--wrap=arm_mat_add_f32 -Wl,--wrap=arm_mat_sub_f32
    -Wl,--wrap=arm_mat_trans_f32 -Wl,--wrap=arm_mat_inverse_f32)
  add_test(NAME production_kf_matrix_failure_atomicity COMMAND kf_matrix_failure_test)
  target_link_libraries(kalman_filter_init_test PRIVATE Threads::Threads)
  target_compile_definitions(kalman_filter_init_test PRIVATE KF_TEST_THREADS=1 KF_TEST_ALLOC_WRAP=1)
  target_link_options(kalman_filter_init_test PRIVATE -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free)
endif()
add_executable(robot_safety_test robot_safety_test.c ../../modules/safety/robot_safety.c)
target_include_directories(robot_safety_test BEFORE PRIVATE robot_cmd_fakes)
target_include_directories(robot_safety_test PRIVATE ../../application ../../modules/imu
  ../../modules/master_machine ../../modules/safety ../../bsp/usart)
add_test(NAME production_safety_reported_faults COMMAND robot_safety_test)
add_test(NAME production_safety_fault_recovery_priority COMMAND robot_safety_test recovery)
add_test(NAME production_safety_init_failure_sticky COMMAND robot_safety_test init-failure)
if(UNIX AND NOT APPLE)
  find_package(Threads REQUIRED)
  add_executable(robot_safety_thread_test robot_safety_test.c
    ../../modules/safety/robot_safety.c host_critical_model.c)
  target_include_directories(robot_safety_thread_test BEFORE PRIVATE remote_fakes robot_cmd_fakes)
  target_include_directories(robot_safety_thread_test PRIVATE ../../application ../../modules/imu
    ../../modules/master_machine ../../modules/safety ../../bsp/usart)
  target_compile_definitions(robot_safety_thread_test PRIVATE MC02_THREAD_CRITICAL=1 MC02_THREAD_TEST=1)
  target_link_libraries(robot_safety_thread_test PRIVATE Threads::Threads)
  add_test(NAME production_safety_concurrent_snapshot COMMAND robot_safety_thread_test threads)
  set_tests_properties(production_safety_concurrent_snapshot PROPERTIES TIMEOUT 10)
endif()
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
if(UNIX AND NOT APPLE)
  add_executable(remote_control_thread_test remote_control_test.c ../../bsp/usart/bsp_usart.c
    ../../modules/remote/remote_safety.c ../../modules/remote/remote_control.c
    ../../modules/daemon/daemon.c ../../modules/safety/robot_safety.c host_critical_model.c)
  target_include_directories(remote_control_thread_test BEFORE PRIVATE remote_fakes robot_cmd_fakes)
  target_include_directories(remote_control_thread_test PRIVATE ../../bsp/usart ../../modules/remote
    ../../modules/daemon ../../modules/safety ../../modules/imu ../../modules/master_machine
    ../../modules/message_center ../../application)
  target_compile_definitions(remote_control_thread_test PRIVATE
    MC02_THREAD_CRITICAL=1 MC02_REMOTE_THREAD_TEST=1)
  target_link_libraries(remote_control_thread_test PRIVATE Threads::Threads)
  add_test(NAME production_remote_concurrent_snapshot COMMAND remote_control_thread_test threads)
  set_tests_properties(production_remote_concurrent_snapshot PROPERTIES TIMEOUT 10)
endif()
add_executable(usart_snapshot_test usart_snapshot_test.c ../../bsp/usart/bsp_usart.c)
target_include_directories(usart_snapshot_test BEFORE PRIVATE remote_fakes)
target_include_directories(usart_snapshot_test PRIVATE ../../bsp/usart ../../modules/message_center)
if(UNIX AND NOT APPLE)
  target_link_options(usart_snapshot_test PRIVATE -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free)
endif()
add_test(NAME production_uart_immutable_dispatch COMMAND usart_snapshot_test)
set_tests_properties(production_uart_immutable_dispatch PROPERTIES TIMEOUT 10)
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
  ../../application/topics/robot_topics.c
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
  ../../application/topics/robot_topics.c
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
    ../../application/topics/robot_topics.c
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
  ../../application/topics/robot_topics.c
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
  ../../application/topics/robot_topics.c
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
add_executable(robot_cmd_v2_test robot_cmd_v2_test.c ../../application/cmd/robot_cmd.c
  ../../modules/remote/remote_safety.c ../../application/topics/robot_topics.c
  ../../modules/safety/robot_safety.c ../../modules/message_center/message_center.c
  ../../modules/master_machine/vision_control.c)
target_include_directories(robot_cmd_v2_test BEFORE PRIVATE robot_cmd_fakes init_resource_fakes)
target_include_directories(robot_cmd_v2_test PRIVATE . ../../application ../../application/cmd
  ../../application/config ../../modules ../../modules/remote ../../modules/imu
  ../../modules/master_machine ../../modules/message_center ../../modules/referee
  ../../modules/safety ../../bsp/usart)
target_compile_definitions(robot_cmd_v2_test PRIVATE MC02_VISION_PROTOCOL_V2=1
  VISION_LIMITS_CONFIGURED=1 VISION_YAW_CONSTRAINT=VISION_YAW_BOUNDED
  VISION_YAW_MIN_DEG=-180.0f VISION_YAW_MAX_DEG=180.0f
  PITCH_MIN_ANGLE=-30.0f PITCH_MAX_ANGLE=30.0f VISION_PITCH_ZERO_OFFSET_DEG=0.0f
  CHASSIS_MOTION_LIMITS_CONFIGURED=1 CHASSIS_MAX_WZ_DEG_S=360.0f)
target_compile_options(robot_cmd_v2_test PRIVATE -Wall -Wextra -Werror)
if(UNIX)
  target_link_libraries(robot_cmd_v2_test PRIVATE m)
endif()
add_test(NAME production_command_task_v2_independent_topics COMMAND robot_cmd_v2_test)
set_tests_properties(production_command_task_v2_independent_topics PROPERTIES TIMEOUT 10)
add_executable(robot_cmd_dual_test robot_cmd_task_test.c ../../application/cmd/robot_cmd.c
  ../../modules/remote/remote_safety.c ../../modules/safety/robot_safety.c
  ../../application/topics/robot_topics.c
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
  ../../application/shoot/shoot.c ../../application/topics/robot_topics.c
  ../../modules/message_center/message_center.c)
target_include_directories(shoot_freshness_test BEFORE PRIVATE shoot_fakes init_resource_fakes)
target_include_directories(shoot_freshness_test PRIVATE ../../application ../../application/shoot
  ../../modules ../../modules/message_center ../../modules/master_machine ../../modules/imu
  ../../modules/algorithm ../../modules/BMI088
  ../../Middlewares/Third_Party/CMSIS-DSP-1.7.0/Include ../../Drivers/CMSIS/Include
  ../../Drivers/CMSIS/Device/ST/STM32H7xx/Include ../../bsp/usart)
add_test(NAME production_shoot_init_and_expired_command_rearm COMMAND shoot_freshness_test)
if(UNIX AND NOT APPLE)
  foreach(profile IN ITEMS bench_safe robot)
    set(target motor_profile_${profile})
    add_executable(${target} motor_profile_test.c ../../modules/motor/motor_task.c)
    target_include_directories(${target} BEFORE PRIVATE motor_profile_fakes)
    target_include_directories(${target} PRIVATE ../../modules/motor ../../bsp/usb)
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
set_tests_properties(crash_frame_range_and_stacking_errors PROPERTIES TIMEOUT 10)

add_executable(
  bsp_crash_record_test
  bsp_crash_record_test.c
  ../../bsp/crash/bsp_crash_record.c)
target_include_directories(bsp_crash_record_test PRIVATE ../../bsp/crash)
add_test(NAME crash_record_layout_crc_and_legacy_detection COMMAND bsp_crash_record_test)
set_tests_properties(crash_record_layout_crc_and_legacy_detection PROPERTIES TIMEOUT 10)

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
target_include_directories(task_monitor_shutdown_test BEFORE PRIVATE task_monitor_fakes fakes)
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

# 同一生产源码分别验证 Windows 顺序交错和 WSL 真实线程；分配器只在该目标重命名。
add_library(bsp_spi_host_model STATIC ../../bsp/spi/bsp_spi.c)
target_include_directories(bsp_spi_host_model BEFORE PRIVATE bsp_spi_fakes)
target_include_directories(bsp_spi_host_model PRIVATE ../../bsp/spi)
target_compile_definitions(bsp_spi_host_model PRIVATE malloc=mc02_spi_malloc free=mc02_spi_free)
add_executable(bsp_spi_status_test bsp_spi_status_test.c)
target_include_directories(bsp_spi_status_test BEFORE PRIVATE bsp_spi_fakes)
target_include_directories(bsp_spi_status_test PRIVATE ../../bsp/spi)
target_link_libraries(bsp_spi_status_test PRIVATE bsp_spi_host_model)
foreach(spi_case IN ITEMS registration operations invalid ownership wait timeout wrap error isolation false-ready early dirty-start late)
  add_test(NAME production_spi_${spi_case} COMMAND bsp_spi_status_test ${spi_case})
  set_tests_properties(production_spi_${spi_case} PROPERTIES TIMEOUT 10)
endforeach()
if(UNIX AND NOT APPLE)
  find_package(Threads REQUIRED)
  target_compile_definitions(bsp_spi_status_test PRIVATE SPI_TEST_THREADS=1)
  target_link_libraries(bsp_spi_status_test PRIVATE Threads::Threads)
  add_test(NAME production_spi_threads COMMAND bsp_spi_status_test threads)
  set_tests_properties(production_spi_threads PROPERTIES TIMEOUT 10)
endif()

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

# 固定向量、错误入口和多实例分别报告，Release 下同样执行全部检查。
foreach(scenario IN ITEMS vectors failures instances)
  add_test(NAME production_crc_${scenario} COMMAND crc_test ${scenario})
  set_tests_properties(production_crc_${scenario} PROPERTIES TIMEOUT 15)
endforeach()
add_executable(crc_a5_test crc_a5_test.c ../../docs/history/seasky/seasky_protocol.c)
target_include_directories(crc_a5_test PRIVATE ../../docs/history/seasky)
# 归档源码只由历史兼容测试编译，仍执行与自有活动源码相同的格式检查。
target_compile_options(crc_a5_test PRIVATE ${MC02_FORMAT_FLAGS})
add_test(NAME historical_crc_a5_compatibility COMMAND crc_a5_test)
set_tests_properties(historical_crc_a5_compatibility PROPERTIES TIMEOUT 10 LABELS historical)

# 传输替身保留真实协议解析；包装只用于让选定的一次 CRC 调用返回失败。
add_executable(crc_protocol_test crc_protocol_test.c
  ../../bsp/usb/usb_transport.c
  ../../modules/master_machine/master_process.c ../../modules/master_machine/vision_protocol.c
  ../../modules/referee/rm_referee.c ../../modules/referee/referee_UI.c)
target_include_directories(crc_protocol_test BEFORE PRIVATE crc_protocol_fakes can_comm_fakes)
target_include_directories(crc_protocol_test PRIVATE ../../modules/master_machine ../../modules/referee
  ../../modules/message_center ../../bsp/usb)
add_test(NAME production_crc_protocol_failure_and_compatibility COMMAND crc_protocol_test)

# UI使用生产构图、组帧和任务逻辑；只替代传输与时钟，包装用于选定的一次计算失败。
add_executable(referee_ui_test referee_ui_test.c crc_injection.c
  ../../modules/referee/referee_UI.c ../../modules/referee/rm_referee.c
  ../../modules/referee/referee_task.c)
target_include_directories(referee_ui_test BEFORE PRIVATE crc_protocol_fakes can_comm_fakes)
target_include_directories(referee_ui_test PRIVATE ../../modules/referee ../../modules/master_machine
  ../../modules/message_center)
target_link_libraries(referee_ui_test PRIVATE mc02_software_crc m)
target_compile_options(referee_ui_test PRIVATE -Wall -Wextra -Werror)
target_link_options(referee_ui_test PRIVATE -Wl,--wrap=CRCCompute)
if(WIN32)
  target_link_options(referee_ui_test PRIVATE -Wl,--wrap=__mingw_vsnprintf)
else()
  target_link_options(referee_ui_test PRIVATE -Wl,--wrap=vsnprintf)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(referee_ui_test PRIVATE -Wl,--wrap=__vsnprintf_chk)
  endif()
endif()
foreach(case IN ITEMS drawing text frames transport task init-failure)
  add_test(NAME production_referee_ui_${case} COMMAND referee_ui_test ${case})
  set_tests_properties(production_referee_ui_${case} PROPERTIES TIMEOUT 10)
endforeach()
# 只给实际使用 CRC 的测试链接算法，其余测试保持各自依赖。
foreach(target IN ITEMS crc_test crc_a5_test crc_protocol_test can_comm_freshness_test
                        vision_protocol_test param_operation_test)
  if(TARGET ${target})
    target_link_libraries(${target} PRIVATE mc02_software_crc)
  endif()
endforeach()
foreach(target IN ITEMS crc_a5_test crc_protocol_test can_comm_freshness_test param_operation_test)
  if(TARGET ${target})
    target_sources(${target} PRIVATE crc_injection.c)
    target_link_options(${target} PRIVATE -Wl,--wrap=CRCCompute)
  endif()
endforeach()
if(TARGET param_operation_test)
  add_test(NAME production_crc_parameter_legacy_read COMMAND param_operation_test legacy-read)
  add_test(NAME production_crc_parameter_read_failure COMMAND param_operation_test read-failure)
endif()

# RLS 主机回归链接真实 CMSIS-DSP；Linux 额外注入矩阵失败并并发运行独立实例。
add_executable(rls_test rls_test.c ../../modules/algorithm/RLS.c)
target_include_directories(rls_test PRIVATE ../../modules/algorithm)
target_link_libraries(rls_test PRIVATE mc02_dsp_host)
if(UNIX)
  target_link_libraries(rls_test PRIVATE m)
  find_package(Threads REQUIRED)
  target_link_libraries(rls_test PRIVATE Threads::Threads)
  target_compile_definitions(rls_test PRIVATE RLS_TEST_THREADS=1)
endif()
if(UNIX AND NOT APPLE)
  target_compile_definitions(rls_test PRIVATE
    RLS_TEST_MATRIX_WRAP=1 RLS_TEST_ALLOC_WRAP=1)
  target_link_options(rls_test PRIVATE
    -Wl,--wrap=arm_mat_mult_f32 -Wl,--wrap=arm_mat_sub_f32
    -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=realloc -Wl,--wrap=free)
endif()
foreach(scenario IN ITEMS math workspace failures instances)
  add_test(NAME production_rls_${scenario} COMMAND rls_test ${scenario})
  set_tests_properties(production_rls_${scenario} PROPERTIES TIMEOUT 30)
endforeach()

# HT04生产接收回调保留真实平均和PID，只替换CAN、任务及计时。
add_executable(ht_numeric_test ht_numeric_test.c ../../modules/motor/HTmotor/HT04.c
  ../../modules/motor/motor_feedback.c ../../modules/motor/motor_pid.c
  ../../modules/algorithm/controller.c)
target_include_directories(ht_numeric_test BEFORE PRIVATE ht_numeric_fakes motor_pid_fakes)
target_include_directories(ht_numeric_test PRIVATE ../../modules/motor/HTmotor
  ../../modules/motor ../../modules/daemon)
target_link_libraries(ht_numeric_test PRIVATE mc02_numeric)
add_test(NAME production_ht_numeric_feedback COMMAND ht_numeric_test)
set_tests_properties(production_ht_numeric_feedback PROPERTIES TIMEOUT 10)

# HT04 控制回归捕获生产任务入口，并用有限延时轮次检查反馈源和失败输出。
add_executable(ht_control_test ht_control_test.c ../../modules/motor/HTmotor/HT04.c
  ../../modules/motor/motor_feedback.c ../../modules/motor/motor_pid.c
  ../../modules/algorithm/controller.c)
target_include_directories(ht_control_test BEFORE PRIVATE ht_control_fakes ht_numeric_fakes
  lk_motor_fakes motor_pid_fakes)
target_include_directories(ht_control_test PRIVATE ../../modules/motor/HTmotor
  ../../modules/motor ../../modules/daemon .)
target_link_libraries(ht_control_test PRIVATE mc02_numeric)
foreach(scenario IN ITEMS feedback_0 feedback_1 feedback_2 feedback_3
    feedforward_1 feedforward_2 feedforward_3 init_failures runtime_recovery
    runtime_speed_failure feedforward_overflow)
  add_test(NAME production_ht_control_${scenario} COMMAND ht_control_test ${scenario})
  set_tests_properties(production_ht_control_${scenario} PROPERTIES TIMEOUT 10)
endforeach()
foreach(direction_case IN ITEMS angle speed)
  foreach(direction IN ITEMS 0 1)
    add_test(NAME production_ht_control_direction_${direction_case}_${direction}
      COMMAND ht_control_test direction-${direction_case}-${direction})
    set_tests_properties(production_ht_control_direction_${direction_case}_${direction}
      PROPERTIES TIMEOUT 10)
  endforeach()
endforeach()
foreach(scenario IN ITEMS direction-current direction-open direction-invalid)
  add_test(NAME production_ht_control_${scenario} COMMAND ht_control_test ${scenario})
  set_tests_properties(production_ht_control_${scenario} PROPERTIES TIMEOUT 10)
endforeach()

# 模长与分配失败注入仅在支持链接包装的Linux运行，其余路径仍执行真实计算。
if(UNIX AND NOT APPLE)
  add_executable(bmi088_numeric_test bmi088_numeric_test.c bmi088_host_fixture.c
    ../../modules/BMI088/bmi088.c)
  target_include_directories(bmi088_numeric_test BEFORE PRIVATE bmi088_fakes)
  target_include_directories(bmi088_numeric_test PRIVATE ../../modules/BMI088)
  target_link_options(bmi088_numeric_test PRIVATE -Wl,--wrap=Vec3NormF32 -Wl,--wrap=calloc)
  foreach(scenario IN ITEMS once persistent allocation)
    add_test(NAME production_bmi088_numeric_${scenario} COMMAND bmi088_numeric_test ${scenario})
    set_tests_properties(production_bmi088_numeric_${scenario} PROPERTIES TIMEOUT 10)
  endforeach()
endif()
# BMI088只替换传输等硬件，其标定使用真实向量算法。
foreach(target IN ITEMS bmi088_calibration_temperature_test bmi088_init_failure_test
    bmi088_post_reset_dummy_read_test bmi088_init_write_error_test bmi088_sample_status_test
    bmi088_numeric_test)
  if(TARGET ${target})
    target_link_libraries(${target} PRIVATE mc02_numeric)
  endif()
endforeach()

# 真实UART队列及两种流式协议一起运行；只替代HAL、时钟与看门狗。
add_executable(uart_stream_test uart_stream_test.c ../../bsp/usart/bsp_usart.c
  ../../modules/master_machine/master_process.c ../../modules/master_machine/vision_protocol.c)
target_include_directories(uart_stream_test BEFORE PRIVATE remote_fakes ../../bsp/usart
  crc_protocol_fakes can_comm_fakes)
target_include_directories(uart_stream_test PRIVATE ../../modules/master_machine ../../modules/referee
  ../../modules/message_center)
target_compile_definitions(uart_stream_test PRIVATE VISION_USE_UART=1)
target_compile_options(uart_stream_test PRIVATE -Wall -Wextra -Werror)
target_link_options(uart_stream_test PRIVATE -Wl,--wrap=USARTRegister)
target_link_libraries(uart_stream_test PRIVATE mc02_software_crc)
add_test(NAME production_uart_stream_protocols COMMAND uart_stream_test)
add_test(NAME production_uart_pool_limit COMMAND usart_snapshot_test pool)
foreach(case IN ITEMS init-race quarantine slots)
  add_test(NAME production_uart_${case} COMMAND usart_snapshot_test ${case})
  set_tests_properties(production_uart_${case} PROPERTIES TIMEOUT 10)
endforeach()
add_test(NAME production_remote_original_receive_time COMMAND remote_control_test source-time)
set_tests_properties(production_uart_stream_protocols production_uart_pool_limit
  production_remote_original_receive_time PROPERTIES TIMEOUT 10)

# 裁判生产解析器和UART块队列一起运行；Linux线程共享真实互斥临界区模型。
add_executable(referee_snapshot_test referee_snapshot_test.c ../../bsp/usart/bsp_usart.c
  ../../modules/referee/rm_referee.c)
target_include_directories(referee_snapshot_test BEFORE PRIVATE referee_snapshot_fakes remote_fakes
  ../../bsp/usart crc_protocol_fakes)
target_include_directories(referee_snapshot_test PRIVATE ../../modules/referee ../../modules/master_machine
  ../../modules/algorithm ../../modules/daemon ../../modules/message_center)
target_compile_options(referee_snapshot_test PRIVATE -Wall -Wextra -Werror)
target_link_libraries(referee_snapshot_test PRIVATE mc02_software_crc)
if(UNIX)
  target_link_libraries(referee_snapshot_test PRIVATE m)
  find_package(Threads REQUIRED)
  target_link_libraries(referee_snapshot_test PRIVATE Threads::Threads)
  target_compile_definitions(referee_snapshot_test PRIVATE REFEREE_TEST_THREADS=1)
endif()
add_test(NAME production_referee_snapshot COMMAND referee_snapshot_test)
set_tests_properties(production_referee_snapshot PROPERTIES TIMEOUT 30)
add_executable(lcd_referee_rows_test lcd_referee_rows_test.c ../../modules/lcd/lcd_referee_rows.c)
target_include_directories(lcd_referee_rows_test PRIVATE ../../modules/lcd ../../modules/referee)
target_compile_options(lcd_referee_rows_test PRIVATE -Wall -Wextra -Werror)
if(UNIX)
  target_link_libraries(lcd_referee_rows_test PRIVATE m)
endif()
add_test(NAME production_lcd_referee_group_freshness COMMAND lcd_referee_rows_test)
# 生产日志核心只替代平台时钟、临界区、任务创建和整条RTT写入。
add_executable(log_test log_test.c ../../bsp/log/bsp_log.c)
target_include_directories(log_test PRIVATE . ../../bsp/log)
target_compile_definitions(log_test PRIVATE LOG_TEST=1 LOG_TEST_WRAP_FORMAT=1 BSP_LOG_TESTING=1)
target_compile_options(log_test PRIVATE -Wall -Wextra -Werror)
if(UNIX)
  target_link_libraries(log_test PRIVATE m)
  find_package(Threads REQUIRED)
  target_link_libraries(log_test PRIVATE Threads::Threads)
  target_compile_definitions(log_test PRIVATE LOG_TEST_THREADS=1)
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_options(log_test PRIVATE -Wl,--wrap=vsnprintf -Wl,--wrap=snprintf
    -Wl,--wrap=__vsnprintf_chk -Wl,--wrap=__snprintf_chk)
endif()
foreach(case IN ITEMS values history sequence queue catalog task threads)
  if(case STREQUAL "threads" AND NOT UNIX)
    continue()
  endif()
  add_test(NAME production_log_${case} COMMAND log_test ${case})
  set_tests_properties(production_log_${case} PROPERTIES TIMEOUT 10)
endforeach()

add_executable(log_ordinary_test log_test.c ../../bsp/log/bsp_log.c)
target_include_directories(log_ordinary_test PRIVATE . ../../bsp/log)
target_compile_definitions(log_ordinary_test PRIVATE MC02_LOG_ORDINARY=1 BSP_LOG_TESTING=1)
target_compile_options(log_ordinary_test PRIVATE -Wall -Wextra -Werror)
if(UNIX)
  target_link_libraries(log_ordinary_test PRIVATE m)
endif()
add_test(NAME production_log_ordinary_rate COMMAND log_ordinary_test ordinary)
set_tests_properties(production_log_ordinary_rate PROPERTIES TIMEOUT 10)
add_test(NAME production_log_ordinary_priority COMMAND log_ordinary_test fairness)
set_tests_properties(production_log_ordinary_priority PROPERTIES TIMEOUT 10)
add_test(NAME production_log_ordinary_queue COMMAND log_ordinary_test queue)
set_tests_properties(production_log_ordinary_queue PROPERTIES TIMEOUT 10)
add_test(NAME production_log_ordinary_format_overflow COMMAND log_ordinary_test format-overflow)
set_tests_properties(production_log_ordinary_format_overflow PROPERTIES TIMEOUT 10)

add_executable(log_disabled_test log_disabled_test.c)
target_include_directories(log_disabled_test PRIVATE ../../bsp/log .)
target_compile_definitions(log_disabled_test PRIVATE DISABLE_LOG_SYSTEM=1)
target_compile_options(log_disabled_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME production_log_disabled COMMAND log_disabled_test)
set_tests_properties(production_log_disabled PROPERTIES TIMEOUT 10)

add_executable(log_stdio_test log_stdio_test.c ../../bsp/log/bsp_log_stdio.c)
target_compile_options(log_stdio_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME production_log_stdio COMMAND log_stdio_test)
set_tests_properties(production_log_stdio PROPERTIES TIMEOUT 10)
foreach(optimization IN ITEMS O0 O2)
  foreach(group IN ITEMS legacy ui_oled macros length)
    add_test(NAME production_format_diagnostics_${optimization}_${group}
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/workflow/check_format_diagnostics.py
        --compiler ${CMAKE_C_COMPILER} --optimization=-${optimization} --group ${group}
        --output ${CMAKE_BINARY_DIR}/format-tests-${optimization}-${group})
    set_tests_properties(production_format_diagnostics_${optimization}_${group} PROPERTIES TIMEOUT 10)
  endforeach()
endforeach()
add_test(NAME production_format_compile_commands
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/workflow/check_format_diagnostics.py
    --compile-commands ${CMAKE_BINARY_DIR}/compile_commands.json)
set_tests_properties(production_format_compile_commands PROPERTIES TIMEOUT 10)
# UART队列专项链接生产注册及队列；RX回归只替换硬件接入。
foreach(rx_target remote_control_test remote_control_thread_test usart_snapshot_test remote_bench_test remote_command_integration_test
    pc_vision_integration_test uart_stream_test referee_snapshot_test)
  if(TARGET ${rx_target})
    target_sources(${rx_target} PRIVATE ../../bsp/usart/bsp_usart_tx.c usart_tx_host_port.c)
    target_include_directories(${rx_target} PRIVATE ../../Core/Inc)
  endif()
endforeach()
add_executable(uart_tx_test uart_tx_test.c usart_tx_host_port.c
  ../../bsp/usart/bsp_usart.c ../../bsp/usart/bsp_usart_tx.c)
target_include_directories(uart_tx_test BEFORE PRIVATE remote_fakes)
target_include_directories(uart_tx_test PRIVATE . ../../bsp/usart ../../modules/message_center ../../Core/Inc)
target_compile_definitions(uart_tx_test PRIVATE USART_TX_THREADS=1 USART_TX_ABORT_MODEL=1 USART_TX_LEASE_MODEL=1)
find_package(Threads REQUIRED)
target_link_libraries(uart_tx_test PRIVATE Threads::Threads)
target_compile_options(uart_tx_test PRIVATE -Wall -Wextra -Werror)
foreach(scenario copy capacity ports busy error timeout isolate wrap cancel rx threads)
  add_test(NAME production_uart_tx_${scenario} COMMAND uart_tx_test ${scenario})
  set_tests_properties(production_uart_tx_${scenario} PROPERTIES TIMEOUT 10)
endforeach()

# UI及协议的纯发送类型已独立，所有相关目标显式获得同一声明。
foreach(protocol_target crc_protocol_test referee_ui_test)
  target_include_directories(${protocol_target} PRIVATE ../../bsp/usart)
endforeach()


# 完整H723发送接入：寄存器/IRQ由模型提供，后端条件检查仍运行生产实现。
add_executable(uart_tx_backend_test uart_tx_backend_test.c uart_tx_hal_model.c
  ../../bsp/usart/bsp_usart.c ../../bsp/usart/bsp_usart_tx.c ../../bsp/usart/usart_tx_port.c)
target_include_directories(uart_tx_backend_test BEFORE PRIVATE usart_tx_fakes remote_fakes)
target_include_directories(uart_tx_backend_test PRIVATE . ../../bsp/usart ../../modules/message_center ../../Core/Inc)
target_link_libraries(uart_tx_backend_test PRIVATE Threads::Threads)
target_compile_options(uart_tx_backend_test PRIVATE -Wall -Wextra -Werror)
foreach(scenario arguments guard completion race deadline-race error timeout isolate lost-dmat bounded)
  add_test(NAME production_uart_backend_${scenario} COMMAND uart_tx_backend_test ${scenario})
  set_tests_properties(production_uart_backend_${scenario} PROPERTIES TIMEOUT 10)
endforeach()

# UI状态机实际消费真实队列结果，身份从生产接收分帧和CRC发布。
add_executable(uart_ui_integration_test uart_ui_integration_test.c uart_tx_hal_model.c
  ../../bsp/usart/bsp_usart.c ../../bsp/usart/bsp_usart_tx.c ../../bsp/usart/usart_tx_port.c
  ../../modules/referee/rm_referee.c ../../modules/referee/referee_UI.c ../../modules/referee/referee_task.c)
target_include_directories(uart_ui_integration_test BEFORE PRIVATE usart_tx_fakes ../../bsp/usart
  remote_fakes crc_protocol_fakes can_comm_fakes)
target_include_directories(uart_ui_integration_test PRIVATE . ../../modules/referee ../../modules/message_center
  ../../modules/master_machine ../../Core/Inc)
target_link_libraries(uart_ui_integration_test PRIVATE mc02_software_crc Threads::Threads m)
target_compile_options(uart_ui_integration_test PRIVATE -Wall -Wextra -Werror)
foreach(scenario identity-before-start old-generation cadence failure latest queue-full partial-init reinit-inflight demo)
  add_test(NAME production_uart_ui_${scenario} COMMAND uart_ui_integration_test ${scenario})
  set_tests_properties(production_uart_ui_${scenario} PROPERTIES TIMEOUT 10)
endforeach()

# 任务创建和运行入口均为生产实现；只以有限循环替代主机上的RTOS调度。
add_executable(uart_tx_worker_test uart_tx_worker_test.c uart_tx_hal_model.c
  ../../bsp/usart/bsp_usart.c ../../bsp/usart/bsp_usart_tx.c ../../bsp/usart/usart_tx_port.c
  ../../bsp/usart/usart_tx_task.c)
target_include_directories(uart_tx_worker_test BEFORE PRIVATE usart_tx_fakes remote_fakes)
target_include_directories(uart_tx_worker_test PRIVATE . ../../bsp/usart ../../Core/Inc ../../modules/message_center ../../modules/task_monitor)
target_link_libraries(uart_tx_worker_test PRIVATE Threads::Threads)
target_compile_options(uart_tx_worker_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME production_uart_tx_worker_creation_and_loop COMMAND uart_tx_worker_test)
set_tests_properties(production_uart_tx_worker_creation_and_loop PROPERTIES TIMEOUT 10)

add_executable(uart_tx_callers_test uart_tx_callers_test.c uart_tx_hal_model.c
  ../../bsp/usart/bsp_usart.c ../../bsp/usart/bsp_usart_tx.c ../../bsp/usart/usart_tx_port.c
  ../../modules/motor/servo_motor/servo_motor.c ../../modules/bluetooth/HC05.c
  ../../modules/master_machine/master_process.c ../../modules/master_machine/vision_protocol.c
  ../../modules/master_machine/vision_control.c)
target_include_directories(uart_tx_callers_test BEFORE PRIVATE usart_tx_fakes ../../bsp/usart
  remote_fakes crc_protocol_fakes can_comm_fakes)
target_include_directories(uart_tx_callers_test PRIVATE . ../../Core/Inc ../../modules/motor/servo_motor
  ../../modules/bluetooth ../../modules/master_machine ../../modules/message_center)
target_compile_definitions(uart_tx_callers_test PRIVATE VISION_USE_UART=1)
target_link_libraries(uart_tx_callers_test PRIVATE Threads::Threads mc02_software_crc m)
target_compile_options(uart_tx_callers_test PRIVATE -Wall -Wextra -Werror)
foreach(scenario servo hc05 vision)
  add_test(NAME production_uart_caller_${scenario} COMMAND uart_tx_callers_test ${scenario})
  set_tests_properties(production_uart_caller_${scenario} PROPERTIES TIMEOUT 10)
endforeach()
# 所有HAL模型目标都拦截分配器，启动登记后检查队列及通知没有分配/释放。
foreach(model_target uart_tx_backend_test uart_ui_integration_test uart_tx_worker_test uart_tx_callers_test)
  target_link_options(${model_target} PRIVATE -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free)
endforeach()
# 专用采集工具离线核对CRC、半帧、MCU记录、身份和只读命令范围。
add_test(NAME referee_budget_capture_tools COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../tools/test_referee_budget_capture.py)
set_tests_properties(referee_budget_capture_tools PROPERTIES TIMEOUT 10)
add_test(NAME usb_lifecycle_capture_tools COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../tools/test_usb_lifecycle_capture.py)
set_tests_properties(usb_lifecycle_capture_tools PROPERTIES TIMEOUT 10)
add_test(NAME read_only_live_observation_tool COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/../tools/test_probe_snapshot_live.py)
add_test(NAME crash_record_offline_decoder COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/../tools/test_crash_log.py)
set_tests_properties(crash_record_offline_decoder PROPERTIES TIMEOUT 10)
add_test(NAME crash_test_permit_fixed_elf_and_one_shot_write COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/../tools/test_crash_test_permit.py)
set_tests_properties(crash_test_permit_fixed_elf_and_one_shot_write PROPERTIES TIMEOUT 10)
set_tests_properties(read_only_live_observation_tool PROPERTIES TIMEOUT 10)
add_test(NAME native_rtt_receive_only_tool COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/../tools/test_rtt_monitor.py)
set_tests_properties(native_rtt_receive_only_tool PROPERTIES TIMEOUT 10)
add_test(NAME vision_dual_protocol_fire_disabled_tool COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/../tools/test_vision_sim.py)
set_tests_properties(vision_dual_protocol_fire_disabled_tool PROPERTIES TIMEOUT 10)
add_test(NAME dt7_input_tool COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/../tools/test_dt7_input.py)
set_tests_properties(dt7_input_tool PROPERTIES TIMEOUT 10)
# 使用真实裁判和UART后端，验证两种业务竞争同一份发送额度。
add_executable(referee_budget_test referee_budget_test.c uart_tx_hal_model.c
  ../../bsp/usart/bsp_usart.c ../../bsp/usart/bsp_usart_tx.c ../../bsp/usart/usart_tx_port.c
  ../../modules/referee/rm_referee.c ../../modules/referee/referee_UI.c ../../modules/referee/referee_task.c)
target_include_directories(referee_budget_test BEFORE PRIVATE usart_tx_fakes ../../bsp/usart remote_fakes crc_protocol_fakes can_comm_fakes)
target_include_directories(referee_budget_test PRIVATE . ../../modules/referee ../../modules/message_center ../../modules/master_machine ../../Core/Inc)
target_link_libraries(referee_budget_test PRIVATE mc02_software_crc Threads::Threads m)
target_compile_options(referee_budget_test PRIVATE -Wall -Wextra -Werror)
target_link_options(referee_budget_test PRIVATE -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free)
foreach(scenario arguments fifo threads ui-expire window radar receiver sequence errors wrap-identity cancel-expire busy-isolate)
  add_test(NAME production_referee_budget_${scenario} COMMAND referee_budget_test ${scenario})
  set_tests_properties(production_referee_budget_${scenario} PROPERTIES TIMEOUT 10)
endforeach()
# 裁判接收目标也提供共享发送预算，正常路径始终链接生产计算。
get_property(referee_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(target IN LISTS referee_targets)
  get_target_property(target_sources ${target} SOURCES)
  if("${target_sources}" MATCHES "rm_referee[.]c")
    target_sources(${target} PRIVATE ../../modules/referee/referee_tx_budget.c)
  endif()
endforeach()
# OLED使用生产显存及刷新，仅替换I2C并注入格式化失败；不增加生产显存读取入口。
add_executable(oled_bounds_test oled_bounds_test.c ../../modules/oled/oled.c)
target_include_directories(oled_bounds_test BEFORE PRIVATE oled_fakes)
target_include_directories(oled_bounds_test PRIVATE . ../../modules/oled)
target_compile_options(oled_bounds_test PRIVATE -Wall -Wextra -Werror -fno-fast-math)
target_link_options(oled_bounds_test PRIVATE -Wl,--wrap=vsnprintf -Wl,--wrap=__vsnprintf_chk)
foreach(scenario lines invalid characters strings formatting)
  add_test(NAME production_oled_${scenario} COMMAND oled_bounds_test ${scenario})
  set_tests_properties(production_oled_${scenario} PROPERTIES TIMEOUT 10)
endforeach()
# 目标定义完成后统一施加；DSP厂商源码不在自有目录范围内。
get_property(host_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(target IN LISTS host_targets)
  get_target_property(type ${target} TYPE)
  if(type STREQUAL "EXECUTABLE" OR type STREQUAL "STATIC_LIBRARY")
    mc02_apply_format_diagnostics(${target})
  endif()
endforeach()
