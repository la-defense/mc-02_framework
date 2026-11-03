# 固件源码按平台、当前通信依赖和板卡角色登记；新增自有 .c 必须写入对应清单。
# 本文件只列参与固件编译的源码，不控制主机测试和历史归档测试。
set(MC02_PLATFORM_C_SOURCES
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/freertos.c
    Core/Src/adc.c
    Core/Src/dma.c
    Core/Src/fdcan.c
    Core/Src/i2c.c
    Core/Src/octospi.c
    Core/Src/spi.c
    Core/Src/tim.c
    Core/Src/usart.c
    Core/Src/stm32h7xx_it.c
    Core/Src/stm32h7xx_hal_msp.c
    Core/Src/stm32h7xx_hal_timebase_tim.c
    Core/Src/system_stm32h7xx.c
    Core/Src/sysmem.c
    Core/Src/syscalls.c
    USB_DEVICE/App/usb_device.c
    USB_DEVICE/App/usbd_desc.c
    USB_DEVICE/App/usbd_cdc_if.c
    USB_DEVICE/Target/usbd_conf.c
    Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/timers.c
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_ospi.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c)

set(MC02_BSP_C_SOURCES
    bsp/os/platform_os_freertos.c # 任务接口的 FreeRTOS 后端。
    bsp/os/platform_signal_freertos.c # 可合并任务信号位的 FreeRTOS 后端。
    bsp/os/platform_sync_freertos.c # 任务临界区与原有毫秒时钟的后端。
    bsp/os/platform_task_stats_freertos.c # 有界任务运行统计的 FreeRTOS 后端。
    bsp/os/platform_irq_stm32.c # STM32 中断屏蔽状态保存与恢复。
    bsp/board/board_can_stm32.c # MC-02 逻辑 CAN 编号和 FDCAN 控制器的板级绑定。
    bsp/board/board_spi_stm32.c # MC-02 逻辑 SPI 编号与控制器绑定。
    bsp/board/board_gpio_stm32.c # MC-02 逻辑 GPIO 与板载片选绑定。
    bsp/board/platform_pwm_stm32.c # 板载加热 PWM 的逻辑通道适配。
    bsp/board/platform_adc_stm32.c # 电源电压逻辑模拟输入适配。
    bsp/board/platform_can_stm32.c # 逻辑 CAN 总线的发送队列适配。
    bsp/board/platform_can_oneshot_stm32.c # 一次性 CAN 命令的逻辑总线适配。
    bsp/board/platform_spi_blocking_stm32.c # 同步 SPI 状态与阻塞模式检查。
    bsp/board/platform_can_status_stm32.c # 逻辑 CAN 控制器状态查询。
    bsp/board/platform_can_periodic_stm32.c # 逻辑 CAN 周期发送及发送前检查。
    bsp/board/platform_can_rx_stm32.c # 逻辑 CAN 接收端点适配。
    bsp/board/board_numeric_caps_mc02.c # MC-02 数值硬件和当前编译能力声明。
    bsp/board/ins_sensor_mc02.c # INS 与板载 BMI088 的 SPI 绑定。
    bsp/board/platform_probe_stm32.c # 原有 DWT 周期计数与短时探针适配。
    bsp/board/platform_delay_500us_stm32.c # BMI088 标定样本之间的短等待。
    bsp/board/platform_probe_elapsed_stm32.c # 原有短时探针的微秒耗时查询。
    bsp/board/platform_time_us_stm32.c # 保留现有微秒运行时间的框架入口。
    bsp/board/platform_timeline_ms_stm32.c # 保留现有浮点毫秒诊断时间。
    bsp/board/platform_dispatch_stm32.c # 任务派发绑定到现有 CAN 和串口队列。
    bsp/board/platform_maintenance_stm32.c # 周期维护绑定到原有 SPI、ADC 和 CAN 入口。
    bsp/board/platform_can_diagnostic_stm32.c # 显式启用时才连接板级 CAN 回环诊断。
    bsp/board/platform_context_stm32.c # 查询当前线程或异常处理上下文。
    bsp/board/platform_startup_delay_stm32.c # 保留调度器启动前的 DWT 等待。
    bsp/board/platform_safety_stm32.c # 核心故障与看门狗绑定到原有安全路径。
    bsp/os/platform_scheduler_freertos.c # 查询调度器的既有运行状态。
    bsp/board/board_uart_stm32.c # MC-02 逻辑 UART 与物理串口绑定。
    bsp/board/platform_uart_stm32.c # 逻辑 UART 接收注册与重启适配。
    bsp/board/platform_uart_tx_stm32.c # 逻辑 UART 发送队列适配。
    bsp/board/platform_uart_task_stm32.c # UART 发送服务任务创建适配。
    bsp/observation/bsp_observation.c
    bsp/bsp_init.c
    bsp/bsp_tools.c
    bsp/adc/bsp_adc.c
    bsp/can/bsp_can.c
    bsp/crash/bsp_crash.c
    bsp/crash/bsp_crash_frame.c
    bsp/crash/bsp_crash_guard.c
    bsp/crash/bsp_crash_record.c
    bsp/dwt/bsp_dwt.c
    bsp/flash/bsp_flash.c
    bsp/gpio/bsp_gpio.c
    bsp/iic/bsp_iic.c
    bsp/log/bsp_log.c
    bsp/log/bsp_log_port_stm32.c
    bsp/log/bsp_log_stdio.c
    bsp/log/float_to_string.c
    bsp/param/bsp_param.c
    bsp/pwm/bsp_pwm.c
    bsp/reset_recovery/bsp_reset_recovery.c
    bsp/reset_recovery/reset_recovery_policy.c
    bsp/safety/bsp_safety.c
    bsp/safety/bsp_safety_error.c
    bsp/spi/bsp_spi.c
    bsp/usb/usb_transport.c
    bsp/usb/usb_transport_port_stm32.c
    bsp/usart/bsp_usart.c
    bsp/usart/bsp_usart_tx.c
    bsp/usart/usart_tx_port.c
    bsp/usart/usart_tx_task.c
    bsp/watchdog/bsp_watchdog.c
    bsp/ws2812/ws2812.c)

set(MC02_COMMON_MODULE_C_SOURCES
    modules/algorithm/attitude_filter.c
    modules/algorithm/QuaternionEKF.c
    modules/algorithm/controller.c
    modules/algorithm/crc.c
    modules/algorithm/kalman_filter.c
    modules/algorithm/math_utils.c
    modules/algorithm/sample_filter.c
    modules/algorithm/vector3.c
    modules/BMI088/bmi088.c
    modules/daemon/daemon.c
    modules/daemon/daemon_event_task.c
    modules/imu/ins_task.c
    modules/lcd/lcd.c
    modules/lcd/lcd_buf.c
    modules/lcd/lcd_font_cn.c
    modules/lcd/lcd_key_input.c
    modules/lcd/lcd_referee_rows.c
    modules/lcd/lcd_task.c
    modules/lcd/lcd_ui.c
    modules/message_center/message_center.c
    modules/motor/DJImotor/dji_motor.c
    bsp/board/dji_can_diagnostic_stm32.c # 已选 DJI 驱动的板级回环诊断，默认不启用。
    modules/motor/HTmotor/HT04.c
    modules/motor/LKmotor/LK9025.c
    modules/motor/motor_feedback.c
    modules/motor/motor_pid.c
    modules/motor/motor_task.c
    modules/power/power_manager.c
    modules/referee/referee_task.c
    modules/referee/referee_tx_budget.c
    modules/referee/rm_referee.c
    modules/remote/remote_control.c
    modules/remote/remote_safety.c
    modules/safety/robot_safety.c
    modules/super_cap/super_cap.c
    modules/task_monitor/task_monitor.c
    modules/alarm/buzzer.c
    modules/alarm/led.c
    modules/alarm/robot_alarm.c
    modules/alarm/robot_alarm_policy.c)

if(MC02_IMU_BACKEND STREQUAL "fusion")
    list(APPEND MC02_COMMON_MODULE_C_SOURCES Middlewares/Third_Party/Fusion/FusionAhrs.c)
endif()

set(MC02_APPLICATION_COMMON_C_SOURCES
    application/robot_task.c # 常驻任务的周期执行与监控。
    application/robot_task_create.c # 使用框架接口创建现有整车任务。
    application/init_diagnostic.c
    application/vehicle_health.c
    application/vehicle_health_diag.c
    application/vehicle_health_port_stm32.c
    application/robot.c
    application/robot_def.c
    application/config/module_plan.c
    application/config/vehicle_selection.c
    application/config/sentinel_robot/sentinel_config.c)
set(MC02_APPLICATION_SINGLE_C_SOURCES
    application/chassis/chassis.c
    application/chassis/chassis_motion.c
    application/cmd/robot_cmd.c
    application/gimbal/gimbal.c
    application/shoot/shoot.c)
set(MC02_APPLICATION_CHASSIS_C_SOURCES
    application/topics/robot_can_bridge.c
    application/chassis/chassis.c
    application/chassis/chassis_motion.c)
set(MC02_APPLICATION_GIMBAL_C_SOURCES
    application/topics/robot_can_bridge.c
    application/cmd/robot_cmd.c
    application/gimbal/gimbal.c
    application/shoot/shoot.c)

if(MC02_PROFILE STREQUAL "robot")
    list(APPEND MC02_APPLICATION_COMMON_C_SOURCES
        application/topics/chassis_topic_codec.c
        application/topics/robot_topics.c
        application/topics/robot_topics_diag.c)
endif()
set(MC02_NEEDS_TIME_MAPPING FALSE)
if(MC02_FEATURE_HEATER)
    # 温控未选用时不编译加热器驱动；禁热状态下的长操作仍由任务监控限时。
    list(APPEND MC02_COMMON_MODULE_C_SOURCES modules/imu/imu_heater.c)
endif()
if(MC02_FEATURE_VISION)
    # 视觉适配与控制仅在车型启用视觉时进入固件。
    list(APPEND MC02_COMMON_MODULE_C_SOURCES
        modules/master_machine/master_process.c
        modules/master_machine/vision_control.c)
    if(MC02_VISION_PROTOCOL STREQUAL "v2")
        list(APPEND MC02_COMMON_MODULE_C_SOURCES modules/master_machine/vision_protocol_v2.c)
        set(MC02_NEEDS_TIME_MAPPING TRUE)
    else()
        list(APPEND MC02_COMMON_MODULE_C_SOURCES modules/master_machine/vision_protocol.c)
    endif()
endif()

# 只有双板固件使用板间话题与时间交换；单板不占用这些源码。
if(MC02_PROFILE STREQUAL "robot" AND NOT MC02_BOARD_ROLE STREQUAL "single")
    list(APPEND MC02_COMMON_MODULE_C_SOURCES
        modules/can_comm/can_topic_wire.c
        modules/can_comm/can_topic_transport.c
        modules/can_comm/can_topic_bridge.c
        modules/can_comm/can_topic_link.c
        modules/can_comm/can_time_sync.c)
    set(MC02_NEEDS_TIME_MAPPING TRUE)
endif()
if(MC02_NEEDS_TIME_MAPPING)
    list(APPEND MC02_COMMON_MODULE_C_SOURCES modules/time_sync/time_mapping.c)
endif()

# 未启用模块连同其输出驱动退出源码清单；依赖者使用同一份生成选择。
if(NOT MC02_FEATURE_LCD)
    list(FILTER MC02_COMMON_MODULE_C_SOURCES EXCLUDE REGEX "^modules/lcd/")
else()
    list(APPEND MC02_BSP_C_SOURCES bsp/board/platform_lcd_mc02.c) # 已选 LCD 的板级总线和引脚。
endif()
if(NOT MC02_FEATURE_ALARM)
    list(FILTER MC02_COMMON_MODULE_C_SOURCES EXCLUDE REGEX "^modules/alarm/")
    list(REMOVE_ITEM MC02_BSP_C_SOURCES bsp/ws2812/ws2812.c)
else()
    list(APPEND MC02_BSP_C_SOURCES bsp/board/platform_buzzer_mc02.c) # 板载蜂鸣器 PWM 绑定。
    list(APPEND MC02_BSP_C_SOURCES bsp/board/platform_rgb_mc02.c) # 板载 RGB 异步输出绑定。
    list(APPEND MC02_APPLICATION_COMMON_C_SOURCES application/alarm_startup.c) # 按依赖处理告警降级。
endif()
foreach(pair IN ITEMS "DJI;DJImotor/dji_motor.c" "LK;LKmotor/LK9025.c" "HT;HTmotor/HT04.c")
    list(GET pair 0 driver)
    list(GET pair 1 implementation)
    if(NOT MC02_SELECTED_DRIVER_${driver})
        list(REMOVE_ITEM MC02_COMMON_MODULE_C_SOURCES "modules/motor/${implementation}")
    endif()
endforeach()
if(NOT MC02_SELECTED_DRIVER_DJI)
    list(REMOVE_ITEM MC02_COMMON_MODULE_C_SOURCES bsp/board/dji_can_diagnostic_stm32.c) # 无 DJI 电机时不编入诊断。
endif()
if(MC02_SELECTED_DRIVER_HT)
    list(APPEND MC02_BSP_C_SOURCES bsp/board/platform_ht_delay_stm32.c) # 仅 HT 沿用的初始化忙等。
endif()
if(NOT MC02_FEATURE_SUPERCAP)
    list(REMOVE_ITEM MC02_COMMON_MODULE_C_SOURCES modules/super_cap/super_cap.c)
else()
    list(APPEND MC02_APPLICATION_COMMON_C_SOURCES application/chassis/chassis_supercap_legacy.c) # 保留旧电容启动检查与板级绑定。
endif()
if(NOT MC02_FEATURE_MOTOR_CONTROL AND NOT MC02_FEATURE_COMM_DISPATCH)
    list(REMOVE_ITEM MC02_COMMON_MODULE_C_SOURCES modules/motor/motor_task.c)
endif()
if(NOT MC02_FEATURE_REFEREE)
    list(REMOVE_ITEM MC02_COMMON_MODULE_C_SOURCES modules/referee/referee_task.c
        modules/referee/referee_tx_budget.c modules/referee/rm_referee.c)
endif()
if(NOT MC02_FEATURE_REMOTE)
    list(REMOVE_ITEM MC02_COMMON_MODULE_C_SOURCES modules/remote/remote_control.c
        modules/remote/remote_safety.c)
endif()
if(MC02_PROFILE STREQUAL "robot" AND MC02_FEATURE_CHASSIS)
    list(APPEND MC02_APPLICATION_COMMON_C_SOURCES application/chassis/chassis_kinematics.c
        "application/chassis/${MC02_CHASSIS_TYPE}.c")
endif()

# 按配置追加确实被入口调用的可选实现；其他保留模块不进入固件。
set(MC02_SELECTED_SOURCES ${MC02_PLATFORM_C_SOURCES} ${MC02_BSP_C_SOURCES} ${MC02_COMMON_MODULE_C_SOURCES} ${MC02_APPLICATION_COMMON_C_SOURCES})
if(MC02_PROFILE STREQUAL "robot")
    if(MC02_BOARD_ROLE STREQUAL "single")
        list(APPEND MC02_SELECTED_SOURCES ${MC02_APPLICATION_SINGLE_C_SOURCES})
    elseif(MC02_BOARD_ROLE STREQUAL "chassis")
        list(APPEND MC02_SELECTED_SOURCES ${MC02_APPLICATION_CHASSIS_C_SOURCES})
    else()
        list(APPEND MC02_SELECTED_SOURCES ${MC02_APPLICATION_GIMBAL_C_SOURCES})
    endif()
    if(MC02_ENABLE_REFEREE_UI)
        list(APPEND MC02_SELECTED_SOURCES modules/referee/referee_UI.c)
    endif()
else()
    if(MC02_BENCH_RC_DIAGNOSTICS)
        list(APPEND MC02_SELECTED_SOURCES modules/remote/remote_bench.c)
    endif()
    if(MC02_BENCH_REFEREE_DIAGNOSTICS)
        list(APPEND MC02_SELECTED_SOURCES modules/referee/referee_bench.c)
    endif()
    if(MC02_BENCH_USB_DIAGNOSTICS)
        list(APPEND MC02_SELECTED_SOURCES bsp/usb/usb_diagnostic.c)
    endif()
endif()

foreach(source IN LISTS MC02_SELECTED_SOURCES)
    if(NOT EXISTS "${CMAKE_SOURCE_DIR}/${source}")
        message(FATAL_ERROR "源码清单中的文件不存在：${source}")
    endif()
endforeach()

# 观测模块只引用当前配置登记的源码，避免诊断功能强行链接未选驱动。
function(mc02_set_observation_source_flag flag source)
    list(FIND MC02_SELECTED_SOURCES "${source}" source_index)
    if(source_index EQUAL -1)
        add_compile_definitions(${flag}=0)
    else()
        add_compile_definitions(${flag}=1)
    endif()
endfunction()
mc02_set_observation_source_flag(MC02_OBS_HAS_INS modules/imu/ins_task.c)
mc02_set_observation_source_flag(MC02_OBS_HAS_MOTOR modules/motor/motor_task.c)
mc02_set_observation_source_flag(MC02_OBS_HAS_DJI modules/motor/DJImotor/dji_motor.c)
mc02_set_observation_source_flag(MC02_OBS_HAS_CAN bsp/can/bsp_can.c)
mc02_set_observation_source_flag(MC02_OBS_HAS_UART bsp/usart/usart_tx_task.c)
mc02_set_observation_source_flag(MC02_OBS_HAS_USB bsp/usb/usb_transport.c)
mc02_set_observation_source_flag(MC02_OBS_HAS_LCD modules/lcd/lcd_task.c)
set(MC02_EXTRA_INCLUDE_DIRS
    application
    application/config
    modules
    modules/motor
    bsp)
