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
    bsp/gpio/bsp_gpio.c
    bsp/iic/bsp_iic.c
    bsp/log/bsp_log.c
    bsp/log/bsp_log_port_stm32.c
    bsp/log/bsp_log_stdio.c
    bsp/log/float_to_string.c
    bsp/param/bsp_param.c
    bsp/pwm/bsp_pwm.c
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
    modules/algorithm/QuaternionEKF.c
    modules/algorithm/controller.c
    modules/algorithm/crc.c
    modules/algorithm/dsp_matrix.c
    modules/algorithm/kalman_filter.c
    modules/algorithm/math_utils.c
    modules/algorithm/sample_filter.c
    modules/algorithm/vector3.c
    modules/BMI088/bmi088.c
    modules/can_comm/can_comm.c
    modules/daemon/daemon.c
    modules/imu/BMI088Middleware.c
    modules/imu/imu_heater.c
    modules/imu/ins_task.c
    modules/lcd/lcd.c
    modules/lcd/lcd_buf.c
    modules/lcd/lcd_font_cn.c
    modules/lcd/lcd_key_input.c
    modules/lcd/lcd_referee_rows.c
    modules/lcd/lcd_task.c
    modules/lcd/lcd_ui.c
    modules/master_machine/master_process.c
    modules/master_machine/vision_control.c
    modules/message_center/message_center.c
    modules/motor/DJImotor/dji_motor.c
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
    modules/alarm/led.c)

set(MC02_APPLICATION_COMMON_C_SOURCES
    application/init_diagnostic.c
    application/robot.c
    application/robot_def.c
    application/config/sentinel_robot/sentinel_config.c)
set(MC02_APPLICATION_SINGLE_C_SOURCES
    application/chassis/chassis.c
    application/chassis/chassis_motion.c
    application/cmd/robot_cmd.c
    application/gimbal/gimbal.c
    application/shoot/shoot.c)
set(MC02_APPLICATION_CHASSIS_C_SOURCES
    application/chassis/chassis.c
    application/chassis/chassis_motion.c)
set(MC02_APPLICATION_GIMBAL_C_SOURCES
    application/cmd/robot_cmd.c
    application/gimbal/gimbal.c
    application/shoot/shoot.c)

if(MC02_PROFILE STREQUAL "robot")
    list(APPEND MC02_APPLICATION_COMMON_C_SOURCES application/topics/robot_topics.c)
endif()
if(MC02_VISION_PROTOCOL STREQUAL "v2")
    list(APPEND MC02_COMMON_MODULE_C_SOURCES modules/master_machine/vision_protocol_v2.c)
else()
    list(APPEND MC02_COMMON_MODULE_C_SOURCES modules/master_machine/vision_protocol.c)
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
