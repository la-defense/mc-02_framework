cmake_minimum_required(VERSION 3.21)

# 固件只通过本目录的 ARM 工具链和显式源码清单构建；主机测试使用 tests/host 工程。
project(Basic_Framework_MC02 C ASM)

include(cmake/mc02_format_diagnostics.cmake)
mc02_check_format_diagnostics()
include(cmake/mc02_dsp.cmake)

set(MC02_PROFILE "bench_safe" CACHE STRING "固件运行配置")
set_property(CACHE MC02_PROFILE PROPERTY STRINGS bench_safe robot)
set(MC02_BOARD_ROLE "single" CACHE STRING "机器人板卡角色")
set_property(CACHE MC02_BOARD_ROLE PROPERTY STRINGS single chassis gimbal)
set(MC02_VISION_TRANSPORT "usb" CACHE STRING "视觉链路传输方式")
set_property(CACHE MC02_VISION_TRANSPORT PROPERTY STRINGS usb uart)
set(MC02_VISION_PROTOCOL "sp" CACHE STRING "视觉业务协议")
set_property(CACHE MC02_VISION_PROTOCOL PROPERTY STRINGS sp v2)
set(MC02_IMU_BACKEND "quaternion_ekf" CACHE STRING "INS姿态算法后端")
set_property(CACHE MC02_IMU_BACKEND PROPERTY STRINGS quaternion_ekf fusion)

option(MC02_ENABLE_HEATER "显式允许机器人配置使用 IMU 加热器" OFF)
option(MC02_BENCH_RC_DIAGNOSTICS "裸板遥控接收诊断；禁止请求 READY" OFF)
option(MC02_BENCH_REFEREE_DIAGNOSTICS "裸板裁判有限负载诊断；默认关闭" OFF)
option(MC02_LOG_TEST "编入类型化日志自测事件" OFF)
option(MC02_BENCH_CRASH_DIAGNOSTICS "仅bench_safe固件允许的一次性崩溃记录自测" OFF)
option(MC02_BENCH_LCD_DIAGNOSTICS "仅bench_safe固件运行四页LCD绘图自测" OFF)
option(MC02_BENCH_USB_DIAGNOSTICS "仅bench_safe固件运行有限USB完成通知丢失诊断" OFF)
set(MC02_LOG_COMPILED_MAX_LEVEL "VERBOSE" CACHE STRING "日志最高编译等级")
set_property(CACHE MC02_LOG_COMPILED_MAX_LEVEL PROPERTY STRINGS FAULT STATUS VERBOSE)
if(MC02_LOG_COMPILED_MAX_LEVEL STREQUAL "FAULT")
    set(MC02_LOG_COMPILED_MAX_LEVEL_VALUE 1)
elseif(MC02_LOG_COMPILED_MAX_LEVEL STREQUAL "STATUS")
    set(MC02_LOG_COMPILED_MAX_LEVEL_VALUE 2)
elseif(MC02_LOG_COMPILED_MAX_LEVEL STREQUAL "VERBOSE")
    set(MC02_LOG_COMPILED_MAX_LEVEL_VALUE 3)
else()
    message(FATAL_ERROR "MC02_LOG_COMPILED_MAX_LEVEL 必须是 FAULT、STATUS 或 VERBOSE")
endif()
add_compile_definitions(MC02_LOG_COMPILED_MAX_LEVEL=${MC02_LOG_COMPILED_MAX_LEVEL_VALUE})

if(NOT MC02_PROFILE STREQUAL "bench_safe" AND NOT MC02_PROFILE STREQUAL "robot")
    message(FATAL_ERROR "MC02_PROFILE 必须是 bench_safe 或 robot")
endif()
if(NOT MC02_BOARD_ROLE MATCHES "^(single|chassis|gimbal)$")
    message(FATAL_ERROR "MC02_BOARD_ROLE 必须是 single、chassis 或 gimbal")
endif()
if(NOT MC02_VISION_TRANSPORT MATCHES "^(usb|uart)$")
    message(FATAL_ERROR "MC02_VISION_TRANSPORT 必须是 usb 或 uart")
endif()
if(NOT MC02_VISION_PROTOCOL MATCHES "^(sp|v2)$")
    message(FATAL_ERROR "MC02_VISION_PROTOCOL 必须是 sp 或 v2")
endif()
if(NOT MC02_IMU_BACKEND MATCHES "^(quaternion_ekf|fusion)$")
    message(FATAL_ERROR "MC02_IMU_BACKEND 必须是 quaternion_ekf 或 fusion")
endif()
if(MC02_IMU_BACKEND STREQUAL "fusion")
    add_compile_definitions(MC02_IMU_BACKEND_FUSION=1)
else()
    add_compile_definitions(MC02_IMU_BACKEND_FUSION=0)
endif()
if(MC02_VISION_PROTOCOL STREQUAL "v2" AND
   (NOT MC02_PROFILE STREQUAL "robot" OR NOT MC02_BOARD_ROLE STREQUAL "single"))
    message(FATAL_ERROR "V2视觉控制目前仅允许robot/single；其它角色缺少经验证的裁判与跨板许可快照")
endif()

# 板卡角色和视觉链路只由下方选项生成宏，避免旧缓存或用户 CFLAGS 注入第二种布局。
foreach(flags_variable IN ITEMS CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELWITHDEBINFO CMAKE_C_FLAGS_RELEASE)
    foreach(managed_macro IN ITEMS ONE_BOARD CHASSIS_BOARD GIMBAL_BOARD VISION_USE_VCP VISION_USE_UART)
        if("${${flags_variable}}" MATCHES "(^|[ \t])-D${managed_macro}(=[^ \t]*)?([ \t]|$)")
            message(FATAL_ERROR "${managed_macro} 由 MC02_BOARD_ROLE/MC02_VISION_TRANSPORT 管理，请从 ${flags_variable} 移除旧定义")
        endif()
    endforeach()
endforeach()

if(MC02_PROFILE STREQUAL "bench_safe" AND NOT MC02_BOARD_ROLE STREQUAL "single")
    message(FATAL_ERROR "bench_safe 仅支持 single 角色")
endif()
if(MC02_PROFILE STREQUAL "bench_safe" AND MC02_ENABLE_HEATER)
    message(FATAL_ERROR "bench_safe 配置不允许启用 IMU 加热器")
endif()
if(MC02_PROFILE STREQUAL "robot" AND MC02_ENABLE_HEATER)
    add_compile_definitions(MC02_PROFILE_ROBOT=1 MC02_HEATER_ENABLED=1)
else()
    if(MC02_PROFILE STREQUAL "robot")
        add_compile_definitions(MC02_PROFILE_ROBOT=1 MC02_HEATER_ENABLED=0)
    else()
        add_compile_definitions(MC02_PROFILE_BENCH_SAFE=1 MC02_HEATER_ENABLED=0)
    endif()
endif()

if(MC02_PROFILE STREQUAL "robot")
    if(MC02_BOARD_ROLE STREQUAL "single")
        set(MC02_REFEREE_UI_DEFAULT ON)
        add_compile_definitions(ONE_BOARD)
    elseif(MC02_BOARD_ROLE STREQUAL "chassis")
        set(MC02_REFEREE_UI_DEFAULT ON)
        add_compile_definitions(CHASSIS_BOARD)
    else()
        set(MC02_REFEREE_UI_DEFAULT OFF)
        add_compile_definitions(GIMBAL_BOARD)
    endif()
else()
    set(MC02_REFEREE_UI_DEFAULT OFF)
endif()
option(MC02_ENABLE_REFEREE_UI "创建裁判显示任务" ${MC02_REFEREE_UI_DEFAULT})
if(MC02_ENABLE_REFEREE_UI AND (NOT MC02_PROFILE STREQUAL "robot" OR MC02_BOARD_ROLE STREQUAL "gimbal"))
    message(FATAL_ERROR "裁判 UI 仅允许 robot 的 single/chassis 角色启用")
endif()

if(MC02_BENCH_RC_DIAGNOSTICS AND MC02_PROFILE STREQUAL "robot")
    message(FATAL_ERROR "遥控诊断仅允许 bench_safe 配置")
endif()
if(MC02_BENCH_REFEREE_DIAGNOSTICS AND (NOT MC02_PROFILE STREQUAL "bench_safe" OR MC02_BENCH_RC_DIAGNOSTICS))
    message(FATAL_ERROR "裁判诊断仅允许独立 bench_safe 配置")
endif()
if(MC02_LOG_TEST AND NOT MC02_PROFILE STREQUAL "bench_safe")
    message(FATAL_ERROR "日志自测仅允许 bench_safe 配置")
endif()
if(MC02_BENCH_CRASH_DIAGNOSTICS AND
   (NOT MC02_PROFILE STREQUAL "bench_safe" OR MC02_BOARD_ROLE STREQUAL "gimbal" OR
    MC02_ENABLE_HEATER OR MC02_BENCH_RC_DIAGNOSTICS OR MC02_BENCH_REFEREE_DIAGNOSTICS))
    message(FATAL_ERROR "崩溃自测仅允许禁热、无冲突诊断的 bench_safe 单板配置")
endif()
if(MC02_BENCH_LCD_DIAGNOSTICS AND
   (NOT MC02_PROFILE STREQUAL "bench_safe" OR NOT MC02_BOARD_ROLE STREQUAL "single" OR
    MC02_ENABLE_HEATER OR MC02_BENCH_RC_DIAGNOSTICS OR MC02_BENCH_REFEREE_DIAGNOSTICS OR
    MC02_BENCH_CRASH_DIAGNOSTICS))
    message(FATAL_ERROR "LCD自测仅允许禁热、无其他硬件自测的 bench_safe 单板配置")
endif()

if(MC02_BENCH_USB_DIAGNOSTICS AND
   (NOT MC02_PROFILE STREQUAL "bench_safe" OR NOT MC02_BOARD_ROLE STREQUAL "single" OR
    NOT MC02_VISION_TRANSPORT STREQUAL "usb" OR MC02_ENABLE_HEATER OR MC02_BENCH_RC_DIAGNOSTICS OR
    MC02_BENCH_REFEREE_DIAGNOSTICS OR MC02_BENCH_CRASH_DIAGNOSTICS OR MC02_BENCH_LCD_DIAGNOSTICS))
    message(FATAL_ERROR "USB诊断仅允许禁热、无冲突诊断的bench_safe/single/usb配置")
endif()

foreach(option_name IN ITEMS MC02_BENCH_RC_DIAGNOSTICS MC02_BENCH_REFEREE_DIAGNOSTICS MC02_LOG_TEST MC02_ENABLE_REFEREE_UI MC02_BENCH_CRASH_DIAGNOSTICS MC02_BENCH_LCD_DIAGNOSTICS MC02_BENCH_USB_DIAGNOSTICS)
    if(${option_name})
        if(option_name STREQUAL "MC02_LOG_TEST")
            add_compile_definitions(LOG_TEST=1)
        else()
            add_compile_definitions(${option_name}=1)
        endif()
    else()
        if(option_name STREQUAL "MC02_LOG_TEST")
            add_compile_definitions(LOG_TEST=0)
        else()
            add_compile_definitions(${option_name}=0)
        endif()
    endif()
endforeach()
if(MC02_VISION_TRANSPORT STREQUAL "usb")
    add_compile_definitions(VISION_USE_VCP)
else()
    add_compile_definitions(VISION_USE_UART)
endif()
if(MC02_VISION_PROTOCOL STREQUAL "v2")
    add_compile_definitions(MC02_VISION_PROTOCOL_V2=1)
else()
    add_compile_definitions(MC02_VISION_PROTOCOL_SP=1)
endif()

# 先检查车型、核心资料和模块依赖，再决定实际参与构建的源码。
include(cmake/mc02_configure_vehicle.cmake)
# 源码清单读取已经解析并校验过的配置，确保协议和板级角色只编译所选实现。
include(cmake/mc02_firmware_sources.cmake)
if(MC02_IMU_BACKEND STREQUAL "fusion")
    list(APPEND MC02_EXTRA_INCLUDE_DIRS Middlewares/Third_Party/Fusion)
endif()

if(MC02_PROFILE STREQUAL "bench_safe")
    set(MC02_OUTPUT_BASENAME "${PROJECT_NAME}_bench_safe")
elseif(MC02_BOARD_ROLE STREQUAL "single")
    set(MC02_OUTPUT_BASENAME "${PROJECT_NAME}_robot")
elseif(MC02_BOARD_ROLE STREQUAL "chassis")
    set(MC02_OUTPUT_BASENAME "${PROJECT_NAME}_chassis_robot")
else()
    set(MC02_OUTPUT_BASENAME "${PROJECT_NAME}_gimbal_robot")
endif()
if(MC02_BENCH_RC_DIAGNOSTICS)
    set(MC02_OUTPUT_BASENAME "${PROJECT_NAME}_rcdiag_bench_safe")
elseif(MC02_BENCH_REFEREE_DIAGNOSTICS)
    set(MC02_OUTPUT_BASENAME "${PROJECT_NAME}_refdiag_bench_safe")
elseif(MC02_BENCH_CRASH_DIAGNOSTICS)
    set(MC02_OUTPUT_BASENAME "${PROJECT_NAME}_crashdiag_bench_safe")
elseif(MC02_BENCH_LCD_DIAGNOSTICS)
    set(MC02_OUTPUT_BASENAME "${PROJECT_NAME}_lcdtest_bench_safe")
elseif(MC02_BENCH_USB_DIAGNOSTICS)
    set(MC02_OUTPUT_BASENAME "${PROJECT_NAME}_usbtest_bench_safe")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "构建类型" FORCE)
endif()

set(MCU_FLAGS -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16)
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32H723VGTx_FLASH.ld")
set(MC02_LINKER_OPTIONS)
execute_process(COMMAND "${CMAKE_LINKER}" --help OUTPUT_VARIABLE MC02_LINKER_HELP ERROR_VARIABLE MC02_LINKER_HELP_ERROR)
if("${MC02_LINKER_HELP}${MC02_LINKER_HELP_ERROR}" MATCHES "--no-warn-rwx-segments")
    list(APPEND MC02_LINKER_OPTIONS -Wl,--no-warn-rwx-segments)
endif()

option(ENABLE_PRINTF_FLOAT "链接 newlib 的 _printf_float，让 %f/%e/%g 能正常格式化" ON)
add_compile_options(-pipe ${MCU_FLAGS} -Wall -fmessage-length=0 -ffunction-sections -fdata-sections -fno-common)
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    add_compile_options(-O2)
    if(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
        add_compile_options(-g)
    endif()
elseif(CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
    add_compile_options(-Os)
else()
    add_compile_options(-Og -g -gdwarf-2)
endif()

# WSL 中的 DWARF 源路径转换为 Windows 工作区路径，供 VSCode 和 Ozone 打开源码。
execute_process(COMMAND wslpath -w "${CMAKE_SOURCE_DIR}" OUTPUT_VARIABLE MC02_DEBUG_SOURCE_ROOT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(MC02_DEBUG_SOURCE_ROOT)
    string(REPLACE "\\" "/" MC02_DEBUG_SOURCE_ROOT "${MC02_DEBUG_SOURCE_ROOT}")
else()
    set(MC02_DEBUG_SOURCE_ROOT "${CMAKE_SOURCE_DIR}")
endif()
add_compile_options("-fdebug-prefix-map=${CMAKE_SOURCE_DIR}=${MC02_DEBUG_SOURCE_ROOT}")

add_link_options(-pipe ${MCU_FLAGS} -T${LINKER_SCRIPT} ${MC02_LINKER_OPTIONS} -lm -lc -Wl,--gc-sections -Wl,--build-id=sha1 -specs=nano.specs
    -Wl,-Map=${PROJECT_BINARY_DIR}/${MC02_OUTPUT_BASENAME}.map -Wl,--cref -Wl,--print-memory-usage)
if(ENABLE_PRINTF_FLOAT)
    add_link_options(-u _printf_float)
endif()

add_compile_definitions(USE_HAL_DRIVER STM32H723xx)

set(MC02_PLATFORM_INCLUDE_DIRS
    Core/Inc
    USB_DEVICE/App
    USB_DEVICE/Target
    Drivers/STM32H7xx_HAL_Driver/Inc
    Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    Drivers/CMSIS/Device/ST/STM32H7xx/Include
    Drivers/CMSIS/Include
    Middlewares/Third_Party/SEGGER/RTT
    Middlewares/Third_Party/SEGGER/Config)
# 公共声明仍供现行参数类型引用；实现是否参与链接由源码清单决定。
set(MC02_INCLUDE_DIRS ${MC02_PLATFORM_INCLUDE_DIRS} ${MC02_EXTRA_INCLUDE_DIRS}
    application/config application/chassis modules/referee modules/remote
    modules/motor/DJImotor modules/motor/HTmotor modules/motor/LKmotor
    modules/super_cap modules/lcd modules/alarm)
foreach(source IN LISTS MC02_SELECTED_SOURCES)
    get_filename_component(source_dir "${source}" DIRECTORY)
    if(source_dir MATCHES "^(bsp|modules|application)(/|$)")
        list(APPEND MC02_INCLUDE_DIRS "${source_dir}")
    endif()
endforeach()
list(REMOVE_DUPLICATES MC02_INCLUDE_DIRS)
foreach(include_dir IN LISTS MC02_INCLUDE_DIRS)
    if(NOT IS_DIRECTORY "${CMAKE_SOURCE_DIR}/${include_dir}")
        message(FATAL_ERROR "已登记的头文件目录不存在：${include_dir}")
    endif()
endforeach()

set(MC02_ASM_SOURCES
    startup_stm32h723xx.s
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s)
set_source_files_properties(${MC02_ASM_SOURCES} PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

# 这些框架文件只依赖自有接口和标准 C；单独编译以阻止继承芯片或 RTOS 头路径。
set(MC02_PORTABLE_FRAMEWORK_SOURCES
    application/init_diagnostic.c # 初始化阶段与故障事件记录。
    application/vehicle_health.c # 整车健康信息的采集和发布。
    application/vehicle_health_diag.c # 健康信息的只读诊断应答。
    application/robot.c # 整车启动与周期协调。
    application/robot_def.c # 车型配置选择与参数校验。
    application/config/module_plan.c # 模块依赖和启动结果。
    application/config/vehicle_selection.c # 车型身份和模块计划。
    application/config/sentinel_robot/sentinel_config.c # 哨兵静态车型配置。
    modules/daemon/daemon.c # 来源期限、状态和事件队列。
    modules/daemon/daemon_event_task.c # 来源事件处理任务的框架入口。
    modules/message_center/message_center.c # 有界消息与话题目录。
    modules/safety/robot_safety.c) # 整车许可、故障与急停状态。
# 板间通信和时间映射仅在车型选择时进入独立目标。
set(MC02_OPTIONAL_PORTABLE_SOURCES
    modules/can_comm/can_topic_wire.c # 板间话题帧编码和校验。
    modules/can_comm/can_topic_transport.c # 话题分片发送与接收。
    modules/can_comm/can_topic_bridge.c # 话题与板间传输转换。
    modules/can_comm/can_topic_link.c # 逻辑 CAN 端点和桥接。
    modules/can_comm/can_time_sync.c # 板间时间同步交互。
    modules/time_sync/time_mapping.c # 不同时钟之间的时间换算。
    modules/referee/referee_UI.c # 裁判图形编码和发送状态。
    modules/referee/referee_task.c # 裁判界面状态与字段更新。
    modules/referee/referee_tx_budget.c # 裁判通信发送额度。
    modules/referee/rm_referee.c # 裁判协议解析和逻辑串口接入。
    modules/remote/remote_control.c # 遥控解析与逻辑串口接入。
    modules/remote/remote_safety.c # 遥控手势与安全请求。
    modules/master_machine/master_process.c # 上位机协议和传输状态处理。
    modules/imu/ins_task.c # 姿态采样、解算和安全状态。
    modules/imu/imu_heater.c # 温控策略使用逻辑 PWM 与模拟输入。
    modules/motor/DJImotor/dji_motor.c # DJI 普通控制及逻辑 CAN 接口。
    modules/motor/motor_task.c # 单一通信派发和电机周期任务。
    modules/motor/motor_feedback.c # 电机能力和输出轴换算。
    modules/motor/motor_pid.c # 电机控制周期与 PID 状态。
    modules/task_monitor/task_monitor.c # 任务进度、期限及核心故障策略。
    modules/alarm/robot_alarm.c # 安全状态与灯声输出的告警协调。
    modules/alarm/robot_alarm_policy.c # 故障仲裁、节奏与静音策略。
    modules/alarm/buzzer.c # 音符映射和蜂鸣器输出参数检查。
    modules/alarm/led.c # RGB 输出状态转换和任务推进。
    application/alarm_startup.c # 按核心依赖处理告警初始化结果。
    application/robot_task.c # 现有任务循环与周期维护。
    application/gimbal/gimbal.c # 云台命令、反馈和电机控制。
    application/shoot/shoot.c # 发射命令和拨弹时序。
    application/chassis/chassis.c # 底盘命令、反馈和电机控制。
    application/cmd/robot_cmd.c # 遥控、视觉和控制命令仲裁。
    modules/lcd/lcd_buf.c # LCD 有界绘图与字体转换。
    modules/lcd/lcd_key_input.c # LCD 按键去抖和事件。
    modules/lcd/lcd_referee_rows.c # 裁判数据显示行格式化。
    modules/lcd/lcd_task.c # LCD 按键、页面、标定提示及任务统计。
    application/robot_task_create.c) # 已选任务的创建与失败处理。
foreach(source IN LISTS MC02_OPTIONAL_PORTABLE_SOURCES)
    if(source IN_LIST MC02_SELECTED_SOURCES)
        list(APPEND MC02_PORTABLE_FRAMEWORK_SOURCES "${source}")
    endif()
endforeach()
set(MC02_EXECUTABLE_SOURCES ${MC02_SELECTED_SOURCES})
list(REMOVE_ITEM MC02_EXECUTABLE_SOURCES ${MC02_PORTABLE_FRAMEWORK_SOURCES})
add_library(mc02_portable_framework OBJECT ${MC02_PORTABLE_FRAMEWORK_SOURCES})
target_include_directories(mc02_portable_framework PRIVATE
    modules/daemon # 来源监控的公开类型。
    modules/message_center # 消息目录和新鲜度类型。
    modules/safety # 整车状态和安全管理的公开类型。
    bsp/log # 安全状态变化使用的有界事件接口。
    bsp/os # 不暴露厂商类型的任务、时间和同步接口。
    bsp/adc # LCD 按键只读的 ADC 样本接口。
    bsp/crash # 崩溃诊断的公开记录接口。
    bsp # 基础启动入口。
    bsp/param # 参数事务公开接口。
    bsp/watchdog # 看门狗公开接口。
    bsp/reset_recovery # 复位恢复状态接口。
    bsp/observation # 只读运行状态记录接口。
    bsp/safety # 启动失败时使用的板级直接关断入口。
    application # LCD 标记初始化诊断结果。
    application/config # 编译选择和生成的车型标识。
    application/config/sentinel_robot # 当前哨兵配置入口。
    application/chassis # 底盘配置和接口。
    application/gimbal # 云台业务接口。
    application/shoot # 发射业务接口。
    application/cmd # 整车命令接口。
    modules/BMI088) # 与板级采样类型分离的标定控制接口。
mc02_apply_format_diagnostics(mc02_portable_framework)
target_include_directories(mc02_portable_framework PRIVATE
    bsp/can # 逻辑 CAN 端点和帧接口。
    modules/can_comm # 板间话题和同步接口。
    modules/time_sync # 时钟映射接口。
    modules/algorithm # 帧校验使用的软件 CRC 接口。
    modules # 电机控制使用的公共单位定义。
    modules/motor # 电机能力、反馈及 PID 接口。
    modules/motor/DJImotor # DJI 电机公开控制接口。
    modules/lcd # 显示状态和绘图接口。
    modules/alarm # 告警策略与灯声输出接口。
    bsp/board # 板级资源和蜂鸣器公开接口。
    modules/referee # 裁判协议与图形接口。
    modules/remote # 遥控输入公开接口。
    modules/master_machine # 上位机协议和传输选择接口。
    bsp/usb # 不含芯片类型的 USB 传输接口。
    modules/imu # 姿态任务及传感器接口。
    modules/BMI088 # 传感器完整样本的数据类型。
    modules/task_monitor # 任务进度与期限接口。
    bsp/dwt # 不暴露寄存器的短时探针接口。
    "${CMAKE_BINARY_DIR}/generated" # 当前车型编译选择。
    bsp/usart) # 串口发送结果的框架类型。

add_executable(${MC02_OUTPUT_BASENAME}.elf ${MC02_EXECUTABLE_SOURCES}
    $<TARGET_OBJECTS:mc02_portable_framework> ${MC02_ASM_SOURCES} ${LINKER_SCRIPT})
# 保留USB任务和恢复后端的静态栈使用报告，配合裸板历史最小余量核对。
set_source_files_properties(Core/Src/freertos.c bsp/usb/usb_transport.c
    bsp/usb/usb_transport_port_stm32.c USB_DEVICE/Target/usbd_conf.c
    PROPERTIES COMPILE_OPTIONS "-fstack-usage")
target_include_directories(${MC02_OUTPUT_BASENAME}.elf PRIVATE ${MC02_INCLUDE_DIRS})
target_include_directories(${MC02_OUTPUT_BASENAME}.elf PRIVATE "${CMAKE_BINARY_DIR}/generated")
mc02_apply_format_diagnostics(${MC02_OUTPUT_BASENAME}.elf)
mc02_add_dsp(mc02_dsp FALSE)
target_link_libraries(${MC02_OUTPUT_BASENAME}.elf PRIVATE mc02_dsp m)

add_custom_command(TARGET ${MC02_OUTPUT_BASENAME}.elf POST_BUILD
    COMMENT "生成 HEX/BIN 并显示固件大小"
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${MC02_OUTPUT_BASENAME}.elf> ${PROJECT_BINARY_DIR}/${MC02_OUTPUT_BASENAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${MC02_OUTPUT_BASENAME}.elf> ${PROJECT_BINARY_DIR}/${MC02_OUTPUT_BASENAME}.bin
    COMMAND ${SIZE} $<TARGET_FILE:${MC02_OUTPUT_BASENAME}.elf>)

file(WRITE "${CMAKE_BINARY_DIR}/mc02-selected-sources.txt" "profile=${MC02_PROFILE}\nrole=${MC02_BOARD_ROLE}\nvision=${MC02_VISION_TRANSPORT}\nvision_protocol=${MC02_VISION_PROTOCOL}\nimu_backend=${MC02_IMU_BACKEND}\ntarget=${MC02_OUTPUT_BASENAME}.elf\n")
foreach(source IN LISTS MC02_SELECTED_SOURCES)
    file(APPEND "${CMAKE_BINARY_DIR}/mc02-selected-sources.txt" "${source}\n")
endforeach()
