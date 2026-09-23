cmake_minimum_required(VERSION 3.16)
# 指定编译平台/架构与语言标准, 推荐指定Ninja为构建工具,可以加快编译速度(相比make)
# 适配 STM32H723 (达妙 MC02), cortex-m7 + fpv5-d16
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
# 指定工具链
set(CMAKE_C_COMPILER_FORCED TRUE) # skip compiler test
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 指定工程名称和语言类型
project(Basic_Framework_MC02 C ASM)

# 选择构建类型(允许命令行覆盖: -DCMAKE_BUILD_TYPE=RelWithDebInfo)
#   Debug        : -Og -g   —— 日常开发/单步调试
#   RelWithDebInfo: -O2 -g   —— 性能构建(测性能、上车), 见 docs/学习笔记/01
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif ()

# board specific settings, arch/fpu/instruction (H723: cortex-m7, fpv5-d16)
set(MCU_FLAGS -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16)
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32H723VGTx_FLASH.ld") # 指定链接脚本
set(DSP_NAME "arm_cortexM7lfdp_math") # 指定DSP库名称 (M7)
link_directories(${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC)

# Generic compiler settings for optimization and basic link lib
add_compile_options(-pipe ${MCU_FLAGS} -Wall -fmessage-length=0 # basic options
                    -ffunction-sections -fdata-sections -fno-common # optimize options
                    )
add_link_options(-pipe ${MCU_FLAGS} -T${LINKER_SCRIPT} -Wl,--no-warn-rwx-segments # close RWX warning
                 -lm -lc # lib options
                 -Wl,--gc-sections -specs=nano.specs # optimize options
                 # 不要加 -lnosys/-specs=nosys.specs:
                 # Core/Src/syscalls.c 与 Core/Src/sysmem.c 已实现 _exit/_sbrk 等,
                 # 再链 libnosys.a 会产生 multiple definition 链接错误。
                 -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map -Wl,--cref -Wl,--print-memory-usage # map options
                 )

# add_compile_definitions() works for compile stage
# while add_definitions() works for both compile and link stage
add_definitions(
    -DUSE_HAL_DRIVER
    -DSTM32H723xx
    -DARM_MATH_CM7
    )

# add inc
# 递归包含头文件的函数
function(include_sub_directories_recursively root_dir)
    if (IS_DIRECTORY ${root_dir})               # 当前路径是一个目录吗，是的话就加入到包含目录
        message("include dir: " ${root_dir})
        include_directories(${root_dir})
    endif()

    file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*) # 获得当前目录下的所有文件，让如ALL_SUB列表中
    foreach(sub ${ALL_SUB})
        if (IS_DIRECTORY ${root_dir}/${sub})
            include_sub_directories_recursively(${root_dir}/${sub}) # 对子目录递归调用，包含
        endif()
    endforeach()
endfunction()

# 注意: 不要递归包含整个 Drivers。
# Drivers/CMSIS/RTOS/Template 下的 cmsis_os.h 是 CMSIS-RTOS v1 模板,
# 与 Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.h 同名,
# 若先被搜到会导致 Core/Src/freertos.c 编译失败。
# 这里按 Makefile 的实际依赖列出头文件搜索路径。
include_directories(
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/USB_DEVICE/App
    ${CMAKE_SOURCE_DIR}/USB_DEVICE/Target
    ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Inc
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/SEGGER/RTT
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/SEGGER/Config
)
# 业务代码目录可以递归, 便于新增模块自动加入
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/bsp)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/modules)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/application)

# add source, only surfix .c
# 只收集实际参与编译的源文件, 避免把 CMSIS DSP/NN 的示例与测试代码
# (Drivers/CMSIS/DSP 下约 500 个 .c) 全部编进工程。
file(GLOB SOURCES
    "Core/Src/*.c"
    "USB_DEVICE/App/*.c"
    "USB_DEVICE/Target/*.c"
    "Middlewares/Third_Party/FreeRTOS/Source/*.c"
    "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/*.c"
    "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/*.c"
    "Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c"
    "Middlewares/Third_Party/SEGGER/RTT/*.c"
    "Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c"
    "Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c"
    "Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c"
    "Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c"
)
# HAL 源文件按 Makefile 实际使用的列表选择。
# 不能 glob 全部 Src/*.c: 部分 LL 驱动 (如 stm32h7xx_ll_utils.c) 在当前
# HAL 版本下无法通过 H723 编译, 且 *_template.c 会与 Core/Src 的实现重复定义。
set(HAL_SOURCES
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
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
)
list(APPEND SOURCES ${HAL_SOURCES})
file(GLOB_RECURSE SOURCES_APP
    "bsp/*.c"
    "modules/*.c"
    "application/*.c"
)
list(APPEND SOURCES ${SOURCES_APP})

# 兜底排除 *_template.c (HAL/USB 目录里的 CubeMX 模板与 Core/Src 下的
# 实际实现存在重复强符号)。
list(FILTER SOURCES EXCLUDE REGEX ".*_template\\.c$")
# bsp/flash/bsp_flash.c 是 F1/F4 风格的 Flash 驱动 (FLASH_TYPEPROGRAM_WORD /
# FLASH_SECTOR_8~11), 与 H723 的 Flash API 不兼容, 且当前没有任何调用方,
# 先排除出 CMake 构建, 待按 H7 规范重写后再加入。
list(FILTER SOURCES EXCLUDE REGEX "bsp/flash/bsp_flash\\.c$")

# 汇编文件路径
set(ASM_SOURCES
    startup_stm32h723xx.s
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s
    )
set_source_files_properties(${ASM_SOURCES} PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

# Build types
# 注意: 原来 Release/RelWithDebInfo 用 -Ofast(= -O3 -ffast-math)。
# fast-math 允许编译器假设"没有 NaN/Inf", 会把 isfinite() 这类安全判据优化掉
# (本工程的 EKF/PID/IMU 有效性/温控保护都依赖它们), 因此改为 -O2。
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Optimization -O2 (fast-math disabled for safety checks)")
    add_compile_options(-O2)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Optimization -O2 with debug info (fast-math disabled)")
    add_compile_options(-O2 -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g -gdwarf-2)
endif ()

# build binary and hex file
add_executable(${PROJECT_NAME}.elf  ${SOURCES} ${ASM_SOURCES} ${LINKER_SCRIPT})
target_link_libraries(${PROJECT_NAME}.elf ${DSP_NAME} m) # link DSP lib and math lib

add_custom_command(
  TARGET ${PROJECT_NAME}.elf POST_BUILD
  COMMENT "Building hex & bin file and printing size..."
  COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
  COMMAND ${SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
)
