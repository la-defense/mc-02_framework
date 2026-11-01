/** @file board_numeric_caps_mc02.c
 * @brief 声明 MC-02 的 H723 数值硬件与当前编译目标能力。
 */
#include "platform_numeric_caps.h"

PlatformNumericCapabilities_t PlatformNumericGetCapabilities(void)
{
    PlatformNumericCapabilities_t caps = {
        .hardware = PLATFORM_NUMERIC_HW_FPU | PLATFORM_NUMERIC_HW_DSP |
                    PLATFORM_NUMERIC_HW_CORDIC | PLATFORM_NUMERIC_HW_FMAC, // H723 芯片具备的硬件。
        .build = PLATFORM_NUMERIC_BUILD_CMSIS, // 当前数值接口选择同源 CMSIS-DSP 后端。
    };
#if defined(__ARM_FP) && ((__ARM_FP & 4) != 0)
    caps.build |= PLATFORM_NUMERIC_BUILD_FPU; // 编译器允许使用单精度浮点指令。
#endif
#if defined(__ARM_FEATURE_DSP) && (__ARM_FEATURE_DSP != 0)
    caps.build |= PLATFORM_NUMERIC_BUILD_DSP; // 编译器目标允许使用核心 DSP 指令。
#endif
#if defined(__ARM_PCS_VFP)
    caps.build |= PLATFORM_NUMERIC_BUILD_HARD_FLOAT_ABI; // 调用约定通过浮点寄存器传递参数。
#endif
    return caps; // 外设是否初始化不由此硬件清单推断。
}
