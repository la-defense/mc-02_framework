/** @file platform_numeric_caps_test.c
 * @brief 检查板级数值硬件与主机编译目标能力分开报告。
 */
#include "platform_numeric_caps.h"
#include "test_check.h"

int main(void)
{
    const PlatformNumericCapabilities_t caps = PlatformNumericGetCapabilities(); // 读取当前板卡声明。
    const uint32_t h723_hardware = PLATFORM_NUMERIC_HW_FPU | PLATFORM_NUMERIC_HW_DSP |
                                  PLATFORM_NUMERIC_HW_CORDIC | PLATFORM_NUMERIC_HW_FMAC; // 核对 H723 的四类硬件。
    TEST_CHECK(caps.hardware == h723_hardware); // 硬件存在信息不能随主机架构消失。
    TEST_CHECK((caps.build & PLATFORM_NUMERIC_BUILD_CMSIS) != 0u); // 当前项目选择 CMSIS-DSP 实现。
#if !defined(__arm__) && !defined(__aarch64__)
    TEST_CHECK((caps.build & (PLATFORM_NUMERIC_BUILD_FPU | PLATFORM_NUMERIC_BUILD_DSP |
                             PLATFORM_NUMERIC_BUILD_HARD_FLOAT_ABI)) == 0u); // 非 ARM 主机不能宣称 ARM 编译能力。
#endif
    return 0;
}
