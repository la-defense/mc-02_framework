/** @file platform_numeric_caps.h
 * @brief 查询所选板卡的数值硬件和当前固件编译能力。
 */
#ifndef MC02_PLATFORM_NUMERIC_CAPS_H
#define MC02_PLATFORM_NUMERIC_CAPS_H
#include <stdint.h>

#define PLATFORM_NUMERIC_HW_FPU      (UINT32_C(1) << 0) /**< 芯片带单精度浮点运算单元。 */
#define PLATFORM_NUMERIC_HW_DSP      (UINT32_C(1) << 1) /**< 芯片核心支持 DSP 指令。 */
#define PLATFORM_NUMERIC_HW_CORDIC   (UINT32_C(1) << 2) /**< 芯片带 CORDIC 外设。 */
#define PLATFORM_NUMERIC_HW_FMAC     (UINT32_C(1) << 3) /**< 芯片带 FMAC 外设。 */

#define PLATFORM_NUMERIC_BUILD_CMSIS (UINT32_C(1) << 0) /**< 已选择同源 CMSIS-DSP 数值后端。 */
#define PLATFORM_NUMERIC_BUILD_FPU   (UINT32_C(1) << 1) /**< 当前编译目标允许单精度浮点指令。 */
#define PLATFORM_NUMERIC_BUILD_DSP   (UINT32_C(1) << 2) /**< 当前编译目标允许核心 DSP 指令。 */
#define PLATFORM_NUMERIC_BUILD_HARD_FLOAT_ABI (UINT32_C(1) << 3) /**< 当前目标使用硬浮点参数传递。 */

typedef struct {
    uint32_t hardware; /**< 所选板卡芯片具备的硬件，位值为 PLATFORM_NUMERIC_HW_*。 */
    uint32_t build;    /**< 本编译目标可使用的后端或指令，位值为 PLATFORM_NUMERIC_BUILD_*。 */
} PlatformNumericCapabilities_t;

/** @brief 读取本板数值硬件和编译目标声明。
 * @return 按值返回固定能力；硬件存在不表示相关外设已初始化或被算法使用。
 * @note 当前 CORDIC 和 FMAC 尚未接入数值后端，不在此接口中报告为已启用。
 */
PlatformNumericCapabilities_t PlatformNumericGetCapabilities(void);
#endif
