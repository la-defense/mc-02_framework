/**
 * @file bsp_pwm.h
* @brief PWM 实例注册、周期及占空比设置；底层定时器、引脚及 DMA 须先配置。
 * @version 0.1
 * @date 2023-02-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "tim.h"
#include "stdint.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h723xx.h"
#define PWM_DEVICE_CNT 16 // 最大支持的PWM实例数量

/* pwm实例结构体 */
typedef struct pwm_ins_temp
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    uint32_t tclk;                           // 时钟频率
    float period;                         // 周期
    float dutyratio;                      // 占空比
    void (*callback)(struct pwm_ins_temp *); // DMA传输完成回调函数
    void *id;                                // 实例ID
} PWMInstance;

typedef struct
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    float period;                         // 周期
    float dutyratio;                      // 占空比
    void (*callback)(PWMInstance*); // DMA传输完成回调函数
    void *id;                                // 实例ID
} PWM_Init_Config_s;

/** @brief 检查配置、注册并启动 PWM，启动前先保持占空比为 0。
* @param[in] config 有效定时器、通道 1～4，period 为大于 0 的秒数，dutyratio 为 0～1，均不能为 NaN 或无穷大。
* @return 成功返回长期有效实例；参数、重名、容量、分配、计数范围或启动失败返回 NULL。
* @note 实例最多 16 个；成功后才写入请求的占空比，失败时清理本次分配。
 */
PWMInstance *PWMRegister(PWM_Init_Config_s *config);
/** @brief 占空比设为 0、停 PWM，移出注册表并释放对象。
* @param[in] pwm 已停止 DMA 及相关回调的实例；成功后不可继续使用。
* @return 1 为已释放，0 为 NULL 或未注册；不等待异步传输结束。
 */
uint8_t PWMUnregister(PWMInstance *pwm);

/** @brief 调用 HAL 启动指定 PWM，不返回 HAL 结果。
* @param[in] pwm 有效已注册实例，不能为 NULL。
 */
void PWMStart(PWMInstance *pwm);
/** @brief 按当前 ARR 写比较值，不检查范围、不修改对象中的配置字段。
* @param[in] pwm 有效实例，不能为 NULL。
* @param[in] dutyratio 正常数值，调用方须保证在 0～1，1 表示比较值等于 ARR。
 */

void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio);
/** @brief 调用 HAL 停止指定 PWM，不返回 HAL 结果。
* @param[in] pwm 有效已注册实例，不能为 NULL。
 */
void PWMStop(PWMInstance *pwm);

/** @brief 根据已保存的时钟和预分频写 ARR，不检查新数值，也不修改对象的 period 字段。
* @param[in] pwm 有效实例，不能为 NULL。
* @param[in] period 周期，单位 s，须为正常正数并适合该定时器计数范围。
 */
void PWMSetPeriod(PWMInstance *pwm, float period);

/** @brief 从数组启动 PWM DMA，接口不返回 HAL 状态。
* @param[in] pwm 有效实例，定时器和 DMA 须事先配置。
* @param[in] pData 有效数据地址，DMA 完成前不得改写或释放，存放宽度须与 DMA 设置相同。
* @param[in] Size 传输元素个数，单位由 DMA 数据宽度决定，不一定是字节数。
 */
void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size);

#endif // BSP_PWM_H
