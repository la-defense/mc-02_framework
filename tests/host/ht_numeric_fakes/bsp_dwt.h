/** @file bsp_dwt.h
 * @brief HT04测试时钟；反馈读时钟计数和控制时钟分别提供。
 */
#ifndef MC02_HT_NUMERIC_DWT_H
#define MC02_HT_NUMERIC_DWT_H
#include <stdint.h>
/** @brief 返回模拟控制时刻。
 * @return 固定0微秒；测试不运行控制任务。
 */
uint64_t DWT_GetTimeline_us(void);
/** @brief 更新模拟反馈计时。
 * @param[in,out] counter 非空反馈计数对象，调用时增加1。
 * @return 固定0.001秒。
 */
float DWT_GetDeltaT(uint32_t *counter);
/** @brief 保留驱动初始化等待，替身立即返回。
 * @param[in] seconds 原等待秒数，不实际等待。
 */
void DWT_Delay(float seconds);
#endif
