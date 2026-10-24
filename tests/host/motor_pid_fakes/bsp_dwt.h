/** @file bsp_dwt.h
 * @brief 以受控微秒时间替代硬件 DWT，统计驱动实际读时钟次数。
 */
#ifndef MC02_PID_TEST_DWT_H
#define MC02_PID_TEST_DWT_H
#include <stdint.h>
typedef struct { uint32_t unused; } DWT_Probe_t;
extern volatile uint32_t dwt_cpu_freq_mhz;
/** @brief 读取并统计测试时钟调用。
 * @return 测试设置的当前微秒数。
 */
uint64_t DWT_GetTimeline_us(void);
/** @brief 模拟反馈计时，与控制计时分别使用。
 * @param[in,out] count 有效的反馈计数对象。
 * @return 固定反馈间隔0.001秒。
 */
float DWT_GetDeltaT(uint32_t *count);
/** @brief 返回测试探针起点。
 * @return 固定为0，不访问硬件。
 */
uint32_t DWT_ProbeStart(void);
/** @brief 保留生产性能探针调用，测试不累计耗时。
 * @param[in,out] probe 测试探针对象，替身不修改。
 * @param[in] start 未使用的起点。
 */
void DWT_ProbeDone(volatile DWT_Probe_t *probe, uint32_t start);
#endif
