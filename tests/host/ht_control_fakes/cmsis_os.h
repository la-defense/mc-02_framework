/** @file cmsis_os.h
 * @brief 捕获 HT04 任务入口，允许主机测试按有限轮数调用生产控制循环。
 */
#ifndef MC02_HT_CONTROL_OS_H
#define MC02_HT_CONTROL_OS_H
#include <stdint.h>
typedef void *osThreadId;
typedef void (*osTaskFunction_t)(void const *argument);
typedef struct
{
    osTaskFunction_t entry;
} osThreadDef_t;
#define osPriorityNormal 0
#define osThreadDef(name, thread_fn, priority, count, stack) \
    const osThreadDef_t os_def_##name = {(thread_fn)}
#define osThread(name) (&os_def_##name)
/** @brief 捕获任务入口和实参，不在创建时启动任务。
 * @param[in] definition 由 osThreadDef 声明的入口对象。
 * @param[in] argument 生产代码传入的 HT 实例指针。
 * @return 本测试保存的任务槽位地址，达到容量时测试失败。
 */
void *osThreadCreate(const osThreadDef_t *definition, void *argument);
/** @brief 推进模拟时间，达到设定轮数时退出测试任务。
 * @param[in] milliseconds 生产循环要求等待的毫秒数。
 */
void osDelay(uint32_t milliseconds);
/** @brief 保留 HT04 的旧整数格式化依赖。
 * @param[in] value 测试支持的任务索引0～1。
 * @param[out] output 至少2字节的字符缓冲区。
 * @param[in] base 测试固定使用十进制10。
 * @return 返回 output 原地址。
 */
char *__itoa(int value, char *output, int base);
#endif
