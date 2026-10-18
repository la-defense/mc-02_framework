/** @file cmsis_os.h
 * @brief 让HT04生产文件通过主机编译；任务创建仅返回占位对象，不执行任务函数。
 */
#ifndef MC02_HT_NUMERIC_OS_H
#define MC02_HT_NUMERIC_OS_H
#include <stdint.h>
typedef void *osThreadId;
#define osPriorityNormal 0
#define osThreadDef(name,function,priority,count,stack) const int os_def_##name=0
#define osThread(name) (&os_def_##name)
/** @brief 返回未启动的占位任务。
 * @param[in] definition 宏生成的测试声明。
 * @param[in] argument 原生产实参，测试不使用。
 * @return 固定空指针。
 */
void *osThreadCreate(const int *definition,void *argument);
/** @brief 保留生产任务延时调用，测试不运行任务。
 * @param[in] milliseconds 原延时毫秒数，替身不等待。
 */
void osDelay(uint32_t milliseconds);
/** @brief 保留旧驱动使用的非标准整数格式化入口。
 * @param[in] value 测试仅支持0～3。
 * @param[out] output 至少2字节。
 * @param[in] base 测试仅支持10。
 * @return output原地址。
 */
char *__itoa(int value,char *output,int base);
#endif
