/** @file cmsis_os.h
 * @brief 协议测试记录发送，不进行实际任务延时。
 */
#include <stdint.h>
/** @brief 保留调用顺序而不实际等待。
 * @param[in] milliseconds 生产调用请求的等待毫秒数。
 */
void osDelay(uint32_t milliseconds);
