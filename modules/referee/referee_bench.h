/** @file referee_bench.h
 * @brief 裸板裁判有限负载入口；仅具名refdiag禁热固件启用，正常配置不发送测试报文。
 */
#ifndef MC02_REFEREE_BENCH_H
#define MC02_REFEREE_BENCH_H
#include <stdint.h>
#include "../../bsp/board/platform_resources.h"
/** @brief 启动阶段绑定USART1接收，不发送、不请求READY。
 * @param[in] port 板级绑定的逻辑串口，外部须连接3.3V TTL、共地且VCC悬空。
 * @return 1初始化成功；0沿应用既有初始化失败路径停止。
 */
uint8_t RefereeBenchInit(PlatformUartPort_t port);
/** @brief 现有RobotTask每轮最多提交一帧；首次新鲜身份后只运行75秒，不自动重启。
 * @note 前30秒满长交互，后45秒混合空操作UI；诊断结果由匹配ELF只读查看。
 */
void RefereeBenchTask(void);
#endif
