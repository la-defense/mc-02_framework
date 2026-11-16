#ifndef MC02_SHOOT_FAKE_MAIN_H
#define MC02_SHOOT_FAKE_MAIN_H
#include <stdint.h>
typedef struct { uint32_t marker; } CAN_HandleTypeDef;
typedef struct { uint32_t marker; } UART_HandleTypeDef;
extern CAN_HandleTypeDef hcan2;
uint32_t HAL_GetTick(void);
static inline uint32_t __get_PRIMASK(void) { return 0u; } // 单线程主机替身返回未屏蔽状态。
static inline void __disable_irq(void) {} // 本测试没有真实中断。
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; } // 保持平台保存／恢复接口可用。
#endif
