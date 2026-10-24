/** @file main.h
 * @brief DWT 累计计时并发测试的 Cortex-M 寄存器与临界区替身。
 * @note 仅由 WSL pthread 测试使用，不参与固件构建。
 */
#ifndef MC02_DWT_THREAD_MAIN_H
#define MC02_DWT_THREAD_MAIN_H

#include <stdatomic.h>
#include <stdint.h>

typedef struct
{
    _Atomic uint32_t CYCCNT;
    uint32_t CTRL;
} DWT_Type;

typedef struct
{
    uint32_t DEMCR;
} CoreDebug_Type;

extern DWT_Type host_dwt;
extern CoreDebug_Type host_core_debug;
extern uint32_t HostGetPrimask(void);
extern void HostDisableIrq(void);
extern void HostSetPrimask(uint32_t mask);

#define DWT (&host_dwt)
#define CoreDebug (&host_core_debug)
#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24)
#define DWT_CTRL_CYCCNTENA_Msk (1u << 0)
#define __get_PRIMASK() HostGetPrimask()
#define __disable_irq() HostDisableIrq()
#define __set_PRIMASK(mask) HostSetPrimask(mask)

#endif
