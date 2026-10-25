/*
 * @file main.h
 * @brief 为复位恢复生产模块提供可控的H723寄存器模型。
 * @note 仅用于主机测试；寄存器值由测试场景设置，不代表板上时序。
 */
#ifndef MC02_RESET_RECOVERY_FAKE_MAIN_H
#define MC02_RESET_RECOVERY_FAKE_MAIN_H

#include <stdint.h>

typedef struct
{
    volatile uint32_t CCR;
} MC02_ResetRecoverySCB_t;

extern MC02_ResetRecoverySCB_t mc02_reset_recovery_scb;
extern uint32_t mc02_reset_recovery_primask;
extern uint32_t mc02_reset_recovery_dsb_count;

#define SCB (&mc02_reset_recovery_scb)
#define SCB_CCR_DC_Msk (1u << 16)
#define RCC_RSR_PORRSTF (1u << 23)
#define RCC_RSR_PINRSTF (1u << 22)
#define RCC_RSR_BORRSTF (1u << 21)
#define RCC_RSR_IWDG1RSTF (1u << 26)
#define RCC_RSR_WWDG1RSTF (1u << 28)

static inline uint32_t __get_PRIMASK(void)
{
    return mc02_reset_recovery_primask;
}

static inline void __disable_irq(void)
{
    mc02_reset_recovery_primask = 1u;
}

static inline void __set_PRIMASK(uint32_t value)
{
    mc02_reset_recovery_primask = value;
}

static inline void __DSB(void)
{
    mc02_reset_recovery_dsb_count++;
}

#endif
