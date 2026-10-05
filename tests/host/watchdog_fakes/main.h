#ifndef MC02_HOST_WATCHDOG_FAKE_MAIN_H
#define MC02_HOST_WATCHDOG_FAKE_MAIN_H

#include <stdint.h>

typedef struct
{
    volatile uint32_t CYCCNT;
} DWT_TypeDef;

typedef struct
{
    volatile uint32_t KR;
    volatile uint32_t PR;
    volatile uint32_t RLR;
    volatile uint32_t SR;
} IWDG_TypeDef;

typedef struct
{
    volatile uint32_t APB4FZ1;
} DBGMCU_TypeDef;

typedef struct
{
    volatile uint32_t RSR;
} RCC_TypeDef;

extern DWT_TypeDef mc02_test_dwt;
extern IWDG_TypeDef mc02_test_iwdg;
extern DBGMCU_TypeDef mc02_test_dbgmcu;
extern RCC_TypeDef mc02_test_rcc;
extern uint32_t mc02_test_tick;
extern uint32_t mc02_test_primask;
DWT_TypeDef *MC02TestDWT(void);

#define DWT (MC02TestDWT())
#define IWDG1 (&mc02_test_iwdg)
#define DBGMCU (&mc02_test_dbgmcu)
#define RCC (&mc02_test_rcc)

#define DBGMCU_APB4FZ1_DBG_IWDG1 (1u << 12u)
#define IWDG_SR_PVU (1u << 0u)
#define IWDG_SR_RVU (1u << 1u)
#define IWDG_SR_WVU (1u << 2u)
#define RCC_RSR_IWDG1RSTF (1u << 0u)
#define RCC_RSR_PORRSTF (1u << 1u)
#define RCC_RSR_PINRSTF (1u << 2u)
#define RCC_RSR_BORRSTF (1u << 3u)
#define RCC_RSR_RMVF (1u << 16u)

#define HAL_GetTick() (mc02_test_tick)

#endif
