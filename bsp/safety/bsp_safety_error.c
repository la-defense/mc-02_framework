#include "main.h"

#include "bsp_safety.h"

void Error_Handler(void)
{
    __disable_irq();
    BSP_SafetyLatchOutputsOff();

#if defined(MC02_HOST_TEST)
    return;
#else
    for (;;)
        ;
#endif
}
