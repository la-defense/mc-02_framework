/**
 * @file bsp_init_link_test.c
 * @brief 主机验证两个源文件的链接及启动顺序；每个进程只调用一次 BSPInit。
 * @note 参数 a/b 选择调用方，底层只记录调用，不操作硬件。
 */
#include "test_check.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_watchdog.h"
#include "bsp_log.h"
#include "bsp_adc.h"
#include "bsp_crash.h"
#include "bsp_param.h"
#include <string.h>

void BSPInitCallerA(void);
void BSPInitCallerB(void);

static unsigned step;

void DWT_Init(uint32_t cpu_mhz)
{
    TEST_CHECK(step == 0u);
    TEST_CHECK(cpu_mhz == 480u);
    ++step;
}

void BSP_WatchdogInit(uint32_t timeout_ms)
{
    TEST_CHECK(step == 3u);
    TEST_CHECK(timeout_ms == 4000u);
    ++step;
}

void BSPLogInit(void)
{
    TEST_CHECK(step == 2u);
    ++step;
}

void CANInitWorkspace(void)
{
    TEST_CHECK(step == 1u);
    ++step;
}

void BSP_ADCInit(void)
{
    TEST_CHECK(step == 4u);
    ++step;
}

void BSP_WatchdogLogResetReason(void)
{
    TEST_CHECK(step == 5u);
    ++step;
}

void CrashLogInit(void)
{
    TEST_CHECK(step == 6u);
    ++step;
}

uint8_t ParamInit(void)
{
    TEST_CHECK(step == 7u);
    ++step;
    // 即使参数载入失败，原有 void 启动入口也不汇总失败，不能据此宣称整体成功。
    return 0u;
}

/** @brief 选择一个调用方；任何顺序、参数或次数不符均以非零退出。 */
int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "a") == 0)
        BSPInitCallerA();
    else
    {
        TEST_CHECK(strcmp(argv[1], "b") == 0);
        BSPInitCallerB();
    }
    TEST_CHECK(step == 8u);
    return 0;
}
