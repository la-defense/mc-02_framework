/** @file fdcan_msp_test.c
 * @brief 直接编译生产 MSP 代码，检查共享时钟和每条总线的重复退出。
 */
#include "fdcan.h"
#include "test_check.h"
#include <stddef.h>

FDCAN_GlobalTypeDef fake_fdcan_devices[3];
static uint32_t clock_config_count, clock_enable_count, clock_disable_count;
static uint32_t gpio_init_count, gpio_deinit_count, errors;
static uint32_t irq_enabled, irq_disabled, irq_cleared;
static uint8_t config_fails;

void FakeFdcanClockEnable(void) { ++clock_enable_count; }
void FakeFdcanClockDisable(void)
{
    TEST_CHECK((irq_disabled & 0x0fu) == 0x0fu); // 最后退出之前，两条已用总线均已关闭中断。
    TEST_CHECK((irq_cleared & 0x0fu) == 0x0fu); // 时钟关闭前不遗留已启用总线的待处理事件。
    ++clock_disable_count; // 记录共享时钟真正关闭的次数。
}
uint32_t HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config)
{
    TEST_CHECK(config != NULL && config->PeriphClockSelection == RCC_PERIPHCLK_FDCAN);
    TEST_CHECK(config->FdcanClockSelection == RCC_FDCANCLKSOURCE_PLL);
    ++clock_config_count; // 记录共享时钟源实际配置次数。
    return config_fails ? HAL_ERROR : HAL_OK; // 可注入时钟源配置失败。
}
uint32_t HAL_FDCAN_Init(FDCAN_HandleTypeDef *handle) { (void)handle; return HAL_OK; }
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config)
{ TEST_CHECK(port != NULL && config != NULL); ++gpio_init_count; }
void HAL_GPIO_DeInit(void *port, uint32_t pins)
{
    TEST_CHECK(port != NULL && pins != 0u); // 不允许使用无效 GPIO 参数。
    if (port == GPIOD && pins == (GPIO_PIN_0 | GPIO_PIN_1))
        TEST_CHECK((irq_disabled & 0x03u) == 0x03u && (irq_cleared & 0x03u) == 0x03u);
    if (port == GPIOB && pins == (GPIO_PIN_5 | GPIO_PIN_6))
        TEST_CHECK((irq_disabled & 0x0cu) == 0x0cu && (irq_cleared & 0x0cu) == 0x0cu);
    ++gpio_deinit_count; // 记录已完成中断清理的 GPIO 退出次数。
}
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority)
{ (void)irq; (void)priority; (void)subpriority; }
void HAL_NVIC_EnableIRQ(int irq) { irq_enabled |= 1u << (unsigned)irq; }
void HAL_NVIC_DisableIRQ(int irq) { irq_disabled |= 1u << (unsigned)irq; }
void HAL_NVIC_ClearPendingIRQ(int irq)
{
    TEST_CHECK((irq_disabled & (1u << (unsigned)irq)) != 0u); // 先禁止再清除，避免旧中断重新挂起。
    irq_cleared |= 1u << (unsigned)irq; // 记录此中断已清除。
}
void Error_Handler(void) { ++errors; }

int main(void)
{
    MX_FDCAN1_Init(); MX_FDCAN2_Init(); MX_FDCAN3_Init(); // 读取生产初始化提供的三份消息 RAM 配置。
    FDCAN_HandleTypeDef *handles[3] = {&hfdcan1, &hfdcan2, &hfdcan3}; // 按总线顺序检查。
    const uint32_t offsets[3] = {0u, 0x406u, 0x812u}; // 对应共享消息 RAM 中互不重叠的起点。
    for (uint32_t index = 0u; index < 3u; ++index)
    {
        const FDCAN_InitTypeDef *init = &handles[index]->Init; // 读取该控制器的实际初始化参数。
        TEST_CHECK(init->MessageRAMOffset == offsets[index]); // 核对各控制器独立的 RAM 起点。
        TEST_CHECK(init->StdFiltersNbr == 14u && init->ExtFiltersNbr == 16u); // 两类过滤器都有固定容量。
        TEST_CHECK(init->RxFifo0ElmtsNbr == 4u && init->RxFifo1ElmtsNbr == 4u); // 保持每个 FIFO 四帧。
        TEST_CHECK(init->RxFifo0ElmtSize == FDCAN_DATA_BYTES_64 && init->RxFifo1ElmtSize == FDCAN_DATA_BYTES_64);
        TEST_CHECK(init->TxFifoQueueElmtsNbr == 16u && init->TxElmtSize == FDCAN_DATA_BYTES_8); // 发送仍为经典 CAN。
        const uint32_t words = 14u + 2u * 16u + 8u * 18u + 16u * 4u; // 过滤器、接收和发送区域的字数。
        const uint32_t end = index < 2u ? offsets[index + 1u] : 0x2800u / 4u; // 最后一段不能越过 SRAMCAN 尾部。
        TEST_CHECK(init->MessageRAMOffset + words <= end); // 所有控制器的实际区域不得重叠。
    }
    hfdcan1.Instance = FDCAN1; // 设置第一条总线的真实控制器身份。
    hfdcan2.Instance = FDCAN2; // 设置第二条总线的真实控制器身份。
    hfdcan3.Instance = FDCAN3; // 第三条保持未初始化，用于检查空退出。
    config_fails = 1u; // 首次时钟配置失败不能占用资源。
    HAL_FDCAN_MspInit(&hfdcan1);
    TEST_CHECK(errors == 1u && clock_enable_count == 0u && gpio_init_count == 0u);
    config_fails = 0u; // 修复后重新初始化第一条总线。
    HAL_FDCAN_MspInit(&hfdcan1);
    TEST_CHECK(clock_config_count == 2u && clock_enable_count == 1u && gpio_init_count == 1u);
    HAL_FDCAN_MspInit(&hfdcan1); // 同一控制器重复初始化不得增加共享占用。
    HAL_FDCAN_MspInit(&hfdcan2); // 第二条使用已有共享时钟。
    TEST_CHECK(clock_config_count == 2u && clock_enable_count == 1u && gpio_init_count == 2u);
    TEST_CHECK((irq_enabled & 0x0fu) == 0x0fu); // 两条总线各自使能两个中断。
    HAL_FDCAN_MspDeInit(&hfdcan1); // 停止第一条不能影响第二条时钟。
    TEST_CHECK(clock_disable_count == 0u && gpio_deinit_count == 1u);
    TEST_CHECK((irq_disabled & 0x03u) == 0x03u && (irq_cleared & 0x03u) == 0x03u);
    TEST_CHECK((irq_disabled & 0x0cu) == 0u); // 第二条总线中断仍保持启用。
    HAL_FDCAN_MspDeInit(&hfdcan1); // 重复退出第一条不能多减一次引用。
    HAL_FDCAN_MspDeInit(&hfdcan3); // 从未启用的总线不能关闭共享时钟。
    TEST_CHECK(clock_disable_count == 0u && gpio_deinit_count == 1u);
    HAL_FDCAN_MspDeInit(&hfdcan2); // 最后一个控制器退出才关闭时钟。
    TEST_CHECK(clock_disable_count == 1u && gpio_deinit_count == 2u);
    TEST_CHECK((irq_cleared & 0x0cu) == 0x0cu); // 中断退出时也清除待处理状态。
    HAL_FDCAN_MspInit(&hfdcan1); // 所有使用者退出后允许重新打开。
    TEST_CHECK(clock_config_count == 3u && clock_enable_count == 2u);
    return 0;
}
