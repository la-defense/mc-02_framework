/** @file ws2812.c
* @brief 将三色各八位编成 24 个 SPI 字节，再发送低电平字节让彩灯接收新颜色。
* @note PA5 是 ADC1_INP19 的按键输入；初始化只把 PA7 配为 SPI6 MOSI，SCK 不映射到 GPIO。
*/
#include "ws2812.h"
#include "stm32h7xx_hal.h"
#include "bsp_hal_tick.h"

#define WS2812_LowLevel    0xC0     // 编码 0
#define WS2812_HighLevel   0xF0     // 编码 1

static SPI_HandleTypeDef hspi6;
static uint8_t spi6_inited = 0;

/* SPI6 时钟由外设内部使用；只把数据输出接到 PA7，保留 PA5 的 ADC 模拟输入。 */
static HAL_StatusTypeDef WS2812_SPI6_Init(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* SPI6 时钟源选择 HSE */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
    PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        return HAL_ERROR;

    __HAL_RCC_SPI6_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA5 接 ADC 按键采样；此处只配置 WS2812 数据线 PA7。 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_SPI6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hspi6.Instance = SPI6;
    hspi6.Init.Mode = SPI_MODE_MASTER;
    hspi6.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
    hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi6.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi6.Init.NSS = SPI_NSS_SOFT;
    hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi6.Init.CRCPolynomial = 0x0;
    hspi6.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi6) != HAL_OK)
        return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef WS2812_Init(void)
{
    if (spi6_inited != 0u)
        return HAL_OK;

    const HAL_StatusTypeDef status = WS2812_SPI6_Init();
    if (status == HAL_OK)
        spi6_inited = 1u;
    return status;
}

HAL_StatusTypeDef WS2812_SetRGB(uint8_t r, uint8_t g, uint8_t b)
{
    if (!spi6_inited)
        return HAL_ERROR;

    uint8_t txbuf[124] = {0u};
    // 数组先排绿、红、蓝，每组倒序填入，让每种颜色的最高位最先发送。
    for (int i = 0; i < 8; i++)
    {
        txbuf[7 - i]  = (((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[15 - i] = (((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[23 - i] = (((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    }
    /* HAL 超时依赖毫秒时基；时基不可用时不进入可能无限等待的发送。 */
    if (BSP_HALTickTryAcquireBlockingWait() == 0u)
        return HAL_BUSY;
    const HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi6, txbuf,
                                                       (uint16_t)sizeof(txbuf), 2u);
    BSP_HALTickReleaseBlockingWait();
    return result;
}
