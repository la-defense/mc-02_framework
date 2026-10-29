/** @file ws2812.c
* @brief 将三色各八位编成 24 个 SPI 字节，再发送低电平字节让彩灯接收新颜色。
* @note PA5 是 ADC1_INP19 的按键输入；初始化只把 PA7 配为 SPI6 MOSI，SCK 不映射到 GPIO。
*/
#include "ws2812.h"
#include "stm32h7xx_hal.h"
#include "bsp_hal_tick.h"
#include <string.h>

#define WS2812_LowLevel    0xC0     // 编码 0
#define WS2812_HighLevel   0xF0     // 编码 1

static SPI_HandleTypeDef hspi6;
static DMA_HandleTypeDef hdma_spi6_tx;
static uint8_t spi6_inited = 0;

// BDMA 只可访问 D3 域；链接脚本将此 NOLOAD 段放入 SRAM4。
static uint8_t ws2812_dma_buffer[128] __attribute__((section(".ws2812_dma"), aligned(32)));

typedef enum
{
    WS2812_IDLE = 0,
    WS2812_STARTING,
    WS2812_ACTIVE,
    WS2812_COMPLETE,
    WS2812_RECOVERY,
    WS2812_RECOVERING,
    WS2812_FAILED,
    WS2812_ISOLATED,
} WS2812Phase_t;

static volatile WS2812Phase_t phase;
static uint32_t started_ms;
static uint32_t recovery_started_ms;
static uint8_t active_r, active_g, active_b;
static uint8_t completed_r, completed_g, completed_b;
static uint8_t completed_valid;
static uint8_t tick_lease;
static volatile HAL_StatusTypeDef last_result;
static uint8_t WS2812_Stopped(void);

// 中断只登记结果；短临界区防止任务和中断同时改变事务阶段。
static uint32_t WS2812_Enter(void)
{
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}

// 恢复进入临界区之前的中断状态。
static void WS2812_Exit(uint32_t saved)
{
    __set_PRIMASK(saved);
}

/* SPI6 时钟由外设内部使用；只把数据输出接到 PA7，保留 PA5 的 ADC 模拟输入。 */
static HAL_StatusTypeDef WS2812_SPI6_Init(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_SPI6_CLK_ENABLE();
    __HAL_RCC_BDMA_CLK_ENABLE();
    // 活动中的 SPI6 或 BDMA 通道不能被初始化过程改写。
    if ((SPI6->CR1 & (SPI_CR1_SPE | SPI_CR1_CSTART)) != 0u ||
        (BDMA_Channel0->CCR & BDMA_CCR_EN) != 0u)
        return HAL_BUSY;

    // 使用 HSE 作为 SPI6 时钟源，保持现有灯带编码速率配置。
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
    PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        return HAL_ERROR;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    // PA5 接 ADC 按键采样；此处只配置 WS2812 数据线 PA7。
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

    hdma_spi6_tx.Instance = BDMA_Channel0;
    hdma_spi6_tx.Init.Request = BDMA_REQUEST_SPI6_TX;
    hdma_spi6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi6_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi6_tx.Init.Mode = DMA_NORMAL;
    hdma_spi6_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi6_tx) != HAL_OK)
        return HAL_ERROR;
    __HAL_LINKDMA(&hspi6, hdmatx, hdma_spi6_tx);
    HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, 5u, 0u);
    HAL_NVIC_SetPriority(SPI6_IRQn, 5u, 0u);
    HAL_NVIC_ClearPendingIRQ(BDMA_Channel0_IRQn);
    HAL_NVIC_ClearPendingIRQ(SPI6_IRQn);
    HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);
    HAL_NVIC_EnableIRQ(SPI6_IRQn);
    return HAL_OK;
}

HAL_StatusTypeDef WS2812_Init(void)
{
    if (spi6_inited != 0u)
        return HAL_OK;

    // 目前没有缓存维护流程；缓存开启时不让 BDMA 使用可能不可见的数据。
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u)
        return HAL_ERROR;

    const HAL_StatusTypeDef status = WS2812_SPI6_Init();
    if (status == HAL_OK)
        spi6_inited = 1u;
    return status;
}

HAL_StatusTypeDef WS2812_SetRGB(uint8_t r, uint8_t g, uint8_t b)
{
    if (!spi6_inited || __get_IPSR() != 0u)
        return HAL_ERROR;

    uint32_t saved = WS2812_Enter();
    if (phase == WS2812_FAILED || phase == WS2812_ISOLATED)
    {
        WS2812_Exit(saved);
        return HAL_ERROR;
    }
    if (phase != WS2812_IDLE)
    {
        WS2812_Exit(saved);
        return HAL_BUSY;
    }
    // 空闲状态必须和真实硬件一致；不覆盖可能仍被 BDMA 读取的数组。
    if (WS2812_Stopped() == 0u || (SCB->CCR & SCB_CCR_DC_Msk) != 0u)
    {
        phase = WS2812_ISOLATED;
        last_result = HAL_ERROR;
        WS2812_Exit(saved);
        return HAL_ERROR;
    }
    if (completed_valid != 0u && completed_r == r && completed_g == g && completed_b == b)
    {
        WS2812_Exit(saved);
        return HAL_OK;
    }
    WS2812_Exit(saved);

    // 空闲时才改写 SRAM4 数组；发送开始后直到完成或确认停止都不复用。
    memset(ws2812_dma_buffer, 0, sizeof(ws2812_dma_buffer));
    // 数组先排绿、红、蓝，每组倒序填入，让每种颜色的最高位最先发送。
    for (int i = 0; i < 8; i++)
    {
        ws2812_dma_buffer[7 - i]  = (((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        ws2812_dma_buffer[15 - i] = (((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        ws2812_dma_buffer[23 - i] = (((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    }
    if (BSP_HALTickTryAcquireBlockingWait() == 0u)
        return HAL_BUSY;

    saved = WS2812_Enter();
    tick_lease = 1u;
    active_r = r; active_g = g; active_b = b;
    started_ms = HAL_GetTick();
    phase = WS2812_STARTING;
    last_result = HAL_BUSY;
    WS2812_Exit(saved);

    // 在启动 BDMA 前完成 SRAM4 写入；当前仅在 DCache 关闭时使用该缓冲。
    __DMB();
    const HAL_StatusTypeDef result = HAL_SPI_Transmit_DMA(&hspi6, ws2812_dma_buffer, 124u);
    uint8_t release = 0u;
    saved = WS2812_Enter();
    if (result == HAL_OK && phase == WS2812_STARTING)
        phase = WS2812_ACTIVE;
    else if (result == HAL_BUSY && phase == WS2812_STARTING)
    {
        phase = WS2812_IDLE;
        tick_lease = 0u;
        release = 1u;
    }
    else if (result != HAL_OK)
    {
        phase = WS2812_RECOVERY;
        last_result = HAL_ERROR;
    }
    WS2812_Exit(saved);
    if (release != 0u)
        BSP_HALTickReleaseBlockingWait();
    return HAL_BUSY;
}

// READY 还需配合寄存器和 BDMA 通道检查，才能确认数组不再被硬件读取。
static uint8_t WS2812_Stopped(void)
{
    return (uint8_t)(HAL_SPI_GetState(&hspi6) == HAL_SPI_STATE_READY &&
        HAL_DMA_GetState(&hdma_spi6_tx) == HAL_DMA_STATE_READY &&
        (hspi6.Instance->CR1 & (SPI_CR1_SPE | SPI_CR1_CSTART)) == 0u &&
        (hspi6.Instance->CFG1 & SPI_CFG1_TXDMAEN) == 0u &&
        (hspi6.Instance->IER & (SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP |
            SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF)) == 0u &&
        (((BDMA_Channel_TypeDef *)hdma_spi6_tx.Instance)->CCR & BDMA_CCR_EN) == 0u);
}

// 硬件确认停止后清除本通道残留的 DMA 标志和待处理中断。
static void WS2812_ClearPending(void)
{
    __HAL_DMA_CLEAR_FLAG(&hdma_spi6_tx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_spi6_tx) |
        __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_spi6_tx) | __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_spi6_tx));
    HAL_NVIC_ClearPendingIRQ(BDMA_Channel0_IRQn);
    HAL_NVIC_ClearPendingIRQ(SPI6_IRQn);
}

HAL_StatusTypeDef WS2812_Process(void)
{
    if (!spi6_inited || __get_IPSR() != 0u)
        return HAL_ERROR;
    uint32_t saved = WS2812_Enter();
    if (phase == WS2812_ACTIVE && (SCB->CCR & SCB_CCR_DC_Msk) != 0u)
    {
        phase = WS2812_RECOVERY;
        last_result = HAL_ERROR;
    }
    else if (phase == WS2812_ACTIVE && (uint32_t)(HAL_GetTick() - started_ms) >= 20u)
    {
        phase = WS2812_RECOVERY;
        last_result = HAL_TIMEOUT;
    }
    if (phase == WS2812_COMPLETE)
    {
        // BDMA 普通模式完成后通道可能仍使能；SPI 已结束时主动关闭并确认。
        if (HAL_DMA_GetState(&hdma_spi6_tx) == HAL_DMA_STATE_READY &&
            (((BDMA_Channel_TypeDef *)hdma_spi6_tx.Instance)->CCR & BDMA_CCR_EN) != 0u)
            __HAL_DMA_DISABLE(&hdma_spi6_tx);
        if (WS2812_Stopped() != 0u)
        {
            WS2812_ClearPending();
            completed_r = active_r; completed_g = active_g; completed_b = active_b;
            completed_valid = 1u;
            phase = WS2812_IDLE;
            last_result = HAL_OK;
            tick_lease = 0u;
            WS2812_Exit(saved);
            BSP_HALTickReleaseBlockingWait();
            return HAL_OK;
        }
        if ((((BDMA_Channel_TypeDef *)hdma_spi6_tx.Instance)->CCR & BDMA_CCR_EN) != 0u &&
            (uint32_t)(HAL_GetTick() - started_ms) < 20u)
        {
            WS2812_Exit(saved);
            return HAL_BUSY;
        }
        phase = WS2812_RECOVERY;
        last_result = HAL_ERROR;
    }
    if (phase == WS2812_RECOVERY)
    {
        phase = WS2812_RECOVERING;
        recovery_started_ms = HAL_GetTick();
        WS2812_Exit(saved);
        // 只发出停止请求；异常外设不允许让 Daemon 在 HAL 内长时间等待。
        HAL_NVIC_DisableIRQ(SPI6_IRQn);
        HAL_NVIC_DisableIRQ(BDMA_Channel0_IRQn);
        __HAL_SPI_DISABLE_IT(&hspi6, SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP |
            SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF);
        HAL_StatusTypeDef dma_stop = HAL_OK;
        if (HAL_DMA_GetState(&hdma_spi6_tx) == HAL_DMA_STATE_BUSY)
        {
            hdma_spi6_tx.XferAbortCallback = NULL;
            dma_stop = HAL_DMA_Abort_IT(&hdma_spi6_tx);
        }
        else if (HAL_DMA_GetState(&hdma_spi6_tx) == HAL_DMA_STATE_READY &&
                 (((BDMA_Channel_TypeDef *)hdma_spi6_tx.Instance)->CCR & BDMA_CCR_EN) != 0u)
            __HAL_DMA_DISABLE(&hdma_spi6_tx);
        if ((hspi6.Instance->CR1 & SPI_CR1_CSTART) != 0u)
            SET_BIT(hspi6.Instance->CR1, SPI_CR1_CSUSP);
        saved = WS2812_Enter();
        uint8_t release = 0u;
        if (dma_stop != HAL_OK)
        {
            phase = WS2812_ISOLATED;
            release = tick_lease;
            tick_lease = 0u;
        }
        WS2812_Exit(saved);
        if (dma_stop != HAL_OK)
        {
            // 不再等待硬件；保留数组，但不能永久占用其他模块的时基。
            if (release != 0u) BSP_HALTickReleaseBlockingWait();
            return HAL_ERROR;
        }
        saved = WS2812_Enter();
    }
    if (phase == WS2812_RECOVERING)
    {
        // 尚未确认停止时让出任务；期限到达后保留缓冲并隔离通道。
        const uint8_t stopped = (uint8_t)((hspi6.Instance->CR1 & SPI_CR1_CSTART) == 0u &&
            (((BDMA_Channel_TypeDef *)hdma_spi6_tx.Instance)->CCR & BDMA_CCR_EN) == 0u &&
            HAL_DMA_GetState(&hdma_spi6_tx) == HAL_DMA_STATE_READY);
        if (stopped == 0u)
        {
            uint8_t release = 0u;
            if ((uint32_t)(HAL_GetTick() - recovery_started_ms) >= 20u)
            {
                phase = WS2812_ISOLATED;
                release = tick_lease;
                tick_lease = 0u;
            }
            const WS2812Phase_t current = phase;
            WS2812_Exit(saved);
            if (release != 0u) BSP_HALTickReleaseBlockingWait();
            return current == WS2812_ISOLATED ? HAL_ERROR : HAL_BUSY;
        }
        WS2812_Exit(saved);
        // CSTART 和 BDMA EN 已清除，HAL 收尾不会进入等待停止的循环。
        const HAL_StatusTypeDef result = HAL_SPI_Abort(&hspi6);
        const uint8_t safe = (uint8_t)(result == HAL_OK && WS2812_Stopped() != 0u);
        if (safe != 0u) WS2812_ClearPending();
        saved = WS2812_Enter();
        phase = safe != 0u ? WS2812_FAILED : WS2812_ISOLATED;
        completed_valid = 0u;
        // 已结束恢复，不再等待时基；隔离时数组仍不允许复用。
        const uint8_t release = tick_lease;
        if (release != 0u) tick_lease = 0u;
        WS2812_Exit(saved);
        if (release != 0u) BSP_HALTickReleaseBlockingWait();
        return HAL_ERROR;
    }
    const WS2812Phase_t current = phase;
    WS2812_Exit(saved);
    if (current == WS2812_IDLE) return HAL_OK;
    if (current == WS2812_FAILED || current == WS2812_ISOLATED) return HAL_ERROR;
    return HAL_BUSY;
}

uint8_t WS2812_OnTxComplete(SPI_HandleTypeDef *handle)
{
    if (handle != &hspi6) return 0u;
    const uint32_t saved = WS2812_Enter();
    if (phase == WS2812_STARTING || phase == WS2812_ACTIVE)
    {
        if ((uint32_t)(HAL_GetTick() - started_ms) >= 20u)
        {
            phase = WS2812_RECOVERY;
            last_result = HAL_TIMEOUT;
        }
        else
            phase = WS2812_COMPLETE;
    }
    WS2812_Exit(saved);
    return 1u;
}

uint8_t WS2812_OnError(SPI_HandleTypeDef *handle)
{
    if (handle != &hspi6) return 0u;
    const uint32_t saved = WS2812_Enter();
    if (phase == WS2812_STARTING || phase == WS2812_ACTIVE || phase == WS2812_COMPLETE)
    {
        phase = WS2812_RECOVERY;
        last_result = HAL_ERROR;
    }
    WS2812_Exit(saved);
    return 1u;
}

HAL_StatusTypeDef WS2812_GetState(WS2812TransferState_t *state)
{
    if (state == NULL || spi6_inited == 0u) return HAL_ERROR;
    WS2812TransferState_t candidate;
    const uint32_t saved = WS2812_Enter();
    if (phase == WS2812_IDLE) candidate.phase = WS2812_TRANSFER_IDLE;
    else if (phase == WS2812_FAILED) candidate.phase = WS2812_TRANSFER_FAILED;
    else if (phase == WS2812_ISOLATED) candidate.phase = WS2812_TRANSFER_ISOLATED;
    else if (phase == WS2812_RECOVERY || phase == WS2812_RECOVERING)
        candidate.phase = WS2812_TRANSFER_RECOVERING;
    else candidate.phase = WS2812_TRANSFER_ACTIVE;
    candidate.last_result = last_result;
    candidate.buffer_reusable = (uint8_t)(phase == WS2812_IDLE || phase == WS2812_FAILED);
    WS2812_Exit(saved);
    *state = candidate;
    return HAL_OK;
}

// SPI6 中断入口将本驱动的句柄交给 HAL 处理。
void SPI6_IRQHandler(void) { HAL_SPI_IRQHandler(&hspi6); }
// BDMA 通道0中断入口将发送句柄交给 HAL 处理。
void BDMA_Channel0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spi6_tx); }
