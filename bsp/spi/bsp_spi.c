/** @file bsp_spi.c
 * @brief 为 SPI 收发保存唯一使用者，统一片选、回调及任务超时恢复；使用条件见 bsp_spi.h。
 */
#include "bsp_spi.h"
#include "bsp_dwt.h"
#include "bsp_hal_tick.h"
#include "ws2812.h"
#include <limits.h>
#include <stdlib.h>
#include <string.h>

typedef enum { SPI_OP_TX, SPI_OP_RX, SPI_OP_TXRX } SPIOperation;
typedef enum { BUS_FREE, BUS_STARTING, BUS_ACTIVE, BUS_CALLBACK, BUS_RECOVERY, BUS_ABORTING, BUS_ISOLATED } SPIBusPhase;
typedef struct
{
    SPIInstance *owner;
    SPIBusPhase phase;
    SPIOperation operation;
    uint32_t start_ms;
    uint8_t lease, completion_pending, error_pending;
} SPIBus;
static SPIInstance *spi_instance[MX_SPI_BUS_SLAVE_CNT];
static uint8_t instance_count;
static SPIBus buses[SPI_DEVICE_CNT];
volatile uint8_t SPIDeviceOnGoing[SPI_DEVICE_CNT] = {1u, 1u};
volatile uint32_t spi_bus_timeout_cnt[SPI_DEVICE_CNT];

static uint32_t SPIEnter(void) { uint32_t saved = __get_PRIMASK(); __disable_irq(); return saved; }
static void SPIExit(uint32_t saved) { __set_PRIMASK(saved); }
static uint8_t SPITaskContext(void)
{
    return (__get_IPSR() == 0u && __get_PRIMASK() == 0u && __get_BASEPRI() == 0u && __get_FAULTMASK() == 0u);
}
static int SPIBusIndex(const SPI_HandleTypeDef *handle)
{
    if (handle == NULL) return -1;
    if (handle->Instance == SPI1) return 0;
    if (handle->Instance == SPI2) return 1;
    return -1;
}
// 必须在短临界区内使用；先比较地址，未注册指针不会被解引用。
static uint8_t SPIRegistered(const SPIInstance *instance)
{
    for (uint8_t i = 0u; i < instance_count; ++i)
        if (spi_instance[i] == instance) return 1u;
    return 0u;
}
static uint8_t SPIValidMode(SPI_TXRX_MODE_e mode)
{ return mode == SPI_BLOCK_MODE || mode == SPI_IT_MODE || mode == SPI_DMA_MODE; }
static IRQn_Type SPIIRQ(const SPI_HandleTypeDef *handle)
{ return handle->Instance == SPI1 ? SPI1_IRQn : SPI2_IRQn; }

// H723 的 SPI DMA 使用 DMA1／DMA2 stream；缺失配置不会自动改为另一种模式。
static IRQn_Type SPIDMAIRQ(const DMA_HandleTypeDef *dma)
{
    DMA_Stream_TypeDef *const streams[] = {DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3,
        DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7, DMA2_Stream0, DMA2_Stream1,
        DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7};
    const IRQn_Type irqs[] = {DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
        DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn,
        DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
        DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn};
    if (dma != NULL)
        for (unsigned i = 0u; i < sizeof(streams) / sizeof(streams[0]); ++i)
            if (dma->Instance == streams[i]) return irqs[i];
    return (IRQn_Type)-1;
}
static uint8_t SPIDMAReady(SPI_HandleTypeDef *handle, DMA_HandleTypeDef *dma, uint32_t direction)
{
    IRQn_Type irq = SPIDMAIRQ(dma);
    return irq >= 0 && NVIC_GetEnableIRQ(irq) && dma->Parent == handle &&
        HAL_DMA_GetState(dma) == HAL_DMA_STATE_READY && dma->Init.Direction == direction &&
        dma->Init.Mode == DMA_NORMAL && dma->Init.MemInc == DMA_MINC_ENABLE &&
        dma->Init.PeriphInc == DMA_PINC_DISABLE && dma->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE &&
        dma->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE;
}
static uint8_t SPIBackendConfigured(SPIInstance *instance, SPIOperation operation)
{
    SPI_HandleTypeDef *handle = instance->spi_handle;
    if (handle->Init.Mode != SPI_MODE_MASTER || handle->Init.DataSize != SPI_DATASIZE_8BIT ||
        (handle->Init.Direction != SPI_DIRECTION_2LINES &&
         !(operation == SPI_OP_TX && handle->Init.Direction == SPI_DIRECTION_2LINES_TXONLY))) return 0u;
    if (instance->spi_work_mode == SPI_BLOCK_MODE) return 1u;
    if (!NVIC_GetEnableIRQ(SPIIRQ(handle))) return 0u;
    if (instance->spi_work_mode == SPI_IT_MODE) return 1u;
    if (operation != SPI_OP_RX && !SPIDMAReady(handle, handle->hdmatx, DMA_MEMORY_TO_PERIPH)) return 0u;
    if (operation != SPI_OP_TX && !SPIDMAReady(handle, handle->hdmarx, DMA_PERIPH_TO_MEMORY)) return 0u;
    return 1u;
}
// READY 本身不足以证明硬件已停止，还要检查请求位和 DMA stream。
static uint8_t SPIBackendStopped(SPIBus *bus)
{
    SPI_HandleTypeDef *handle = bus->owner->spi_handle;
    if (HAL_SPI_GetState(handle) != HAL_SPI_STATE_READY ||
        (handle->Instance->CR1 & (SPI_CR1_SPE | SPI_CR1_CSTART)) != 0u ||
        (handle->Instance->CFG1 & (SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN)) != 0u ||
        (handle->Instance->IER & (SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP |
            SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF)) != 0u) return 0u;
    if (bus->owner->spi_work_mode != SPI_DMA_MODE) return 1u;
    DMA_HandleTypeDef *dma[] = {handle->hdmatx, handle->hdmarx};
    for (unsigned i = 0u; i < 2u; ++i)
    {
        if ((i == 0u && bus->operation == SPI_OP_RX) || (i == 1u && bus->operation == SPI_OP_TX)) continue;
        if (dma[i] == NULL || HAL_DMA_GetState(dma[i]) != HAL_DMA_STATE_READY ||
            (((DMA_Stream_TypeDef *)dma[i]->Instance)->CR & DMA_SxCR_EN) != 0u) return 0u;
    }
    return 1u;
}
// 已确认停止后清除本总线的待处理中断，不影响另一条总线。
static void SPIClearPending(SPIBus *bus)
{
    SPI_HandleTypeDef *handle = bus->owner->spi_handle;
    NVIC_ClearPendingIRQ(SPIIRQ(handle));
    if (bus->owner->spi_work_mode == SPI_DMA_MODE)
    {
        DMA_HandleTypeDef *dma[] = {handle->hdmatx, handle->hdmarx};
        for (unsigned i = 0u; i < 2u; ++i)
        {
            if ((i == 0u && bus->operation == SPI_OP_RX) || (i == 1u && bus->operation == SPI_OP_TX)) continue;
            __HAL_DMA_CLEAR_FLAG(dma[i], __HAL_DMA_GET_TC_FLAG_INDEX(dma[i]) |
                __HAL_DMA_GET_HT_FLAG_INDEX(dma[i]) | __HAL_DMA_GET_TE_FLAG_INDEX(dma[i]) |
                __HAL_DMA_GET_DME_FLAG_INDEX(dma[i]) | __HAL_DMA_GET_FE_FLAG_INDEX(dma[i]));
            NVIC_ClearPendingIRQ(SPIDMAIRQ(dma[i]));
        }
    }
}
// 调用时已关普通中断；先完成结果、片选与计时释放，再让其他任务取得总线。
static void SPIFinish(SPIBus *bus, HAL_StatusTypeDef result)
{
    SPIInstance *owner = bus->owner;
    HAL_GPIO_WritePin(owner->GPIOx, owner->cs_pin, GPIO_PIN_SET);
    owner->CS_State = GPIO_PIN_SET;
    owner->transfer = (SPITransferState_t){result == HAL_OK ? SPI_TRANSFER_SUCCEEDED : SPI_TRANSFER_FAILED, result, 1u};
    if (bus->lease) BSP_HALTickReleaseBlockingWait();
    int index = SPIBusIndex(owner->spi_handle);
    memset(bus, 0, sizeof(*bus));
    SPIDeviceOnGoing[index] = 1u;
}
// HAL 停止可能等待，只能在临界区外的任务中执行；ABORTING 期间中断不能再次提交结果。
static uint8_t SPIRecover(SPIBus *bus)
{
    HAL_StatusTypeDef stopped = HAL_SPI_Abort(bus->owner->spi_handle);
    uint32_t saved = SPIEnter();
    if (stopped == HAL_OK && SPIBackendStopped(bus))
    {
        SPIClearPending(bus);
        SPIFinish(bus, bus->owner->transfer.result);
        SPIExit(saved);
        return 1u;
    }
    // 不让新请求覆盖可能仍被 DMA 使用的数组，也不反复调用停止。
    bus->phase = BUS_ISOLATED;
    bus->owner->transfer.phase = SPI_TRANSFER_ISOLATED;
    SPIExit(saved);
    return 0u;
}

SPIInstance *SPIRegister(const SPI_Init_Config_s *conf)
{
    if (!SPITaskContext() || conf == NULL || SPIBusIndex(conf->spi_handle) < 0 || conf->GPIOx == NULL ||
        conf->cs_pin == 0u || (conf->cs_pin & (conf->cs_pin - 1u)) != 0u ||
        !SPIValidMode(conf->spi_work_mode) || conf->async_timeout_ms > INT32_MAX) return NULL;
    uint32_t saved = SPIEnter();
    uint8_t valid = instance_count < MX_SPI_BUS_SLAVE_CNT;
    for (uint8_t i = 0u; i < instance_count; ++i)
    {
        SPIInstance *old = spi_instance[i];
        if ((old->GPIOx == conf->GPIOx && old->cs_pin == conf->cs_pin) ||
            (old->spi_handle->Instance == conf->spi_handle->Instance && old->spi_handle != conf->spi_handle)) valid = 0u;
    }
    SPIExit(saved);
    if (!valid) return NULL;
    SPIInstance *instance = malloc(sizeof(*instance));
    if (instance == NULL) return NULL;
    // 先准备完整候选，再发布指针；启动注册由调用方串行执行。
    memset(instance, 0, sizeof(*instance));
    instance->spi_handle = conf->spi_handle;
    instance->GPIOx = conf->GPIOx;
    instance->cs_pin = conf->cs_pin;
    instance->spi_work_mode = conf->spi_work_mode;
    instance->callback = conf->callback;
    instance->id = conf->id;
    instance->async_timeout_ms = conf->async_timeout_ms == 0u ? SPI_ASYNC_TIMEOUT_DEFAULT_MS : conf->async_timeout_ms;
    instance->CS_State = GPIO_PIN_SET;
    instance->cs_pin_state = &SPIDeviceOnGoing[SPIBusIndex(conf->spi_handle)];
    instance->transfer = (SPITransferState_t){SPI_TRANSFER_IDLE, HAL_OK, 1u};
    saved = SPIEnter();
    spi_instance[instance_count++] = instance;
    SPIExit(saved);
    return instance;
}
uint8_t SPIUnregister(SPIInstance *instance)
{
    if (!SPITaskContext()) return 0u;
    uint32_t saved = SPIEnter();
    if (!SPIRegistered(instance) || buses[SPIBusIndex(instance->spi_handle)].phase != BUS_FREE)
    { SPIExit(saved); return 0u; }
    for (uint8_t i = 0u; i < instance_count; ++i)
        if (spi_instance[i] == instance)
        {
            HAL_GPIO_WritePin(instance->GPIOx, instance->cs_pin, GPIO_PIN_SET);
            memmove(&spi_instance[i], &spi_instance[i + 1u], (instance_count - i - 1u) * sizeof(spi_instance[0]));
            spi_instance[--instance_count] = NULL;
            break;
        }
    SPIExit(saved);
    free(instance);
    return 1u;
}
// 在短临界区内认领完成；任务在 HAL 返回前收到的完成也走同一检查。
static SPIInstance *SPIClaimCompletion(SPIBus *bus)
{
    uint8_t expired = (uint32_t)(HAL_GetTick() - bus->start_ms) >= bus->owner->async_timeout_ms;
    if (expired || !SPIBackendStopped(bus))
    {
        bus->phase = BUS_RECOVERY;
        bus->owner->transfer.phase = SPI_TRANSFER_RECOVERING;
        bus->owner->transfer.result = expired ? HAL_TIMEOUT : HAL_ERROR;
        return NULL;
    }
    SPIClearPending(bus);
    bus->phase = BUS_CALLBACK;
    SPIInstance *owner = bus->owner;
    // 回调前拉高片选，保留占用，避免接收字段被下一次提交覆盖。
    HAL_GPIO_WritePin(owner->GPIOx, owner->cs_pin, GPIO_PIN_SET);
    owner->CS_State = GPIO_PIN_SET;
    return owner;
}
// BUS_CALLBACK 保留占用，临界区外交付数据；此期间超时服务不会停止或释放本次事务。
static void SPIDeliverCompletion(SPIBus *bus, SPIInstance *owner, SPIOperation operation)
{
    if (operation != SPI_OP_TX && owner->callback != NULL) owner->callback(owner);
    uint32_t saved = SPIEnter();
    SPIFinish(bus, HAL_OK);
    SPIExit(saved);
}
static void SPIComplete(SPI_HandleTypeDef *handle, SPIOperation operation)
{
    int index = SPIBusIndex(handle);
    if (index < 0) return;
    SPIBus *bus = &buses[index];
    uint32_t saved = SPIEnter();
    if (bus->owner == NULL || bus->owner->spi_handle != handle || bus->operation != operation ||
        bus->owner->spi_work_mode == SPI_BLOCK_MODE) { SPIExit(saved); return; }
    if (bus->phase == BUS_STARTING) { bus->completion_pending = 1u; SPIExit(saved); return; }
    if (bus->phase != BUS_ACTIVE) { SPIExit(saved); return; }
    SPIInstance *owner = SPIClaimCompletion(bus);
    SPIExit(saved);
    if (owner != NULL) SPIDeliverCompletion(bus, owner, operation);
}
static HAL_StatusTypeDef SPIStart(SPIInstance *instance, uint8_t *rx, uint8_t *tx, uint8_t len, SPIOperation operation)
{
    if (!SPITaskContext()) return HAL_BUSY;
    if (len == 0u || (operation != SPI_OP_TX && rx == NULL) || (operation != SPI_OP_RX && tx == NULL)) return HAL_ERROR;
    if (operation == SPI_OP_TXRX && ((uintptr_t)rx <= (uintptr_t)tx ?
        (uintptr_t)tx - (uintptr_t)rx < len : (uintptr_t)rx - (uintptr_t)tx < len)) return HAL_ERROR;
    uint32_t wait_start = DWT_ProbeStart();
    SPIBus *bus;
    // 每次重新检查注册；到期不释放其他事务，不改变任何片选。
    for (;;)
    {
        uint32_t saved = SPIEnter();
        if (!SPIRegistered(instance) || !SPIValidMode(instance->spi_work_mode)) { SPIExit(saved); return HAL_ERROR; }
        int index = SPIBusIndex(instance->spi_handle);
        bus = &buses[index];
        if (bus->phase == BUS_ISOLATED) { SPIExit(saved); return HAL_ERROR; }
        if (bus->phase == BUS_FREE)
        {
            bus->owner = instance;
            bus->operation = operation;
            bus->phase = BUS_STARTING;
            SPIDeviceOnGoing[index] = 0u;
            SPIExit(saved);
            break;
        }
        uint8_t block = instance->spi_work_mode == SPI_BLOCK_MODE;
        SPIExit(saved);
        if (!block) return HAL_BUSY;
        if (DWT_ProbeElapsedUs(wait_start) >= 1000u)
        {
            saved = SPIEnter();
            spi_bus_timeout_cnt[index]++;
            SPIExit(saved);
            return HAL_TIMEOUT;
        }
    }
    HAL_StatusTypeDef rejection = HAL_ERROR;
    uint8_t lease = 0u;
    if (SPIBackendConfigured(instance, operation))
    {
        rejection = HAL_BUSY;
        if (HAL_SPI_GetState(instance->spi_handle) == HAL_SPI_STATE_READY)
            lease = BSP_HALTickTryAcquireBlockingWait();
    }
    uint32_t saved = SPIEnter();
    if (!lease)
    {
        // 尚未启动 HAL，不触碰片选和旧结果，也不停止外部 HAL 操作。
        memset(bus, 0, sizeof(*bus));
        SPIDeviceOnGoing[SPIBusIndex(instance->spi_handle)] = 1u;
        SPIExit(saved);
        return rejection;
    }
    bus->lease = 1u;
    bus->start_ms = HAL_GetTick();
    instance->rx_buffer = rx;
    instance->rx_size = operation == SPI_OP_TX ? 0u : len;
    instance->transfer = (SPITransferState_t){SPI_TRANSFER_ACTIVE, HAL_BUSY, 0u};
    HAL_GPIO_WritePin(instance->GPIOx, instance->cs_pin, GPIO_PIN_RESET);
    instance->CS_State = GPIO_PIN_RESET;
    SPIExit(saved);
    SPI_HandleTypeDef *handle = instance->spi_handle;
    HAL_StatusTypeDef result;
    if (instance->spi_work_mode == SPI_BLOCK_MODE)
    {
        if (operation == SPI_OP_TX) result = HAL_SPI_Transmit(handle, tx, len, 2u);
        else if (operation == SPI_OP_RX) result = HAL_SPI_Receive(handle, rx, len, 2u);
        else result = HAL_SPI_TransmitReceive(handle, tx, rx, len, 2u);
    }
    else if (instance->spi_work_mode == SPI_IT_MODE)
    {
        if (operation == SPI_OP_TX) result = HAL_SPI_Transmit_IT(handle, tx, len);
        else if (operation == SPI_OP_RX) result = HAL_SPI_Receive_IT(handle, rx, len);
        else result = HAL_SPI_TransmitReceive_IT(handle, tx, rx, len);
    }
    else
    {
        if (operation == SPI_OP_TX) result = HAL_SPI_Transmit_DMA(handle, tx, len);
        else if (operation == SPI_OP_RX) result = HAL_SPI_Receive_DMA(handle, rx, len);
        else result = HAL_SPI_TransmitReceive_DMA(handle, tx, rx, len);
    }
    saved = SPIEnter();
    uint8_t block = instance->spi_work_mode == SPI_BLOCK_MODE;
    if (block || result != HAL_OK || bus->error_pending)
    {
        HAL_StatusTypeDef final_result = bus->error_pending ? HAL_ERROR : result;
        instance->transfer.result = final_result;
        if (SPIBackendStopped(bus))
        {
            SPIClearPending(bus);
            SPIFinish(bus, final_result);
        }
        else
        {
            instance->transfer.phase = SPI_TRANSFER_RECOVERING;
            bus->phase = block ? BUS_ABORTING : BUS_RECOVERY;
            // 阻塞路径无 DMA／中断继续访问数组，返回后其栈数组可释放。
            if (block) instance->transfer.buffers_released = 1u;
        }
        uint8_t recover_now = bus->phase == BUS_ABORTING;
        SPIExit(saved);
        if (recover_now && !SPIRecover(bus)) return HAL_ERROR;
        return final_result;
    }
    bus->phase = BUS_ACTIVE;
    // 已到的完成必须在开放任务切换前认领，不能稍后把它误用到另一笔事务。
    SPIInstance *completed = bus->completion_pending ? SPIClaimCompletion(bus) : NULL;
    SPIExit(saved);
    if (completed != NULL) SPIDeliverCompletion(bus, completed, operation);
    return result;
}
HAL_StatusTypeDef SPITransmit(SPIInstance *instance, uint8_t *data, uint8_t len)
{ return SPIStart(instance, NULL, data, len, SPI_OP_TX); }
HAL_StatusTypeDef SPIRecv(SPIInstance *instance, uint8_t *data, uint8_t len)
{ return SPIStart(instance, data, NULL, len, SPI_OP_RX); }
HAL_StatusTypeDef SPITransRecv(SPIInstance *instance, uint8_t *rx, uint8_t *tx, uint8_t len)
{ return SPIStart(instance, rx, tx, len, SPI_OP_TXRX); }
HAL_StatusTypeDef SPISetMode(SPIInstance *instance, SPI_TXRX_MODE_e mode)
{
    if (!SPITaskContext()) return HAL_BUSY;
    uint32_t saved = SPIEnter();
    HAL_StatusTypeDef result = HAL_ERROR;
    if (SPIRegistered(instance) && SPIValidMode(mode))
    {
        result = buses[SPIBusIndex(instance->spi_handle)].phase == BUS_FREE ? HAL_OK : HAL_BUSY;
        if (result == HAL_OK) instance->spi_work_mode = mode;
    }
    SPIExit(saved);
    return result;
}
HAL_StatusTypeDef SPIGetTransferState(const SPIInstance *instance, SPITransferState_t *output)
{
    if (output == NULL) return HAL_ERROR;
    uint32_t saved = SPIEnter();
    if (!SPIRegistered(instance)) { SPIExit(saved); return HAL_ERROR; }
    *output = instance->transfer;
    SPIExit(saved);
    return HAL_OK;
}
HAL_StatusTypeDef SPIProcessTimeouts(void)
{
    if (!SPITaskContext()) return HAL_BUSY;
    HAL_StatusTypeDef result = HAL_OK;
    // 每条总线独立认领恢复，其他任务或中断不会重复停止同一事务。
    for (unsigned i = 0u; i < SPI_DEVICE_CNT; ++i)
    {
        SPIBus *bus = &buses[i];
        uint32_t saved = SPIEnter();
        if (bus->phase == BUS_ACTIVE && (uint32_t)(HAL_GetTick() - bus->start_ms) >= bus->owner->async_timeout_ms)
        {
            bus->phase = BUS_RECOVERY;
            bus->owner->transfer.phase = SPI_TRANSFER_RECOVERING;
            bus->owner->transfer.result = HAL_TIMEOUT;
        }
        uint8_t recover = bus->phase == BUS_RECOVERY;
        if (recover) bus->phase = BUS_ABORTING;
        if (bus->phase == BUS_ISOLATED) result = HAL_ERROR;
        SPIExit(saved);
        if (recover && !SPIRecover(bus)) result = HAL_ERROR;
    }
    return result;
}
/** @brief HAL 接收完成入口；仅匹配当前接收事务。 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *handle) { SPIComplete(handle, SPI_OP_RX); }
/** @brief HAL 全双工完成入口；迟到或错误完成不发布数据。 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *handle) { SPIComplete(handle, SPI_OP_TXRX); }
/** @brief HAL 发送完成入口；不会调用接收业务回调。 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *handle)
{
    if (WS2812_OnTxComplete(handle) == 0u) SPIComplete(handle, SPI_OP_TX);
}
/** @brief HAL 错误入口；只登记失败，停止流程延至任务。 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *handle)
{
    if (WS2812_OnError(handle) != 0u) return;
    int index = SPIBusIndex(handle);
    if (index < 0) return;
    uint32_t saved = SPIEnter();
    SPIBus *bus = &buses[index];
    if (bus->owner != NULL && bus->owner->spi_handle == handle)
    {
        if (bus->phase == BUS_STARTING) bus->error_pending = 1u;
        else if (bus->phase == BUS_ACTIVE)
        {
            bus->phase = BUS_RECOVERY;
            bus->owner->transfer.phase = SPI_TRANSFER_RECOVERING;
            bus->owner->transfer.result = HAL_ERROR;
        }
    }
    SPIExit(saved);
}
