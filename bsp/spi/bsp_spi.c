#include "bsp_spi.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

/* 所有的spi instance保存于此,用于callback时判断中断来源*/
static SPIInstance *spi_instance[SPI_DEVICE_CNT] = {NULL};
static uint8_t idx = 0;                         // 配合中断以及初始化
/* 用于判断当前 spi 是否正在传输, 防止多个模块同时使用一条总线 (0: 正在传输, 1: 空闲)
   @attention 必须 volatile: 它在 SPI 完成中断里被写、在任务里被轮询。
   之前漏了 volatile, -Og 下"碰巧"能跑, 换 -O2 后轮询被优化成死循环(BMI088 初始化卡死)。 */
volatile uint8_t SPIDeviceOnGoing[SPI_DEVICE_CNT] = {[0 ... SPI_DEVICE_CNT - 1] = 1};
static SPIInstance *spi_bus_owner[SPI_DEVICE_CNT] = {NULL};

/* SPI 总线忙等待超时(us): 正常一次传输是个位数微秒~百微秒量级, 1ms 足够;
   超时说明完成中断没来(总线/中断异常), 不能永久卡死在这里。 */
#define SPI_BUSY_TIMEOUT_US 1000u
volatile uint32_t spi_bus_timeout_cnt[SPI_DEVICE_CNT] = {0};

static int8_t SPIBusIndex(const SPIInstance *spi_ins)
{
    if (spi_ins == NULL || spi_ins->spi_handle == NULL)
        return -1;
    if (spi_ins->spi_handle->Instance == SPI1)
        return 0;
    if (spi_ins->spi_handle->Instance == SPI2)
        return 1;
    return -1;
}

/* Claim the bus flag and owner pointer together so a preempting caller cannot
   observe idle and overwrite the ownership of an active transfer. */
static uint8_t SPIBusTryAcquire(SPIInstance *spi_ins, uint8_t bus_idx)
{
    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    uint8_t acquired = (uint8_t)(SPIDeviceOnGoing[bus_idx] && spi_bus_owner[bus_idx] == NULL);
    if (acquired)
    {
        SPIDeviceOnGoing[bus_idx] = 0u;
        spi_bus_owner[bus_idx] = spi_ins;
    }
    __set_PRIMASK(previous_primask);
    return acquired;
}

static void SPIBusRelease(SPIInstance *spi_ins)
{
    int8_t bus_idx = SPIBusIndex(spi_ins);
    if (bus_idx < 0 || (uint8_t)bus_idx >= SPI_DEVICE_CNT)
        return;

    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    if (spi_bus_owner[(uint8_t)bus_idx] == spi_ins)
    {
        spi_bus_owner[(uint8_t)bus_idx] = NULL;
        SPIDeviceOnGoing[(uint8_t)bus_idx] = 1u;
    }
    __set_PRIMASK(previous_primask);
}

static SPIInstance *SPIBusAbortOwner(SPI_HandleTypeDef *hspi)
{
    int8_t bus_idx = -1;
    if (hspi != NULL && hspi->Instance == SPI1)
        bus_idx = 0;
    else if (hspi != NULL && hspi->Instance == SPI2)
        bus_idx = 1;
    if (bus_idx < 0 || (uint8_t)bus_idx >= SPI_DEVICE_CNT)
        return NULL;

    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    SPIInstance *owner = spi_bus_owner[(uint8_t)bus_idx];
    if (owner != NULL && owner->spi_handle == hspi)
    {
        HAL_GPIO_WritePin(owner->GPIOx, owner->cs_pin, GPIO_PIN_SET);
        owner->CS_State = GPIO_PIN_SET;
        spi_bus_owner[(uint8_t)bus_idx] = NULL;
        SPIDeviceOnGoing[(uint8_t)bus_idx] = 1u;
    }
    else
    {
        owner = NULL;
    }
    __set_PRIMASK(previous_primask);
    return owner;
}

/* Wait for and atomically claim the bus; timeout never changes another owner's state. */
static HAL_StatusTypeDef SPIBusAcquire(SPIInstance *spi_ins, uint8_t bus_idx, uint32_t timeout_us)
{
    uint32_t t0 = DWT_ProbeStart();
    while (!SPIBusTryAcquire(spi_ins, bus_idx))
    {
        if (DWT_ProbeElapsedUs(t0) >= timeout_us)
        {
            static LogRateLimit_t rl_spi_busy = {0};
            spi_bus_timeout_cnt[bus_idx]++;
            if (LogRateLimitAllow(&rl_spi_busy, 1000u))
                LOGERROR("[bsp_spi] bus busy wait timed out; keeping ownership with active transfer");
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

/* HAL's blocking SPI timeout depends on HAL tick progress. Refuse contexts that
   can mask the tick so a nominal timeout cannot become an unbounded wait. */
static uint8_t SPIBlockingContextCanWait(void)
{
    return (__get_IPSR() == 0u &&
            __get_PRIMASK() == 0u &&
            __get_BASEPRI() == 0u &&
            __get_FAULTMASK() == 0u) ? 1u : 0u;
}

SPIInstance *SPIRegister(SPI_Init_Config_s *conf)
{
    if (idx >= MX_SPI_BUS_SLAVE_CNT) // 超过最大实例数
        while (1)
            ;
    SPIInstance *instance = (SPIInstance *)malloc(sizeof(SPIInstance));
    memset(instance, 0, sizeof(SPIInstance));

    instance->spi_handle = conf->spi_handle;
    instance->GPIOx = conf->GPIOx;
    instance->cs_pin = conf->cs_pin;
    instance->spi_work_mode = conf->spi_work_mode;
    instance->callback = conf->callback;
    instance->id = conf->id;
    if (instance->spi_handle->Instance == SPI1)
    {
        instance->cs_pin_state = &SPIDeviceOnGoing[0];
    }
    else if (instance->spi_handle->Instance == SPI2)
    {
        instance->cs_pin_state = &SPIDeviceOnGoing[1];
    }
    else
    {
        while (1)
            ;
    }
    spi_instance[idx++] = instance;
    return instance;
}

void SPITransmit(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len)
{
    // 拉低片选,开始传输(选中从机)
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    switch (spi_ins->spi_work_mode)
    {
    case SPI_DMA_MODE:
        HAL_SPI_Transmit_DMA(spi_ins->spi_handle, ptr_data, len);
        break;
    case SPI_IT_MODE:
        HAL_SPI_Transmit_IT(spi_ins->spi_handle, ptr_data, len);
        break;
    case SPI_BLOCK_MODE:
        HAL_SPI_Transmit(spi_ins->spi_handle, ptr_data, len, 1000); // 默认50ms超时
        // 阻塞模式不会调用回调函数,传输完成后直接拉高片选结束
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
        break;
    default:
        while (1)
            ; // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况
        break;
    }
}

void SPIRecv(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len)
{
    // 用于稍后回调使用
    spi_ins->rx_size = len;
    spi_ins->rx_buffer = ptr_data;
    // 拉低片选,开始传输
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    switch (spi_ins->spi_work_mode)
    {
    case SPI_DMA_MODE:
        HAL_SPI_Receive_DMA(spi_ins->spi_handle, ptr_data, len);
        break;
    case SPI_IT_MODE:
        HAL_SPI_Receive_IT(spi_ins->spi_handle, ptr_data, len);
        break;
    case SPI_BLOCK_MODE:
        HAL_SPI_Receive(spi_ins->spi_handle, ptr_data, len, 1000);
        // 阻塞模式不会调用回调函数,传输完成后直接拉高片选结束
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
        break;
    default:
        while (1)
            ; // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况
        break;
    }
}

HAL_StatusTypeDef SPITransRecv(SPIInstance *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len)
{
    if (spi_ins == NULL || spi_ins->spi_handle == NULL || spi_ins->GPIOx == NULL ||
        spi_ins->cs_pin_state == NULL || ptr_data_rx == NULL || ptr_data_tx == NULL || len == 0u)
        return HAL_ERROR;

    int8_t bus_idx = SPIBusIndex(spi_ins);
    if (bus_idx < 0 || (uint8_t)bus_idx >= SPI_DEVICE_CNT)
        return HAL_ERROR;

    if (spi_ins->spi_work_mode == SPI_BLOCK_MODE && !SPIBlockingContextCanWait())
        return HAL_BUSY;

    HAL_StatusTypeDef status = SPIBusAcquire(spi_ins, (uint8_t)bus_idx, SPI_BUSY_TIMEOUT_US);
    if (status != HAL_OK)
        return status;

    spi_ins->rx_size = len;
    spi_ins->rx_buffer = ptr_data_rx;
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    spi_ins->CS_State = GPIO_PIN_RESET;

    switch (spi_ins->spi_work_mode)
    {
    case SPI_DMA_MODE:
        status = HAL_SPI_TransmitReceive_DMA(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len);
        break;
    case SPI_IT_MODE:
        status = HAL_SPI_TransmitReceive_IT(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len);
        break;
    case SPI_BLOCK_MODE:
        status = HAL_SPI_TransmitReceive(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len, 2u);
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
        SPIBusRelease(spi_ins);
        spi_ins->CS_State = GPIO_PIN_SET;
        return status;
    default:
        status = HAL_ERROR;
        break;
    }

    if (status != HAL_OK)
    {
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
        SPIBusRelease(spi_ins);
        spi_ins->CS_State = GPIO_PIN_SET;
    }
    return status;
}

void SPISetMode(SPIInstance *spi_ins, SPI_TXRX_MODE_e spi_mode)
{
    if (spi_mode != SPI_DMA_MODE && spi_mode != SPI_IT_MODE && spi_mode != SPI_BLOCK_MODE)
        while (1)
            ; // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况

    if (spi_ins->spi_work_mode != spi_mode)
    {
        spi_ins->spi_work_mode = spi_mode;
    }
}

/**
 * @brief 当SPI接收完成,将会调用此回调函数,可以进行协议解析或其他必须的数据处理等
 *
 * @param hspi spi handle
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    for (size_t i = 0; i < idx; i++)
    {
        // 如果是当前spi硬件发出的complete,且cs_pin为低电平(说明正在传输),则尝试调用回调函数
        if (spi_instance[i]->spi_handle == hspi && // 显然同一时间一条总线只能有一个从机在接收数据
            HAL_GPIO_ReadPin(spi_instance[i]->GPIOx, spi_instance[i]->cs_pin) == GPIO_PIN_RESET)
        {
            // 先拉高片选,结束传输,在判断是否有回调函数,如果有则调用回调函数
            HAL_GPIO_WritePin(spi_instance[i]->GPIOx, spi_instance[i]->cs_pin, GPIO_PIN_SET);
            spi_instance[i]->CS_State = HAL_GPIO_ReadPin(spi_instance[i]->GPIOx, spi_instance[i]->cs_pin);
            SPIBusRelease(spi_instance[i]);
            // @todo 后续添加holdon模式,由用户自行决定何时释放片选,允许进行连续传输
            if (spi_instance[i]->callback != NULL) // 回调函数不为空, 则调用回调函数
                spi_instance[i]->callback(spi_instance[i]);
            return;
        }
    }
}

/**
 * @brief 和RxCpltCallback共用解析即可,这里只是形式上封装一下,不用重复写
 *        这是对HAL库的__weak函数的重写,传输使用IT或DMA模式,在传输完成时会调用此函数
 * @param hspi spi handle
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_RxCpltCallback(hspi); // 直接调用接收完成的回调函数
}

/**
 * @brief 纯发送(DMA/IT)完成回调: 释放片选,复用与接收完成相同的 CS 释放逻辑
 *
 * @param hspi spi handle
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_RxCpltCallback(hspi);
}

/**
 * @brief SPI 出错回调: 释放总线忙标志
 *        传输出错时完成回调不会来, 如果不在这里清标志, 总线会永久"忙", 后续所有
 *        SPI 访问都会卡在等待里(尤其 -O2 下再也"碰巧"不过去)。
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    SPIInstance *owner = SPIBusAbortOwner(hspi);
    if (owner != NULL)
    {
        for (size_t i = 0; i < idx; ++i)
        {
            if (spi_instance[i] == owner)
            {
                LOGERROR("[bsp_spi] SPI 传输出错, 已释放总线 (idx %u)", (unsigned)i);
                return;
            }
        }
        return;
    }

    /* Legacy SPITransmit/SPIRecv paths do not claim the synchronous bus owner. */
    for (size_t i = 0; i < idx; i++)
    {
        if (spi_instance[i]->spi_handle == hspi &&
            HAL_GPIO_ReadPin(spi_instance[i]->GPIOx, spi_instance[i]->cs_pin) == GPIO_PIN_RESET)
        {
            HAL_GPIO_WritePin(spi_instance[i]->GPIOx, spi_instance[i]->cs_pin, GPIO_PIN_SET);
            spi_instance[i]->CS_State = 1u;
            LOGERROR("[bsp_spi] SPI 传输出错, 已释放总线 (idx %u)", (unsigned)i);
            return;
        }
    }
}
