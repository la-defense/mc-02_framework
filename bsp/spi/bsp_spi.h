/** @file bsp_spi.h
 * @brief SPI1／SPI2 设备、独占收发和任务超时处理；先初始化 HAL、GPIO、DWT 和 HAL tick。
 * @note 同一总线运行时不能混用直接 HAL 调用。注册／注销由启动流程串行执行。
 */
#ifndef MC02_BSP_SPI_H
#define MC02_BSP_SPI_H
#include "spi.h"
#include "gpio.h"
#include <stdint.h>

#define SPI_DEVICE_CNT 2u
#define MX_SPI_BUS_SLAVE_CNT 4u // 两条总线合计的设备容量。
#define SPI_ASYNC_TIMEOUT_DEFAULT_MS 20u
typedef enum { SPI_BLOCK_MODE = 0, SPI_IT_MODE, SPI_DMA_MODE } SPI_TXRX_MODE_e;
typedef enum
{
    SPI_TRANSFER_IDLE = 0, SPI_TRANSFER_ACTIVE, SPI_TRANSFER_RECOVERING,
    SPI_TRANSFER_SUCCEEDED, SPI_TRANSFER_FAILED,
    SPI_TRANSFER_ISOLATED // 停止失败；保留占用，必须重新初始化系统。
} SPITransferPhase_t;
typedef struct
{
    SPITransferPhase_t phase;
    HAL_StatusTypeDef result; // ACTIVE 为 HAL_BUSY；恢复时为原始错误或 HAL_TIMEOUT。
    uint8_t buffers_released; // 1 才允许复用异步数组；还须检查结果才可使用接收数据。
} SPITransferState_t;
typedef struct spi_ins_temp
{
    SPI_HandleTypeDef *spi_handle;
    GPIO_TypeDef *GPIOx;
    uint16_t cs_pin;
    SPI_TXRX_MODE_e spi_work_mode;
    uint8_t rx_size;
    uint8_t *rx_buffer;
    uint8_t CS_State;
    volatile uint8_t *cs_pin_state; // 兼容诊断字段：0 忙、1 空闲，调用方不得写入。
    void (*callback)(struct spi_ins_temp *);
    void *id;
    uint32_t async_timeout_ms;
    SPITransferState_t transfer; // 仅本模块改写；跨任务读取须使用查询入口。
} SPIInstance;
typedef void (*spi_rx_callback)(SPIInstance *);
typedef struct
{
    SPI_HandleTypeDef *spi_handle;
    GPIO_TypeDef *GPIOx;
    uint16_t cs_pin; // 一个 GPIO_PIN_x 位，事先配置为输出并拉高。
    SPI_TXRX_MODE_e spi_work_mode;
    spi_rx_callback callback; // 可为 NULL；只交付成功的异步接收／全双工。
    void *id; // 可为 NULL，由业务回调自行解释。
    uint32_t async_timeout_ms; // 0 使用 20 ms；显式值 1～INT32_MAX ms，阻塞模式不使用。
} SPI_Init_Config_s;

/** @brief 复制设备配置，分配一个长期实例。
 * @param[in] conf 非空；SPI1／SPI2、单片选位、合法模式；总计最多 4 个设备。
 * @return 成功返回实例；参数、重复、容量或分配失败返回 NULL，不改写旧对象。
 * @note 启动阶段串行调用；同一总线共用同一 HAL 句柄，同一 GPIO 片选不可重复。
 * 配置返回后可释放，HAL 句柄须长期有效；实例字段不得自行修改或释放。
 */
SPIInstance *SPIRegister(const SPI_Init_Config_s *conf);
/** @brief 移除空闲实例并释放。
 * @param[in] instance 已注册实例；注销不与其他任务的提交、查询或注册并行。
 * @return 1 成功；0 为上下文不符、未注册或本总线仍在传输／回调／恢复，不改写对象。
 * @note 仅普通任务；成功后不得继续访问。隔离状态不允许注销。
 */
uint8_t SPIUnregister(SPIInstance *instance);
/** @brief 独占总线发送字节。
 * @param[in,out] spi_ins 已注册的 8 位主机实例。
 * @param[in] ptr_data 至少 len 字节；异步数组保留到查询确认 buffers_released=1。
 * @param[in] len 字节数，1～255。
 * @return HAL_OK 为阻塞完成或异步已接受；HAL_BUSY 为上下文、时钟或异步总线忙；
 * HAL_TIMEOUT 为阻塞等总线 1000 us 到期或 HAL 超时；其他失败返回 HAL_ERROR／HAL 结果。
 * @note 三类收发均限普通任务且中断未屏蔽。阻塞 HAL 超时 2 ms，不调用接收回调。
 * 拒绝请求不改在途数据；提交后失败不能使用接收数据，HAL 可能已写入部分字节。
 */
HAL_StatusTypeDef SPITransmit(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len);
/** @brief 独占总线接收字节，成功异步完成才调用接收回调。
 * @param[in,out] spi_ins 已注册的 8 位双线主机实例。
 * @param[out] ptr_data 至少 len 字节；异步期间不得读取、修改或释放。
 * @param[in] len 字节数，1～255。
 * @return 与 SPITransmit 相同；HAL_OK 启动异步不代表已完成。
 * @note 沿用 SPITransmit 的上下文、等待及失败规则；释放后须检查最终 result。
 */
HAL_StatusTypeDef SPIRecv(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len);
/** @brief 独占总线同时收发等长字节。
 * @param[in,out] spi_ins 已注册的 8 位双线主机实例。
 * @param[out] ptr_data_rx 至少 len 字节，不与发送数组重叠；异步期间保持有效。
 * @param[in] ptr_data_tx 至少 len 字节；异步期间保持不变，不与接收数组重叠。
 * @param[in] len 字节数，1～255。
 * @return 与 SPITransmit 相同；失败接收数据不可用，不承诺恢复旧字节。
 * @note 接收回调通常在中断中执行；HAL 返回前到达的完成延至提交函数内处理。
 * 回调必须短且不阻塞，结束前保留总线；不得重入提交、注销、改模式或超时服务。
 */
HAL_StatusTypeDef SPITransRecv(SPIInstance *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len);
/** @brief 修改空闲总线上的实例模式，不自动配置 IRQ 或 DMA。
 * @param[in,out] spi_ins 已注册实例，仅普通任务调用。
 * @param[in] mode 三种已定义模式之一。
 * @return HAL_OK 已修改；HAL_BUSY 上下文或总线忙；HAL_ERROR 参数错误，失败不改模式。
 */
HAL_StatusTypeDef SPISetMode(SPIInstance *spi_ins, SPI_TXRX_MODE_e mode);
/** @brief 检查异步期限及错误，停止硬件后释放本次事务。
 * @return HAL_OK 检查完成（可包含已处理的失败）；HAL_BUSY 上下文不符；HAL_ERROR 存在隔离总线。
 * @note 普通任务约每 10 ms 调用；elapsed >= timeout 判超时，使用 HAL_GetTick 毫秒时钟。
 * 调度和 HAL 停止另有耗时，不保证精确截止。停止失败保留异步数组及占用至系统重新初始化。
 */
HAL_StatusTypeDef SPIProcessTimeouts(void);
/** @brief 复制状态，不消耗结果、不推进超时。
 * @param[in] spi_ins 已注册且未并行注销的实例。
 * @param[out] output 非空；失败保持原值。
 * @return HAL_OK 复制成功，HAL_ERROR 参数错误或未注册。
 */
HAL_StatusTypeDef SPIGetTransferState(const SPIInstance *spi_ins, SPITransferState_t *output);
#endif
