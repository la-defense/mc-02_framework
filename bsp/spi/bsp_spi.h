/** @file bsp_spi.h
* @brief SPI1、SPI2 设备实例和收发接口；先配置 HAL 外设、片选 GPIO 及所需中断、DMA。
* @note Transmit、Recv 是旧入口，调用方自行保证同一总线空闲；TransRecv 另检查总线使用和等待条件。
*       异步期间发送和接收数组必须保留，完成或错误回调结束前不能改写、释放或注销实例。
 */
#include "spi.h"
#include "stdint.h"
#include "gpio.h"

/* 根据开发板引出的spi引脚以及CubeMX中的初始化配置设定 */
#define SPI_DEVICE_CNT 2       // 本封装当前只识别 SPI1、SPI2，两路总线分别保存状态。
#define MX_SPI_BUS_SLAVE_CNT 4 // 单个spi总线上挂载的从机数目

/* spi transmit recv mode enumerate*/
typedef enum
{
    SPI_BLOCK_MODE = 0, // 默认使用阻塞模式
    SPI_IT_MODE,
    SPI_DMA_MODE,
} SPI_TXRX_MODE_e;

/* SPI实例结构体定义 */
typedef struct spi_ins_temp
{
    SPI_HandleTypeDef *spi_handle; // SPI外设handle
    GPIO_TypeDef *GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等

    SPI_TXRX_MODE_e spi_work_mode; // 传输工作模式
    uint8_t rx_size;               // 本次接收的数据长度
    uint8_t *rx_buffer;            // 本次接收的数据缓冲区
    uint8_t CS_State;                       // 片选信号状态,用于中断模式下的片选控制
    volatile uint8_t *cs_pin_state;         // 总线忙标志(中断里写、任务里轮询 → 必须 volatile)
    void (*callback)(struct spi_ins_temp *); // 接收回调函数
    void *id;                                // 模块指针
} SPIInstance;

/* 接收回调函数定义,包含SPI的module按照此格式构建回调函数 */
typedef void (*spi_rx_callback)(SPIInstance *);

// @todo: 这里可以将GPIO_TypeDef *GPIOx; uint16_t cs_pin合并为bsp_gpio以简化代码实现
/* SPI初始化配置,其实基本和SPIIstance一模一样,为了代码风格统一因此再次定义 */
typedef struct
{
    SPI_HandleTypeDef *spi_handle; // SPI外设handle
    GPIO_TypeDef *GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等

    SPI_TXRX_MODE_e spi_work_mode; // 传输工作模式

    spi_rx_callback callback; // 接收回调函数
    void *id;                 // 模块指针
} SPI_Init_Config_s;

/** @brief 保存 SPI 配置，返回用于后续操作的实例。
* @param[in] conf 有效配置；只支持 SPI1、SPI2、非零片选引脚及三种已定义模式。
* @return 成功返回长期有效对象；参数、重名或分配不符返回 NULL。
* @note 实例数组只有 SPI_DEVICE_CNT 项（当前 2 项），调用方须遵守该容量，不能按 4 从机宏超额注册。
 */
SPIInstance *SPIRegister(SPI_Init_Config_s *conf);
/** @brief 将片选拉高，移出实例表并释放对象。
* @param[in] instance 已停止全部传输及回调的对象，成功后不可再使用。
* @return 1 为已找到并释放，0 为 NULL 或未注册；不负责等待传输完成。
 */
uint8_t SPIUnregister(SPIInstance *instance);

/** @brief 拉低片选后发送数据，阻塞结束或异步完成后拉高。
* @param[in] spi_ins 有效实例，不能为 NULL，调用方保证总线空闲。
* @param[in] ptr_data 至少可读 len 字节；IT、DMA 完成前须保持不变。
* @param[in] len 字节数。
* @note 旧入口不报告 HAL 状态；阻塞等待参数为 1000 ms，须保证 HAL 毫秒计时可运行。
 */
void SPITransmit(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len);

/** @brief 拉低片选后接收数据，记录接收数组供异步回调使用。
* @param[in,out] spi_ins 有效实例，不能为 NULL，调用方保证总线空闲。
* @param[out] ptr_data 至少可写 len 字节；IT、DMA 完成前须保持有效，不可提前读取。
* @param[in] len 字节数。
* @note 旧入口不报告 HAL 状态，失败也不清数组；阻塞等待参数为 1000 ms，不调用完成回调。
 */
void SPIRecv(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len);

/** @brief 同时收发同样长度的数据，先检查总线是否能使用。
* @param[in,out] spi_ins 有效实例；NULL、外设或片选信息不完整时返回 HAL_ERROR。
* @param[out] ptr_data_rx 至少可写 len 字节，不能为 NULL；异步完成前保持有效。
* @param[in] ptr_data_tx 至少可读 len 字节，不能为 NULL；异步完成前不得改写。
* @param[in] len 字节数，不能为 0。
* @return HAL_OK 表示阻塞完成或异步已启动；HAL_BUSY 表示阻塞条件不满足；
*         等待总线达到 1000 us 返回 HAL_TIMEOUT，其他 HAL 错误原样返回。
* @note 阻塞收发的 HAL 等待参数为 2 ms；须在普通任务中且 HAL 毫秒计时可运行，等待期间防止计时被停止。
*       失败不会清接收数组；异步启动成功还需等待完成回调，不能立即使用接收结果。
 */

HAL_StatusTypeDef SPITransRecv(SPIInstance *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len);

/** @brief 改写实例的传输模式，不等待或中断当前传输。
* @param[in,out] spi_ins 有效实例，不能为 NULL，须在总线空闲时调用。
* @param[in] spi_mode 阻塞、IT、DMA；其他值会进入持续等待循环。
 */
void SPISetMode(SPIInstance *spi_ins, SPI_TXRX_MODE_e spi_mode);
