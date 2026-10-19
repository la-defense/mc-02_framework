/** @file bsp_iic.h
* @brief I²C 设备登记、普通收发及寄存器访问。
* @note 调用接口前先完成 HAL I²C、GPIO 和所需 DMA 配置；设备登记只在启动阶段串行进行。
 */
#ifndef BSP_IIC_H
#define BSP_IIC_H

#include "i2c.h"
#include <stdint.h>

#define IIC_DEVICE_CNT 1u   // 当前板级配置使用的物理 I²C 外设数量。
#define MX_IIC_SLAVE_CNT 8u // 登记表可保存的设备数量，与物理总线数量分开计算。

/* i2c 工作模式枚举 */
typedef enum
{
    // 基本工作模式
    IIC_BLOCK_MODE = 0, // 阻塞模式
    IIC_IT_MODE,        // 中断模式
    IIC_DMA_MODE,       // DMA模式

} IIC_Work_Mode_e;

/* I2C MEM工作模式枚举,这两种方法都是阻塞 */
typedef enum
{
    IIC_READ_MEM = 0, // 读取从机内部的寄存器or内存
    IIC_WRITE_MEM,    // 写入从机内部的寄存器or内存
} IIC_Mem_Mode_e;

/* Seq传输工作模式枚举,注意HOLD_ON要在IT或DMA下使用 */
// 必须以IIC_RELEASE为最后一次传输,否则会导致总线占有权无法释放
typedef enum
{
    IIC_SEQ_RELEASE,    // 完成传输后释放总线占有权,这是默认的传输方式
    IIC_SEQ_HOLDON = 1, // 保持总线占有权不释放,只支持IT和DMA模式(不能与RELEASE同为0)
} IIC_Seq_Mode_e;

/* i2c实例 */
typedef struct iic_temp_s
{
    I2C_HandleTypeDef *handle; // i2c handle
    uint8_t dev_address;       // 暂时只支持7位地址(还有一位是读写位)

    IIC_Work_Mode_e work_mode;             // 工作模式
    uint8_t *rx_buffer;                    // 接收缓冲区指针
    uint8_t rx_len;                        // 接收长度
    void (*callback)(struct iic_temp_s *); // 接收完成后的回调函数

    void *id; // 用于标识i2c instance
} IICInstance;

/* I2C 初始化结构体配置 */
typedef struct
{
    I2C_HandleTypeDef *handle;       // i2c handle
    uint8_t dev_address;             // 暂时只支持7位地址(还有一位是读写位),注意不需要左移
    IIC_Work_Mode_e work_mode;       // 工作模式
    void (*callback)(IICInstance *); // 接收完成后的回调函数
    void *id;                        // 用于标识i2c instance
} IIC_Init_Config_s;

/**
 * @brief 检查配置并登记一个 I²C 设备，不配置底层外设。
 * @param[in] conf 配置地址；handle 非空，dev_address 为未左移的 0x00～0x7F 地址，
 *                 work_mode 必须是阻塞、中断或 DMA 模式；callback 和 id 可为空。
 * @return 登记成功返回长期有效的实例；空配置、非法字段、重复设备、容量满或分配失败返回 NULL。
 * @note 同一 HAL 句柄和同一 7 位地址只能登记一次；相同地址可用于不同句柄，不同地址可共用句柄。
 *       成功实例由登记表持有，调用方不得释放；配置可在返回后释放，HAL 句柄须持续有效。
 *       只允许启动阶段串行调用。登记失败不占用容量，也不更改已有实例。
 */
IICInstance *IICRegister(const IIC_Init_Config_s *conf);

/** @brief 改写实例的工作模式，不等待上次传输结束。
* @param[in,out] iic 有效实例，不能为 NULL。
* @param[in] mode 阻塞、IT 或 DMA，须在总线空闲时切换。
 */
void IICSetMode(IICInstance *iic, IIC_Work_Mode_e mode);

/** @brief 按当前模式发送原始字节，不检查返回状态。
* @param[in] iic 有效实例，不能为 NULL。
* @param[in] data 至少可读 size 字节；异步发送完成前不可改写或释放。
* @param[in] size 字节数。
* @param[in] mode RELEASE 为最后一段，HOLDON 保持连续传输，仅 IT、DMA 支持。
* @note 阻塞模式的 HAL 等待参数为 100 ms；非法模式会停在循环中。
 */
void IICTransmit(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e mode);

/** @brief 按当前模式接收数据，失败不会自动清输出，也不报告状态。
* @param[in,out] iic 有效实例，不能为 NULL；接收地址保存到实例中供回调使用。
* @param[out] data 至少可写 size 字节，异步完成前须保持有效，不可提前读取结果。
* @param[in] size 字节数；实例 rx_len 只有 8 位，依赖此字段的回调须将长度限制在 255 内。
* @param[in] mode RELEASE 为最后一段，HOLDON 保持连续传输，仅 IT、DMA 支持。
* @note 阻塞模式等待参数为 100 ms，不调用异步完成回调；非法模式持续等待。
 */
void IICReceive(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e mode);

/** @brief 阻塞读写寄存器，不受实例异步工作模式影响，HAL 等待参数为 100 ms。
* @param[in] iic 有效实例，不能为 NULL。
* @param[in] mem_addr 从机寄存器地址，宽度由 mem8bit_flag 选择。
* @param[in,out] data 至少有 size 字节可读或可写空间，取决于 mode，不检查 NULL。
* @param[in] size 字节数。
* @param[in] mode IIC_READ_MEM 读，IIC_WRITE_MEM 写，其他值持续等待。
* @param[in] mem8bit_flag 非 0 使用 8 位地址，0 使用 16 位地址。
* @note 不返回 HAL 状态；读失败时不能假定全部字节已更新，函数也不清零数组。
 */
void IICAccessMem(IICInstance *iic, uint16_t mem_addr, uint8_t *data, uint16_t size, IIC_Mem_Mode_e mode, uint8_t mem8bit_flag);

#endif // BSP_IIC_H
