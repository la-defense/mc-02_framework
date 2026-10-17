/**
 * @file ist8310.c
 * @brief 初始化可选 IST8310，接到数据就绪通知后读取并换算三轴磁场。
 *
 * 初始化先登记 I²C/GPIO、复位，再检查 ID 和配置；GPIO 回调中阻塞读6字节后直接解析。
 * 当前应用未调用本模块。公开入口的配置改写、等待条件和失败处理见 ist8310.h。
 */
#include "bsp_dwt.h"
#include "ist8310.h"
#include "bsp_log.h"
#include <memory.h>
#include <stdlib.h>

// 保存最近成功初始化的实例；回调通过 I²C/GPIO 的 id 找到各自的实例。
static IST8310Instance *ist8310_instance = NULL;

#define IST8310_WRITE_REG_NUM 4     // 要逐项写入并读回的配置个数
#define IST8310_DATA_REG 0x03       // 三轴数据的起始寄存器地址
#define IST8310_WHO_AM_I 0x00       // 器件 ID 寄存器地址
#define IST8310_WHO_AM_I_VALUE 0x10 // 初始化要求读到的器件 ID

// 每行依次为寄存器地址、写入值、读回不同时用于日志的编号；第三列不作为函数返回值。
static uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] = {
    {0x0B, 0x08, 0x01},  // 数据就绪通知设置
    {0x41, 0x09, 0x02},  // 采样平均设置
    {0x42, 0xC0, 0x03},  // 初始化要求的固定配置值
    {0x0A, 0x0B, 0x04}}; // 本驱动使用的工作设置

/* iic 和它的 id 必须指向已登记对象；将缓冲区前6字节换算为 x/y/z 轴磁场。 */
static void IST8310Decode(IICInstance *iic)
{
    int16_t temp[3];
    IST8310Instance *ist = (IST8310Instance *)(iic->id); // 登记时将所属 IST8310 实例存入 id

    // 先复制到三个 int16_t，再逐轴乘0.3，结果单位为 µT。
    memcpy(temp, ist->iic_buffer, 6 * sizeof(uint8_t));
    for (uint8_t i = 0; i < 3; i++)
        ist->mag[i] = (float)temp[i] * MAG_SEN;
}

/* 数据就绪 GPIO 回调：通过 id 找到实例，阻塞读取后直接调用解析。 */
static void IST8310StartTransfer(GPIOInstance *gpio)
{
    IST8310Instance *ist_for_transfer = (IST8310Instance *)gpio->id;
    // 从0x03开始读6字节；调用会等待返回，底层 HAL 使用100ms超时参数。
    IICAccessMem(ist_for_transfer->iic, IST8310_DATA_REG, ist_for_transfer->iic_buffer, 6, IIC_READ_MEM, 1);
    // IICAccessMem 不返回读写状态，读取失败也会继续解析当前缓冲区。
    IST8310Decode(ist_for_transfer->iic);
}

IST8310Instance *IST8310Init(IST8310_Init_Config_s *config)
{
    static const uint8_t sleepTime = 50; // 两次复位等待均为50ms
    uint8_t check_who_i_am = 0;          // 先接收 ID，后续复用来接收配置读回值

    // 分配实例后将这块 RAM 置0，避免指针和数据带有旧内存内容。
    IST8310Instance *ist = (IST8310Instance *)malloc(sizeof(IST8310Instance));
    memset(ist, 0, sizeof(IST8310Instance));

    // 三个底层对象都记住所属实例，回调据此找到缓冲区和三轴结果。
    config->iic_config.id = config->gpio_conf_exti.id = config->gpio_conf_rst.id = ist;
    // 同时登记 I²C 解析回调和 GPIO 数据就绪回调；本文件阻塞读取后还会直接调用解析。
    config->iic_config.callback = IST8310Decode;
    config->gpio_conf_exti.gpio_model_callback = IST8310StartTransfer;
    // 登记对象只复制配置字段，I²C 和 GPIO 的硬件初始化需在调用前完成。
    ist->iic = IICRegister(&config->iic_config);
    ist->gpio_exti = GPIORegister(&config->gpio_conf_exti);
    ist->gpio_rst = GPIORegister(&config->gpio_conf_rst);

    // 复位引脚先拉低再拉高，各等待50ms，让传感器完成复位。
    GPIOReset(ist->gpio_rst);
    HAL_Delay(sleepTime);
    GPIOSet(ist->gpio_rst);
    HAL_Delay(sleepTime);

    // ID 不等于0x10就返回 NULL；前面申请和登记的对象在这里不会释放。
    IICAccessMem(ist->iic, IST8310_WHO_AM_I, &check_who_i_am, 1, IIC_READ_MEM, 1);
    if (check_who_i_am != IST8310_WHO_AM_I_VALUE)
        return NULL;

    // 逐项写入4个配置，每次读回同一个配置寄存器，再与写入值比较。
    for (uint8_t i = 0; i < IST8310_WRITE_REG_NUM; i++)
    {
        IICAccessMem(ist->iic, ist8310_write_reg_data_error[i][0], &ist8310_write_reg_data_error[i][1], 1, IIC_WRITE_MEM, 1);
        IICAccessMem(ist->iic, ist8310_write_reg_data_error[i][0], &check_who_i_am, 1, IIC_READ_MEM, 1);
        // 配置读回不符时持续打印本项编号，停在这里，不执行后续配置或返回。
        if (check_who_i_am != ist8310_write_reg_data_error[i][1])
            while (1)
                LOGERROR("[ist8310] init error, code %d", ist8310_write_reg_data_error[i][2]);
    }

    ist8310_instance = ist; // 只有 ID 和全部配置检查通过后才更新此记录
    return ist;
}
