/**
* @file bmi088.c
* @brief 实现 BMI088 的寄存器读写、采样、启动检查和在线标定。
* BMI088Acquire 只在整份样本读取成功后更新输出；标定由 INS 任务执行。
 */
#include "bmi088_regNdef.h"
#include "bmi088.h"
#include "bmi088_temperature.h"
#include "vector3.h"
#include "daemon.h"
#include "bsp_log.h"
#include "bsp_param.h"
#include "robot_safety.h"
#include "task_monitor.h"
#include "config/build_selection.h"
#if MC02_FEATURE_HEATER
#include "imu_heater.h"
#endif
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

volatile uint32_t bmi088_init_error = BMI088_NO_ERROR;
volatile uint32_t bmi088_init_retry_count = 0u;
volatile uint32_t bmi088_init_stage = BMI088_INIT_STAGE_IDLE;
volatile uint32_t bmi088_init_failure_stage = BMI088_INIT_STAGE_IDLE;
volatile uint32_t bmi088_init_acc_status = HAL_OK;
volatile uint32_t bmi088_init_gyro_status = HAL_OK;
volatile uint32_t bmi088_init_acc_chip_id = 0u;
volatile uint32_t bmi088_init_gyro_chip_id = 0u;

static void BMI088RecordInitFailureStage(void)
{
    if (bmi088_init_failure_stage == BMI088_INIT_STAGE_IDLE)
        bmi088_init_failure_stage = bmi088_init_stage;
}

/* 标定诊断计数可供 Live Watch / OpenOCD 查看重试、采样和超时进度。 */
volatile uint32_t cali_diag_outer = 0;        /* 外层 do-while 执行次数 */
volatile uint32_t cali_diag_inner = 0;        /* 内层循环最后一次的 i */
volatile uint32_t cali_diag_acq_ok = 0;       /* BMI088Acquire 返回 1 次数 */
volatile uint32_t cali_diag_acq_fail = 0;     /* BMI088Acquire 返回 0 次数 */
volatile uint32_t cali_diag_timeout_hit = 0;  /* 12s 超时分支命中次数 */
volatile uint32_t cali_diag_diff_break = 0;   /* 因数据跳动过大提前 break 次数 */
volatile uint32_t cali_diag_elapsed_ms = 0;   /* 进入外层时已耗时(ms) */
volatile uint32_t cali_diag_finish = 0;       /* 标定函数正常返回次数 */
volatile float cali_diag_acc_norm = 0.0f;     /* 最近一次 |acc| */
volatile float cali_diag_gyro_x = 0.0f;       /* 最近一次 gyro[0] */
volatile uint32_t cali_diag_acq_us_max = 0;   /* BMI088Acquire 单次最大耗时(us) */
volatile uint32_t cali_diag_delay_us_max = 0; /* DWT_Delay(0.5ms) 实际最大耗时(us) */
volatile uint32_t cali_diag_iter_us_max = 0;  /* 内层单轮最大总耗时(us) */

// ---------------------------以下私有函数,用于读写BMI088寄存器封装,blocking--------------------------------//
/**
 * @brief 读取BMI088寄存器Accel. BMI088要求在不释放CS的情况下连续读取
 *
 * @param bmi088 待读取的BMI088实例
 * @param reg 待读取的寄存器地址
 * @param dataptr 读取到的数据存放的指针
 * @param len 读取长度
 */
static HAL_StatusTypeDef BMI088AccelRead(BMI088Instance *bmi088, uint8_t reg, uint8_t *dataptr, uint8_t len)
{
    /* The local receive buffer is valid only for a synchronous SPI transfer. */
    if (bmi088 == NULL || bmi088->spi_acc == NULL || dataptr == NULL || len == 0u || len > 6u ||
        bmi088->spi_acc->spi_work_mode != SPI_BLOCK_MODE)
        return HAL_ERROR;
    uint8_t tx[8] = {0};
    uint8_t rx[8] = {0};
    tx[0] = (uint8_t)(0x80u | reg);
    HAL_StatusTypeDef status = SPITransRecv(bmi088->spi_acc, rx, tx, (uint8_t)(len + 2u));
    if (status == HAL_OK)
        memcpy(dataptr, rx + 2, len);
    return status;
}

/**
 * @brief 读取BMI088寄存器Gyro, BMI088要求在不释放CS的情况下连续读取
 *
 * @param bmi088 待读取的BMI088实例
 * @param reg  待读取的寄存器地址
 * @param dataptr 读取到的数据存放的指针
 * @param len 读取长度
 */
static HAL_StatusTypeDef BMI088GyroRead(BMI088Instance *bmi088, uint8_t reg, uint8_t *dataptr, uint8_t len)
{
    /* The local receive buffer is valid only for a synchronous SPI transfer. */
    if (bmi088 == NULL || bmi088->spi_gyro == NULL || dataptr == NULL || len == 0u || len > 6u ||
        bmi088->spi_gyro->spi_work_mode != SPI_BLOCK_MODE)
        return HAL_ERROR;
    uint8_t tx[7] = {0};
    uint8_t rx[7] = {0};
    tx[0] = (uint8_t)(0x80u | reg);
    HAL_StatusTypeDef status = SPITransRecv(bmi088->spi_gyro, rx, tx, (uint8_t)(len + 1u));
    if (status == HAL_OK)
        memcpy(dataptr, rx + 1, len);
    return status;
}

/**
 * @brief 写accel寄存器.对spitransmit形式上的封装
 * @attention 只会向目标reg写入一个字节,因为只有1个字节所以直接传值(指针是32位反而浪费)
 *
 * @param bmi088 待写入的BMI088实例
 * @param reg  待写入的寄存器地址
 * @param data 待写入的数据(注意不是指针)
 */
static HAL_StatusTypeDef BMI088AccelWriteSingleReg(BMI088Instance *bmi088, uint8_t reg, uint8_t data)
{
    if (bmi088 == NULL || bmi088->spi_acc == NULL ||
        bmi088->spi_acc->spi_work_mode != SPI_BLOCK_MODE)
        return HAL_ERROR;
    uint8_t tx[2] = {reg, data};
    uint8_t rx[2] = {0};
    return SPITransRecv(bmi088->spi_acc, rx, tx, sizeof(tx));
}

/**
 * @brief 写gyro寄存器.形式上的封装
 * @attention 只会向目标reg写入一个字节,因为只有1个字节所以直接传值(指针是32位反而浪费)
 *
 * @param bmi088 待写入的BMI088实例
 * @param reg  待写入的寄存器地址
 * @param data 待写入的数据(注意不是指针)
 */
static HAL_StatusTypeDef BMI088GyroWriteSingleReg(BMI088Instance *bmi088, uint8_t reg, uint8_t data)
{
    if (bmi088 == NULL || bmi088->spi_gyro == NULL ||
        bmi088->spi_gyro->spi_work_mode != SPI_BLOCK_MODE)
        return HAL_ERROR;
    uint8_t tx[2] = {reg, data};
    uint8_t rx[2] = {0};
    return SPITransRecv(bmi088->spi_gyro, rx, tx, sizeof(tx));
}
// -------------------------以上为私有函数,封装了BMI088寄存器读写函数,blocking--------------------------------//

// -------------------------以下为私有函数,用于初始化BMI088acc和gyro的硬件和配置--------------------------------//
#define BMI088REG 0
#define BMI088DATA 1
#define BMI088ERROR 2
// BMI088初始化配置数组for accel,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
static uint8_t BMI088_Accel_Init_Table[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}};
// BMI088初始化配置数组for gyro,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
static uint8_t BMI088_Gyro_Init_Table[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}};
// @attention : 以上两个数组配合各自的初始化函数使用. 若要修改请参照BMI088 datasheet

/**
 * @brief 初始化BMI088加速度计,提高可读性分拆功能
 *
 * @param bmi088 待初始化的BMI088实例
 * @return uint8_t BMI088ERROR CODE if any problems here
 */
static uint8_t BMI088AccelInit(BMI088Instance *bmi088)
{
    uint8_t whoami_check = 0;

    // 加速度计以I2C模式启动,需要一次上升沿来切换到SPI模式,因此进行一次fake write
    bmi088_init_stage = BMI088_INIT_STAGE_ACCEL_INTERFACE_PROBE;
    HAL_StatusTypeDef status = BMI088AccelRead(bmi088, BMI088_ACC_CHIP_ID, &whoami_check, 1);
    bmi088_init_acc_status = (uint32_t)status;
    if (status != HAL_OK)
    {
        BMI088RecordInitFailureStage();
        return BMI088_NO_SENSOR;
    }
    TaskMonitorDelayMs(TASK_MONITOR_INS, 1u);

    bmi088_init_stage = BMI088_INIT_STAGE_ACCEL_SOFT_RESET;
    status = BMI088AccelWriteSingleReg(bmi088, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    bmi088_init_acc_status = (uint32_t)status;
    if (status != HAL_OK)
    {
        BMI088RecordInitFailureStage();
        return BMI088_NO_SENSOR;
    }
    TaskMonitorDelayMs(TASK_MONITOR_INS, BMI088_COM_WAIT_SENSOR_TIME);

    /* The accelerometer's first SPI read after soft reset is a dummy read. */
    bmi088_init_stage = BMI088_INIT_STAGE_ACCEL_DUMMY_READ;
    whoami_check = 0u;
    status = BMI088AccelRead(bmi088, BMI088_ACC_CHIP_ID, &whoami_check, 1);
    bmi088_init_acc_status = (uint32_t)status;
    if (status != HAL_OK)
    {
        BMI088RecordInitFailureStage();
        return BMI088_NO_SENSOR;
    }

    // 检查ID,如果不是0x1E(bmi088 whoami寄存器值),则返回错误
    bmi088_init_stage = BMI088_INIT_STAGE_ACCEL_ID_CHECK;
    whoami_check = 0u;
    status = BMI088AccelRead(bmi088, BMI088_ACC_CHIP_ID, &whoami_check, 1);
    bmi088_init_acc_status = (uint32_t)status;
    bmi088_init_acc_chip_id = whoami_check;
    if (status != HAL_OK || whoami_check != BMI088_ACC_CHIP_ID_VALUE)
    {
        BMI088RecordInitFailureStage();
        return BMI088_NO_SENSOR;
    }
    TaskMonitorDelayMs(TASK_MONITOR_INS, 1u);
    // 初始化寄存器,提高可读性
    uint8_t reg = 0, data = 0;
    BMI088_ERORR_CODE_e error = 0;
    // 使用sizeof而不是magic number,这样如果修改了数组大小,不用修改这里的代码;或者使用宏定义
    for (uint8_t i = 0; i < sizeof(BMI088_Accel_Init_Table) / sizeof(BMI088_Accel_Init_Table[0]); i++)
    {
        bmi088_init_stage = BMI088_INIT_STAGE_ACCEL_CONFIG;
        reg = BMI088_Accel_Init_Table[i][BMI088REG];
        data = BMI088_Accel_Init_Table[i][BMI088DATA];
        status = BMI088AccelWriteSingleReg(bmi088, reg, data);
        bmi088_init_acc_status = (uint32_t)status;
        if (status != HAL_OK)
        {
            BMI088RecordInitFailureStage();
            return BMI088_NO_SENSOR;
        }
        TaskMonitorDelayMs(TASK_MONITOR_INS, 10u);
        status = BMI088AccelRead(bmi088, reg, &data, 1); // 写完之后立刻读回检查
        bmi088_init_acc_status = (uint32_t)status;
        TaskMonitorDelayMs(TASK_MONITOR_INS, 10u);
        if (status != HAL_OK)
        {
            BMI088RecordInitFailureStage();
            return BMI088_NO_SENSOR;
        }
        if (data != BMI088_Accel_Init_Table[i][BMI088DATA])
            error |= BMI088_Accel_Init_Table[i][BMI088ERROR];
        //{i--;} 可以设置retry次数,如果retry次数用完了,则返回error
    }
    if (error != BMI088_NO_ERROR)
        BMI088RecordInitFailureStage();
    return error;
}

/**
 * @brief 初始化BMI088陀螺仪,提高可读性分拆功能
 *
 * @param bmi088 待初始化的BMI088实例
 * @return uint8_t BMI088ERROR CODE
 */
static uint8_t BMI088GyroInit(BMI088Instance *bmi088)
{
    // 后续添加reset和通信检查?
    // code to go here ...
    bmi088_init_stage = BMI088_INIT_STAGE_GYRO_SOFT_RESET;
    HAL_StatusTypeDef status = BMI088GyroWriteSingleReg(
        bmi088, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    bmi088_init_gyro_status = (uint32_t)status;
    if (status != HAL_OK)
    {
        BMI088RecordInitFailureStage();
        return BMI088_NO_SENSOR;
    }
    TaskMonitorDelayMs(TASK_MONITOR_INS, BMI088_LONG_DELAY_TIME);

    // 检查ID,如果不是0x0F(bmi088 whoami寄存器值),则返回错误
    bmi088_init_stage = BMI088_INIT_STAGE_GYRO_ID_CHECK;
    uint8_t whoami_check = 0;
    status = BMI088GyroRead(bmi088, BMI088_GYRO_CHIP_ID, &whoami_check, 1);
    bmi088_init_gyro_status = (uint32_t)status;
    bmi088_init_gyro_chip_id = whoami_check;
    if (status != HAL_OK || whoami_check != BMI088_GYRO_CHIP_ID_VALUE)
    {
        BMI088RecordInitFailureStage();
        return BMI088_NO_SENSOR;
    }
    TaskMonitorDelayMs(TASK_MONITOR_INS, 1u);

    // 初始化寄存器,提高可读性
    uint8_t reg = 0, data = 0;
    BMI088_ERORR_CODE_e error = 0;
    // 使用sizeof而不是magic number,这样如果修改了数组大小,不用修改这里的代码;或者使用宏定义
    for (uint8_t i = 0; i < sizeof(BMI088_Gyro_Init_Table) / sizeof(BMI088_Gyro_Init_Table[0]); i++)
    {
        bmi088_init_stage = BMI088_INIT_STAGE_GYRO_CONFIG;
        reg = BMI088_Gyro_Init_Table[i][BMI088REG];
        data = BMI088_Gyro_Init_Table[i][BMI088DATA];
        status = BMI088GyroWriteSingleReg(bmi088, reg, data);
        bmi088_init_gyro_status = (uint32_t)status;
        if (status != HAL_OK)
        {
            BMI088RecordInitFailureStage();
            return BMI088_NO_SENSOR;
        }
        TaskMonitorDelayMs(TASK_MONITOR_INS, 1u);
        status = BMI088GyroRead(bmi088, reg, &data, 1); // 写完之后立刻读回对应寄存器检查是否写入成功
        bmi088_init_gyro_status = (uint32_t)status;
        TaskMonitorDelayMs(TASK_MONITOR_INS, 1u);
        if (status != HAL_OK)
        {
            BMI088RecordInitFailureStage();
            return BMI088_NO_SENSOR;
        }
        if (data != BMI088_Gyro_Init_Table[i][BMI088DATA])
            error |= BMI088_Gyro_Init_Table[i][BMI088ERROR];
        //{i--;} 可以设置retry次数,尝试重新写入.如果retry次数用完了,则返回error
    }

    if (error != BMI088_NO_ERROR)
        BMI088RecordInitFailureStage();
    return error;
}
// -------------------------以上为私有函数,用于初始化BMI088acc和gyro的硬件和配置--------------------------------//

// -------------------------以下为私有函数,private用于IT模式下的中断处理---------------------------------//

/**
 * @brief 待编写,很快!
 *
 */
static void BMI088AccSPIFinishCallback(SPIInstance *spi)
{
    // static BMI088Instance *bmi088;
    // bmi088 = (BMI088Instance *)(spi->id);
   
    // 若第一次读取加速度,则在这里启动温度读取
// 若采用异步姿态更新，可在这里唤醒负责处理新测量的任务。
}

static void BMI088GyroSPIFinishCallback(SPIInstance *spi)
{
    // static BMI088Instance *bmi088;
    // bmi088 = (BMI088Instance *)(spi->id);
    // 若不是异步,啥也不做;否则启动姿态的预测步(propagation)
}

static void BMI088AccINTCallback(GPIOInstance *gpio)
{
    BMI088Instance *bmi088 = (BMI088Instance *)gpio->id;
    uint8_t buf[6] = {0};
    if (BMI088AccelRead(bmi088, BMI088_ACCEL_XOUT_L, buf, 6u) != HAL_OK)
        return;
    for (uint8_t i = 0; i < 3u; ++i)
        bmi088->acc[i] = bmi088->acc_coef * (float)(int16_t)(((uint16_t)buf[2u * i + 1u] << 8u) | buf[2u * i]);
    bmi088->acc_sample_timestamp_ms = HAL_GetTick();
    bmi088->update_flag.acc = 1u;
    bmi088->update_flag.imu_ready = bmi088->update_flag.gyro;
}

static void BMI088GyroINTCallback(GPIOInstance *gpio)
{
    BMI088Instance *bmi088 = (BMI088Instance *)gpio->id;
    uint8_t buf[6] = {0};
    if (BMI088GyroRead(bmi088, BMI088_GYRO_X_L, buf, 6u) != HAL_OK)
        return;
    for (uint8_t i = 0; i < 3u; ++i)
        bmi088->gyro[i] = bmi088->BMI088_GYRO_SEN * (float)(int16_t)(((uint16_t)buf[2u * i + 1u] << 8u) | buf[2u * i]);
    bmi088->gyro_sample_timestamp_ms = HAL_GetTick();
    bmi088->update_flag.gyro = 1u;
    bmi088->update_flag.imu_ready = bmi088->update_flag.acc;
}

// -------------------------以上为私有函数,private用于IT模式下的中断处理---------------------------------//

// -------------------------以下为公有函数,用于注册BMI088,标定和数据读取--------------------------------//

/* 检查样本中的温度、三轴加速度和角速度，拒绝 NaN 或无穷大。 */
static uint8_t BMI088SampleValuesAreFinite(const BMI088_Data_t *sample)
{
    if (sample == NULL || !isfinite(sample->temperature))
        return 0u;
    // 三个轴都必须是可用数值，整份样本才会交给调用者。
    for (uint8_t axis = 0; axis < 3u; ++axis)
    {
        if (!isfinite(sample->acc[axis]) || !isfinite(sample->gyro[axis]))
            return 0u;
    }
    return 1u;
}


BMI088_AcquireStatus_e BMI088Acquire(BMI088Instance *bmi088, BMI088_Data_t *data_store)
{
    if (bmi088 == NULL || data_store == NULL)
        return BMI088_ACQUIRE_NO_DATA;

    // 先在局部变量中准备数据；所有检查通过后再一次写入实例和输出。
    BMI088_Data_t staged = {0};
    uint8_t raw[6] = {0};
    HAL_StatusTypeDef spi_status;

    if (bmi088->work_mode == BMI088_BLOCK_PERIODIC_MODE)
    {
        spi_status = BMI088AccelRead(bmi088, BMI088_ACCEL_XOUT_L, raw, 6u);
        if (spi_status != HAL_OK)
            goto spi_failure;
        // 每两个寄存器字节组成一个有符号轴数据，再乘对应换算系数。
        for (uint8_t i = 0; i < 3u; ++i)
            staged.acc[i] = bmi088->acc_coef * (float)(int16_t)(((uint16_t)raw[2u * i + 1u] << 8u) | raw[2u * i]);

        spi_status = BMI088GyroRead(bmi088, BMI088_GYRO_X_L, raw, 6u);
        if (spi_status != HAL_OK)
            goto spi_failure;
        // 陀螺仪同样按 x、y、z 顺序读取和换算。
        for (uint8_t i = 0; i < 3u; ++i)
            staged.gyro[i] = bmi088->BMI088_GYRO_SEN * (float)(int16_t)(((uint16_t)raw[2u * i + 1u] << 8u) | raw[2u * i]);

        spi_status = BMI088AccelRead(bmi088, BMI088_TEMP_M, raw, 2u);
        if (spi_status != HAL_OK)
            goto spi_failure;
        staged.temperature = BMI088DecodeTemperature(raw[0], raw[1]);
    }
    else if (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE)
    {
        uint32_t now_ms = HAL_GetTick();
        if (!(bmi088->update_flag.imu_ready && bmi088->update_flag.acc && bmi088->update_flag.gyro))
        {
            bmi088->last_acquire_status = BMI088_ACQUIRE_NO_DATA;
            return BMI088_ACQUIRE_NO_DATA;
        }
        // 加速度或角速度数据过期时清除 ready 标志，下次等新中断数据。
        if ((uint32_t)(now_ms - bmi088->acc_sample_timestamp_ms) > BMI088_SAMPLE_MAX_AGE_MS ||
            (uint32_t)(now_ms - bmi088->gyro_sample_timestamp_ms) > BMI088_SAMPLE_MAX_AGE_MS)
        {
            bmi088->update_flag.acc = 0u;
            bmi088->update_flag.gyro = 0u;
            bmi088->update_flag.imu_ready = 0u;
            bmi088->last_acquire_status = BMI088_ACQUIRE_NO_DATA;
            return BMI088_ACQUIRE_NO_DATA;
        }
        memcpy(staged.acc, bmi088->acc, sizeof(staged.acc));
        memcpy(staged.gyro, bmi088->gyro, sizeof(staged.gyro));
        spi_status = BMI088AccelRead(bmi088, BMI088_TEMP_M, raw, 2u);
        if (spi_status != HAL_OK)
            goto spi_failure;
        staged.temperature = BMI088DecodeTemperature(raw[0], raw[1]);
        bmi088->update_flag.acc = 0u;
        bmi088->update_flag.gyro = 0u;
        bmi088->update_flag.imu_ready = 0u;
    }
    else
    {
        bmi088->last_acquire_status = BMI088_ACQUIRE_NO_DATA;
        return BMI088_ACQUIRE_NO_DATA;
    }

    if (!BMI088SampleValuesAreFinite(&staged))
    {
        bmi088->last_acquire_status = BMI088_ACQUIRE_INVALID_DATA;
        return BMI088_ACQUIRE_INVALID_DATA;
    }

    // 成功样本才递增序号、记录时间并对外标记可用。
    staged.sequence = bmi088->sample_sequence + 1u;
    if (staged.sequence == 0u)
        staged.sequence = 1u;
    staged.timestamp_ms = HAL_GetTick();
    staged.valid = 1u;

    memcpy(bmi088->acc, staged.acc, sizeof(staged.acc));
    memcpy(bmi088->gyro, staged.gyro, sizeof(staged.gyro));
    bmi088->temperature = staged.temperature;
    bmi088->sample_sequence = staged.sequence;
    bmi088->last_acquire_status = BMI088_ACQUIRE_OK;
    *data_store = staged;
    return BMI088_ACQUIRE_OK;

spi_failure:
    if (spi_status == HAL_BUSY)
    {
        bmi088->last_acquire_status = BMI088_ACQUIRE_SPI_BUSY;
        return BMI088_ACQUIRE_SPI_BUSY;
    }
    if (spi_status == HAL_TIMEOUT)
    {
        bmi088->last_acquire_status = BMI088_ACQUIRE_SPI_TIMEOUT;
        return BMI088_ACQUIRE_SPI_TIMEOUT;
    }
    bmi088->last_acquire_status = BMI088_ACQUIRE_SPI_ERROR;
    return BMI088_ACQUIRE_SPI_ERROR;
}



/* pre calibrate parameter to go here */
#pragma message "REMEMBER TO SET PRE CALIBRATE PARAMETER IF YOU CHOOSE NOT TO CALIBRATE"
#define BMI088_PRE_CALI_GYRO_X_OFFSET PARAM_IMU_GYRO_OFFSET_DEFAULT_RAD_S
#define BMI088_PRE_CALI_GYRO_Y_OFFSET PARAM_IMU_GYRO_OFFSET_DEFAULT_RAD_S
#define BMI088_PRE_CALI_GYRO_Z_OFFSET PARAM_IMU_GYRO_OFFSET_DEFAULT_RAD_S
#define BMI088_PRE_CALI_G_NORM PARAM_IMU_G_NORM_DEFAULT_M_S2

/* 在线标定最多重试3轮；每轮最多收集6000份样本。 */
#define BMI088_CALI_MAX_ATTEMPTS 3u
/* 标定循环里每隔多少次采样让出一次 CPU(给 daemon 喂狗、给 LCD 显示"标定中") */
#define BMI088_CALI_YIELD_INTERVAL 4u
#define BMI088_CALI_YIELD_MS 1u

uint8_t BMI088CalibrateIMU(BMI088Instance *_bmi088)
{
    uint8_t online_ok = 0;
    uint8_t timed_out = 0; /* 1=超时放弃, 结果不可信 */
    uint16_t attempt = 0;  /* 已尝试轮数(最多 BMI088_CALI_MAX_ATTEMPTS) */

    if (_bmi088->cali_mode == BMI088_CALIBRATE_ONLINE_MODE) // 性感bmi088在线标定,耗时6s
    {
        _bmi088->acc_coef = BMI088_ACCEL_6G_SEN;         // 标定完后要乘以9.805/gNorm
        _bmi088->BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN; // 后续改为从initTable中获取
        // 一次性参数用完就丢,不用static
        /* 计时改用周期计数器(DWT_ProbeStart/ElapsedUs), 不再用 DWT_GetTimeline_s():
           -O2 下 DWT 时间轴(带 64bit 除法与回绕处理的那套)返回值不前进, 导致下面
           12s 超时永远不触发, 标定会无限重试(实测 outer 计数一直涨) → 启动卡死。 */
        uint32_t startCycle;                 // 开始标定的周期计数, 用于超时判断
        uint16_t CaliTimes = 6000;           // 标定次数(6s)
        float gyroMax[3] = {0}, gyroMin[3] = {0};
        float gNormTemp = 0.0f, gNormMax = 0.0f, gNormMin = 0.0f;
        float gyroDiff[3] = {INFINITY, INFINITY, INFINITY};
        float gNormDiff = INFINITY;
        uint16_t valid_samples = 0u;

        BMI088_Data_t raw_data = {0};
        startCycle = DWT_ProbeStart();
        // 环境不合适时重新收集一轮；最多尝试 BMI088_CALI_MAX_ATTEMPTS 次。
        do // 用do while至少执行一次,省得对上面的参数进行初始化
        {  // 标定超时,直接使用预标定参数(如果有)
            cali_diag_outer++;
            attempt++;
            bmi088_calib_attempts = (uint8_t)attempt;
            cali_diag_elapsed_ms = DWT_ProbeElapsedUs(startCycle) / 1000u;
            if (DWT_ProbeElapsedUs(startCycle) > 12010000u) /* 12.01s */
            { // 两次都没有成功就切换标定模式,丢给下一个if处理,使用预标定参数
                cali_diag_timeout_hit++;
                timed_out = 1;
                break;
            }

            DWT_Delay(0.0005);
            _bmi088->gNorm = 0;
            valid_samples = 0u;
            gNormDiff = INFINITY;
            for (uint8_t axis = 0; axis < 3u; ++axis)
                gyroDiff[axis] = INFINITY;
            for (uint8_t i = 0; i < 3; i++) // 重置gNorm和零飘
                _bmi088->gyro_offset[i] = 0;

            // @todo : 这里也有获取bmi088数据的操作,后续与BMI088Acquire合并.注意标定时的工作模式是阻塞,且offset和acc_coef要初始化成0和1,标定完成后再设定为标定值
            while (valid_samples < CaliTimes)
            {
                uint32_t diag_iter = DWT_ProbeStart();
                cali_diag_inner = valid_samples;
                if (DWT_ProbeElapsedUs(startCycle) > 12010000u)
                {
                    cali_diag_timeout_hit++;
                    timed_out = 1u;
                    break;
                }
                /* 周期性让出 CPU，让 daemon 和 LCD 继续运行；标定不直接喂狗。 */
                if (((valid_samples % BMI088_CALI_YIELD_INTERVAL) == 0u) && osKernelRunning())
                    osDelay(BMI088_CALI_YIELD_MS);
                uint32_t diag_acq = DWT_ProbeStart();
                if (BMI088Acquire(_bmi088, &raw_data) != BMI088_ACQUIRE_OK)
                {
                    cali_diag_acq_fail++;
                    DWT_Delay(0.0005);
                    continue;
                }
                cali_diag_acq_ok++;
                // 数学失败的样本不计数、不累加，仍受本轮总超时限制。
                if (Vec3NormF32(raw_data.acc, &gNormTemp) != MATH_OK)
                {
                    DWT_Delay(0.0005);
                    continue;
                }
                valid_samples++;
                uint32_t acq_us = DWT_ProbeElapsedUs(diag_acq);
                if (acq_us > cali_diag_acq_us_max)
                    cali_diag_acq_us_max = acq_us;
                cali_diag_acc_norm = gNormTemp;
                cali_diag_gyro_x = raw_data.gyro[0];
                _bmi088->gNorm += gNormTemp; // 计算范数并累加,最后除以calib times获取单次值
                for (uint8_t ii = 0; ii < 3; ii++)
                    _bmi088->gyro_offset[ii] += raw_data.gyro[ii]; // 因为标定时传感器静止,所以采集到的值就是漂移,累加当前值,最后除以calib times获得零飘

                if (valid_samples == 1u) // 避免未定义的行为(else中)
                {
                    gNormMax = gNormMin = gNormTemp; // 初始化成当前的重力加速度模长
                    for (uint8_t j = 0; j < 3; ++j)
                    {
                        gyroMax[j] = raw_data.gyro[j];
                        gyroMin[j] = raw_data.gyro[j];
                    }
                }
                else // 更新gNorm的Min Max和gyro的minmax
                {
                    gNormMax = gNormMax > gNormTemp ? gNormMax : gNormTemp;
                    gNormMin = gNormMin < gNormTemp ? gNormMin : gNormTemp;
                    for (uint8_t j = 0; j < 3; ++j)
                    {
                        gyroMax[j] = gyroMax[j] > _bmi088->gyro[j] ? gyroMax[j] : _bmi088->gyro[j];
                        gyroMin[j] = gyroMin[j] < _bmi088->gyro[j] ? gyroMin[j] : _bmi088->gyro[j];
                    }
                }

                gNormDiff = gNormMax - gNormMin; // 最大值和最小值的差
                for (uint8_t j = 0; j < 3; ++j)
                    gyroDiff[j] = gyroMax[j] - gyroMin[j]; // 分别计算三轴
                if (gNormDiff > 0.5f ||
                    gyroDiff[0] > 0.15f ||
                    gyroDiff[1] > 0.15f ||
                    gyroDiff[2] > 0.15f)
                {
                    cali_diag_diff_break++;
                    break;         // 超出范围了,重开! remake到while循环,外面还有一层
                }
                uint32_t diag_delay = DWT_ProbeStart();
                DWT_Delay(0.0005); // 休息一会再开始下一轮数据获取,IMU准备数据需要时间
                uint32_t delay_us = DWT_ProbeElapsedUs(diag_delay);
                if (delay_us > cali_diag_delay_us_max)
                    cali_diag_delay_us_max = delay_us;
                uint32_t iter_us = DWT_ProbeElapsedUs(diag_iter);
                if (iter_us > cali_diag_iter_us_max)
                    cali_diag_iter_us_max = iter_us;
            }
            if (valid_samples > 0u)
            {
                _bmi088->gNorm /= (float)valid_samples;
                for (uint8_t i = 0; i < 3; ++i)
                    _bmi088->gyro_offset[i] /= (float)valid_samples;
                _bmi088->temperature = raw_data.temperature;
            }
            // caliTryOutCount++; 保存已经尝试的标定次数?由你.
        } while ((valid_samples < CaliTimes ||
                  gNormDiff > 0.5f ||
                  fabsf(_bmi088->gNorm - 9.8f) > 0.5f ||
                  gyroDiff[0] > 0.15f ||
                  gyroDiff[1] > 0.15f ||
                  gyroDiff[2] > 0.15f ||
                  fabsf(_bmi088->gyro_offset[0]) > 0.01f ||
                  fabsf(_bmi088->gyro_offset[1]) > 0.01f ||
                  fabsf(_bmi088->gyro_offset[2]) > 0.01f) && // 满足条件说明标定环境不好
                 attempt < BMI088_CALI_MAX_ATTEMPTS); // 最多试 3 轮, 避免开机无限重标定

        // 只有按满样本数且所有稳定性检查都通过，才使用这轮标定结果。
        online_ok = (uint8_t)(timed_out == 0 && valid_samples == CaliTimes &&
                              gNormDiff <= 0.5f &&
                              fabsf(_bmi088->gNorm - 9.8f) <= 0.5f &&
                              gyroDiff[0] <= 0.15f && gyroDiff[1] <= 0.15f && gyroDiff[2] <= 0.15f &&
                              fabsf(_bmi088->gyro_offset[0]) <= 0.01f &&
                              fabsf(_bmi088->gyro_offset[1]) <= 0.01f &&
                              fabsf(_bmi088->gyro_offset[2]) <= 0.01f);
    }

    if (online_ok)
    {
        // 通过检查后才用 gNorm 计算系数，避免失败数据导致除以0或产生无效数值。
        _bmi088->acc_coef = BMI088_ACCEL_6G_SEN * (9.805f / _bmi088->gNorm);
        bmi088_calib_temp = _bmi088->temperature;
        /* 注意: RTT 的 printf 实现不支持 %f, 而且遇到 %f 时**不会消费参数**,
           会让后面的 %s 取到错位的参数(见 LOG_PROTO 的结尾 %s) → 把浮点位模式
           当指针解引用 → BusFault/HardFault。所以这里统一用"放大成整数"打印。 */
        BSP_LOG_EVENT_6(EV_MODULES_BMI088_BMI088_C_01, BSP_LOG_FIELD_U32((unsigned)attempt), BSP_LOG_FIELD_I32((long)(_bmi088->gNorm * 1000.0f)), BSP_LOG_FIELD_I32((long)(_bmi088->gyro_offset[0] * 10000.0f)), BSP_LOG_FIELD_I32((long)(_bmi088->gyro_offset[1] * 10000.0f)), BSP_LOG_FIELD_I32((long)(_bmi088->gyro_offset[2] * 10000.0f)), BSP_LOG_FIELD_I32((long)(bmi088_calib_temp * 100.0f)));
    }
    else
    {
        _bmi088->acc_coef = BMI088_ACCEL_6G_SEN;
        BSP_LOG_EVENT_4(EV_MODULES_BMI088_BMI088_C_02, BSP_LOG_FIELD_U32((unsigned)attempt), BSP_LOG_FIELD_U32((unsigned)timed_out), BSP_LOG_FIELD_I32((long)(_bmi088->gNorm * 1000.0f)), BSP_LOG_FIELD_U32((unsigned long)cali_diag_elapsed_ms));
    }

    // 离线标定: 上层显式要求(离线调试)时使用编译期默认值
    if (_bmi088->cali_mode == BMI088_LOAD_PRE_CALI_MODE) // 直接使用离线数据
    {
        _bmi088->gyro_offset[0] = BMI088_PRE_CALI_GYRO_X_OFFSET;
        _bmi088->gyro_offset[1] = BMI088_PRE_CALI_GYRO_Y_OFFSET;
        _bmi088->gyro_offset[2] = BMI088_PRE_CALI_GYRO_Z_OFFSET;
        _bmi088->gNorm = BMI088_PRE_CALI_G_NORM;
        _bmi088->BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
        _bmi088->acc_coef = BMI088_ACCEL_6G_SEN;
    }
    cali_diag_finish++;
    return online_ok;
}

   /* 启动时先尝试读取已保存的标定值；缺少可用记录时再在线标定或使用默认值。
   运行中由 LCD 左键或右键长按申请，INS 任务负责执行，避免两个任务同时读 SPI。 */

volatile uint8_t bmi088_calib_source = BMI088_CALIB_SRC_NONE; /* 标定来源 */
volatile float bmi088_calib_temp = 0.0f;                      /* 标定时 IMU 温度 */
volatile uint8_t bmi088_calib_attempts = 0;                   /* 最近一次标定用了几轮 */

static BMI088Instance *s_calib_instance = NULL; /* 由 BMI088Register 设置 */
static volatile uint8_t s_calib_ok = 0;         /* 当前 gyro_offset/gNorm 是否可信 */
static volatile uint8_t s_recalib_state = BMI088_RECALIB_IDLE;
static volatile uint8_t s_recalib_request = 0;

/**
 * @brief 把一组标定值写进实例(顺带把 acc_coef / 陀螺灵敏度一起算好, 防止漏设)
 */
static void BMI088ApplyCalib(BMI088Instance *b, const float *off, float gNorm)
{
    for (uint8_t i = 0; i < 3; ++i)
        b->gyro_offset[i] = off[i];
    b->gNorm = gNorm;
    b->BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
    b->acc_coef = BMI088_ACCEL_6G_SEN * (9.805f / gNorm);
}

static uint8_t BMI088CalibrationOperationBegin(void)
{
#if MC02_FEATURE_HEATER
    if (!IMUHeaterBeginLongOperation(IMU_HEATER_LONG_OPERATION_CALIBRATION))
        return 0u;
#endif
    if (!TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_CALIBRATION))
    {
#if MC02_FEATURE_HEATER
        IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_CALIBRATION, 0u);
#endif
        return 0u;
    }
    return 1u;
}

static uint8_t BMI088CalibrationOperationEnd(uint8_t success)
{
    uint8_t within_limit = TaskMonitorEndLongOperation(TASK_MONITOR_INS,
                                                       TASK_MONITOR_LONG_OPERATION_CALIBRATION);
#if MC02_FEATURE_HEATER
    IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_CALIBRATION,
                              (uint8_t)(success && within_limit));
#endif
    return within_limit;
}

/** @brief 从参数区读标定值并应用; 返回 1=成功 */
static uint8_t BMI088CalibLoad(BMI088Instance *b)
{
    ParamImuCalibrationGroup_t calibration;
    if (ParamImuCalibrationRead(&calibration, NULL) != PARAM_RESULT_OK)
        return 0;
    BMI088ApplyCalib(b, calibration.gyro_offset_rad_s, calibration.g_norm_m_s2);

    // 元数据只供界面显示；旧记录缺少或含有无效温度时仍可使用有效标定值。
    if (calibration.has_meta != 0u)
        bmi088_calib_temp = calibration.meta.temperature;
    return 1;
}

/** @brief 把当前标定值写进参数区并提交(擦写期间看门狗被参数模块临时放长); 返回 1=成功 */
static uint8_t BMI088CalibSave(BMI088Instance *b)
{
    ParamImuCalibrationGroup_t calibration = {
        .gyro_offset_rad_s = {b->gyro_offset[0], b->gyro_offset[1], b->gyro_offset[2]},
        .g_norm_m_s2 = b->gNorm,
        .meta = {
            .result = (uint32_t)PARAM_IMU_CALIB_OK,
            .temperature = b->temperature,
            .time_s = (uint32_t)(HAL_GetTick() / 1000u),
        },
        .has_meta = 1u,
    };
    ParamImuCalibrationTransaction_t transaction;
    if (ParamImuCalibrationStage(&calibration, &transaction) != PARAM_RESULT_OK ||
        ParamImuCalibrationAccept(&transaction) != PARAM_RESULT_OK)
    {
        BSP_LOG_EVENT_0(EV_MODULES_BMI088_BMI088_C_04);
        return 0;
    }
    return ParamCommit();
}

/**
 * @brief 启动期决定这次用哪套标定值: Flash 记录 → 首次自动标定 → 编译期默认值
 */
static void BMI088CalibInit(BMI088Instance *b, BMI088_Calibrate_Mode_e cfg_mode)
{
    const float default_off[3] = {BMI088_PRE_CALI_GYRO_X_OFFSET,
                                  BMI088_PRE_CALI_GYRO_Y_OFFSET,
                                  BMI088_PRE_CALI_GYRO_Z_OFFSET};

    ParamInit(); /* 幂等: 已经扫过就直接返回, 不会覆盖 RAM 缓存 */

    if (cfg_mode == BMI088_LOAD_PRE_CALI_MODE)
    { /* 上层显式要求离线标定(纯硬件调试), 尊重配置, 不去读 Flash */
        BMI088ApplyCalib(b, default_off, BMI088_PRE_CALI_G_NORM);
        bmi088_calib_source = BMI088_CALIB_SRC_DEFAULT;
        s_calib_ok = 0;
        BSP_LOG_EVENT_0(EV_MODULES_BMI088_BMI088_C_05);
        return;
    }

    if (BMI088CalibLoad(b))
    {
        bmi088_calib_source = BMI088_CALIB_SRC_FLASH;
        s_calib_ok = 1;
        RobotSafetySetCalibValid(1);
        BSP_LOG_EVENT_4(EV_MODULES_BMI088_BMI088_C_06, BSP_LOG_FIELD_I32((long)(b->gNorm * 1000.0f)), BSP_LOG_FIELD_I32((long)(b->gyro_offset[0] * 10000.0f)), BSP_LOG_FIELD_I32((long)(b->gyro_offset[1] * 10000.0f)), BSP_LOG_FIELD_I32((long)(b->gyro_offset[2] * 10000.0f)));
        return;
    }

    /* 没有有效记录: 首次自动标定(最多 3 轮), 成功即写参数区 */
    BSP_LOG_EVENT_0(EV_MODULES_BMI088_BMI088_C_07);
    if (!BMI088CalibrationOperationBegin())
    {
        BMI088ApplyCalib(b, default_off, BMI088_PRE_CALI_G_NORM);
        bmi088_calib_source = BMI088_CALIB_SRC_DEFAULT;
        s_calib_ok = 0u;
        RobotSafetySetCalibValid(0u);
        BSP_LOG_EVENT_0(EV_MODULES_BMI088_BMI088_C_08);
        return;
    }

    BMI088_Calibrate_Mode_e saved_mode = b->cali_mode;
    b->cali_mode = BMI088_CALIBRATE_ONLINE_MODE;
    bmi088_calib_attempts = 0;
    uint8_t ok = BMI088CalibrateIMU(b);
    b->cali_mode = saved_mode;

    if (ok)
    {
        bmi088_calib_source = BMI088_CALIB_SRC_FIRST_AUTO;
        s_calib_ok = 1;
        RobotSafetySetCalibValid(1);
        if (BMI088CalibSave(b))
            BSP_LOG_EVENT_1(EV_MODULES_BMI088_BMI088_C_09, BSP_LOG_FIELD_U32((unsigned)bmi088_calib_attempts));
        else
            BSP_LOG_EVENT_0(EV_MODULES_BMI088_BMI088_C_10);
    }
    else
    {
        BMI088ApplyCalib(b, default_off, BMI088_PRE_CALI_G_NORM);
        bmi088_calib_source = BMI088_CALIB_SRC_DEFAULT;
        s_calib_ok = 0;
        RobotSafetySetCalibValid(0); /* → ROBOT_FAULT_CALIB_INVALID, 不阻塞启动 */
        BSP_LOG_EVENT_1(EV_MODULES_BMI088_BMI088_C_11, BSP_LOG_FIELD_U32((unsigned)bmi088_calib_attempts));
    }
    if (!BMI088CalibrationOperationEnd(ok))
    {
        BMI088ApplyCalib(b, default_off, BMI088_PRE_CALI_G_NORM);
        bmi088_calib_source = BMI088_CALIB_SRC_DEFAULT;
        s_calib_ok = 0u;
        RobotSafetySetCalibValid(0u);
        BSP_LOG_EVENT_0(EV_MODULES_BMI088_BMI088_C_12);
    }
}

uint8_t BMI088CalibRequest(void)
{
    if (s_calib_instance == NULL)
        return 0;
    // 未完成的请求只保留一份，避免重复启动长时间标定。
    if (s_recalib_state == BMI088_RECALIB_BUSY)
        return 0; /* 已经在标定中, 不重复受理 */

    s_recalib_state = BMI088_RECALIB_BUSY;
    s_recalib_request = 1;
    return 1;
}

uint8_t BMI088CalibService(void)
{
    // 请求只由 INS 任务取走并处理，保证 BMI088 的 SPI 不与采样同时使用。
    if (!s_recalib_request)
        return 0;
    s_recalib_request = 0;

    BMI088Instance *b = s_calib_instance;
    s_recalib_state = BMI088_RECALIB_BUSY;
    if (b == NULL)
    {
        s_recalib_state = BMI088_RECALIB_FAIL;
        return 1;
    }

    BSP_LOG_EVENT_0(EV_MODULES_BMI088_BMI088_C_13);
    if (!BMI088CalibrationOperationBegin())
    {
        s_calib_ok = 0u;
        RobotSafetySetCalibValid(0u);
        s_recalib_state = BMI088_RECALIB_FAIL;
        return 1u;
    }

    BMI088_Calibrate_Mode_e saved_mode = b->cali_mode;
    b->cali_mode = BMI088_CALIBRATE_ONLINE_MODE;
    bmi088_calib_attempts = 0;
    uint8_t ok = BMI088CalibrateIMU(b);
    b->cali_mode = saved_mode;

    if (ok)
    {
        s_calib_ok = 1;
        RobotSafetySetCalibValid(1);
        bmi088_calib_source = BMI088_CALIB_SRC_MANUAL;
        if (BMI088CalibSave(b))
            BSP_LOG_EVENT_1(EV_MODULES_BMI088_BMI088_C_14, BSP_LOG_FIELD_U32((unsigned)bmi088_calib_attempts));
        else
            BSP_LOG_EVENT_0(EV_MODULES_BMI088_BMI088_C_15);
    }
    else
    {
        /* 保留旧值/默认值, 并上报 CALIB_INVALID 阻止进入 READY */
        s_calib_ok = 0;
        RobotSafetySetCalibValid(0);
        BSP_LOG_EVENT_1(EV_MODULES_BMI088_BMI088_C_16, BSP_LOG_FIELD_U32((unsigned)bmi088_calib_attempts));
    }

    if (!BMI088CalibrationOperationEnd(ok))
    {
        ok = 0u;
        s_calib_ok = 0u;
        RobotSafetySetCalibValid(0u);
    }

    s_recalib_state = ok ? BMI088_RECALIB_OK : BMI088_RECALIB_FAIL;
    return 1;
}

uint8_t BMI088CalibGetState(void)
{
    return s_recalib_state;
}

void BMI088CalibAckResult(void)
{
    if (s_recalib_state == BMI088_RECALIB_OK || s_recalib_state == BMI088_RECALIB_FAIL)
        s_recalib_state = BMI088_RECALIB_IDLE;
}

uint8_t BMI088CalibIsValid(void)
{
    return s_calib_ok;
}

BMI088Instance *BMI088Register(BMI088_Init_Config_s *config)
{
    bmi088_init_error = BMI088_NO_ERROR;
    bmi088_init_retry_count = 0u;
    bmi088_init_stage = BMI088_INIT_STAGE_IDLE;
    bmi088_init_failure_stage = BMI088_INIT_STAGE_IDLE;
    bmi088_init_acc_status = HAL_OK;
    bmi088_init_gyro_status = HAL_OK;
    bmi088_init_acc_chip_id = 0u;
    bmi088_init_gyro_chip_id = 0u;

    if (config == NULL || config->work_mode != BMI088_BLOCK_PERIODIC_MODE)
    {
        bmi088_init_error = BMI088_NO_SENSOR;
        BSP_LOG_EVENT_0(EV_MODULES_BMI088_BMI088_C_17);
        return NULL;
    }

    // 在本地副本中填写内部 id 和 SPI 模式，保留调用者传入的配置。
    BMI088_Init_Config_s local_config = *config;
    config = &local_config;

    // 申请内存
    BMI088Instance *bmi088_instance = (BMI088Instance *)calloc(1u, sizeof(BMI088Instance));
    if (bmi088_instance == NULL)
    {
        bmi088_init_error = BMI088_NO_SENSOR;
        return NULL;
    }
    // 旧温控 PID 先于外设注册检查；上层独立温控不需要初始化该零配置实例。
    if (config->heat_pwm_config.htim != NULL &&
        PIDInit(&bmi088_instance->heat_pid, &config->heat_pid_config) != PID_OK)
    {
        bmi088_init_error = BMI088_RESOURCE_ERROR;
        free(bmi088_instance);
        return NULL;
    }
    // 从右向左赋值,让bsp instance保存指向bmi088_instance的指针(父指针),便于在底层中断中访问bmi088_instance
    config->acc_int_config.id =
        config->gyro_int_config.id =
            config->spi_acc_config.id =
                config->spi_gyro_config.id =
                    config->heat_pwm_config.id = bmi088_instance;
    // @todo:
    // 目前只实现了!!!阻塞读取模式!!!.如果需要使用IT模式,则需要修改这里的代码,为spi和gpio注册callback(默认为NULL)
    // 还需要设置SPI的传输模式为DMA模式或IT模式(默认为blocking)
    // 可以通过conditional compilation或者runtime参数判断
    // code to go here ...

    // INT_ACC EXTI CALLBACK: 检查是否有传输正在进行,如果没有则开启SPI DMA传输,有则置位wait标志位;
    // 第一次是加速度计,第二次是温度.

    // INT_GYRO EXTI CALLBACK: 开启SPI DMA传输,不会出现等待传输的情况
    // SPI_GYRO DMA CALLBACK: 解算陀螺仪数据,
    // SPI_ACC DMA CALLBACK: 解算加速度计数据,清除温度wait标志位并启动温度传输,第二次进入中断时解算温度数据

    // 还有其他方案可用,比如阻塞等待传输完成,但是比较笨.
        config->spi_acc_config.spi_work_mode = SPI_BLOCK_MODE;
        config->spi_gyro_config.spi_work_mode = SPI_BLOCK_MODE;
    // 根据参数选择工作模式
    bmi088_instance->spi_acc = SPIRegister(&config->spi_acc_config);
    bmi088_instance->spi_gyro = SPIRegister(&config->spi_gyro_config);
    if (bmi088_instance->spi_acc == NULL || bmi088_instance->spi_gyro == NULL)
    {
        bmi088_init_error = BMI088_RESOURCE_ERROR;
        (void)SPIUnregister(bmi088_instance->spi_acc);
        (void)SPIUnregister(bmi088_instance->spi_gyro);
        free(bmi088_instance);
        return NULL;
    }
    if (config->heat_pwm_config.htim != NULL) // 温控 PWM 可选: 未配置 htim 时不注册, 由上层自行控温
    {
        bmi088_instance->heat_pwm = PWMRegister(&config->heat_pwm_config);
        if (bmi088_instance->heat_pwm == NULL)
        {
            bmi088_init_error = BMI088_RESOURCE_ERROR;
            (void)SPIUnregister(bmi088_instance->spi_acc);
            (void)SPIUnregister(bmi088_instance->spi_gyro);
            free(bmi088_instance);
            return NULL;
        }
    }

    // 两个传感器都要完成初始化；遇到故障最多重试5次，随后释放已申请资源。
    BMI088_ERORR_CODE_e error = BMI088_NO_ERROR;
    uint8_t init_retry = 0;
    do
    {
        error = BMI088_NO_ERROR;
        bmi088_init_failure_stage = BMI088_INIT_STAGE_IDLE;
        error |= BMI088AccelInit(bmi088_instance);
        error |= BMI088GyroInit(bmi088_instance);
        init_retry++;
        bmi088_init_retry_count = init_retry;
        bmi088_init_error = (uint32_t)error;
    } while (error != 0 && init_retry < BMI088_INIT_MAX_RETRY);

    if (error != 0)
    {
        BSP_LOG_EVENT_6(EV_MODULES_BMI088_BMI088_C_18, BSP_LOG_FIELD_U32((unsigned)init_retry), BSP_LOG_FIELD_HEX32((unsigned)error), BSP_LOG_FIELD_HEX32((unsigned)bmi088_init_acc_chip_id), BSP_LOG_FIELD_U32((unsigned)bmi088_init_acc_status), BSP_LOG_FIELD_HEX32((unsigned)bmi088_init_gyro_chip_id), BSP_LOG_FIELD_U32((unsigned)bmi088_init_gyro_status));
        (void)PWMUnregister(bmi088_instance->heat_pwm);
        (void)SPIUnregister(bmi088_instance->spi_acc);
        (void)SPIUnregister(bmi088_instance->spi_gyro);
        free(bmi088_instance);
        return NULL;
    }

    bmi088_init_stage = BMI088_INIT_STAGE_READY;

    bmi088_instance->work_mode = BMI088_BLOCK_PERIODIC_MODE; // 临时设置为阻塞模式
    s_calib_instance = bmi088_instance;                      // 供按需标定使用
    // 优先使用已保存的标定值；没有可用记录时在线标定，失败则保留默认值并报告无效。
    BMI088CalibInit(bmi088_instance, config->cali_mode);
    bmi088_instance->work_mode = config->work_mode; // 恢复工作模式
    if (config->work_mode == BMI088_BLOCK_TRIGGER_MODE)
    {
        bmi088_instance->spi_acc->spi_work_mode = SPI_DMA_MODE;
        bmi088_instance->spi_gyro->spi_work_mode = SPI_DMA_MODE;
        
        // 设置回调函数
        bmi088_instance->spi_acc->callback = BMI088AccSPIFinishCallback;
        bmi088_instance->spi_gyro->callback = BMI088GyroSPIFinishCallback;
        
        bmi088_instance->acc_int = GPIORegister(&config->acc_int_config); // 只有在非阻塞模式下才需要注册中断
        bmi088_instance->gyro_int = GPIORegister(&config->gyro_int_config);

        bmi088_instance->acc_int->gpio_model_callback = BMI088AccINTCallback;
        bmi088_instance->gyro_int->gpio_model_callback = BMI088GyroINTCallback;
    } // 注册实例
    return bmi088_instance;
}
