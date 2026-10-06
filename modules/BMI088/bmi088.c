#include "bmi088_regNdef.h"
#include "bmi088.h"
#include "bmi088_temperature.h"
#include "user_lib.h"
#include "daemon.h"
#include "bsp_log.h"
#include "bsp_param.h"
#include "robot_safety.h"
#include "task_monitor.h"
#include "imu_heater.h"
#include "cmsis_os.h"
#include <math.h>

static DaemonInstance *bmi088_daemon_instance;

/* ---------------- 在线标定诊断量(2026-09-22, 排查 -O2 启动卡死) ----------------
   标定循环"卡住"时, 光看 PC 只能知道卡在哪, 看不出"为什么出不来"。这几个量把循环
   内部状态暴露出来: 外层重试了几次、内层跑到第几轮、采集成功/失败、加速度模长、
   超时分支是否命中。变量是 volatile, 方便 Live Watch / OpenOCD 直接读。 */
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
static void BMI088AccelWriteSingleReg(BMI088Instance *bmi088, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    SPITransmit(bmi088->spi_acc, tx, 2);
}

/**
 * @brief 写gyro寄存器.形式上的封装
 * @attention 只会向目标reg写入一个字节,因为只有1个字节所以直接传值(指针是32位反而浪费)
 *
 * @param bmi088 待写入的BMI088实例
 * @param reg  待写入的寄存器地址
 * @param data 待写入的数据(注意不是指针)
 */
static void BMI088GyroWriteSingleReg(BMI088Instance *bmi088, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    SPITransmit(bmi088->spi_gyro, tx, 2);
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
    BMI088AccelRead(bmi088, BMI088_ACC_CHIP_ID, &whoami_check, 1);
    DWT_Delay(0.001);

    BMI088AccelWriteSingleReg(bmi088, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE); // 软复位
    DWT_Delay(BMI088_COM_WAIT_SENSOR_TIME / 1000);

    // 检查ID,如果不是0x1E(bmi088 whoami寄存器值),则返回错误
    BMI088AccelRead(bmi088, BMI088_ACC_CHIP_ID, &whoami_check, 1);
    if (whoami_check != BMI088_ACC_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;
    DWT_Delay(0.001);
    // 初始化寄存器,提高可读性
    uint8_t reg = 0, data = 0;
    BMI088_ERORR_CODE_e error = 0;
    // 使用sizeof而不是magic number,这样如果修改了数组大小,不用修改这里的代码;或者使用宏定义
    for (uint8_t i = 0; i < sizeof(BMI088_Accel_Init_Table) / sizeof(BMI088_Accel_Init_Table[0]); i++)
    {
        reg = BMI088_Accel_Init_Table[i][BMI088REG];
        data = BMI088_Accel_Init_Table[i][BMI088DATA];
        BMI088AccelWriteSingleReg(bmi088, reg, data); // 写入寄存器
        DWT_Delay(0.01);
        BMI088AccelRead(bmi088, reg, &data, 1); // 写完之后立刻读回检查
        DWT_Delay(0.01);
        if (data != BMI088_Accel_Init_Table[i][BMI088DATA])
            error |= BMI088_Accel_Init_Table[i][BMI088ERROR];
        //{i--;} 可以设置retry次数,如果retry次数用完了,则返回error
    }
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
    BMI088GyroWriteSingleReg(bmi088, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE); // 软复位
    DWT_Delay(0.08);

    // 检查ID,如果不是0x0F(bmi088 whoami寄存器值),则返回错误
    uint8_t whoami_check = 0;
    BMI088GyroRead(bmi088, BMI088_GYRO_CHIP_ID, &whoami_check, 1);
    if (whoami_check != BMI088_GYRO_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;
    DWT_Delay(0.001);

    // 初始化寄存器,提高可读性
    uint8_t reg = 0, data = 0;
    BMI088_ERORR_CODE_e error = 0;
    // 使用sizeof而不是magic number,这样如果修改了数组大小,不用修改这里的代码;或者使用宏定义
    for (uint8_t i = 0; i < sizeof(BMI088_Gyro_Init_Table) / sizeof(BMI088_Gyro_Init_Table[0]); i++)
    {
        reg = BMI088_Gyro_Init_Table[i][BMI088REG];
        data = BMI088_Gyro_Init_Table[i][BMI088DATA];
        BMI088GyroWriteSingleReg(bmi088, reg, data); // 写入寄存器
        DWT_Delay(0.001);
        BMI088GyroRead(bmi088, reg, &data, 1); // 写完之后立刻读回对应寄存器检查是否写入成功
        DWT_Delay(0.001);
        if (data != BMI088_Gyro_Init_Table[i][BMI088DATA])
            error |= BMI088_Gyro_Init_Table[i][BMI088ERROR];
        //{i--;} 可以设置retry次数,尝试重新写入.如果retry次数用完了,则返回error
    }

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
    // 如果使用异步姿态更新,此处唤醒量测更新的任务
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

/* Reject non-finite values before they cross the published sample boundary. */
static uint8_t BMI088SampleValuesAreFinite(const BMI088_Data_t *sample)
{
    if (sample == NULL || !isfinite(sample->temperature))
        return 0u;
    for (uint8_t axis = 0; axis < 3u; ++axis)
    {
        if (!isfinite(sample->acc[axis]) || !isfinite(sample->gyro[axis]))
            return 0u;
    }
    return 1u;
}

/**
 * @brief Read and publish one complete, finite BMI088 sample.
 * @param bmi088 BMI088 sensor instance.
 * @param data_store Receives the sample only when acquisition succeeds.
 * @return BMI088_AcquireStatus_e Acquisition status; output remains unchanged on failure.
 */
BMI088_AcquireStatus_e BMI088Acquire(BMI088Instance *bmi088, BMI088_Data_t *data_store)
{
    if (bmi088 == NULL || data_store == NULL)
        return BMI088_ACQUIRE_NO_DATA;

    BMI088_Data_t staged = {0};
    uint8_t raw[6] = {0};
    HAL_StatusTypeDef spi_status;

    if (bmi088->work_mode == BMI088_BLOCK_PERIODIC_MODE)
    {
        spi_status = BMI088AccelRead(bmi088, BMI088_ACCEL_XOUT_L, raw, 6u);
        if (spi_status != HAL_OK)
            goto spi_failure;
        for (uint8_t i = 0; i < 3u; ++i)
            staged.acc[i] = bmi088->acc_coef * (float)(int16_t)(((uint16_t)raw[2u * i + 1u] << 8u) | raw[2u * i]);

        spi_status = BMI088GyroRead(bmi088, BMI088_GYRO_X_L, raw, 6u);
        if (spi_status != HAL_OK)
            goto spi_failure;
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
#define BMI088_PRE_CALI_ACC_X_OFFSET 0.0f
#define BMI088_PRE_CALI_ACC_Y_OFFSET 0.0f
#define BMI088_PRE_CALI_ACC_Z_OFFSET 0.0f
#define BMI088_PRE_CALI_G_NORM 9.805f

/* 在线标定最多重试轮数(每轮 6000 次采样, 约 3.6s)。
   原来是 12s 超时前无限重试; -O2 下超时判据失效就变成无限重标定 → 启动卡死。 */
#define BMI088_CALI_MAX_ATTEMPTS 3u
/* 标定循环里每隔多少次采样让出一次 CPU(给 daemon 喂狗、给 LCD 显示"标定中") */
#define BMI088_CALI_YIELD_INTERVAL 512u
#define BMI088_CALI_YIELD_MS 10u
/**
 * @brief BMI088 acc gyro 标定
 * @note 标定后的数据存储在bmi088->bias和gNorm中,用于后续数据消噪和单位转换归一化
 * @attention 标定使用阻塞采样路径，必须由 INS 任务独占 BMI088 SPI；系统中断保持开启以支持 HAL 超时和 RTOS 调度。
 * @attention 标定精度和等待时间有关,目前使用线性回归.后续考虑引入非线性回归
 * @todo 将标定次数(等待时间)变为参数供设定
 * @section 整体流程为1.累加加速度数据计算gNrom() 2.累加陀螺仪数据计算零飘
 *          3. 如果标定过程运动幅度过大,重新标定  4.保存标定参数
 *
 * @param _bmi088 待标定的BMI088实例
 */
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
        // 循环继续的条件为标定环境不满足
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
                /* 标定在 INS 任务中运行，只临时豁免 INS 的普通任务期限；其他任务仍须
                   正常运行，TaskMonitor 才会继续喂狗。周期性让出 CPU 让 daemon/LCD
                   获得调度；标定没有直接喂狗路径，整体期限由 15s 监控窗口限制。 */
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
                valid_samples++;
                uint32_t acq_us = DWT_ProbeElapsedUs(diag_acq);
                if (acq_us > cali_diag_acq_us_max)
                    cali_diag_acq_us_max = acq_us;
                cali_diag_acc_norm = NormOf3d(raw_data.acc);
                cali_diag_gyro_x = raw_data.gyro[0];
                gNormTemp = NormOf3d(raw_data.acc);
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

        /* 判据与循环条件完全一致: 循环是"因为判据满足才退出"才算成功 */
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
        /* 只有判据通过才允许算 acc_coef: 失败时 gNorm 可能是垃圾值(甚至 0),
           原来无条件 `*= 9.805/gNorm` 会算出 inf/NaN 并污染整条加速度链路 */
        _bmi088->acc_coef = BMI088_ACCEL_6G_SEN * (9.805f / _bmi088->gNorm);
        bmi088_calib_temp = _bmi088->temperature;
        /* 注意: RTT 的 printf 实现不支持 %f, 而且遇到 %f 时**不会消费参数**,
           会让后面的 %s 取到错位的参数(见 LOG_PROTO 的结尾 %s) → 把浮点位模式
           当指针解引用 → BusFault/HardFault。所以这里统一用"放大成整数"打印。 */
        LOGINFO("[bmi088] online calib OK: round=%u gNorm_x1000=%ld off_x10000=%ld/%ld/%ld temp_x100=%ld",
                (unsigned)attempt,
                (long)(_bmi088->gNorm * 1000.0f),
                (long)(_bmi088->gyro_offset[0] * 10000.0f),
                (long)(_bmi088->gyro_offset[1] * 10000.0f),
                (long)(_bmi088->gyro_offset[2] * 10000.0f),
                (long)(bmi088_calib_temp * 100.0f));
    }
    else
    {
        _bmi088->acc_coef = BMI088_ACCEL_6G_SEN;
        LOGERROR("[bmi088] online calib FAILED: round=%u timed_out=%u gNorm_x1000=%ld elapsed=%lums",
                 (unsigned)attempt, (unsigned)timed_out,
                 (long)(_bmi088->gNorm * 1000.0f),
                 (unsigned long)cali_diag_elapsed_ms);
    }

    // 离线标定: 上层显式要求(离线调试)时使用编译期默认值
    if (_bmi088->cali_mode == BMI088_LOAD_PRE_CALI_MODE) // 直接使用离线数据
    {
        _bmi088->gyro_offset[0] = BMI088_PRE_CALI_ACC_X_OFFSET;
        _bmi088->gyro_offset[1] = BMI088_PRE_CALI_ACC_Y_OFFSET;
        _bmi088->gyro_offset[2] = BMI088_PRE_CALI_ACC_Z_OFFSET;
        _bmi088->gNorm = BMI088_PRE_CALI_G_NORM;
        _bmi088->BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
        _bmi088->acc_coef = BMI088_ACCEL_6G_SEN;
    }
    cali_diag_finish++;
    return online_ok;
}

/* ============================================================================
   标定结果持久化 + 按需标定 (2026-09)
   ----------------------------------------------------------------------------
   背景: 原来 BMI088Register 无条件做 6000 次采样在线标定, 启动要 3~15s;
         -O2 下更会因为 DWT 时间轴失效导致 12s 超时永不触发 → 无限重标定 → 启动卡死。
   现在:
     1) 启动先查内部 Flash 参数区(bsp/param) 有没有有效记录 → 有就直接用(启动 <1s),
        并 RobotSafetySetCalibValid(1);
     2) 没有记录 → 首次自动标定一次(最多 BMI088_CALI_MAX_ATTEMPTS 轮), 成功即写参数区;
        失败用编译期默认值 + CALIB_INVALID 报警, 但**不阻塞启动**;
     3) 运行期由 LCD 长按中键 3s 触发按需标定(BMI088CalibRequest),
        由 INS 任务在自身上下文里执行(BMI088CalibService), 避免和 INS_Task 抢 SPI。
   ========================================================================== */

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
    if (!IMUHeaterBeginLongOperation(IMU_HEATER_LONG_OPERATION_CALIBRATION))
        return 0u;
    if (!TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_CALIBRATION))
    {
        IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_CALIBRATION, 0u);
        return 0u;
    }
    return 1u;
}

static uint8_t BMI088CalibrationOperationEnd(uint8_t success)
{
    uint8_t within_limit = TaskMonitorEndLongOperation(TASK_MONITOR_INS,
                                                       TASK_MONITOR_LONG_OPERATION_CALIBRATION);
    IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_CALIBRATION,
                              (uint8_t)(success && within_limit));
    return within_limit;
}

/**
 * @brief 标定值合理性检查
 * @note  CRC32 只能证明"写进去的字节没坏", 证不了"数值合理"
 *        (比如参数区被别的固件写过), 所以这里再卡一次范围
 */
static uint8_t BMI088CalibSanity(const float *off, float gNorm)
{
    if (!isfinite(off[0]) || !isfinite(off[1]) || !isfinite(off[2]) || !isfinite(gNorm))
        return 0;
    if (fabsf(off[0]) > 0.1f || fabsf(off[1]) > 0.1f || fabsf(off[2]) > 0.1f)
        return 0;
    if (gNorm < 9.0f || gNorm > 10.5f)
        return 0;
    return 1;
}

/** @brief 从参数区读标定值并应用; 返回 1=成功 */
static uint8_t BMI088CalibLoad(BMI088Instance *b)
{
    float off[3];
    float gNorm = 0.0f;

    if (!ParamGetFloats(PARAM_KEY_IMU_GYRO_OFFSET, off, 3))
        return 0;
    if (!ParamGetFloat(PARAM_KEY_IMU_G_NORM, &gNorm))
        return 0;
    if (!BMI088CalibSanity(off, gNorm))
    {
        LOGERROR("[bmi088] Flash 里的标定值超出合理范围, 视为无效");
        return 0;
    }
    BMI088ApplyCalib(b, off, gNorm);

    /* 顺带把"标定时的温度"读出来给 LCD 显示(meta 缺失也不影响标定值的使用) */
    ParamImuCalibMeta_t meta;
    if (ParamGet(PARAM_KEY_IMU_CALIB_META, &meta, sizeof(meta)) &&
        meta.result == (uint32_t)PARAM_IMU_CALIB_OK &&
        isfinite(meta.temperature) && meta.temperature > -40.0f && meta.temperature < 100.0f)
        bmi088_calib_temp = meta.temperature;
    return 1;
}

/** @brief 把当前标定值写进参数区并提交(擦写期间看门狗被参数模块临时放长); 返回 1=成功 */
static uint8_t BMI088CalibSave(BMI088Instance *b)
{
    ParamImuCalibMeta_t meta;
    float off[3] = {b->gyro_offset[0], b->gyro_offset[1], b->gyro_offset[2]};

    meta.result = (uint32_t)PARAM_IMU_CALIB_OK;
    meta.temperature = b->temperature;
    meta.time_s = (uint32_t)(HAL_GetTick() / 1000u);

    if (!ParamSetFloats(PARAM_KEY_IMU_GYRO_OFFSET, off, 3) ||
        !ParamSetFloat(PARAM_KEY_IMU_G_NORM, b->gNorm) ||
        !ParamSet(PARAM_KEY_IMU_CALIB_META, &meta, sizeof(meta)))
    {
        LOGERROR("[bmi088] 标定值写入参数缓存失败");
        return 0;
    }
    return ParamCommit();
}

/**
 * @brief 启动期决定这次用哪套标定值: Flash 记录 → 首次自动标定 → 编译期默认值
 */
static void BMI088CalibInit(BMI088Instance *b, BMI088_Calibrate_Mode_e cfg_mode)
{
    const float default_off[3] = {BMI088_PRE_CALI_ACC_X_OFFSET,
                                  BMI088_PRE_CALI_ACC_Y_OFFSET,
                                  BMI088_PRE_CALI_ACC_Z_OFFSET};

    ParamInit(); /* 幂等: 已经扫过就直接返回, 不会覆盖 RAM 缓存 */

    if (cfg_mode == BMI088_LOAD_PRE_CALI_MODE)
    { /* 上层显式要求离线标定(纯硬件调试), 尊重配置, 不去读 Flash */
        BMI088ApplyCalib(b, default_off, BMI088_PRE_CALI_G_NORM);
        bmi088_calib_source = BMI088_CALIB_SRC_DEFAULT;
        s_calib_ok = 0;
        LOGWARNING("[bmi088] cali_mode=LOAD_PRE_CALI: 使用编译期默认标定值");
        return;
    }

    if (BMI088CalibLoad(b))
    {
        bmi088_calib_source = BMI088_CALIB_SRC_FLASH;
        s_calib_ok = 1;
        RobotSafetySetCalibValid(1);
        LOGINFO("[bmi088] 使用 Flash 标定记录: gNorm_x1000=%ld off_x10000=%ld/%ld/%ld",
                (long)(b->gNorm * 1000.0f), (long)(b->gyro_offset[0] * 10000.0f),
                (long)(b->gyro_offset[1] * 10000.0f), (long)(b->gyro_offset[2] * 10000.0f));
        return;
    }

    /* 没有有效记录: 首次自动标定(最多 3 轮), 成功即写参数区 */
    LOGWARNING("[bmi088] 无 Flash 标定记录, 首次自动标定(请保持静止)...");
    if (!BMI088CalibrationOperationBegin())
    {
        BMI088ApplyCalib(b, default_off, BMI088_PRE_CALI_G_NORM);
        bmi088_calib_source = BMI088_CALIB_SRC_DEFAULT;
        s_calib_ok = 0u;
        RobotSafetySetCalibValid(0u);
        LOGERROR("[bmi088] startup calibration refused: monitor or heater guard unavailable");
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
            LOGINFO("[bmi088] 首次自动标定成功并已保存(rounds=%u)", (unsigned)bmi088_calib_attempts);
        else
            LOGERROR("[bmi088] 首次自动标定成功但保存失败, 下次启动会重新标定");
    }
    else
    {
        BMI088ApplyCalib(b, default_off, BMI088_PRE_CALI_G_NORM);
        bmi088_calib_source = BMI088_CALIB_SRC_DEFAULT;
        s_calib_ok = 0;
        RobotSafetySetCalibValid(0); /* → ROBOT_FAULT_CALIB_INVALID, 不阻塞启动 */
        LOGERROR("[bmi088] 首次自动标定失败(rounds=%u), 使用默认值并置 CALIB_INVALID",
                 (unsigned)bmi088_calib_attempts);
    }
    if (!BMI088CalibrationOperationEnd(ok))
    {
        BMI088ApplyCalib(b, default_off, BMI088_PRE_CALI_G_NORM);
        bmi088_calib_source = BMI088_CALIB_SRC_DEFAULT;
        s_calib_ok = 0u;
        RobotSafetySetCalibValid(0u);
        LOGERROR("[bmi088] startup calibration exceeded bounded monitoring window");
    }
}

uint8_t BMI088CalibRequest(void)
{
    if (s_calib_instance == NULL)
        return 0;
    if (s_recalib_state == BMI088_RECALIB_BUSY)
        return 0; /* 已经在标定中, 不重复受理 */

    s_recalib_state = BMI088_RECALIB_BUSY;
    s_recalib_request = 1;
    return 1;
}

uint8_t BMI088CalibService(void)
{
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

    LOGWARNING("[bmi088] 按需标定开始, 请保持静止...");
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
            LOGINFO("[bmi088] 按需标定成功并已保存(rounds=%u)", (unsigned)bmi088_calib_attempts);
        else
            LOGERROR("[bmi088] 按需标定成功, 但写参数区失败(重启会退回旧值)");
    }
    else
    {
        /* 保留旧值/默认值, 并上报 CALIB_INVALID 阻止进入 READY */
        s_calib_ok = 0;
        RobotSafetySetCalibValid(0);
        LOGERROR("[bmi088] 按需标定失败(rounds=%u), 保留旧值并置 CALIB_INVALID",
                 (unsigned)bmi088_calib_attempts);
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

// 考虑阻塞模式和非阻塞模式的兼容性,通过条件编译(则需要在编译前修改宏定义)或runtime参数判断
// runtime的开销不大(一次性判断),但是需要修改函数原型,增加参数,代码长度增加(但不多)
// runtime如何修改callback?根据参数选择是否给spi传入callback,如果是阻塞模式,则不传入callback,如果是非阻塞模式,则传入callback(bsp会检查是否NULL)
// 条件编译的开销小,但是需要修改宏定义,增加编译时间,同时人力介入
// 根据实际情况选择(说了和没说一样!)

BMI088Instance *BMI088Register(BMI088_Init_Config_s *config)
{
    if (config == NULL || config->work_mode != BMI088_BLOCK_PERIODIC_MODE)
    {
        LOGERROR("[bmi088] asynchronous trigger sampling is not supported by the synchronous snapshot API");
        return NULL;
    }

    // 申请内存
    BMI088Instance *bmi088_instance = (BMI088Instance *)zmalloc(sizeof(BMI088Instance));
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
    if (config->heat_pwm_config.htim != NULL) // 温控 PWM 可选: 未配置 htim 时不注册, 由上层自行控温
    {
        bmi088_instance->heat_pwm = PWMRegister(&config->heat_pwm_config);
    }
    PIDInit(&bmi088_instance->heat_pid, &config->heat_pid_config);

    // 初始化acc和gyro; 传感器异常时不能无限重试, 避免开机死循环
    BMI088_ERORR_CODE_e error = BMI088_NO_ERROR;
    uint8_t init_retry = 0;
    do
    {
        error = BMI088_NO_ERROR;
        error |= BMI088AccelInit(bmi088_instance);
        error |= BMI088GyroInit(bmi088_instance);
        init_retry++;
    } while (error != 0 && init_retry < BMI088_INIT_MAX_RETRY);

    if (error != 0)
    {
        LOGERROR("[bmi088] init failed after %u retries, error=0x%02X", init_retry, error);
        return NULL;
    }

    bmi088_instance->work_mode = BMI088_BLOCK_PERIODIC_MODE; // 临时设置为阻塞模式
    s_calib_instance = bmi088_instance;                      // 供按需标定使用
    // 标定: Flash 有记录→直接用(启动<1s); 没有→首次自动标定并保存; 失败→默认值+报警
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
