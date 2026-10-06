#ifndef __BMI088_H__ // 防止重复包含
#define __BMI088_H__

#include "bsp_spi.h"
#include "bmi088_data.h"
#include "bsp_gpio.h"
#include "controller.h"
#include "bsp_pwm.h"
#include "stdint.h"

#define BMI088_INIT_MAX_RETRY 5u // 初始化重试上限, 传感器异常时避免开机死循环

// bmi088工作模式枚举
typedef enum
{
    BMI088_BLOCK_PERIODIC_MODE = 0, // 阻塞模式,周期性读取
    BMI088_BLOCK_TRIGGER_MODE,      // 阻塞模式,触发读取(中断)
} BMI088_Work_Mode_e;

// bmi088标定方式枚举,若使用预设标定参数,注意修改预设参数
typedef enum
{
    BMI088_CALIBRATE_ONLINE_MODE = 0, // 初始化时进行标定
    BMI088_LOAD_PRE_CALI_MODE,        // 使用预设标定参数,
} BMI088_Calibrate_Mode_e;

typedef enum
{
    BMI088_ACQUIRE_NO_DATA = 0,
    BMI088_ACQUIRE_OK = 1,
    BMI088_ACQUIRE_SPI_ERROR,
    BMI088_ACQUIRE_SPI_BUSY,
    BMI088_ACQUIRE_SPI_TIMEOUT
} BMI088_AcquireStatus_e;

/* BMI088实例结构体定义 */
typedef struct
{
    // 传输模式和工作模式控制
    BMI088_Work_Mode_e work_mode;
    BMI088_Calibrate_Mode_e cali_mode;
    // SPI接口
    SPIInstance *spi_gyro; // 注意,SPIInstnace内部也有一个GPIOInstance,用于控制片选CS
    SPIInstance *spi_acc;  // 注意,SPIInstnace内部也有一个GPIOInstance,用于控制片选CS
    // EXTI GPIO,如果BMI088工作在中断模式,则需要配置中断引脚(有数据产生时触发解算)
    GPIOInstance *gyro_int;
    GPIOInstance *acc_int;
    // 温度控制
    PIDInstance heat_pid; // 恒温PID
    PWMInstance *heat_pwm; // 加热PWM
    // IMU数据
    float gyro[3];     // 陀螺仪数据,xyz
    float acc[3];      // 加速度计数据,xyz
    float temperature; // 温度
    // 标定数据
    float gyro_offset[3]; // 陀螺仪零偏
    float gNorm;          // 重力加速度模长,从标定获取
    float acc_coef;       // 加速度计原始数据转换系数
    // 传感器灵敏度,用于计算实际值(regNdef.h中定义)
    float BMI088_ACCEL_SEN;
    float BMI088_GYRO_SEN;
    uint32_t sample_sequence;
    uint32_t acc_sample_timestamp_ms;
    uint32_t gyro_sample_timestamp_ms;
    BMI088_AcquireStatus_e last_acquire_status;
    // 用于计算两次采样的时间间隔
    uint32_t bias_dwt_cnt;
    // 数据更新标志位
    struct // 位域,节省空间提高可读性
    {
        uint8_t gyro : 1; // 1:有新数据,0:无新数据
        uint8_t acc : 1;
        uint8_t temp : 1;
        uint8_t gyro_overrun : 1; // 1:数据溢出,0:无溢出
        uint8_t acc_overrun : 1;
        uint8_t temp_overrun : 1;
        uint8_t imu_ready : 1; // 1:IMU数据准备好,0:IMU数据未准备好(gyro+acc)
        // 后续可添加其他标志位,不够用可以扩充16or32,太多可以删
    } update_flag;
} BMI088Instance;

/* BMI088初始化配置 */
typedef struct
{
    BMI088_Work_Mode_e work_mode;
    BMI088_Calibrate_Mode_e cali_mode;
    SPI_Init_Config_s spi_gyro_config;
    SPI_Init_Config_s spi_acc_config;
    GPIO_Init_Config_s gyro_int_config;
    GPIO_Init_Config_s acc_int_config;
    PID_Init_Config_s heat_pid_config;
    PWM_Init_Config_s heat_pwm_config;
} BMI088_Init_Config_s;

/**
 * @brief 初始化BMI088,返回BMI088实例指针
 * @note  一般一个开发板只有一个BMI088,所以这里就叫BMI088Init而不是Register
 *
 * @param config bmi088初始化配置
 * @return BMI088Instance* 实例指针
 */
BMI088Instance *BMI088Register(BMI088_Init_Config_s *config);

/**
 * @brief 读取BMI088数据
 * @param bmi088 BMI088实例指针
 * @return BMI088_Data_t 读取到的数据
 */
BMI088_AcquireStatus_e BMI088Acquire(BMI088Instance *bmi088, BMI088_Data_t *data_store);

/**
 * @brief 标定传感器.BMI088在初始化的时候会调用此函数. 提供接口方便标定离线数据
 * @attention 本函数只做"在线标定"这一件事, 不再自己决定失败后用哪套参数
 *            (Flash 记录 / 编译期默认值由 BMI088CalibInit 与 BMI088CalibService 决定)。
 *            运行期请通过 BMI088CalibRequest + BMI088CalibService 触发, 不要直接调用。
 *
 * @param _bmi088 待标定的实例
 * @return 1=在线标定成功(已通过严格判据); 0=超时/判据不满足, 结果不可信
 */
uint8_t BMI088CalibrateIMU(BMI088Instance *_bmi088);

/* ---------------- 标定来源(供 LCD / RTT / OpenOCD 观察) ---------------- */
typedef enum
{
    BMI088_CALIB_SRC_NONE = 0,   /* 尚未确定 */
    BMI088_CALIB_SRC_FLASH,      /* 直接用内部 Flash 参数区里的记录 */
    BMI088_CALIB_SRC_FIRST_AUTO, /* 首次启动自动标定, 已保存 */
    BMI088_CALIB_SRC_MANUAL,     /* 运行期按需标定, 已保存 */
    BMI088_CALIB_SRC_DEFAULT,    /* 标定失败, 退回编译期默认值 */
} BMI088_CalibSource_e;

/* ---------------- 按需标定状态机 ---------------- */
typedef enum
{
    BMI088_RECALIB_IDLE = 0, /* 空闲 */
    BMI088_RECALIB_BUSY,     /* 已受理, 标定/写参数进行中 */
    BMI088_RECALIB_OK,       /* 成功(已写参数区) */
    BMI088_RECALIB_FAIL,     /* 失败(保留旧值, 已置 CALIB_INVALID) */
} BMI088_RecalibState_e;

extern volatile uint8_t bmi088_calib_source;   /* BMI088_CalibSource_e */
extern volatile float bmi088_calib_temp;       /* 标定时 IMU 温度(摄氏度) */
extern volatile uint8_t bmi088_calib_attempts; /* 最近一次标定用了几轮 */

/**
 * @brief 申请一次"按需标定"(由 LCD 长按中键触发, 进程内只允许一个未完成请求)
 * @return 1=已受理(状态变 BUSY); 0=已在进行中或 IMU 未注册
 */
uint8_t BMI088CalibRequest(void);

/**
 * @brief 按需标定的执行体. 必须在 INS 任务里每轮调用(IMU 的 SPI 只被该任务使用)
 * @return 1=本次真的执行了标定(调用方应跳过本轮姿态解算); 0=无请求, 什么都没做
 */
uint8_t BMI088CalibService(void);

/** @brief 取按需标定状态(BMI088_RecalibState_e) */
uint8_t BMI088CalibGetState(void);
/** @brief LCD 显示完结果后调用, 把 OK/FAIL 状态清回 IDLE */
void BMI088CalibAckResult(void);
/** @brief 当前正在使用的标定值是否可信(0 时不允许进入 READY) */
uint8_t BMI088CalibIsValid(void);

#endif // !__BMI088_H__
