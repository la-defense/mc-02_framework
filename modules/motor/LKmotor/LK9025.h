/**
 * @file LK9025.h
 * @brief 声明 LK9025 实例注册、独立 PID 闭环和按总线组帧接口。
 * 反馈及前馈指针由调用方长期保存；注册在启动阶段串行完成。
 */
#ifndef LK9025_H
#define LK9025_H

#include "stdint.h"
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define LK_MOTOR_PER_BUS_CNT 4u /**< 每条 CAN 总线最多登记的 LK 电机数。 */
#define LK_MOTOR_MX_CNT (DEVICE_CAN_CNT * LK_MOTOR_PER_BUS_CNT) /**< H723 三条总线共最多12个。 */

#define I_MIN -2000
#define I_MAX 2000
#define CURRENT_SMOOTH_COEF 0.9f
#define SPEED_SMOOTH_COEF 0.85f
#define REDUCTION_RATIO_DRIVEN 1
#define ECD_ANGLE_COEF_LK (360.0f / 65536.0f)
#define CURRENT_TORQUE_COEF_LK 0.003645f // 电流设定值转换成扭矩的系数,算出来的设定值除以这个系数就是扭矩值

typedef struct // 9025
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 当前编码器值
    float angle_single_round; // 单圈角度
    float speed_rads;         // speed rad/s
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度,C°

    float total_angle;   // 总角度
    int32_t total_round; // 总圈数

    float feed_dt;
    uint32_t feed_dwt_cnt;
} LKMotor_Measure_t;

typedef struct
{
    LKMotor_Measure_t measure;

    Motor_Control_Setting_s motor_settings;
    MotorPIDClock_t pid_clock; // 独立记录控制调用时刻，不复用反馈计时。

    float *other_angle_feedback_ptr; // 外部角度反馈，单位deg，按配置选择后读取。
    float *other_speed_feedback_ptr; // 外部速度反馈，单位rad/s，按配置选择后读取。
    float *speed_feedforward_ptr;   // 角度分支后加入的速度前馈，单位由串级配置决定。
    float *current_feedforward_ptr; // 速度分支后加入的当前输出前馈，单位由串级配置决定。
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float pid_ref;

    Motor_Working_Type_e stop_flag; // 启停标志

    CANInstance *motor_can_ins;
    uint8_t motor_id;      // 反馈编号1～4，与发送帧中的两字节位置对应。
    uint8_t can_bus_index; // 对应 DEVICE_CAN_CNT 中的物理总线序号。

    DaemonInstance *daemon;

} LKMotorInstance;

/**
 * @brief 登记一个 LK 电机并准备它的 PID、离线计数和 CAN 接收。
 * @param[in] config 有效初始化配置；motor_type 须为 LK9025，can_handle 为已支持的总线句柄，tx_id 填1～4的电机编号。
 * @return 长期有效实例；空参数、未知总线、编号错误、同总线编号重复、活动反馈／前馈缺失、PID 或资源初始化失败返回 NULL。
 * @note 每条总线最多4个，允许编号稀疏及乱序。CAN接收ID设为0x140+编号，发送ID固定为0x280。
 *       配置本身保持不变。只在启动阶段串行调用，并在 CAN 派发、Daemon 与控制任务运行前完成；
 *       本任务未提供运行期注销，成功实例及其资源会保留到复位。实际参与闭环的外部反馈及前馈
 *       必须提供非空指针，来源枚举须为 MOTOR_FEED 或 OTHER_FEED；未参与分支的字段可为空。
 *       外部数值在每轮读取一次，单位为角度deg、速度rad/s；指向的存储须在实例使用期间有效。
 */
LKMotorInstance *LKMotorInit(const Motor_Init_Config_s *config);

/**
 * @brief 保存参考值供下一轮控制使用，不自动换单位。
 * @param[in,out] motor 由 LKMotorInit 返回的有效实例，不能为 NULL。
 * @param[in] ref 须与所选最外层环匹配；角度参考和总角度反馈用 deg，速度反馈用 rad/s，
 *        速度 PID 接收 rad/s；各环的参考与反馈必须使用相同单位。
 * @note 外部角度及速度分别由 angle_feedback_source、speed_feedback_source 选择；三环各自保存 PID 历史。
 */
void LKMotorSetRef(LKMotorInstance *motor, float ref);

/**
 * @brief 分别计算各已登记电机，并为每条非空总线提交一帧八字节0x280命令。
 * @return 无返回值；CAN 本机提交结果由现有 CAN 发送统计记录，不代表对端已收到。
 * @note 由电机控制任务顺序调用。编号1～4对应帧内四个两字节位置；空槽和停止电机写零，
 *       非有限输入、非法反馈来源、PID失败、前馈缺失或计算溢出时只清本实例槽位并清除三环历史；
 *       发送失败的帧不缓存，下一周期重新计算。
 */
void LKMotorControl(void);

/**
 * @brief 设置停止标记，下一次控制将对应两字节输出清零；保留目标值。
 * @param[in,out] motor 由 LKMotorInit 返回的有效实例，不能为 NULL。
 */
void LKMotorStop(LKMotorInstance *motor);

/**
 * @brief 设置运行标记，下一次控制继续使用已保存目标。
 * @param[in,out] motor 由 LKMotorInit 返回的有效实例，不能为 NULL。
 */
void LKMotorEnable(LKMotorInstance *motor);

/**
 * @brief 读取对应 Daemon 是否仍有剩余计数。
 * @param[in] motor 由 LKMotorInit 返回的有效实例，不能为 NULL。
 * @return 1 为计数未耗尽，0 为离线；不代表数据有效或已允许输出。
 */
uint8_t LKMotorIsOnline(LKMotorInstance *motor);

#endif // LK9025_H
