/**
 * @file LK9025.h
 * @brief 声明 LK9025 实例注册、独立 PID 闭环和按总线组帧接口。
 * 外部反馈及前馈使用调用方长期保存的只读快照来源；注册在启动阶段串行完成。
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
    float angle_single_round; // 原始单圈角度，单位deg；公开快照转换为输出轴rad。
    float speed_rads;         // speed rad/s
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度,C°

    float total_angle;   // 原始多圈角度，单位deg；闭环及公开快照使用输出轴rad。
    int32_t total_round; // 总圈数

    float feed_dt;
    uint32_t feed_dwt_cnt;
} LKMotor_Measure_t;

typedef struct
{
    LKMotor_Measure_t measure;
    Motor_Output_Axis_Config_s output_axis;

    Motor_Control_Setting_s motor_settings;
    MotorPIDClock_t pid_clock; // 独立记录控制调用时刻，不复用反馈计时。

    Motor_Signal_Source_s other_angle_feedback;
    Motor_Signal_Source_s other_speed_feedback;
    Motor_Signal_Source_s speed_feedforward;
    Motor_Signal_Source_s current_feedforward;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float pid_ref;

    Motor_Working_Type_e stop_flag; // 启停标志

    CANInstance *motor_can_ins;
    uint8_t motor_id;      // 反馈编号1～4，与发送帧中的两字节位置对应。
    uint8_t can_bus_index; // 对应 DEVICE_CAN_CNT 中的物理总线序号。
    uint32_t feedback_timestamp_ms; // 最后一份 CAN 反馈进入接收队列的 HAL 毫秒时刻。
    uint32_t feedback_sequence; // 有效反馈递增序号，0 保留给未收到样本。

    DaemonInstance *daemon;

} LKMotorInstance;

/**
 * @brief 登记一个 LK 电机并准备它的 PID、离线计数和 CAN 接收。
 * @param[in] config 有效初始化配置；motor_type 须为 LK9025，can_bus 为板级已绑定的逻辑总线，tx_id 填1～4的电机编号。
 * @return 长期有效实例；空参数、未知总线、编号错误、同总线编号重复、活动反馈／前馈缺失、PID 或资源初始化失败返回 NULL。
 * @note 每条总线最多4个，允许编号稀疏及乱序。CAN接收ID设为0x140+编号，发送ID固定为0x280。
 *       output_axis 必须提供有限正减速比和合法方向；输出轴目标、测量及 PID 使用 rad、rad/s。
 *       配置本身保持不变。只在启动阶段串行调用，并在 CAN 派发、Daemon 与控制任务运行前完成；
 *       本任务未提供运行期注销，成功实例及其资源会保留到复位。实际参与闭环的外部反馈及前馈
 *       必须提供完整来源，来源枚举须为 MOTOR_FEED 或 OTHER_FEED；未参与分支可为空。
 *       输出轴闭环参考、反馈和 PID 单位为 rad、rad/s；外部来源显式声明单位、参考系、时间和有效性。
 */
LKMotorInstance *LKMotorInit(const Motor_Init_Config_s *config);

/** @brief 复制 LK9025 的统一单位反馈快照；读取不刷新设备样本时间。 */
Motor_Feedback_Status_e LKMotorGetFeedbackSnapshot(const LKMotorInstance *motor,
                                                    uint32_t now_ms,
                                                    Motor_Feedback_Snapshot_s *snapshot);

/**
 * @brief 保存参考值供下一轮控制使用，不自动换单位。
 * @param[in,out] motor 由 LKMotorInit 返回的有效实例，不能为 NULL。
 * @param[in] ref 须与最外层环匹配；角度环使用输出轴rad，速度环使用输出轴rad/s；直接输出使用设备电流量纲。
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
