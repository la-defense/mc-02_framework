/**
 * @file dji_motor.h
 * @author neozng
 * @brief DJI智能电机头文件
 * @version 0.2
 * @date 2022-11-01
 *
 * @todo  为M2006和M3508增加开环的零位校准函数,并在初始化时调用(根据用户配置决定是否调用)

 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"
#include "sample_filter.h"

#define DJI_MOTOR_CNT 12

/* 原DJIMotorInit()继续沿用这两个逐帧兼容系数。 */
#define SPEED_SMOOTH_COEF 0.85f
#define CURRENT_SMOOTH_COEF 0.9f
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制

/** @brief 每个反馈量可单独选择旧逐帧滤波、按真实dt计算的一阶低通或原始值。 */
typedef enum
{
    DJI_MOTOR_FILTER_LEGACY_FRAME = 0,
    DJI_MOTOR_FILTER_TIME_CONSTANT = 1,
    DJI_MOTOR_FILTER_DISABLED = 2
} DJI_Motor_Filter_Mode_e;

/** @brief 反馈滤波状态；异常间隔后，下一份合法样本会明确标记为重建。 */
typedef enum
{
    DJI_FILTER_STATUS_WAITING = 0,
    DJI_FILTER_STATUS_VALID = 1,
    DJI_FILTER_STATUS_REBUILT = 2,
    DJI_FILTER_STATUS_GAP = 3,
    DJI_FILTER_STATUS_BYPASSED = 4,
    DJI_FILTER_STATUS_ERROR = 5
} DJI_Motor_Filter_Status_e;

/** @brief 每实例固定滤波配置；时间常数和间隔单位均为秒。 */
typedef struct
{
    DJI_Motor_Filter_Mode_e speed_mode;
    DJI_Motor_Filter_Mode_e current_mode;
    float speed_tau_s; /**< 速度时间常数；仅 TIME_CONSTANT 模式要求大于0。 */
    float current_tau_s; /**< 电流时间常数；仅 TIME_CONSTANT 模式要求大于0。 */
    float dt_min_s; /**< 相邻反馈间隔下限，含边界，默认与 PID 下限相同。 */
    float dt_max_s; /**< 相邻反馈间隔上限，含边界，最大不超过50ms。 */
} DJI_Motor_Filter_Config_s;

/* DJI电机CAN反馈信息*/
typedef struct
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 0-8191,刻度总共有8192格
    float angle_single_round; // 单圈角度，单位 deg。
    float speed_aps;          // 当前有效速度反馈，单位deg/s。
    int16_t real_current;     // 当前有效电流的整数兼容值，不换算为A。
    float raw_speed_aps;      // 当前CAN帧直接换算的速度，单位deg/s。
    int16_t raw_current;      // 当前CAN帧中的原始电流字段，不换算为A。
    float current_feedback;   // 控制器实际使用的电流反馈，保留时间常数滤波的小数结果。
    uint8_t temperature;      // 温度 Celsius

    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
} DJI_Motor_Measure_s;

/**
 * @brief DJI intelligent motor typedef
 *
 */
typedef struct
{
    DJI_Motor_Measure_s measure;            // 电机测量值
    Motor_Output_Axis_Config_s output_axis; // 控制与公开位置／速度均按此配置换算。
    Motor_Control_Setting_s motor_settings; // 电机设置
    MotorPIDClock_t pid_clock; // 独立记录控制调用时刻，不复用反馈计时。
    Motor_Controller_s motor_controller;    // 电机控制器

    CANInstance *motor_can_instance; // 电机CAN实例
    // 分组发送设置
    uint8_t sender_group;
    uint8_t message_num;

    Motor_Type_e motor_type;        // 电机类型
    Motor_Working_Type_e stop_flag; // 启停标志
    Motor_Online_Flag_e online_flag; // 在线标志

    DaemonInstance* daemon;
    DJI_Motor_Filter_Config_s filter_config;
    SampleFilter_t speed_filter;
    SampleFilter_t current_filter;
    uint32_t last_feedback_cycles;
    uint32_t last_feedback_timestamp_ms;
    uint32_t feedback_sample_count;
    float feedback_dt_s;
    DJI_Motor_Filter_Status_e speed_filter_status;
    DJI_Motor_Filter_Status_e current_filter_status;
    uint8_t has_feedback_timestamp;
    uint8_t speed_filter_valid;
    uint8_t current_filter_valid;
    uint8_t speed_filter_started;
    uint8_t current_filter_started;
    uint8_t speed_filter_needs_rebuild;
    uint8_t current_filter_needs_rebuild;
    uint8_t encoder_baseline_valid;
    uint8_t output_position_trusted;
    uint32_t feedback_sequence;
} DJIMotorInstance;

typedef struct
{
    uint8_t online;
    uint8_t enabled;
    float speed_aps;
    float angle_single_round;
    float total_angle;
    int16_t current;
    uint8_t temperature;
    float raw_speed_aps;
    int16_t raw_current;
    float current_feedback;
    float feedback_dt_s;
    uint32_t feedback_sample_count;
    DJI_Motor_Filter_Status_e speed_filter_status;
    DJI_Motor_Filter_Status_e current_filter_status;
    uint8_t speed_filter_valid;
    uint8_t current_filter_valid;
} DJIMotorSummary_t;

/** @brief 注册 DJI 电机、CAN 接收与离线计数，成功后保持停止输出。
* @param[in] config 有效配置；支持 M2006、M3508、GM6020，当前检查设备编号 1～8，按型号分别计算接收 ID。
* @return 成功返回长期有效对象，参数、重复 ID、容量或注册分配失败返回 NULL，并清理本次已分配资源。
* @note 先配置BSP，再在应用初始化阶段注册；output_axis须明确减速比与方向。
*       角度／速度闭环的参考、反馈和PID输入分别使用输出轴rad、rad/s；电流环保留设备电流数值。
*       外部来源上下文须在实例使用期间有效，回调复制带单位、参考系、时刻、序号和有效标记的快照。
 */
DJIMotorInstance *DJIMotorInit(const Motor_Init_Config_s *config);

/** @brief 使用独立反馈滤波配置注册 DJI 电机，滤波历史由返回实例长期保存。
* @param[in] config 有效电机配置；函数不修改输入，成功实例长期保存 CAN 与 Daemon 资源。
* @param[in] filter_config 非空配置；速度／电流可各自选择逐帧兼容、时间常数或关闭滤波。
*        dt_min_s、dt_max_s 使用秒，范围须满足 0 < min <= max <= 0.05；边界允许。
*        TIME_CONSTANT 模式的对应 tau 使用秒且须有限并大于0；其他模式的 tau 须为0。
* @return 成功返回长期实例；参数、滤波配置、PID、Daemon 或 CAN 注册失败返回 NULL，且不发布半成品。
* @note 初始化必须在接收派发与电机控制任务启动前串行调用。输入配置在返回后可释放；
*       DJIMotorInit() 仍默认使用速度alpha=0.85、电流alpha=0.9的逐帧兼容滤波。
*       时间常数模式首个有效样本直接建立历史，后续使用CAN RX中断记录的DWT周期差。
*/
DJIMotorInstance *DJIMotorInitWithFilter(const Motor_Init_Config_s *config,
                                         const DJI_Motor_Filter_Config_s *filter_config);

/** @brief 保存目标，供下一轮 DJIMotorControl() 计算，不直接发送。
* @param[in,out] motor 有效实例，不能为 NULL。
* @param[in] ref 有限参考；角度外环使用输出轴rad，速度外环使用输出轴rad/s，直接输出仍为电调设备数值。
* @note 外部来源须声明匹配单位和参考系；电调原始电流不自动换算为A。
 */
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);

/** @brief 改写角度或速度反馈来源，下一轮控制生效。
* @param[in,out] motor 有效实例，不能为 NULL。
* @param[in] loop ANGLE_LOOP 或 SPEED_LOOP，其他值只打印错误。
* @param[in] type MOTOR_FEED 用本机反馈，OTHER_FEED 用初始化时保存且本轮有效的快照来源。
 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

/** @brief 逐电机计算已选 PID 环，再按四电机一组提交 CAN；由 MotorControlTask() 周期调用。 */
void DJIMotorControl();

/** @brief 设置停止标记，下一轮控制将该电机对应的发送字节清零，保留原目标值。
* @param[in,out] motor 有效实例，不能为 NULL；需要清目标时另调用 DJIMotorSetRef()。
 */
void DJIMotorStop(DJIMotorInstance *motor);

/** @brief 设置运行标记，下一轮控制重新使用已保存目标，不负责整车许可检查。
* @param[in,out] motor 有效实例，不能为 NULL；初始化成功时默认保持停止。
 */
void DJIMotorEnable(DJIMotorInstance *motor);

/** @brief 改写从哪个控制环开始使用参考值。
* @param[in,out] motor 有效实例，不能为 NULL。
* @param[in] outer_loop 目标最外层类型，须与 close_loop_type 和 PID 配置匹配。
 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop);

/** @brief 查询 Daemon 并更新实例 online_flag，本函数没有返回值。
* @param[in,out] motor 有效实例，不能为 NULL；在线不等于已允许输出。
 */
void DJIMotorIsOnline(DJIMotorInstance *motor);

/** @brief 改写参与计算的控制环组合。
* @param[in,out] motor 有效实例，不能为 NULL。
* @param[in] outer_loop 实际写入 close_loop_type 的标志组合，可按位或，名称沿用旧接口。
 */
void DJIMotorCloseLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop);

/** @brief 复制在线、运行及当前测量值，供显示读取。
* @param[in] motor 电机实例，NULL 时不处理。
* @param[out] summary 输出对象，NULL 时不处理；失败时保持原值，不自动清零。
* @note 兼容摘要的angle／speed保留设备侧deg、deg/s；控制闭环及统一反馈接口使用输出轴rad、rad/s。
*       温度用摄氏度，current为电调原始反馈整数，不代表安培。
 */
void DJIMotorGetSummary(DJIMotorInstance *motor, DJIMotorSummary_t *summary);

/** @brief 以当前有效编码器读数为新的相对零点，显式恢复多圈位置可信度。 */
Motor_Feedback_Status_e DJIMotorRebaseReference(DJIMotorInstance *motor, uint32_t now_ms);

/**
 * @brief 复制统一反馈数据；读取不刷新样本时间或滤波状态。
 * @param[in] motor 已注册实例。
 * @param[in] now_ms 调用方毫秒时刻，允许0和回绕；读取时还会检查同一时钟域的本机时刻。
 * @param[out] snapshot 输出对象；参数或换算失败时保持原值。
 * @return MOTOR_FEEDBACK_OK 表示已收到样本；各字段是否可用由字段 valid 表示。
 * @note 使用调用时刻与数据复制时刻中较新的本机时刻，避免把并发发布的新样本判为过期。
 */
Motor_Feedback_Status_e DJIMotorGetFeedbackSnapshot(const DJIMotorInstance *motor,
                                                     uint32_t now_ms,
                                                     Motor_Feedback_Snapshot_s *snapshot);

/* ============================================================================
   DJI 假反馈注入自测(默认关闭)
   ----------------------------------------------------------------------------
   用途: 在没有真实电机的情况下验证整条 CAN 反馈链路 ——
         "回环发送 → 中断入队 → CANProcessRx() 派发 → DecodeDJIMotor 解码 → 多圈累加"。
   原理: 把某条总线切到内部回环, 注册一个"只发不收"的临时实例(tx_id = 目标电机的
         rx_id, 比如 0x201), 它发出去的帧会命中该电机的接收滤镜, 于是走进正常解码路径。
   取值: 0 = 关闭(默认); 1 = 启动时跑一次功能自测; 2 = 功能自测 + daemon 循环持续压测
   注意: 会覆盖目标电机的 measure, 只在确认没接真实电机时打开。
   ========================================================================== */
#ifndef CAN_DJI_INJECT_TEST
#define CAN_DJI_INJECT_TEST 0
#endif

#if CAN_DJI_INJECT_TEST >= 1
/** @brief 跑一次注入自测(解码 + 多圈累加), 返回 1=全通过 */
uint8_t CANRunDjiFeedbackInjectTest(void);
extern volatile uint32_t can_dji_inject_pass;
extern volatile uint32_t can_dji_inject_fail;
extern volatile uint32_t can_dji_inject_multi_turn_err;
extern volatile uint32_t can_dji_inject_frame_cnt;
#endif
#if CAN_DJI_INJECT_TEST >= 2
/** @brief 压测模式: 由 daemon 循环周期调用, 每轮塞一批帧观察队列深度与丢帧 */
void DJIInjectStressTick(void);
#endif

#endif // !DJI_MOTOR_H
