/** @file config.h
* @brief 机器人机械、电机和控制参数类型，以及只读配置读取入口。
* 当前只有哨兵配置；未确认的实物参数仍保持未确认状态，不用测试数值替代。
 */
#pragma once

#include "build_selection.h"
#include "control_task_config.h"
#include "chassis_motion.h"
#include "controller.h"
#include "motor_def.h"
#include "../../modules/motor/motor_task.h"
#include "robot_alarm.h"
#include "topics/robot_can_bridge_config.h"
#include "referee_ui_config.h"
#include "shoot_config_types.h"

/*
 * 机器人配置框架. VISION_USE_VCP / VISION_USE_UART / ONE_BOARD 等板级宏
 * 仍保留在 robot_def.h 中定义, 此处不重复, 避免冲突.
 */

/* 当前机器人类型(决定 RobotConfigInit 返回哪套配置) */
#define ROBOT_CONFIG_SUPPORTED_ID SENTINEL_ROBOT

#define RC_TO_YAW_ANGLE 0.002f    // 每个命令周期、每单位摇杆读数增加的 yaw 角度，单位 deg。
#define RC_TO_PITCH_ANGLE 0.001f  // 每个命令周期、每单位摇杆读数增加的 pitch 角度，单位 deg。

// #define _IS_IMU_ROLL    // 是否使用IMU的roll角作为pitch角(由于不常修改,作为宏). 哨兵pitch反馈源依机械安装而定

// 电机位置
typedef enum
{
    MOTOR_LF = 0,
    MOTOR_RF,
    MOTOR_LB,
    MOTOR_RB,
    MOTOR_UP_LF,
    MOTOR_UP_RF,
    MOTOR_UP_LB,
    MOTOR_UP_RB
} Motor_Turn_e;

// 机器人底盘类型
typedef enum
{
    MECANUM_WHEEL = 0,
    OMNI_WHEEL,
    STEER_WHEEL,
    WHEEL_LEG,
    HALF_STEER_WHEEL
} Wheel_Type_e;

/** @brief 一只驱动轮的电机接线编号及输出轴安装方向。 */
typedef struct {
    uint8_t device_id; /**< DJI 设备编号，1～8；总线来自公共轮电机配置。 */
    Motor_Axis_Direction_e direction; /**< 轮轴正方向相对电机轴的符号，0 表示待确认。 */
} ChassisWheelMotorConfig_t;

typedef struct
{
    Wheel_Type_e wheel_type;
    MecanumGeometry_t mecanum_geometry; /**< 麦轮尺寸与计算中心，前左上坐标，m。 */
    ChassisMotionLimits_s motion_limits;
    OmniGeometry_t omni_geometry;
    uint32_t odometry_max_gap_ms; /**< 两个来源样本间隔的上限，ms；0 关闭轮速里程计。 */

    Motor_Init_Config_s chassis_motor_config; /**< 共用总线、减速比与控制参数。设备编号和方向从 wheel_motors 读取。 */
    PID_Init_Config_s chassis_follow_gimbal_PID;

    ChassisWheelMotorConfig_t wheel_motors[CHASSIS_WHEEL_COUNT]; /**< 左前、右前、左后、右后。 */
} Chassis_Config_s;

// 机器人云台类型
typedef enum
{
    SINGLE_GIMBAL = 0,
    MINI_GIMBAL,
    DUAL_GIMBAL,
} Gimbal_Type_e;

// 云台归中参数
typedef struct
{
    // 小 yaw、底盘对齐偏置及 pitch 上下限，角度单位均为度。
    float mini_yaw_offset;
    float yaw_offset;
    float pitch_max_angle, pitch_min_angle;
} Gimbal_Measure_s;

typedef struct
{
    Gimbal_Type_e gimbal_type;
    Gimbal_Measure_s gimbal_offset;
    Motor_Init_Config_s yaw_motor_config;
    Motor_Init_Config_s mini_yaw_motor_config;
    Motor_Init_Config_s pitch_motor_config;
} Gimbal_Config_s;

typedef struct
{
    Shoot_Type_e shoot_type;
    Motor_Init_Config_s friction_motor_config;
    Motor_Init_Config_s loader_motor_config;

    uint8_t friction_motor_id[4];
    Motor_Reverse_Flag_e friction_motor_reverse_flag[4];
    uint8_t loader_slot_num;     // 拨弹盘槽位数(每转一圈发射的弹丸数)
    float loader_gearbox_ratio;  // 电机转子 -> 输出轴减速比(M2006=36, M3508=19)
    float loader_disk_ratio;     // 输出轴 -> 拨弹盘减速比(哨兵=1)
    float friction_default_speed_rad_s; // 摩擦轮输出轴默认目标速度，rad/s
    float loader_reverse_speed_rad_s; // 拨弹盘输出轴反转目标速度，rad/s
} Shoot_Config_s;

/** @brief 核心 CAN 恢复所需的车型确认数据。 */
typedef struct {
    uint32_t feedback_stable_ms; // 所有必需反馈持续有效的候选时间，单位 ms。
    uint8_t confirmed; // 1 表示已按实车反馈周期和故障注入确认；0 仅供软件验证。
} RobotCanRecoveryConfig_s;

struct StdCmdRemoteMap; // 人工输入映射由 standard_cmd 定义，配置只保留只读指针。
typedef struct {
    const struct StdCmdRemoteMap *map; // 实际摇杆、键盘比例和方向；未确认时为 NULL。
    uint32_t max_dt_ms; // 人工运动计算允许的最大实测间隔，单位 ms；未确认时为 0。
    float commissioning_linear_accel_m_s2; // 受限底盘平移目标向量的变化率上限，单位 m/s²；待确认时填 0。
    float commissioning_angular_accel_rad_s2; // 受限底盘绕上轴目标的变化率上限，单位 rad/s²；待确认时填 0。
} RobotManualInputConfig_s;

#if defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
/** @brief 检查受限底盘的人工参数，不访问设备或话题。
 * @param[in] input 车型提供的只读映射、最大间隔与变化率，不能为空。
 * @return 1 表示本体映射和间隔有效，最大速度步长与两种来源的速度边界均可换算；否则返回 0。
 */
uint8_t RobotCommissioningChassisInputIsValid(const RobotManualInputConfig_s *input);

/** @brief 检查受限底盘的人工输入期限是否至少覆盖一个名义命令周期。
 * @param[in] input 已通过车型参数检查的人工输入配置。
 * @param[in] command_period_us 实际命令发布任务的名义周期，单位 μs。
 * @return 1 表示最大计算间隔与两种来源的最长有效时间均覆盖一个周期；否则返回 0。
 * @note 这是启动检查。运行时仍按帧的原始时间和实测间隔撤销过期目标。
 */
uint8_t RobotCommissioningChassisInputFitsPeriod(const RobotManualInputConfig_s *input,
                                               uint32_t command_period_us);

/** @brief 检查受限底盘的名义拍点等待和执行预算能否落在输入及话题有效期内。
 * @param[in] input 已校验的人工输入配置。
 * @param[in] command 人工命令任务的周期和执行预算，单位 μs。
 * @param[in] chassis 独立底盘任务的周期和执行预算，单位 μs。
 * @param[in] motor 电机计算和发送的固定周期，单位 μs。
 * @param[in] dispatch 电机派发任务的周期和执行预算，单位 μs。
 * @return 1 表示按各阶段最不利的名义相位估算，仍未越过输入及话题期限。缺项或超期返回 0。
 * @note 输入范围从合法数据已可读取开始。实际接收、调度抖动、总线等待和停车时延需另行验证。
 */
uint8_t RobotCommissioningChassisTimingFits(const RobotManualInputConfig_s *input,
    const PlatformPeriodicConfig_t *command, const PlatformPeriodicConfig_t *chassis,
    const MotorTaskPeriods_t *motor, const PlatformPeriodicConfig_t *dispatch);
#endif

// 机器人参数结构体
typedef struct
{
    Chassis_Config_s chassis_param;
    RobotControlTaskConfig_t control_tasks[ROBOT_CONTROL_TASK_COUNT]; // 各控制任务的拍点、监控期限及栈容量。
    MotorTaskPeriods_t motor_periods; // 电机计算与队列发送拍点，启动后不可改变。
    Gimbal_Config_s gimbal_param;
    Shoot_Config_s shoot_param;
    RobotCanRecoveryConfig_s can_recovery; // CAN 自动恢复参数；未确认时不能作为实车恢复授权。
    RobotManualInputConfig_s manual_input; // 人工映射及周期保护；未确认时不允许运动。
    PlatformImuPort_t imu_sensor; // 姿态解算使用的逻辑 IMU，由板级适配绑定。
    PlatformPwmChannel_t imu_heater_pwm; // 温控输出使用的逻辑 PWM。
    PlatformAdcChannel_t imu_heater_supply; // 温控检查电源电压使用的逻辑 ADC。
    PlatformUartPort_t referee_uart; // 裁判接收使用的逻辑串口，由板级适配绑定。
    PlatformUartPort_t vision_uart; // 上位机 UART 模式使用的逻辑串口；USB 模式忽略。
    PlatformUartPort_t remote_uart; // 遥控接收使用的逻辑串口，由板级适配绑定。
    const RobotAlarmConfig_t *alarm_config; // 长期只读的状态与故障提示表。
    const RefereeUIConfig_t *referee_ui_config; // 可选的只读裁判界面布局；不影响裁判接收和控制许可。
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD)
    RobotCanBridgeConfig_t can_bridge; // 本板的板间总线、ID和底盘话题。
#endif
} Robot_Config_s;

// 机器人类型
typedef enum
{
    HERO_ROBOT = 1,
    ENGINEER_ROBOT,
    INFANTRY_ROBOT_3,
    INFANTRY_ROBOT_4,
    INFANTRY_ROBOT_5,
    AERIAL_ROBOT,
    SENTINEL_ROBOT,
    DART_ROBOT,
} Robot_Type_e;

#include "robot_def.h"

/* 配置入口(在 robot_def.c 中实现) */
typedef enum {
    ROBOT_CONFIG_OK = 0, ROBOT_CONFIG_ARGUMENT, ROBOT_CONFIG_UNSUPPORTED,
    ROBOT_CONFIG_BAD_PARAMETERS, // 机构配置无效。
    ROBOT_CONFIG_BAD_MANUAL_INPUT // 受限底盘人工映射、间隔或变化率无效。
} RobotConfigStatus_t;
/** @brief 选择唯一已实现车型；失败清除活动选择，不退回其他车型。
 * @param[in] id 车型编号，当前仅支持SENTINEL_ROBOT。
 * @param[out] output 长期只读配置，不可为 NULL。车型不支持或参数无效时清空输出和当前选择。
 * @return 成功为 OK，失败区分参数空缺、不支持、机构配置无效及受限底盘人工配置无效。调用不操作硬件。
 */
RobotConfigStatus_t RobotConfigSelect(uint8_t id, const Robot_Config_s **output);
/** @brief 选择车型并返回其只读配置，兼容历史调用接口。
 * @param[in] id 机器人类型；当前仅支持 SENTINEL_ROBOT。
 * @return 成功时返回长期有效的内部配置地址；选择失败时返回NULL并清除当前选择。
 */
const Robot_Config_s *RobotConfigInit(uint8_t id);
/** @brief 读取已选车型的完整只读配置。
 * @return 配置长期有效；尚未成功选择车型时返回NULL。
 */
const Robot_Config_s *RobotConfigFeed(void);
/** @brief 取得已选配置中的底盘部分。
* @return 内部字段地址，长期有效；未初始化或未启用此模块时返回NULL。
 */
const Chassis_Config_s *ChassisConfigFeed(void);
/** @brief 取得已选配置中的云台部分。
* @return 内部字段地址，长期有效；未初始化或未启用此模块时返回NULL。
 */
const Gimbal_Config_s *GimbalConfigFeed(void);
/** @brief 取得已选配置中的发射部分。
* @return 内部字段地址，长期有效；未初始化或未启用此模块时返回NULL。
 */
const Shoot_Config_s *ShootConfigFeed(void);
/** @brief 取得已选配置中的裁判 UI 布局。
* @return 长期只读布局；未完成 RobotConfigInit() 或车型未配置时返回NULL。
*/
const RefereeUIConfig_t *RefereeUIConfigFeed(void);
