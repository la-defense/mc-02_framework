/**
 * @file dji_motor.h
 * @author neozng
 * @brief DJI智能电机头文件
 * @version 0.2
 * @date 2022-11-01
 *
 * @todo  1. 给不同的电机设置不同的低通滤波器惯性系数而不是统一使用宏
          2. 为M2006和M3508增加开环的零位校准函数,并在初始化时调用(根据用户配置决定是否调用)

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

#define DJI_MOTOR_CNT 12

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制

/* DJI电机CAN反馈信息*/
typedef struct
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 0-8191,刻度总共有8192格
    float angle_single_round; // 单圈角度，单位 deg。
    float speed_aps;          // 角速度,单位为:度/秒
    int16_t real_current;     // 电调原始电流反馈整数，未换算为 A。
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
    uint32_t feed_cnt;
    float dt;
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
} DJIMotorSummary_t;

/** @brief 注册 DJI 电机、CAN 接收与离线计数，成功后保持停止输出。
* @param[in] config 有效配置；支持 M2006、M3508、GM6020，当前检查设备编号 1～8，按型号分别计算接收 ID。
* @return 成功返回长期有效对象，参数、重复 ID、容量或注册分配失败返回 NULL，并清理本次已分配资源。
* @note 先配置 BSP，再在应用初始化阶段注册；外部反馈和前馈指针须长期有效，数据单位由调用方匹配。
 */
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config);

/** @brief 保存目标，供下一轮 DJIMotorControl() 计算，不直接发送。
* @param[in,out] motor 有效实例，不能为 NULL。
* @param[in] ref 正常数值；使用电机反馈时，角度目标为 deg、速度目标为 deg/s，直接输出为电调原始数值。
* @note 外部反馈不会自动换单位；目标、反馈和 PID 配置须配套，不可直接把原始电调值当作 A。
 */
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);

/** @brief 改写角度或速度反馈来源，下一轮控制生效。
* @param[in,out] motor 有效实例，不能为 NULL。
* @param[in] loop ANGLE_LOOP 或 SPEED_LOOP，其他值只打印错误。
* @param[in] type MOTOR_FEED 用电机反馈，OTHER_FEED 用初始化时保存的有效指针。
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
* @note 角度用 deg，速度用 deg/s，温度用摄氏度，current 为电调原始反馈整数。
 */
void DJIMotorGetSummary(DJIMotorInstance *motor, DJIMotorSummary_t *summary);

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
