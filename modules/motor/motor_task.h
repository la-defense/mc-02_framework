/**
 * @file motor_task.h
 * @author neozng
 * @brief  对所有电机,舵机等控制任务的进一步封装,MotorControlTask()将在操作系统中按一定频率调用
 * @version beta
 * @date 2022-11-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H
#include "../../bsp/os/platform_periodic.h"

#define MOTOR_DJI_QUEUE_LIFETIME_MS 4u // DJI 周期帧进入软件队列后最多保留 4 ms。

/** @brief 车型指定的电机工作周期。两项均须在启动时确认。 */
typedef struct {
    uint32_t compute_us; /**< 闭环计算与提交候选帧的周期，单位 μs。 */
    uint32_t transmit_us; /**< 推进 CAN 硬件发送队列的周期，单位 μs。 */
} MotorTaskPeriods_t;

typedef struct {
    PlatformPeriodicClock_t compute; /**< 计算自己的固定拍点。 */
    PlatformPeriodicClock_t transmit; /**< 发送自己的固定拍点。 */
} MotorTaskSchedule_t;

/** @brief 校验并建立电机任务的两个工作周期。
 * @param[out] schedule 两项都通过才写入。
 * @param[in] periods 车型周期配置，不可为空。
 * @param[in] dispatch_us 单一接收任务轮询间隔，单位 μs。
 * @param[in] quantum_us 平台每刻度的 μs 数。
 * @param[in] first_tick 两项相同的启动拍点，使用平台计数器。
 * @return READY 为有效，CONFIG 为未确认或无法按当前轮询间隔运行。
 */
PlatformPeriodicResult_t MotorTaskScheduleInit(MotorTaskSchedule_t *schedule, const MotorTaskPeriods_t *periods,
    uint32_t dispatch_us, uint32_t quantum_us, uint32_t first_tick);

/** @brief 在推进发送队列前检查当前派发轮次，返回 0 时本轮不进入发送阶段。 */
typedef uint8_t (*MotorTaskTransmitCheck_t)(void *context);

/** @brief 派发接收，执行到期的计算，检查通过后再进入发送。过期拍点不会补跑。
 * @param[in,out] schedule 当前电机任务独占的周期状态。
 * @param[in] now_tick 本次从接收任务时钟观察到的计数值。
 * @param[in] check_transmit 发送到期时调用的检查，不可为空。
 * @param[in,out] context 传给检查函数的调用者状态，允许为空。
 * @return READY 为两个拍点均已检查，发送还需获得 check_transmit 允许。
 * @note 拍点检查失败时不访问收发或电机；发送检查拒绝时不推进 CAN 发送服务。
 */
PlatformPeriodicResult_t MotorTaskRunScheduled(MotorTaskSchedule_t *schedule, uint32_t now_tick,
    MotorTaskTransmitCheck_t check_transmit, void *context);

/** @brief 先派发 CAN、UART、USB 接收，再在正式配置计算 DJI、LK 输出；裸板配置跳过电机计算。
* @note 由 MOTOR 任务约每 1 ms 调用，全工程仅此处周期派发接收；实际延迟仍取决于调度。
 */
void MotorControlTask(void);

#endif // !MOTOR_TASK_H

