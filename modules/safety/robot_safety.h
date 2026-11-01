/** @file robot_safety.h
* @brief 保存使能、标定、急停和故障请求，统一选择整车当前状态。
* @note 先初始化；普通状态上报和请求之后须调用 RobotSafetyUpdate() 才更新运行状态。
*/
#ifndef ROBOT_SAFETY_H
#define ROBOT_SAFETY_H

#include <stdint.h>
#include "robot_def.h"

/* 机器人故障位, 可同时置位多个 */
typedef enum
{
    ROBOT_FAULT_NONE = 0,
    ROBOT_FAULT_RC_OFFLINE = 1u << 0,
    ROBOT_FAULT_IMU_INVALID = 1u << 1,
    ROBOT_FAULT_MOTOR_OFFLINE = 1u << 2,
    ROBOT_FAULT_CAN_BUSOFF = 1u << 3,
    ROBOT_FAULT_TILT = 1u << 4,
    ROBOT_FAULT_CALIB_INVALID = 1u << 5,
    ROBOT_FAULT_TASK_TIMEOUT = 1u << 6,
    ROBOT_FAULT_INIT_FAILURE = 1u << 7,
    ROBOT_FAULT_RESET_LOCKOUT = 1u << 8,
} Robot_Fault_e;

/* 急停原因 */
typedef enum
{
    ESTOP_REASON_NONE = 0,
    ESTOP_REASON_RC_OFFLINE,
    ESTOP_REASON_MANUAL,
} Estop_Reason_e;

/** @brief 一次读取的 safety 状态及其输入，所有字段来自同一提交时刻。 */
typedef struct
{
    Robot_Status_e state;
    uint32_t fault_flags;
    uint8_t rc_online;
    uint8_t imu_valid;
    uint8_t calib_valid;
    uint8_t task_healthy;
    uint8_t enable_requested;
    uint8_t calib_requested;
    uint8_t estop_latched;
    uint8_t remote_chassis_permit;
    uint32_t chassis_inhibit_generation;
    Estop_Reason_e estop_reason;
} RobotSafetySnapshot_t;

/** @brief 清空故障、急停和请求，进入 SAFE；只在系统初始化时调用。 */
void RobotSafetyInit(void);
/** @brief 根据当前输入选择状态；顺序为初始化失败、急停、任务超时、标定、普通故障、使能请求。 */
void RobotSafetyUpdate(void);

/** @brief 记录遥控在线情况，并设置或清除遥控离线故障。
* @param[in] online 非 0 为在线，为 0 时下次 Update 将触发急停。 */
void RobotSafetySetRcOnline(uint8_t online);
/** @brief 记录 IMU 数据能否使用，并更新对应故障。
* @param[in] valid 非 0 为可用，为 0 为不可用；恢复可用后允许重新报告一次故障日志。 */
void RobotSafetySetImuValid(uint8_t valid);
/** @brief 记录标定结果能否使用，并更新对应故障。
* @param[in] valid 非 0 为可用，为 0 时阻止 READY。 */
void RobotSafetySetCalibValid(uint8_t valid);
/** @brief 记录关键任务是否正常，并更新任务超时故障。
* @param[in] healthy 非 0 为正常；为 0 时，下次 Update 清除使能请求。 */
void RobotSafetySetTaskHealthy(uint8_t healthy);

/** @brief 保存使能请求、取消标定请求；已有初始化失败时忽略，最终状态由 Update 检查。
* @note CHASSIS_BOARD 不使用本地使能请求；请用新鲜的云台侧运动许可。 */
void RobotSafetyRequestEnable(void);
/** @brief 清除使能和标定请求，运行状态在下次 Update 更新。
* @note CHASSIS_BOARD 同时撤销远端底盘许可；恢复前须重新接管新命令。 */
void RobotSafetyRequestDisable(void);
/** @brief 设置或取消标定请求。
* @param[in] enable 非 0 请求标定并清除使能请求，为 0 只取消标定；初始化失败时拒绝新标定。 */
void RobotSafetyRequestCalib(uint8_t enable);

/** @brief 更新双板模式下由云台板授予的底盘运动许可。
 * @param[in] permitted 非0表示本轮收到新鲜且允许运动的底盘命令；0表示禁止或命令失效。
 * @note 仅CHASSIS_BOARD使用；本板任务、姿态、标定、故障和急停检查仍独立生效。
 */
void RobotSafetySetRemoteChassisPermit(uint8_t permitted);
/** @brief 查询底盘板本地条件是否健康，不包含远端运动命令许可。
* @return 本地核心状态健康且未急停、标定或故障时返回1，否则返回0。
* @note 仅CHASSIS_BOARD控制任务用于恢复检查；不代表可以驱动电机。 */
uint8_t RobotSafetyIsChassisLocalReady(void);

// 除底盘板本地 RC_OFFLINE 外，所有已报告故障都阻止 READY。
/** @brief 添加故障位，不立即更新运行状态。
* @param[in] fault 可以同时包含多个 Robot_Fault_e 故障位。 */
void RobotFaultReport(Robot_Fault_e fault);
/** @brief 清除指定故障位；初始化失败和复位额度锁定只能由对应维护流程解除。
* @param[in] fault 需要清除的故障位，可组合多个。 */
void RobotFaultClear(Robot_Fault_e fault);

/** @brief 保存急停和原因并清除使能请求；运行状态在 Update 中转为 ESTOP。
* @param[in] reason 急停原因，重复相同原因不重复报告。 */
void EstopRequest(Estop_Reason_e reason);
/** @brief 尝试清除急停；遥控离线、任务超时或初始化失败未清除时拒绝。
* @return 1 为已清除或原本未急停，0 为仍有上述故障；实际清除时进入 SAFE，须重新请求使能。 */
uint8_t EstopClear(void);
/** @brief 读取当前是否仍保留急停。
* @return 1 为急停未清除，0 为无急停。 */
uint8_t EstopIsLatched(void);
/** @brief 读取最近保存的急停原因。
* @return 当前原因，清除后为 ESTOP_REASON_NONE。 */
Estop_Reason_e EstopGetReason(void);

/** @brief 读取上次状态更新的结果。
* @return 当前 Robot_Status_e 状态。 */
Robot_Status_e RobotSafetyGetState(void);
/** @brief 查询当前是否允许运行。
* @return 1 为上次状态为 READY 且当前输入仍允许运行，0 为其他情况。
* @note 故障、急停、失能和标定请求在上报后立即使该查询返回 0；状态名称仍由 Update 推进。 */
uint8_t RobotSafetyIsReady(void);
/** @brief 读取已设置的全部故障位。
* @return Robot_Fault_e 各位的组合。 */
uint32_t RobotSafetyGetFaults(void);
/** @brief 原子复制 safety 状态、故障和许可输入。
* @param[out] snapshot 输出对象；NULL 返回 0，其余失败不改写输出。
* @return 1 表示复制成功，0 表示参数错误。
* @note 可在普通任务上下文调用；不推进状态机。输出各字段来自同一临界区快照。
 */
uint8_t RobotSafetyGetSnapshot(RobotSafetySnapshot_t *snapshot);
/** @brief 取得状态对应的短名称。
* @param[in] state 要显示的状态。
* @return 长期有效的常量字符串，未知状态返回 STOP，不可改写。 */
const char *RobotSafetyStateName(Robot_Status_e state);

#endif // !ROBOT_SAFETY_H
