/** @file robot_safety.h
* @brief 保存使能、标定、急停和故障请求，统一选择整车当前状态。
* @note 先初始化；普通状态上报和请求之后须调用 RobotSafetyUpdate() 才更新运行状态。
*/
#ifndef ROBOT_SAFETY_H
#define ROBOT_SAFETY_H

#include <stdint.h>
#include "robot_state.h"
#include "robot_control_policy.h"

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
    uint32_t control_generation; // 相关禁止条件每次发生时递增，防止周期之间的短故障被漏过。
} RobotSafetySnapshot_t;

/** @brief 唯一命令仲裁者提交的来源事实；核心、急停和人工使能仍由原安全状态管理。 */
typedef struct {
    RobotControlMode_t mode; // 本轮选择的控制模式。
    RobotControlLease_t manual; // 已选人工来源，保留其原接收时间。
    RobotControlLease_t aim; // 瞄准来源自己的期限。
    RobotControlLease_t navigation; // 导航来源自己的期限。
    RobotControlLease_t match; // 裁判阶段自己的期限。
    RobotControlLease_t feedback[3]; // 底盘、云台、发射各自的反馈原始时刻和最短有效期限。
    uint32_t feedback_valid_functions; // 本轮必需反馈有效的功能位。
    uint32_t blocked_functions; // 按已确认故障影响范围禁止的功能位。
    uint8_t match_running; // 有效裁判阶段是否允许运行。
    uint8_t manual_fire_requested; // 当前人工请求，不重放旧事件。
    uint8_t vision_fire_requested; // 当前视觉请求，不替代辅助模式的人工请求。
} RobotSafetyControlInputs_t;

/** @brief 启动时配置单板／云台侧模式策略；原默认路径在未配置时保持兼容。
 * @param[in] policy 已确认的车型策略，函数复制；只能在初始SAFE、无使能／标定／急停时配置一次。
 * @return 1为成功；0为非法配置、错误阶段、重复配置或底盘板角色。失败须由启动者报告初始化失败。
 */
uint8_t RobotSafetyConfigureControl(const RobotControlPolicy_t *policy);
/** @brief 由唯一命令仲裁者提交同一轮来源事实，不改变原始来源时刻。
 * @param[in] inputs 非空来源数据，内部复制；错误时撤销旧来源，保持原模式等待新数据。
 * @note 每个有效反馈位必须携带该功能所有必需反馈的最早到期时间，不能因重新汇总而续期。
 * @return 1为接受；0为未配置或输入错误。提交不代替RobotSafetyUpdate推进状态。
 */
uint8_t RobotSafetyPublishControlInputs(const RobotSafetyControlInputs_t *inputs);
/** @brief 同时读取分功能许可；当前来源过期或立即禁止会在查询时再次撤销。
 * @param[in] now_ms 本机毫秒时刻。
 * @param[out] output 非空，仅返回1时整份改写；未进入READY时返回成功的全零许可。
 * @return 1为已配置并完成计算，0为未配置或参数错误；返回0时调用方必须禁止执行。
 */
uint8_t RobotSafetyGetFunctionPermitsAt(uint32_t now_ms, RobotControlPermits_t *output);
/** @brief 查询当前是否选择了车型允许的正式自主；仅用于决定遥控离线是否请求急停。
 * @return 1不代表取得运行许可；比赛阶段、核心条件和急停仍由安全管理检查。
 */
uint8_t RobotSafetyIsFormalAutonomous(void);

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

/** @brief 按安全代际和来源期限原子授予双板底盘运动许可。
 * @param[in] expected_generation 读取本地安全状态时的底盘抑制代际。
 * @param[in] source_time_ms 经可信对时映射的来源命令时刻。
 * @param[in] max_age_ms 来源命令的最大有效期，范围1～INT32_MAX。
 * @param[in] now_ms 与来源时刻同域的当前本机毫秒时刻。
 * @return 当前本地条件、代际和来源期限全部匹配时返回1，否则撤销许可并返回0。
 * @note 仅CHASSIS_BOARD使用；代际比较、输入复核及许可写入在同一临界区完成。
 */
uint8_t RobotSafetyTryGrantRemoteChassisPermit(uint32_t expected_generation,
                                                uint32_t source_time_ms,
                                                uint32_t max_age_ms,
                                                uint32_t now_ms);
/** @brief 立即撤销双板底盘许可；已授予的许可撤销会推进抑制代际。 */
void RobotSafetyRevokeRemoteChassisPermit(void);
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

/** @brief 查询 CAN 恢复所需的安全资格，不清除任何故障。
 * @param[in] expected_generation 本轮读取的安全代际。
 * @return 1 为 CAN 故障存在且其他核心条件及当前运行资格有效；否则返回 0。
 * @note 来源期限使用安全管理实际读取的时间；不检查总线、电机反馈或新命令。
 */
uint8_t RobotSafetyCanRecover(uint32_t expected_generation);

/** @brief 在同一安全代际中检查 CAN 故障以外的条件，并尝试清除 CAN 故障位。
 * @param[in] expected_generation 恢复检查开始时读取的安全代际。
 * @param[in] now_ms 调用方读取的本机时间；实际检查使用安全管理重新读取的时刻。
 * @return 1 为故障位已清除，0 为代际、核心条件或运行资格不满足。
 * @note 只供 CAN 恢复协调者使用；调用方还须确认总线、逐台反馈、稳定时间和恢复后新命令，
 * 并与 CAN 故障代次的最终比较保持同一中断保护。函数不设置 READY 或电机输出。
 */
uint8_t RobotSafetyTryClearCanFault(uint32_t expected_generation, uint32_t now_ms);

/** @brief 保存急停和原因并清除使能请求；运行状态在 Update 中转为 ESTOP。
* @param[in] reason 急停原因，重复相同原因不重复报告。 */
void EstopRequest(Estop_Reason_e reason);
/** @brief 尝试清除急停；任务超时、初始化失败和恢复额度锁定时拒绝，非正式自主还要求遥控在线。
* @return 1 为已清除或原本未急停，0 为仍有上述故障；实际清除进入SAFE，人工模式须重新使能，正式自主重新检查比赛与核心条件。 */
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
* @return 1 为 READY 且当前输入仍允许运行，0 为其他情况。
* @note 故障、急停、失能和标定请求在上报后立即使该查询返回 0；CHASSIS_BOARD 还按当前 HAL 时刻检查远端许可租约。状态名称仍由 Update 推进。 */
uint8_t RobotSafetyIsReady(void);
/** @brief 在指定时刻查询运行许可，并检查双板远程命令的来源期限。
 * @param[in] now_ms 当前本机毫秒时刻。
 * @return 状态、输入、远程许可和租约均有效时返回1。
 */
uint8_t RobotSafetyIsReadyAt(uint32_t now_ms);
/** @brief 同时检查当前整车许可和计算目标时的安全代际。
 * @param[in] now_ms 本机毫秒时刻，用于检查来源期限。
 * @param[in] expected_generation 目标计算前读取的安全代际。
 * @return 1 表示当前许可有效且代际未变化，否则返回 0。
 * @note 两项检查在同一保护内完成，不提供单个功能的许可。
 */
uint8_t RobotSafetyIsReadyForGeneration(uint32_t now_ms, uint32_t expected_generation);
/** @brief 同一安全代际下的功能及击发许可。
 * @param[in] now_ms 本机毫秒时刻，检查来源期限。
 * @param[in] expected_generation 目标计算前读取的安全代际。
 * @param[out] permits 非空；有效查询时写入当前功能及击发许可，否则保持原值。
 * @param[out] policy_active 非空；1 表示使用分功能策略，0 表示使用旧整车兼容路径。
 * @return 1 表示查询成功；参数错误或代际变化时返回 0，调用者必须禁止输出。
 * @note 旧路径只提供整车许可，不能证明单个功能或击发条件。
 */
uint8_t RobotSafetyGetOutputPermitsForGeneration(uint32_t now_ms, uint32_t expected_generation,
                                                  RobotControlPermits_t *permits, uint8_t *policy_active);
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
