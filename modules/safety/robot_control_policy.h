/** @file robot_control_policy.h
 * @brief 根据同一批来源事实计算三模式的功能许可；调用方负责一致读取和立即禁止。
 */
#ifndef ROBOT_CONTROL_POLICY_H
#define ROBOT_CONTROL_POLICY_H
#include <stdint.h>

typedef enum {
    ROBOT_CONTROL_MANUAL = 0, // 人工给出底盘、枪口和击发请求。
    ROBOT_CONTROL_ASSIST, // 人工行驶和击发，视觉可辅助瞄准。
    ROBOT_CONTROL_AUTONOMOUS // 按车型资格和有效比赛阶段自主运行。
} RobotControlMode_t;

typedef enum {
    ROBOT_CONTROL_CHASSIS = 1u << 0, // 底盘运动许可。
    ROBOT_CONTROL_GIMBAL = 1u << 1, // 云台运动许可。
    ROBOT_CONTROL_SHOOTER = 1u << 2, // 发射机构运行许可，击发仍另检查。
    ROBOT_CONTROL_ALL = 7u
} RobotControlFunction_t;

typedef enum {
    ROBOT_CONTROL_SOURCE_NONE = 0, // 禁止该功能或无有效来源。
    ROBOT_CONTROL_SOURCE_MANUAL, // 已选择的人工连续输入。
    ROBOT_CONTROL_SOURCE_VISION, // 有效视觉瞄准或视觉击发请求。
    ROBOT_CONTROL_SOURCE_NAVIGATION, // 独立有效的导航移动请求。
    ROBOT_CONTROL_SOURCE_LOCAL // 下位机本地自旋或三轴索敌。
} RobotControlSource_t;

typedef struct {
    uint32_t source_time_ms; // 已映射到本机的原始时刻，转发不能重写。
    uint32_t max_age_ms; // 1～INT32_MAX ms；等于期限允许，valid为0时不使用。
    uint8_t valid; // 仅表示来源数据已校验，实际期限由本策略再次检查。
} RobotControlLease_t;

typedef struct {
    uint32_t enabled_functions; // 三个功能位，0表示纯观察，未知位拒绝。
    uint8_t configured; // 只有车型配置完整且校验通过后置1。
    uint8_t formal_autonomous; // 是否允许正式自主资格，不代替比赛阶段条件。
    uint8_t commissioning; // 受限调试：仅纯手动，发射功能不得启用。
} RobotControlPolicy_t;

typedef struct {
    RobotControlMode_t mode; // 由唯一命令仲裁者选择，本函数不解释遥控开关。
    RobotControlLease_t manual; // 当前已选择的人工来源，键鼠与摇杆不能混用期限。
    RobotControlLease_t aim; // 瞄准来源，与导航分别计时。
    RobotControlLease_t navigation; // 导航来源，更新不替瞄准续期。
    RobotControlLease_t match; // 裁判比赛阶段的独立来源时间。
    uint32_t feedback_valid_functions; // 对应功能全部必需反馈仍有效时置位。
    uint32_t blocked_functions; // 局部故障影响的功能位，由safety依赖关系计算。
    uint8_t core_ready; // 统一安全管理检查的核心条件，不能由通信在线直接代替。
    uint8_t immediate_inhibit; // 急停、致命初始化等立即禁止条件。
    uint8_t manual_enabled; // 人工使能请求；健康模式切换保留该请求。
    uint8_t match_running; // 裁判确认处于允许运行阶段，还须match未过期。
    uint8_t manual_fire_requested; // 当前人工击发请求，与manual同期限。
    uint8_t vision_fire_requested; // 当前视觉击发请求，与aim同期限。
} RobotControlFacts_t;

typedef struct {
    uint32_t functions; // 经过全局条件、局部故障及必需反馈检查后的功能位。
    RobotControlSource_t chassis_source; // 人工、导航或本地自旋。
    RobotControlSource_t gimbal_source; // 人工、视觉或本地索敌。
    RobotControlSource_t fire_source; // 只标识当前可执行的击发来源，无请求则NONE。
    uint8_t fire_allowed; // 还需发射模块执行摩擦轮就绪、裁判热量等检查。
} RobotControlPermits_t;

typedef enum {
    ROBOT_CONTROL_OK = 0, // 已交付结果；functions为0表示当前不允许动作。
    ROBOT_CONTROL_BAD_ARGUMENT,
    ROBOT_CONTROL_BAD_CONFIG,
    ROBOT_CONTROL_BAD_FACTS
} RobotControlResult_t;

/** @brief 计算三模式来源和分功能许可；不修改配置、事实或任何设备。
 * @param[in] policy 本车型策略；调试策略只允许人工且不得声明发射。
 * @param[in] facts 同次一致读取的来源与安全条件，由统一安全管理提供。
 * @param[in] now_ms 本机毫秒时刻，允许32位回绕。
 * @param[out] output 仅在OK时完整改写；错误时保留原值，调用者必须禁止执行。
 * @return OK包含“当前全部禁止”的有效结果；其他返回表示输入或配置错误。
 * @note 不保存状态，不实现平滑接管或物理关断；发布者和执行路径仍须复核立即禁止。
 */
RobotControlResult_t RobotControlEvaluate(const RobotControlPolicy_t *policy,
                                          const RobotControlFacts_t *facts,
                                          uint32_t now_ms, RobotControlPermits_t *output);
#endif
