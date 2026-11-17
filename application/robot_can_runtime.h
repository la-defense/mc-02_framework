/** @file robot_can_runtime.h
 * @brief 将车型核心模块依赖的 CAN 总线故障送入整车安全管理。
 */
#ifndef MC02_ROBOT_CAN_RUNTIME_H
#define MC02_ROBOT_CAN_RUNTIME_H
#include "robot_can_capacity.h"

/** @brief 已安装的核心总线集合和从启动以来观察到的故障事件。 */
typedef struct {
    uint8_t core_bus_mask; /**< 位 0～2 对应核心逻辑总线 1～3。 */
    uint8_t fault_bus_mask; /**< 从安装至今发生过故障的核心总线；不代表当前硬件状态。 */
    uint32_t fault_generation; /**< 任一核心总线检测到故障时递增，允许 32 位回绕。 */
    uint32_t bus_generation[3]; /**< 各核心总线各自的故障事件代次。 */
} RobotCanFaultSnapshot_t;

/** @brief 一次读取核心总线及故障事件代次。
 * @param[out] output 接收完整结果，不能为空；未安装或参数错误时保持原值。
 * @return 1 为成功，0 为观察者未安装或参数无效。
 * @note 该信息只记录已检测到的事件；同一故障随后触发隔离时可能再次推进代次。
 * 不代表总线已恢复，也不允许清除故障。
 */
uint8_t RobotCanRuntimeGetFaultSnapshot(RobotCanFaultSnapshot_t *output);

/** @brief 一轮查询所需总线的控制器状态和故障代次。 */
typedef struct {
    uint8_t core_bus_mask; /**< 车型声明的全部核心 CAN 总线。 */
    uint8_t ready_bus_mask; /**< 本轮确认控制器正常的核心总线；查询期间出现故障时清零。 */
    uint32_t fault_generation; /**< 本轮结束时的核心故障事件代次。 */
} RobotCanReadiness_t;

/** @brief 检查核心总线是否能参与后续恢复条件计算。
 * @param[out] output 成功时接收完整结果；空指针或观察者未安装时保持原值。
 * @return 1 为查询完成，0 为参数错误或观察者未安装；调用者须比较两个总线位图。
 * @note 不检查设备反馈、授权或新命令，不清除故障；最终放行前仍须再次核对故障代次。
 */
uint8_t RobotCanRuntimeGetReadiness(RobotCanReadiness_t *output);

/** @brief 在同一短中断保护中复核总线故障代次及安全资格，再尝试清除 CAN 故障。
 * @param[in] expected_fault_generation 反馈及命令恢复检查使用的 CAN 故障代次。
 * @param[in] expected_safety_generation 同轮读取的安全代次。
 * @param[in] now_ms 调用方读取的本机时间；安全管理重新读取实际检查时刻。
 * @return 1 为已清除 CAN 故障位；状态变化、资格失效或未安装时返回 0。
 * @note 仅供完成总线、逐台反馈连续有效和恢复后新命令检查的协调者调用；
 * 此函数不查询 HAL 或反馈，不推进 READY，也不替代现场验收。
 */
uint8_t RobotCanRuntimeTryClearFault(uint32_t expected_fault_generation,
                                     uint32_t expected_safety_generation, uint32_t now_ms);

/** @brief 根据启动计划安装核心总线故障观察者。
 * @param[in] plan 已声明的模块计划，不可为 NULL。
 * @param[in] demands 端点需求数组；count 为零时可为 NULL。
 * @param[in] count 需求数量，最多为 8。
 * @return 1 为配置成功或没有核心 CAN 需求；0 为输入无效、总线无绑定或观察者安装失败。
 * @note 启动阶段调用一次；已报告的故障不会因控制器重启而自动清除。
 */
uint8_t RobotCanRuntimeInstall(const RobotModulePlan_t *plan,
                               const RobotCanFilterDemand_t *demands, uint8_t count);
#endif
