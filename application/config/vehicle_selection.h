/** @file vehicle_selection.h
 * @brief 只读构建身份和模块启动结果；芯片类型不出现在接口中。
 */
#ifndef MC02_VEHICLE_SELECTION_H
#define MC02_VEHICLE_SELECTION_H
#include "module_plan.h"
typedef struct {
    uint32_t layout, version;
    char vehicle[16], platform[16], board[16], rtos[16], role[16];
    uint32_t enabled_modules, required_modules;
} RobotBuildIdentity_t;
extern const RobotBuildIdentity_t robot_build_identity;
extern RobotModuleStartup_t robot_module_startup;
/** @brief 取得由车型声明生成的依赖表；始终返回长期只读对象。 */
const RobotModulePlan_t *RobotBuildGetPlan(void);
/** @brief 按依赖启动本板启用模块，保存可查询的结果。
 * @param[in] start 业务启动函数，不可为NULL。
 * @param[in] context 原样交给start，允许NULL。
 * @return 同RobotModuleStart；只允许启动阶段调用一次。
 */
RobotModuleStatus_t RobotBuildStart(RobotModuleStartFn_t start, void *context);
/** @brief 查询模块启动是否已接受；越界、禁用或已失败时返回0。
 * @param[in] module 模块编号。
 * @return 非0只表示启动已接受；运行数据有效性由模块自身报告。
 */
uint8_t RobotBuildModuleStarted(RobotModule_t module);
/** @brief 任务创建或异步初始化失败时撤销模块及其下游启动结果。
 * @param[in] module 已启用模块；越界或未启用时不修改状态。
 * @return 非0表示失败影响核心，上层必须关断并锁存初始化失败。
 * @note 普通任务调用；内部保护共享状态，不自动清故障或重试。
 */
uint8_t RobotBuildMarkFailure(RobotModule_t module);
#endif
