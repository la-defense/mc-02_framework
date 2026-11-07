/** @file module_plan.h
 * @brief 固定容量的模块依赖和启动处理；本文件不依赖芯片或RTOS。
 */
#ifndef MC02_MODULE_PLAN_H
#define MC02_MODULE_PLAN_H
#include <stdint.h>
typedef enum {
    ROBOT_MODULE_SAFETY = 0, ROBOT_MODULE_IMU, ROBOT_MODULE_COMMUNICATION,
    ROBOT_MODULE_COMMAND, ROBOT_MODULE_CHASSIS, ROBOT_MODULE_GIMBAL,
    ROBOT_MODULE_SHOOTER, ROBOT_MODULE_REFEREE, ROBOT_MODULE_VISION,
    ROBOT_MODULE_REMOTE, ROBOT_MODULE_ALARM, ROBOT_MODULE_LCD, ROBOT_MODULE_UI,
    ROBOT_MODULE_SUPERCAP, ROBOT_MODULE_LOG, ROBOT_MODULE_HEATER, ROBOT_MODULE_COUNT
} RobotModule_t;
#define ROBOT_MODULE_BIT(module_) (UINT32_C(1) << (unsigned)(module_))
typedef enum {
    ROBOT_MODULE_OK = 0, ROBOT_MODULE_ARGUMENT, ROBOT_MODULE_DEPENDENCY,
    ROBOT_MODULE_CYCLE, ROBOT_MODULE_CORE_FAILURE
} RobotModuleStatus_t;
typedef struct {
    uint32_t enabled;   ///< 启用模块位图，位号对应 RobotModule_t 的枚举值。
    uint32_t required;  ///< 必需模块位图，必须是 enabled 的子集。
    uint32_t dependencies[ROBOT_MODULE_COUNT]; ///< 下标是模块枚举值，元素是其前置模块位图。
} RobotModulePlan_t;
typedef struct {
    uint32_t started; /**< 初始化或异步启动已接受，不表示传感器数据已经可信。 */
    uint32_t failed; /**< 启动或运行失败的模块位图，位号对应 RobotModule_t。 */
    uint32_t blocked; /**< 因依赖失败而未启动的模块位图，位号对应 RobotModule_t。 */
    uint8_t core_failed; /**< 非零表示核心功能或其必需依赖已失败。 */
    uint8_t failed_module; /**< 首个导致核心失败的模块编号；无核心失败时为 ROBOT_MODULE_COUNT。 */
} RobotModuleStartup_t;
/** @brief 初始化一个模块；只允许启动计划中启用的模块。
 * @param[in] context 原样传给回调的上下文，可为NULL。
 * @param[in] module 本次需要启动的模块编号，范围为有效模块枚举值。
 * @return 非零表示模块已接受启动，0表示启动失败。
 */
typedef uint8_t (*RobotModuleStartFn_t)(void *context, unsigned module);
/** @brief 两阶段启动之间执行资源准备，并报告受影响模块。
 * @param[in] context 原样传给回调的上下文。
 * @param[in] prior_state 第一阶段已完成的模块状态，只读且仅在回调期间有效。
 * @param[out] failed_modules 失败资源影响的模块位图，须只包含延后模块。
 * @return 成功返回非零；不能取得可信结果返回 0。
 */
typedef uint8_t (*RobotModuleBarrierFn_t)(void *context, const RobotModuleStartup_t *prior_state,
                                          uint32_t *failed_modules);
/** @brief 检查启用项、必需项、依赖完整性和循环。
 * @param[in] plan 启动声明，不可为NULL。
 * @param[out] module 非NULL时接收错误模块，成功时为ROBOT_MODULE_COUNT。
 * @return OK、ARGUMENT、DEPENDENCY或CYCLE；不修改plan。
 */
RobotModuleStatus_t RobotModulePlanValidate(const RobotModulePlan_t *plan, unsigned *module);
/** @brief 依赖顺序启动启用模块，核心失败立即停止，辅助失败跳过依赖它的模块。
 * @param[in] plan 已声明的固定模块集合，不可为NULL。
 * @param[in] start 普通同步启动入口，不可为NULL；依赖已接受后才调用。
 * @param[in] context 原样交给start，允许NULL。
 * @param[out] state 启动结果，不可为NULL；调用先清零。失能由上层处理。
 * @return OK表示所有核心启动已接受，辅助失败仍保存在state；否则返回具体错误。
 */
RobotModuleStatus_t RobotModuleStart(const RobotModulePlan_t *plan,
    RobotModuleStartFn_t start, void *context, RobotModuleStartup_t *state);
/** @brief 先启动独立模块，再执行阶段准备，最后启动受其影响的模块及下游依赖。
 * @param[in] plan 已声明的模块计划。
 * @param[in] deferred 需等阶段准备完成的模块位图，必须属于启用集合。
 * @param[in] barrier 阶段准备回调，不可为 NULL；只调用一次。
 * @param[in] start 单个模块启动回调，不可为 NULL。
 * @param[in] context 同时传给两个回调的上下文。
 * @param[out] state 启动和失败结果，不可为 NULL。
 * @return OK、ARGUMENT、依赖错误或 CORE_FAILURE；核心失败时停止后续步骤。
 */
RobotModuleStatus_t RobotModuleStartPhased(const RobotModulePlan_t *plan, uint32_t deferred,
    RobotModuleBarrierFn_t barrier, RobotModuleStartFn_t start, void *context,
    RobotModuleStartup_t *state);
/** @brief 记录已启动模块后续失败，并阻止它的下游模块继续运行。
 * @param[in] plan 已验证的固定模块计划。
 * @param[in,out] state 当前启动结果。
 * @param[in] module 发生失败的已启用模块。
 * @return 失败仅影响辅助功能时返回OK；影响核心闭包时返回CORE_FAILURE。
 */
RobotModuleStatus_t RobotModuleStartupMarkFailure(const RobotModulePlan_t *plan,
    RobotModuleStartup_t *state, RobotModule_t module);
/** @brief 根据启用集合计算所有核心依赖。
 * @param[in] plan 已通过Validate的声明；NULL返回0。
 * @return 核心模块及其全部必要依赖位图，不改变输入。
 */
uint32_t RobotModuleCoreClosure(const RobotModulePlan_t *plan);
#endif
