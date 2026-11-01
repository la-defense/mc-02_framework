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
    uint32_t enabled, required;
    uint32_t dependencies[ROBOT_MODULE_COUNT];
} RobotModulePlan_t;
typedef struct {
    uint32_t started; /**< 初始化或异步启动已接受，不表示传感器数据已经可信。 */
    uint32_t failed, blocked;
    uint8_t core_failed, failed_module;
} RobotModuleStartup_t;
/** @brief 初始化指定模块；返回非0表示启动已接受，0表示失败；不得启动未指定模块。 */
typedef uint8_t (*RobotModuleStartFn_t)(void *context, unsigned module);
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
/** @brief 根据启用集合计算所有核心依赖。
 * @param[in] plan 已通过Validate的声明；NULL返回0。
 * @return 核心模块及其全部必要依赖位图，不改变输入。
 */
uint32_t RobotModuleCoreClosure(const RobotModulePlan_t *plan);
#endif
