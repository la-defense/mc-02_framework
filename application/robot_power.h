/** @file robot_power.h
 * @brief 按车型声明执行受控电源阶段，并报告受影响模块。
 */
#ifndef MC02_ROBOT_POWER_H
#define MC02_ROBOT_POWER_H
#include <stdint.h>
#include "config/module_plan.h"
#include "platform_power.h"

typedef enum {
    ROBOT_POWER_OFF = 0, /**< 此板不使用该路电源。 */
    ROBOT_POWER_BEFORE_DEVICES = 1, /**< 在业务模块初始化前开启。 */
    ROBOT_POWER_AFTER_DEVICES = 2 /**< 独立模块初始化后、依赖模块初始化前开启。 */
} RobotPowerStage_t;
typedef struct {
    RobotPowerStage_t stage; /**< 本板开启阶段。 */
    uint32_t modules; /**< 使用此电源的已启用模块位图。 */
} RobotPowerRailConfig_t;

/** @brief 校验完整计划并执行设备前电源阶段。
 * @param[in] rails 固定三项电源声明，不可为 NULL。
 * @param[in] modules 已选择的模块依赖计划，不可为 NULL。
 * @return 配置有效且设备前核心电源成功返回 1；否则返回 0。
 * @note 只可由启动协调者调用一次；裸板后端拒绝任何开启请求。
 */
uint8_t RobotPowerBegin(const RobotPowerRailConfig_t rails[PLATFORM_POWER_COUNT],
                        const RobotModulePlan_t *modules);
/** @brief 查询必须延迟到设备后电源准备完成的模块。
 * @return 直接使用后阶段电源的模块位图；未初始化返回 0。
 */
uint32_t RobotPowerDeferredModules(void);
/** @brief 两阶段模块启动使用的设备后电源回调。
 * @param[in] context 未使用，允许 NULL。
 * @param[in] prior_state 第一阶段的模块结果，不可为 NULL。
 * @param[out] failed_modules 本阶段电源失败影响的模块位图，不可为 NULL。
 * @return 已执行阶段返回 1；参数错误、未初始化或重复调用返回 0。
 */
uint8_t RobotPowerAfterDevices(void *context, const RobotModuleStartup_t *prior_state,
                               uint32_t *failed_modules);
/** @brief 查询模块所需电源是否全部已准备。
 * @param[in] module 车型模块编号。
 * @return 所需电源均已准备返回 1；无效编号或未准备返回 0。
 */
uint8_t RobotPowerModuleReady(unsigned module);
#endif
