/** @file ins_startup.h
 * @brief 在任务上下文初始化姿态传感器并按车型依赖报告结果。
 */
#ifndef MC02_INS_STARTUP_H
#define MC02_INS_STARTUP_H

/** @brief INS 初始化对当前车型启动的影响。 */
typedef enum
{
    ROBOT_INS_STARTUP_READY = 0, /**< INS 已初始化，可启动周期任务；辅助温控可能降级。 */
    ROBOT_INS_STARTUP_AUXILIARY_FAILURE, /**< INS 失败但不影响已声明的核心模块。 */
    ROBOT_INS_STARTUP_CORE_FAILURE, /**< INS 或其必要温控依赖失败，已请求关断。 */
} RobotInsStartupResult_e;

/** @brief 初始化姿态传感器并检查已启用温控的初始化结果。
 * @return INS 可用、辅助功能失败或核心功能失败；核心失败会请求失能并关断板级输出。
 * @note 仅由唯一启动任务调用，须先完成车型依赖启动和看门狗准备；内部可能等待和标定。
 *       不表示运行数据新鲜，也不标记整车启动完成或创建周期任务。
 */
RobotInsStartupResult_e RobotInitializeIns(void);
#endif
