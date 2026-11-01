/** @file build_selection.h
 * @brief 为车型启动测试提供固定的核心和辅助模块选择。
 */
#ifndef MC02_PLATFORM_IRQ_TEST_SELECTION_H
#define MC02_PLATFORM_IRQ_TEST_SELECTION_H
#define MC02_VEHICLE_CONFIG_VERSION 1u // 测试构建的配置版本。
#define MC02_VEHICLE_NAME "test" // 测试车型标识。
#define MC02_PLATFORM_NAME "test" // 测试芯片平台标识。
#define MC02_BOARD_NAME "test" // 测试板卡标识。
#define MC02_RTOS_NAME "test" // 测试调度器标识。
#define MC02_BUILD_ROLE_NAME "test" // 测试板角色标识。
#define MC02_FEATURE_MASK (ROBOT_MODULE_BIT(ROBOT_MODULE_SAFETY) | ROBOT_MODULE_BIT(ROBOT_MODULE_LCD)) // 启用安全和显示。
#define MC02_REQUIRED_MASK ROBOT_MODULE_BIT(ROBOT_MODULE_SAFETY) // 安全模块是核心。
#define MC02_DEPENDENCY_INITIALIZER {0u} // 测试两个模块没有前置依赖。
#endif
