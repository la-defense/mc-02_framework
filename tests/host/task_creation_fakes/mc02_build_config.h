/** @file mc02_build_config.h
 * @brief 只用于任务创建测试的车型选择。
 */
#define MC02_FEATURE_COMM_DISPATCH 1 /* 启用通信和电机任务。 */
#define MC02_FEATURE_UI 1 /* 启用裁判图形任务。 */
#define MC02_FEATURE_LCD 1 /* 启用 LCD 任务。 */
#define MC02_FEATURE_HEATER 1 /* 测试温控核心与辅助失败处理。 */
#ifndef MC02_SELECTED_DRIVER_HT
#define MC02_SELECTED_DRIVER_HT 0 /* 默认不启动 HT 独立任务；专项测试单独启用。 */
#endif
