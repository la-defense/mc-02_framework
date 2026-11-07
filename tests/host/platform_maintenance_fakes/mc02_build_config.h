/** @file mc02_build_config.h
 * @brief 由测试目标传入 ADC 选择，模拟生成配置的接口。
 */
#ifndef TEST_MAINTENANCE_BUILD_CONFIG_H
#define TEST_MAINTENANCE_BUILD_CONFIG_H
#ifndef MC02_FEATURE_ADC
#error "测试必须声明 ADC 选择"
#endif
#endif
