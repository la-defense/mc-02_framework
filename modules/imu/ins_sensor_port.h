/** @file ins_sensor_port.h
 * @brief INS 使用的完整 IMU 样本入口，不暴露总线或芯片句柄。
 */
#ifndef MC02_INS_SENSOR_PORT_H
#define MC02_INS_SENSOR_PORT_H
#include "bmi088_data.h"
#include <stdint.h>

/** @brief 平台持有的传感器对象，业务只保存指针。 */
typedef void InsSensor;

/** @brief 注册当前板级 IMU；启动阶段串行调用。
 * @return 成功返回长期有效的对象，失败返回 NULL。
 * @note 当前 MC-02 后端使用板载 BMI088、周期读取和原有启动标定配置。
 */
InsSensor *InsSensorRegister(void);

/** @brief 读取一组加速度、角速度和温度。
 * @param[in] sensor 成功注册的对象。
 * @param[out] sample 成功时写入完整样本，失败不承诺修改结果。
 * @return 1 为完整读取成功，0 为读取失败。
 */
uint8_t InsSensorAcquire(InsSensor *sensor, BMI088_Data_t *sample);

/** @brief 执行一次当前传感器的标定服务。
 * @return 非零表示本轮处理了标定请求，0 表示没有请求；结果需另查传感器状态。
 */
uint8_t InsSensorCalibrationService(void);

/** @brief 读取最近一次初始化错误和重试次数。
 * @param[out] error 错误记录，允许为 NULL。
 * @param[out] retries 重试次数，允许为 NULL。
 */
void InsSensorGetInitDiagnostics(uint32_t *error, uint32_t *retries);
#endif
