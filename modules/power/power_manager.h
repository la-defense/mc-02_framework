/** @file power_manager.h
 * @brief 功率管理的预留结构；目前只分配参数并设置开关，反馈和更新均未实现。
 */
#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "general_def.h"
#include "stdint.h"

#define TOQUE_COEFFICIENT_3508 1.9067382e-5f     // 3508电机电流转换扭矩 Nm

#define ERROR_POWERDISTRIBUTE 15.0f
#define PROP_POWERDISTRIBUTE 5.0f

typedef struct
{
    uint8_t rlsEnabled;
    // 预留模型系数，目前只保存、不参与计算；单位和公式待实际功率模型确定。
    float k1, k2, k3;
} PowerManagerInstance;

/* powermanager初始化配置 */
typedef struct
{
    float k1, k2, k3;
} PowerManager_Init_Config_s;

/** @brief 分配对象并复制三个配置系数，不执行功率计算。
 * @param[in] power_manager_config 有效配置地址，不检查 NULL。
 * @return 分配的对象，由模块保存；没有检查分配失败，也没有释放入口，不应重复初始化。
 */
PowerManagerInstance *PowerControlInit(PowerManager_Init_Config_s *power_manager_config);
/** @brief 预留反馈入口，目前为空函数。
 * @param[in] instance 预留对象参数，当前未使用。
 * @param[in] maxPowerLimited 预留功率参数，当前未使用，不能据此认为已实施功率限制。
 */
void PowerManagerFeedback(PowerManagerInstance *instance, float maxPowerLimited);
/** @brief 只将 rlsEnabled 设为 1，当前没有后续估计计算。
 * @param[in,out] instance 有效对象地址，不能为 NULL。
 */
void PowerManagerEnable(PowerManagerInstance *instance);
/** @brief 预留模型更新入口，目前不处理任何输入。
 * @param[in] power 预留功率输入。
 * @param[in] sample_a,sample_b 预留模型输入，当前用途及单位尚未实现。
 */
void PowerManagerUpdate(float power, float sample_a, float sample_b);

#endif // !POWER_CONTROL_H
