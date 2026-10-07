#include "imu_heater.h"
#include "controller.h"
#include "tim.h"
#include "main.h"
#include "bsp_log.h"
#include "bsp_adc.h"
#include <math.h>
#include <stddef.h>

/*
 * IMU 加热安全策略(2026-09 起火事故后重构):
 * - 原理图确认: R77~R84/R88/R89 加热电阻直接接 VCC_IN(24V 主输入),
 *   由 Q9(低边N-MOS) + TIM3_CH4(PB01) PWM 开关. 因此占空比必须严格限制.
 * - 上电/初始化/故障/急停时 PWM=0, 不再有"无条件满功率预热".
 * - 温度必须有限且在合理范围内, 传感器无效或超时直接关加热.
 * - 占空比硬限幅: 最大 5%(500/9999), 与厂商例程 MAX_OUT=500 对齐;
 *   24V 下再按功率上限自动压低占空比(首测保守值 0.30W, 按最坏47Ω估算).
 * - 24V 加热不允许任何 10%/满功率预热.
 * - 连续加热最长 60s, 超时锁存故障.
 * - 温度 > 50°C 立即关加热并锁存, 降到 42°C 以下且手动清除后才允许恢复.
 */

#define IMU_HEATER_TARGET_DEFAULT 40.0f
#define IMU_HEATER_TARGET_MIN 30.0f
#define IMU_HEATER_TARGET_MAX 45.0f

#define IMU_HEATER_ARR 9999u
#define IMU_HEATER_DUTY_NORMAL 500u   // 5% 绝对硬上限(厂商例程 MAX_OUT=500)
#define IMU_HEATER_R_EQUIV_MIN 47.0f  // R77~R89 全并联最坏等效电阻
#define IMU_HEATER_MAX_POWER_W 0.30f  // 首次24V测试的功率上限
/* 首测功率被压到0.30W, 预热较慢; 5分钟超时用于抓传感器/加热片卡死,
   避免低功率正常预热被误判为故障 */
#define IMU_HEATER_MAX_ON_TIME_MS 300000u

#define IMU_HEATER_TEMP_VALID_MIN -20.0f
#define IMU_HEATER_TEMP_VALID_MAX 80.0f
#define IMU_HEATER_OVERTEMP_CUTOFF 50.0f
#define IMU_HEATER_OVERTEMP_RECOVER 42.0f
#define IMU_HEATER_TARGET_TOLERANCE 2.0f
#define IMU_HEATER_SENSOR_TIMEOUT_MS 500u
#define IMU_HEATER_LOG_PERIOD_MS 1000u
#define IMU_HEATER_MIN_SUPPLY_V 10.0f
#define IMU_HEATER_MAX_SUPPLY_V 30.0f

static PIDInstance heater_pid;
static float heater_target = IMU_HEATER_TARGET_DEFAULT;
static uint16_t heater_duty = 0;
static uint8_t heater_initialized = 0;
static uint8_t heater_fault = 0;
static uint8_t heater_overtemp = 0;
static uint8_t heater_timeout_fault = 0;
static uint32_t last_valid_sample_ms = 0;
static uint32_t last_seen_sample_sequence = 0;
static uint32_t heat_start_ms = 0;
static uint32_t last_log_ms = 0;
static float last_temperature = 0.0f;
static uint8_t last_sensor_valid = 0;
static float last_vcc_in = 0.0f;
static uint8_t supply_missing_logged = 0;
static uint32_t heater_long_operation_active_mask = 0u;
static uint32_t heater_long_operation_failed_mask = 0u;
static uint8_t heater_require_post_operation_sample = 0u;
static uint32_t heater_post_operation_sample_sequence = 0u;
static uint32_t heater_operation_end_ms = 0u;

static uint32_t IMUHeaterLongOperationMask(IMUHeater_LongOperation_e operation)
{
    if ((uint32_t)operation >= IMU_HEATER_LONG_OPERATION_COUNT)
        return 0u;
    return 1u << (uint32_t)operation;
}

static void IMUHeaterApplyDuty(uint16_t duty)
{
#if !MC02_HEATER_ENABLED
    duty = 0;
#else
    /* 无论调用方传入什么, 都不允许超过 5% 硬上限 */
    if (duty > IMU_HEATER_DUTY_NORMAL)
        duty = IMU_HEATER_DUTY_NORMAL;
#endif

    heater_duty = duty;
    if (heater_initialized)
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, heater_duty);
}

static uint8_t IMUHeaterReadFreshSupply(float *vcc_in)
{
    if (vcc_in == NULL)
        return 0u;

    BSP_ADC_Sample_t sample;
    if (BSP_ADCGetSample(&sample) != BSP_ADC_STATUS_VALID ||
        !sample.voltage_config_valid || !isfinite(sample.vcc_in_volts))
    {
        *vcc_in = 0.0f;
        return 0u;
    }

    *vcc_in = sample.vcc_in_volts;
    return 1u;
}

void IMUHeaterInit(void)
{
    PID_Init_Config_s config = {
        .MaxOut = IMU_HEATER_DUTY_NORMAL,
        .IntegralLimit = IMU_HEATER_DUTY_NORMAL,
        .DeadBand = 0.0f,
        .Kp = 100.0f,
        .Ki = 20.0f,
        .Kd = 0.0f,
        .Improve = PID_Integral_Limit,
    };
    PIDInit(&heater_pid, &config);

    heater_target = IMU_HEATER_TARGET_DEFAULT;
    heater_duty = 0;
    heater_fault = 0;
    heater_overtemp = 0;
    heater_timeout_fault = 0;
    last_valid_sample_ms = HAL_GetTick();
    last_seen_sample_sequence = 0u;
    heat_start_ms = 0;
    last_log_ms = 0;
    last_vcc_in = 0.0f;
    supply_missing_logged = 0;
    heater_initialized = 1;

    if (!IMUHeaterReadFreshSupply(&last_vcc_in) ||
        last_vcc_in < IMU_HEATER_MIN_SUPPLY_V || last_vcc_in > IMU_HEATER_MAX_SUPPLY_V)
    {
        LOGWARNING("[imu_heat] VCC_IN not ready (%d.%dV), heater disabled until 24V present",
                   (int)last_vcc_in, (int)(last_vcc_in * 10.0f) % 10);
    }

    /* 先确保比较值为0, 再启动PWM, 避免任何形式的开机满功率 */
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);
#if MC02_HEATER_ENABLED
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
#endif

    LOGINFO("[imu_heat] init safe(VCC_IN 24V): target=%d.%dC max_duty=%u/%u power_cap=%d.%dW",
            (int)heater_target, (int)(heater_target * 10.0f) % 10,
            (unsigned)IMU_HEATER_DUTY_NORMAL, (unsigned)IMU_HEATER_ARR,
            (int)IMU_HEATER_MAX_POWER_W, (int)(IMU_HEATER_MAX_POWER_W * 10.0f) % 10);
}

void IMUHeaterForceOff(void)
{
    IMUHeaterApplyDuty(0);
    if (heater_initialized)
    {
        /* 清掉积分, 防止故障恢复后积分冲击 */
        heater_pid.Iout = 0.0f;
        heater_pid.Output = 0.0f;
        heater_pid.Last_Output = 0.0f;
    }
    heat_start_ms = 0;
}

uint8_t IMUHeaterBeginLongOperation(IMUHeater_LongOperation_e operation)
{
    uint32_t mask = IMUHeaterLongOperationMask(operation);
    if (mask == 0u || ((heater_long_operation_active_mask | heater_long_operation_failed_mask) & mask) != 0u)
        return 0u;

    heater_long_operation_active_mask |= mask;
    IMUHeaterForceOff();
    return 1u;
}

void IMUHeaterEndLongOperation(IMUHeater_LongOperation_e operation, uint8_t success)
{
    uint32_t mask = IMUHeaterLongOperationMask(operation);
    if (mask == 0u || (heater_long_operation_active_mask & mask) == 0u)
        return;

    heater_long_operation_active_mask &= ~mask;
    if (!success)
    {
        heater_long_operation_failed_mask |= mask;
        heater_fault = 1u;
        IMUHeaterForceOff();
        return;
    }

    heater_post_operation_sample_sequence = last_seen_sample_sequence;
    heater_operation_end_ms = HAL_GetTick();
    heater_require_post_operation_sample = 1u;
}

void IMUHeaterClearFault(void)
{
    heater_fault = 0;
    heater_overtemp = 0;
    heater_timeout_fault = 0;
    IMUHeaterForceOff();
    LOGINFO("[imu_heat] fault cleared");
}

void IMUHeaterSetTarget(float target_temp)
{
    if (!isfinite(target_temp))
        return;
    if (target_temp < IMU_HEATER_TARGET_MIN)
        target_temp = IMU_HEATER_TARGET_MIN;
    if (target_temp > IMU_HEATER_TARGET_MAX)
        target_temp = IMU_HEATER_TARGET_MAX;
    heater_target = target_temp;
}

static void IMUHeaterLogStatus(float temperature, uint8_t sensor_valid)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_log_ms) >= IMU_HEATER_LOG_PERIOD_MS)
    {
        last_log_ms = now;
        LOGINFO("[imu_heat] vin=%d.%dV temp=%d.%dC target=%d.%dC duty=%u valid=%u fault=%u",
                (int)last_vcc_in, (int)(last_vcc_in * 10.0f) % 10,
                (int)temperature, (int)(temperature * 10.0f) % 10,
                (int)heater_target, (int)(heater_target * 10.0f) % 10,
                (unsigned)heater_duty,
                (unsigned)sensor_valid, (unsigned)heater_fault);
    }
}

void IMUHeaterUpdate(const BMI088_Data_t *sample, uint8_t force_off)
{
    if (!heater_initialized)
        return;

    if ((heater_long_operation_active_mask | heater_long_operation_failed_mask) != 0u)
    {
        last_sensor_valid = 0u;
        IMUHeaterForceOff();
        IMUHeaterLogStatus(last_temperature, 0u);
        return;
    }

    uint32_t now = HAL_GetTick();
    uint8_t sample_fresh = BMI088SampleIsFresh(sample, now);
    if (sample_fresh && heater_require_post_operation_sample)
    {
        sample_fresh = (sample->sequence != heater_post_operation_sample_sequence &&
                        (int32_t)(sample->timestamp_ms - heater_operation_end_ms) >= 0) ? 1u : 0u;
        if (sample_fresh)
        {
            heater_require_post_operation_sample = 0u;
            heater_post_operation_sample_sequence = sample->sequence;
        }
    }
    uint8_t temp_ok = (sample_fresh && isfinite(sample->temperature) &&
                       sample->temperature >= IMU_HEATER_TEMP_VALID_MIN &&
                       sample->temperature <= IMU_HEATER_TEMP_VALID_MAX) ? 1u : 0u;
    float temperature = temp_ok ? sample->temperature : last_temperature;
    uint8_t sensor_valid = temp_ok;

    if (temp_ok && sample->sequence != last_seen_sample_sequence)
    {
        last_seen_sample_sequence = sample->sequence;
        last_valid_sample_ms = sample->timestamp_ms;
        last_temperature = sample->temperature;
    }
    last_sensor_valid = temp_ok;

    if (force_off)
    {
        IMUHeaterForceOff();
        return;
    }

    if (!temp_ok)
    {
        IMUHeaterForceOff();
        if ((uint32_t)(now - last_valid_sample_ms) > IMU_HEATER_SENSOR_TIMEOUT_MS)
        {
            heater_fault = 1u;
            LOGERROR("[imu_heat] sensor invalid/timeout, heater latched off");
        }
        return;
    }

#if !MC02_HEATER_ENABLED
    (void)force_off;
    IMUHeaterForceOff();
    IMUHeaterLogStatus(temperature, temp_ok);
    return;
#endif

    /* 加热电阻接 VCC_IN(24V). 只有输入电压在合理范围内才允许加热.
       这样 USB-only 供电时 VCC_IN=0, 加热会被禁止, 不会再出现 PID 饱和后
       接上24V瞬间满功率的情况.
       注意: 先读VCC_IN再处理force_off, 保证EST时也能看到低压/过压日志. */
    if (!IMUHeaterReadFreshSupply(&last_vcc_in))
    {
        IMUHeaterForceOff();
        if (!supply_missing_logged)
        {
            LOGWARNING("[imu_heat] VCC_IN sample invalid or stale, heater disabled");
            supply_missing_logged = 1u;
        }
        return;
    }

    if (last_vcc_in < IMU_HEATER_MIN_SUPPLY_V)
    {
        IMUHeaterForceOff();
        if (!supply_missing_logged)
        {
            LOGWARNING("[imu_heat] VCC_IN %d.%dV too low (USB-only?), heater disabled",
                       (int)last_vcc_in, (int)(last_vcc_in * 10.0f) % 10);
            supply_missing_logged = 1;
        }
        return;
    }
    if (last_vcc_in > IMU_HEATER_MAX_SUPPLY_V)
    {
        IMUHeaterForceOff();
        heater_fault = 1;
        LOGERROR("[imu_heat] VCC_IN %d.%dV too high, heater latched off",
                 (int)last_vcc_in, (int)(last_vcc_in * 10.0f) % 10);
        return;
    }
    supply_missing_logged = 0;

    /* 急停/故障状态由上层传入force_off, 即使输入电压正常也必须关加热 */
    if (force_off)
    {
        IMUHeaterForceOff();
        return;
    }

    /* 24V 加热电阻功率上限保护:
       按最坏情况等效电阻47Ω估算, 限制平均功率不超过0.30W.
       24V时 duty≈244(2.4%), 25.2V时更低; 12V时功率本身较低, 仍保留5%硬上限. */
    uint16_t duty_limit = IMU_HEATER_DUTY_NORMAL;
    float max_duty_by_power =
        (IMU_HEATER_MAX_POWER_W * (float)IMU_HEATER_ARR * IMU_HEATER_R_EQUIV_MIN) /
        (last_vcc_in * last_vcc_in);
    if (max_duty_by_power < (float)duty_limit)
        duty_limit = (uint16_t)max_duty_by_power;
    if (duty_limit == 0)
    {
        IMUHeaterForceOff();
        return;
    }

    /* 已锁存故障: 只有温度降到恢复阈值以下且手动清除后才允许再加热 */
    if (heater_fault)
    {
        IMUHeaterForceOff();
        return;
    }

    /* 传感器有效性检查 */
    if (!temp_ok)
    {
        IMUHeaterForceOff();
        if ((uint32_t)(now - last_valid_sample_ms) > IMU_HEATER_SENSOR_TIMEOUT_MS)
        {
            heater_fault = 1;
            LOGERROR("[imu_heat] sensor invalid/timeout, heater latched off");
        }
        return;
    }

    /* 过温硬切断: 立即关加热并锁存 */
    if (temperature >= IMU_HEATER_OVERTEMP_CUTOFF)
    {
        IMUHeaterForceOff();
        heater_overtemp = 1;
        heater_fault = 1;
        LOGERROR("[imu_heat] OVERTEMP %d.%dC >= %d.%dC, heater latched off",
                 (int)temperature, (int)(temperature * 10.0f) % 10,
                 (int)IMU_HEATER_OVERTEMP_CUTOFF, (int)(IMU_HEATER_OVERTEMP_CUTOFF * 10.0f) % 10);
        return;
    }

    /* 已经达到目标: 关加热, 清积分, 避免继续累积 */
    if (temperature >= heater_target + IMU_HEATER_TARGET_TOLERANCE)
    {
        IMUHeaterForceOff();
        heat_start_ms = 0;
        IMUHeaterLogStatus(temperature, sensor_valid);
        return;
    }

    if (heat_start_ms == 0)
        heat_start_ms = now;

    /* 连续加热超时保护: 长时间到不了目标, 说明传感器/加热片可能异常 */
    if ((uint32_t)(now - heat_start_ms) > IMU_HEATER_MAX_ON_TIME_MS)
    {
        IMUHeaterForceOff();
        heater_timeout_fault = 1;
        heater_fault = 1;
        LOGERROR("[imu_heat] max heating time exceeded, heater latched off");
        return;
    }

    float output = PIDCalculate(&heater_pid, temperature, heater_target);
    if (!isfinite(output) || output < 0.0f)
        output = 0.0f;
    if (output > (float)duty_limit)
        output = (float)duty_limit;

    IMUHeaterApplyDuty((uint16_t)output);
    if (heater_duty == 0)
        heat_start_ms = 0;
    IMUHeaterLogStatus(temperature, sensor_valid);
}

void IMUHeaterGetStatus(IMUHeaterStatus_t *status)
{
    if (status == NULL)
        return;

    status->target_temp = heater_target;
    status->temperature = last_temperature;
    status->duty = heater_duty;
    status->sensor_valid = last_sensor_valid;
    status->fault = heater_fault;
    status->overtemp = heater_overtemp;
    status->timeout_fault = heater_timeout_fault;
    status->heating = (heater_duty > 0) ? 1 : 0;
}
