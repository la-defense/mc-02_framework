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
 * - 连续非零加热最长 300000ms (5分钟), 超时锁存故障.
 * - 温度 >= 50°C 立即关加热并锁存, 降到 42°C 以下且手动清除后才允许恢复.
 */

#define IMU_HEATER_TARGET_DEFAULT 40.0f
#define IMU_HEATER_TARGET_MIN 30.0f
#define IMU_HEATER_TARGET_MAX 45.0f

#define IMU_HEATER_ARR 9999u
#define IMU_HEATER_DUTY_NORMAL 500u   // 5% 绝对硬上限(厂商例程 MAX_OUT=500)
#define IMU_HEATER_R_EQUIV_MIN 47.0f  // R77~R89 全并联最坏等效电阻
#define IMU_HEATER_MAX_POWER_W 0.30f  // 首次24V测试的功率上限
/* 首测功率被压到0.30W, 预热较慢; 5分钟连续加热超时用于抓传感器/加热片卡死,
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
volatile uint32_t imu_heater_duty_diag = 0u;
static uint8_t heater_initialized = 0;
static uint8_t heater_fault = 0;
static uint8_t heater_overtemp = 0;
static uint8_t heater_timeout_fault = 0;
static uint32_t last_valid_sample_ms = 0;
static uint32_t last_seen_sample_sequence = 0;
static BMI088_Data_t last_valid_sample;
static uint8_t heater_has_valid_sample = 0u;
static uint32_t heat_start_ms = 0;
static uint8_t heat_timer_active = 0u;
static uint32_t last_log_ms = 0;
static float last_temperature = 0.0f;
static uint8_t last_sensor_valid = 0;
static float last_vcc_in = 0.0f;
static uint8_t supply_missing_logged = 0;
static uint32_t heater_long_operation_active_mask = 0u;
static uint32_t heater_long_operation_failed_mask = 0u;
static uint8_t heater_require_new_sample = 0u;
static uint32_t heater_rearm_sample_sequence = 0u;
static uint32_t heater_rearm_timestamp_ms = 0u;
static uint8_t heater_last_force_off_request = 0u;
static uint8_t heater_fault_epoch_valid = 0u;
static uint32_t heater_fault_sample_sequence = 0u;
static uint32_t heater_fault_timestamp_ms = 0u;

static uint8_t IMUHeaterSequenceIsNewer(uint32_t candidate, uint32_t previous)
{
    uint32_t delta = candidate - previous;
    return (candidate != 0u && delta != 0u && delta < 0x80000000u) ? 1u : 0u;
}

static uint8_t IMUHeaterTimestampIsNotBefore(uint32_t candidate, uint32_t reference)
{
    return ((uint32_t)(candidate - reference) < 0x80000000u) ? 1u : 0u;
}

static uint8_t IMUHeaterLatchFault(uint32_t now)
{
    if (!heater_fault)
    {
        heater_fault = 1u;
        heater_fault_epoch_valid = 1u;
        heater_fault_sample_sequence = last_seen_sample_sequence;
        heater_fault_timestamp_ms = now;
        return 1u;
    }
    return 0u;
}

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
    imu_heater_duty_diag = duty;
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
    if (heater_initialized)
    {
        IMUHeaterForceOff();
        return;
    }

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
    heater_duty = 0u;
    imu_heater_duty_diag = 0u;
    heater_fault = 0u;
    heater_overtemp = 0u;
    heater_timeout_fault = 0u;
    last_valid_sample_ms = HAL_GetTick();
    last_seen_sample_sequence = 0u;
    last_valid_sample = (BMI088_Data_t){0};
    heater_has_valid_sample = 0u;
    heat_start_ms = 0u;
    heat_timer_active = 0u;
    last_log_ms = 0u;
    last_temperature = 0.0f;
    last_sensor_valid = 0u;
    last_vcc_in = 0.0f;
    supply_missing_logged = 0u;
    heater_long_operation_active_mask = 0u;
    heater_long_operation_failed_mask = 0u;
    heater_require_new_sample = 0u;
    heater_rearm_sample_sequence = 0u;
    heater_rearm_timestamp_ms = 0u;
    heater_last_force_off_request = 0u;
    heater_fault_epoch_valid = 0u;
    heater_fault_sample_sequence = 0u;
    heater_fault_timestamp_ms = 0u;
    heater_initialized = 1u;

    if (!IMUHeaterReadFreshSupply(&last_vcc_in) ||
        last_vcc_in < IMU_HEATER_MIN_SUPPLY_V || last_vcc_in > IMU_HEATER_MAX_SUPPLY_V)
    {
        LOGWARNING("[imu_heat] VCC_IN not ready (%d.%dV), heater disabled until valid supply",
                   (int)last_vcc_in, (int)(last_vcc_in * 10.0f) % 10);
    }

    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0u);
#if MC02_HEATER_ENABLED
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
#endif

    LOGINFO("[imu_heat] init safe: target=%d.%dC max_duty=%u/%u power_cap=%d.%dW",
            (int)heater_target, (int)(heater_target * 10.0f) % 10,
            (unsigned)IMU_HEATER_DUTY_NORMAL, (unsigned)IMU_HEATER_ARR,
            (int)IMU_HEATER_MAX_POWER_W, (int)(IMU_HEATER_MAX_POWER_W * 10.0f) % 10);
}

void IMUHeaterForceOff(void)
{
    IMUHeaterApplyDuty(0u);
    if (heater_initialized)
    {
        heater_pid.Iout = 0.0f;
        heater_pid.Output = 0.0f;
        heater_pid.Last_Output = 0.0f;
    }
    heat_start_ms = 0u;
    heat_timer_active = 0u;
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
        (void)IMUHeaterLatchFault(HAL_GetTick());
        IMUHeaterForceOff();
        return;
    }

    heater_rearm_sample_sequence = last_seen_sample_sequence;
    heater_rearm_timestamp_ms = HAL_GetTick();
    heater_require_new_sample = 1u;
}

uint8_t IMUHeaterClearFault(void)
{
    IMUHeaterForceOff();
    float supply = 0.0f;
    uint8_t supply_valid = IMUHeaterReadFreshSupply(&supply);

    if (!heater_initialized || heater_last_force_off_request ||
        heater_long_operation_active_mask != 0u || heater_long_operation_failed_mask != 0u ||
        heater_require_new_sample || !heater_has_valid_sample || !last_sensor_valid)
        return 0u;

    uint32_t now = HAL_GetTick();
    if (!BMI088SampleIsFresh(&last_valid_sample, now) ||
        !isfinite(last_valid_sample.temperature) ||
        last_valid_sample.temperature < IMU_HEATER_TEMP_VALID_MIN ||
        last_valid_sample.temperature >= IMU_HEATER_OVERTEMP_RECOVER)
        return 0u;

    if (heater_fault &&
        (!heater_fault_epoch_valid ||
         !IMUHeaterSequenceIsNewer(last_valid_sample.sequence, heater_fault_sample_sequence) ||
         !IMUHeaterTimestampIsNotBefore(last_valid_sample.timestamp_ms, heater_fault_timestamp_ms)))
        return 0u;

    if (!supply_valid || supply < IMU_HEATER_MIN_SUPPLY_V || supply > IMU_HEATER_MAX_SUPPLY_V)
        return 0u;

    heater_fault = 0u;
    heater_overtemp = 0u;
    heater_timeout_fault = 0u;
    heater_fault_epoch_valid = 0u;
    heater_rearm_sample_sequence = last_valid_sample.sequence;
    heater_rearm_timestamp_ms = last_valid_sample.timestamp_ms;
    heater_require_new_sample = 1u;
    LOGINFO("[imu_heat] fault cleared; waiting for a new IMU sample");
    return 1u;
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

    heater_last_force_off_request = force_off ? 1u : 0u;
    if ((heater_long_operation_active_mask | heater_long_operation_failed_mask) != 0u)
    {
        last_sensor_valid = 0u;
        IMUHeaterForceOff();
        IMUHeaterLogStatus(last_temperature, 0u);
        return;
    }

    uint32_t now = HAL_GetTick();
    uint8_t incoming_fresh = BMI088SampleIsFresh(sample, now);
    uint8_t sample_fresh = 0u;
    const BMI088_Data_t *accepted_sample = NULL;

    /* 重复序号只使用缓存值，不能以调用者重放的时间戳续期。 */
    if (incoming_fresh && heater_has_valid_sample &&
        sample->sequence == last_valid_sample.sequence)
    {
        /* A replay cannot replace cached values or renew the sample timestamp. */
        accepted_sample = &last_valid_sample;
        sample_fresh = (!heater_require_new_sample &&
                        BMI088SampleIsFresh(accepted_sample, now)) ? 1u : 0u;
    }
    else if (incoming_fresh &&
             /* 只接受序号前进且时间戳不回退的完整新样本。 */
             (!heater_has_valid_sample ||
              IMUHeaterSequenceIsNewer(sample->sequence, last_valid_sample.sequence)) &&
             (!heater_has_valid_sample ||
              IMUHeaterTimestampIsNotBefore(sample->timestamp_ms, last_valid_sample.timestamp_ms)) &&
             (!heater_require_new_sample ||
              (IMUHeaterSequenceIsNewer(sample->sequence, heater_rearm_sample_sequence) &&
               IMUHeaterTimestampIsNotBefore(sample->timestamp_ms, heater_rearm_timestamp_ms))))
    {
        if (isfinite(sample->temperature) &&
            sample->temperature >= IMU_HEATER_TEMP_VALID_MIN &&
            sample->temperature <= IMU_HEATER_TEMP_VALID_MAX)
        {
            last_valid_sample = *sample;
            heater_has_valid_sample = 1u;
            last_seen_sample_sequence = sample->sequence;
            last_valid_sample_ms = sample->timestamp_ms;
            last_temperature = sample->temperature;
            accepted_sample = &last_valid_sample;
            sample_fresh = 1u;
            if (heater_require_new_sample)
                heater_require_new_sample = 0u;
        }
    }

    uint8_t temp_ok = (sample_fresh && accepted_sample != NULL &&
                       isfinite(accepted_sample->temperature) &&
                       accepted_sample->temperature >= IMU_HEATER_TEMP_VALID_MIN &&
                       accepted_sample->temperature <= IMU_HEATER_TEMP_VALID_MAX) ? 1u : 0u;
    float temperature = temp_ok ? accepted_sample->temperature : last_temperature;
    last_sensor_valid = temp_ok;

    /* 即使编译时禁热，温度失效和过温仍然参与故障判断。 */
    if (!temp_ok)
    {
        IMUHeaterForceOff();
        if ((uint32_t)(now - last_valid_sample_ms) >= IMU_HEATER_SENSOR_TIMEOUT_MS)
        {
            if (IMUHeaterLatchFault(now))
                LOGERROR("[imu_heat] sensor invalid/timeout, heater latched off");
        }
    }
    else if (temperature >= IMU_HEATER_OVERTEMP_CUTOFF)
    {
        IMUHeaterForceOff();
        heater_overtemp = 1u;
        if (IMUHeaterLatchFault(now))
            LOGERROR("[imu_heat] OVERTEMP %d.%dC >= %d.%dC, heater latched off",
                     (int)temperature, (int)(temperature * 10.0f) % 10,
                     (int)IMU_HEATER_OVERTEMP_CUTOFF, (int)(IMU_HEATER_OVERTEMP_CUTOFF * 10.0f) % 10);
    }

    if (force_off || !temp_ok || heater_fault)
    {
        IMUHeaterForceOff();
        IMUHeaterLogStatus(temperature, temp_ok);
        return;
    }

    /* 电压检查也先于禁热构建的返回，保证裸板仍能验证采样状态。 */
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
            supply_missing_logged = 1u;
        }
        return;
    }
    if (last_vcc_in > IMU_HEATER_MAX_SUPPLY_V)
    {
        IMUHeaterForceOff();
        if (IMUHeaterLatchFault(now))
            LOGERROR("[imu_heat] VCC_IN %d.%dV too high, heater latched off",
                     (int)last_vcc_in, (int)(last_vcc_in * 10.0f) % 10);
        return;
    }
    supply_missing_logged = 0u;

#if !MC02_HEATER_ENABLED
    IMUHeaterForceOff();
    IMUHeaterLogStatus(temperature, temp_ok);
    return;
#endif

    uint16_t duty_limit = IMU_HEATER_DUTY_NORMAL;
    float max_duty_by_power =
        (IMU_HEATER_MAX_POWER_W * (float)IMU_HEATER_ARR * IMU_HEATER_R_EQUIV_MIN) /
        (last_vcc_in * last_vcc_in);
    if (max_duty_by_power < (float)duty_limit)
        duty_limit = (uint16_t)max_duty_by_power;
    if (duty_limit == 0u)
    {
        IMUHeaterForceOff();
        return;
    }

    /* 只累计实际非零 PWM；active 标志允许起始 tick 为 0。 */
    if (heat_timer_active &&
        (uint32_t)(now - heat_start_ms) >= IMU_HEATER_MAX_ON_TIME_MS)
    {
        IMUHeaterForceOff();
        heater_timeout_fault = 1u;
        if (IMUHeaterLatchFault(now))
            LOGERROR("[imu_heat] max heating time exceeded, heater latched off");
        return;
    }

    if (temperature >= heater_target + IMU_HEATER_TARGET_TOLERANCE)
    {
        IMUHeaterForceOff();
        IMUHeaterLogStatus(temperature, temp_ok);
        return;
    }

    float output = PIDCalculate(&heater_pid, temperature, heater_target);
    if (!isfinite(output) || output < 0.0f)
        output = 0.0f;
    if (output > (float)duty_limit)
        output = (float)duty_limit;

    IMUHeaterApplyDuty((uint16_t)output);
    if (heater_duty == 0u)
    {
        heat_start_ms = 0u;
        heat_timer_active = 0u;
    }
    else if (!heat_timer_active)
    {
        heat_start_ms = now;
        heat_timer_active = 1u;
    }
    IMUHeaterLogStatus(temperature, temp_ok);
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
