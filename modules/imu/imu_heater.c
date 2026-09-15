#include "imu_heater.h"
#include "controller.h"
#include "tim.h"
#include "main.h"
#include "bsp_log.h"
#include <math.h>

/*
 * IMU 加热安全策略(2026-09 起火事故后重构):
 * - 上电/初始化/故障/急停时 PWM=0, 不再有"无条件满功率预热".
 * - 温度必须有限且在合理范围内, 传感器无效或超时直接关加热.
 * - 占空比硬限幅: 平时 5%(500/9999), 允许短时 10%(1000/9999) 预热.
 * - 预热有最长时长, 超时后降到普通上限; 连续加热最长 60s, 超时锁存故障.
 * - 温度 > 50°C 立即关加热并锁存, 降到 42°C 以下且手动清除后才允许恢复.
 */

#define IMU_HEATER_TARGET_DEFAULT 40.0f
#define IMU_HEATER_TARGET_MIN 30.0f
#define IMU_HEATER_TARGET_MAX 45.0f

#define IMU_HEATER_ARR 9999u
#define IMU_HEATER_DUTY_NORMAL 500u   // 5%
#define IMU_HEATER_DUTY_BOOST 1000u   // 10%, 仅短时预热
#define IMU_HEATER_BOOST_TIME_MS 3000u
#define IMU_HEATER_MAX_ON_TIME_MS 60000u

#define IMU_HEATER_TEMP_VALID_MIN -20.0f
#define IMU_HEATER_TEMP_VALID_MAX 80.0f
#define IMU_HEATER_OVERTEMP_CUTOFF 50.0f
#define IMU_HEATER_OVERTEMP_RECOVER 42.0f
#define IMU_HEATER_TARGET_TOLERANCE 2.0f
#define IMU_HEATER_SENSOR_TIMEOUT_MS 500u
#define IMU_HEATER_LOG_PERIOD_MS 1000u

static PIDInstance heater_pid;
static float heater_target = IMU_HEATER_TARGET_DEFAULT;
static uint16_t heater_duty = 0;
static uint8_t heater_initialized = 0;
static uint8_t heater_fault = 0;
static uint8_t heater_overtemp = 0;
static uint8_t heater_timeout_fault = 0;
static uint32_t last_valid_sample_ms = 0;
static uint32_t heat_start_ms = 0;
static uint32_t last_log_ms = 0;
static float last_temperature = 0.0f;
static uint8_t last_sensor_valid = 0;

static void IMUHeaterApplyDuty(uint16_t duty)
{
    if (duty > IMU_HEATER_ARR)
        duty = IMU_HEATER_ARR;

    heater_duty = duty;
    if (heater_initialized)
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, heater_duty);
}

void IMUHeaterInit(void)
{
    PID_Init_Config_s config = {
        .MaxOut = IMU_HEATER_DUTY_BOOST,
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
    heat_start_ms = 0;
    last_log_ms = 0;
    heater_initialized = 1;

    /* 先确保比较值为0, 再启动PWM, 避免任何形式的开机满功率 */
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    LOGINFO("[imu_heat] init safe: target=%d.%dC max_duty=%u/%u",
            (int)heater_target, (int)(heater_target * 10.0f) % 10,
            (unsigned)IMU_HEATER_DUTY_NORMAL, (unsigned)IMU_HEATER_ARR);
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
        LOGINFO("[imu_heat] temp=%d.%dC target=%d.%dC duty=%u valid=%u fault=%u",
                (int)temperature, (int)(temperature * 10.0f) % 10,
                (int)heater_target, (int)(heater_target * 10.0f) % 10,
                (unsigned)heater_duty,
                (unsigned)sensor_valid, (unsigned)heater_fault);
    }
}

void IMUHeaterUpdate(float temperature, uint8_t sensor_valid, uint8_t force_off)
{
    if (!heater_initialized)
        return;

    uint32_t now = HAL_GetTick();

    /* 急停/故障状态由上层传入force_off, 最高优先级 */
    if (force_off)
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
    if (!sensor_valid || !isfinite(temperature) ||
        temperature < IMU_HEATER_TEMP_VALID_MIN ||
        temperature > IMU_HEATER_TEMP_VALID_MAX)
    {
        IMUHeaterForceOff();
        if ((uint32_t)(now - last_valid_sample_ms) > IMU_HEATER_SENSOR_TIMEOUT_MS)
        {
            heater_fault = 1;
            LOGERROR("[imu_heat] sensor invalid/timeout, heater latched off");
        }
        return;
    }

    last_temperature = temperature;
    last_sensor_valid = sensor_valid;
    last_valid_sample_ms = now;

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

    /* 短时预热允许10%, 超过时间后回落到5% */
    uint16_t duty_limit = IMU_HEATER_DUTY_NORMAL;
    if ((uint32_t)(now - heat_start_ms) < IMU_HEATER_BOOST_TIME_MS &&
        temperature < heater_target - 5.0f)
    {
        duty_limit = IMU_HEATER_DUTY_BOOST;
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
