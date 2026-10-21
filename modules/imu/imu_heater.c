/**
 * @file imu_heater.c
 * @brief 检查 IMU 数据、输入电压和故障状态，再决定本次加热输出。
 *
 * 加热电阻由 VCC_IN 供电，经 Q9 和 PB1/TIM3_CH4 控制；PWM 比较值最多500，
 * 还按输入电压和47Ω估算电阻，将允许值降到约0.30W对应的范围。
 * INS_Init 初始化后由 INS 定期更新，标定与 Flash 操作通过 Begin/End 暂时禁止加热。
 * 公开接口的参数、调用顺序和恢复条件见 imu_heater.h。
 */
#include "imu_heater.h"
#include "controller.h"
#include "tim.h"
#include "main.h"
#include "bsp_log.h"
#include "bsp_adc.h"
#include <math.h>
#include <stddef.h>

/* 目标温度和温度检查值均以 °C 为单位。 */
#define IMU_HEATER_TARGET_DEFAULT 40.0f
#define IMU_HEATER_TARGET_MIN 30.0f
#define IMU_HEATER_TARGET_MAX 45.0f

#define IMU_HEATER_ARR 9999u
#define IMU_HEATER_DUTY_NORMAL 500u   // PWM 比较值最多500，ARR为9999时约占每周期的5%
#define IMU_HEATER_R_EQUIV_MIN 47.0f  // 功率估算使用的等效电阻，单位 Ω
#define IMU_HEATER_MAX_POWER_W 0.30f  // 功率估算采用的上限，单位 W
/* 一段连续非零输出最多持续5分钟，输出归0后重新计时。 */
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
static uint32_t heater_pid_tick_ms;
static uint8_t heater_pid_has_tick;
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
/* 每一位对应一种耗时操作；任意进行中或失败的记录都会禁止加热。 */
static uint32_t heater_long_operation_active_mask = 0u;
static uint32_t heater_long_operation_failed_mask = 0u;
/* 等待恢复时，保存比较新数据所用的序号和毫秒数。 */
static uint8_t heater_require_new_sample = 0u;
static uint32_t heater_rearm_sample_sequence = 0u;
static uint32_t heater_rearm_timestamp_ms = 0u;
static uint8_t heater_last_force_off_request = 0u;
/* 首次故障时记下序号和时刻，清故障时检查后续数据。 */
static uint8_t heater_fault_epoch_valid = 0u;
static uint32_t heater_fault_sample_sequence = 0u;
static uint32_t heater_fault_timestamp_ms = 0u;

/* 候选序号非0，且差值在1～0x7FFFFFFF时视为更新，可处理序号超过最大值后从1继续。 */
static uint8_t IMUHeaterSequenceIsNewer(uint32_t candidate, uint32_t previous)
{
    uint32_t delta = candidate - previous;
    return (candidate != 0u && delta != 0u && delta < 0x80000000u) ? 1u : 0u;
}

/* 与序号类似，用无符号差值比较毫秒计数；相同时间也允许，过大的差值按更早处理。 */
static uint8_t IMUHeaterTimestampIsNotBefore(uint32_t candidate, uint32_t reference)
{
    return ((uint32_t)(candidate - reference) < 0x80000000u) ? 1u : 0u;
}

/* 仅在首次故障时保存序号和时刻并返回1，让调用方只打印一次故障日志。 */
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

/* 将操作类型换成对应的一位；编号不对时返回0，不改动其他类型的记录。 */
static uint32_t IMUHeaterLongOperationMask(IMUHeater_LongOperation_e operation)
{
    if ((uint32_t)operation >= IMU_HEATER_LONG_OPERATION_COUNT)
        return 0u;
    return 1u << (uint32_t)operation;
}

/* 统一设置软件记录和 PWM；未初始化时只改软件记录，不写 TIM3。 */
static void IMUHeaterApplyDuty(uint16_t duty)
{
#if !MC02_HEATER_ENABLED
    // 禁热构建把任何请求都改为0，避免其他调用路径产生非零输出。
    duty = 0;
#else
    // 启热时也不允许超过500；正常更新还会按输入电压进一步降低允许值。
    if (duty > IMU_HEATER_DUTY_NORMAL)
        duty = IMU_HEATER_DUTY_NORMAL;
#endif

    heater_duty = duty;
    imu_heater_duty_diag = duty;
    if (heater_initialized)
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, heater_duty);
}

/* 读取 ADC 层确认有效且未超过20ms的电压；失败时将非空输出置0，单位 V。 */
static uint8_t IMUHeaterReadFreshSupply(float *vcc_in)
{
    if (vcc_in == NULL)
        return 0u;

    BSP_ADC_Sample_t sample;
    if (BSP_ADCGetSample(&sample) != BSP_ADC_STATUS_VALID ||
        !sample.voltage_config_valid || !isfinite(sample.vcc_in_volts))
    {
        // 采样失败、换算设置不可用或出现 NaN/Inf 时，不能沿用旧电压计算允许功率。
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
        // 重复初始化保留已有故障和操作记录，只关闭当前输出。
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
        .DtMin_s = 0.00001f,
        .DtMax_s = 0.02f,
    };
    PID_Status_e pid_status = PIDInit(&heater_pid, &config);

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
    heater_pid_has_tick = 0u;
    if (pid_status != PID_OK)
    {
        // PID 参数被拒绝时保持故障和零输出，不启动加热 PWM。
        (void)IMUHeaterLatchFault(HAL_GetTick());
        IMUHeaterApplyDuty(0u);
        return;
    }

    if (!IMUHeaterReadFreshSupply(&last_vcc_in) ||
        last_vcc_in < IMU_HEATER_MIN_SUPPLY_V || last_vcc_in > IMU_HEATER_MAX_SUPPLY_V)
    {
        // 此处只提示供电尚不可用；Update 会重新检查，初始化的输出始终为0。
        BSP_LOG_EVENT_1(EV_MODULES_IMU_IMU_HEATER_C_01, BSP_LOG_FIELD_F32(last_vcc_in));
    }

    // 先写0，启热构建才启动 PWM，避免启动时沿用旧比较值。
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0u);
#if MC02_HEATER_ENABLED
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
#endif

    BSP_LOG_EVENT_4(EV_MODULES_IMU_IMU_HEATER_C_02,
                    BSP_LOG_FIELD_F32(heater_target),
                    BSP_LOG_FIELD_U32(IMU_HEATER_DUTY_NORMAL),
                    BSP_LOG_FIELD_U32(IMU_HEATER_ARR),
                    BSP_LOG_FIELD_F32(IMU_HEATER_MAX_POWER_W));
}

void IMUHeaterForceOff(void)
{
    IMUHeaterApplyDuty(0u);
    if (heater_initialized)
    {
        // 关闭后清除全部计算历史，重新加热前先建立新的计时起点。
        (void)PIDReset(&heater_pid);
    }
    heater_pid_has_tick = 0u;
    heat_start_ms = 0u;
    heat_timer_active = 0u;
}

uint8_t IMUHeaterBeginLongOperation(IMUHeater_LongOperation_e operation)
{
    uint32_t mask = IMUHeaterLongOperationMask(operation);
    // 类型不对，或同类正在进行、曾经失败时拒绝开始，保持原状态。
    if (mask == 0u || ((heater_long_operation_active_mask | heater_long_operation_failed_mask) & mask) != 0u)
        return 0u;

    // 先登记再关闭输出，后续 Update 看到任一登记就保持关闭。
    heater_long_operation_active_mask |= mask;
    IMUHeaterForceOff();
    return 1u;
}

void IMUHeaterEndLongOperation(IMUHeater_LongOperation_e operation, uint8_t success)
{
    uint32_t mask = IMUHeaterLongOperationMask(operation);
    // 没有对应的开始记录时不处理，避免误改其他操作的状态。
    if (mask == 0u || (heater_long_operation_active_mask & mask) == 0u)
        return;

    heater_long_operation_active_mask &= ~mask;
    if (!success)
    {
        // 保留这一类的失败记录；普通清故障和再次开始都不能解除它。
        heater_long_operation_failed_mask |= mask;
        (void)IMUHeaterLatchFault(HAL_GetTick());
        IMUHeaterForceOff();
        return;
    }

    // 成功结束也要等到序号更新、采样时间不早于此刻的数据，避免使用操作前的旧数据。
    heater_rearm_sample_sequence = last_seen_sample_sequence;
    heater_rearm_timestamp_ms = HAL_GetTick();
    heater_require_new_sample = 1u;
}

uint8_t IMUHeaterClearFault(void)
{
    // 先关闭输出，即使下面拒绝清除，本次也继续保持关闭。
    IMUHeaterForceOff();
    float supply = 0.0f;
    uint8_t supply_valid = IMUHeaterReadFreshSupply(&supply);

    // 运行状态尚未允许恢复，或最近一次更新没有接受可用温度时，保留故障。
    if (!heater_initialized || heater_last_force_off_request ||
        heater_long_operation_active_mask != 0u || heater_long_operation_failed_mask != 0u ||
        heater_require_new_sample || !heater_has_valid_sample || !last_sensor_valid)
        return 0u;

    uint32_t now = HAL_GetTick();
    // 清除要求数据仍在5ms内，温度至少-20°C且严格低于42°C；等于42°C也拒绝。
    if (!BMI088SampleIsFresh(&last_valid_sample, now) ||
        !isfinite(last_valid_sample.temperature) ||
        last_valid_sample.temperature < IMU_HEATER_TEMP_VALID_MIN ||
        last_valid_sample.temperature >= IMU_HEATER_OVERTEMP_RECOVER)
        return 0u;

    // 已有故障还要看首次故障后收到的数据，重复或更早的数据不能用来清除。
    if (heater_fault &&
        (!heater_fault_epoch_valid ||
         !IMUHeaterSequenceIsNewer(last_valid_sample.sequence, heater_fault_sample_sequence) ||
         !IMUHeaterTimestampIsNotBefore(last_valid_sample.timestamp_ms, heater_fault_timestamp_ms)))
        return 0u;

    // 恢复还需当前电压有效且在10～30V之间，两端均允许。
    if (!supply_valid || supply < IMU_HEATER_MIN_SUPPLY_V || supply > IMU_HEATER_MAX_SUPPLY_V)
        return 0u;

    heater_fault = 0u;
    heater_overtemp = 0u;
    heater_timeout_fault = 0u;
    heater_fault_epoch_valid = 0u;
    // 以本次清除所用的数据为起点，等到后续序号更新、时间不早于它的数据再继续温控。
    heater_rearm_sample_sequence = last_valid_sample.sequence;
    heater_rearm_timestamp_ms = last_valid_sample.timestamp_ms;
    heater_require_new_sample = 1u;
    BSP_LOG_EVENT_0(EV_MODULES_IMU_IMU_HEATER_C_03);
    return 1u;
}

void IMUHeaterSetTarget(float target_temp)
{
    // NaN/Inf 不参与温控计算，收到这些值时保留原目标。
    if (!isfinite(target_temp))
        return;
    if (target_temp < IMU_HEATER_TARGET_MIN)
        target_temp = IMU_HEATER_TARGET_MIN;
    if (target_temp > IMU_HEATER_TARGET_MAX)
        target_temp = IMU_HEATER_TARGET_MAX;
    heater_target = target_temp;
}

/* 最多每1000ms记录一次状态，避免按温控更新频率刷满日志。 */
static void IMUHeaterLogStatus(float temperature, uint8_t sensor_valid)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_log_ms) >= IMU_HEATER_LOG_PERIOD_MS)
    {
        last_log_ms = now;
        BSP_LOG_EVENT_6(EV_MODULES_IMU_IMU_HEATER_C_04,
                        BSP_LOG_FIELD_F32(last_vcc_in),
                        BSP_LOG_FIELD_F32(temperature),
                        BSP_LOG_FIELD_F32(heater_target),
                        BSP_LOG_FIELD_U32(heater_duty),
                        BSP_LOG_FIELD_U32(sensor_valid),
                        BSP_LOG_FIELD_U32(heater_fault));
    }
}

void IMUHeaterUpdate(const BMI088_Data_t *sample, uint8_t force_off)
{
    // 初始化前不访问 PWM 硬件，也不自动开始温控。
    if (!heater_initialized)
        return;

    heater_last_force_off_request = force_off ? 1u : 0u;
    if ((heater_long_operation_active_mask | heater_long_operation_failed_mask) != 0u)
    {
        // 标定或 Flash 操作仍在进行，或曾经失败时，不使用传入温度恢复输出。
        last_sensor_valid = 0u;
        IMUHeaterForceOff();
        IMUHeaterLogStatus(last_temperature, 0u);
        return;
    }

    uint32_t now = HAL_GetTick();
    uint8_t incoming_fresh = BMI088SampleIsFresh(sample, now);
    uint8_t sample_fresh = 0u;
    const BMI088_Data_t *accepted_sample = NULL;

    // 同序号只能继续使用原数据；传入新的温度或时间也不能替换已保存的内容。
    if (incoming_fresh && heater_has_valid_sample &&
        sample->sequence == last_valid_sample.sequence)
    {
        // 等待新数据时不接受重复序号；其余情况也按原采样时间检查是否超过5ms。
        accepted_sample = &last_valid_sample;
        sample_fresh = (!heater_require_new_sample &&
                        BMI088SampleIsFresh(accepted_sample, now)) ? 1u : 0u;
    }
    else if (incoming_fresh &&
             // 新序号的时间不能更早；等待恢复时还要满足此前记下的序号和时间。
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
            // 只有温度通过-20～80°C检查才保存整份数据，并更新传感器失联计时的起点。
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

    // 即使禁热，也先检查温度失效和过温，让故障记录仍能反映数据问题。
    if (!temp_ok)
    {
        IMUHeaterForceOff();
        // 数据无效先关闭；距上次接受的采样时刻达到500ms，再记录故障。
        if ((uint32_t)(now - last_valid_sample_ms) >= IMU_HEATER_SENSOR_TIMEOUT_MS)
        {
            if (IMUHeaterLatchFault(now))
                BSP_LOG_EVENT_0(EV_MODULES_IMU_IMU_HEATER_C_05);
        }
    }
    else if (temperature >= IMU_HEATER_OVERTEMP_CUTOFF)
    {
        // 可用温度达到50°C就记下过温，之后降温也不会自动清除。
        IMUHeaterForceOff();
        heater_overtemp = 1u;
        if (IMUHeaterLatchFault(now))
            BSP_LOG_EVENT_2(EV_MODULES_IMU_IMU_HEATER_C_06,
                            BSP_LOG_FIELD_F32(temperature),
                            BSP_LOG_FIELD_F32(IMU_HEATER_OVERTEMP_CUTOFF));
    }

    if (force_off || !temp_ok || heater_fault)
    {
        // 任何一项要求关闭，都在电压和 PID 计算之前结束本次处理。
        IMUHeaterForceOff();
        IMUHeaterLogStatus(temperature, temp_ok);
        return;
    }

    // 禁热构建也检查电压；ADC 数据无效时不使用上一次电压继续计算。
    if (!IMUHeaterReadFreshSupply(&last_vcc_in))
    {
        IMUHeaterForceOff();
        if (!supply_missing_logged)
        {
            BSP_LOG_EVENT_0(EV_MODULES_IMU_IMU_HEATER_C_07);
            supply_missing_logged = 1u;
        }
        return;
    }

    if (last_vcc_in < IMU_HEATER_MIN_SUPPLY_V)
    {
        // 低于10V暂时关闭，后续供电恢复可重新检查；此分支不记录持续故障。
        IMUHeaterForceOff();
        if (!supply_missing_logged)
        {
            BSP_LOG_EVENT_1(EV_MODULES_IMU_IMU_HEATER_C_08,
                            BSP_LOG_FIELD_F32(last_vcc_in));
            supply_missing_logged = 1u;
        }
        return;
    }
    if (last_vcc_in > IMU_HEATER_MAX_SUPPLY_V)
    {
        // 高于30V则保留故障，供电恢复后也需要通过清故障检查。
        IMUHeaterForceOff();
        if (IMUHeaterLatchFault(now))
            BSP_LOG_EVENT_1(EV_MODULES_IMU_IMU_HEATER_C_09,
                            BSP_LOG_FIELD_F32(last_vcc_in));
        return;
    }
    supply_missing_logged = 0u;

#if !MC02_HEATER_ENABLED
    // 禁热构建在状态检查后返回，不计算 PID，也不累计非零加热时间。
    IMUHeaterForceOff();
    IMUHeaterLogStatus(temperature, temp_ok);
    return;
#endif

    // 输入电压越高，同一电阻上的功率越大；按0.30W估算允许值，并与500取较小值。
    uint16_t duty_limit = IMU_HEATER_DUTY_NORMAL;
    float max_duty_by_power =
        (IMU_HEATER_MAX_POWER_W * (float)IMU_HEATER_ARR * IMU_HEATER_R_EQUIV_MIN) /
        (last_vcc_in * last_vcc_in);
    if (max_duty_by_power < (float)duty_limit)
        duty_limit = (uint16_t)max_duty_by_power;
    if (duty_limit == 0u)
    {
        // 允许值不足一个比较计数时保持关闭。
        IMUHeaterForceOff();
        return;
    }

    // 达到连续加热300000ms时先报超时，再检查是否已达到目标加2°C。
    if (heat_timer_active &&
        (uint32_t)(now - heat_start_ms) >= IMU_HEATER_MAX_ON_TIME_MS)
    {
        IMUHeaterForceOff();
        heater_timeout_fault = 1u;
        if (IMUHeaterLatchFault(now))
            BSP_LOG_EVENT_0(EV_MODULES_IMU_IMU_HEATER_C_10);
        return;
    }

    if (temperature >= heater_target + IMU_HEATER_TARGET_TOLERANCE)
    {
        // 高于目标至少2°C时直接关闭，本次不再让 PID 输出加热值。
        IMUHeaterForceOff();
        IMUHeaterLogStatus(temperature, temp_ok);
        return;
    }

    // 温控任务使用现有毫秒时基测量实际调用间隔，无符号减法允许计数回绕。
    uint32_t elapsed_ms = now - heater_pid_tick_ms;
    uint8_t had_tick = heater_pid_has_tick;
    heater_pid_tick_ms = now;
    heater_pid_has_tick = 1u;
    if (!had_tick || elapsed_ms == 0u || elapsed_ms > 20u)
    {
        // 首轮或时间间隔异常时只建立新起点，本轮不使用过去累计的积分。
        (void)PIDReset(&heater_pid);
        IMUHeaterApplyDuty(0u);
        heat_start_ms = 0u;
        heat_timer_active = 0u;
        return;
    }
    float output = 0.0f;
    if (PIDCalculate(&heater_pid, temperature, heater_target,
                     (float)elapsed_ms / 1000.0f, &output) != PID_OK)
    {
        // 计算失败不能沿用上次 PWM；下一轮重新计时后再尝试。
        IMUHeaterForceOff();
        return;
    }
    // PID 输出出现 NaN/Inf 或小于0时按0处理，过大时按刚才算出的允许值处理。
    if (!isfinite(output) || output < 0.0f)
        output = 0.0f;
    if (output > (float)duty_limit)
        output = (float)duty_limit;

    IMUHeaterApplyDuty((uint16_t)output);
    if (heater_duty == 0u)
    {
        // 输出归0就结束这一段连续计时，下次非零时重新开始。
        heat_start_ms = 0u;
        heat_timer_active = 0u;
    }
    else if (!heat_timer_active)
    {
        // 单独用 active 标志记录是否已开始，毫秒计数为0时也能正常计时。
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
