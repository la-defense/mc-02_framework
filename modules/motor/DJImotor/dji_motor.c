/** @file dji_motor.c
* @brief 解码 DJI 电调反馈，逐电机计算参考输出，并按总线与设备编号组成四电机 CAN 帧。
 */
#include "dji_motor.h"
#include <string.h>
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_can.h"
#include "motor_pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"
#include <math.h>
#include <stdlib.h>

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用

/* ---------------- 性能探针(2026-09, CPU 占用排查) ----------------
   dji_prof_all : 单次 DJIMotorControl 总耗时
   dji_prof_pid : 串级 PID 计算 + 组帧(不含 CAN 发送)
   dji_prof_send: CAN 提交耗时；bsp_can 当前发送空间满即返回，不等待。 */
volatile DWT_Probe_t dji_prof_all = {0};
volatile DWT_Probe_t dji_prof_pid = {0};
volatile DWT_Probe_t dji_prof_send = {0};
volatile uint32_t dji_current_sat_count = 0; /* 电流指令被饱和的次数(MOTOR-09 观测) */
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算


#ifdef FDCAN
static CANInstance sender_assignment[9] = {
    [0] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x1ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [1] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x200, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [2] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x2ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [3] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x1ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [4] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x200, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [5] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x2ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [6] = {.can_handle = &hfdcan3, .txconf.Identifier = 0x1ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [7] = {.can_handle = &hfdcan3, .txconf.Identifier = 0x200, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [8] = {.can_handle = &hfdcan3, .txconf.Identifier = 0x2ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
};

#else

/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
 *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
 *
 * @note  因为只用于发送,所以不需要在bsp_can中注册
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
static CANInstance sender_assignment[6] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [4] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [5] = {.can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
};

#endif

// 当前 FDCAN 三条总线各有三组发送帧；只给已有电机注册的组提交数据。
static uint8_t sender_enable_flag[9] = {0};

#define DJI_MULTITURN_MAX_GAP_MS 20u /* 超过此间隔无法从单圈编码器证明遗漏圈数。 */

static uint8_t FilterModeValid(DJI_Motor_Filter_Mode_e mode)
{
    return mode == DJI_MOTOR_FILTER_LEGACY_FRAME || mode == DJI_MOTOR_FILTER_TIME_CONSTANT ||
           mode == DJI_MOTOR_FILTER_DISABLED;
}

static uint8_t FilterConfigValid(const DJI_Motor_Filter_Config_s *config)
{
    if (config == NULL || !FilterModeValid(config->speed_mode) || !FilterModeValid(config->current_mode) ||
        !isfinite(config->dt_min_s) || !isfinite(config->dt_max_s) ||
        config->dt_min_s <= 0.0f || config->dt_max_s < config->dt_min_s ||
        config->dt_max_s > MOTOR_PID_DT_MAX_S)
        return 0u;

    const uint8_t speed_tau_valid = config->speed_mode == DJI_MOTOR_FILTER_TIME_CONSTANT
        ? isfinite(config->speed_tau_s) && config->speed_tau_s > 0.0f
        : config->speed_tau_s == 0.0f;
    const uint8_t current_tau_valid = config->current_mode == DJI_MOTOR_FILTER_TIME_CONSTANT
        ? isfinite(config->current_tau_s) && config->current_tau_s > 0.0f
        : config->current_tau_s == 0.0f;
    return speed_tau_valid && current_tau_valid;
}

static SampleFilterStatus_t InitTimeConstantFilter(SampleFilter_t *filter,
                                                   const DJI_Motor_Filter_Config_s *config,
                                                   float tau_s)
{
    const SampleFilterConfig_t filter_config = {
        .type = SAMPLE_FILTER_LOWPASS1,
        .dt_min_s = config->dt_min_s,
        .dt_max_s = config->dt_max_s,
        .tau_s = tau_s,
    };
    return SampleFilterInit(filter, &filter_config, NULL, 0u);
}

static uint8_t FeedbackSourceValid(const Motor_Signal_Source_s *source,
                                   Motor_Unit_e unit,
                                   Motor_Reference_e reference)
{
    return (uint8_t)(MotorFeedbackValidateSignalSource(source) == MOTOR_FEEDBACK_OK &&
                     source->unit == unit && source->reference == reference);
}

static uint8_t DjiConfigValid(const Motor_Init_Config_s *config)
{
    const Motor_Control_Setting_s *setting = &config->controller_setting_init_config;
    const Motor_Controller_Init_s *controller = &config->controller_param_init_config;
    Motor_Capabilities_s capabilities;
    if (MotorFeedbackValidateOutputAxis(&config->output_axis) != MOTOR_FEEDBACK_OK ||
        MotorFeedbackGetCapabilities(config->motor_type, &capabilities) != MOTOR_FEEDBACK_OK ||
        (capabilities.measurement_mask & MOTOR_CAPABILITY_CURRENT) == 0u ||
        setting->motor_reverse_flag != MOTOR_DIRECTION_NORMAL ||
        setting->feedback_reverse_flag != FEEDBACK_DIRECTION_NORMAL ||
        (setting->feedforward_flag & (uint8_t)~CURRENT_AND_SPEED_FEEDFORWARD) != 0u)
        return 0u;

    if ((setting->close_loop_type & ANGLE_LOOP) != 0u &&
        (setting->outer_loop_type == ANGLE_LOOP) &&
        (setting->angle_feedback_source == OTHER_FEED &&
         !FeedbackSourceValid(&controller->other_angle_feedback, MOTOR_UNIT_RAD,
                              MOTOR_REFERENCE_EXTERNAL)))
        return 0u;
    if ((setting->close_loop_type & SPEED_LOOP) != 0u &&
        (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) != 0u &&
        (setting->speed_feedback_source == OTHER_FEED &&
         !FeedbackSourceValid(&controller->other_speed_feedback, MOTOR_UNIT_RAD_S,
                              MOTOR_REFERENCE_EXTERNAL)))
        return 0u;
    if ((setting->feedforward_flag & SPEED_FEEDFORWARD) != 0u &&
        !FeedbackSourceValid(&controller->speed_feedforward, MOTOR_UNIT_RAD_S,
                             MOTOR_REFERENCE_OUTPUT_AXIS))
        return 0u;
    if ((setting->feedforward_flag & CURRENT_FEEDFORWARD) != 0u &&
        !FeedbackSourceValid(&controller->current_feedforward, MOTOR_UNIT_DEVICE_CURRENT,
                             MOTOR_REFERENCE_OUTPUT_AXIS))
        return 0u;
    if (setting->angle_feedback_source != MOTOR_FEED && setting->angle_feedback_source != OTHER_FEED)
        return 0u;
    if (setting->speed_feedback_source != MOTOR_FEED && setting->speed_feedback_source != OTHER_FEED)
        return 0u;
    return 1u;
}

static float FeedbackIntervalSeconds(const CANInstance *instance, const DJIMotorInstance *motor)
{
    const uint32_t elapsed_cycles = (uint32_t)(instance->rx_timestamp_cycles - motor->last_feedback_cycles);
    const uint32_t elapsed_ms = (uint32_t)(instance->rx_timestamp_ms - motor->last_feedback_timestamp_ms);
    const uint32_t frequency_mhz = dwt_cpu_freq_mhz;
    const float max_interval_ms = motor->filter_config.dt_max_s * 1000.0f;

    // 毫秒时间戳用于识别超过DWT单圈周期的长停顿；精确间隔仍使用帧入队时的周期差。
    if (frequency_mhz == 0u || (float)elapsed_ms > max_interval_ms + 1.0f)
        return INFINITY;
    return (float)elapsed_cycles / ((float)frequency_mhz * 1000000.0f);
}

static void UpdateFeedbackFilter(DJIMotorInstance *motor, uint8_t is_speed, float raw,
                                 float dt_s, float legacy_previous, float *effective)
{
    const DJI_Motor_Filter_Mode_e mode = is_speed ? motor->filter_config.speed_mode
                                                   : motor->filter_config.current_mode;
    SampleFilter_t *sample_filter = is_speed ? &motor->speed_filter : &motor->current_filter;
    DJI_Motor_Filter_Status_e *status = is_speed ? &motor->speed_filter_status
                                                  : &motor->current_filter_status;
    uint8_t *valid = is_speed ? &motor->speed_filter_valid : &motor->current_filter_valid;
    uint8_t *started = is_speed ? &motor->speed_filter_started : &motor->current_filter_started;
    uint8_t *needs_rebuild = is_speed ? &motor->speed_filter_needs_rebuild
                                       : &motor->current_filter_needs_rebuild;

    if (mode == DJI_MOTOR_FILTER_DISABLED)
    {
        *effective = raw;
        *status = DJI_FILTER_STATUS_BYPASSED;
        *valid = 1u;
        *started = 1u;
        *needs_rebuild = 0u;
        return;
    }

    if (mode == DJI_MOTOR_FILTER_TIME_CONSTANT)
    {
        float candidate = *effective;
        const SampleFilterStatus_t result = SampleFilterUpdate(sample_filter, 1u, raw, dt_s, &candidate);
        if (result == FILTER_OK || result == FILTER_REBUILT)
        {
            *effective = candidate;
            *status = result == FILTER_REBUILT ? DJI_FILTER_STATUS_REBUILT : DJI_FILTER_STATUS_VALID;
            *valid = 1u;
        }
        else
        {
            *status = result == FILTER_ERROR_DT ? DJI_FILTER_STATUS_GAP : DJI_FILTER_STATUS_ERROR;
            *valid = 0u;
        }
        *started = 1u;
        return;
    }

    if (!*started)
    {
        // 首帧仍从零开始按旧系数平滑，保持原DJI初始化后的默认响应。
        *effective = (1.0f - (is_speed ? SPEED_SMOOTH_COEF : CURRENT_SMOOTH_COEF)) * legacy_previous +
                     (is_speed ? SPEED_SMOOTH_COEF : CURRENT_SMOOTH_COEF) * raw;
        *started = 1u;
        *valid = 1u;
        *status = DJI_FILTER_STATUS_VALID;
        return;
    }

    if (!isfinite(dt_s) || dt_s < motor->filter_config.dt_min_s || dt_s > motor->filter_config.dt_max_s)
    {
        *valid = 0u;
        *status = DJI_FILTER_STATUS_GAP;
        *needs_rebuild = 1u;
        return;
    }

    if (*needs_rebuild)
    {
        // 间隔异常后的第一份连续样本直接建立常值起点，不混入停顿前历史。
        *effective = raw;
        *needs_rebuild = 0u;
        *valid = 1u;
        *status = DJI_FILTER_STATUS_REBUILT;
        return;
    }

    const float alpha = is_speed ? SPEED_SMOOTH_COEF : CURRENT_SMOOTH_COEF;
    const float candidate = (1.0f - alpha) * legacy_previous + alpha * raw;
    if (!isfinite(candidate))
    {
        *valid = 0u;
        *status = DJI_FILTER_STATUS_ERROR;
        *needs_rebuild = 1u;
        return;
    }
    *effective = candidate;
    *valid = 1u;
    *status = DJI_FILTER_STATUS_VALID;
}

static void InvalidateFeedbackForMissingSample(DJIMotorInstance *motor)
{
    motor->output_position_trusted = 0u;
    if (motor->speed_filter_valid)
    {
        if (motor->filter_config.speed_mode == DJI_MOTOR_FILTER_TIME_CONSTANT)
        {
            float unused_output = motor->measure.speed_aps;
            (void)SampleFilterUpdate(&motor->speed_filter, 0u, 0.0f, 0.0f, &unused_output);
        }
        motor->speed_filter_valid = 0u;
        motor->speed_filter_status = DJI_FILTER_STATUS_GAP;
        motor->speed_filter_needs_rebuild = 1u;
    }
    if (motor->current_filter_valid)
    {
        if (motor->filter_config.current_mode == DJI_MOTOR_FILTER_TIME_CONSTANT)
        {
            float unused_output = motor->measure.current_feedback;
            (void)SampleFilterUpdate(&motor->current_filter, 0u, 0.0f, 0.0f, &unused_output);
        }
        motor->current_filter_valid = 0u;
        motor->current_filter_status = DJI_FILTER_STATUS_GAP;
        motor->current_filter_needs_rebuild = 1u;
    }
}

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static uint8_t MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config)
{
    if (motor == NULL || config == NULL || config->can_handle == NULL ||
        config->tx_id == 0u || config->tx_id > 8u)
        return 0u;
    uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    uint8_t grouping_offset;
    //通过CAN计算分组偏移量
    if(config->can_handle == &hcan1)
    {
        grouping_offset=0;
    }
    else if(config->can_handle == &hcan2)
    {
        grouping_offset=3;
    }
    else
    {
#ifdef FDCAN
        if (config->can_handle == &hfdcan1)
            grouping_offset = 0u;
        else if (config->can_handle == &hfdcan2)
            grouping_offset = 3u;
        else if (config->can_handle == &hfdcan3)
            grouping_offset = 6u;
        else
            return 0u;
#else
        return 0u;
#endif
    }

    switch (motor->motor_type)
    {
    case M2006:
    case M3508:
        if (motor_id < 4) // 根据ID分组
        {
            motor_send_num = motor_id;
            motor_grouping = grouping_offset + 1;
            
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = grouping_offset + 0;
        }

        // 计算接收id并设置分组发送id
        config->rx_id = 0x200 + motor_id + 1;   // 把ID+1,进行分组设置
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        // 检查是否发生id冲突
        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                BSP_LOG_EVENT_0(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_01);
                return 0u;
            }
        }
        break;

    case GM6020:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = grouping_offset + 0;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = grouping_offset + 2;
        }

        config->rx_id = 0x204 + motor_id + 1;   // 把ID+1,进行分组设置
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                BSP_LOG_EVENT_0(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_02);
                return 0u;
            }
        }
        break;

    default: // other motors should not be registered here
        return 0u;
    }
    if (motor_grouping >= (sizeof(sender_enable_flag) / sizeof(sender_enable_flag[0])))
        return 0u;
    return 1u;
}

/**
 * @todo  是否可以简化多圈角度的计算？
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void DecodeDJIMotor(CANInstance *_instance)
{
    if (_instance == NULL || _instance->id == NULL || _instance->rx_len != 8u)
        return;
    uint8_t *rxbuff = _instance->rx_buff;
    DJIMotorInstance *motor = (DJIMotorInstance *)_instance->id;
    DJI_Motor_Measure_s *measure = &motor->measure;
    const uint8_t first_feedback = motor->has_feedback_timestamp == 0u;
    const uint32_t elapsed_ms = first_feedback ? 0u :
        (uint32_t)(_instance->rx_timestamp_ms - motor->last_feedback_timestamp_ms);
    const float dt_s = first_feedback ? 0.0f : FeedbackIntervalSeconds(_instance, motor);
    const int16_t raw_rpm = (int16_t)(((uint16_t)rxbuff[2] << 8u) | rxbuff[3]);
    const int16_t raw_current = (int16_t)(((uint16_t)rxbuff[4] << 8u) | rxbuff[5]);
    const float raw_speed_aps = RPM_2_ANGLE_PER_SEC * (float)raw_rpm;
    float candidate_speed = measure->speed_aps;
    float candidate_current = measure->current_feedback;

    DaemonReload(motor->daemon);

    // 原始量先独立保存；滤波失败时保留旧有效输出，但控制环会检查有效标记。
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->raw_speed_aps = raw_speed_aps;
    measure->raw_current = raw_current;
    motor->feedback_dt_s = dt_s;

    UpdateFeedbackFilter(motor, 1u, raw_speed_aps, dt_s, measure->speed_aps, &candidate_speed);
    UpdateFeedbackFilter(motor, 0u, (float)raw_current, dt_s, (float)measure->real_current, &candidate_current);

    if (motor->speed_filter_valid)
        measure->speed_aps = candidate_speed;
    if (motor->current_filter_valid)
    {
        if (motor->filter_config.current_mode == DJI_MOTOR_FILTER_LEGACY_FRAME)
            candidate_current = (float)(int16_t)candidate_current;
        if (candidate_current >= (float)INT16_MIN && candidate_current <= (float)INT16_MAX)
        {
            measure->current_feedback = candidate_current;
            // 保留原整数成员供旧诊断读取；滤波控制使用上面的浮点结果。
            measure->real_current = (int16_t)candidate_current;
        }
        else
        {
            motor->current_filter_valid = 0u;
            motor->current_filter_status = DJI_FILTER_STATUS_ERROR;
        }
    }
    measure->temperature = rxbuff[6];

    motor->feedback_dt_s = dt_s;
    motor->last_feedback_cycles = _instance->rx_timestamp_cycles;
    motor->last_feedback_timestamp_ms = _instance->rx_timestamp_ms;
    motor->has_feedback_timestamp = 1u;
    if (motor->feedback_sample_count < UINT32_MAX)
        motor->feedback_sample_count++;
    motor->feedback_sequence++;
    if (motor->feedback_sequence == 0u)
        motor->feedback_sequence++;

    if (motor->encoder_baseline_valid == 0u)
    {
        // 首个有效帧只建立相对零点，避免把上电时的编码器残值当作已知多圈角度。
        motor->encoder_baseline_valid = 1u;
        motor->output_position_trusted = 1u;
        measure->last_ecd = measure->ecd;
        measure->total_round = 0;
        measure->total_angle = 0.0f;
    }
    else if (motor->output_position_trusted != 0u && elapsed_ms <= DJI_MULTITURN_MAX_GAP_MS)
    {
        // 只有反馈连续时才补单圈跨界；计数溢出时撤销可信度，不做有符号溢出。
        const int32_t delta = (int32_t)measure->ecd - (int32_t)measure->last_ecd;
        if (delta > 4096)
        {
            if (measure->total_round == INT32_MIN)
                motor->output_position_trusted = 0u;
            else
                measure->total_round--;
        }
        else if (delta < -4096)
        {
            if (measure->total_round == INT32_MAX)
                motor->output_position_trusted = 0u;
            else
                measure->total_round++;
        }
        if (motor->output_position_trusted != 0u)
            measure->total_angle = (float)measure->total_round * 360.0f + measure->angle_single_round;
    }
    else
    {
        // 超过可证明补圈的间隔后，仅更新单圈基准；多圈位置等待显式重建。
        motor->output_position_trusted = 0u;
    }
}

static void DJIMotorLostCallback(void *motor_ptr)
{
    DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
    BSP_LOG_EVENT_2(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_03,
                    BSP_LOG_FIELD_I32((int)can_bus),
                    BSP_LOG_FIELD_I32((int)motor->motor_can_instance->tx_id));
}

// 电机初始化,返回一个电机实例
DJIMotorInstance *DJIMotorInit(const Motor_Init_Config_s *config)
{
    const DJI_Motor_Filter_Config_s default_filter = {
        .speed_mode = DJI_MOTOR_FILTER_LEGACY_FRAME,
        .current_mode = DJI_MOTOR_FILTER_LEGACY_FRAME,
        .speed_tau_s = 0.0f,
        .current_tau_s = 0.0f,
        .dt_min_s = MOTOR_PID_DT_MIN_S,
        .dt_max_s = MOTOR_PID_DT_MAX_S,
    };
    return DJIMotorInitWithFilter(config, &default_filter);
}

DJIMotorInstance *DJIMotorInitWithFilter(const Motor_Init_Config_s *config,
                                         const DJI_Motor_Filter_Config_s *filter_config)
{
    if (config == NULL || idx >= DJI_MOTOR_CNT || !FilterConfigValid(filter_config) ||
        !DjiConfigValid(config))
        return NULL;
    DJIMotorInstance *instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(DJIMotorInstance));
    instance->filter_config = *filter_config;

    if ((filter_config->speed_mode == DJI_MOTOR_FILTER_TIME_CONSTANT &&
         InitTimeConstantFilter(&instance->speed_filter, filter_config, filter_config->speed_tau_s) != FILTER_OK) ||
        (filter_config->current_mode == DJI_MOTOR_FILTER_TIME_CONSTANT &&
         InitTimeConstantFilter(&instance->current_filter, filter_config, filter_config->current_tau_s) != FILTER_OK))
    {
        free(instance);
        return NULL;
    }

    // motor basic setting 电机基本设置
    instance->motor_type = config->motor_type;                         // 6020 or 2006 or 3508
    instance->output_axis = config->output_axis;
    instance->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等

    // motor controller init 电机控制器初始化
    // 只检查实际启用的环，失败发生在注册 CAN 和发送模式命令之前。
    if (MotorPIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID,
                     (config->controller_setting_init_config.close_loop_type & CURRENT_LOOP) != 0u) != PID_OK ||
        MotorPIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID,
                     (config->controller_setting_init_config.close_loop_type & SPEED_LOOP) != 0u) != PID_OK ||
        MotorPIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID,
                     (config->controller_setting_init_config.close_loop_type & ANGLE_LOOP) != 0u) != PID_OK)
    {
        free(instance);
        return NULL;
    }
    instance->motor_controller.other_angle_feedback = config->controller_param_init_config.other_angle_feedback;
    instance->motor_controller.other_speed_feedback = config->controller_param_init_config.other_speed_feedback;
    instance->motor_controller.current_feedforward = config->controller_param_init_config.current_feedforward;
    instance->motor_controller.speed_feedforward = config->controller_param_init_config.speed_feedforward;
    // 后续增加电机前馈控制器(速度和电流)

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    CAN_Init_Config_s can_config = config->can_init_config;
    if (!MotorSenderGrouping(instance, &can_config))
    {
        free(instance);
        return NULL;
    }

    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .callback = DJIMotorLostCallback,
        .owner_id = instance,
        .reload_count = 2, // 20ms未收到数据则丢失
    };
    instance->daemon = DaemonRegister(&daemon_config);
    if (instance->daemon == NULL)
    {
        free(instance);
        return NULL;
    }

    // 最后注册 CAN；失败时撤销先前离线计数并释放电机，避免残留回调。
    can_config.can_module_callback = DecodeDJIMotor;
    can_config.id = instance;
    instance->motor_can_instance = CANRegister(&can_config);
    if (instance->motor_can_instance == NULL)
    {
        (void)DaemonUnregister(instance->daemon);
        free(instance);
        return NULL;
    }

    // 安全默认态: 初始化后保持失能, 由上层状态机在确认IMU/标定/使能条件后
    // 通过 DJIMotorEnable() 显式恢复输出
    DJIMotorStop(instance);
    dji_motor_instance[idx++] = instance;
    sender_enable_flag[instance->sender_group] = 1u;
    return instance;
}

/* 电流只能通过电机自带传感器监测,后续考虑加入力矩传感器应变片等 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (motor == NULL || (type != MOTOR_FEED && type != OTHER_FEED))
        return;
    taskENTER_CRITICAL();
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
    {
        taskEXIT_CRITICAL();
        BSP_LOG_EVENT_0(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_04); // 检查是否传入了正确的LOOP类型,或发生了指针越界
        return;
    }
    taskEXIT_CRITICAL();
}

void DJIMotorStop(DJIMotorInstance *motor)
{
    taskENTER_CRITICAL();
    motor->stop_flag = MOTOR_STOP;
    taskEXIT_CRITICAL();
}

void DJIMotorEnable(DJIMotorInstance *motor)
{
    taskENTER_CRITICAL();
    motor->stop_flag = MOTOR_ENALBED;
    taskEXIT_CRITICAL();
}

/* 修改电机的实际闭环对象 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    taskENTER_CRITICAL();
    motor->motor_settings.outer_loop_type = outer_loop;
    taskEXIT_CRITICAL();
}

/**
 * @brief 判断电机是否离线
 *
 */
void DJIMotorIsOnline(DJIMotorInstance *motor)
{
    if (DaemonIsOnline(motor->daemon) > 0)
        motor->online_flag = MOTOR_ONLINE;
    else
        motor->online_flag = MOTOR_OFFLINE;
}

/**
 * @brief 修改电机的实际闭环目标(内层闭环)
 *
 * @param motor  要修改的电机实例指针
 * @param outer_loop 闭环类型(用于设置 close_loop_type)
 */
void DJIMotorCloseLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    taskENTER_CRITICAL();
    motor->motor_settings.close_loop_type = outer_loop;
    taskEXIT_CRITICAL();
}

static void StoreSignal(Motor_Signal_Snapshot_s *signal, float value, Motor_Unit_e unit,
                        Motor_Reference_e reference, uint32_t timestamp_ms,
                        uint32_t sequence, uint8_t valid)
{
    *signal = (Motor_Signal_Snapshot_s){
        .value = value,
        .unit = unit,
        .reference = reference,
        .source_time_ms = timestamp_ms,
        .sequence = sequence,
        .valid = valid,
    };
}

static PID_Status_e ReadSignalForPID(const Motor_Signal_Source_s *source,
                                    Motor_Unit_e unit,
                                    Motor_Reference_e reference,
                                    uint32_t now_ms,
                                    float *value)
{
    Motor_Signal_Snapshot_s snapshot = {0};
    if (value == NULL)
        return PID_BAD_ARGUMENT;
    const Motor_Feedback_Status_e status = MotorFeedbackReadSignal(
        source, unit, reference, now_ms, &snapshot);
    if (status == MOTOR_FEEDBACK_NONFINITE || status == MOTOR_FEEDBACK_OVERFLOW)
        return PID_NUMERIC_ERROR;
    if (status != MOTOR_FEEDBACK_OK)
        return PID_BAD_ARGUMENT;
    *value = snapshot.value;
    return PID_OK;
}

Motor_Feedback_Status_e DJIMotorRebaseReference(DJIMotorInstance *motor, uint32_t now_ms)
{
    if (motor == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;

    uint32_t timestamp_ms;
    uint8_t has_sample;
    taskENTER_CRITICAL();
    timestamp_ms = motor->last_feedback_timestamp_ms;
    has_sample = motor->has_feedback_timestamp;
    taskEXIT_CRITICAL();
    if (has_sample == 0u || DaemonIsOnline(motor->daemon) == 0u ||
        (uint32_t)(now_ms - timestamp_ms) > DJI_MULTITURN_MAX_GAP_MS)
        return MOTOR_FEEDBACK_NOT_READY;

    taskENTER_CRITICAL();
    motor->measure.last_ecd = motor->measure.ecd;
    motor->measure.total_round = 0;
    motor->measure.total_angle = 0.0f;
    motor->encoder_baseline_valid = 1u;
    motor->output_position_trusted = 1u;
    taskEXIT_CRITICAL();
    return MOTOR_FEEDBACK_OK;
}

Motor_Feedback_Status_e DJIMotorGetFeedbackSnapshot(const DJIMotorInstance *motor,
                                                     uint32_t now_ms,
                                                     Motor_Feedback_Snapshot_s *snapshot)
{
    if (motor == NULL || snapshot == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;

    DJI_Motor_Measure_s measure;
    Motor_Output_Axis_Config_s output_axis;
    uint32_t timestamp_ms;
    uint32_t sequence;
    uint8_t has_sample;
    uint8_t position_trusted;
    uint8_t speed_valid;
    uint8_t current_valid;
    taskENTER_CRITICAL();
    measure = motor->measure;
    output_axis = motor->output_axis;
    timestamp_ms = motor->last_feedback_timestamp_ms;
    sequence = motor->feedback_sequence;
    has_sample = motor->has_feedback_timestamp;
    position_trusted = motor->encoder_baseline_valid && motor->output_position_trusted;
    speed_valid = motor->speed_filter_valid;
    current_valid = motor->current_filter_valid;
    taskEXIT_CRITICAL();
    if (has_sample == 0u)
        return MOTOR_FEEDBACK_NOT_READY;

    float position_rad = 0.0f;
    float speed_rad_s = 0.0f;
    Motor_Feedback_Status_e status = MotorFeedbackConvertDegreesToOutputAxis(
        &output_axis, measure.total_angle, measure.speed_aps, &position_rad, &speed_rad_s);
    if (status != MOTOR_FEEDBACK_OK)
        return status;

    const uint8_t fresh = (DaemonIsOnline(motor->daemon) != 0u) &&
        ((uint32_t)(now_ms - timestamp_ms) <= DJI_MULTITURN_MAX_GAP_MS);
    Motor_Feedback_Snapshot_s candidate = {0};
    candidate.sample_sequence = sequence;
    StoreSignal(&candidate.position, position_rad, MOTOR_UNIT_RAD, MOTOR_REFERENCE_OUTPUT_AXIS,
                timestamp_ms, sequence, (uint8_t)(fresh && position_trusted));
    StoreSignal(&candidate.speed, speed_rad_s, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_OUTPUT_AXIS,
                timestamp_ms, sequence, (uint8_t)(fresh && speed_valid));
    StoreSignal(&candidate.current, measure.current_feedback * (float)output_axis.direction,
                MOTOR_UNIT_DEVICE_CURRENT,
                MOTOR_REFERENCE_OUTPUT_AXIS, timestamp_ms, sequence,
                (uint8_t)(fresh && current_valid));
    StoreSignal(&candidate.torque, 0.0f, MOTOR_UNIT_UNSPECIFIED,
                MOTOR_REFERENCE_UNSPECIFIED, timestamp_ms, sequence, 0u);
    StoreSignal(&candidate.voltage, 0.0f, MOTOR_UNIT_UNSPECIFIED,
                MOTOR_REFERENCE_UNSPECIFIED, timestamp_ms, sequence, 0u);
    *snapshot = candidate;
    return MOTOR_FEEDBACK_OK;
}

void DJIMotorGetSummary(DJIMotorInstance *motor, DJIMotorSummary_t *summary)
{
    if (motor == NULL || summary == NULL)
        return;

    summary->online = (DaemonIsOnline(motor->daemon) > 0) ? 1 : 0;
    summary->enabled = (motor->stop_flag == MOTOR_ENALBED) ? 1 : 0;
    summary->speed_aps = motor->measure.speed_aps;
    summary->angle_single_round = motor->measure.angle_single_round;
    summary->total_angle = motor->measure.total_angle;
    summary->current = motor->measure.real_current;
    summary->temperature = motor->measure.temperature;
    summary->raw_speed_aps = motor->measure.raw_speed_aps;
    summary->raw_current = motor->measure.raw_current;
    summary->current_feedback = motor->measure.current_feedback;
    summary->feedback_dt_s = motor->feedback_dt_s;
    summary->feedback_sample_count = motor->feedback_sample_count;
    summary->speed_filter_status = motor->speed_filter_status;
    summary->current_filter_status = motor->current_filter_status;
    summary->speed_filter_valid = motor->speed_filter_valid;
    summary->current_filter_valid = motor->current_filter_valid;
}

// 设置参考值
void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{
    if (motor == NULL)
        return;
    taskENTER_CRITICAL();
    motor->motor_controller.pid_ref = ref;
    taskEXIT_CRITICAL();
}

// 为所有电机实例计算三环PID,发送控制报文
void DJIMotorControl()
{
    uint32_t probe_all = DWT_ProbeStart();
    // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
    uint8_t group, num; // 电机组号和组内编号
    int16_t set;        // 电机控制CAN发送设定值
    DJIMotorInstance *motor;
    Motor_Control_Setting_s setting_snapshot; // 本周期使用同一份控制设置。
    Motor_Controller_s *motor_controller;   // 电机控制器
    DJI_Motor_Measure_s measure_snapshot;   // 本周期使用同一帧 CAN 反馈。
    Motor_Output_Axis_Config_s output_axis;
    float pid_measure, pid_ref;             // 电机PID测量值和设定值

    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < idx; ++i)
    { // 减小访存开销,先保存指针引用
        motor = dji_motor_instance[i];
        motor_controller = &motor->motor_controller;
        if (motor->has_feedback_timestamp && DaemonIsOnline(motor->daemon) == 0u)
            InvalidateFeedbackForMissingSample(motor);
        uint8_t speed_filter_valid;
        uint8_t current_filter_valid;
        uint8_t position_trusted;
        taskENTER_CRITICAL();
        setting_snapshot = motor->motor_settings;
        measure_snapshot = motor->measure;
        output_axis = motor->output_axis;
        pid_ref = motor_controller->pid_ref;
        speed_filter_valid = motor->speed_filter_valid;
        current_filter_valid = motor->current_filter_valid;
        position_trusted = (uint8_t)(motor->encoder_baseline_valid && motor->output_position_trusted);
        taskEXIT_CRITICAL();
        const Motor_Control_Setting_s *motor_setting = &setting_snapshot;
        DJI_Motor_Measure_s *measure = &measure_snapshot;
        const uint32_t now_ms = HAL_GetTick();
        // 首轮或间隔异常时仅重新计时，最后统一将本电机输出写0。
        float control_dt = 0.0f;
        PID_Status_e pid_status = MotorPIDBegin(&motor->pid_clock, DWT_GetTimeline_us(), &control_dt);
        const uint8_t axis_closed_loop = (uint8_t)(
            (motor_setting->outer_loop_type == ANGLE_LOOP &&
             (motor_setting->close_loop_type & ANGLE_LOOP) != 0u) ||
            (motor_setting->outer_loop_type == SPEED_LOOP &&
             (motor_setting->close_loop_type & SPEED_LOOP) != 0u));
        const uint8_t speed_internal_active =
            (motor_setting->close_loop_type & SPEED_LOOP) != 0u &&
            (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) != 0u &&
            motor_setting->speed_feedback_source == MOTOR_FEED;
        const uint8_t current_internal_active =
            (motor_setting->close_loop_type & CURRENT_LOOP) != 0u;
        const uint8_t feedback_valid =
            (!speed_internal_active || speed_filter_valid) &&
            (!current_internal_active || current_filter_valid);
        float output_position_rad = 0.0f;
        float output_speed_rad_s = 0.0f;
        const Motor_Feedback_Status_e axis_status = MotorFeedbackConvertDegreesToOutputAxis(
            &output_axis, measure->total_angle, measure->speed_aps,
            &output_position_rad, &output_speed_rad_s);
        const uint8_t internal_axis_active =
            ((motor_setting->outer_loop_type == ANGLE_LOOP) &&
             (motor_setting->close_loop_type & ANGLE_LOOP) != 0u &&
             motor_setting->angle_feedback_source == MOTOR_FEED) ||
            ((motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) != 0u &&
             (motor_setting->close_loop_type & SPEED_LOOP) != 0u &&
             motor_setting->speed_feedback_source == MOTOR_FEED);
        if (axis_status != MOTOR_FEEDBACK_OK && internal_axis_active && pid_status == PID_OK)
            pid_status = PID_NUMERIC_ERROR;

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if (pid_status == PID_OK && feedback_valid && (motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_status = ReadSignalForPID(&motor_controller->other_angle_feedback,
                    MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL, now_ms, &pid_measure);
            else if (motor_setting->angle_feedback_source == MOTOR_FEED && position_trusted != 0u)
                pid_measure = output_position_rad;
            else if (motor_setting->angle_feedback_source == MOTOR_FEED)
                pid_status = PID_BAD_ARGUMENT; // 多圈参考失效后必须显式重建，不能拿旧值继续闭环。
            else
                pid_status = PID_BAD_ARGUMENT;
            // 更新pid_ref进入下一个环
            if (pid_status == PID_OK)
                pid_status = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref, control_dt, &pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if (pid_status == PID_OK && feedback_valid && (motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
            {
                float feedforward = 0.0f;
                pid_status = ReadSignalForPID(&motor_controller->speed_feedforward,
                    MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_OUTPUT_AXIS, now_ms, &feedforward);
                if (pid_status == PID_OK)
                    pid_ref += feedforward;
            }

            if (pid_status == PID_OK && motor_setting->speed_feedback_source == OTHER_FEED)
                pid_status = ReadSignalForPID(&motor_controller->other_speed_feedback,
                    MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_EXTERNAL, now_ms, &pid_measure);
            else if (pid_status == PID_OK && motor_setting->speed_feedback_source == MOTOR_FEED)
                pid_measure = output_speed_rad_s;
            else if (pid_status == PID_OK)
                pid_status = PID_BAD_ARGUMENT;
            // 更新pid_ref进入下一个环
            if (pid_status == PID_OK)
                pid_status = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref, control_dt, &pid_ref);
        }

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        if (pid_status == PID_OK && feedback_valid &&
            (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD))
        {
            float feedforward = 0.0f;
            pid_status = ReadSignalForPID(&motor_controller->current_feedforward,
                MOTOR_UNIT_DEVICE_CURRENT, MOTOR_REFERENCE_OUTPUT_AXIS, now_ms, &feedforward);
            if (pid_status == PID_OK)
                pid_ref += feedforward;
        }
        if (motor_setting->close_loop_type & CURRENT_LOOP)
        {
            if (pid_status == PID_OK && feedback_valid)
            {
                const float current_measure = measure->current_feedback *
                    ((axis_closed_loop != 0u && output_axis.direction == MOTOR_AXIS_DIRECTION_REVERSE) ? -1.0f : 1.0f);
                pid_status = PIDCalculate(&motor_controller->current_PID, current_measure,
                                          pid_ref, control_dt, &pid_ref);
            }
        }

        // 输出轴控制量按安装方向映射回电机转子方向后再编码设备电流。
        if (axis_closed_loop != 0u && output_axis.direction == MOTOR_AXIS_DIRECTION_REVERSE)
            pid_ref *= -1;

        if (pid_status != PID_OK || !feedback_valid || !isfinite(pid_ref))
        {
            // 一环失败后丢弃本轮串级结果，恢复前清除各环已有积分和微分历史。
            MotorPIDResetLoops(&motor_controller->angle_PID, &motor_controller->speed_PID, &motor_controller->current_PID);
            motor->pid_clock.last_status = pid_status != PID_OK ? pid_status : PID_NUMERIC_ERROR;
            pid_ref = 0.0f;
        }

        // 获取最终输出,功率限制时直接输出电流值
        // 先把最终输出限制在本实现采用的 -16384～16384，再转成 int16_t，避免超范围转换。
        // 不同电调的电流或电压比例另按协议确认，这里没有统一换算为 A。
        if (pid_ref > 16384.0f)
        {
            pid_ref = 16384.0f;
            dji_current_sat_count++;
        }
        else if (pid_ref < -16384.0f)
        {
            pid_ref = -16384.0f;
            dji_current_sat_count++;
        }
        set = (int16_t)pid_ref;

        // 分组填入发送数据
        group = motor->sender_group;
        num = motor->message_num;
        sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8);         // 高八位。
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // 低八位。

        // 若该电机处于停止状态,直接将buff置零
        // 即使前面完成 PID 计算，停止标记仍覆盖对应的两个发送字节。
        if (motor->stop_flag == MOTOR_STOP)
            memset(sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
    }

    // 遍历flag,检查是否要发送这一帧报文
    DWT_ProbeDone(&dji_prof_pid, probe_all);
    uint32_t probe_send = DWT_ProbeStart();
#ifdef FDCAN
    for (size_t i = 0; i < 9; ++i)
#else
    for (size_t i = 0; i < 6; ++i)
#endif
    {
        if (sender_enable_flag[i])
        {
#ifdef FDCAN
            const uint32_t identifier = sender_assignment[i].txconf.Identifier;
#else
            const uint32_t identifier = sender_assignment[i].txconf.StdId;
#endif
            const CAN_TxFrame_t frame = {
                .identifier = identifier,
                .identifier_type = CAN_TX_ID_STANDARD,
                .data = sender_assignment[i].tx_buff,
                .length = 8u,
            };
            uint32_t request_id = 0u;
            (void)CANSubmitPeriodic(sender_assignment[i].can_handle,
                                    &sender_assignment[i], 0u, &frame,
                                    CAN_TX_PRIORITY_CONTROL, 4u, 1000u, &request_id);
        }
    }
    DWT_ProbeDone(&dji_prof_send, probe_send);
    DWT_ProbeDone(&dji_prof_all, probe_all);
}

/* ============================================================================
   DJI 假反馈注入自测(默认关闭, 开关见 dji_motor.h 的 CAN_DJI_INJECT_TEST)
   ========================================================================== */
#if CAN_DJI_INJECT_TEST >= 1

volatile uint32_t can_dji_inject_pass = 0;
volatile uint32_t can_dji_inject_fail = 0;
volatile uint32_t can_dji_inject_multi_turn_err = 0;
volatile uint32_t can_dji_inject_frame_cnt = 0;

static uint8_t s_inject_source;                     /* 固定来源身份，不持有发送缓存 */
static uint8_t s_inject_frame[8];
static DJIMotorInstance *s_inject_target = NULL;    /* 被注入的电机 */
static DJI_Motor_Measure_s s_inject_backup;         /* 测试前的 measure, 结束后还原 */
static FDCAN_HandleTypeDef *s_inject_h = NULL;

/* 按 DJI 反馈帧格式填好 tx_buff 并发出(不等待接收) */
static void DJIInjectSend(uint16_t ecd, int16_t speed_rpm, int16_t current_raw, uint8_t temp)
{
    if (s_inject_h == NULL)
        return;
    s_inject_frame[0] = (uint8_t)(ecd >> 8);
    s_inject_frame[1] = (uint8_t)(ecd & 0xFFu);
    s_inject_frame[2] = (uint8_t)(((uint16_t)speed_rpm) >> 8);
    s_inject_frame[3] = (uint8_t)(((uint16_t)speed_rpm) & 0xFFu);
    s_inject_frame[4] = (uint8_t)(((uint16_t)current_raw) >> 8);
    s_inject_frame[5] = (uint8_t)(((uint16_t)current_raw) & 0xFFu);
    s_inject_frame[6] = temp;
    s_inject_frame[7] = 0u;
    can_dji_inject_frame_cnt++;
    const CAN_TxFrame_t frame = {
        .identifier = s_inject_target->motor_can_instance->rx_id,
        .identifier_type = CAN_TX_ID_STANDARD,
        .data = s_inject_frame,
        .length = 8u,
    };
    uint32_t request_id = 0u;
    if (CANSubmitOneShot(s_inject_h, &s_inject_source, &frame,
                         CAN_TX_PRIORITY_URGENT, 3u, &request_id) != CAN_TX_SUBMIT_ACCEPTED)
        can_dji_inject_fail++;
}

/* 边消费接收队列边等, 直到目标电机的 ecd 变成期望值; 返回 1=等到 */
static uint8_t DJIInjectWaitEcd(uint16_t expect, uint32_t timeout_us)
{
    uint32_t t0 = DWT_ProbeStart();
    while (DWT_ProbeElapsedUs(t0) < timeout_us)
    {
        CANProcessRx();
        CANProcessTx();
        if (s_inject_target != NULL && s_inject_target->measure.ecd == expect)
            return 1u;
    }
    return 0u;
}

uint8_t CANRunDjiFeedbackInjectTest(void)
{
    can_dji_inject_pass = 0;
    can_dji_inject_fail = 0;
    can_dji_inject_multi_turn_err = 0;
    can_dji_inject_frame_cnt = 0;

    if (idx == 0u || dji_motor_instance[0] == NULL)
    {
        BSP_LOG_EVENT_0(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_05);
        return 0u;
    }

    s_inject_target = dji_motor_instance[0];
    s_inject_h = s_inject_target->motor_can_instance->can_handle;
    uint32_t target_rx_id = s_inject_target->motor_can_instance->rx_id;
    s_inject_backup = s_inject_target->measure;

    /* 1. 切内部回环: 帧不会真的上线, 但自己发自己收, 不会干扰总线上的其它设备 */
    if (CANForceMode(s_inject_h, FDCAN_MODE_INTERNAL_LOOPBACK, 0u) == 0u)
    {
        BSP_LOG_EVENT_0(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_06);
        /* CANForceMode 失败时可能已经把总线 Stop 掉了, 这里必须恢复一次,
           否则这条总线上线不了, 后面的电机控制帧全都发不出去。 */
        (void)CANForceMode(s_inject_h, FDCAN_MODE_NORMAL, 0u);
        CANReapplyFilters();
        return 0u;
    }

    /* 关键: CANForceMode 内部的 HAL_FDCAN_Init() 会重配消息 RAM, 把各电机实例
       原先注册的接收滤镜一起清掉。不重装的话, 我们发出去的 0x201 根本没有滤镜
       去接, 注入帧会被硬件直接丢弃(实测: 5 项全失败、只发出 4 帧就是这个原因)。
       回环自测不需要这一步, 因为它注册测试实例时自己会加滤镜。 */
    CANReapplyFilters();

    /* 发送接口按已运行总线句柄提交复制帧，无需制造无接收用途的注册实例。 */
    (void)target_rx_id;

    /* ---- 测试 1: 单帧解码(ecd / 温度) ---- */
    DJIInjectSend(1000u, 100, 50, 42u);
    if (DJIInjectWaitEcd(1000u, 3000u) && s_inject_target->measure.temperature == 42u)
        can_dji_inject_pass++;
    else
        can_dji_inject_fail++;

    /* ---- 测试 2: 正向跨圈。ecd 1000→8100, 差 +7100 > 4096(半圈) → total_round 应 -1 ---- */
    int32_t r0 = s_inject_target->measure.total_round;
    DJIInjectSend(8100u, 0, 0, 42u);
    if (DJIInjectWaitEcd(8100u, 3000u) && s_inject_target->measure.total_round == r0 - 1)
        can_dji_inject_pass++;
    else
    {
        can_dji_inject_fail++;
        can_dji_inject_multi_turn_err++;
    }

    /* ---- 测试 3: 反向跨圈。ecd 8100→1000, 差 -7100 < -4096 → total_round 应回到 r0 ---- */
    DJIInjectSend(1000u, 0, 0, 42u);
    if (DJIInjectWaitEcd(1000u, 3000u) && s_inject_target->measure.total_round == r0)
        can_dji_inject_pass++;
    else
    {
        can_dji_inject_fail++;
        can_dji_inject_multi_turn_err++;
    }

    /* ---- 测试 4: total_angle 与 (total_round, ecd) 一致 ---- */
    float expect_angle = (float)(r0 * 360) + ECD_ANGLE_COEF_DJI * 1000.0f;
    if (fabsf(s_inject_target->measure.total_angle - expect_angle) < 0.5f)
        can_dji_inject_pass++;
    else
        can_dji_inject_fail++;

    /* ---- 测试 5: 连续多帧的 ecd 序列(验证逐帧、按到达顺序被处理) ---- */
    uint8_t seq_ok = 1u;
    for (uint16_t k = 0; k < 20u; ++k)
    {
        uint16_t e = (uint16_t)((1000u + k * 13u) & 0x1FFFu);
        DJIInjectSend(e, 0, 0, 42u);
        if (!DJIInjectWaitEcd(e, 2000u))
        {
            seq_ok = 0u;
            break;
        }
    }
    if (seq_ok)
        can_dji_inject_pass++;
    else
        can_dji_inject_fail++;

#if CAN_DJI_INJECT_TEST >= 2
    /* 压测模式: 保留回环与注入实例, 由 daemon 循环持续注入, 用来观察队列深度与丢帧 */
    BSP_LOG_EVENT_2(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_08, BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_pass), BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_fail));
#else
    /* 收尾: 还原 measure、切回正常模式并重装滤镜 */
    s_inject_target->measure = s_inject_backup;
    (void)CANForceMode(s_inject_h, FDCAN_MODE_NORMAL, 0u);
    CANReapplyFilters();
    s_inject_h = NULL;
#endif

    BSP_LOG_EVENT_4(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_09, BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_pass), BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_fail), BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_multi_turn_err), BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_frame_cnt));
    return (can_dji_inject_fail == 0u) ? 1u : 0u;
}

#if CAN_DJI_INJECT_TEST >= 2
void DJIInjectStressTick(void)
{
    if (s_inject_h == NULL)
        return;
    /* 每轮塞一小批帧(不等待消费), 制造"任务还没来得及消费"的场景,
       从而观察 rxq_peak(队列最大占用)与 rxq_drop(队列满丢帧)。 */
    static uint16_t ecd = 0u;
    for (uint8_t n = 0; n < 10u; ++n)
    {
        ecd = (uint16_t)((ecd + 7u) & 0x1FFFu);
        DJIInjectSend(ecd, 50, 20, 40u);
    }
}
#endif

#endif /* CAN_DJI_INJECT_TEST >= 1 */
