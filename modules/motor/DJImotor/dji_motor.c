/** @file dji_motor.c
* @brief 解码 DJI 电调反馈，逐电机计算参考输出，并按总线与设备编号组成四电机 CAN 帧。
 */
#include "dji_motor.h"
#include <string.h>
#include "general_def.h"
#include "../../../bsp/dwt/platform_probe.h"
#include "../../../bsp/os/platform_sync.h"
#include "../../../bsp/os/platform_irq.h"
#include "bsp_log.h"
#include "motor_pid.h"
#include "../motor_task.h"
#include "dji_motor_diagnostic_internal.h" // 回环诊断只查询已注册的第一个电机。
#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
#include "robot_safety.h"
#endif
#include <math.h>
#include <stdlib.h>

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用

/* ---------------- 性能探针(2026-09, CPU 占用排查) ----------------
   dji_prof_all : 单次 DJIMotorControl 总耗时
   dji_prof_pid : 串级 PID 计算 + 组帧(不含 CAN 发送)
   dji_prof_send: CAN 提交耗时；bsp_can 当前发送空间满即返回，不等待。 */
volatile PlatformProbe_t dji_prof_all = {0};
volatile PlatformProbe_t dji_prof_pid = {0};
volatile PlatformProbe_t dji_prof_send = {0};
volatile uint32_t dji_current_sat_count = 0; /* 电流指令被饱和的次数(MOTOR-09 观测) */
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算

#if CAN_DJI_INJECT_TEST >= 1
DJIMotorInstance *DJIMotorDiagnosticFirst(void)
{
    return idx == 0u ? NULL : dji_motor_instance[0]; // 注册表为空时不提供诊断对象。
}
#endif


// 每条逻辑总线按三个 ID 分组；组对象的地址长期稳定，供队列识别周期来源。
typedef struct {
    PlatformCanBus_t bus; // 车型使用的逻辑总线。
    uint32_t identifier; // 四个电机共用的标准帧 ID。
    uint8_t tx_buff[8]; // 每电机占两个字节，提交时由队列复制。
} DJISendGroup_t;
static DJISendGroup_t sender_assignment[9] = {
    {PLATFORM_CAN_BUS_1, 0x1ffu, {0}}, // 总线 1 的 0x1ff 组。
    {PLATFORM_CAN_BUS_1, 0x200u, {0}}, // 总线 1 的 0x200 组。
    {PLATFORM_CAN_BUS_1, 0x2ffu, {0}}, // 总线 1 的 0x2ff 组。
    {PLATFORM_CAN_BUS_2, 0x1ffu, {0}}, // 总线 2 的 0x1ff 组。
    {PLATFORM_CAN_BUS_2, 0x200u, {0}}, // 总线 2 的 0x200 组。
    {PLATFORM_CAN_BUS_2, 0x2ffu, {0}}, // 总线 2 的 0x2ff 组。
    {PLATFORM_CAN_BUS_3, 0x1ffu, {0}}, // 总线 3 的 0x1ff 组。
    {PLATFORM_CAN_BUS_3, 0x200u, {0}}, // 总线 3 的 0x200 组。
    {PLATFORM_CAN_BUS_3, 0x2ffu, {0}}, // 总线 3 的 0x2ff 组。
};

static uint8_t sender_enable_flag[9] = {0};
#if defined(ONE_BOARD)
static uint32_t sender_safety_generation[9]; // 每组已排队目标所属的安全代际。
static DJIChassisReferenceSourceCheck_t chassis_source_check; // 底盘启动时登记，同一固件运行期间使用同一检查函数。
enum {
    DJI_QUEUED_SPEED = 1u << 0, // 依赖本电机的速度反馈。
    DJI_QUEUED_CURRENT = 1u << 1, // 依赖本电机的电流反馈。
    DJI_QUEUED_POSITION = 1u << 2, // 依赖本电机可信的多圈位置。
    DJI_QUEUED_REFERENCE = 1u << 3, // 拨弹参考属于本次计算使用的安全代际。
    DJI_QUEUED_DEADLINE = 1u << 4, // 已按实际使用的数据计算到期时刻。
    DJI_QUEUED_REFEREE_POWER = 1u << 5, // 外部编号来自裁判供电，不能当作方向序号。
};
typedef struct {
    uint32_t deadline_ms[4]; // 每个槽位按原始采样时间计算的最早到期时刻。
    uint32_t gap_generation[4]; // 每个目标使用反馈时看到的间断代次。
    uint32_t chassis_stop_generation[4]; // 四个槽位计算时各自看到的底盘停止代次。
    uint32_t source_sequence[4]; // 保存各槽位计算时的外部底盘来源序号，0 表示无此依赖。
    uint8_t requirements[4]; // 每个槽位需要复查的反馈、参考及有效期标记。
} DJI_Queued_Feedback_s;
static DJI_Queued_Feedback_s sender_feedback[9]; // 每组当前排队目标的反馈依据。
#endif

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
    if (config->dji_chassis_output_limit_raw > DJI_OUTPUT_MAX_RAW ||
        (config->dji_chassis_output_limit_raw != 0u && config->output_role != MOTOR_OUTPUT_CHASSIS))
        return 0u; // 专用上限只能用于底盘，且不得超出驱动的报文范围。
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

static float FeedbackIntervalSeconds(const PlatformCanRxFrame_t *frame, const DJIMotorInstance *motor)
{
    const uint32_t elapsed_ms = (uint32_t)(frame->received_ms - motor->last_feedback_timestamp_ms);
    const float max_interval_ms = motor->filter_config.dt_max_s * 1000.0f;
    float seconds;

    // 毫秒时间戳用于识别超过DWT单圈周期的长停顿；精确间隔仍使用帧入队时的周期差。
    if ((float)elapsed_ms > max_interval_ms + 1.0f ||
        !PlatformProbeElapsedSeconds(motor->last_feedback_cycles, frame->received_ticks, &seconds))
        return INFINITY;
    return seconds;
}

static void UpdateFeedbackFilter(DJIMotorInstance *motor, uint8_t is_speed, float raw,
                                 float dt_s, float legacy_previous, float *effective,
                                 DJI_Motor_Filter_Status_e *status, uint8_t *valid)
{
    const DJI_Motor_Filter_Mode_e mode = is_speed ? motor->filter_config.speed_mode
                                                   : motor->filter_config.current_mode;
    SampleFilter_t *sample_filter = is_speed ? &motor->speed_filter : &motor->current_filter;
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

// 调用方已保护电机反馈数据；底盘轮位共用整组许可。
static void RecordFeedbackGap(DJIMotorInstance *motor)
{
    ++motor->feedback_gap_generation; // 发送前复核和反馈汇总都能识别已恢复的短断流。
#if defined(ONE_BOARD)
    if (motor->output_role == MOTOR_OUTPUT_CHASSIS)
        RobotSafetyRevokeVehicleFeedback(ROBOT_CONTROL_CHASSIS); // 单轮失效立即撤销整组，覆盖其他 CAN 分组的旧帧。
#endif
}

static void InvalidateFeedbackForMissingSample(DJIMotorInstance *motor)
{
    PlatformTaskCriticalEnter(); // 使读取者一次看到失效标记和对应的间断代次。
    if (motor->speed_filter_valid || motor->current_filter_valid || motor->output_position_trusted)
        RecordFeedbackGap(motor); // 主动判定缺样也留下间断记录，重复失效不重复计数。
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
    PlatformTaskCriticalExit(); // 完成本次失效更新后才允许其他任务读取。
}

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static uint8_t MotorSenderGrouping(DJIMotorInstance *motor, PlatformCanEndpoint_t *config)
{
    if (motor == NULL || config == NULL ||
        config->tx_id == 0u || config->tx_id > 8u)
        return 0u;
    uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    uint8_t grouping_offset;
    // 逻辑编号决定分组偏移；资源绑定在申请电机之前单独检查。
    switch (config->can_bus)
    {
    case PLATFORM_CAN_BUS_1: grouping_offset = 0u; break; // 总线 1 使用第 0～2 组。
    case PLATFORM_CAN_BUS_2: grouping_offset = 3u; break; // 总线 2 使用第 3～5 组。
    case PLATFORM_CAN_BUS_3: grouping_offset = 6u; break; // 总线 3 使用第 6～8 组。
    default: return 0u; // 其他编号没有发送组。
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
            if (dji_motor_instance[i]->motor_can_endpoint.can_bus == motor->motor_can_endpoint.can_bus &&
                dji_motor_instance[i]->motor_can_endpoint.rx_id == config->rx_id)
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
            if (dji_motor_instance[i]->motor_can_endpoint.can_bus == motor->motor_can_endpoint.can_bus &&
                dji_motor_instance[i]->motor_can_endpoint.rx_id == config->rx_id)
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
 * @brief 解析逻辑 CAN 端点交付的电机反馈报文
 *
 * @param context 注册端点时绑定的电机实例
 * @param frame 任务派发时提供的只读接收帧，回调结束后不能保留
 */
static void DecodeDJIMotor(void *context, const PlatformCanRxFrame_t *frame)
{
    if (context == NULL || frame == NULL || frame->length != 8u)
        return; // 无所有者或长度不完整时不能更新反馈。
    const uint8_t *rxbuff = frame->data; // 接收帧只在回调期间可读。
    DJIMotorInstance *motor = (DJIMotorInstance *)context; // 注册时绑定的电机实例长期有效。
    DJI_Motor_Measure_s *measure = &motor->measure;
    const uint8_t first_feedback = motor->has_feedback_timestamp == 0u;
    const uint32_t elapsed_ms = first_feedback ? 0u :
        (uint32_t)(frame->received_ms - motor->last_feedback_timestamp_ms);
    const float dt_s = first_feedback ? 0.0f : FeedbackIntervalSeconds(frame, motor);
    const int16_t raw_rpm = (int16_t)(((uint16_t)rxbuff[2] << 8u) | rxbuff[3]);
    const int16_t raw_current = (int16_t)(((uint16_t)rxbuff[4] << 8u) | rxbuff[5]);
    const float raw_speed_aps = RPM_2_ANGLE_PER_SEC * (float)raw_rpm;
    float candidate_speed = measure->speed_aps;
    float candidate_current = measure->current_feedback;
    DJI_Motor_Filter_Status_e speed_status, current_status; // 计算完成后再公布每路滤波结果。
    uint8_t speed_valid, current_valid; // 临界区外的临时值不供其他任务读取。
    UpdateFeedbackFilter(motor, 1u, raw_speed_aps, dt_s, measure->speed_aps, &candidate_speed,
                         &speed_status, &speed_valid); // CAN 接收任务独占滤波器内部状态。
    UpdateFeedbackFilter(motor, 0u, (float)raw_current, dt_s, (float)measure->real_current, &candidate_current,
                         &current_status, &current_valid); // 滤波计算留在临界区外。

    PlatformTaskCriticalEnter(); // 本帧的测量值、时间和间断代次在同一次更新中交付。
    // 原始量先独立保存；滤波失败时保留旧有效输出，但控制环会检查有效标记。
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->raw_speed_aps = raw_speed_aps;
    measure->raw_current = raw_current;
    motor->feedback_dt_s = dt_s;

    motor->speed_filter_status = speed_status; // 将读数和有效性同时发布给读者。
    motor->speed_filter_valid = speed_valid; // 此后有效标志对应当前帧。
    motor->current_filter_status = current_status; // 电流由独立滤波器计算。
    motor->current_filter_valid = current_valid; // 电流换算失败时下方会撤销该标志。
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

    if ((!first_feedback && elapsed_ms > DJI_FEEDBACK_MAX_AGE_MS) ||
        !motor->speed_filter_valid || !motor->current_filter_valid)
        RecordFeedbackGap(motor); // 检查周期之间曾断流或质量失效也须重新计时。
    motor->feedback_dt_s = dt_s;
    motor->last_feedback_cycles = frame->received_ticks;
    motor->last_feedback_timestamp_ms = frame->received_ms;
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
    const uint8_t data_valid = (uint8_t)(motor->speed_filter_valid != 0u &&
                                          motor->current_filter_valid != 0u);
    PlatformTaskCriticalExit(); // 离线监测上报在完成帧更新后执行。
    (void)DaemonReportAt(motor->daemon, frame->received_ms,
        DAEMON_REPORT_TRANSPORT_VALID | DAEMON_REPORT_DATA_PRESENT |
        (data_valid != 0u ? DAEMON_REPORT_DATA_VALID : 0u));
}

static void DJIMotorLostCallback(const DaemonEvent_t *event, void *motor_ptr)
{
    if (event == NULL || event->state != DAEMON_SOURCE_OFFLINE)
        return;
    DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;
    uint16_t can_bus = (uint16_t)motor->motor_can_endpoint.can_bus;
    BSP_LOG_EVENT_2(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_03,
                    BSP_LOG_FIELD_I32((int)can_bus),
                    BSP_LOG_FIELD_I32((int)motor->motor_can_endpoint.tx_id));
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
    if (config == NULL || idx >= DJI_MOTOR_CNT ||
        (unsigned)config->output_role > (unsigned)MOTOR_OUTPUT_LOADER ||
        config->can_init_config.identifier_type != CAN_TX_ID_STANDARD || !FilterConfigValid(filter_config) ||
        !DjiConfigValid(config))
        return NULL;
    PlatformCanEndpoint_t can_config = config->can_init_config; // 复制配置，分组不改写车型参数。
    if (!PlatformCanBusIsBound(can_config.can_bus)) return NULL; // 总线未绑定时不申请资源。
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
    instance->output_role = config->output_role; // 记录本电机所属机构，后续对共享帧逐项检查。
    instance->output_limit_raw = config->dji_chassis_output_limit_raw != 0u ?
        config->dji_chassis_output_limit_raw : DJI_OUTPUT_MAX_RAW; // 旧驱动调用没有专用上限时沿用原范围。
    instance->motor_can_endpoint = config->can_init_config; // 保存车型逻辑总线和电机编号。
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
    if (!MotorSenderGrouping(instance, &can_config))
    {
        free(instance);
        return NULL;
    }
    instance->motor_can_endpoint.rx_id = can_config.rx_id; // 分组后保存实际反馈 ID。

    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .transport_timeout_ms = 20u,
        .data_timeout_ms = 20u,
        .startup_grace_ms = 1000u,
        .event_max_age_ms = 1000u,
        .source_id = (uint16_t)(0x2000u | (can_config.rx_id & 0x01ffu)),
        .event_capacity = 1u,
        .consumer_id = DAEMON_CONSUMER_MOTOR_TASK,
        .event_policy = DAEMON_EVENT_COUNT,
        .consumer_mode = DAEMON_CONSUMER_BOUND_TASK,
        .event_handler = DJIMotorLostCallback,
        .owner_id = instance,
    };
    instance->daemon = DaemonRegister(&daemon_config);
    if (instance->daemon == NULL)
    {
        free(instance);
        return NULL;
    }

    // 最后注册 CAN；失败时撤销先前离线计数并释放电机，避免残留回调。
    if (!PlatformCanRxRegister(&instance->motor_can_port, &instance->motor_can_endpoint,
                               DecodeDJIMotor, instance))
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

uint8_t DJIMotorRollbackStartup(DJIMotorInstance *motor)
{
    if (motor == NULL) return 0u; // 空指针不能对应已登记实例。
    size_t position = 0u; // 从登记表查找指针，不解引用未知地址。
    while (position < idx && dji_motor_instance[position] != motor)
        ++position; // 找到实例或遍历完整张表。
    if (position == idx) return 0u; // 未登记实例不能撤销其他资源。
    if (!DaemonUnregister(motor->daemon)) return 0u; // 监控已运行时保留实例和端点，禁止部分释放。
    const uint8_t group = motor->sender_group; // 保存该实例使用的发送组。
    const uint8_t number = motor->message_num; // 保存它在组内占用的两字节位置。
    const uint8_t filter_closed = PlatformCanRxRollback(&motor->motor_can_port); // 撤销软件端点并取得硬件关闭结果。
    memset(&sender_assignment[group].tx_buff[2u * number], 0, 2u); // 清除该电机留下的输出字节。
    for (size_t i = position + 1u; i < idx; ++i)
        dji_motor_instance[i - 1u] = dji_motor_instance[i]; // 压紧登记表并保留其他实例。
    dji_motor_instance[--idx] = NULL; // 释放最后一个重复表项。
    sender_enable_flag[group] = 0u; // 先假定该组已无其他电机。
    for (size_t i = 0u; i < idx; ++i)
        if (dji_motor_instance[i]->sender_group == group)
            sender_enable_flag[group] = 1u; // 同组仍有实例时继续发送该组。
    free(motor); // 回调和登记表均不再引用实例后释放内存。
    return filter_closed; // 硬件关闭失败也必须上报；调用者同样须放弃旧指针。
}

/* 电流只能通过电机自带传感器监测,后续考虑加入力矩传感器应变片等 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (motor == NULL || (type != MOTOR_FEED && type != OTHER_FEED))
        return;
    PlatformTaskCriticalEnter();
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
    {
        PlatformTaskCriticalExit();
        BSP_LOG_EVENT_0(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_04); // 检查是否传入了正确的LOOP类型,或发生了指针越界
        return;
    }
    PlatformTaskCriticalExit();
}

void DJIMotorStop(DJIMotorInstance *motor)
{
    PlatformTaskCriticalEnter(); // 停止标记、代次和参考一起更新。
    motor->stop_flag = MOTOR_STOP; // 停止期间发送的槽位直接清零。
#if defined(ONE_BOARD)
    if (motor->output_role == MOTOR_OUTPUT_CHASSIS)
    {
        ++motor->chassis_stop_generation; // 已计算的轮速目标属于旧代次，重新启用也不能发送。
        motor->reference_generation_valid = 0u; // 重新计算前要有底盘提交的新参考。
    }
#endif
    PlatformTaskCriticalExit(); // 发送方只能看到更新前或更新后的一组数据。
}

void DJIMotorEnable(DJIMotorInstance *motor)
{
    PlatformTaskCriticalEnter();
    motor->stop_flag = MOTOR_ENALBED;
    PlatformTaskCriticalExit();
}

/* 修改电机的实际闭环对象 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    PlatformTaskCriticalEnter();
    motor->motor_settings.outer_loop_type = outer_loop;
    PlatformTaskCriticalExit();
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
    PlatformTaskCriticalEnter();
    motor->motor_settings.close_loop_type = outer_loop;
    PlatformTaskCriticalExit();
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
                                    float *value,
                                    uint32_t *remaining_ms)
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
    if (remaining_ms != NULL)
    {
        const uint32_t age_ms = now_ms - snapshot.source_time_ms; // 已验证该样本在计算时还有效。
        const uint32_t signal_remaining = source->max_age_ms - age_ms; // 用实际使用的读数计算剩余时间。
        if (signal_remaining < *remaining_ms) *remaining_ms = signal_remaining; // 任一来源先到期都要撤销该目标。
    }
    *value = snapshot.value;
    return PID_OK;
}

Motor_Feedback_Status_e DJIMotorRebaseReference(DJIMotorInstance *motor, uint32_t now_ms)
{
    if (motor == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;

    PlatformTaskCriticalEnter(); // 检查并使用同一帧编码器数据设置零点。
    const uint32_t actual_ms = PlatformTimeNowMs(); // 调用后可能已经收到另一帧反馈。
    if ((uint32_t)(actual_ms - now_ms) <= INT32_MAX) now_ms = actual_ms; // 使用较新的本机时刻，允许回绕。
    if (!motor->has_feedback_timestamp || !DaemonIsOnline(motor->daemon) ||
        (uint32_t)(now_ms - motor->last_feedback_timestamp_ms) > DJI_MULTITURN_MAX_GAP_MS)
    {
        PlatformTaskCriticalExit(); // 无法使用当前编码器值重建多圈参考。
        return MOTOR_FEEDBACK_NOT_READY;
    }
    motor->measure.last_ecd = motor->measure.ecd; // 下一次补圈以当前单圈读数为起点。
    motor->measure.total_round = 0; // 清除旧参考下累计的圈数。
    motor->measure.total_angle = 0.0f; // 新相对零点对应当前位置。
    motor->encoder_baseline_valid = 1u; // 已有可信的补圈起点。
    motor->output_position_trusted = 1u; // 允许后续控制使用新参考。
    ++motor->feedback_gap_generation; // 旧参考计算的排队目标不能沿用到新零点。
    PlatformTaskCriticalExit(); // 原子交付新零点和间断代次。
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
    uint32_t continuity_generation; // 反馈间断记录必须与时间和序号同时复制。
    uint32_t feedback_read_time_ms;
    uint8_t has_sample;
    uint8_t position_trusted;
    uint8_t speed_valid;
    uint8_t current_valid;
    PlatformTaskCriticalEnter();
    measure = motor->measure;
    output_axis = motor->output_axis;
    timestamp_ms = motor->last_feedback_timestamp_ms;
    sequence = motor->feedback_sequence;
    continuity_generation = motor->feedback_gap_generation; // 即使读到的速度已经恢复，也保留此前发生的间断。
    has_sample = motor->has_feedback_timestamp;
    position_trusted = motor->encoder_baseline_valid && motor->output_position_trusted;
    speed_valid = motor->speed_filter_valid;
    current_valid = motor->current_filter_valid;
    feedback_read_time_ms = PlatformTimeNowMs();
    PlatformTaskCriticalExit();
    if (has_sample == 0u)
        return MOTOR_FEEDBACK_NOT_READY;

    float position_rad = 0.0f;
    float speed_rad_s = 0.0f;
    Motor_Feedback_Status_e status = MotorFeedbackConvertDegreesToOutputAxis(
        &output_axis, measure.total_angle, measure.speed_aps, &position_rad, &speed_rad_s);
    if (status != MOTOR_FEEDBACK_OK)
        return status;

    const uint32_t caller_elapsed = feedback_read_time_ms - now_ms;
    const uint32_t effective_now_ms = caller_elapsed < 0x80000000u ?
                                      feedback_read_time_ms : now_ms;
    const uint8_t fresh = (DaemonIsOnline(motor->daemon) != 0u) &&
        ((uint32_t)(effective_now_ms - timestamp_ms) <= DJI_FEEDBACK_MAX_AGE_MS);
    Motor_Feedback_Snapshot_s candidate = {0};
    candidate.sample_sequence = sequence;
    candidate.continuity_generation = continuity_generation; // 同一份反馈及其间断记录一起交付。
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

uint8_t DJIMotorSpeedFeedbackIsContinuous(const DJIMotorInstance *motor,
                                          uint32_t generation, uint32_t now_ms)
{
    if (motor == NULL) return 0u; // 没有登记的电机就没有可复核的轮速。
    PlatformTaskCriticalEnter(); // 将有效位、时间和间断记录作为一组读取。
    const uint8_t valid = (uint8_t)(motor->has_feedback_timestamp != 0u &&
        motor->speed_filter_valid != 0u && // 滤波拒绝当前样本后，旧估计不能再读取。
        motor->feedback_gap_generation == generation && // 即使新反馈已恢复，旧估计也不能跨越间断。
        now_ms - motor->last_feedback_timestamp_ms <= DJI_FEEDBACK_MAX_AGE_MS &&
        DaemonIsOnline(motor->daemon) != 0u); // 设备离线时一并拒绝。
    PlatformTaskCriticalExit(); // 只查询状态，保持原始读数和滤波状态。
    return valid;
}

uint8_t DJIMotorGetSingleTurnAngle(const DJIMotorInstance *motor, float *angle_deg, uint32_t *source_time_ms)
{
    if (motor == NULL || angle_deg == NULL || source_time_ms == NULL)
        return 0u; // 所有输入输出齐全时才读电机状态。
    PlatformTaskCriticalEnter(); // 防止角度和接收时间分别来自两帧。
    const float angle = motor->measure.angle_single_round; // 单圈转子角度保留设备侧单位。
    const uint32_t source_ms = motor->last_feedback_timestamp_ms; // 保留帧的原始接收时刻。
    const uint32_t now_ms = PlatformTimeNowMs(); // 读时刻和样本一起取，避免新帧晚于读时刻。
    const uint8_t has_sample = (uint8_t)(motor->has_feedback_timestamp && motor->encoder_baseline_valid); // 必须已收到编码器数据。
    PlatformTaskCriticalExit(); // 完成复制后释放任务保护。
    if (!has_sample || DaemonIsOnline(motor->daemon) == 0u ||
        (uint32_t)(now_ms - source_ms) > DJI_FEEDBACK_MAX_AGE_MS ||
        !isfinite(angle) || angle < 0.0f || angle >= 360.0f)
        return 0u; // 未收到、已离线、过期或不在单圈范围时不交付。
    *angle_deg = angle; // 两项检查完再写回角度。
    *source_time_ms = source_ms; // 交付同一帧的原始时刻。
    return 1u;
}

void DJIMotorGetFeedbackRoles(uint32_t now_ms, DJI_Feedback_Roles_s *output)
{
    if (output == NULL) return; // 调用方没有提供存储时不能写入结果。
    DJI_Feedback_Roles_s result = {0}; // 从空集合开始，缺失的机构不会被当作有效。
    uint32_t invalid_roles = 0u; // 任一电机无效时，记录其所属机构。
    uint32_t timestamp_roles = 0u; // 区分尚未保存时刻与合法的 0ms 采样。
    result.observed_at_ms = now_ms; // 保留调用方的读取时刻，供后续判断使用期限。
    for (size_t index = 0u; index < idx; ++index)
    {
        const DJIMotorInstance *motor = dji_motor_instance[index]; // 逐台检查启动时登记的电机。
        if (motor == NULL) continue; // 空槽不属于任何机构。
        const unsigned role = (unsigned)motor->output_role; // 先转换再校验，避免非法位移。
        if (role == MOTOR_OUTPUT_UNASSIGNED || role > MOTOR_OUTPUT_LOADER)
        {
            ++result.unassigned_count; // 未分配的电机必须由调用方检查。
            continue; // 不把未知用途算作任何机构的有效反馈。
        }
        const uint32_t role_bit = 1u << role; // 每种已声明机构对应独立位。
        result.present_roles |= role_bit; // 记录该机构至少注册了一台电机。
        ++result.motor_count[role]; // 调用方还需核对数量是否符合车型配置。
        Motor_Feedback_Snapshot_s feedback = {0}; // 每台电机独立取得带时间的反馈。
        if (DJIMotorGetFeedbackSnapshot(motor, now_ms, &feedback) != MOTOR_FEEDBACK_OK ||
            feedback.speed.valid == 0u || feedback.current.valid == 0u)
        {
            invalid_roles |= role_bit; // 本机构任何一台无效，整体速度和电流反馈就不完整。
            continue; // 不把无效样本的时间用于计算有效期限。
        }
        const uint32_t source_ms = feedback.speed.source_time_ms; // 速度和电流来自同一帧的接收时间。
        const uint32_t earlier_by = result.oldest_source_ms[role] - source_ms; // 无符号差值允许时钟回绕。
        if (!(timestamp_roles & role_bit) || (earlier_by != 0u && earlier_by <= INT32_MAX))
            result.oldest_source_ms[role] = source_ms; // 同机构选择最早的有效 DJI 样本。
        timestamp_roles |= role_bit; // 标明该机构已有可比较的时刻。
    }
    result.valid_roles = result.present_roles & ~invalid_roles; // 只交付存在且全部检查通过的机构。
    *output = result; // 汇总结束后一次交付结果。
}

uint8_t DJIMotorGetRecoveryFeedback(uint32_t now_ms, DJI_RecoveryFeedback_s *output)
{
    if (output == NULL || idx > DJI_MOTOR_CNT) return 0u; // 输出无效或登记数量异常时不交付部分数据。
    DJI_RecoveryFeedback_s result = {0}; // 无样本的电机保持序号零和各量无效。
    result.count = idx; // 保留完整登记数量，调用方须与车型要求比较。
    for (size_t index = 0u; index < idx; ++index)
    {
        const DJIMotorInstance *motor = dji_motor_instance[index]; // 使用启动时固定的登记顺序。
        if (motor == NULL) return 0u; // 登记表不完整时不能把缺失电机当作健康。
        DJI_RecoveryFeedbackItem_s *item = &result.motors[index]; // 每台电机占独立位置。
        item->bus = motor->motor_can_endpoint.can_bus; // 使用注册时实际选择的总线。
        item->role = motor->output_role; // 保留机构用途供车型检查。
        item->motor_type = motor->motor_type; // 按型号核对车型所需设备。
        item->motor_id = motor->motor_can_endpoint.tx_id; // 使用登记时的设备编号。
        Motor_Feedback_Snapshot_s feedback = {0}; // 每台电机按实际接收时间取得反馈。
        if (DJIMotorGetFeedbackSnapshot(motor, now_ms, &feedback) != MOTOR_FEEDBACK_OK) continue; // 没有合法样本时保持序号零和无效标志。
        PlatformTaskCriticalEnter(); // 读回当前身份，防止换算期间发生失效或收到新帧。
        const uint8_t same_sample = (uint8_t)(motor->feedback_sequence == feedback.sample_sequence &&
            motor->feedback_gap_generation == feedback.continuity_generation); // 失效可以发生在两份反馈之间，序号未必改变。
        PlatformTaskCriticalExit(); // 不在保护内进行后续数据转换。
        if (!same_sample) continue; // 下轮重读，不能混合两次反馈。
        item->sequence = feedback.sample_sequence; // 保存这份原始样本的序号。
        item->gap_generation = feedback.continuity_generation; // 使用与本次有效标志一同读取的间断记录。
        item->source_time_ms = feedback.speed.source_time_ms; // 三种反馈来自同一帧的接收时刻。
        item->speed_valid = feedback.speed.valid; // 不把过期的速度视为可用。
        item->current_valid = feedback.current.valid; // 电流有效性单独报告。
        item->position_valid = feedback.position.valid; // 多圈参考失效不能被速度代替。
    }
    *output = result; // 只有完整遍历后才交付。
    return 1u;
}

uint8_t DJIMotorRecoveryStillValid(const DJI_RecoveryFeedback_s *observed,
                                   uint32_t registered_mask, uint32_t now_ms)
{
    if (observed == NULL || observed->count == 0u || observed->count > DJI_MOTOR_CNT ||
        registered_mask == 0u || (registered_mask >> observed->count) != 0u) return 0u; // 无效结果或越界位不能参与恢复。
    const PlatformIrqState_t irq = PlatformIrqSaveAndDisable(); // 与最终安全检查保持同一原子窗口。
    uint8_t valid = (uint8_t)(idx == observed->count); // 运行中设备登记变化时拒绝旧结果。
    for (uint8_t index = 0u; valid && index < observed->count; ++index)
    {
        if ((registered_mask & (UINT32_C(1) << index)) == 0u) continue; // 辅助电机不阻止核心恢复。
        const DJIMotorInstance *motor = dji_motor_instance[index]; // 定位已登记的必需实例。
        const DJI_RecoveryFeedbackItem_s *item = &observed->motors[index]; // 使用同一下标的原始读取结果。
        if (motor == NULL || motor->motor_can_endpoint.can_bus != item->bus ||
            motor->output_role != item->role || motor->motor_type != item->motor_type ||
            motor->motor_can_endpoint.tx_id != item->motor_id ||
            motor->feedback_sequence == 0u || item->sequence == 0u ||
            motor->feedback_gap_generation != item->gap_generation ||
            (uint32_t)(motor->last_feedback_timestamp_ms - item->source_time_ms) > INT32_MAX ||
            !motor->has_feedback_timestamp || !motor->speed_filter_valid || !motor->current_filter_valid ||
            !item->speed_valid || !item->current_valid ||
            (item->position_valid && (!motor->encoder_baseline_valid || !motor->output_position_trusted)) ||
            (uint32_t)(now_ms - motor->last_feedback_timestamp_ms) > DJI_FEEDBACK_MAX_AGE_MS)
            valid = 0u; // 间断、倒退、身份或有效性变化时让调用方重新等待。
    }
    PlatformIrqRestore(irq); // 恢复调用者进入前的中断状态。
    return valid; // 此结果仍不代替总线和整车许可检查。
}

#if defined(CHASSIS_BOARD)
static uint8_t DJIMotorChassisFeedbackIsHealthy(uint32_t now_ms)
{
    for (size_t i = 0u; i < idx; ++i)
    {
        Motor_Feedback_Snapshot_s feedback = {0};
        if (DJIMotorGetFeedbackSnapshot(dji_motor_instance[i], now_ms, &feedback) != MOTOR_FEEDBACK_OK ||
            feedback.speed.valid == 0u || feedback.current.valid == 0u)
        {
            RobotFaultReport(ROBOT_FAULT_MOTOR_OFFLINE);
            return 0u;
        }
    }
    return 1u;
}

static void DJIMotorPrepareChassisOutput(const void *source, uint32_t now_ms,
                                         uint8_t *payload, uint8_t length)
{
    (void)source;
    // 队列等待期间命令或反馈可能过期，因此按 HAL 即将接受帧的时刻复核。
    if (payload == NULL || length != 8u || RobotSafetyIsReadyAt(now_ms) == 0u ||
        DJIMotorChassisFeedbackIsHealthy(now_ms) == 0u)
    {
        if (payload != NULL)
            memset(payload, 0, length);
    }
}
#endif

#if defined(ONE_BOARD)
static void DJIMotorPrepareSingleOutput(const void *source, uint32_t now_ms,
                                        uint8_t *payload, uint8_t length)
{
    uint8_t found = 0u; // 只有已登记的发送组才能使用代际信息。
    size_t group = 0u; // 保存当前 CAN 帧所属分组。
    uint32_t generation = 0u; // 保存该组已排队目标的代际。
    for (size_t index = 0u; index < sizeof(sender_assignment) / sizeof(sender_assignment[0]); ++index)
        if (source == &sender_assignment[index])
        {
            generation = sender_safety_generation[index]; // 读取成功入队时保存的代际。
            group = index; // 后续只处理属于此帧的电机。
            found = 1u; // 标记来源属于本驱动。
            break;
        }
    RobotControlPermits_t permits = {0}; // 查询失败或未获许可时保持禁止。
    uint8_t policy_active = 0u; // 区分旧整车路径与新分功能策略。
    if (payload == NULL || length != 8u || found == 0u ||
        RobotSafetyGetOutputPermitsForGeneration(now_ms, generation, &permits, &policy_active) == 0u)
    {
        if (payload != NULL)
            memset(payload, 0, length); // 等待期间失能时撤销队列副本中的电流目标。
        return;
    }
    uint8_t requested[8]; // 保存待复核目标，再以全零构造最终报文。
    memcpy(requested, payload, sizeof(requested)); // 只读取本次队列副本中的目标。
    memset(payload, 0, length); // 未注册或未获许可的槽位保持零输出。
    uint32_t checked_source = 0u; // 同一帧的四轮通常使用同一个来源，只检查一次。
    DJIChassisReferenceKind_t checked_kind = DJI_CHASSIS_SOURCE_DIRECTION; // 不同来源可能有相同的数字编号。
    uint8_t source_current = 0u; // 保存最近一次来源检查结果。
    for (size_t index = 0u; index < idx; ++index)
    {
        const DJIMotorInstance *motor = dji_motor_instance[index]; // 逐电机核对当前帧中的输出。
        if (motor == NULL || motor->sender_group != group || motor->message_num >= 4u) continue; // 忽略其他帧及非法槽位。
        const uint8_t slot = motor->message_num; // 找到该电机在帧内的两字节位置。
        const DJI_Queued_Feedback_s *basis = &sender_feedback[group]; // 使用成功入队的依据，不能借新目标更新时间。
        const uint8_t required = basis->requirements[slot]; // 读取计算这一个槽位时记录的条件。
        if (motor->stop_flag == MOTOR_STOP) continue; // 入队后执行器被停止，硬件提交前清零。
        if (motor->output_role == MOTOR_OUTPUT_CHASSIS &&
            motor->chassis_stop_generation != basis->chassis_stop_generation[slot])
            continue; // 即便本轮已再次启用，也丢弃停止前计算的轮位目标。
        const uint32_t source_sequence = basis->source_sequence[slot]; // 使用计算入队时的来源，不能借用后来提交的新参考。
        if (source_sequence != 0u)
        {
            const DJIChassisReferenceKind_t kind = (required & DJI_QUEUED_REFEREE_POWER)
                ? DJI_CHASSIS_SOURCE_REFEREE_POWER : DJI_CHASSIS_SOURCE_DIRECTION; // 复核本槽位自己的来源类型。
            if (source_sequence != checked_source || kind != checked_kind)
            {
                checked_source = source_sequence; // 下一个槽位可复用同一序号的结果。
                checked_kind = kind; // 序号和类型都相同才复用结果。
                source_current = chassis_source_check != NULL && chassis_source_check(kind, source_sequence, now_ms); // 在交给 HAL 前检查原来源。
            }
            if (!source_current) continue; // 来源撤销或过期时保留本槽位的零输出。
        }
        const uint32_t after_deadline = now_ms - basis->deadline_ms[slot]; // 无符号差值支持毫秒时钟回绕。
        if ((required & DJI_QUEUED_DEADLINE) && after_deadline != 0u && after_deadline <= INT32_MAX)
            continue; // 在到期时刻仍有效，下一毫秒开始清零。
        if ((required & (DJI_QUEUED_SPEED | DJI_QUEUED_CURRENT | DJI_QUEUED_POSITION)) &&
            (motor->feedback_gap_generation != basis->gap_generation[slot] ||
             ((required & DJI_QUEUED_SPEED) && !motor->speed_filter_valid) ||
             ((required & DJI_QUEUED_CURRENT) && !motor->current_filter_valid) ||
             ((required & DJI_QUEUED_POSITION) && !(motor->encoder_baseline_valid && motor->output_position_trusted))))
            continue; // 内部反馈间断或所需测量量失效，旧目标也必须清零。
        if (policy_active == 0u && permits.functions != 0u)
        {
            memcpy(payload + 2u * slot, requested + 2u * slot, 2u); // 旧整车许可仍允许有效电机输出。
            continue; // 旧路径无需检查机构分类。
        }
        if (motor->output_role == MOTOR_OUTPUT_LOADER &&
            (permits.fire_allowed == 0u || (required & DJI_QUEUED_REFERENCE) == 0u))
            continue; // 拨弹还需要击发许可，且参考必须属于本次目标的安全代际。
        uint32_t function = 0u; // 未声明机构时保持禁止。
        switch (motor->output_role)
        {
        case MOTOR_OUTPUT_CHASSIS: function = ROBOT_CONTROL_CHASSIS; break; // 底盘轮组。
        case MOTOR_OUTPUT_GIMBAL: function = ROBOT_CONTROL_GIMBAL; break; // 云台轴。
        case MOTOR_OUTPUT_FRICTION:
        case MOTOR_OUTPUT_LOADER: function = ROBOT_CONTROL_SHOOTER; break; // 发射机构；按次击发由发射模块处理。
        default: break; // 未知或未声明的机构不得输出。
        }
        if (function != 0u && (permits.functions & function) != 0u)
            memcpy(payload + 2u * motor->message_num, requested + 2u * motor->message_num, 2u); // 恢复已许可电机的目标。
    }
}
#endif

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
    PlatformTaskCriticalEnter();
    motor->motor_controller.pid_ref = ref;
    motor->reference_generation_valid = 0u; // 普通参考不具备拨弹所需的代际凭据。
    motor->reference_has_deadline = 0u; // 清零或其他普通参考不能继承旧目标的期限。
    motor->reference_source_sequence = 0u; // 普通参考不继承此前的外部来源。
    motor->reference_source_kind = DJI_CHASSIS_SOURCE_DIRECTION; // 没有来源编号时恢复默认类型。
    PlatformTaskCriticalExit();
}

// 保存参考、安全代际及可选截止点。调用者提供原始命令期限。
static uint8_t DJIMotorSetGenerationRef(DJIMotorInstance *motor, float ref, uint32_t generation,
                                        const uint32_t *deadline_ms, uint32_t source_sequence, DJIChassisReferenceKind_t source_kind)
{
    if (motor == NULL || !isfinite(ref)) return 0u; // 没有实例或参考不是有限数时不能提交。
    if (motor->output_role != MOTOR_OUTPUT_CHASSIS && motor->output_role != MOTOR_OUTPUT_GIMBAL &&
        motor->output_role != MOTOR_OUTPUT_LOADER) return 0u; // 未声明用途的电机不能借用此接口。
#if defined(ONE_BOARD)
    uint32_t required_function = 0u; // 按电机用途选择所需的功能许可。
    switch (motor->output_role)
    {
    case MOTOR_OUTPUT_CHASSIS: required_function = ROBOT_CONTROL_CHASSIS; break; // 轮组需要底盘许可。
    case MOTOR_OUTPUT_GIMBAL: required_function = ROBOT_CONTROL_GIMBAL; break; // 云台轴需要云台许可。
    case MOTOR_OUTPUT_LOADER: required_function = ROBOT_CONTROL_SHOOTER; break; // 拨弹需要发射机构许可。
    default: return 0u; // 其他电机不接受此接口。
    }
    const PlatformIrqState_t irq = PlatformIrqSaveAndDisable(); // 许可检查与参考写入之间不允许插入禁止事件。
    RobotControlPermits_t permits = {0}; // 查询失败时默认禁止发射。
    uint8_t policy_active = 0u; // 区分旧整车许可与分功能许可。
    const uint32_t now_ms = PlatformTimeNowMs(); // 许可和期限用同一时刻判断。
    const uint8_t allowed = (uint8_t)((deadline_ms == NULL || *deadline_ms - now_ms <= INT32_MAX) &&
        (source_sequence == 0u || chassis_source_check != NULL) &&
        RobotSafetyGetOutputPermitsForGeneration(now_ms, generation, &permits, &policy_active) &&
        (permits.functions & required_function) &&
        (motor->output_role != MOTOR_OUTPUT_LOADER || !policy_active || permits.fire_allowed)); // 拨弹还需要击发许可。
    if (allowed)
    {
        motor->motor_controller.pid_ref = ref; // 原子写入已授权的参考值。
        motor->reference_safety_generation = generation; // 记录参考所属的代际。
        motor->reference_has_deadline = deadline_ms != NULL; // 区分无期限和截止时刻为零。
        motor->reference_deadline_ms = deadline_ms != NULL ? *deadline_ms : 0u; // 后续计算和发送不得换成本轮时间。
        motor->reference_source_sequence = source_sequence; // 后续检查本参考自身的依赖。
        motor->reference_source_kind = (uint8_t)source_kind; // 目标和依赖类型同时生效。
        motor->reference_generation_valid = 1u; // 完整写入后才标记代际有效。
    }
    PlatformIrqRestore(irq); // 恢复进入前的中断状态。
    return allowed; // 拒绝时保留旧参考，但发送路径不会借新代际重用它。
#else
    if (deadline_ms != NULL || source_sequence != 0u) return 0u; // 其他板卡的发送路径不接受此接口。
    (void)generation; // 其他板角色仍由各自既有发送路径约束。
    (void)source_kind; // 其他板卡没有单板来源字段。
    DJIMotorSetRef(motor, ref); // 沿用现有参考设置。
    return 1u;
#endif
}

uint8_t DJIMotorSetRefForGeneration(DJIMotorInstance *motor, float ref, uint32_t generation)
{
    return DJIMotorSetGenerationRef(motor, ref, generation, NULL, 0u, DJI_CHASSIS_SOURCE_DIRECTION);
}

uint8_t DJIMotorSetChassisRefUntil(DJIMotorInstance *motor, float ref, uint32_t generation, uint32_t deadline_ms)
{
    return DJIMotorSetChassisRefUntilWithSource(motor, ref, generation, deadline_ms, 0u);
}

uint8_t DJIMotorBindChassisReferenceSource(DJIChassisReferenceSourceCheck_t check)
{
#if defined(ONE_BOARD)
    if (check == NULL || (chassis_source_check != NULL && chassis_source_check != check)) return 0u;
    chassis_source_check = check; // 启动时绑定一次，排队报文不会改用另一个函数。
    return 1u;
#else
    (void)check;
    return 0u;
#endif
}

uint8_t DJIMotorSetChassisRefUntilWithSource(DJIMotorInstance *motor, float ref, uint32_t generation,
                                            uint32_t deadline_ms, uint32_t source_sequence)
{
    if (motor == NULL || motor->output_role != MOTOR_OUTPUT_CHASSIS) return 0u; // 只为底盘传递该截止点。
    return DJIMotorSetGenerationRef(motor, ref, generation, &deadline_ms, source_sequence, DJI_CHASSIS_SOURCE_DIRECTION);
}

uint8_t DJIMotorSetChassisRefUntilWithPower(DJIMotorInstance *motor, float ref, uint32_t generation,
                                           uint32_t deadline_ms, uint32_t power_generation)
{
    if (motor == NULL || motor->output_role != MOTOR_OUTPUT_CHASSIS || power_generation == 0u)
        return 0u; // 只有底盘可以引用一个已存在的裁判供电区间。
    return DJIMotorSetGenerationRef(motor, ref, generation, &deadline_ms, power_generation, DJI_CHASSIS_SOURCE_REFEREE_POWER);
}

// 为所有电机实例计算三环PID,发送控制报文
void DJIMotorControl()
{
#if defined(ONE_BOARD)
    RobotSafetySnapshot_t cycle_safety = {0}; // 在计算之前记录本周期所属的安全代际。
    const uint8_t cycle_safety_valid = RobotSafetyGetSnapshot(&cycle_safety); // 查询失败时本周期只输出零值。
    DJI_Queued_Feedback_s computed_feedback[9] = {0}; // 本次计算暂存每组、每电机的反馈时间。
#endif
    uint32_t probe_all = PlatformProbeStart();
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
        uint8_t has_feedback_timestamp; // 区分尚未收到反馈与接收时间恰好为零。
        uint32_t feedback_timestamp_ms; // 保存这份测量数据的原始接收时间。
        uint32_t now_ms; // 与测量数据一起读取，避免新反馈晚于读取时刻。
#if defined(ONE_BOARD)
        uint32_t reference_generation; // 与参考值同时复制其安全代际。
        uint8_t reference_generation_valid; // 复制参考是否具备代际凭据。
        uint8_t reference_has_deadline; // 复制截止点是否存在。
        uint32_t reference_deadline_ms; // 复制该参考的原始截止点。
        uint32_t reference_source_sequence; // 复制该参考实际采用的外部来源序号。
        DJIChassisReferenceKind_t reference_source_kind; // 同一序号在不同来源中含义不同。
        uint32_t feedback_gap_generation; // 随用于计算的内部反馈一起复制间断代次。
        uint32_t chassis_stop_generation; // 与用于计算的轮速目标一起复制停止代次。
#endif
        PlatformTaskCriticalEnter();
        setting_snapshot = motor->motor_settings;
        measure_snapshot = motor->measure;
        output_axis = motor->output_axis;
        pid_ref = motor_controller->pid_ref;
#if defined(ONE_BOARD)
        reference_generation = motor->reference_safety_generation; // 与参考值读取同一组数据。
        reference_has_deadline = motor->reference_has_deadline; // 与参考值在同一临界区读取。
        reference_deadline_ms = motor->reference_deadline_ms; // 原始截止时刻不会随电机周期刷新。
        reference_source_sequence = motor->reference_source_sequence; // 参考、来源和安全代际在同一次复制中取得。
        reference_source_kind = (DJIChassisReferenceKind_t)motor->reference_source_kind; // 留给本轮和排队后的复核。
        feedback_gap_generation = motor->feedback_gap_generation; // 后续发生间断时只撤销用旧依据计算的目标。
        chassis_stop_generation = motor->chassis_stop_generation; // 计算途中发生停止也能在发送时识别。
#endif
        speed_filter_valid = motor->speed_filter_valid;
        current_filter_valid = motor->current_filter_valid;
        position_trusted = (uint8_t)(motor->encoder_baseline_valid && motor->output_position_trusted);
        has_feedback_timestamp = motor->has_feedback_timestamp; // 时间戳只有在收到反馈后才有意义。
        feedback_timestamp_ms = motor->last_feedback_timestamp_ms; // 与本轮测量使用同一份接收时间。
        now_ms = PlatformTimeNowMs(); // 在读取同一组数据后取当前时间。
#if defined(ONE_BOARD)
        if (reference_has_deadline && reference_deadline_ms - now_ms > INT32_MAX)
            motor->reference_generation_valid = 0u; // 观察到过期后保持撤销，时钟再次回绕不能恢复旧参考。
        if (motor->reference_generation_valid && reference_source_sequence != 0u &&
            (chassis_source_check == NULL || !chassis_source_check(reference_source_kind, reference_source_sequence, now_ms)))
            motor->reference_generation_valid = 0u; // 来源失效后停止使用此参考，须重新提交。
        reference_generation_valid = motor->reference_generation_valid; // 复制经本轮期限检查的代际状态。
#endif
        PlatformTaskCriticalExit();
        const Motor_Control_Setting_s *motor_setting = &setting_snapshot;
        DJI_Motor_Measure_s *measure = &measure_snapshot;
        // 即使离线监测尚未超时，内部测量也不能超过它自己的有效期。
        const uint8_t feedback_fresh = has_feedback_timestamp != 0u &&
            (uint32_t)(now_ms - feedback_timestamp_ms) <= DJI_FEEDBACK_MAX_AGE_MS;
        position_trusted = (uint8_t)(position_trusted && feedback_fresh); // 位置环同样不能使用过期反馈。
        // 首轮或间隔异常时仅重新计时，最后统一将本电机输出写0。
        float control_dt = 0.0f;
        PID_Status_e pid_status = MotorPIDBegin(&motor->pid_clock, PlatformTimeNowUs(), &control_dt);
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
            (!speed_internal_active || (speed_filter_valid && feedback_fresh)) &&
            (!current_internal_active || (current_filter_valid && feedback_fresh));
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
#if defined(ONE_BOARD)
        DJI_Queued_Feedback_s *basis = &computed_feedback[motor->sender_group]; // 计算结果的依据与目标同组。
        const uint8_t output_slot = motor->message_num; // 在组内对应到电机自己的位置。
        uint8_t required = 0u; // 逐项记录这一轮计算实际依赖的信号。
        basis->gap_generation[output_slot] = feedback_gap_generation; // 用相同的代次检查短暂断流。
        basis->chassis_stop_generation[output_slot] = chassis_stop_generation; // 发送时拒绝停止前算出的轮位目标。
        basis->source_sequence[output_slot] = reference_source_sequence; // 后续发送也要用本次计算的原来源。
        if (reference_source_kind == DJI_CHASSIS_SOURCE_REFEREE_POWER)
            required |= DJI_QUEUED_REFEREE_POWER; // 同样保留编号所属的裁判区间类型。
        if (speed_internal_active) required |= DJI_QUEUED_SPEED; // 内部速度环须持续有可信测量。
        if (current_internal_active) required |= DJI_QUEUED_CURRENT; // 内部电流环同样检查结果。
        if (motor_setting->outer_loop_type == ANGLE_LOOP && (motor_setting->close_loop_type & ANGLE_LOOP) != 0u &&
            motor_setting->angle_feedback_source == MOTOR_FEED) required |= DJI_QUEUED_POSITION; // 位置环使用编码器时需要多圈参考。
        if (motor->output_role == MOTOR_OUTPUT_LOADER && reference_generation_valid &&
            reference_generation == cycle_safety.control_generation)
            required |= DJI_QUEUED_REFERENCE; // 保留已有的参考代际检查条件。
        uint32_t remaining_ms = UINT32_MAX; // 没有读取任何限时信号时保留此值。
        if ((internal_axis_active || current_internal_active) && feedback_fresh)
            remaining_ms = DJI_FEEDBACK_MAX_AGE_MS - (now_ms - feedback_timestamp_ms); // 先计入内部反馈剩余时间。
        if (reference_has_deadline && reference_generation_valid)
        {
            const uint32_t reference_remaining = reference_deadline_ms - now_ms; // 过期已在临界区内撤销。
            if (reference_remaining < remaining_ms) remaining_ms = reference_remaining; // 同时受原始命令的剩余时间限制。
        }
        uint32_t *signal_remaining = &remaining_ms; // 外部信号可以缩短上述期限。
#else
        uint32_t *signal_remaining = NULL; // 其他板角色沿用原发送检查。
#endif
#if defined(ONE_BOARD)
        if ((motor->output_role == MOTOR_OUTPUT_CHASSIS || motor->output_role == MOTOR_OUTPUT_GIMBAL) &&
            (cycle_safety_valid == 0u || reference_generation_valid == 0u ||
             reference_generation != cycle_safety.control_generation))
            pid_status = PID_BAD_ARGUMENT; // 底盘和云台只计算本安全代际明确接受的参考。
#endif
        if (axis_status != MOTOR_FEEDBACK_OK && internal_axis_active && pid_status == PID_OK)
            pid_status = PID_NUMERIC_ERROR;

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if (pid_status == PID_OK && feedback_valid && (motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_status = ReadSignalForPID(&motor_controller->other_angle_feedback,
                    MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL, now_ms, &pid_measure, signal_remaining);
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
                    MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_OUTPUT_AXIS, now_ms, &feedforward, signal_remaining);
                if (pid_status == PID_OK)
                    pid_ref += feedforward;
            }

            if (pid_status == PID_OK && motor_setting->speed_feedback_source == OTHER_FEED)
                pid_status = ReadSignalForPID(&motor_controller->other_speed_feedback,
                    MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_EXTERNAL, now_ms, &pid_measure, signal_remaining);
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
                MOTOR_UNIT_DEVICE_CURRENT, MOTOR_REFERENCE_OUTPUT_AXIS, now_ms, &feedforward, signal_remaining);
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

        // 所有控制环和方向换算结束后，对报文中的最终有符号值限幅。
        const float output_limit = (float)motor->output_limit_raw; // 单位是电调原始命令值，各型号量纲不能统一换算为 A。
        if (pid_ref > output_limit)
        {
            pid_ref = output_limit; // 正方向最多输出该电机登记的上限。
            dji_current_sat_count++; // 沿用触及输出上限的诊断计数。
        }
        else if (pid_ref < -output_limit)
        {
            pid_ref = -output_limit; // 反方向也使用同一绝对上限。
            dji_current_sat_count++; // 反方向限幅计入同一个诊断项。
        }
        set = (int16_t)pid_ref; // 有限值已在可编码范围内，转换为报文整数。

#if defined(ONE_BOARD)
        if (remaining_ms != UINT32_MAX)
        {
            basis->deadline_ms[output_slot] = now_ms + remaining_ms; // 合并使用过的所有信号的最早到期时刻。
            required |= DJI_QUEUED_DEADLINE; // 时刻零也可能是有效截止点，单独记录是否有期限。
        }
        basis->requirements[output_slot] = required; // 计算结果和依据一起交给队列。
#endif

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
    PlatformProbeDone(&dji_prof_pid, probe_all);
    uint32_t probe_send = PlatformProbeStart();
    // 遍历全部组，未注册电机的组不发送。
    for (size_t i = 0; i < sizeof(sender_assignment) / sizeof(sender_assignment[0]); ++i)
    {
        if (sender_enable_flag[i])
        {
#if defined(CHASSIS_BOARD)
            // 反馈异常先上报故障，再把许可检查与周期值复制放在同一临界区。
            const uint8_t feedback_healthy = DJIMotorChassisFeedbackIsHealthy(PlatformTimeNowMs());
#endif
            const CAN_TxFrame_t frame = {
                .identifier = sender_assignment[i].identifier,
                .identifier_type = CAN_TX_ID_STANDARD,
                .data = sender_assignment[i].tx_buff,
                .length = 8u,
            };
            uint32_t request_id = 0u;
#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
            PlatformTaskCriticalEnter();
            // 许可被其他任务撤销时，只把零目标交给周期队列。
#if defined(CHASSIS_BOARD)
            if (feedback_healthy == 0u || RobotSafetyIsReadyAt(PlatformTimeNowMs()) == 0u)
#else
            if (cycle_safety_valid == 0u ||
                RobotSafetyIsReadyForGeneration(PlatformTimeNowMs(), cycle_safety.control_generation) == 0u)
#endif
                memset(sender_assignment[i].tx_buff, 0, sizeof(sender_assignment[i].tx_buff));
#endif
#if defined(CHASSIS_BOARD)
            (void)PlatformCanSubmitPeriodic(sender_assignment[i].bus,
                                    &sender_assignment[i], 0u, &frame,
                                    CAN_TX_PRIORITY_CONTROL, MOTOR_DJI_QUEUE_LIFETIME_MS, 1000u,
                                    DJIMotorPrepareChassisOutput, &request_id);
#elif defined(ONE_BOARD)
            const uint32_t previous_generation = sender_safety_generation[i]; // 提交失败时恢复旧队列的代际。
            const DJI_Queued_Feedback_s previous_feedback = sender_feedback[i]; // 保存旧队列对应的反馈时间。
            sender_safety_generation[i] = cycle_safety.control_generation; // 目标保留计算前的代际。
            sender_feedback[i] = computed_feedback[i]; // 同时保存新目标的反馈依据。
            const CAN_TxSubmitStatus_t submit_status = PlatformCanSubmitPeriodic(sender_assignment[i].bus,
                                    &sender_assignment[i], 0u, &frame,
                                    CAN_TX_PRIORITY_CONTROL, MOTOR_DJI_QUEUE_LIFETIME_MS, 1000u,
                                    DJIMotorPrepareSingleOutput, &request_id);
            if (submit_status != CAN_TX_SUBMIT_ACCEPTED && submit_status != CAN_TX_SUBMIT_COALESCED)
            {
                sender_safety_generation[i] = previous_generation; // 旧队列没有替换，不能替它更新代际。
                sender_feedback[i] = previous_feedback; // 同样不能用新反馈时间续期旧目标。
            }
#else
            (void)PlatformCanSubmitPeriodic(sender_assignment[i].bus,
                                    &sender_assignment[i], 0u, &frame,
                                    CAN_TX_PRIORITY_CONTROL, MOTOR_DJI_QUEUE_LIFETIME_MS, 1000u, NULL, &request_id);
#endif
#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
            PlatformTaskCriticalExit();
#endif
        }
    }
    PlatformProbeDone(&dji_prof_send, probe_send);
    PlatformProbeDone(&dji_prof_all, probe_all);
}
