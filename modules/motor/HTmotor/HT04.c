/** @file HT04.c
* @brief 解码 HT04 协议，并用每电机独立任务计算所选控制环及发送力矩字段。
 */
#include "HT04.h"
#include <string.h>
#include <math.h>
#include "memory.h"
#include "general_def.h"
#include "sample_filter.h"
#include "../../../bsp/os/platform_os.h" // 保留现有任务创建与毫秒等待。
#include "../../../bsp/os/platform_sync.h" // 反馈共享及原有毫秒／微秒时间。
#include "../../../bsp/dwt/platform_probe.h" // 保留原反馈周期计算。
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
static HTMotorInstance *ht_motor_instance[HT_MOTOR_CNT];
static PlatformTaskHandle_t ht_task_handle[HT_MOTOR_CNT];
static uint8_t zero_buff[6] = {0};

static uint8_t HTSourceValid(const Motor_Signal_Source_s *source,
                             Motor_Unit_e unit,
                             Motor_Reference_e reference)
{
    return (uint8_t)(MotorFeedbackValidateSignalSource(source) == MOTOR_FEEDBACK_OK &&
                     source->unit == unit && source->reference == reference);
}

static PID_Status_e HTReadExternal(const Motor_Signal_Source_s *source,
                                   Motor_Unit_e unit,
                                   Motor_Reference_e reference,
                                   uint32_t now_ms,
                                   float *feedback)
{
    if (feedback == NULL)
        return PID_BAD_ARGUMENT;

    Motor_Signal_Snapshot_s snapshot = {0};
    Motor_Feedback_Status_e status = MotorFeedbackReadSignal(source, unit, reference,
                                                             now_ms, &snapshot);
    if (status == MOTOR_FEEDBACK_NONFINITE || status == MOTOR_FEEDBACK_OVERFLOW)
        return PID_NUMERIC_ERROR;
    if (status != MOTOR_FEEDBACK_OK)
        return PID_BAD_ARGUMENT;
    *feedback = snapshot.value;
    return PID_OK;
}

static uint8_t HTControlConfigValid(const Motor_Init_Config_s *config)
{
    const Motor_Control_Setting_s *setting = &config->controller_setting_init_config;
    const Motor_Controller_Init_s *controller = &config->controller_param_init_config;
    uint8_t angle_active = (uint8_t)((setting->close_loop_type & ANGLE_LOOP) != 0u &&
                                     setting->outer_loop_type == ANGLE_LOOP);
    uint8_t speed_active = (uint8_t)((setting->close_loop_type & SPEED_LOOP) != 0u &&
                                     (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) != 0u);

    Motor_Capabilities_s capabilities;
    if (MotorFeedbackValidateOutputAxis(&config->output_axis) != MOTOR_FEEDBACK_OK ||
        MotorFeedbackGetCapabilities(HT04, &capabilities) != MOTOR_FEEDBACK_OK ||
        (capabilities.measurement_mask & MOTOR_CAPABILITY_TORQUE) == 0u ||
        setting->motor_reverse_flag != MOTOR_DIRECTION_NORMAL ||
        setting->feedback_reverse_flag != FEEDBACK_DIRECTION_NORMAL ||
        (setting->feedforward_flag & (uint8_t)~CURRENT_AND_SPEED_FEEDFORWARD) != 0u)
        return 0u;

    if ((angle_active != 0u &&
         (setting->angle_feedback_source != MOTOR_FEED &&
          (setting->angle_feedback_source != OTHER_FEED ||
           !HTSourceValid(&controller->other_angle_feedback, MOTOR_UNIT_RAD,
                          MOTOR_REFERENCE_EXTERNAL)))) ||
        (speed_active != 0u &&
         ((setting->speed_feedback_source != MOTOR_FEED &&
           (setting->speed_feedback_source != OTHER_FEED ||
            !HTSourceValid(&controller->other_speed_feedback, MOTOR_UNIT_RAD_S,
                           MOTOR_REFERENCE_EXTERNAL))) ||
          ((setting->feedforward_flag & SPEED_FEEDFORWARD) != 0u &&
           !HTSourceValid(&controller->speed_feedforward, MOTOR_UNIT_RAD_S,
                          MOTOR_REFERENCE_OUTPUT_AXIS)))) ||
        ((setting->feedforward_flag & CURRENT_FEEDFORWARD) != 0u &&
         !HTSourceValid(&controller->current_feedforward, MOTOR_UNIT_DEVICE_TORQUE,
                        MOTOR_REFERENCE_OUTPUT_AXIS)))
        return 0u;

    return 1u;
}

/**
 * @brief 设置电机模式,报文内容[0xff,0xff,0xff,0xff,0xff,0xff,0xff,cmd]
 *
 * @param cmd
 * @param motor
 */
static void HTMotorSetMode(HTMotor_Mode_t cmd, HTMotorInstance *motor)
{
    memset(motor->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    const CAN_TxFrame_t frame = {
        .identifier = motor->can_endpoint.tx_id, // 使用车型配置的电机发送 ID。
        .identifier_type = CAN_TX_ID_STANDARD, // HT 模式命令使用标准帧。
        .data = motor->tx_buff, // 提交接口会复制这 8 字节。
        .length = 8u, // 前 7 字节和命令字节组成完整模式命令。
    };
    uint32_t request_id = 0u;
    // 单次命令进入有界重试队列，不能被下一次周期控制覆盖。
    (void)PlatformCanSubmitOneShot(motor->can_endpoint.can_bus, &motor->can_rx,
                                   &frame, CAN_TX_PRIORITY_URGENT, 100u, &request_id);
    // 提交时已复制数据，恢复控制帧的前 6 字节供周期任务使用。
    memcpy(motor->tx_buff, zero_buff, 6);
}
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 解析电机反馈值
 *
 * @param context 长期保存的电机对象。
 * @param frame 接收任务交付的只读帧和原始接收时间。
 */
static void HTMotorDecode(void *context, const PlatformCanRxFrame_t *frame)
{
    if (context == NULL || frame == NULL || frame->length < 6u)
        return; // 无完整反馈时不能读取或刷新状态。
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t const *rxbuff = frame->data; // 数据仅在本次回调期间可读。
    HTMotorInstance *motor = context; // 逻辑接收端点保存长期电机对象。
    HTMotor_Measure_t *measure = &(motor->measure);

    HTMotor_Measure_t candidate = *measure;
    candidate.last_angle = measure->total_angle;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    candidate.total_angle = uint_to_float(tmp, P_MIN, P_MAX, 16);

    tmp = (uint16_t)(rxbuff[3] << 4) | (rxbuff[4] >> 4);
    // 先在本地历史上平均，失败时整份反馈和接收计时保持原样。
    if (MovingAverageF32(uint_to_float(tmp, V_MIN, V_MAX, 12) - HT_SPEED_BIAS,
                         candidate.speed_buff, SPEED_BUFFER_SIZE, SPEED_BUFFER_SIZE,
                         &candidate.speed_rads) != MATH_OK)
        return;

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    candidate.real_current = CURRENT_SMOOTH_COEF * uint_to_float(tmp, T_MIN, T_MAX, 12) +
                            (1 - CURRENT_SMOOTH_COEF) * measure->real_current;
    if (!isfinite(candidate.real_current)) return;
    candidate.feed_dt = PlatformProbeDeltaSeconds(&candidate.feed_cnt);
    PlatformTaskCriticalEnter();
    *measure = candidate;
    motor->feedback_timestamp_ms = frame->received_ms;
    motor->feedback_sequence++;
    if (motor->feedback_sequence == 0u)
        motor->feedback_sequence++;
    PlatformTaskCriticalExit();
    (void)DaemonReportAt(motor->motor_daemon, frame->received_ms,
        DAEMON_REPORT_TRANSPORT_VALID | DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID);
}

static void HTMotorLostCallback(const DaemonEvent_t *event, void *motor_ptr)
{
    if (event == NULL || event->state != DAEMON_SOURCE_OFFLINE)
        return;
    HTMotorInstance *motor = (HTMotorInstance *)motor_ptr;
    // 每次确认进入离线只提交一条模式请求；重试须等待新通信恢复后再次失联。
    BSP_LOG_EVENT_1(EV_MODULES_MOTOR_HTMOTOR_HT04_C_01,
                    BSP_LOG_FIELD_I32((int)motor->can_endpoint.tx_id));
    motor->lost_cnt++;
    HTMotorSetMode(CMD_MOTOR_MODE, motor);
}

// 先发零控制量再置零，并保存公共报文字段，供之后的模式命令恢复使用。
void HTMotorCalibEncoder(HTMotorInstance *motor)
{
    uint16_t p, v, kp, kd, t;
    p = float_to_uint(0, P_MIN, P_MAX, 16);
    v = float_to_uint(0, V_MIN, V_MAX, 12);
    kp = float_to_uint(0, KP_MIN, KP_MAX, 12);
    kd = float_to_uint(0, KD_MIN, KD_MAX, 12);
    t = float_to_uint(0, T_MIN, T_MAX, 12);

    uint8_t *buf = motor->tx_buff;
    buf[0] = p >> 8;
    buf[1] = p & 0xFF;
    buf[2] = v >> 4;
    buf[3] = ((v & 0xF) << 4) | (kp >> 8);
    buf[4] = kp & 0xFF;
    buf[5] = kd >> 4;
    buf[6] = ((kd & 0xF) << 4) | (t >> 8);
    buf[7] = t & 0xff;
    memcpy(zero_buff, buf, 6); // 初始化的时候至少调用一次,故将其他指令为0时发送的报文保存一下,详见ht04电机说明
    const CAN_TxFrame_t control_frame = {
        .identifier = motor->can_endpoint.tx_id,
        .identifier_type = CAN_TX_ID_STANDARD,
        .data = buf,
        .length = 8u,
    };
    uint32_t request_id = 0u;
    (void)PlatformCanSubmitOneShot(motor->can_endpoint.can_bus, &motor->can_rx,
                                   &control_frame, CAN_TX_PRIORITY_URGENT, 100u, &request_id);
    PlatformHtLegacyDelayMs(5u);
    HTMotorSetMode(CMD_ZERO_POSITION, motor); // 发出置零命令；本接口没有读取结果确认零位。
    PlatformHtLegacyDelayMs(5u);
    // HTMotorSetMode(CMD_MOTOR_MODE, motor);
}

HTMotorInstance *HTMotorInit(const Motor_Init_Config_s *config)
{
    if (config == NULL || !HTControlConfigValid(config))
        return NULL;
    if (!PlatformCanBusIsBound(config->can_init_config.can_bus)) return NULL; // 总线未绑定时不分配或注册。

    const Motor_Control_Setting_s *setting = &config->controller_setting_init_config;
    uint8_t angle_active = (uint8_t)((setting->close_loop_type & ANGLE_LOOP) != 0u &&
                                     setting->outer_loop_type == ANGLE_LOOP);
    uint8_t speed_active = (uint8_t)((setting->close_loop_type & SPEED_LOOP) != 0u &&
                                     (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) != 0u);
    uint8_t current_active = (uint8_t)((setting->close_loop_type & CURRENT_LOOP) != 0u);
    PIDInstance current_pid = {0};
    PIDInstance speed_pid = {0};
    PIDInstance angle_pid = {0};
    if (MotorPIDInit(&current_pid, &config->controller_param_init_config.current_PID,
                     current_active) != PID_OK ||
        MotorPIDInit(&speed_pid, &config->controller_param_init_config.speed_PID,
                     speed_active) != PID_OK ||
        MotorPIDInit(&angle_pid, &config->controller_param_init_config.angle_PID,
                     angle_active) != PID_OK)
        return NULL;

    HTMotorInstance *motor = (HTMotorInstance *)malloc(sizeof(HTMotorInstance));
    if (motor == NULL)
    {
        return NULL;
    }
    memset(motor, 0, sizeof(HTMotorInstance));

    motor->motor_settings = *setting;
    motor->can_endpoint = config->can_init_config; // 保存车型声明的逻辑收发端点。
    motor->output_axis = config->output_axis;
    motor->current_PID = current_pid;
    motor->speed_PID = speed_pid;
    motor->angle_PID = angle_pid;
    motor->other_angle_feedback = config->controller_param_init_config.other_angle_feedback;
    motor->other_speed_feedback = config->controller_param_init_config.other_speed_feedback;
    motor->speed_feedforward = config->controller_param_init_config.speed_feedforward;
    motor->current_feedforward = config->controller_param_init_config.current_feedforward;

    Daemon_Init_Config_s conf = {
        .transport_timeout_ms = 50u,
        .data_timeout_ms = 50u,
        .startup_grace_ms = 1000u,
        .event_max_age_ms = 1000u,
        .source_id = (uint16_t)(0x4000u | (config->can_init_config.rx_id & 0x01ffu)),
        .event_capacity = 4u,
        .consumer_id = DAEMON_EVENT_TASK_CONSUMER,
        .event_policy = DAEMON_EVENT_FIFO,
        .consumer_mode = DAEMON_CONSUMER_INDEPENDENT_TASK,
        .event_handler = HTMotorLostCallback,
        .owner_id = motor,
    };
    motor->motor_daemon = DaemonRegister(&conf);
    if (motor->motor_daemon == NULL)
    {
        free(motor);
        return NULL;
    }

    if (!PlatformCanRxRegister(&motor->can_rx, &motor->can_endpoint, HTMotorDecode, motor))
    {
        // 接收端点注册失败时撤销此前注册的离线监控。
        (void)DaemonUnregister(motor->motor_daemon);
        // 释放尚未交给周期任务的电机对象。
        free(motor);
        return NULL;
    }

    HTMotorEnable(motor);
    HTMotorSetMode(CMD_MOTOR_MODE, motor); // 确保电机已经上电并执行电机模式
    PlatformHtLegacyDelayMs(50u);
    HTMotorCalibEncoder(motor); // 将当前编码器位置作为零位
    PlatformHtLegacyDelayMs(50u); // 延续原初始化等待，不能在中断中执行。
    ht_motor_instance[idx++] = motor;
    return motor;
}

Motor_Feedback_Status_e HTMotorGetFeedbackSnapshot(const HTMotorInstance *motor,
                                                    uint32_t now_ms,
                                                    Motor_Feedback_Snapshot_s *snapshot)
{
    if (motor == NULL || snapshot == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;

    HTMotor_Measure_t measure;
    Motor_Output_Axis_Config_s output_axis;
    uint32_t timestamp_ms;
    uint32_t sequence;
    PlatformTaskCriticalEnter();
    measure = motor->measure;
    output_axis = motor->output_axis;
    timestamp_ms = motor->feedback_timestamp_ms;
    sequence = motor->feedback_sequence;
    PlatformTaskCriticalExit();
    if (sequence == 0u)
        return MOTOR_FEEDBACK_NOT_READY;

    float output_position = 0.0f;
    float output_speed = 0.0f;
    Motor_Feedback_Status_e status = MotorFeedbackConvertOutputAxis(
        &output_axis, measure.total_angle, measure.speed_rads,
        &output_position, &output_speed);
    if (status != MOTOR_FEEDBACK_OK)
        return status;

    const uint8_t fresh = DaemonIsOnline(motor->motor_daemon) != 0u &&
        (uint32_t)(now_ms - timestamp_ms) <= 50u;
    Motor_Feedback_Snapshot_s candidate = {0};
    candidate.sample_sequence = sequence;
    candidate.position = (Motor_Signal_Snapshot_s){
        .value = output_position, .unit = MOTOR_UNIT_RAD,
        .reference = MOTOR_REFERENCE_OUTPUT_AXIS, .source_time_ms = timestamp_ms,
        .sequence = sequence, .valid = fresh,
    };
    candidate.speed = (Motor_Signal_Snapshot_s){
        .value = output_speed, .unit = MOTOR_UNIT_RAD_S,
        .reference = MOTOR_REFERENCE_OUTPUT_AXIS, .source_time_ms = timestamp_ms,
        .sequence = sequence, .valid = fresh,
    };
    candidate.torque = (Motor_Signal_Snapshot_s){
        .value = measure.real_current * (float)output_axis.direction,
        .unit = MOTOR_UNIT_DEVICE_TORQUE,
        .reference = MOTOR_REFERENCE_OUTPUT_AXIS, .source_time_ms = timestamp_ms,
        .sequence = sequence, .valid = fresh,
    };
    *snapshot = candidate;
    return MOTOR_FEEDBACK_OK;
}

void HTMotorSetRef(HTMotorInstance *motor, float ref)
{
    if (motor == NULL)
        return;
    PlatformTaskCriticalEnter();
    motor->pid_ref = ref;
    PlatformTaskCriticalExit();
}

/**
 * @brief 为了避免总线堵塞,为每个电机创建一个发送任务
 * @param argument 传入的电机指针
 */
__attribute__((noreturn)) void HTMotorTask(void const *argument)
{
    float set, pid_measure, pid_ref;
    HTMotorInstance *motor = (HTMotorInstance *)argument;
    uint16_t tmp;

    // 持续取目标，依次计算已选角度、速度和当前反馈环；最后停止标记覆盖为零力矩。
    while (1)
    {
        HTMotor_Measure_t measure;
        Motor_Control_Setting_s setting;
        Motor_Output_Axis_Config_s output_axis;
        Motor_Signal_Source_s angle_source, speed_source, speed_feedforward, current_feedforward;
        Motor_Working_Type_e stop_flag;
        PlatformTaskCriticalEnter();
        measure = motor->measure;
        setting = motor->motor_settings;
        output_axis = motor->output_axis;
        angle_source = motor->other_angle_feedback;
        speed_source = motor->other_speed_feedback;
        speed_feedforward = motor->speed_feedforward;
        current_feedforward = motor->current_feedforward;
        pid_ref = motor->pid_ref;
        stop_flag = motor->stop_flag;
        PlatformTaskCriticalExit();
        const uint8_t axis_outer = (uint8_t)((setting.outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) != 0u);
        const float axis_direction = (float)output_axis.direction;
        const uint32_t now_ms = PlatformTimeNowMs();
        // 首轮或间隔异常时仅重新计时，最后统一将本电机输出写0。
        float control_dt = 0.0f;
        PID_Status_e pid_status = MotorPIDBegin(&motor->pid_clock, PlatformTimeNowUs(), &control_dt);
        if (pid_status == PID_OK && (setting.close_loop_type & ANGLE_LOOP) &&
            setting.outer_loop_type == ANGLE_LOOP)
        {
            if (setting.angle_feedback_source == OTHER_FEED)
                pid_status = HTReadExternal(&angle_source, MOTOR_UNIT_RAD,
                    MOTOR_REFERENCE_EXTERNAL, now_ms, &pid_measure);
            else if (setting.angle_feedback_source == MOTOR_FEED)
            {
                float unused_speed = 0.0f;
                Motor_Feedback_Status_e conversion = MotorFeedbackConvertOutputAxis(
                    &output_axis, measure.total_angle, measure.speed_rads,
                    &pid_measure, &unused_speed);
                if (conversion != MOTOR_FEEDBACK_OK)
                    pid_status = conversion == MOTOR_FEEDBACK_NONFINITE ?
                        PID_BAD_ARGUMENT : PID_NUMERIC_ERROR;
            }
            else
                pid_status = PID_BAD_ARGUMENT;
            if (pid_status == PID_OK)
                pid_status = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref, control_dt, &pid_ref);
        }

        if (pid_status == PID_OK && (setting.close_loop_type & SPEED_LOOP) &&
            (setting.outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if ((setting.feedforward_flag & SPEED_FEEDFORWARD) != 0u)
                pid_status = HTReadExternal(&speed_feedforward, MOTOR_UNIT_RAD_S,
                    MOTOR_REFERENCE_OUTPUT_AXIS, now_ms, &pid_measure);
            if (pid_status == PID_OK && (setting.feedforward_flag & SPEED_FEEDFORWARD) != 0u)
            {
                pid_ref += pid_measure;
                if (!isfinite(pid_ref)) pid_status = PID_NUMERIC_ERROR;
            }
            if (pid_status == PID_OK && setting.speed_feedback_source == OTHER_FEED)
                pid_status = HTReadExternal(&speed_source, MOTOR_UNIT_RAD_S,
                    MOTOR_REFERENCE_EXTERNAL, now_ms, &pid_measure);
            else if (pid_status == PID_OK && setting.speed_feedback_source == MOTOR_FEED)
            {
                float unused_position = 0.0f;
                Motor_Feedback_Status_e conversion = MotorFeedbackConvertOutputAxis(
                    &output_axis, measure.total_angle, measure.speed_rads,
                    &unused_position, &pid_measure);
                if (conversion != MOTOR_FEEDBACK_OK)
                    pid_status = conversion == MOTOR_FEEDBACK_NONFINITE ?
                        PID_BAD_ARGUMENT : PID_NUMERIC_ERROR;
            }
            else if (pid_status == PID_OK)
                pid_status = PID_BAD_ARGUMENT;
            if (pid_status == PID_OK)
                pid_status = PIDCalculate(&motor->speed_PID, pid_measure, pid_ref, control_dt, &pid_ref);
        }

        // 当前前馈保留在原来的速度环之后、当前环之前；失败时不再读取后续输入。
        if (pid_status == PID_OK && (setting.feedforward_flag & CURRENT_FEEDFORWARD) != 0u)
        {
            pid_status = HTReadExternal(&current_feedforward, MOTOR_UNIT_DEVICE_TORQUE,
                MOTOR_REFERENCE_OUTPUT_AXIS, now_ms, &pid_measure);
            if (pid_status == PID_OK)
            {
                pid_ref += pid_measure;
                if (!isfinite(pid_ref)) pid_status = PID_NUMERIC_ERROR;
            }
        }
        if (pid_status == PID_OK && (setting.close_loop_type & CURRENT_LOOP) != 0u)
        {
            pid_measure = measure.real_current;
            if (axis_outer != 0u)
                pid_measure *= axis_direction;
            pid_status = PIDCalculate(&motor->current_PID, pid_measure, pid_ref, control_dt, &pid_ref);
        }

        // 输出轴上的力矩方向映回电机转子；电流／力矩开环值仍沿用设备坐标。
        if (axis_outer != 0u)
            pid_ref *= axis_direction;

        if (pid_status != PID_OK || !isfinite(pid_ref))
        {
            // 一环失败后丢弃本轮串级结果，恢复前清除各环已有积分和微分历史。
            MotorPIDResetLoops(&motor->angle_PID, &motor->speed_PID, &motor->current_PID);
            motor->pid_clock.last_status = pid_status != PID_OK ? pid_status : PID_NUMERIC_ERROR;
            pid_ref = 0.0f;
        }

        set = pid_ref;

        LIMIT_MIN_MAX(set, T_MIN, T_MAX);
        tmp = float_to_uint(set, T_MIN, T_MAX, 12);
        if (stop_flag == MOTOR_STOP)
            tmp = float_to_uint(0, T_MIN, T_MAX, 12);
        motor->tx_buff[6] = (tmp >> 8);
        motor->tx_buff[7] = tmp & 0xff;

        const CAN_TxFrame_t frame = {
            .identifier = motor->can_endpoint.tx_id,
            .identifier_type = CAN_TX_ID_STANDARD,
            .data = motor->tx_buff,
            .length = 8u,
        };
        uint32_t request_id = 0u;
        (void)PlatformCanSubmitPeriodic(motor->can_endpoint.can_bus, &motor->can_rx, 0u, &frame,
                                        CAN_TX_PRIORITY_CONTROL, 4u, 1000u, NULL, &request_id);

        PlatformTaskDelayMs(1u); // 保留原来的 1 ms 任务让出。
    }
}

void HTMotorControlInit()
{
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        const PlatformTaskConfig_t config = {
            .name = "ht_task_name", // 保留 CMSIS 宏原先生成的任务名。
            .entry = HTMotorTask, // 每个任务继续执行原控制循环。
            .argument = ht_motor_instance[i], // 绑定对应的长期电机实例。
            .stack_words = 128u, // 保持原有栈容量。
            .priority = PLATFORM_TASK_NORMAL, // 保持原有普通优先级。
        };
        (void)PlatformTaskCreateTracked(&config, &ht_task_handle[i]); // 原接口仍不报告创建失败。
    }
}

void HTMotorStop(HTMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void HTMotorEnable(HTMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void HTMotorOuterLoop(HTMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}
