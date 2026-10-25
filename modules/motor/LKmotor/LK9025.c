/**
 * @file LK9025.c
 * @brief 管理 LK 实例、解析反馈，并按物理 CAN 总线分别提交多电机帧。
 * 实例只在启动阶段登记，资源在运行期间长期保留。
 */
#include "LK9025.h"
#include <string.h>
#include <math.h>
#include <float.h>
#include "stdlib.h"
#include "general_def.h"
#include "daemon.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"

static LKMotorInstance *lkmotor_instance[DEVICE_CAN_CNT][LK_MOTOR_PER_BUS_CNT] = {{NULL}};
static CANInstance *lk_sender_instance[DEVICE_CAN_CNT] = {NULL};
static uint8_t lk_bus_count[DEVICE_CAN_CNT] = {0};

// 拒绝未知句柄，避免资源申请后才发现目标不属于当前 H723 的三条 CAN 总线。
static int8_t LKBusIndexFromHandle(FDCAN_HandleTypeDef *can_handle)
{
    if (can_handle == &hfdcan1)
        return 0;
    if (can_handle == &hfdcan2)
        return 1;
    if (can_handle == &hfdcan3)
        return 2;
    return -1;
}

static uint8_t LKSourceValid(const Motor_Signal_Source_s *source,
                             Motor_Unit_e unit,
                             Motor_Reference_e reference)
{
    return (uint8_t)(MotorFeedbackValidateSignalSource(source) == MOTOR_FEEDBACK_OK &&
                     source->unit == unit && source->reference == reference);
}

static PID_Status_e LKReadExternal(const Motor_Signal_Source_s *source,
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

static uint8_t LKControlConfigValid(const Motor_Init_Config_s *config)
{
    const Motor_Control_Setting_s *setting = &config->controller_setting_init_config;
    const Motor_Controller_Init_s *controller = &config->controller_param_init_config;
    uint8_t angle_active = (uint8_t)((setting->close_loop_type & ANGLE_LOOP) != 0u &&
                                     setting->outer_loop_type == ANGLE_LOOP);
    uint8_t speed_active = (uint8_t)((setting->close_loop_type & SPEED_LOOP) != 0u &&
                                     (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) != 0u);

    Motor_Capabilities_s capabilities;
    if (MotorFeedbackValidateOutputAxis(&config->output_axis) != MOTOR_FEEDBACK_OK ||
        MotorFeedbackGetCapabilities(LK9025, &capabilities) != MOTOR_FEEDBACK_OK ||
        (capabilities.measurement_mask & MOTOR_CAPABILITY_CURRENT) == 0u ||
        setting->motor_reverse_flag != MOTOR_DIRECTION_NORMAL ||
        setting->feedback_reverse_flag != FEEDBACK_DIRECTION_NORMAL ||
        (setting->feedforward_flag & (uint8_t)~CURRENT_AND_SPEED_FEEDFORWARD) != 0u ||
        (setting->angle_feedback_source != MOTOR_FEED && setting->angle_feedback_source != OTHER_FEED) ||
        (setting->speed_feedback_source != MOTOR_FEED && setting->speed_feedback_source != OTHER_FEED))
        return 0u;

    if (angle_active != 0u)
    {
        if ((setting->angle_feedback_source == OTHER_FEED &&
             !LKSourceValid(&controller->other_angle_feedback, MOTOR_UNIT_RAD,
                            MOTOR_REFERENCE_EXTERNAL)) ||
            ((setting->feedforward_flag & SPEED_FEEDFORWARD) != 0u &&
             !LKSourceValid(&controller->speed_feedforward, MOTOR_UNIT_RAD_S,
                            MOTOR_REFERENCE_OUTPUT_AXIS)))
            return 0u;
    }
    if (speed_active != 0u &&
        ((setting->speed_feedback_source == OTHER_FEED &&
          !LKSourceValid(&controller->other_speed_feedback, MOTOR_UNIT_RAD_S,
                         MOTOR_REFERENCE_EXTERNAL)) ||
         ((setting->feedforward_flag & CURRENT_FEEDFORWARD) != 0u &&
          !LKSourceValid(&controller->current_feedforward, MOTOR_UNIT_DEVICE_CURRENT,
                         MOTOR_REFERENCE_OUTPUT_AXIS))))
        return 0u;

    return 1u;
}

/**
 * @brief 电机反馈报文解析
 *
 * @param _instance 发生中断的caninstance
 */
static void LKMotorDecode(CANInstance *_instance)
{
    if (_instance == NULL || _instance->id == NULL || _instance->rx_len != 8u)
        return;
    LKMotorInstance *motor = (LKMotorInstance *)_instance->id; // 通过父对象指针取得对应实例。
    LKMotor_Measure_t candidate;
    taskENTER_CRITICAL();
    candidate = motor->measure;
    taskEXIT_CRITICAL();
    LKMotor_Measure_t *measure = &candidate;
    uint8_t *rx_buff = _instance->rx_buff;

    measure->feed_dt = DWT_GetDeltaT(&measure->feed_dwt_cnt);

    measure->last_ecd = measure->ecd;
    measure->ecd = (uint16_t)((rx_buff[7] << 8) | rx_buff[6]);

    measure->angle_single_round = ECD_ANGLE_COEF_LK * measure->ecd;

    measure->speed_rads = (1 - SPEED_SMOOTH_COEF) * measure->speed_rads +
                          DEGREE_2_RAD * SPEED_SMOOTH_COEF * (float)((int16_t)(rx_buff[5] << 8 | rx_buff[4]));

    measure->real_current = (1 - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rx_buff[3] << 8 | rx_buff[2]));

    measure->temperature = rx_buff[1];

    // 假定两次反馈间转动小于半圈，跨越单圈零点时补减圈数。
    if (measure->ecd - measure->last_ecd > 32768)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -32768)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;

    taskENTER_CRITICAL();
    motor->measure = candidate;
    motor->feedback_timestamp_ms = _instance->rx_timestamp_ms;
    motor->feedback_sequence++;
    if (motor->feedback_sequence == 0u)
        motor->feedback_sequence++;
    taskEXIT_CRITICAL();
    (void)DaemonReportAt(motor->daemon, _instance->rx_timestamp_ms,
        DAEMON_REPORT_TRANSPORT_VALID | DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID);
}

static void LKMotorLostCallback(const DaemonEvent_t *event, void *motor_ptr)
{
    if (event == NULL || event->state != DAEMON_SOURCE_OFFLINE)
        return;
    LKMotorInstance *motor = (LKMotorInstance *)motor_ptr;
    (void)motor; /* 仅诊断使用的局部指针在关闭日志的主机替身中也保持无告警。 */
    BSP_LOG_EVENT_1(EV_MODULES_MOTOR_LKMOTOR_LK9025_C_01,
                    BSP_LOG_FIELD_I32((int)motor->motor_id));
}

LKMotorInstance *LKMotorInit(const Motor_Init_Config_s *config)
{
    if (config == NULL || config->motor_type != LK9025)
        return NULL;

    int8_t bus_index = LKBusIndexFromHandle(config->can_init_config.can_handle);
    uint32_t motor_id = config->can_init_config.tx_id;
    if (bus_index < 0 || motor_id < 1u || motor_id > LK_MOTOR_PER_BUS_CNT)
        return NULL;

    uint8_t slot = (uint8_t)(motor_id - 1u);
    if (lkmotor_instance[(uint8_t)bus_index][slot] != NULL ||
        lk_bus_count[(uint8_t)bus_index] >= LK_MOTOR_PER_BUS_CNT)
        return NULL;

    if (!LKControlConfigValid(config))
        return NULL;

    const Motor_Control_Setting_s *setting = &config->controller_setting_init_config;
    uint8_t angle_active = (uint8_t)((setting->close_loop_type & ANGLE_LOOP) != 0u &&
                                     setting->outer_loop_type == ANGLE_LOOP);
    uint8_t speed_active = (uint8_t)((setting->close_loop_type & SPEED_LOOP) != 0u &&
                                     (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) != 0u);
    uint8_t current_active = (uint8_t)((setting->close_loop_type & CURRENT_LOOP) != 0u);

    // 只初始化本配置会执行的 PID；外层环后续切换到未初始化环时，本轮会按计算失败清零。
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

    LKMotorInstance *motor = (LKMotorInstance *)malloc(sizeof(LKMotorInstance));
    if (motor == NULL)
        return NULL;
    memset(motor, 0, sizeof(LKMotorInstance));

    motor->motor_id = (uint8_t)motor_id;
    motor->can_bus_index = (uint8_t)bus_index;

    motor->motor_settings = config->controller_setting_init_config;
    motor->output_axis = config->output_axis;
    motor->current_PID = current_pid;
    motor->speed_PID = speed_pid;
    motor->angle_PID = angle_pid;
    motor->other_angle_feedback = config->controller_param_init_config.other_angle_feedback;
    motor->other_speed_feedback = config->controller_param_init_config.other_speed_feedback;
    motor->speed_feedforward = config->controller_param_init_config.speed_feedforward;
    motor->current_feedforward = config->controller_param_init_config.current_feedforward;

    DWT_GetDeltaT(&motor->measure.feed_dwt_cnt);

    Daemon_Init_Config_s daemon_config = {
        .transport_timeout_ms = 50u,
        .data_timeout_ms = 50u,
        .startup_grace_ms = 1000u,
        .event_max_age_ms = 1000u,
        .source_id = (uint16_t)(0x5000u | (config->can_init_config.rx_id & 0x01ffu)),
        .event_capacity = 1u,
        .consumer_id = DAEMON_CONSUMER_MOTOR_TASK,
        .event_policy = DAEMON_EVENT_COALESCED_STATE,
        .consumer_mode = DAEMON_CONSUMER_BOUND_TASK,
        .event_handler = LKMotorLostCallback,
        .owner_id = motor,
    };
    motor->daemon = DaemonRegister(&daemon_config);
    if (motor->daemon == NULL)
    {
        free(motor);
        return NULL;
    }

    CAN_Init_Config_s can_config = config->can_init_config;
    can_config.id = motor;
    can_config.can_module_callback = LKMotorDecode;
    can_config.rx_id = 0x140u + motor_id;
    can_config.tx_id = 0x280u;
    motor->motor_can_ins = CANRegister(&can_config);
    if (motor->motor_can_ins == NULL)
    {
        // CAN 失败时先移除离线回调，避免它引用即将释放的候选对象。
        (void)DaemonUnregister(motor->daemon);
        free(motor);
        return NULL;
    }

    // 所有可能失败的资源申请已完成，至此才让控制和接收路径看到实例。
    if (lk_sender_instance[(uint8_t)bus_index] == NULL)
        lk_sender_instance[(uint8_t)bus_index] = motor->motor_can_ins;
    LKMotorEnable(motor);
    lkmotor_instance[(uint8_t)bus_index][slot] = motor;
    lk_bus_count[(uint8_t)bus_index]++;

    return motor;
}

/* 各总线使用本组首个成功登记实例的发送缓冲区。 */
static Motor_Feedback_Status_e LKConvertMeasureToOutputAxis(
    const Motor_Output_Axis_Config_s *config, const LKMotor_Measure_t *measure,
    float *position_rad, float *speed_rad_s)
{
    if (config == NULL || measure == NULL || position_rad == NULL || speed_rad_s == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;
    float candidate_position = 0.0f;
    float candidate_speed = 0.0f;
    float unused_speed = 0.0f;
    float unused_position = 0.0f;
    // LK 反馈位置在线格式为度，速度已由接收回调换成 rad/s，分开换算避免重复转换。
    Motor_Feedback_Status_e status = MotorFeedbackConvertDegreesToOutputAxis(
        config, measure->total_angle, 0.0f, &candidate_position, &unused_speed);
    if (status != MOTOR_FEEDBACK_OK)
        return status;
    status = MotorFeedbackConvertOutputAxis(config, 0.0f, measure->speed_rads,
                                            &unused_position, &candidate_speed);
    if (status != MOTOR_FEEDBACK_OK)
        return status;
    *position_rad = candidate_position;
    *speed_rad_s = candidate_speed;
    return MOTOR_FEEDBACK_OK;
}

Motor_Feedback_Status_e LKMotorGetFeedbackSnapshot(const LKMotorInstance *motor,
                                                    uint32_t now_ms,
                                                    Motor_Feedback_Snapshot_s *snapshot)
{
    if (motor == NULL || snapshot == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;
    LKMotor_Measure_t measure;
    Motor_Output_Axis_Config_s axis;
    uint32_t timestamp_ms, sequence;
    taskENTER_CRITICAL();
    measure = motor->measure;
    axis = motor->output_axis;
    timestamp_ms = motor->feedback_timestamp_ms;
    sequence = motor->feedback_sequence;
    taskEXIT_CRITICAL();
    if (sequence == 0u)
        return MOTOR_FEEDBACK_NOT_READY;

    float position = 0.0f, speed = 0.0f;
    Motor_Feedback_Status_e status = LKConvertMeasureToOutputAxis(&axis, &measure, &position, &speed);
    if (status != MOTOR_FEEDBACK_OK)
        return status;
    const uint8_t fresh = DaemonIsOnline(motor->daemon) != 0u &&
                          (uint32_t)(now_ms - timestamp_ms) <= 50u;
    Motor_Feedback_Snapshot_s candidate = {0};
    candidate.sample_sequence = sequence;
    candidate.position = (Motor_Signal_Snapshot_s){position, MOTOR_UNIT_RAD,
        MOTOR_REFERENCE_OUTPUT_AXIS, timestamp_ms, sequence, fresh};
    candidate.speed = (Motor_Signal_Snapshot_s){speed, MOTOR_UNIT_RAD_S,
        MOTOR_REFERENCE_OUTPUT_AXIS, timestamp_ms, sequence, fresh};
    candidate.current = (Motor_Signal_Snapshot_s){
        (float)measure.real_current * (float)axis.direction, MOTOR_UNIT_DEVICE_CURRENT,
        MOTOR_REFERENCE_OUTPUT_AXIS, timestamp_ms, sequence, fresh};
    *snapshot = candidate;
    return MOTOR_FEEDBACK_OK;
}

/* 各总线使用本组首个成功登记实例的发送缓冲区。 */
void LKMotorControl(void)
{
    uint8_t tx_frames[DEVICE_CAN_CNT][8] = {{0}};
    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        for (uint8_t slot = 0u; slot < LK_MOTOR_PER_BUS_CNT; ++slot)
        {
            LKMotorInstance *motor = lkmotor_instance[bus][slot];
            if (motor == NULL)
                continue;
            LKMotor_Measure_t measure;
            Motor_Control_Setting_s setting;
            Motor_Output_Axis_Config_s axis;
            Motor_Signal_Source_s angle_source, speed_source, speed_ff, current_ff;
            Motor_Working_Type_e stop_flag;
            float pid_ref;
            taskENTER_CRITICAL();
            measure = motor->measure;
            setting = motor->motor_settings;
            axis = motor->output_axis;
            angle_source = motor->other_angle_feedback;
            speed_source = motor->other_speed_feedback;
            speed_ff = motor->speed_feedforward;
            current_ff = motor->current_feedforward;
            pid_ref = motor->pid_ref;
            stop_flag = motor->stop_flag;
            taskEXIT_CRITICAL();

            const uint8_t angle_active = (uint8_t)((setting.close_loop_type & ANGLE_LOOP) != 0u &&
                                                    setting.outer_loop_type == ANGLE_LOOP);
            const uint8_t speed_active = (uint8_t)((setting.close_loop_type & SPEED_LOOP) != 0u &&
                (setting.outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) != 0u);
            const uint8_t current_active = (uint8_t)((setting.close_loop_type & CURRENT_LOOP) != 0u);
            const uint8_t axis_outer = (uint8_t)((setting.outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) != 0u);
            const uint32_t now_ms = HAL_GetTick();
            float dt_s = 0.0f, feedback = 0.0f, position = 0.0f, speed = 0.0f;
            uint8_t internal_valid = 0u;
            PID_Status_e result = MotorPIDBegin(&motor->pid_clock, DWT_GetTimeline_us(), &dt_s);
            if ((angle_active && setting.angle_feedback_source == MOTOR_FEED) ||
                (speed_active && setting.speed_feedback_source == MOTOR_FEED))
            {
                Motor_Feedback_Status_e conversion =
                    LKConvertMeasureToOutputAxis(&axis, &measure, &position, &speed);
                if (conversion == MOTOR_FEEDBACK_OK)
                    internal_valid = 1u;
                else
                    result = conversion == MOTOR_FEEDBACK_NONFINITE ? PID_BAD_ARGUMENT : PID_NUMERIC_ERROR;
            }

            if (result == PID_OK && angle_active)
            {
                if (setting.angle_feedback_source == OTHER_FEED)
                    result = LKReadExternal(&angle_source, MOTOR_UNIT_RAD,
                        MOTOR_REFERENCE_EXTERNAL, now_ms, &feedback);
                else if (setting.angle_feedback_source == MOTOR_FEED && internal_valid)
                    feedback = position;
                else
                    result = PID_BAD_ARGUMENT;
                if (result == PID_OK)
                    result = PIDCalculate(&motor->angle_PID, feedback, pid_ref, dt_s, &pid_ref);
                if (result == PID_OK && (setting.feedforward_flag & SPEED_FEEDFORWARD) != 0u)
                {
                    result = LKReadExternal(&speed_ff, MOTOR_UNIT_RAD_S,
                        MOTOR_REFERENCE_OUTPUT_AXIS, now_ms, &feedback);
                    if (result == PID_OK)
                    {
                        pid_ref += feedback;
                        if (!isfinite(pid_ref)) result = PID_NUMERIC_ERROR;
                    }
                }
            }
            if (result == PID_OK && speed_active)
            {
                if (setting.speed_feedback_source == OTHER_FEED)
                    result = LKReadExternal(&speed_source, MOTOR_UNIT_RAD_S,
                        MOTOR_REFERENCE_EXTERNAL, now_ms, &feedback);
                else if (setting.speed_feedback_source == MOTOR_FEED && internal_valid)
                    feedback = speed;
                else
                    result = PID_BAD_ARGUMENT;
                if (result == PID_OK)
                    result = PIDCalculate(&motor->speed_PID, feedback, pid_ref, dt_s, &pid_ref);
                if (result == PID_OK && (setting.feedforward_flag & CURRENT_FEEDFORWARD) != 0u)
                {
                    result = LKReadExternal(&current_ff, MOTOR_UNIT_DEVICE_CURRENT,
                        MOTOR_REFERENCE_OUTPUT_AXIS, now_ms, &feedback);
                    if (result == PID_OK)
                    {
                        pid_ref += feedback;
                        if (!isfinite(pid_ref)) result = PID_NUMERIC_ERROR;
                    }
                }
            }
            if (result == PID_OK && current_active)
            {
                feedback = (float)measure.real_current * (axis_outer ? (float)axis.direction : 1.0f);
                result = PIDCalculate(&motor->current_PID, feedback, pid_ref, dt_s, &pid_ref);
            }
            if (result == PID_OK && axis_outer)
                pid_ref *= (float)axis.direction;
            if (result != PID_OK || !isfinite(pid_ref))
            {
                MotorPIDResetLoops(&motor->angle_PID, &motor->speed_PID, &motor->current_PID);
                motor->pid_clock.last_status = result != PID_OK ? result : PID_NUMERIC_ERROR;
                pid_ref = 0.0f;
            }
            if (stop_flag == MOTOR_STOP)
                pid_ref = 0.0f;

            int32_t command = 0;
            if (pid_ref >= 32768.0f || pid_ref <= -32769.0f)
            {
                MotorPIDResetLoops(&motor->angle_PID, &motor->speed_PID, &motor->current_PID);
                motor->pid_clock.last_status = PID_NUMERIC_ERROR;
            }
            else
                command = (int32_t)pid_ref;
            const uint16_t wire = (uint16_t)(int16_t)command;
            const uint8_t offset = (uint8_t)(motor->motor_id - 1u) * 2u;
            tx_frames[bus][offset] = (uint8_t)wire;
            tx_frames[bus][offset + 1u] = (uint8_t)(wire >> 8u);
        }
    }
    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        if (lk_sender_instance[bus] == NULL)
            continue;
        const CAN_TxFrame_t frame = {
            .identifier = 0x280u,
            .identifier_type = CAN_TX_ID_STANDARD,
            .data = tx_frames[bus],
            .length = 8u,
        };
        uint32_t request_id = 0u;
        (void)CANSubmitPeriodic(lk_sender_instance[bus]->can_handle,
                                lk_sender_instance[bus], 0u, &frame,
                                CAN_TX_PRIORITY_CONTROL, 4u, 1000u, &request_id);
    }
}

void LKMotorStop(LKMotorInstance *motor)
{
    if (motor == NULL)
        return;
    taskENTER_CRITICAL();
    motor->stop_flag = MOTOR_STOP;
    taskEXIT_CRITICAL();
}

void LKMotorEnable(LKMotorInstance *motor)
{
    if (motor == NULL)
        return;
    taskENTER_CRITICAL();
    motor->stop_flag = MOTOR_ENALBED;
    taskEXIT_CRITICAL();
}

void LKMotorSetRef(LKMotorInstance *motor, float ref)
{
    if (motor == NULL)
        return;
    taskENTER_CRITICAL();
    motor->pid_ref = ref;
    taskEXIT_CRITICAL();
}

uint8_t LKMotorIsOnline(LKMotorInstance *motor)
{
    return DaemonIsOnline(motor->daemon);
}
