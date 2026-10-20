/**
 * @file LK9025.c
 * @brief 管理 LK 实例、解析反馈，并按物理 CAN 总线分别提交多电机帧。
 * 实例只在启动阶段登记，资源在运行期间长期保留。
 */
#include "LK9025.h"
#include <string.h>
#include <math.h>
#include "stdlib.h"
#include "general_def.h"
#include "daemon.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

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

static uint8_t LKFeedbackConfigValid(Feedback_Source_e source, const float *external_feedback)
{
    if (source == MOTOR_FEED)
        return 1u;
    return (uint8_t)(source == OTHER_FEED && external_feedback != NULL);
}

static PID_Status_e LKReadFeedback(Feedback_Source_e source, const float *external_feedback,
                                   float internal_feedback, float *feedback)
{
    if (feedback == NULL)
        return PID_BAD_ARGUMENT;

    float candidate;
    if (source == MOTOR_FEED)
        candidate = internal_feedback;
    else if (source == OTHER_FEED && external_feedback != NULL)
        candidate = *external_feedback;
    else
        return PID_BAD_ARGUMENT;

    if (!isfinite(candidate))
        return PID_BAD_ARGUMENT;
    *feedback = candidate;
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

    if ((angle_active != 0u &&
         (!LKFeedbackConfigValid(setting->angle_feedback_source, controller->other_angle_feedback_ptr) ||
          ((setting->feedforward_flag & SPEED_FEEDFORWARD) != 0u &&
           controller->speed_feedforward_ptr == NULL))) ||
        (speed_active != 0u &&
         (!LKFeedbackConfigValid(setting->speed_feedback_source, controller->other_speed_feedback_ptr) ||
          ((setting->feedforward_flag & CURRENT_FEEDFORWARD) != 0u &&
           controller->current_feedforward_ptr == NULL))))
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
    LKMotorInstance *motor = (LKMotorInstance *)_instance->id; // 通过caninstance保存的father id获取对应的motorinstance
    LKMotor_Measure_t *measure = &motor->measure;
    uint8_t *rx_buff = _instance->rx_buff;

    DaemonReload(motor->daemon); // 喂狗
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
}

static void LKMotorLostCallback(void *motor_ptr)
{
    LKMotorInstance *motor = (LKMotorInstance *)motor_ptr;
    static LogRateLimit_t rl_lk_lost = {0};
    if (LogRateLimitAllow(&rl_lk_lost, 1000u))
        LOGWARNING("[LKMotor] 电机离线: id %d (累计 %lu 次, 期间限速 %lu 条)",
                   (int)motor->motor_id,
                   (unsigned long)rl_lk_lost.total, (unsigned long)rl_lk_lost.dropped);
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
    motor->current_PID = current_pid;
    motor->speed_PID = speed_pid;
    motor->angle_PID = angle_pid;
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    motor->current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;

    DWT_GetDeltaT(&motor->measure.feed_dwt_cnt);

    Daemon_Init_Config_s daemon_config = {
        .callback = LKMotorLostCallback,
        .owner_id = motor,
        .reload_count = 5, // 50ms
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
void LKMotorControl(void)
{
    float pid_measure, pid_ref;
    int16_t set;
    LKMotorInstance *motor;
    LKMotor_Measure_t *measure;
    Motor_Control_Setting_s *setting;
    uint8_t tx_frames[DEVICE_CAN_CNT][8] = {{0}};

    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        for (uint8_t slot = 0u; slot < LK_MOTOR_PER_BUS_CNT; ++slot)
        {
            motor = lkmotor_instance[bus][slot];
            if (motor == NULL)
                continue;

            measure = &motor->measure;
            setting = &motor->motor_settings;
            pid_ref = motor->pid_ref;
            // 首轮或间隔异常时仅重新计时，最后统一将本电机输出写0。
            float control_dt = 0.0f;
            PID_Status_e pid_status = MotorPIDBegin(&motor->pid_clock, DWT_GetTimeline_us(), &control_dt);

            // 角度环使用角度来源；外部值和电机总角度均以 deg 参与 PID。
            if (pid_status == PID_OK && (setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
            {
                pid_status = LKReadFeedback(setting->angle_feedback_source,
                                            motor->other_angle_feedback_ptr,
                                            measure->total_angle, &pid_measure);
                if (pid_status == PID_OK)
                    pid_status = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref, control_dt, &pid_ref);
                if (pid_status == PID_OK && (setting->feedforward_flag & SPEED_FEEDFORWARD) != 0u)
                {
                    if (motor->speed_feedforward_ptr == NULL)
                        pid_status = PID_BAD_ARGUMENT;
                    else
                    {
                        float feedforward = *motor->speed_feedforward_ptr;
                        if (!isfinite(feedforward))
                            pid_status = PID_BAD_ARGUMENT;
                        else
                        {
                            pid_ref += feedforward;
                            if (!isfinite(pid_ref))
                                pid_status = PID_NUMERIC_ERROR;
                        }
                    }
                }
            }

            if (pid_status == PID_OK && (setting->close_loop_type & SPEED_LOOP) &&
                (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
            {
                // 速度闭环按 speed_feedback_source 选反馈，并使用自己的 PID 历史。
                pid_status = LKReadFeedback(setting->speed_feedback_source,
                                            motor->other_speed_feedback_ptr,
                                            measure->speed_rads, &pid_measure);
                if (pid_status == PID_OK)
                    pid_status = PIDCalculate(&motor->speed_PID, pid_measure, pid_ref, control_dt, &pid_ref);
                if (pid_status == PID_OK && (setting->feedforward_flag & CURRENT_FEEDFORWARD) != 0u)
                {
                    if (motor->current_feedforward_ptr == NULL)
                        pid_status = PID_BAD_ARGUMENT;
                    else
                    {
                        float feedforward = *motor->current_feedforward_ptr;
                        if (!isfinite(feedforward))
                            pid_status = PID_BAD_ARGUMENT;
                        else
                        {
                            pid_ref += feedforward;
                            if (!isfinite(pid_ref))
                                pid_status = PID_NUMERIC_ERROR;
                        }
                    }
                }
            }

            if ((setting->close_loop_type & CURRENT_LOOP) != 0u && pid_status == PID_OK)
            {
                pid_measure = (float)measure->real_current;
                pid_status = PIDCalculate(&motor->current_PID, pid_measure, pid_ref, control_dt, &pid_ref);
            }

            if (pid_status != PID_OK || !isfinite(pid_ref))
            {
                // 一环失败后丢弃本轮串级结果，恢复前清除各环已有积分和微分历史。
                MotorPIDResetLoops(&motor->angle_PID, &motor->speed_PID, &motor->current_PID);
                motor->pid_clock.last_status = pid_status != PID_OK ? pid_status : PID_NUMERIC_ERROR;
                pid_ref = 0.0f;
            }

            if (motor->stop_flag == MOTOR_STOP)
                pid_ref = 0.0f;

            // 先按原有向零截断规则检查可表示范围，避免越界浮点转整数。
            int32_t command = 0;
            if (pid_ref >= 32768.0f || pid_ref <= -32769.0f)
            {
                MotorPIDResetLoops(&motor->angle_PID, &motor->speed_PID, &motor->current_PID);
                motor->pid_clock.last_status = PID_NUMERIC_ERROR;
            }
            else
            {
                command = (int32_t)pid_ref;
                if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                    command = -command;
                if (command < -32768 || command > 32767)
                {
                    // 反向后无法装入有符号16位命令时，按本周期计算失败清零。
                    MotorPIDResetLoops(&motor->angle_PID, &motor->speed_PID, &motor->current_PID);
                    motor->pid_clock.last_status = PID_NUMERIC_ERROR;
                    command = 0;
                }
            }
            set = (int16_t)command;

            // 用编号选择固定位置，并显式按 CAN 小端顺序写字节。
            uint16_t wire_value = (uint16_t)set;
            uint8_t offset = (uint8_t)(motor->motor_id - 1u) * 2u;
            tx_frames[bus][offset] = (uint8_t)(wire_value & 0xFFu);
            tx_frames[bus][offset + 1u] = (uint8_t)(wire_value >> 8);
        }
    }

    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        if (lk_sender_instance[bus] == NULL)
            continue;
        // 先完整准备本组八字节，再提交；失败帧不会留作下周期重发。
        memcpy(lk_sender_instance[bus]->tx_buff, tx_frames[bus], sizeof(tx_frames[bus]));
        (void)CANTransmit(lk_sender_instance[bus], 0.2f);
    }
}

void LKMotorStop(LKMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void LKMotorEnable(LKMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void LKMotorSetRef(LKMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

uint8_t LKMotorIsOnline(LKMotorInstance *motor)
{
    return DaemonIsOnline(motor->daemon);
}
