/** @file xmmotor.c
* @brief 保存 XM 软件控制结构，扩展帧收发和独立任务仍保留在未启用分支中。
*/
#include "xmmotor.h"
#include <string.h>
#include <math.h>
#include "memory.h"
#include "general_def.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include "bsp_dwt.h"

static uint8_t idx;
static XMMotorInstance *xm_motor_instance[XM_MOTOR_CNT];
static osThreadId xm_task_handle[XM_MOTOR_CNT] __attribute__((unused));

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) __attribute__((unused));
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint16_to_float(uint16_t x_int, float x_min, float x_max, int bits) __attribute__((unused));
static float uint16_to_float(uint16_t x_int, float x_min, float x_max, int bits)
{
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x_int / span + x_min;
}

#ifdef XM_MOTOR_ENABLE
/* 需要本地 bsp_can 支持扩展帧(CAN_ID_EXT)后才能启用以下硬件访问 */
// 仅硬件分支使用：模式选择影响扩展帧编号和数据字段，当前依赖待适配的 BSP 类型。
static void XMMotorSetMode(XMMotor_Mode_e cmd, XMMotorInstance *motor)
{
    uint32_t identifier = 0x0300007fu;
    if (cmd == XM_CMD_MOTOR_MODE)
    {
        memset(motor->motor_can_instace->tx_buff, 0, 8);
    }
    else if (cmd == XM_CMD_CURRENT_MODE)
    {
        identifier = 0x1200007fu;
        motor->motor_can_instace->tx_buff[0] = (uint8_t)0x05;
        motor->motor_can_instace->tx_buff[1] = (uint8_t)0x70;
        motor->motor_can_instace->tx_buff[4] = (uint8_t)0x03;
    }

    const CAN_TxFrame_t frame = {
        .identifier = identifier,
        .identifier_type = CAN_TX_ID_EXTENDED,
        .data = motor->motor_can_instace->tx_buff,
        .length = 8u,
    };
    uint32_t request_id = 0u;
    (void)CANSubmitOneShot(motor->motor_can_instace->can_handle, motor->motor_can_instace,
                           &frame, CAN_TX_PRIORITY_URGENT, 100u, &request_id);
}

static void XMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    XMMotorInstance *motor = (XMMotorInstance *)motor_can->id;
    XM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    tmp = (uint16_t)(rxbuff[0] << 8 | rxbuff[1]);
    measure->position = uint16_to_float(tmp, XM_P_MIN, XM_P_MAX, 16);

    tmp = (uint16_t)(rxbuff[2] << 8 | rxbuff[3]);
    measure->velocity = uint16_to_float(tmp, XM_V_MIN, XM_V_MAX, 16);

    tmp = (uint16_t)(rxbuff[4] << 8 | rxbuff[5]);
    measure->torque = uint16_to_float(tmp, XM_T_MIN, XM_T_MAX, 16);

    measure->temperature = (float)((rxbuff[6] << 8) | rxbuff[7]) * 0.1;
}

static void XMMotorLostCallback(void *motor_ptr)
{
}
#endif /* XM_MOTOR_ENABLE */

void XMMotorCaliEncoder(XMMotorInstance *motor)
{
    // XMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    // DWT_Delay(0.1);
}

XMMotorInstance *XMMotorInit(const Motor_Init_Config_s *config)
{
    Motor_Capabilities_s capabilities;
    if (config == NULL ||
        MotorFeedbackGetCapabilities(config->motor_type, &capabilities) != MOTOR_FEEDBACK_OK ||
        MotorFeedbackValidateOutputAxis(&config->output_axis) != MOTOR_FEEDBACK_OK)
        return NULL;
    XMMotorInstance *motor = (XMMotorInstance *)malloc(sizeof(XMMotorInstance));
    if (motor == NULL)
        return NULL;
    memset(motor, 0, sizeof(XMMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    // 只检查实际启用的环，失败发生在注册 CAN 和发送模式命令之前。
    if (MotorPIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID,
                     (config->controller_setting_init_config.close_loop_type & CURRENT_LOOP) != 0u) != PID_OK ||
        MotorPIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID,
                     (config->controller_setting_init_config.close_loop_type & SPEED_LOOP) != 0u) != PID_OK ||
        MotorPIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID,
                     (config->controller_setting_init_config.close_loop_type & ANGLE_LOOP) != 0u) != PID_OK)
    {
        free(motor);
        return NULL;
    }
    motor->motor_controller.other_angle_feedback = config->controller_param_init_config.other_angle_feedback;
    motor->motor_controller.other_speed_feedback = config->controller_param_init_config.other_speed_feedback;
    motor->motor_controller.speed_feedforward = config->controller_param_init_config.speed_feedforward;
    motor->motor_controller.current_feedforward = config->controller_param_init_config.current_feedforward;

#ifdef XM_MOTOR_ENABLE
    config->can_init_config.can_module_callback = XMMotorDecode;
    config->can_init_config.id = motor;
    config->can_init_config.mode = CAN_ID_EXT; // 开启拓展帧(需 bsp_can 支持)
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    XMMotorEnable(motor);

    XMMotorSetMode(XM_CMD_CURRENT_MODE, motor);     // 设置电流环模式
    DWT_Delay(0.1);
    XMMotorSetMode(XM_CMD_MOTOR_MODE, motor);       // 使能电机
    DWT_Delay(0.1);

    Daemon_Init_Config_s conf = {
        .callback = XMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 5, // 50ms
    };
    motor->motor_daemon = DaemonRegister(&conf);
#else
    /* 骨架模式: 未启用扩展帧, 不注册 CAN 与 daemon */
    BSP_LOG_EVENT_0(EV_MODULES_MOTOR_XMMOTOR_XMMOTOR_C_01);
#endif

    xm_motor_instance[idx++] = motor;
    return motor;
}

void XMMotorSetRef(XMMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

void XMMotorChangeFeed(XMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        BSP_LOG_EVENT_0(EV_MODULES_MOTOR_XMMOTOR_XMMOTOR_C_02); // 检查是否传入了正确的LOOP类型,或发生了指针越界
}

void XMMotorEnable(XMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void XMMotorStop(XMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

/** @brief 仅改写最外层控制环，默认没有硬件任务执行。
* @param[in,out] motor 有效对象，不能为 NULL。
* @param[in] type 保存的控制环类型。
*/
void XMMotorOuterLoop(XMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

#ifdef XM_MOTOR_ENABLE
//@Todo: 目前不使用自带控制模式，通过电流反馈进行PID控制
/** @brief 硬件分支的独立任务，持续设置模式、计算所选环并发送当前值。
* @param[in] argument 有效电机对象；默认不编译此任务，循环末等待 2 ms。
*/
void XMMotorTask(void const *argument)
{
    XMMotorInstance *motor = (XMMotorInstance *)argument;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    XM_Motor_Measure_s *measure;
    float pid_ref, pid_measure, set;
    // 持续按当前设置计算；停止标记只把最终数值置零，模式命令仍会发送。
    while (1)
    {
        XMMotorSetMode(XM_CMD_CURRENT_MODE, motor);     // 设置电流环模式
        XMMotorSetMode(XM_CMD_MOTOR_MODE, motor);       // 使能电机

        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref;
        // 首轮或间隔异常时仅重新计时，最后统一将本电机输出写0。
        float control_dt = 0.0f;
        PID_Status_e pid_status = MotorPIDBegin(&motor->pid_clock, DWT_GetTimeline_us(), &control_dt);
        const uint32_t now_ms = HAL_GetTick();

        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
            pid_ref *= -1; // 设置反转

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if (pid_status == PID_OK && (motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
            {
                Motor_Signal_Snapshot_s signal = {0};
                if (MotorFeedbackReadSignal(&motor_controller->other_angle_feedback, MOTOR_UNIT_RAD,
                                            MOTOR_REFERENCE_EXTERNAL, now_ms, &signal) != MOTOR_FEEDBACK_OK)
                    pid_status = PID_BAD_ARGUMENT;
                else
                    pid_measure = signal.value;
            }
            else
                pid_measure = measure->total_round; // MOTOR_FEED,对total_round闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            if (pid_status == PID_OK)
                pid_status = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref, control_dt, &pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if (pid_status == PID_OK && (motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
            {
                Motor_Signal_Snapshot_s signal = {0};
                if (MotorFeedbackReadSignal(&motor_controller->speed_feedforward, MOTOR_UNIT_RAD_S,
                                            MOTOR_REFERENCE_OUTPUT_AXIS, now_ms, &signal) != MOTOR_FEEDBACK_OK)
                    pid_status = PID_BAD_ARGUMENT;
                else
                    pid_ref += signal.value;
            }

            if (motor_setting->speed_feedback_source == OTHER_FEED)
            {
                Motor_Signal_Snapshot_s signal = {0};
                if (MotorFeedbackReadSignal(&motor_controller->other_speed_feedback, MOTOR_UNIT_RAD_S,
                                            MOTOR_REFERENCE_EXTERNAL, now_ms, &signal) != MOTOR_FEEDBACK_OK)
                    pid_status = PID_BAD_ARGUMENT;
                else
                    pid_measure = signal.value;
            }
            else // MOTOR_FEED
                pid_measure = measure->velocity;
            // 更新pid_ref
            if (pid_status == PID_OK)
                pid_status = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref, control_dt, &pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
            pid_ref *= -1;

        // 获取最终输出,功率限制时直接输出电流值
        if (pid_status != PID_OK || !isfinite(pid_ref))
        {
            // 一环失败后丢弃本轮串级结果，恢复前清除各环已有积分和微分历史。
            MotorPIDResetLoops(&motor_controller->angle_PID, &motor_controller->speed_PID, &motor_controller->current_PID);
            motor->pid_clock.last_status = pid_status != PID_OK ? pid_status : PID_NUMERIC_ERROR;
            pid_ref = 0.0f;
        }

        set = pid_ref;

        LIMIT_MIN_MAX(set, XM_I_MIN, XM_I_MAX);

        if (motor->stop_flag == MOTOR_STOP)
            set = 0;

        // 拓展帧
        memset(motor->motor_can_instace->tx_buff, 0, 8);
        motor->motor_can_instace->tx_buff[0] = (uint8_t)(0x06);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(0x70);
        motor->motor_can_instace->tx_buff[4] = (uint8_t)((*(unsigned long*)&set & 0x000000FF));
        motor->motor_can_instace->tx_buff[5] = (uint8_t)((*(unsigned long*)&set & 0x0000FF00) >> 8);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)((*(unsigned long*)&set & 0x00FF0000) >> 16);
        motor->motor_can_instace->tx_buff[7] = (uint8_t)((*(unsigned long*)&set & 0xFF000000) >> 24);

        const CAN_TxFrame_t frame = {
            .identifier = 0x1200007fu,
            .identifier_type = CAN_TX_ID_EXTENDED,
            .data = motor->motor_can_instace->tx_buff,
            .length = 8u,
        };
        uint32_t request_id = 0u;
        (void)CANSubmitPeriodic(motor->motor_can_instace->can_handle,
                                motor->motor_can_instace, 0u, &frame,
                                CAN_TX_PRIORITY_CONTROL, 10u, 2000u, &request_id);

        osDelay(2);
    }
}
#endif /* XM_MOTOR_ENABLE */

void XMMotorControlInit()
{
#ifdef XM_MOTOR_ENABLE
    char xm_task_name[5] = "xm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char xm_id_buff[2] = {0};
        __itoa(i, xm_id_buff, 10);
        strcat(xm_task_name, xm_id_buff);
        osThreadDef(xm_task_name, XMMotorTask, osPriorityNormal, 0, 128);
        xm_task_handle[i] = osThreadCreate(osThread(xm_task_name), xm_motor_instance[i]);
    }
#else
    /* 骨架模式: 不创建任务 */
    if (idx)
        BSP_LOG_EVENT_1(EV_MODULES_MOTOR_XMMOTOR_XMMOTOR_C_03, BSP_LOG_FIELD_I32(idx));
#endif
}
