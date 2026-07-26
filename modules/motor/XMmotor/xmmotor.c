#include "xmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
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
static void XMMotorSetMode(XMMotor_Mode_e cmd, XMMotorInstance *motor)
{
    if (cmd == XM_CMD_MOTOR_MODE)
    {
        memset(motor->motor_can_instace->tx_buff, 0, 8);
        motor->motor_can_instace->txconf.ExtId = 0x0300007f;
    }
    else if (cmd == XM_CMD_CURRENT_MODE)
    {
        motor->motor_can_instace->txconf.ExtId = 0x12 << 24 | 0x0 << 8 | motor->motor_can_instace->tx_id;
        motor->motor_can_instace->txconf.ExtId = 0x1200007f;
        motor->motor_can_instace->tx_buff[0] = (uint8_t)0x05;
        motor->motor_can_instace->tx_buff[1] = (uint8_t)0x70;
        motor->motor_can_instace->tx_buff[4] = (uint8_t)0x03;
    }

    CANTransmit(motor->motor_can_instace, 1);
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

XMMotorInstance *XMMotorInit(Motor_Init_Config_s *config)
{
    XMMotorInstance *motor = (XMMotorInstance *)malloc(sizeof(XMMotorInstance));
    memset(motor, 0, sizeof(XMMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

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
    LOGWARNING("[xmmotor] XM_MOTOR_ENABLE not defined, hardware not initialized");
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
        LOGERROR("[xmmotor] loop type error, check memory access and func param"); // 检查是否传入了正确的LOOP类型,或发生了指针越界
}

void XMMotorEnable(XMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void XMMotorStop(XMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void XMMotorOuterLoop(XMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

#ifdef XM_MOTOR_ENABLE
//@Todo: 目前不使用自带控制模式，通过电流反馈进行PID控制
void XMMotorTask(void const *argument)
{
    XMMotorInstance *motor = (XMMotorInstance *)argument;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    XM_Motor_Measure_s *measure;
    float pid_ref, pid_measure, set;
    while (1)
    {
        XMMotorSetMode(XM_CMD_CURRENT_MODE, motor);     // 设置电流环模式
        XMMotorSetMode(XM_CMD_MOTOR_MODE, motor);       // 使能电机

        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref;

        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
            pid_ref *= -1; // 设置反转

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_round; // MOTOR_FEED,对total_round闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->velocity;
            // 更新pid_ref
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
            pid_ref *= -1;

        // 获取最终输出,功率限制时直接输出电流值
        set = pid_ref;

        LIMIT_MIN_MAX(set, XM_I_MIN, XM_I_MAX);

        if (motor->stop_flag == MOTOR_STOP)
            set = 0;

        // 拓展帧
        motor->motor_can_instace->txconf.ExtId = 0x12 << 24 | 0x0 << 8 | motor->motor_can_instace->tx_id;
        motor->motor_can_instace->txconf.StdId = 0;

        memset(motor->motor_can_instace->tx_buff, 0, 8);
        motor->motor_can_instace->tx_buff[0] = (uint8_t)(0x06);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(0x70);
        motor->motor_can_instace->tx_buff[4] = (uint8_t)((*(unsigned long*)&set & 0x000000FF));
        motor->motor_can_instace->tx_buff[5] = (uint8_t)((*(unsigned long*)&set & 0x0000FF00) >> 8);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)((*(unsigned long*)&set & 0x00FF0000) >> 16);
        motor->motor_can_instace->tx_buff[7] = (uint8_t)((*(unsigned long*)&set & 0xFF000000) >> 24);

        CANTransmit(motor->motor_can_instace, 1);

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
        LOGINFO("[xmmotor] %d XM motor instance registered (skeleton, hardware disabled)", idx);
#endif
}
