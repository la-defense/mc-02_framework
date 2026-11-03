/** @file ht_control_test.c
 * @brief 通过生产 HT04 初始化和任务入口验证闭环反馈、前馈及失败零力矩。
 * @note CAN、Daemon、任务延时和 DWT 只在内存中模拟，不连接或驱动真实电机。
 */
#include "HT04.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
#include "test_check.h"
#include "general_def.h"
#include "motor_feedback_test_support.h"
#include "../../bsp/os/platform_sync.h"
#include "../../bsp/dwt/platform_probe.h"
#include <float.h>
#include <math.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

FDCAN_HandleTypeDef hcan1, hcan2; // 为生产板级映射提供两条测试 CAN 总线。
static CANInstance can_instances[2];
static void (*callbacks[2])(CANInstance *);
static DaemonInstance daemons[2];
static unsigned registered;
static unsigned daemon_count;
static unsigned transmit_count;
static unsigned task_count;
static struct
{
    osTaskFunction_t entry;
    void *argument;
} tasks[2];
static uint64_t now_us;
static unsigned task_iterations_remaining;
static jmp_buf task_exit;

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (config == NULL || registered >= 2u)
        return NULL;
    CANInstance *instance = &can_instances[registered];
    memset(instance, 0, sizeof(*instance));
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->id = config->id;
    callbacks[registered] = config->can_module_callback;
    ++registered;
    return instance;
}

static CAN_TxSubmitStatus_t fake_tx_submit(const CAN_TxFrame_t *frame, uint32_t *request_id)
{
    TEST_CHECK(frame != NULL && request_id != NULL && frame->length <= 8u);
    ++transmit_count;
    *request_id = transmit_count;
    return CAN_TX_SUBMIT_ACCEPTED;
}
CAN_TxSubmitStatus_t CANSubmitPeriodic(FDCAN_HandleTypeDef *hcan,const void *source,uint32_t key,
                                       const CAN_TxFrame_t *frame,CAN_TxPriority_t priority,
                                       uint32_t deadline_ms,uint32_t period_us,uint32_t *request_id)
{(void)hcan;(void)source;(void)key;(void)priority;(void)deadline_ms;(void)period_us;return fake_tx_submit(frame,request_id);}
CAN_TxSubmitStatus_t CANSubmitOneShot(FDCAN_HandleTypeDef *hcan,const void *source,
                                      const CAN_TxFrame_t *frame,CAN_TxPriority_t priority,
                                      uint32_t deadline_ms,uint32_t *request_id)
{(void)hcan;(void)source;(void)priority;(void)deadline_ms;return fake_tx_submit(frame,request_id);}

DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config)
{
    (void)config;
    if (daemon_count >= 2u)
        return NULL;
    return &daemons[daemon_count++];
}

void DaemonReload(DaemonInstance *instance) { (void)instance; }
DaemonStatus_t DaemonReportAt(DaemonInstance *instance, uint32_t received_ms, uint8_t flags)
{ (void)instance; (void)received_ms; (void)flags; return DAEMON_OK; }
uint8_t DaemonIsOnline(DaemonInstance *instance) { (void)instance; return 1u; }
uint8_t DaemonUnregister(DaemonInstance *instance) { (void)instance; return 1u; }
uint64_t DWT_GetTimeline_us(void) { return now_us; }
float DWT_GetDeltaT(uint32_t *counter) { ++*counter; return 0.001f; }
void DWT_Delay(float seconds) { (void)seconds; }
uint32_t HAL_GetTick(void) { return (uint32_t)(now_us / 1000u); }
void PlatformTaskCriticalEnter(void) {} // 该测试的控制循环按确定顺序执行。
void PlatformTaskCriticalExit(void) {}
uint32_t PlatformTimeNowMs(void) { return HAL_GetTick(); }
uint64_t PlatformTimeNowUs(void) { return DWT_GetTimeline_us(); }
float PlatformProbeDeltaSeconds(uint32_t *counter) { return DWT_GetDeltaT(counter); }

void *osThreadCreate(const osThreadDef_t *definition, void *argument)
{
    TEST_CHECK(definition != NULL && definition->pthread != NULL && task_count < 2u);
    TEST_CHECK(strcmp(definition->name, "ht_task_name") == 0 && definition->stacksize == 128u);
    tasks[task_count].entry = definition->pthread;
    tasks[task_count].argument = argument;
    return &tasks[task_count++];
}

void osDelay(uint32_t milliseconds)
{
    now_us += (uint64_t)milliseconds * 1000u;
    if (task_iterations_remaining != 0u && --task_iterations_remaining == 0u)
        longjmp(task_exit, 1);
}

char *__itoa(int value, char *output, int base)
{
    TEST_CHECK(value >= 0 && value <= 1 && base == 10);
    output[0] = (char)('0' + value);
    output[1] = '\0';
    return output;
}

static PID_Init_Config_s pid_config(float kp, float max_out)
{
    PID_Init_Config_s config = {0};
    config.Kp = kp;
    config.MaxOut = max_out;
    config.DtMin_s = MOTOR_PID_DT_MIN_S;
    config.DtMax_s = MOTOR_PID_DT_MAX_S;
    return config;
}

static Motor_Init_Config_s config_for(float *external_angle, float *external_speed,
                                      float *speed_feedforward, float *current_feedforward,
                                      Feedfoward_Type_e feedforward_flags)
{
    Motor_Init_Config_s config = {0};
    Motor_Control_Setting_s *setting = &config.controller_setting_init_config;
    Motor_Controller_Init_s *controller = &config.controller_param_init_config;
    config.motor_type = HT04;
    config.can_init_config.can_bus = PLATFORM_CAN_BUS_1; // 本场景的测试电机使用第一条总线。
    config.output_axis = (Motor_Output_Axis_Config_s){1.0f, MOTOR_AXIS_DIRECTION_NORMAL};
    setting->outer_loop_type = ANGLE_LOOP;
    setting->close_loop_type = ALL_THREE_LOOP;
    setting->motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    setting->angle_feedback_source = MOTOR_FEED;
    setting->speed_feedback_source = MOTOR_FEED;
    setting->feedforward_flag = feedforward_flags;
    controller->other_angle_feedback = MotorFeedbackTestMakeSource(external_angle,
        MotorFeedbackTestReadRadExternal, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL);
    controller->other_speed_feedback = MotorFeedbackTestMakeSource(external_speed,
        MotorFeedbackTestReadRadSExternal, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_EXTERNAL);
    controller->speed_feedforward = MotorFeedbackTestMakeSource(speed_feedforward,
        MotorFeedbackTestReadRadSOutput, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_OUTPUT_AXIS);
    controller->current_feedforward = MotorFeedbackTestMakeSource(current_feedforward,
        MotorFeedbackTestReadDeviceTorqueOutput, MOTOR_UNIT_DEVICE_TORQUE,
        MOTOR_REFERENCE_OUTPUT_AXIS);
    controller->angle_PID = pid_config(0.5f, 1000.0f);
    // 速度输入从每秒度数换成 rad/s 后，按比例换算保留旧输出量级。
    controller->speed_PID = pid_config((float)RAD_2_DEGREE, 1000.0f);
    controller->current_PID = pid_config(1.0f, 1000.0f);
    return config;
}

static uint16_t expected_wire_torque(double torque)
{
    if (torque < -18.0)
        torque = -18.0;
    if (torque > 18.0)
        torque = 18.0;
    return (uint16_t)((torque + 18.0) * 4095.0 / 36.0);
}

static void run_task(unsigned index, unsigned iterations)
{
    TEST_CHECK(index < task_count && iterations != 0u);
    task_iterations_remaining = iterations;
    if (setjmp(task_exit) == 0)
    {
        tasks[index].entry(tasks[index].argument);
        TEST_CHECK(0 && "HTMotorTask unexpectedly returned");
    }
    TEST_CHECK(task_iterations_remaining == 0u);
}

static void check_torque(const HTMotorInstance *motor, double torque)
{
    uint16_t expected = expected_wire_torque(torque);
    TEST_CHECK(motor->motor_can_instace->tx_buff[6] == (uint8_t)(expected >> 8u));
    TEST_CHECK(motor->motor_can_instace->tx_buff[7] == (uint8_t)expected);
}

static void test_feedback_combination(unsigned combination)
{
    float external_angle = 20.0f / (float)RAD_2_DEGREE;
    float external_speed = 4.0f / (float)RAD_2_DEGREE;
    Motor_Init_Config_s config = config_for(&external_angle, &external_speed, NULL, NULL, FEEDFORWARD_NONE);
    config.controller_setting_init_config.angle_feedback_source =
        (combination & 1u) != 0u ? OTHER_FEED : MOTOR_FEED;
    config.controller_setting_init_config.speed_feedback_source =
        (combination & 2u) != 0u ? OTHER_FEED : MOTOR_FEED;
    HTMotorInstance *motor = HTMotorInit(&config);
    TEST_CHECK(motor != NULL);
    TEST_CHECK(motor->other_angle_feedback.context == &external_angle);
    TEST_CHECK(motor->other_speed_feedback.context == &external_speed);
    HTMotorSetRef(motor, 30.0f / (float)RAD_2_DEGREE);
    motor->measure.total_angle = 10.0f / (float)RAD_2_DEGREE;
    motor->measure.speed_rads = 2.0f / (float)RAD_2_DEGREE;
    motor->measure.real_current = 3.0f;
    HTMotorControlInit();
    TEST_CHECK(task_count == 1u && tasks[0].argument == motor);

    now_us = 0u;
    run_task(0u, 2u);
    static const double expected[4] = {5.0, 0.0, 3.0, -2.0};
    check_torque(motor, expected[combination]);
    TEST_CHECK(transmit_count >= 4u);
}

static void test_direction_combination(uint8_t speed_outer, unsigned combination)
{
    TEST_CHECK(combination < 2u);
    Motor_Init_Config_s config = config_for(NULL, NULL, NULL, NULL, FEEDFORWARD_NONE);
    Motor_Control_Setting_s *setting = &config.controller_setting_init_config;
    setting->outer_loop_type = speed_outer ? SPEED_LOOP : ANGLE_LOOP;
    setting->close_loop_type = speed_outer ? SPEED_LOOP : (ANGLE_LOOP | SPEED_LOOP);
    config.output_axis.direction = combination == 0u ? MOTOR_AXIS_DIRECTION_NORMAL :
                                                       MOTOR_AXIS_DIRECTION_REVERSE;
    config.controller_param_init_config.angle_PID = pid_config(0.25f, 1000.0f);
    config.controller_param_init_config.speed_PID = pid_config(
        (speed_outer ? 0.25f : 1.0f) * (float)RAD_2_DEGREE, 1000.0f);

    HTMotorInstance *motor = HTMotorInit(&config);
    TEST_CHECK(motor != NULL);
    HTMotorSetRef(motor, 30.0f / (float)RAD_2_DEGREE);
    motor->measure.total_angle = 10.0f / (float)RAD_2_DEGREE;
    motor->measure.speed_rads = (speed_outer ? 10.0f : 2.0f) / (float)RAD_2_DEGREE;
    HTMotorControlInit();

    now_us = 0u;
    run_task(0u, 2u);
    const double positive_torque = speed_outer ?
        (combination == 0u ? 5.0 : -10.0) : (combination == 0u ? 3.0 : -12.0);
    check_torque(motor, positive_torque);

    HTMotorSetRef(motor, -30.0f / (float)RAD_2_DEGREE);
    run_task(0u, 1u);
    const double negative_torque = speed_outer ?
        (combination == 0u ? -10.0 : 5.0) : (combination == 0u ? -12.0 : 3.0);
    check_torque(motor, negative_torque);

    HTMotorSetRef(motor, 0.0f);
    motor->measure.total_angle = 0.0f;
    motor->measure.speed_rads = 0.0f;
    run_task(0u, 1u);
    check_torque(motor, 0.0);
}

static void test_current_loop_ignores_axis_direction_flags(void)
{
    Motor_Init_Config_s config = config_for(NULL, NULL, NULL, NULL, FEEDFORWARD_NONE);
    config.controller_setting_init_config.outer_loop_type = CURRENT_LOOP;
    config.controller_setting_init_config.close_loop_type = CURRENT_LOOP;
    config.controller_param_init_config.current_PID = pid_config(0.25f, 1000.0f);
    HTMotorInstance *motor = HTMotorInit(&config);
    TEST_CHECK(motor != NULL);
    HTMotorSetRef(motor, 30.0f);
    HTMotorControlInit();

    now_us = 0u;
    run_task(0u, 2u);
    check_torque(motor, 7.5);
}

static void test_open_loop_keeps_motor_direction(void)
{
    Motor_Init_Config_s config = config_for(NULL, NULL, NULL, NULL, FEEDFORWARD_NONE);
    Motor_Control_Setting_s *setting = &config.controller_setting_init_config;
    setting->outer_loop_type = OPEN_LOOP;
    setting->close_loop_type = OPEN_LOOP;
    config.output_axis.direction = MOTOR_AXIS_DIRECTION_REVERSE;
    HTMotorInstance *motor = HTMotorInit(&config);
    TEST_CHECK(motor != NULL);
    HTMotorSetRef(motor, 9.0f);
    HTMotorControlInit();

    now_us = 0u;
    run_task(0u, 2u);
    check_torque(motor, 9.0);

    HTMotorSetRef(motor, -9.0f);
    run_task(0u, 1u);
    check_torque(motor, -9.0);

    HTMotorSetRef(motor, 0.0f);
    run_task(0u, 1u);
    check_torque(motor, 0.0);
}

static void test_invalid_direction_flags_rejected(void)
{
    Motor_Init_Config_s config = config_for(NULL, NULL, NULL, NULL, FEEDFORWARD_NONE);
    config.can_init_config.can_bus = PLATFORM_CAN_INVALID; // 无绑定时不得登记来源或发送配置。
    TEST_CHECK(HTMotorInit(&config) == NULL);
    config = config_for(NULL, NULL, NULL, NULL, FEEDFORWARD_NONE);
    config.output_axis.direction = (Motor_Axis_Direction_e)2;
    TEST_CHECK(HTMotorInit(&config) == NULL);
    config = config_for(NULL, NULL, NULL, NULL, FEEDFORWARD_NONE);
    config.controller_setting_init_config.motor_reverse_flag = (Motor_Reverse_Flag_e)2;
    TEST_CHECK(HTMotorInit(&config) == NULL);
    config = config_for(NULL, NULL, NULL, NULL, FEEDFORWARD_NONE);
    config.controller_setting_init_config.feedback_reverse_flag = (Feedback_Reverse_Flag_e)2;
    TEST_CHECK(HTMotorInit(&config) == NULL);
    TEST_CHECK(registered == 0u && daemon_count == 0u && transmit_count == 0u && task_count == 0u);
}

static void test_feedforward_copy_and_use(unsigned flags)
{
    float speed_feedforward = 2.0f / (float)RAD_2_DEGREE;
    float current_feedforward = 1.0f;
    Motor_Init_Config_s config = config_for(NULL, NULL, &speed_feedforward,
                                            &current_feedforward, (Feedfoward_Type_e)flags);
    HTMotorInstance *motor = HTMotorInit(&config);
    TEST_CHECK(motor != NULL);
    TEST_CHECK(motor->speed_feedforward.context == &speed_feedforward);
    TEST_CHECK(motor->current_feedforward.context == &current_feedforward);
    HTMotorSetRef(motor, 30.0f / (float)RAD_2_DEGREE);
    motor->measure.total_angle = 10.0f / (float)RAD_2_DEGREE;
    motor->measure.speed_rads = 2.0f / (float)RAD_2_DEGREE;
    motor->measure.real_current = 3.0f;
    HTMotorControlInit();
    now_us = 0u;
    run_task(0u, 2u);

    double target = 10.0;
    if ((flags & SPEED_FEEDFORWARD) != 0u)
        target += 2.0;
    double speed_output = target - 2.0;
    if ((flags & CURRENT_FEEDFORWARD) != 0u)
        speed_output += 1.0;
    check_torque(motor, speed_output - 3.0);
}

static void test_init_rejects_missing_inputs(void)
{
    Motor_Init_Config_s config = config_for(NULL, NULL, NULL, NULL, FEEDFORWARD_NONE);
    config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    TEST_CHECK(HTMotorInit(&config) == NULL);
    TEST_CHECK(registered == 0u && daemon_count == 0u && transmit_count == 0u);

    config = config_for(NULL, NULL, NULL, NULL, FEEDFORWARD_NONE);
    config.controller_setting_init_config.angle_feedback_source = (Feedback_Source_e)2;
    TEST_CHECK(HTMotorInit(&config) == NULL);
    TEST_CHECK(registered == 0u && daemon_count == 0u && transmit_count == 0u);

    config = config_for(NULL, NULL, NULL, NULL, SPEED_FEEDFORWARD);
    TEST_CHECK(HTMotorInit(&config) == NULL);
    TEST_CHECK(registered == 0u && daemon_count == 0u && transmit_count == 0u);

    config = config_for(NULL, NULL, NULL, NULL, CURRENT_FEEDFORWARD);
    TEST_CHECK(HTMotorInit(&config) == NULL);
    TEST_CHECK(registered == 0u && daemon_count == 0u && transmit_count == 0u);

    // 未执行的速度环可保留外部来源空指针和未初始化 PID。
    config = config_for(NULL, NULL, NULL, NULL, FEEDFORWARD_NONE);
    config.controller_setting_init_config.close_loop_type = ANGLE_LOOP;
    config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
    config.controller_param_init_config.speed_PID = (PID_Init_Config_s){0};
    TEST_CHECK(HTMotorInit(&config) != NULL);
    TEST_CHECK(registered == 1u && daemon_count == 1u);
}

static void test_runtime_failure_and_recovery(void)
{
    float external_angle = 20.0f / (float)RAD_2_DEGREE;
    Motor_Init_Config_s config = config_for(&external_angle, NULL, NULL, NULL, FEEDFORWARD_NONE);
    config.controller_setting_init_config.close_loop_type = ANGLE_LOOP;
    config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
    config.controller_param_init_config.speed_PID = (PID_Init_Config_s){0};
    HTMotorInstance *motor = HTMotorInit(&config);
    TEST_CHECK(motor != NULL);
    HTMotorSetRef(motor, 30.0f / (float)RAD_2_DEGREE);
    motor->measure.total_angle = 10.0f / (float)RAD_2_DEGREE;
    HTMotorControlInit();

    now_us = 0u;
    run_task(0u, 2u);
    check_torque(motor, 0.5 * (10.0 / (double)RAD_2_DEGREE));

    external_angle = NAN;
    run_task(0u, 1u);
    TEST_CHECK(motor->pid_clock.last_status == PID_NUMERIC_ERROR);
    check_torque(motor, 0.0);

    external_angle = 10.0f / (float)RAD_2_DEGREE;
    run_task(0u, 1u);
    check_torque(motor, 0.5 * (20.0 / (double)RAD_2_DEGREE));

    motor->motor_settings.angle_feedback_source = (Feedback_Source_e)2;
    run_task(0u, 1u);
    TEST_CHECK(motor->pid_clock.last_status == PID_BAD_ARGUMENT);
    check_torque(motor, 0.0);

    motor->motor_settings.angle_feedback_source = MOTOR_FEED;
    run_task(0u, 1u);
    check_torque(motor, 0.5 * (20.0 / (double)RAD_2_DEGREE));

    // 切到启动时未配置的速度 PID 后，本轮失败并清零；切回有效角度环可恢复。
    motor->motor_settings.close_loop_type = ANGLE_LOOP | SPEED_LOOP;
    motor->motor_settings.outer_loop_type = SPEED_LOOP;
    motor->motor_settings.speed_feedback_source = MOTOR_FEED;
    run_task(0u, 1u);
    TEST_CHECK(motor->pid_clock.last_status == PID_NOT_INITIALIZED);
    check_torque(motor, 0.0);
    motor->motor_settings.close_loop_type = ANGLE_LOOP;
    motor->motor_settings.outer_loop_type = ANGLE_LOOP;
    run_task(0u, 1u);
    check_torque(motor, 0.5 * (20.0 / (double)RAD_2_DEGREE));
}

static void test_runtime_speed_failure_and_recovery(void)
{
    float external_speed = 4.0f / (float)RAD_2_DEGREE;
    Motor_Init_Config_s config = config_for(NULL, &external_speed, NULL, NULL, FEEDFORWARD_NONE);
    config.controller_setting_init_config.outer_loop_type = SPEED_LOOP;
    config.controller_setting_init_config.close_loop_type = SPEED_LOOP | CURRENT_LOOP;
    config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
    HTMotorInstance *motor = HTMotorInit(&config);
    TEST_CHECK(motor != NULL);
    HTMotorSetRef(motor, 30.0f / (float)RAD_2_DEGREE);
    motor->measure.speed_rads = 2.0f / (float)RAD_2_DEGREE;
    motor->measure.real_current = 3.0f;
    HTMotorControlInit();

    now_us = 0u;
    run_task(0u, 1u);
    run_task(0u, 1u);
    check_torque(motor, 23.0);

    external_speed = NAN;
    run_task(0u, 1u);
    TEST_CHECK(motor->pid_clock.last_status == PID_NUMERIC_ERROR);
    check_torque(motor, 0.0);
    PID_State_t speed_state = {0};
    PID_State_t current_state = {0};
    TEST_CHECK(PIDGetState(&motor->speed_PID, &speed_state) == PID_OK);
    TEST_CHECK(PIDGetState(&motor->current_PID, &current_state) == PID_OK);
    TEST_CHECK(speed_state.Err == 0.0f && speed_state.Last_Err == 0.0f &&
               speed_state.ITerm == 0.0f && speed_state.Iout == 0.0f &&
               speed_state.Output == 0.0f);
    TEST_CHECK(current_state.Err == 0.0f && current_state.Last_Err == 0.0f &&
               current_state.ITerm == 0.0f && current_state.Iout == 0.0f &&
               current_state.Output == 0.0f);

    // 恢复有限速度后先重新建立 PID 历史，再检查非法来源和指针。
    external_speed = 4.0f / (float)RAD_2_DEGREE;
    run_task(0u, 1u);
    check_torque(motor, 23.0);

    motor->motor_settings.speed_feedback_source = (Feedback_Source_e)2;
    run_task(0u, 1u);
    TEST_CHECK(motor->pid_clock.last_status == PID_BAD_ARGUMENT);
    check_torque(motor, 0.0);

    motor->motor_settings.speed_feedback_source = OTHER_FEED;
    motor->other_speed_feedback.read = NULL;
    run_task(0u, 1u);
    TEST_CHECK(motor->pid_clock.last_status == PID_BAD_ARGUMENT);
    check_torque(motor, 0.0);
    motor->other_speed_feedback = MotorFeedbackTestMakeSource(&external_speed,
        MotorFeedbackTestReadRadSExternal, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_EXTERNAL);

    // FLT_MAX 本身有限，乘以弧度转角度系数后会溢出，不能送入 PID。
    external_speed = FLT_MAX;
    run_task(0u, 1u);
    TEST_CHECK(motor->pid_clock.last_status == PID_NUMERIC_ERROR);
    check_torque(motor, 0.0);

    external_speed = 4.0f / (float)RAD_2_DEGREE;
    run_task(0u, 1u);
    check_torque(motor, 23.0);
}

static void test_feedforward_overflow(void)
{
    float speed_feedforward = INFINITY;
    Motor_Init_Config_s config = config_for(NULL, NULL, &speed_feedforward, NULL,
                                            SPEED_FEEDFORWARD);
    config.controller_param_init_config.angle_PID = pid_config(1.0f, FLT_MAX);
    HTMotorInstance *motor = HTMotorInit(&config);
    TEST_CHECK(motor != NULL);
    HTMotorSetRef(motor, 100.0f / (float)RAD_2_DEGREE);
    motor->measure.total_angle = 0.0f;
    HTMotorControlInit();
    now_us = 0u;
    run_task(0u, 2u);
    TEST_CHECK(motor->pid_clock.last_status == PID_NUMERIC_ERROR);
    check_torque(motor, 0.0);

    speed_feedforward = FLT_MAX;
    HTMotorSetRef(motor, FLT_MAX);
    run_task(0u, 1u);
    TEST_CHECK(motor->pid_clock.last_status == PID_NUMERIC_ERROR);
    check_torque(motor, 0.0);
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strncmp(argv[1], "feedback_", 9u) == 0)
        test_feedback_combination((unsigned)(argv[1][9] - '0'));
    else if (strncmp(argv[1], "direction-angle-", 16u) == 0)
        test_direction_combination(0u, (unsigned)(argv[1][16] - '0'));
    else if (strncmp(argv[1], "direction-speed-", 16u) == 0)
        test_direction_combination(1u, (unsigned)(argv[1][16] - '0'));
    else if (strcmp(argv[1], "direction-current") == 0)
        test_current_loop_ignores_axis_direction_flags();
    else if (strcmp(argv[1], "direction-open") == 0)
        test_open_loop_keeps_motor_direction();
    else if (strcmp(argv[1], "direction-invalid") == 0)
        test_invalid_direction_flags_rejected();
    else if (strcmp(argv[1], "feedforward_overflow") == 0)
        test_feedforward_overflow();
    else if (strncmp(argv[1], "feedforward_", 12u) == 0)
        test_feedforward_copy_and_use((unsigned)(argv[1][12] - '0'));
    else if (strcmp(argv[1], "init_failures") == 0)
        test_init_rejects_missing_inputs();
    else if (strcmp(argv[1], "runtime_recovery") == 0)
        test_runtime_failure_and_recovery();
    else if (strcmp(argv[1], "runtime_speed_failure") == 0)
        test_runtime_speed_failure_and_recovery();
    else
        TEST_CHECK(0 && "unknown HT control scenario");
    return EXIT_SUCCESS;
}
