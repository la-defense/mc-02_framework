/**
 * @file lk_motor_bsp_can_test.c
 * @brief 通过生产 LK 和 BSP CAN 验证注册回退、真实 CAN 帧头及跨总线发送。
 * @note HAL、时间与日志由测试替身提供；每种场景单独启动，测试不连接设备。
 */
#include "LK9025.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "test_check.h"
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static FDCAN_GlobalTypeDef fdcan_registers[DEVICE_CAN_CNT];
FDCAN_HandleTypeDef hfdcan1 = {.Init = {.StdFiltersNbr = 14u}, .Instance = &fdcan_registers[0]};
FDCAN_HandleTypeDef hfdcan2 = {.Init = {.StdFiltersNbr = 14u}, .Instance = &fdcan_registers[1]};
FDCAN_HandleTypeDef hfdcan3 = {.Init = {.StdFiltersNbr = 14u}, .Instance = &fdcan_registers[2]};
volatile uint32_t dwt_cpu_freq_mhz = 480u;

static uint64_t now_us;
static uint32_t filter_fail_once;
static uint32_t tx_busy_mask;
static uint32_t tx_error_mask;
static uint32_t bus_start_count[DEVICE_CAN_CNT];
static uint32_t filter_count[DEVICE_CAN_CNT];
static uint32_t filter_rx_id[DEVICE_CAN_CNT];
static uint32_t tx_attempt_count[DEVICE_CAN_CNT];
static uint32_t tx_accept_count[DEVICE_CAN_CNT];
static uint32_t tx_header_id[DEVICE_CAN_CNT];
static uint8_t tx_frame[DEVICE_CAN_CNT][8];

#if defined(LK_TEST_ALLOC_WRAP)
static unsigned malloc_calls;
static unsigned free_calls;
void *__real_malloc(size_t size);
void __real_free(void *pointer);
void *__wrap_malloc(size_t size)
{
    ++malloc_calls;
    return __real_malloc(size);
}
void __wrap_free(void *pointer)
{
    if (pointer != NULL)
        ++free_calls;
    __real_free(pointer);
}
#endif

uint64_t DWT_GetTimeline_us(void) { return now_us; }
float DWT_GetDeltaT(uint32_t *counter)
{
    if (counter != NULL)
        ++*counter;
    return 0.001f;
}
uint32_t HAL_GetTick(void) { return (uint32_t)(now_us / 1000u); }

static uint8_t bus_for_handle(const FDCAN_HandleTypeDef *handle)
{
    if (handle == &hfdcan1) return 0u;
    if (handle == &hfdcan2) return 1u;
    if (handle == &hfdcan3) return 2u;
    return UINT8_MAX;
}

uint32_t HAL_FDCAN_ConfigRxFifoOverwrite(FDCAN_HandleTypeDef *handle, uint32_t fifo, uint32_t mode)
{
    (void)handle; (void)fifo; (void)mode;
    return HAL_OK;
}
uint32_t HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *handle, uint32_t nonmatching_std,
                                      uint32_t nonmatching_ext, uint32_t remote_std, uint32_t remote_ext)
{
    (void)handle; (void)nonmatching_std; (void)nonmatching_ext; (void)remote_std; (void)remote_ext;
    return HAL_OK;
}
uint32_t HAL_FDCAN_Start(FDCAN_HandleTypeDef *handle)
{
    uint8_t bus = bus_for_handle(handle);
    if (bus < DEVICE_CAN_CNT)
        ++bus_start_count[bus];
    handle->State = 2u;
    handle->Instance->CCCR &= ~FDCAN_CCCR_INIT;
    return HAL_OK;
}
uint32_t HAL_FDCAN_Stop(FDCAN_HandleTypeDef *handle)
{
    handle->State = 1u;
    handle->Instance->CCCR |= FDCAN_CCCR_INIT;
    return HAL_OK;
}
uint32_t HAL_FDCAN_Init(FDCAN_HandleTypeDef *handle)
{
    handle->Instance->CCCR |= FDCAN_CCCR_INIT;
    handle->State = 1u;
    return HAL_OK;
}
uint32_t HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *handle, uint32_t mask, uint32_t buffer_indexes)
{
    (void)buffer_indexes;
    handle->Instance->IE = mask;
    return HAL_OK;
}
uint32_t HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *handle, FDCAN_FilterTypeDef *filter)
{
    uint8_t bus = bus_for_handle(handle);
    if (filter_fail_once != 0u)
    {
        filter_fail_once = 0u;
        return HAL_ERROR;
    }
    if (bus >= DEVICE_CAN_CNT)
        return HAL_ERROR;
    filter_rx_id[bus] = filter->FilterID1;
    ++filter_count[bus];
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *handle)
{
    uint8_t bus = bus_for_handle(handle);
    return bus < DEVICE_CAN_CNT && (tx_busy_mask & (1u << bus)) == 0u ? 1u : 0u;
}
uint32_t HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *handle,
                                       FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    uint8_t bus = bus_for_handle(handle);
    if (bus >= DEVICE_CAN_CNT)
        return HAL_ERROR;
    ++tx_attempt_count[bus];
    if ((tx_error_mask & (1u << bus)) != 0u)
        return HAL_ERROR;
    ++tx_accept_count[bus];
    tx_header_id[bus] = header->Identifier;
    memcpy(tx_frame[bus], data, sizeof(tx_frame[bus]));
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *handle, FDCAN_ProtocolStatusTypeDef *status)
{
    (void)handle;
    memset(status, 0, sizeof(*status));
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *handle, FDCAN_ErrorCountersTypeDef *counters)
{
    (void)handle;
    memset(counters, 0, sizeof(*counters));
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *handle, uint32_t fifo)
{
    (void)handle; (void)fifo;
    return 0u;
}
uint32_t HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *handle, uint32_t fifo,
                                FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)handle; (void)fifo; (void)header; (void)data;
    return HAL_ERROR;
}

static Motor_Init_Config_s config_for(uint8_t bus, uint32_t motor_id)
{
    Motor_Init_Config_s config = {0};
    config.motor_type = LK9025;
    config.can_init_config.can_handle = bus == 0u ? &hfdcan1 : bus == 1u ? &hfdcan2 : &hfdcan3;
    config.can_init_config.tx_id = motor_id;
    config.controller_setting_init_config.close_loop_type = OPEN_LOOP;
    config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    return config;
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

static void test_production_registration_rollback(void)
{
    Motor_Init_Config_s config = config_for(2u, 4u);
#if defined(LK_TEST_ALLOC_WRAP)
    unsigned alloc_before = malloc_calls;
    unsigned free_before = free_calls;
#endif
    filter_fail_once = 1u;
    TEST_CHECK(LKMotorInit(&config) == NULL);
#if defined(LK_TEST_ALLOC_WRAP)
    TEST_CHECK(malloc_calls == alloc_before + 3u);
    TEST_CHECK(free_calls == free_before + 3u);
#endif
    TEST_CHECK(filter_count[2] == 0u);

    LKMotorInstance *motor = LKMotorInit(&config);
    TEST_CHECK(motor != NULL);
    TEST_CHECK(motor->motor_id == 4u && motor->can_bus_index == 2u);
    TEST_CHECK(motor->motor_can_ins->rx_id == 0x144u);
    TEST_CHECK(motor->motor_can_ins->tx_id == 0x280u);
    TEST_CHECK(motor->motor_can_ins->txconf.Identifier == 0x280u);
    TEST_CHECK(filter_count[2] == 1u && filter_rx_id[2] == 0x144u);
}

static void test_production_can_routing(void)
{
    Motor_Init_Config_s config_a = config_for(2u, 4u);
    Motor_Init_Config_s config_b = config_for(0u, 2u);
    LKMotorInstance *motor_a = LKMotorInit(&config_a);
    LKMotorInstance *motor_b = LKMotorInit(&config_b);
    TEST_CHECK(motor_a != NULL && motor_b != NULL);
    TEST_CHECK(filter_count[2] == 1u && filter_rx_id[2] == 0x144u);
    TEST_CHECK(filter_count[0] == 1u && filter_rx_id[0] == 0x142u);
    TEST_CHECK(bus_start_count[0] == 1u && bus_start_count[1] == 1u && bus_start_count[2] == 1u);

    LKMotorSetRef(motor_a, 1234.0f);
    LKMotorSetRef(motor_b, -321.0f);
    now_us = 0u;
    LKMotorControl();
    TEST_CHECK(tx_accept_count[0] == 1u && tx_accept_count[2] == 1u);
    TEST_CHECK(tx_header_id[0] == 0x280u && tx_header_id[2] == 0x280u);
    TEST_CHECK(memcmp(tx_frame[0], (uint8_t[8]){0}, 8u) == 0);
    TEST_CHECK(memcmp(tx_frame[2], (uint8_t[8]){0}, 8u) == 0);

    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(tx_accept_count[0] == 2u && tx_accept_count[2] == 2u);
    TEST_CHECK(tx_frame[0][2] == 0xBFu && tx_frame[0][3] == 0xFEu);
    TEST_CHECK(tx_frame[2][6] == 0xD2u && tx_frame[2][7] == 0x04u);

    // FIFO 忙时该总线只累计丢弃，其他总线仍成功；恢复后发送当周期的新值。
    tx_busy_mask = 1u << 2;
    LKMotorSetRef(motor_a, 777.0f);
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_accept_count[0] == 3u && tx_accept_count[2] == 2u);
    CAN_TxStats_t stats = {0};
    CANGetTxStats(&hfdcan3, &stats);
    TEST_CHECK(stats.tx_drop == 1u && stats.tx_ok == 2u);

    tx_busy_mask = 0u;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_accept_count[2] == 3u && tx_frame[2][6] == 0x09u && tx_frame[2][7] == 0x03u);

    // HAL 拒绝一条总线的提交不影响另一条；下轮仍使用最新计算值。
    tx_error_mask = 1u;
    LKMotorSetRef(motor_b, 456.0f);
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_accept_count[0] == 4u && tx_accept_count[2] == 4u);
    CANGetTxStats(&hfdcan1, &stats);
    TEST_CHECK(stats.tx_error == 1u && stats.tx_ok == 4u);

    tx_error_mask = 0u;
    LKMotorSetRef(motor_b, -654.0f);
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_accept_count[0] == 5u && tx_frame[0][2] == 0x72u && tx_frame[0][3] == 0xFDu);
}

static void test_closed_loop_feedback_selection(void)
{
    float external_angle = 20.0f;
    float external_speed = 4.0f;
    LKMotorInstance *motors[LK_MOTOR_PER_BUS_CNT] = {NULL};
    static const int16_t expected_current[LK_MOTOR_PER_BUS_CNT] = {14, -46, 8, -52};
    static const uint8_t expected_frame[8] = {0x0Eu, 0u, 0xD2u, 0xFFu, 0x08u, 0u, 0xCCu, 0xFFu};

    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
    {
        Motor_Init_Config_s config = config_for(0u, (uint32_t)index + 1u);
        config.controller_setting_init_config.close_loop_type = ALL_THREE_LOOP;
        config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
        config.controller_setting_init_config.angle_feedback_source =
            (index & 1u) != 0u ? OTHER_FEED : MOTOR_FEED;
        config.controller_setting_init_config.speed_feedback_source =
            (index & 2u) != 0u ? OTHER_FEED : MOTOR_FEED;
        config.controller_param_init_config.other_angle_feedback_ptr = &external_angle;
        config.controller_param_init_config.other_speed_feedback_ptr = &external_speed;
        config.controller_param_init_config.angle_PID = pid_config(2.0f, 1000.0f);
        config.controller_param_init_config.speed_PID = pid_config(3.0f, 1000.0f);
        config.controller_param_init_config.current_PID = pid_config(1.0f, 1000.0f);

        motors[index] = LKMotorInit(&config);
        TEST_CHECK(motors[index] != NULL);
        motors[index]->measure.total_angle = 10.0f;
        motors[index]->measure.speed_rads = 2.0f;
        motors[index]->measure.real_current = 100;
        LKMotorSetRef(motors[index], 30.0f);
    }

    now_us = 0u;
    LKMotorControl();
    TEST_CHECK(memcmp(tx_frame[0], (uint8_t[8]){0}, 8u) == 0);

    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(tx_header_id[0] == 0x280u);
    TEST_CHECK(memcmp(tx_frame[0], expected_frame, sizeof(expected_frame)) == 0);
    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
        TEST_CHECK((int16_t)((uint16_t)tx_frame[0][index * 2u] |
                            ((uint16_t)tx_frame[0][index * 2u + 1u] << 8u)) == expected_current[index]);
}

static void test_feedforward_paths_and_recovery(void)
{
    float external_angle = 20.0f;
    float external_speed = 4.0f;
    float speed_feedforward = 5.0f;
    float current_feedforward = 7.0f;
    static const int16_t expected_current[LK_MOTOR_PER_BUS_CNT] = {14, -31, 15, -30};
    static const uint8_t expected_frame[8] = {14u, 0u, 0xE1u, 0xFFu, 15u, 0u, 0xE2u, 0xFFu};

    Motor_Init_Config_s invalid = config_for(0u, 1u);
    invalid.controller_setting_init_config.close_loop_type = ALL_THREE_LOOP;
    invalid.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    invalid.controller_setting_init_config.feedforward_flag = SPEED_FEEDFORWARD;
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    invalid.controller_setting_init_config.feedforward_flag = CURRENT_FEEDFORWARD;
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    invalid.controller_setting_init_config.feedforward_flag = FEEDFORWARD_NONE;
    invalid.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    invalid.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    invalid.controller_setting_init_config.speed_feedback_source = (Feedback_Source_e)2;
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    TEST_CHECK(filter_count[0] == 0u && bus_start_count[0] == 0u);

    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
    {
        Motor_Init_Config_s config = config_for(0u, (uint32_t)index + 1u);
        config.controller_setting_init_config.close_loop_type = ALL_THREE_LOOP;
        config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
        config.controller_setting_init_config.angle_feedback_source =
            (index == 1u || index == 3u) ? OTHER_FEED : MOTOR_FEED;
        config.controller_setting_init_config.speed_feedback_source =
            (index == 2u || index == 3u) ? OTHER_FEED : MOTOR_FEED;
        config.controller_setting_init_config.feedforward_flag =
            index == 1u ? SPEED_FEEDFORWARD : index == 2u ? CURRENT_FEEDFORWARD :
            index == 3u ? CURRENT_AND_SPEED_FEEDFORWARD : FEEDFORWARD_NONE;
        config.controller_param_init_config.other_angle_feedback_ptr =
            (index == 1u || index == 3u) ? &external_angle : NULL;
        config.controller_param_init_config.other_speed_feedback_ptr =
            (index == 2u || index == 3u) ? &external_speed : NULL;
        config.controller_param_init_config.speed_feedforward_ptr =
            (index == 1u || index == 3u) ? &speed_feedforward : NULL;
        config.controller_param_init_config.current_feedforward_ptr =
            (index == 2u || index == 3u) ? &current_feedforward : NULL;
        config.controller_param_init_config.angle_PID = pid_config(2.0f, 1000.0f);
        config.controller_param_init_config.speed_PID = pid_config(3.0f, 1000.0f);
        config.controller_param_init_config.current_PID = pid_config(1.0f, 1000.0f);

        LKMotorInstance *motor = LKMotorInit(&config);
        TEST_CHECK(motor != NULL);
        TEST_CHECK(motor->speed_feedforward_ptr == config.controller_param_init_config.speed_feedforward_ptr);
        TEST_CHECK(motor->current_feedforward_ptr == config.controller_param_init_config.current_feedforward_ptr);
        motor->measure.total_angle = 10.0f;
        motor->measure.speed_rads = 2.0f;
        motor->measure.real_current = 100;
        LKMotorSetRef(motor, 30.0f);
    }

    now_us = 0u;
    LKMotorControl();
    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(memcmp(tx_frame[0], expected_frame, sizeof(expected_frame)) == 0);
    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
        TEST_CHECK((int16_t)((uint16_t)tx_frame[0][index * 2u] |
                            ((uint16_t)tx_frame[0][index * 2u + 1u] << 8u)) == expected_current[index]);

    // 非有限前馈只清本实例槽位；恢复有限输入后新一轮可重新计算。
    speed_feedforward = NAN;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][2] == 0u && tx_frame[0][3] == 0u);
    TEST_CHECK(tx_frame[0][0] == 14u && tx_frame[0][4] == 15u);
    TEST_CHECK(tx_frame[0][6] == 0u && tx_frame[0][7] == 0u);
    speed_feedforward = 5.0f;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][2] == 0xE1u && tx_frame[0][3] == 0xFFu);
    TEST_CHECK(tx_frame[0][6] == 0xE2u && tx_frame[0][7] == 0xFFu);
}

static void test_feedback_failure_recovery(void)
{
    float external_angle = 20.0f;
    Motor_Init_Config_s invalid = config_for(0u, 1u);
    invalid.controller_setting_init_config.close_loop_type = ANGLE_LOOP;
    invalid.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    invalid.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    invalid.controller_setting_init_config.angle_feedback_source = (Feedback_Source_e)2;
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    invalid.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    invalid.controller_setting_init_config.close_loop_type = ALL_THREE_LOOP;
    invalid.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
    invalid.controller_param_init_config.other_angle_feedback_ptr = &external_angle;
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    TEST_CHECK(filter_count[0] == 0u && bus_start_count[0] == 0u);

    Motor_Init_Config_s config = config_for(0u, 1u);
    config.controller_setting_init_config.close_loop_type = ANGLE_LOOP;
    config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
    config.controller_param_init_config.other_angle_feedback_ptr = &external_angle;
    config.controller_param_init_config.angle_PID = pid_config(2.0f, 1000.0f);
    LKMotorInstance *motor = LKMotorInit(&config);
    TEST_CHECK(motor != NULL);
    TEST_CHECK(motor->other_speed_feedback_ptr == NULL);
    motor->measure.total_angle = 10.0f;
    LKMotorSetRef(motor, 30.0f);

    now_us = 0u;
    LKMotorControl();
    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 20u && tx_frame[0][1] == 0u);

    external_angle = NAN;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(motor->pid_clock.last_status == PID_BAD_ARGUMENT);
    TEST_CHECK(tx_frame[0][0] == 0u && tx_frame[0][1] == 0u);
    PID_State_t state = {0};
    TEST_CHECK(PIDGetState(&motor->angle_PID, &state) == PID_OK);
    TEST_CHECK(state.Err == 0.0f && state.Last_Err == 0.0f && state.ITerm == 0.0f &&
               state.Iout == 0.0f && state.Pout == 0.0f && state.Dout == 0.0f &&
               state.Output == 0.0f);

    external_angle = 10.0f;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 40u && tx_frame[0][1] == 0u);

    motor->motor_settings.angle_feedback_source = (Feedback_Source_e)2;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(motor->pid_clock.last_status == PID_BAD_ARGUMENT);
    TEST_CHECK(tx_frame[0][0] == 0u && tx_frame[0][1] == 0u);
    motor->motor_settings.angle_feedback_source = MOTOR_FEED;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 40u && tx_frame[0][1] == 0u);

    motor->motor_settings.close_loop_type = ANGLE_LOOP | SPEED_LOOP;
    motor->motor_settings.outer_loop_type = SPEED_LOOP;
    motor->motor_settings.speed_feedback_source = MOTOR_FEED;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(motor->pid_clock.last_status == PID_NOT_INITIALIZED);
    TEST_CHECK(tx_frame[0][0] == 0u && tx_frame[0][1] == 0u);
    motor->motor_settings.close_loop_type = ANGLE_LOOP;
    motor->motor_settings.outer_loop_type = ANGLE_LOOP;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 40u && tx_frame[0][1] == 0u);
}

static void test_feedforward_overflow(void)
{
    float feedforward = FLT_MAX;
    Motor_Init_Config_s config = config_for(0u, 1u);
    config.controller_setting_init_config.close_loop_type = ANGLE_LOOP;
    config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    config.controller_setting_init_config.feedforward_flag = SPEED_FEEDFORWARD;
    config.controller_param_init_config.speed_feedforward_ptr = &feedforward;
    config.controller_param_init_config.angle_PID = pid_config(1.0f, FLT_MAX);
    LKMotorInstance *motor = LKMotorInit(&config);
    TEST_CHECK(motor != NULL);
    motor->measure.total_angle = 0.0f;
    LKMotorSetRef(motor, FLT_MAX);
    now_us = 0u;
    LKMotorControl();
    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(motor->pid_clock.last_status == PID_NUMERIC_ERROR);
    TEST_CHECK(tx_frame[0][0] == 0u && tx_frame[0][1] == 0u);
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "rollback") == 0)
        test_production_registration_rollback();
    else if (strcmp(argv[1], "routing") == 0)
        test_production_can_routing();
    else if (strcmp(argv[1], "feedforward") == 0)
        test_feedforward_paths_and_recovery();
    else if (strcmp(argv[1], "feedback_failure") == 0)
        test_feedback_failure_recovery();
    else
    {
        if (strcmp(argv[1], "closed_loops") == 0)
            test_closed_loop_feedback_selection();
        else
        {
            TEST_CHECK(strcmp(argv[1], "feedforward_overflow") == 0);
            test_feedforward_overflow();
        }
    }
    return EXIT_SUCCESS;
}
