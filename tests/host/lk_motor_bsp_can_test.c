/**
 * @file lk_motor_bsp_can_test.c
 * @brief 通过生产 LK 和 BSP CAN 验证注册回退、真实 CAN 帧头及跨总线发送。
 * @note HAL、时间与日志由测试替身提供；每种场景单独启动，测试不连接设备。
 */
#include "LK9025.h"
#include "dji_motor.h"
#include "bsp_can.h"
#include "../../bsp/can/platform_can.h"
#include "can_topic_transport.h"
#include "can_topic_link.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "test_check.h"
#include "motor_feedback_test_support.h"
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* 测试按生产任务的顺序计算目标，再由唯一调度入口推进一次发送。 */
#define LKMotorControl() (LKMotorControl(), CANProcessTx())

static FDCAN_GlobalTypeDef fdcan_registers[DEVICE_CAN_CNT];
FDCAN_HandleTypeDef hfdcan1 = {.Init = {.StdFiltersNbr = 14u}, .Instance = &fdcan_registers[0]};
FDCAN_HandleTypeDef hfdcan2 = {.Init = {.StdFiltersNbr = 14u}, .Instance = &fdcan_registers[1]};
FDCAN_HandleTypeDef hfdcan3 = {.Init = {.StdFiltersNbr = 14u}, .Instance = &fdcan_registers[2]};
volatile uint32_t dwt_cpu_freq_mhz = 480u;
volatile uint32_t test_dwt_cycles;
volatile uint32_t test_ipsr;
extern volatile DWT_Probe_t can_prof_rx_dispatch;

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
static uint32_t tx_latest_buffer[DEVICE_CAN_CNT];
static uint8_t tx_next_buffer_index[DEVICE_CAN_CNT];
static uint8_t tx_frame[DEVICE_CAN_CNT][8];
static uint32_t tx_id_history[DEVICE_CAN_CNT][64];
static uint8_t tx_id_type_history[DEVICE_CAN_CNT][64];
static uint8_t tx_length_history[DEVICE_CAN_CNT][64];
static uint8_t tx_data_history[DEVICE_CAN_CNT][64][8];
typedef struct
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
} TestRxFrame_t;
static TestRxFrame_t rx_frames[DEVICE_CAN_CNT][64];
static uint8_t rx_frame_count[DEVICE_CAN_CNT];
static uint8_t rx_frame_read[DEVICE_CAN_CNT];
static uint32_t burst_rx_count[DEVICE_CAN_CNT];
static uint32_t burst_rx_bad[DEVICE_CAN_CNT];
static uint32_t burst_last_cycle[DEVICE_CAN_CNT];

#if defined(LK_TEST_ALLOC_WRAP)
static unsigned malloc_calls;
static unsigned free_calls;
void *__real_malloc(size_t size);
void *__real_calloc(size_t count, size_t size);
void __real_free(void *pointer);
void *__wrap_malloc(size_t size)
{
    ++malloc_calls;
    return __real_malloc(size);
}
void *__wrap_calloc(size_t count, size_t size)
{
    ++malloc_calls;
    return __real_calloc(count, size);
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

static uint8_t test_dlc_to_length(uint32_t encoded_dlc)
{
    for (uint8_t length = 0u; length <= 8u; ++length)
        if (DLC_LookUp_Table[length] == encoded_dlc)
            return length;
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
    const uint32_t history_index = tx_accept_count[bus] - 1u;
    if (history_index < 64u)
    {
        tx_id_history[bus][history_index] = header->Identifier;
        tx_id_type_history[bus][history_index] = header->IdType;
        tx_length_history[bus][history_index] = test_dlc_to_length(header->DataLength);
        memcpy(tx_data_history[bus][history_index], data, 8u);
    }
    tx_latest_buffer[bus] = 1u << tx_next_buffer_index[bus];
    tx_next_buffer_index[bus] = (uint8_t)((tx_next_buffer_index[bus] + 1u) % 16u);
    tx_header_id[bus] = header->Identifier;
    memcpy(tx_frame[bus], data, sizeof(tx_frame[bus]));
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(FDCAN_HandleTypeDef *handle)
{
    const uint8_t bus = bus_for_handle(handle);
    return bus < DEVICE_CAN_CNT ? tx_latest_buffer[bus] : 0u;
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
    (void)fifo;
    uint8_t bus = bus_for_handle(handle);
    return bus < DEVICE_CAN_CNT ? (uint32_t)(rx_frame_count[bus] - rx_frame_read[bus]) : 0u;
}
uint32_t HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *handle, uint32_t fifo,
                                FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)fifo;
    uint8_t bus = bus_for_handle(handle);
    if (bus >= DEVICE_CAN_CNT || rx_frame_read[bus] >= rx_frame_count[bus])
        return HAL_ERROR;
    TestRxFrame_t *frame = &rx_frames[bus][rx_frame_read[bus]++];
    test_dwt_cycles += 480000u;
    *header = frame->header;
    memcpy(data, frame->data, sizeof(frame->data));
    return HAL_OK;
}

static void burst_rx_callback(CANInstance *instance)
{
    uint8_t bus = bus_for_handle(instance->can_handle);
    if (bus >= DEVICE_CAN_CNT || instance->rx_len != 8u || instance->rx_buff[0] != burst_rx_count[bus] ||
        instance->rx_timestamp_cycles == 0u ||
        (burst_rx_count[bus] != 0u &&
         (uint32_t)(instance->rx_timestamp_cycles - burst_last_cycle[bus]) != 480000u))
    {
        if (bus < DEVICE_CAN_CNT)
            ++burst_rx_bad[bus];
        return;
    }
    burst_last_cycle[bus] = instance->rx_timestamp_cycles;
    ++burst_rx_count[bus];
}

static void test_production_can_receive_burst(void)
{
    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        CAN_Init_Config_s config = {0};
        config.can_handle = bus == 0u ? &hfdcan1 : bus == 1u ? &hfdcan2 : &hfdcan3;
        config.tx_id = 0x300u + bus;
        config.rx_id = 0x320u + bus;
        config.can_module_callback = burst_rx_callback;
        TEST_CHECK(CANRegister(&config) != NULL);

        rx_frame_count[bus] = 32u;
        rx_frame_read[bus] = 0u;
        for (uint8_t frame = 0u; frame < rx_frame_count[bus]; ++frame)
        {
            TestRxFrame_t *item = &rx_frames[bus][frame];
            memset(item, 0, sizeof(*item));
            item->header.Identifier = config.rx_id;
            item->header.DataLength = 8u << 16u;
            item->header.RxFrameType = FDCAN_DATA_FRAME;
            item->header.IdType = FDCAN_STANDARD_ID;
            item->data[0] = frame;
        }
        HAL_FDCAN_RxFifo0Callback(config.can_handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        TEST_CHECK(rx_frame_read[bus] == 32u);
        TEST_CHECK(CANGetRxQueueDrop(config.can_handle) == 1u);
    }

    now_us = 0u;
    CANProcessRx();
    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        TEST_CHECK(burst_rx_count[bus] == 31u);
        TEST_CHECK(burst_rx_bad[bus] == 0u);
    }
    TEST_CHECK(can_prof_rx_dispatch.calls == 1u);
    TEST_CHECK(can_prof_rx_dispatch.max_us >= can_prof_rx_dispatch.last_us);
}

/* 通过生产 CAN 派发器调用真实 DJI 解码回调，连续压入三条总线的满队列。 */
static void test_production_dji_receive_burst(void)
{
    DJIMotorInstance *motors[DEVICE_CAN_CNT] = {0};
    FDCAN_HandleTypeDef *handles[DEVICE_CAN_CNT] = {&hfdcan1, &hfdcan2, &hfdcan3};
    uint32_t expected_accepted[DEVICE_CAN_CNT] = {0u};

    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        Motor_Init_Config_s config = {0};
        config.motor_type = M3508;
        config.output_axis = (Motor_Output_Axis_Config_s){1.0f, MOTOR_AXIS_DIRECTION_NORMAL};
        config.can_init_config.can_bus = (PlatformCanBus_t)(PLATFORM_CAN_BUS_1 + bus);
        config.can_init_config.tx_id = 1u;
        config.controller_setting_init_config.close_loop_type = OPEN_LOOP;
        motors[bus] = DJIMotorInit(&config);
        TEST_CHECK(motors[bus] != NULL);
        TEST_CHECK(motors[bus]->motor_can_instance->rx_id == 0x201u);
    }

    for (uint8_t batch = 0u; batch < 24u; ++batch)
    {
        for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
        {
            CANInstance *can = motors[bus]->motor_can_instance;
            rx_frame_count[bus] = 32u;
            rx_frame_read[bus] = 0u;
            for (uint8_t frame = 0u; frame < rx_frame_count[bus]; ++frame)
            {
                TestRxFrame_t *item = &rx_frames[bus][frame];
                uint16_t ecd = (uint16_t)((7000u + batch * 73u + frame) % 8192u);
                int16_t rpm = (int16_t)((int16_t)(batch * 31u) - (int16_t)(frame * 3u));
                int16_t current = (int16_t)((int16_t)(frame * 17u) - (int16_t)(batch * 11u));
                memset(item, 0, sizeof(*item));
                item->header.Identifier = can->rx_id;
                item->header.DataLength = 8u << 16u;
                item->header.RxFrameType = FDCAN_DATA_FRAME;
                item->header.IdType = FDCAN_STANDARD_ID;
                item->data[0] = (uint8_t)(ecd >> 8u);
                item->data[1] = (uint8_t)ecd;
                item->data[2] = (uint8_t)((uint16_t)rpm >> 8u);
                item->data[3] = (uint8_t)rpm;
                item->data[4] = (uint8_t)((uint16_t)current >> 8u);
                item->data[5] = (uint8_t)current;
                item->data[6] = (uint8_t)(batch + frame);
            }
            HAL_FDCAN_RxFifo0Callback(handles[bus], FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
            TEST_CHECK(rx_frame_read[bus] == 32u);
            expected_accepted[bus] += 31u;
        }

        now_us = (uint64_t)batch * 1000u;
        CANProcessRx();
    }

    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        DJIMotorInstance *motor = motors[bus];
        uint16_t expected_ecd = (uint16_t)((7000u + 23u * 73u + 30u) % 8192u);
        TEST_CHECK(motor->feedback_sample_count == expected_accepted[bus]);
        TEST_CHECK(motor->measure.ecd == expected_ecd);
        TEST_CHECK(motor->measure.temperature == (uint8_t)(23u + 30u));
        TEST_CHECK(isfinite(motor->measure.angle_single_round));
        TEST_CHECK(isfinite(motor->measure.speed_aps));
        TEST_CHECK(isfinite(motor->measure.total_angle));
        TEST_CHECK(CANGetRxQueueDrop(handles[bus]) == 24u);
    }
    TEST_CHECK(can_prof_rx_dispatch.calls == 24u);
}

static Motor_Init_Config_s config_for(uint8_t bus, uint32_t motor_id)
{
    Motor_Init_Config_s config = {0};
    config.motor_type = LK9025;
    config.output_axis = (Motor_Output_Axis_Config_s){1.0f, MOTOR_AXIS_DIRECTION_NORMAL};
    config.can_init_config.can_bus = (PlatformCanBus_t)(PLATFORM_CAN_BUS_1 + bus);
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

    CAN_ObservationCounters_t observation = {0};
    TEST_CHECK(CANGetObservationCounters(2u, &observation) == 1u);
    TEST_CHECK(observation.active == 1u && observation.tx_ok == 5u && observation.tx_drop == 1u);
    TEST_CHECK(CANGetObservationCounters(0u, &observation) == 1u);
    TEST_CHECK(observation.tx_error == 1u && observation.tx_ok == 5u);
    CAN_ObservationCounters_t unchanged = {.tx_ok = 0xA5A5A5A5u};
    TEST_CHECK(CANGetObservationCounters(DEVICE_CAN_CNT, &unchanged) == 0u);
    TEST_CHECK(unchanged.tx_ok == 0xA5A5A5A5u);
}

static void test_closed_loop_feedback_selection(void)
{
    float external_angle = 0.5f;
    float external_speed = 0.5f;
    LKMotorInstance *motors[LK_MOTOR_PER_BUS_CNT] = {NULL};
    static const int16_t expected_current[LK_MOTOR_PER_BUS_CNT] = {5, 2, 4, 1};
    static const uint8_t expected_frame[8] = {5u, 0u, 2u, 0u, 4u, 0u, 1u, 0u};

    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
    {
        Motor_Init_Config_s config = config_for(0u, (uint32_t)index + 1u);
        config.controller_setting_init_config.close_loop_type = ALL_THREE_LOOP;
        config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
        config.controller_setting_init_config.angle_feedback_source =
            (index & 1u) != 0u ? OTHER_FEED : MOTOR_FEED;
        config.controller_setting_init_config.speed_feedback_source =
            (index & 2u) != 0u ? OTHER_FEED : MOTOR_FEED;
        config.controller_param_init_config.other_angle_feedback = MotorFeedbackTestMakeSource(
            &external_angle, MotorFeedbackTestReadRadExternal, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL);
        config.controller_param_init_config.other_speed_feedback = MotorFeedbackTestMakeSource(
            &external_speed, MotorFeedbackTestReadRadSExternal, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_EXTERNAL);
        config.controller_param_init_config.angle_PID = pid_config(2.0f, 1000.0f);
        config.controller_param_init_config.speed_PID = pid_config(3.0f, 1000.0f);
        config.controller_param_init_config.current_PID = pid_config(1.0f, 1000.0f);

        motors[index] = LKMotorInit(&config);
        TEST_CHECK(motors[index] != NULL);
        motors[index]->measure.total_angle = 0.0f;
        motors[index]->measure.speed_rads = 0.25f;
        motors[index]->measure.real_current = 0;
        LKMotorSetRef(motors[index], 1.0f);
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

static void test_direction_flags(uint8_t speed_outer)
{
    LKMotorInstance *motors[LK_MOTOR_PER_BUS_CNT] = {NULL};

    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
    {
        Motor_Init_Config_s config = config_for(0u, (uint32_t)index + 1u);
        Motor_Control_Setting_s *setting = &config.controller_setting_init_config;
        setting->outer_loop_type = speed_outer ? SPEED_LOOP : ANGLE_LOOP;
        setting->close_loop_type = speed_outer ? SPEED_LOOP : (ANGLE_LOOP | SPEED_LOOP);
        config.output_axis.direction = (index & 1u) != 0u ? MOTOR_AXIS_DIRECTION_REVERSE :
                                                            MOTOR_AXIS_DIRECTION_NORMAL;
        config.controller_param_init_config.angle_PID = pid_config(4.0f, 1000.0f);
        config.controller_param_init_config.speed_PID = pid_config(4.0f, 1000.0f);

        motors[index] = LKMotorInit(&config);
        TEST_CHECK(motors[index] != NULL);
        motors[index]->measure.total_angle = 0.0f;
        motors[index]->measure.speed_rads = 0.25f;
        LKMotorSetRef(motors[index], 1.0f);
    }

    now_us = 0u;
    LKMotorControl();
    now_us = 1000u;
    LKMotorControl();
    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
    {
        int16_t actual = (int16_t)((uint16_t)tx_frame[0][index * 2u] |
                                   ((uint16_t)tx_frame[0][index * 2u + 1u] << 8u));
        const int16_t expected_positive = speed_outer ?
            ((index & 1u) != 0u ? -5 : 3) : ((index & 1u) != 0u ? -17 : 15);
        TEST_CHECK(actual == expected_positive);
        LKMotorSetRef(motors[index], -1.0f);
    }

    now_us = 2000u;
    LKMotorControl();
    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
    {
        int16_t actual = (int16_t)((uint16_t)tx_frame[0][index * 2u] |
                                   ((uint16_t)tx_frame[0][index * 2u + 1u] << 8u));
        const int16_t expected_negative = speed_outer ?
            ((index & 1u) != 0u ? 3 : -5) : ((index & 1u) != 0u ? 15 : -17);
        TEST_CHECK(actual == expected_negative);
        LKMotorSetRef(motors[index], 0.0f);
        motors[index]->measure.total_angle = 0.0f;
        motors[index]->measure.speed_rads = 0.0f;
    }

    now_us = 3000u;
    LKMotorControl();
    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
        TEST_CHECK(tx_frame[0][index * 2u] == 0u && tx_frame[0][index * 2u + 1u] == 0u);
}

static void test_current_loop_ignores_axis_direction_flags(void)
{
    Motor_Init_Config_s config = config_for(0u, 1u);
    config.controller_setting_init_config.outer_loop_type = CURRENT_LOOP;
    config.controller_setting_init_config.close_loop_type = CURRENT_LOOP;
    config.output_axis.direction = MOTOR_AXIS_DIRECTION_REVERSE;
    config.controller_param_init_config.current_PID = pid_config(0.25f, 1000.0f);
    LKMotorInstance *motor = LKMotorInit(&config);
    TEST_CHECK(motor != NULL);
    LKMotorSetRef(motor, 30.0f);

    now_us = 0u;
    LKMotorControl();
    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 7u && tx_frame[0][1] == 0u);
}

static void test_open_loop_keeps_motor_direction(void)
{
    Motor_Init_Config_s config = config_for(0u, 1u);
    config.controller_setting_init_config.outer_loop_type = OPEN_LOOP;
    config.controller_setting_init_config.close_loop_type = OPEN_LOOP;
    config.output_axis.direction = MOTOR_AXIS_DIRECTION_REVERSE;
    LKMotorInstance *motor = LKMotorInit(&config);
    TEST_CHECK(motor != NULL);
    LKMotorSetRef(motor, 9.0f);

    now_us = 0u;
    LKMotorControl();
    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 9u && tx_frame[0][1] == 0u);

    LKMotorSetRef(motor, -9.0f);
    now_us = 2000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 0xF7u && tx_frame[0][1] == 0xFFu);

    LKMotorSetRef(motor, 0.0f);
    now_us = 3000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 0u && tx_frame[0][1] == 0u);
}

static void test_invalid_direction_flags_rejected(void)
{
    Motor_Init_Config_s config = config_for(0u, 1u);
    config.controller_setting_init_config.motor_reverse_flag = (Motor_Reverse_Flag_e)2;
    TEST_CHECK(LKMotorInit(&config) == NULL);
    config = config_for(0u, 1u);
    config.controller_setting_init_config.feedback_reverse_flag = (Feedback_Reverse_Flag_e)2;
    TEST_CHECK(LKMotorInit(&config) == NULL);
    config = config_for(0u, 1u);
    config.output_axis.direction = (Motor_Axis_Direction_e)0;
    TEST_CHECK(LKMotorInit(&config) == NULL);
    TEST_CHECK(filter_count[0] == 0u && bus_start_count[0] == 0u);
}

static void test_feedforward_paths_and_recovery(void)
{
    float external_angle = 0.5f;
    float external_speed = 0.5f;
    float speed_feedforward = 0.25f;
    float current_feedforward = 0.5f;
    static const int16_t expected_current[LK_MOTOR_PER_BUS_CNT] = {5, 3, 5, 2};
    static const uint8_t expected_frame[8] = {5u, 0u, 3u, 0u, 5u, 0u, 2u, 0u};

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
        config.controller_param_init_config.other_angle_feedback =
            (index == 1u || index == 3u) ? MotorFeedbackTestMakeSource(&external_angle,
                MotorFeedbackTestReadRadExternal, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL) :
                (Motor_Signal_Source_s){0};
        config.controller_param_init_config.other_speed_feedback =
            (index == 2u || index == 3u) ? MotorFeedbackTestMakeSource(&external_speed,
                MotorFeedbackTestReadRadSExternal, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_EXTERNAL) :
                (Motor_Signal_Source_s){0};
        config.controller_param_init_config.speed_feedforward =
            (index == 1u || index == 3u) ? MotorFeedbackTestMakeSource(&speed_feedforward,
                MotorFeedbackTestReadRadSOutput, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_OUTPUT_AXIS) :
                (Motor_Signal_Source_s){0};
        config.controller_param_init_config.current_feedforward =
            (index == 2u || index == 3u) ? MotorFeedbackTestMakeSource(&current_feedforward,
                MotorFeedbackTestReadDeviceCurrentOutput, MOTOR_UNIT_DEVICE_CURRENT,
                MOTOR_REFERENCE_OUTPUT_AXIS) : (Motor_Signal_Source_s){0};
        config.controller_param_init_config.angle_PID = pid_config(2.0f, 1000.0f);
        config.controller_param_init_config.speed_PID = pid_config(3.0f, 1000.0f);
        config.controller_param_init_config.current_PID = pid_config(1.0f, 1000.0f);

        LKMotorInstance *motor = LKMotorInit(&config);
        TEST_CHECK(motor != NULL);
        TEST_CHECK(motor->speed_feedforward.context == config.controller_param_init_config.speed_feedforward.context);
        TEST_CHECK(motor->current_feedforward.context == config.controller_param_init_config.current_feedforward.context);
        motor->measure.total_angle = 0.0f;
        motor->measure.speed_rads = 0.25f;
        motor->measure.real_current = 0;
        LKMotorSetRef(motor, 1.0f);
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
    TEST_CHECK(tx_frame[0][0] == 5u && tx_frame[0][4] == 5u);
    TEST_CHECK(tx_frame[0][6] == 0u && tx_frame[0][7] == 0u);
    speed_feedforward = 0.25f;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][2] == 3u && tx_frame[0][3] == 0u);
    TEST_CHECK(tx_frame[0][6] == 2u && tx_frame[0][7] == 0u);
}

static void test_feedback_failure_recovery(void)
{
    float external_angle = 0.5f;
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
    invalid.controller_param_init_config.other_angle_feedback = MotorFeedbackTestMakeSource(
        &external_angle, MotorFeedbackTestReadRadExternal, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL);
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    TEST_CHECK(filter_count[0] == 0u && bus_start_count[0] == 0u);

    Motor_Init_Config_s config = config_for(0u, 1u);
    config.controller_setting_init_config.close_loop_type = ANGLE_LOOP;
    config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
    config.controller_param_init_config.other_angle_feedback = MotorFeedbackTestMakeSource(
        &external_angle, MotorFeedbackTestReadRadExternal, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL);
    config.controller_param_init_config.angle_PID = pid_config(2.0f, 1000.0f);
    LKMotorInstance *motor = LKMotorInit(&config);
    TEST_CHECK(motor != NULL);
    TEST_CHECK(motor->other_speed_feedback.read == NULL);
    motor->measure.total_angle = 0.5f * (float)RAD_2_DEGREE;
    LKMotorSetRef(motor, 1.0f);

    now_us = 0u;
    LKMotorControl();
    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 1u && tx_frame[0][1] == 0u);

    external_angle = NAN;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(motor->pid_clock.last_status == PID_NUMERIC_ERROR);
    TEST_CHECK(tx_frame[0][0] == 0u && tx_frame[0][1] == 0u);
    PID_State_t state = {0};
    TEST_CHECK(PIDGetState(&motor->angle_PID, &state) == PID_OK);
    TEST_CHECK(state.Err == 0.0f && state.Last_Err == 0.0f && state.ITerm == 0.0f &&
               state.Iout == 0.0f && state.Pout == 0.0f && state.Dout == 0.0f &&
               state.Output == 0.0f);

    external_angle = 0.25f;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 1u && tx_frame[0][1] == 0u);

    motor->motor_settings.angle_feedback_source = (Feedback_Source_e)2;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(motor->pid_clock.last_status == PID_BAD_ARGUMENT);
    TEST_CHECK(tx_frame[0][0] == 0u && tx_frame[0][1] == 0u);
    motor->motor_settings.angle_feedback_source = MOTOR_FEED;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 1u && tx_frame[0][1] == 0u);

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
    TEST_CHECK(tx_frame[0][0] == 1u && tx_frame[0][1] == 0u);
}

static void test_feedforward_overflow(void)
{
    float feedforward = FLT_MAX;
    Motor_Init_Config_s config = config_for(0u, 1u);
    config.controller_setting_init_config.close_loop_type = ANGLE_LOOP;
    config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    config.controller_setting_init_config.feedforward_flag = SPEED_FEEDFORWARD;
    config.controller_param_init_config.speed_feedforward = MotorFeedbackTestMakeSource(
        &feedforward, MotorFeedbackTestReadRadSOutput, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_OUTPUT_AXIS);
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

static void scheduler_rx_callback(CANInstance *instance)
{
    (void)instance;
}

static void scheduler_register(void)
{
    CAN_Init_Config_s config = {0};
    config.can_handle = &hfdcan1;
    config.tx_id = 0x280u;
    config.rx_id = 0x500u;
    config.can_module_callback = scheduler_rx_callback;
    TEST_CHECK(CANRegister(&config) != NULL);
}

static void scheduler_complete_latest(FDCAN_HandleTypeDef *handle)
{
    const uint8_t bus = bus_for_handle(handle);
    TEST_CHECK(bus < DEVICE_CAN_CNT && tx_latest_buffer[bus] != 0u);
    const uint32_t completed = tx_latest_buffer[bus];
    HAL_FDCAN_TxBufferCompleteCallback(handle, completed);
    for (uint8_t index = 0u; index < 16u; ++index)
    {
        if ((completed & (1u << index)) != 0u)
        {
            tx_next_buffer_index[bus] = index;
            break;
        }
    }
}

static CAN_TxFrame_t scheduler_frame(uint32_t id, CAN_TxIdType_t id_type,
                                     const uint8_t *data, uint8_t length)
{
    CAN_TxFrame_t frame = {
        .identifier = id,
        .identifier_type = id_type,
        .data = data,
        .length = length,
    };
    return frame;
}

static void scheduler_periodic_guard(const void *source, uint32_t now_ms,
                                     uint8_t *payload, uint8_t length)
{
    (void)now_ms;
    const uint8_t *allowed = (const uint8_t *)source;
    if (allowed == NULL || *allowed == 0u)
        memset(payload, 0, length);
}

static void test_can_tx_periodic_guard_after_rejected_replacement(void)
{
    scheduler_register();
    static const uint8_t payload_nonzero = 0xA5u;
    static const uint8_t payload_zero = 0u;
    static uint8_t allow_nonzero = 1u;
    CAN_TxFrame_t frame = scheduler_frame(0x356u, CAN_TX_ID_STANDARD,
                                          &payload_nonzero, 1u);
    uint32_t request_id = 0u;

    // 先占满所有 HAL 结果槽和硬件缓冲，随后让普通请求填满软件等待队列。
    for (uint8_t i = 0u; i < 16u; ++i)
    {
        const CAN_TxFrame_t filler = scheduler_frame(0x400u + i,
            CAN_TX_ID_STANDARD, &payload_nonzero, 1u);
        TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &filler, CAN_TX_PRIORITY_NORMAL,
                                    100u, &request_id) == CAN_TX_SUBMIT_ACCEPTED);
        CANProcessTx();
    }
    tx_busy_mask = 1u;
    for (uint8_t i = 0u; i < 15u; ++i)
    {
        const CAN_TxFrame_t waiting = scheduler_frame(0x500u + i,
            CAN_TX_ID_STANDARD, &payload_nonzero, 1u);
        TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &waiting, CAN_TX_PRIORITY_BACKGROUND,
                                    100u, &request_id) == CAN_TX_SUBMIT_ACCEPTED);
    }

    TEST_CHECK(CANSubmitPeriodicPrepared(&hfdcan1, &allow_nonzero, 1u, &frame,
        CAN_TX_PRIORITY_CONTROL, 100u, 1000u, scheduler_periodic_guard,
        &request_id) == CAN_TX_SUBMIT_ACCEPTED);
    const uint32_t guarded_request_id = request_id;

    // 结果记录已满时，新零值提交失败，旧非零周期项仍留在队列中。
    frame.data = &payload_zero;
    uint32_t rejected_id = 0xA5A5A5A5u;
    TEST_CHECK(CANSubmitPeriodicPrepared(&hfdcan1, &allow_nonzero, 1u, &frame,
        CAN_TX_PRIORITY_CONTROL, 100u, 1000u, scheduler_periodic_guard,
        &rejected_id) == CAN_TX_SUBMIT_FULL);
    TEST_CHECK(rejected_id == 0xA5A5A5A5u);

    // 撤销许可后即使队列内仍是旧非零副本，真正交给 HAL 前也会被改为零。
    allow_nonzero = 0u;
    scheduler_complete_latest(&hfdcan1);
    tx_busy_mask = 0u;
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 17u);
    TEST_CHECK(tx_id_history[0][16] == 0x356u);
    TEST_CHECK(tx_data_history[0][16][0] == 0u);
    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(guarded_request_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_IN_FLIGHT);
}

static void test_can_tx_arguments_and_copy(void)
{
    TEST_CHECK(CANClassicFrameWorstCaseBits(CAN_TX_ID_STANDARD, 8u) == 136u);
    TEST_CHECK(CANClassicFrameWorstCaseBits(CAN_TX_ID_EXTENDED, 8u) == 161u);
    TEST_CHECK(CANClassicFrameWorstCaseBits((CAN_TxIdType_t)2, 8u) == 0u);
    TEST_CHECK(CANClassicFrameWorstCaseBits(CAN_TX_ID_STANDARD, 9u) == 0u);

    CAN_TxBusConfig_t bus_config = {
        .queue_capacity = 4u,
        .utilization_limit_permille = 500u,
        .nominal_bitrate_bps = 1000000u,
    };
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, NULL) == CAN_TX_SUBMIT_INVALID);
    bus_config.queue_capacity = 0u;
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, &bus_config) == CAN_TX_SUBMIT_INVALID);
    bus_config.queue_capacity = 4u;
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, &bus_config) == CAN_TX_SUBMIT_ACCEPTED);

    uint8_t payload[8] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};
    CAN_TxFrame_t frame = scheduler_frame(0x123u, CAN_TX_ID_STANDARD, payload, sizeof(payload));
    uint32_t request_id = 0xA5A5A5A5u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &request_id) == CAN_TX_SUBMIT_NOT_READY);
    TEST_CHECK(request_id == 0xA5A5A5A5u);
    scheduler_register();
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, &bus_config) == CAN_TX_SUBMIT_NOT_READY);

    frame = scheduler_frame(0x800u, CAN_TX_ID_STANDARD, payload, sizeof(payload));
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &request_id) == CAN_TX_SUBMIT_INVALID);
    frame = scheduler_frame(0x20000000u, CAN_TX_ID_EXTENDED, payload, sizeof(payload));
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &request_id) == CAN_TX_SUBMIT_INVALID);
    frame = scheduler_frame(0x123u, CAN_TX_ID_STANDARD, payload, 9u);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &request_id) == CAN_TX_SUBMIT_INVALID);
    frame = scheduler_frame(0x123u, CAN_TX_ID_STANDARD, NULL, 1u);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &request_id) == CAN_TX_SUBMIT_INVALID);
    frame = scheduler_frame(0x123u, CAN_TX_ID_STANDARD, payload, sizeof(payload));
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, (CAN_TxPriority_t)4,
                                20u, &request_id) == CAN_TX_SUBMIT_INVALID);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, (CAN_TxPriority_t)-1,
                                20u, &request_id) == CAN_TX_SUBMIT_INVALID);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                0u, &request_id) == CAN_TX_SUBMIT_INVALID);
    TEST_CHECK(request_id == 0xA5A5A5A5u);

    test_ipsr = 1u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &request_id) == CAN_TX_SUBMIT_CONTEXT);
    test_ipsr = 0u;

    static const uint8_t expected_payload[8] = {0x10u, 0x21u, 0x32u, 0x43u,
                                                0x54u, 0x65u, 0x76u, 0x87u};
    for (uint8_t length = 0u; length <= 8u; ++length)
    {
        uint8_t source_data[8];
        memcpy(source_data, expected_payload, sizeof(source_data));
        const CAN_TxIdType_t id_type = (length & 1u) != 0u ?
            CAN_TX_ID_EXTENDED : CAN_TX_ID_STANDARD;
        const uint32_t id = id_type == CAN_TX_ID_STANDARD ?
            (length == 0u ? 0u : length == 2u ? 0x7ffu : 0x321u) :
            (length == 1u ? 0u : length == 3u ? 0x1fffffffu : 0x1234567u);
        frame = scheduler_frame(id, id_type, length == 0u ? NULL : source_data, length);
        now_us = (uint64_t)length * 1000u;
        request_id = 0u;
        TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                    20u, &request_id) == CAN_TX_SUBMIT_ACCEPTED);
        memset(source_data, 0xEE, sizeof(source_data));
        CANProcessTx();

        const uint32_t history_index = length;
        TEST_CHECK(tx_id_history[0][history_index] == id);
        TEST_CHECK(tx_id_type_history[0][history_index] == (uint8_t)id_type);
        TEST_CHECK(tx_length_history[0][history_index] == length);
        TEST_CHECK(memcmp(tx_data_history[0][history_index], expected_payload, length) == 0);
        scheduler_complete_latest(&hfdcan1);

        CAN_TxResult_t result = {0};
        TEST_CHECK(CANGetTxRequestState(request_id, &result) == 1u);
        TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);
        TEST_CHECK(result.accepted_frames == 1u && result.completed_frames == 1u);
    }

    CAN_TxSchedulerStats_t stats = {0};
    TEST_CHECK(CANGetTxSchedulerStats(&hfdcan1, &stats) == 1u);
    TEST_CHECK(stats.completed == 9u && stats.queued == 0u);
}

static void test_can_tx_queue_batch_and_buses(void)
{
    const CAN_TxBusConfig_t bus0_config = {
        .queue_capacity = 2u,
        .utilization_limit_permille = 500u,
        .nominal_bitrate_bps = 1000000u,
    };
    const CAN_TxBusConfig_t bus1_config = {
        .queue_capacity = 1u,
        .utilization_limit_permille = 400u,
        .nominal_bitrate_bps = 500000u,
    };
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, &bus0_config) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANConfigureTxBus(&hfdcan2, &bus1_config) == CAN_TX_SUBMIT_ACCEPTED);
    scheduler_register();

    const uint8_t a_data[2] = {0xA1u, 0xA2u};
    const uint8_t b_data[2] = {0xB1u, 0xB2u};
    const uint8_t c_data[1] = {0xC1u};
    const CAN_TxFrame_t batch[2] = {
        {0x222u, CAN_TX_ID_STANDARD, a_data, sizeof(a_data)},
        {0x223u, CAN_TX_ID_STANDARD, b_data, sizeof(b_data)},
    };
    const CAN_TxFrame_t urgent = scheduler_frame(0x111u, CAN_TX_ID_STANDARD,
                                                  c_data, sizeof(c_data));
    const CAN_TxFrame_t other_bus = scheduler_frame(0x400u, CAN_TX_ID_STANDARD,
                                                     c_data, sizeof(c_data));
    uint32_t batch_id = 0u;
    uint32_t urgent_id = 0u;
    uint32_t other_bus_id = 0u;
    TEST_CHECK(CANSubmitOneShotBatch(&hfdcan1, NULL, batch, 2u, CAN_TX_PRIORITY_NORMAL,
                                     30u, &batch_id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &urgent, CAN_TX_PRIORITY_URGENT,
                                30u, &urgent_id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANSubmitOneShot(&hfdcan2, NULL, &other_bus, CAN_TX_PRIORITY_NORMAL,
                                30u, &other_bus_id) == CAN_TX_SUBMIT_ACCEPTED);

    uint32_t rejected_id = 0x5A5A5A5Au;
    TEST_CHECK(CANSubmitOneShotBatch(&hfdcan1, NULL, batch, 2u, CAN_TX_PRIORITY_NORMAL,
                                     30u, &rejected_id) == CAN_TX_SUBMIT_FULL);
    TEST_CHECK(rejected_id == 0x5A5A5A5Au);
    CAN_TxSchedulerStats_t stats = {0};
    TEST_CHECK(CANGetTxSchedulerStats(&hfdcan1, &stats) == 1u);
    TEST_CHECK(stats.queued == 2u && stats.queue_peak == 2u);
    TEST_CHECK(CANGetTxSchedulerStats(&hfdcan2, &stats) == 1u && stats.queued == 1u);

    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 1u && tx_id_history[0][0] == 0x111u);
    TEST_CHECK(tx_accept_count[1] == 1u && tx_id_history[1][0] == 0x400u);
    scheduler_complete_latest(&hfdcan1);
    scheduler_complete_latest(&hfdcan2);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 2u && tx_id_history[0][1] == 0x222u);
    scheduler_complete_latest(&hfdcan1);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 3u && tx_id_history[0][2] == 0x223u);
    scheduler_complete_latest(&hfdcan1);

    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(batch_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE && result.completed_frames == 2u);
    TEST_CHECK(CANGetTxRequestState(urgent_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);
    TEST_CHECK(CANGetTxRequestState(other_bus_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);
    TEST_CHECK(CANCancelTxRequest(urgent_id) == 0u);

    CAN_TxFrame_t invalid_batch[2] = {batch[0], batch[1]};
    invalid_batch[1].identifier = 0x800u;
    rejected_id = 0x5A5A5A5Au;
    TEST_CHECK(CANSubmitOneShotBatch(&hfdcan1, NULL, invalid_batch, 2u,
                                     CAN_TX_PRIORITY_NORMAL, 30u,
                                     &rejected_id) == CAN_TX_SUBMIT_INVALID);
    TEST_CHECK(rejected_id == 0x5A5A5A5Au);
    CAN_TxFrame_t too_many[CAN_TX_MAX_BATCH_FRAMES + 1u];
    for (size_t i = 0u; i < CAN_TX_MAX_BATCH_FRAMES + 1u; ++i)
        too_many[i] = batch[0];
    TEST_CHECK(CANSubmitOneShotBatch(&hfdcan1, NULL, too_many,
                                     CAN_TX_MAX_BATCH_FRAMES + 1u,
                                     CAN_TX_PRIORITY_NORMAL, 30u,
                                     &rejected_id) == CAN_TX_SUBMIT_INVALID);

    // 64字节话题连同时间和校验按新版格式需要13帧，必须整组提交。
    TEST_CHECK(CAN_TX_MAX_BATCH_FRAMES >= 13u);
    CAN_TxFrame_t maximum_batch[CAN_TX_MAX_BATCH_FRAMES];
    for (size_t i = 0u; i < CAN_TX_MAX_BATCH_FRAMES; ++i)
    {
        maximum_batch[i] = batch[0];
        maximum_batch[i].identifier = 0x600u + (uint32_t)i;
    }
    uint32_t maximum_batch_id = 0u;
    TEST_CHECK(CANSubmitOneShotBatch(&hfdcan1, NULL, maximum_batch,
                                     CAN_TX_MAX_BATCH_FRAMES,
                                     CAN_TX_PRIORITY_NORMAL, 30u,
                                     &maximum_batch_id) == CAN_TX_SUBMIT_ACCEPTED);
    for (size_t i = 0u; i < CAN_TX_MAX_BATCH_FRAMES; ++i)
    {
        CANProcessTx();
        TEST_CHECK(tx_id_history[0][3u + i] == 0x600u + i);
        scheduler_complete_latest(&hfdcan1);
    }
    TEST_CHECK(CANGetTxRequestState(maximum_batch_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE &&
               result.completed_frames == CAN_TX_MAX_BATCH_FRAMES);

    uint32_t later_deadline_id = 0u;
    uint32_t earlier_deadline_id = 0u;
    const CAN_TxFrame_t later_deadline = scheduler_frame(0x501u, CAN_TX_ID_STANDARD,
                                                          c_data, sizeof(c_data));
    const CAN_TxFrame_t earlier_deadline = scheduler_frame(0x500u, CAN_TX_ID_STANDARD,
                                                            c_data, sizeof(c_data));
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &later_deadline,
                                CAN_TX_PRIORITY_NORMAL, 20u,
                                &later_deadline_id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &earlier_deadline,
                                CAN_TX_PRIORITY_NORMAL, 10u,
                                &earlier_deadline_id) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(tx_id_history[0][3u + CAN_TX_MAX_BATCH_FRAMES] == 0x500u);
    scheduler_complete_latest(&hfdcan1);
    CANProcessTx();
    TEST_CHECK(tx_id_history[0][4u + CAN_TX_MAX_BATCH_FRAMES] == 0x501u);
    scheduler_complete_latest(&hfdcan1);
    TEST_CHECK(CANGetTxRequestState(later_deadline_id, &result) == 1u &&
               result.state == CAN_TX_RESULT_COMPLETE);
    TEST_CHECK(CANGetTxRequestState(earlier_deadline_id, &result) == 1u &&
               result.state == CAN_TX_RESULT_COMPLETE);
}

// 生产调度器按组发送完整话题，并将内容从实际提交的帧中还原。
static void test_can_tx_topic_group(void)
{
    CAN_Init_Config_s config={0};
    config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config);
    TEST_CHECK(can != NULL);
    CanTopicTransport_t transport;
    TEST_CHECK(CanTopicTransportInit(&transport,can));
    CanTopicMessage_t input={.topic_id=7u,.length=64u,.source_ms=25u};
    for (unsigned i=0u;i<64u;++i) input.payload[i]=(uint8_t)i;
    uint32_t topic_id=0u;
    TEST_CHECK(CanTopicTransportSend(&transport,&input,30u,&topic_id)==CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx(); scheduler_complete_latest(&hfdcan1);
    uint8_t normal_byte=0u; uint32_t normal_id=0u;
    CAN_TxFrame_t normal=scheduler_frame(0x102u,CAN_TX_ID_STANDARD,&normal_byte,1u);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1,NULL,&normal,CAN_TX_PRIORITY_NORMAL,30u,&normal_id)==CAN_TX_SUBMIT_ACCEPTED);
    for (size_t i=1u;i<CAN_TOPIC_MAX_FRAMES;++i) {
        CANProcessTx(); scheduler_complete_latest(&hfdcan1);
        TEST_CHECK(tx_id_history[0][i]==0x321u);
    }
    CANProcessTx(); scheduler_complete_latest(&hfdcan1);
    TEST_CHECK(tx_id_history[0][CAN_TOPIC_MAX_FRAMES]==0x102u);
    CAN_TxResult_t result;
    TEST_CHECK(CANGetTxRequestState(topic_id,&result) && result.state==CAN_TX_RESULT_COMPLETE && result.completed_frames==13u);
    CanTopicReceiver_t receiver={0}; CanTopicMessage_t output={0};
    for (size_t i=0u;i<CAN_TOPIC_MAX_FRAMES;++i) {
        CanTopicFrame_t frame={.length=tx_length_history[0][i]};
        memcpy(frame.bytes,tx_data_history[0][i],frame.length);
        CanTopicWireStatus_t status=CanTopicReceive(&receiver,&frame,(uint32_t)i,(uint32_t)i,50u,&output);
        TEST_CHECK(status==(i+1u==CAN_TOPIC_MAX_FRAMES ? CAN_TOPIC_OK : CAN_TOPIC_INCOMPLETE));
    }
    TEST_CHECK(output.source_ms==25u && output.first_rx_ms==0u && output.length==64u && output.payload[63]==63u);
}

// 控制请求到达后放弃可中断的旧话题组，不让剩余分片占用控制期限。
static void test_can_tx_topic_interrupt(void)
{
    CAN_Init_Config_s config={0};
    config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config); TEST_CHECK(can != NULL);
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,can));
    CanTopicMessage_t input={.topic_id=7u,.length=64u,.source_ms=25u};
    uint32_t topic_id=0u;
    TEST_CHECK(CanTopicTransportSend(&transport,&input,30u,&topic_id)==CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx(); scheduler_complete_latest(&hfdcan1);
    uint8_t byte=0u; uint32_t control_id=0u;
    CAN_TxFrame_t control=scheduler_frame(0x101u,CAN_TX_ID_STANDARD,&byte,1u);
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1,&byte,1u,&control,CAN_TX_PRIORITY_CONTROL,4u,1000u,&control_id)==CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx(); scheduler_complete_latest(&hfdcan1);
    TEST_CHECK(tx_id_history[0][1u]==0x101u);
    CAN_TxResult_t result;
    TEST_CHECK(CANGetTxRequestState(topic_id,&result) && result.state==CAN_TX_RESULT_FAILED && result.accepted_frames==1u);
    CANProcessTx(); TEST_CHECK(tx_accept_count[0]==2u);
}

// 来源和本机时钟在此测试中相同；真实双板必须由对时模块提供映射。
static uint8_t test_topic_map(void *context,uint32_t remote,uint32_t now,uint32_t *local,uint32_t *uncertainty)
{
    (void)now;
    if (!*(uint8_t *)context) return 0u;
    *local=remote; *uncertainty=0u; return 1u;
}
static uint8_t test_topic_copy(void *context,const void *input,size_t input_size,void *output,size_t output_size)
{
    (void)context;
    if (input_size!=4u || output_size!=4u) return 0u;
    memcpy(output,input,4u); return 1u;
}
// 从生产中断队列进入CAN派发、拼包、映射和生产消息中心。
static void test_can_tx_topic_rx(void)
{
    const MessageTopicDescriptor_t descriptor={19u,4u,50u,1u,MESSAGE_DELIVERY_SNAPSHOT,
        MESSAGE_OVERFLOW_REPLACE_OLDEST,MESSAGE_CLOCK_LOCAL_MAPPED,1u,2u,0u};
    TEST_CHECK(MessageCenterInit(&descriptor,1u)==MESSAGE_OK);
    MessagePublisher_t *publisher=MessagePublisherRegister(19u,1u);
    MessageSubscriber_t *subscriber=MessageSubscriberRegister(19u,2u);
    TEST_CHECK(publisher && subscriber);
    CanTopicLink_t link={0}; uint8_t synchronized=1u;
    const CanTopicCodec_t codec={test_topic_copy,NULL,4u};
    CAN_Init_Config_s config={0}; config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x501u;
    TEST_CHECK(CanTopicLinkInit(&link,&config,NULL,publisher,NULL,&codec,test_topic_map,&synchronized,50u));
    CanTopicMessage_t message={.topic_id=19u,.length=4u,.source_ms=100u,.payload={1u,2u,3u,4u}};
    CanTopicFrame_t frames[CAN_TOPIC_MAX_FRAMES]; size_t count=0u;
    TEST_CHECK(CanTopicEncode(&message,1u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    for (size_t i=0u;i<count;++i) {
        TestRxFrame_t *item=&rx_frames[0][i]; memset(item,0,sizeof(*item));
        item->header.Identifier=0x501u; item->header.DataLength=(uint32_t)frames[i].length<<16u;
        item->header.RxFrameType=FDCAN_DATA_FRAME; item->header.IdType=FDCAN_STANDARD_ID;
        memcpy(item->data,frames[i].bytes,frames[i].length);
    }
    rx_frame_count[0]=(uint8_t)count; rx_frame_read[0]=0u; now_us=100000u;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    CANProcessRx();
    uint8_t output[4]={0}; MessageFreshness_t freshness;
    TEST_CHECK(link.last_receive==CAN_BRIDGE_OK);
    TEST_CHECK(MessageReadSnapshot(subscriber,output,sizeof(output),100u,&freshness)==MESSAGE_OK &&
               memcmp(output,message.payload,4u)==0 && freshness.received_time_ms==100u);
    message.source_ms=101u;
    TEST_CHECK(CanTopicEncode(&message,2u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    frames[count-1u].bytes[frames[count-1u].length-1u]^=1u;
    for (size_t i=0u;i<count;++i) {
        rx_frames[0][i].header.DataLength=(uint32_t)frames[i].length<<16u;
        memcpy(rx_frames[0][i].data,frames[i].bytes,frames[i].length);
    }
    rx_frame_count[0]=(uint8_t)count; rx_frame_read[0]=0u; now_us=101000u;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    CANProcessRx();
    TEST_CHECK(link.last_receive==CAN_BRIDGE_WIRE);
    TEST_CHECK(MessageReadSnapshot(subscriber,output,sizeof(output),101u,&freshness)==MESSAGE_OK &&
               freshness.received_time_ms==100u && freshness.source_time_ms==100u);
    synchronized=0u; uint32_t id=0u; CanTopicBridgeStatus_t sync;
    TEST_CHECK(CanTopicLinkService(&link,101u,&id,&sync)==CAN_BRIDGE_UNSYNCED);
    TEST_CHECK(MessageReadSnapshot(subscriber,output,sizeof(output),101u,&freshness)==MESSAGE_ERR_EMPTY);
}

// 新版本到达时停止旧组，已经进入硬件的帧结束前不能提交新组。
static void test_can_tx_topic_cancel(void)
{
    const MessageTopicDescriptor_t descriptor={19u,4u,50u,1u,MESSAGE_DELIVERY_SNAPSHOT,
        MESSAGE_OVERFLOW_REPLACE_OLDEST,MESSAGE_CLOCK_LOCAL,1u,2u,0u};
    TEST_CHECK(MessageCenterInit(&descriptor,1u)==MESSAGE_OK);
    MessagePublisher_t *publisher=MessagePublisherRegister(19u,1u);
    MessageSubscriber_t *subscriber=MessageSubscriberRegister(19u,2u);
    TEST_CHECK(publisher && subscriber);
    CAN_Init_Config_s config={0}; config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x501u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config); TEST_CHECK(can!=NULL);
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,can));
    const CanTopicCodec_t codec={test_topic_copy,NULL,4u};
    CanTopicBridge_t bridge;
    TEST_CHECK(CanTopicBridgeInit(&bridge,&transport,subscriber,NULL,&codec,NULL,NULL,NULL)==CAN_BRIDGE_OK);
    uint32_t value=1u, first_id=0u, second_id=0xabcdu;
    TEST_CHECK(MessagePublishAt(publisher,&value,0u,0u,0u,MESSAGE_CLOCK_LOCAL)==MESSAGE_OK);
    TEST_CHECK(CanTopicBridgeSend(&bridge,0u,&first_id)==CAN_BRIDGE_OK);
    CANProcessTx(); uint32_t first_mask=tx_latest_buffer[0];
    value=2u; now_us=1000u;
    TEST_CHECK(MessagePublishAt(publisher,&value,1u,1u,1u,MESSAGE_CLOCK_LOCAL)==MESSAGE_OK);
    TEST_CHECK(CanTopicBridgeSend(&bridge,1u,&second_id)==CAN_BRIDGE_SEND && second_id==0xabcdu);
    CAN_TxResult_t result;
    TEST_CHECK(CANGetTxRequestState(first_id,&result) && result.state==CAN_TX_RESULT_CANCELLED && result.pending_frames==1u);
    HAL_FDCAN_TxBufferCompleteCallback(&hfdcan1,first_mask);
    TEST_CHECK(CanTopicBridgeSend(&bridge,1u,&second_id)==CAN_BRIDGE_OK && second_id!=0xabcdu);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0]==2u && tx_data_history[0][1u][0]==0xa0u && tx_data_history[0][1u][1]==1u);
}

// 同总线每轮都有控制帧时，话题期限耗尽必须报告过期，不能伪称已发送。
static void test_can_tx_topic_contention(void)
{
    CAN_Init_Config_s config={0}; config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config); TEST_CHECK(can!=NULL);
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,can));
    CanTopicMessage_t message={.topic_id=7u,.length=64u}; uint32_t topic_id=0u;
    TEST_CHECK(CanTopicTransportSend(&transport,&message,5u,&topic_id)==CAN_TX_SUBMIT_ACCEPTED);
    uint8_t data=0u; CAN_TxFrame_t control=scheduler_frame(0x101u,CAN_TX_ID_STANDARD,&data,1u);
    for (unsigned i=0u;i<5u;++i) {
        now_us=(uint64_t)i*1000u; uint32_t control_id=0u;
        TEST_CHECK(CANSubmitPeriodic(&hfdcan1,&data,1u,&control,CAN_TX_PRIORITY_CONTROL,4u,1000u,
                                     &control_id)==CAN_TX_SUBMIT_ACCEPTED);
        CANProcessTx(); scheduler_complete_latest(&hfdcan1);
        TEST_CHECK(tx_id_history[0][i]==0x101u);
    }
    now_us=5000u; CANProcessTx();
    CAN_TxResult_t result;
    TEST_CHECK(CANGetTxRequestState(topic_id,&result) && result.state==CAN_TX_RESULT_EXPIRED &&
               result.accepted_frames==0u);
}

// 专用桥接总线每5ms产生5帧，并故意漏掉每组首轮服务，验证有界补充推进。
static void test_can_tx_topic_drain(void)
{
    CAN_Init_Config_s config={0}; config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config); TEST_CHECK(can!=NULL);
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,can));
    CanTopicMessage_t message={.topic_id=7u,.length=21u}; uint32_t ids[6]={0};
    for (unsigned tick=0u;tick<30u;++tick) {
        now_us=(uint64_t)tick*1000u;
        if (tick%5u==0u) {
            message.source_ms=tick;
            TEST_CHECK(CanTopicTransportSend(&transport,&message,50u,&ids[tick/5u])==CAN_TX_SUBMIT_ACCEPTED);
            if (tick==0u) {
                CANProcessTxBus(NULL);
                test_ipsr=1u; CANProcessTxBus(&hfdcan1); test_ipsr=0u;
                TEST_CHECK(tx_accept_count[0]==0u);
            }
            continue; // 模拟服务任务本轮没有得到调度。
        }
        for (unsigned attempt=0u;attempt<2u;++attempt) {
            uint32_t before=tx_accept_count[0];
            CANProcessTxBus(&hfdcan1);
            if (tx_accept_count[0]!=before) scheduler_complete_latest(&hfdcan1);
        }
    }
    for (unsigned i=0u;i<6u;++i) {
        CAN_TxResult_t result;
        TEST_CHECK(CANGetTxRequestState(ids[i],&result) && result.state==CAN_TX_RESULT_COMPLETE && result.completed_frames==5u);
    }
    TEST_CHECK(tx_accept_count[0]==30u && tx_accept_count[1]==0u && tx_accept_count[2]==0u);
}

// 任务在计算期限后被抢占，入队时应保留原来的绝对截止时刻。
static void test_can_tx_topic_absolute_deadline(void)
{
    CAN_Init_Config_s config={0};
    config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    TEST_CHECK(CANRegister(&config)!=NULL);
    const uint8_t data=0x11u;
    const CAN_TxFrame_t frame=scheduler_frame(0x321u,CAN_TX_ID_STANDARD,&data,1u);
    uint32_t id=99u;
    now_us=115000u;
    TEST_CHECK(CANSubmitInterruptibleBatchUntil(&hfdcan1,&data,&frame,1u,
        CAN_TX_PRIORITY_NORMAL,150u,&id)==CAN_TX_SUBMIT_ACCEPTED);
    now_us=150000u;
    CANProcessTx();
    CAN_TxResult_t result;
    TEST_CHECK(CANGetTxRequestState(id,&result) && result.state==CAN_TX_RESULT_EXPIRED);
    id=99u;
    TEST_CHECK(CANSubmitInterruptibleBatchUntil(&hfdcan1,&data,&frame,1u,
        CAN_TX_PRIORITY_NORMAL,150u,&id)==CAN_TX_SUBMIT_INVALID && id==99u);
    now_us=(uint64_t)(UINT32_MAX-2u)*1000u;
    TEST_CHECK(CANSubmitInterruptibleBatchUntil(&hfdcan1,&data,&frame,1u,
        CAN_TX_PRIORITY_NORMAL,2u,&id)==CAN_TX_SUBMIT_ACCEPTED);
    now_us=(uint64_t)2u*1000u;
    CANProcessTx();
    TEST_CHECK(CANGetTxRequestState(id,&result) && result.state==CAN_TX_RESULT_EXPIRED);
}

// 模拟专用桥接总线同时发送底盘话题、对时和可选电容命令。
static void test_can_tx_topic_sync_budget(void)
{
    CAN_Init_Config_s config={0}; config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config); TEST_CHECK(can!=NULL);
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,can));
    CanTimeSync_t sync={0}; TEST_CHECK(CanTimeSyncInit(&sync,PLATFORM_CAN_BUS_1,can->tx_id,can));
    CanTopicMessage_t message={.topic_id=7u,.length=21u};
    uint32_t ids[6]={0u};
    uint8_t cap_data[8]={0u};
    CAN_TxFrame_t cap=scheduler_frame(0x302u,CAN_TX_ID_STANDARD,cap_data,8u);
    // 固定 CRC8/0x31 向量：请求编号0x1234，发送板时间1000ms。
    const uint8_t peer_probe[8]={0xc1u,0x34u,0x12u,0xe8u,0x03u,0u,0u,0x84u};
    for (unsigned tick=0u;tick<30u;++tick) {
        now_us=(uint64_t)tick*1000u;
        if (tick%5u==0u) {
            if (tick>0u) {
                CAN_TxResult_t previous;
                TEST_CHECK(CANGetTxRequestState(ids[tick/5u-1u],&previous) &&
                           previous.state==CAN_TX_RESULT_COMPLETE && previous.completed_frames==5u);
            }
            if (tick==0u) TEST_CHECK(CanTimeSyncService(&sync,0u)==CAN_TIME_SYNC_OK);
            if (tick==10u)
                TEST_CHECK(CanTimeSyncReceive(&sync,peer_probe,8u,10u,10u)==CAN_TIME_SYNC_OK);
            if (tick%10u==0u) {
                uint32_t cap_id=0u;
                TEST_CHECK(CANSubmitPeriodic(&hfdcan1,&cap_data,1u,&cap,CAN_TX_PRIORITY_NORMAL,
                                             20u,10000u,&cap_id)==CAN_TX_SUBMIT_ACCEPTED);
            }
            message.source_ms=tick;
            TEST_CHECK(CanTopicTransportSend(&transport,&message,50u,&ids[tick/5u])==CAN_TX_SUBMIT_ACCEPTED);
            continue; // 每组首轮不推进，给任务调度抖动留出一轮余量。
        }
        for (unsigned attempt=0u;attempt<2u;++attempt) {
            const uint32_t before=tx_accept_count[0];
            CANProcessTxBus(&hfdcan1);
            if (tx_accept_count[0]!=before) scheduler_complete_latest(&hfdcan1);
        }
    }
    CAN_TxResult_t final;
    TEST_CHECK(CANGetTxRequestState(ids[5],&final) && final.state==CAN_TX_RESULT_COMPLETE &&
               final.completed_frames==5u);
    TEST_CHECK(tx_accept_count[0]==36u && tx_accept_count[1]==0u && tx_accept_count[2]==0u);
}

// 即使有额外推进机会，共用总线上的新控制帧仍会中止未发送完的话题。
static void test_can_tx_topic_shared(void)
{
    CAN_Init_Config_s config={0}; config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config); TEST_CHECK(can!=NULL);
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,can));
    CanTopicMessage_t message={.topic_id=7u,.length=21u}; uint32_t topic_id=0u;
    TEST_CHECK(CanTopicTransportSend(&transport,&message,50u,&topic_id)==CAN_TX_SUBMIT_ACCEPTED);
    uint8_t data=0u; CAN_TxFrame_t control=scheduler_frame(0x101u,CAN_TX_ID_STANDARD,&data,1u);
    uint32_t control_id=0u;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1,&data,1u,&control,CAN_TX_PRIORITY_CONTROL,4u,1000u,&control_id)==CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx(); scheduler_complete_latest(&hfdcan1);
    CANProcessTxBus(&hfdcan1); scheduler_complete_latest(&hfdcan1);
    TEST_CHECK(tx_id_history[0][0u]==0x101u && tx_id_history[0][1u]==0x321u);
    now_us=1000u;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1,&data,1u,&control,CAN_TX_PRIORITY_CONTROL,4u,1000u,&control_id)==CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx(); scheduler_complete_latest(&hfdcan1);
    CANProcessTxBus(&hfdcan1);
    CAN_TxResult_t result;
    TEST_CHECK(tx_accept_count[0]==3u && tx_id_history[0][2u]==0x101u);
    TEST_CHECK(CANGetTxRequestState(topic_id,&result) && result.state==CAN_TX_RESULT_FAILED && result.accepted_frames==1u);
}

// 同一轮两帧可同时被HAL接受，完成回调逆序时仍要等待整组结束。
static void test_can_tx_topic_pending(void)
{
    CAN_Init_Config_s config={0}; config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config); TEST_CHECK(can!=NULL);
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,can));
    CanTopicMessage_t message={.topic_id=7u,.length=21u}; uint32_t topic_id=0u;
    TEST_CHECK(CanTopicTransportSend(&transport,&message,50u,&topic_id)==CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx(); uint32_t first=tx_latest_buffer[0];
    CANProcessTxBus(&hfdcan1); uint32_t second=tx_latest_buffer[0];
    TEST_CHECK(first!=second && tx_accept_count[0]==2u);
    CAN_TxResult_t result;
    TEST_CHECK(CANGetTxRequestState(topic_id,&result) && result.pending_frames==2u);
    HAL_FDCAN_TxBufferCompleteCallback(&hfdcan1,second);
    HAL_FDCAN_TxBufferCompleteCallback(&hfdcan1,first);
    TEST_CHECK(CANGetTxRequestState(topic_id,&result) && result.pending_frames==0u &&
               result.completed_frames==2u && result.state==CAN_TX_RESULT_ACTIVE);
    for (unsigned i=2u;i<5u;++i) { CANProcessTxBus(&hfdcan1); scheduler_complete_latest(&hfdcan1); }
    TEST_CHECK(CANGetTxRequestState(topic_id,&result) && result.completed_frames==5u && result.state==CAN_TX_RESULT_COMPLETE);
}

static void test_can_tx_periodic_latest_and_budget(void)
{
    const CAN_TxBusConfig_t bus_config = {
        .queue_capacity = 4u,
        .utilization_limit_permille = 250u,
        .nominal_bitrate_bps = 1000000u,
    };
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, &bus_config) == CAN_TX_SUBMIT_ACCEPTED);
    scheduler_register();

    static const uint8_t first[8] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};
    static const uint8_t latest[8] = {8u, 7u, 6u, 5u, 4u, 3u, 2u, 1u};
    static const uint8_t extended[8] = {9u, 8u, 7u, 6u, 5u, 4u, 3u, 2u};
    static int source_a;
    static int source_b;
    uint32_t first_id = 0u;
    uint32_t latest_id = 0u;
    uint32_t second_stream_id = 0xABABABABu;
    CAN_TxFrame_t frame_a = scheduler_frame(0x300u, CAN_TX_ID_STANDARD, first, 8u);
    CAN_TxFrame_t frame_b = scheduler_frame(0x1234567u, CAN_TX_ID_EXTENDED, extended, 8u);

    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_a, 1u, &frame_a,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 1000u,
                                 &first_id) == CAN_TX_SUBMIT_ACCEPTED);
    frame_a.data = latest;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_a, 1u, &frame_a,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 1000u,
                                 &latest_id) == CAN_TX_SUBMIT_COALESCED);
    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(first_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_SUPERSEDED);

    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_b, 2u, &frame_b,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 1000u,
                                 &second_stream_id) == CAN_TX_SUBMIT_BANDWIDTH);
    TEST_CHECK(second_stream_id == 0xABABABABu);
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_b, 2u, &frame_b,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 2000u,
                                 &second_stream_id) == CAN_TX_SUBMIT_ACCEPTED);

    CAN_TxSchedulerStats_t stats = {0};
    TEST_CHECK(CANGetTxSchedulerStats(&hfdcan1, &stats) == 1u);
    TEST_CHECK(stats.periodic_reserved_permille == 217u && stats.queued == 2u);
    CANProcessTx();
    TEST_CHECK(tx_id_history[0][0] == 0x300u);
    TEST_CHECK(memcmp(tx_data_history[0][0], latest, 8u) == 0);
    scheduler_complete_latest(&hfdcan1);
    CANProcessTx();
    TEST_CHECK(tx_id_history[0][1] == 0x1234567u);
    TEST_CHECK(tx_id_type_history[0][1] == FDCAN_EXTENDED_ID);
    scheduler_complete_latest(&hfdcan1);
    TEST_CHECK(CANGetTxRequestState(latest_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);
    TEST_CHECK(CANGetTxRequestState(second_stream_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);

    static const uint8_t busy_value[8] = {0xB0u, 0xB1u, 0xB2u, 0xB3u,
                                          0xB4u, 0xB5u, 0xB6u, 0xB7u};
    static const uint8_t recovered_value[8] = {0xC0u, 0xC1u, 0xC2u, 0xC3u,
                                               0xC4u, 0xC5u, 0xC6u, 0xC7u};
    uint32_t busy_id = 0u;
    tx_busy_mask = 1u;
    frame_a.data = busy_value;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_a, 1u, &frame_a,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 1000u,
                                 &busy_id) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(CANGetTxRequestState(busy_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_FAILED);
    TEST_CHECK(tx_accept_count[0] == 2u);

    tx_busy_mask = 0u;
    uint32_t recovered_id = 0u;
    frame_a.data = recovered_value;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_a, 1u, &frame_a,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 1000u,
                                 &recovered_id) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 3u);
    TEST_CHECK(memcmp(tx_data_history[0][2], recovered_value, 8u) == 0);
    scheduler_complete_latest(&hfdcan1);

    static const uint8_t rejected_value[8] = {0xD0u, 0xD1u, 0xD2u, 0xD3u,
                                              0xD4u, 0xD5u, 0xD6u, 0xD7u};
    static const uint8_t error_recovery_value[8] = {0xE0u, 0xE1u, 0xE2u, 0xE3u,
                                                    0xE4u, 0xE5u, 0xE6u, 0xE7u};
    uint32_t rejected_id = 0u;
    tx_error_mask = 1u;
    frame_a.data = rejected_value;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_a, 1u, &frame_a,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 1000u,
                                 &rejected_id) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(CANGetTxRequestState(rejected_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_FAILED && tx_accept_count[0] == 3u);

    tx_error_mask = 0u;
    uint32_t error_recovery_id = 0u;
    frame_a.data = error_recovery_value;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_a, 1u, &frame_a,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 1000u,
                                 &error_recovery_id) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 4u);
    TEST_CHECK(memcmp(tx_data_history[0][3], error_recovery_value, 8u) == 0);
    scheduler_complete_latest(&hfdcan1);
}

static void test_can_tx_expiry_cancel_and_wrap(void)
{
    const CAN_TxBusConfig_t bus_config = {
        .queue_capacity = 2u,
        .utilization_limit_permille = 500u,
        .nominal_bitrate_bps = 1000000u,
    };
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, &bus_config) == CAN_TX_SUBMIT_ACCEPTED);
    scheduler_register();
    const uint8_t data[1] = {0x42u};
    const CAN_TxFrame_t frame = scheduler_frame(0x123u, CAN_TX_ID_STANDARD, data, 1u);
    uint32_t expired_id = 0u;
    uint32_t cancelled_id = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                5u, &expired_id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &cancelled_id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANCancelTxRequest(cancelled_id) == 1u);
    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(cancelled_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_CANCELLED);
    now_us = 5000u;
    uint32_t fresh_id = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &fresh_id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANGetTxRequestState(expired_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_EXPIRED);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 1u);
    TEST_CHECK(CANGetTxRequestState(fresh_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_IN_FLIGHT);
    scheduler_complete_latest(&hfdcan1);

    now_us = (uint64_t)(UINT32_MAX - 2u) * 1000u;
    uint32_t wrap_id = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                5u, &wrap_id) == CAN_TX_SUBMIT_ACCEPTED);
    now_us = ((uint64_t)UINT32_MAX + 1u) * 1000u;
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 2u);
    TEST_CHECK(CANGetTxRequestState(wrap_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_IN_FLIGHT);
    scheduler_complete_latest(&hfdcan1);
    TEST_CHECK(CANGetTxRequestState(wrap_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);

    now_us = (uint64_t)(UINT32_MAX - 2u) * 1000u;
    uint32_t wrapped_expiry_id = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                5u, &wrapped_expiry_id) == CAN_TX_SUBMIT_ACCEPTED);
    now_us = 2000u;
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 2u);
    TEST_CHECK(CANGetTxRequestState(wrapped_expiry_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_EXPIRED);
}

static void test_can_tx_budget_large_arithmetic(void)
{
    const CAN_TxBusConfig_t bus_config = {
        .queue_capacity = 2u,
        .utilization_limit_permille = 1u,
        .nominal_bitrate_bps = UINT32_MAX,
    };
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, &bus_config) == CAN_TX_SUBMIT_ACCEPTED);
    scheduler_register();

    static const uint8_t data[8] = {0u};
    const CAN_TxFrame_t frame = scheduler_frame(0x456u, CAN_TX_ID_STANDARD, data, 8u);
    static int source;
    uint32_t request_id = 0u;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source, 1u, &frame,
                                 CAN_TX_PRIORITY_CONTROL, INT32_MAX, UINT32_MAX,
                                 &request_id) == CAN_TX_SUBMIT_ACCEPTED);
    CAN_TxSchedulerStats_t stats = {0};
    TEST_CHECK(CANGetTxSchedulerStats(&hfdcan1, &stats) == 1u);
    TEST_CHECK(stats.periodic_reserved_permille == 1u);
}

static void test_can_tx_retry_boundary_and_abort(void)
{
    scheduler_register();
    const uint8_t data[2] = {0xAAu, 0x55u};
    const CAN_TxFrame_t frame = scheduler_frame(0x321u, CAN_TX_ID_STANDARD, data, 2u);
    uint32_t busy_id = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_URGENT,
                                100u, &busy_id) == CAN_TX_SUBMIT_ACCEPTED);
    tx_busy_mask = 1u;
    for (uint8_t attempt = 0u; attempt < 4u; ++attempt)
        CANProcessTx();
    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(busy_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_FAILED && result.accepted_frames == 0u);

    CAN_TxSchedulerStats_t stats = {0};
    TEST_CHECK(CANGetTxSchedulerStats(&hfdcan1, &stats) == 1u);
    TEST_CHECK(stats.fifo_busy == 4u && stats.retries == 3u && stats.failed == 1u);
    tx_busy_mask = 0u;
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 0u);

    uint32_t hal_error_id = 0u;
    tx_error_mask = 1u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_URGENT,
                                100u, &hal_error_id) == CAN_TX_SUBMIT_ACCEPTED);
    for (uint8_t attempt = 0u; attempt < 4u; ++attempt)
        CANProcessTx();
    TEST_CHECK(CANGetTxRequestState(hal_error_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_FAILED && result.accepted_frames == 0u);
    TEST_CHECK(result.last_hal_error == HAL_ERROR);
    tx_error_mask = 0u;

    uint32_t accepted_id = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_URGENT,
                                100u, &accepted_id) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 1u);
    HAL_FDCAN_TxBufferAbortCallback(&hfdcan1, tx_latest_buffer[0]);
    TEST_CHECK(CANGetTxRequestState(accepted_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_FAILED && result.accepted_frames == 1u);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 1u);

    // 多帧组中任一已接受帧中止后，未提交的后续分片不得继续发送。
    CAN_TxFrame_t group[3]={frame,frame,frame};
    uint32_t group_id=0u;
    TEST_CHECK(CANSubmitOneShotBatch(&hfdcan1,NULL,group,3u,CAN_TX_PRIORITY_NORMAL,100u,&group_id)==CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0]==2u);
    HAL_FDCAN_TxBufferAbortCallback(&hfdcan1,tx_latest_buffer[0]);
    // 结果历史回绕时，尚未清理的失败组仍持有自己的结果槽。
    for (unsigned i=0u;i<40u;++i) {
        uint32_t temporary_id=0u;
        TEST_CHECK(CANSubmitOneShot(&hfdcan1,NULL,&frame,CAN_TX_PRIORITY_NORMAL,100u,&temporary_id)==CAN_TX_SUBMIT_ACCEPTED);
        TEST_CHECK(CANCancelTxRequest(temporary_id));
    }
    TEST_CHECK(CANGetTxRequestState(group_id,&result) && result.state==CAN_TX_RESULT_FAILED);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0]==2u);
    TEST_CHECK(CANGetTxRequestState(group_id,&result) && result.state==CAN_TX_RESULT_FAILED);
}

static void test_can_tx_retry_yields_for_priority(void)
{
    scheduler_register();
    const uint8_t data = 0x5Au;
    const CAN_TxFrame_t normal = scheduler_frame(0x321u, CAN_TX_ID_STANDARD, &data, 1u);
    const CAN_TxFrame_t urgent = scheduler_frame(0x111u, CAN_TX_ID_STANDARD, &data, 1u);
    uint32_t normal_id = 0u;
    uint32_t urgent_id = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &normal, CAN_TX_PRIORITY_NORMAL,
                                100u, &normal_id) == CAN_TX_SUBMIT_ACCEPTED);

    tx_busy_mask = 1u;
    CANProcessTx();
    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(normal_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_QUEUED);

    tx_busy_mask = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &urgent, CAN_TX_PRIORITY_URGENT,
                                100u, &urgent_id) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(tx_id_history[0][0] == urgent.identifier);
    scheduler_complete_latest(&hfdcan1);
    CANProcessTx();
    TEST_CHECK(tx_id_history[0][1] == normal.identifier);
    scheduler_complete_latest(&hfdcan1);

    TEST_CHECK(CANGetTxRequestState(urgent_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);
    TEST_CHECK(CANGetTxRequestState(normal_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);
}

// 直接连接生产 CAN 队列，检查逻辑总线适配不会改变期限和结果。
static void test_platform_can_queue(void)
{
    CAN_Init_Config_s config = {0};
    config.can_handle = &hfdcan1; // 为测试总线启动生产 CAN 服务。
    config.tx_id = 0x321u;
    config.rx_id = 0x500u;
    config.can_module_callback = scheduler_rx_callback;
    TEST_CHECK(CANRegister(&config) != NULL);
    const uint8_t data = 0x11u;
    const CAN_TxFrame_t frame = scheduler_frame(0x321u, CAN_TX_ID_STANDARD, &data, 1u);
    uint32_t id = 99u;
    // 未绑定总线不能提交，也不能改写调用者保留的请求编号。
    TEST_CHECK(PlatformCanSubmitBatch(PLATFORM_CAN_INVALID, &data, &frame, 1u,
        CAN_TX_PRIORITY_NORMAL, 50u, &id) == CAN_TX_SUBMIT_NOT_READY && id == 99u);
    TEST_CHECK(PlatformCanProcessTxBus(PLATFORM_CAN_INVALID) == 0u);
    now_us = 115000u;
    TEST_CHECK(PlatformCanSubmitBatchUntil(PLATFORM_CAN_BUS_1, &data, &frame, 1u,
        CAN_TX_PRIORITY_NORMAL, 150u, &id) == CAN_TX_SUBMIT_ACCEPTED);
    now_us = 150000u; // 到达原始截止时刻后，队列必须过期而不能发送。
    TEST_CHECK(PlatformCanProcessTxBus(PLATFORM_CAN_BUS_1) == 1u);
    CAN_TxResult_t result;
    TEST_CHECK(PlatformCanGetRequest(id, &result) && result.state == CAN_TX_RESULT_EXPIRED);
    TEST_CHECK(tx_accept_count[0] == 0u);
    id = 0u;
    TEST_CHECK(PlatformCanSubmitBatch(PLATFORM_CAN_BUS_1, &data, &frame, 1u,
        CAN_TX_PRIORITY_NORMAL, 50u, &id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(PlatformCanCancelRequest(id) == 1u);
    TEST_CHECK(PlatformCanGetRequest(id, &result) && result.state == CAN_TX_RESULT_CANCELLED);
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    CANInitWorkspace();
    if (strcmp(argv[1], "platform-can") == 0)
        test_platform_can_queue();
    else if (strcmp(argv[1], "rollback") == 0)
        test_production_registration_rollback();
    else if (strcmp(argv[1], "routing") == 0)
        test_production_can_routing();
    else if (strcmp(argv[1], "feedforward") == 0)
        test_feedforward_paths_and_recovery();
    else if (strcmp(argv[1], "feedback_failure") == 0)
        test_feedback_failure_recovery();
    else if (strcmp(argv[1], "direction-angle") == 0)
        test_direction_flags(0u);
    else if (strcmp(argv[1], "direction-speed") == 0)
        test_direction_flags(1u);
    else if (strcmp(argv[1], "direction-current") == 0)
        test_current_loop_ignores_axis_direction_flags();
    else if (strcmp(argv[1], "direction-open") == 0)
        test_open_loop_keeps_motor_direction();
    else if (strcmp(argv[1], "direction-invalid") == 0)
        test_invalid_direction_flags_rejected();
    else if (strcmp(argv[1], "rx-burst") == 0)
        test_production_can_receive_burst();
    else if (strcmp(argv[1], "dji-rx-burst") == 0)
        test_production_dji_receive_burst();
    else if (strcmp(argv[1], "tx-arguments-copy") == 0)
        test_can_tx_arguments_and_copy();
    else if (strcmp(argv[1], "tx-queue-batch-buses") == 0)
        test_can_tx_queue_batch_and_buses();
    else if (strcmp(argv[1], "tx-topic-group") == 0)
        test_can_tx_topic_group();
    else if (strcmp(argv[1], "tx-topic-interrupt") == 0)
        test_can_tx_topic_interrupt();
    else if (strcmp(argv[1], "tx-topic-rx") == 0)
        test_can_tx_topic_rx();
    else if (strcmp(argv[1], "tx-topic-cancel") == 0)
        test_can_tx_topic_cancel();
    else if (strcmp(argv[1], "tx-topic-contention") == 0)
        test_can_tx_topic_contention();
    else if (strcmp(argv[1], "tx-topic-drain") == 0)
        test_can_tx_topic_drain();
    else if (strcmp(argv[1], "tx-topic-sync-budget") == 0)
        test_can_tx_topic_sync_budget();
    else if (strcmp(argv[1], "tx-topic-absolute-deadline") == 0)
        test_can_tx_topic_absolute_deadline();
    else if (strcmp(argv[1], "tx-topic-shared") == 0)
        test_can_tx_topic_shared();
    else if (strcmp(argv[1], "tx-topic-pending") == 0)
        test_can_tx_topic_pending();
    else if (strcmp(argv[1], "tx-periodic-budget") == 0)
        test_can_tx_periodic_latest_and_budget();
    else if (strcmp(argv[1], "tx-periodic-guarded-revoke") == 0)
        test_can_tx_periodic_guard_after_rejected_replacement();
    else if (strcmp(argv[1], "tx-expiry-cancel-wrap") == 0)
        test_can_tx_expiry_cancel_and_wrap();
    else if (strcmp(argv[1], "tx-budget-large-arithmetic") == 0)
        test_can_tx_budget_large_arithmetic();
    else if (strcmp(argv[1], "tx-retry-abort") == 0)
        test_can_tx_retry_boundary_and_abort();
    else if (strcmp(argv[1], "tx-retry-priority") == 0)
        test_can_tx_retry_yields_for_priority();
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
