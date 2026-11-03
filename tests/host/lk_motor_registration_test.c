/**
 * @file lk_motor_registration_test.c
 * @brief 回归 LK 注册回退及三条总线组帧；CAN 只在本机记录，不连接设备。
 * @note 每个场景由独立进程运行，避免依赖生产注册表清理接口。
 */
#include "LK9025.h"
#include "bsp_can.h" // 本测试直接检查底层注册替身接收的参数。
#include "bsp_dwt.h"
#include "test_check.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1 = {.bus_index = 0u};
FDCAN_HandleTypeDef hfdcan2 = {.bus_index = 1u};
FDCAN_HandleTypeDef hfdcan3 = {.bus_index = 2u};

typedef struct
{
    uint8_t bus;
    uint32_t tx_id;
    uint32_t rx_id;
    uint32_t header_id;
    uint8_t frame[8];
    uint8_t accepted;
} Submission_t;

static CANInstance can_instances[CAN_MX_REGISTER_CNT];
static unsigned can_attempts;
static unsigned can_registered;
static unsigned can_transmit_calls;
static uint8_t can_fail_registration;
static uint8_t can_fail_bus_mask;
static Submission_t submissions[64];
static unsigned submission_count;
static uint64_t now_us;

#if defined(LK_TEST_ALLOC_WRAP)
static unsigned malloc_calls;
static unsigned free_calls;
static unsigned malloc_fail_call;
void *__real_malloc(size_t size);
void *__real_calloc(size_t count, size_t size);
void __real_free(void *pointer);

void *__wrap_malloc(size_t size)
{
    ++malloc_calls;
    if (malloc_fail_call != 0u && malloc_calls == malloc_fail_call)
        return NULL;
    return __real_malloc(size);
}

void *__wrap_calloc(size_t count, size_t size)
{
    ++malloc_calls;
    if (malloc_fail_call != 0u && malloc_calls == malloc_fail_call)
        return NULL;
    return __real_calloc(count, size);
}

void __wrap_free(void *pointer)
{
    if (pointer != NULL)
        ++free_calls;
    __real_free(pointer);
}
#endif

uint64_t DWT_GetTimeline_us(void)
{
    return now_us;
}

uint32_t HAL_GetTick(void)
{
    return (uint32_t)(now_us / 1000u);
}

float DWT_GetDeltaT(uint32_t *counter)
{
    ++*counter;
    return 0.001f;
}

float PlatformProbeDeltaSeconds(uint32_t *counter)
{
    return DWT_GetDeltaT(counter); // 测试继续使用原反馈间隔替身。
}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    ++can_attempts;
    if (can_fail_registration)
    {
        can_fail_registration = 0u;
        return NULL;
    }
    if (config == NULL || can_registered >= CAN_MX_REGISTER_CNT)
        return NULL;

    CANInstance *instance = &can_instances[can_registered++];
    memset(instance, 0, sizeof(*instance));
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->txconf.Identifier = config->tx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;
    return instance;
}

uint8_t PlatformCanBusIsBound(PlatformCanBus_t bus)
{
    return (uint8_t)(bus >= PLATFORM_CAN_BUS_1 && bus <= PLATFORM_CAN_BUS_3);
}

uint8_t PlatformCanRxRegister(PlatformCanRxPort_t *port, const PlatformCanEndpoint_t *endpoint,
                              PlatformCanRxCallback_t callback, void *context)
{
    FDCAN_HandleTypeDef *handles[] = {&hfdcan1, &hfdcan2, &hfdcan3};
    if (port == NULL || endpoint == NULL || !PlatformCanBusIsBound(endpoint->can_bus)) return 0u;
    CAN_Init_Config_s config = {0};
    config.can_handle = handles[endpoint->can_bus - PLATFORM_CAN_BUS_1]; // 按逻辑编号选择替身。
    config.tx_id = endpoint->tx_id;
    config.rx_id = endpoint->rx_id;
    config.id = port;
    CANInstance *instance = CANRegister(&config); // 保留原有注册失败注入。
    if (instance == NULL) return 0u;
    port->backend = instance;
    port->callback = callback;
    port->context = context;
    return 1u;
}

CAN_TxSubmitStatus_t CANSubmitPeriodic(FDCAN_HandleTypeDef *hcan, const void *source,
                                       uint32_t key, const CAN_TxFrame_t *frame,
                                       CAN_TxPriority_t priority, uint32_t deadline_ms,
                                       uint32_t period_us, uint32_t *request_id)
{
    (void)source; (void)key; (void)priority; (void)deadline_ms; (void)period_us;
    TEST_CHECK(hcan != NULL && frame != NULL && request_id != NULL && frame->length <= 8u);
    TEST_CHECK(submission_count < sizeof(submissions) / sizeof(submissions[0]));
    Submission_t *submission = &submissions[submission_count++];
    submission->bus = hcan->bus_index;
    submission->tx_id = frame->identifier;
    submission->rx_id = 0u;
    submission->header_id = frame->identifier;
    memset(submission->frame, 0, sizeof(submission->frame));
    memcpy(submission->frame, frame->data, frame->length);
    submission->accepted = (uint8_t)((can_fail_bus_mask & (1u << submission->bus)) == 0u);
    ++can_transmit_calls;
    *request_id = submission_count;
    return submission->accepted != 0u ? CAN_TX_SUBMIT_ACCEPTED : CAN_TX_SUBMIT_FULL;
}

CAN_TxSubmitStatus_t PlatformCanSubmitPeriodic(PlatformCanBus_t bus, const void *source,
    uint32_t key, const CAN_TxFrame_t *frame, CAN_TxPriority_t priority, uint32_t deadline_ms,
    uint32_t period_us, CAN_TxPeriodicPrepareFn_t prepare, uint32_t *request_id)
{
    FDCAN_HandleTypeDef *handles[] = {&hfdcan1, &hfdcan2, &hfdcan3};
    TEST_CHECK(PlatformCanBusIsBound(bus) && prepare == NULL);
    return CANSubmitPeriodic(handles[bus - PLATFORM_CAN_BUS_1], source, key, frame,
                             priority, deadline_ms, period_us, request_id);
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

static void expect_empty_bus_frame(const Submission_t *submission)
{
    static const uint8_t empty[8] = {0};
    TEST_CHECK(memcmp(submission->frame, empty, sizeof(empty)) == 0);
}

static void test_registration(void)
{
    Motor_Init_Config_s config = config_for(0u, 1u);
    Motor_Init_Config_s before;
    TEST_CHECK(LKMotorInit(NULL) == NULL);
#if defined(LK_TEST_ALLOC_WRAP)
    unsigned alloc_before = malloc_calls;
    unsigned free_before = free_calls;
#endif
    config.can_init_config.can_bus = PLATFORM_CAN_INVALID;
    TEST_CHECK(LKMotorInit(&config) == NULL);
    config = config_for(0u, 0u);
    TEST_CHECK(LKMotorInit(&config) == NULL);
    config.can_init_config.tx_id = 5u;
    TEST_CHECK(LKMotorInit(&config) == NULL);
    TEST_CHECK(can_attempts == 0u && can_registered == 0u);
#if defined(LK_TEST_ALLOC_WRAP)
    TEST_CHECK(malloc_calls == alloc_before && free_calls == free_before);

    // 第一次分配用于电机对象，第二次用于生产 Daemon 对象。
    malloc_fail_call = malloc_calls + 1u;
    config = config_for(0u, 1u);
    TEST_CHECK(LKMotorInit(&config) == NULL);
    TEST_CHECK(malloc_calls == alloc_before + 1u && free_calls == free_before);

    malloc_fail_call = malloc_calls + 2u;
    TEST_CHECK(LKMotorInit(&config) == NULL);
    TEST_CHECK(malloc_calls == alloc_before + 3u && free_calls == free_before + 1u);
    TEST_CHECK(can_attempts == 0u && can_registered == 0u);
    malloc_fail_call = 0u;
#endif

    config = config_for(0u, 1u);
    config.controller_setting_init_config.close_loop_type = CURRENT_LOOP;
    config.controller_param_init_config.current_PID.MaxOut = -1.0f;
#if defined(LK_TEST_ALLOC_WRAP)
    alloc_before = malloc_calls;
    free_before = free_calls;
#endif
    TEST_CHECK(LKMotorInit(&config) == NULL);
    TEST_CHECK(can_attempts == 0u && can_registered == 0u);
#if defined(LK_TEST_ALLOC_WRAP)
    TEST_CHECK(malloc_calls == alloc_before && free_calls == free_before);
#endif

    config = config_for(0u, 1u);
    config.motor_type = MOTOR_TYPE_NONE;
    TEST_CHECK(LKMotorInit(&config) == NULL);
#if defined(LK_TEST_ALLOC_WRAP)
    TEST_CHECK(malloc_calls == alloc_before && free_calls == free_before);
#endif

    config = config_for(0u, 1u);
    before = config;
#if defined(LK_TEST_ALLOC_WRAP)
    alloc_before = malloc_calls;
    free_before = free_calls;
#endif
    LKMotorInstance *motor = LKMotorInit(&config);
    TEST_CHECK(motor != NULL);
#if defined(LK_TEST_ALLOC_WRAP)
    TEST_CHECK(malloc_calls == alloc_before + 2u && free_calls == free_before);
#endif
    TEST_CHECK(memcmp(&config, &before, sizeof(config)) == 0);
    TEST_CHECK(motor->motor_id == 1u && motor->can_bus_index == 0u);
    TEST_CHECK(can_registered == 1u && can_attempts == 1u);
    TEST_CHECK(((CANInstance *)motor->can_rx.backend)->tx_id == 0x280u);
    TEST_CHECK(((CANInstance *)motor->can_rx.backend)->txconf.Identifier == 0x280u);
    TEST_CHECK(((CANInstance *)motor->can_rx.backend)->rx_id == 0x141u);
    TEST_CHECK(((CANInstance *)motor->can_rx.backend)->id == &motor->can_rx);
    TEST_CHECK(motor->stop_flag == MOTOR_ENALBED);

    // 同总线同编号会在资源申请前拒绝，已有 CAN 和 Daemon 实例保持不变。
    config = config_for(0u, 1u);
#if defined(LK_TEST_ALLOC_WRAP)
    alloc_before = malloc_calls;
    free_before = free_calls;
#endif
    TEST_CHECK(LKMotorInit(&config) == NULL);
#if defined(LK_TEST_ALLOC_WRAP)
    TEST_CHECK(malloc_calls == alloc_before && free_calls == free_before);
#endif
    TEST_CHECK(can_registered == 1u && can_attempts == 1u);
}

static void test_can_failure_rollback(void)
{
    Motor_Init_Config_s config = config_for(1u, 3u);
#if defined(LK_TEST_ALLOC_WRAP)
    unsigned malloc_before = malloc_calls;
    unsigned free_before = free_calls;
#endif
    can_fail_registration = 1u;
    TEST_CHECK(LKMotorInit(&config) == NULL);
    TEST_CHECK(can_attempts == 1u && can_registered == 0u);
#if defined(LK_TEST_ALLOC_WRAP)
    TEST_CHECK(malloc_calls == malloc_before + 2u);
    TEST_CHECK(free_calls == free_before + 2u);
#endif

    // CAN 拒绝后 Daemon 和对象都已释放；相同编号能立即重新登记。
    config = config_for(1u, 3u);
    LKMotorInstance *motor = LKMotorInit(&config);
    TEST_CHECK(motor != NULL && motor->motor_id == 3u);
    TEST_CHECK(can_registered == 1u && can_attempts == 2u);
    TEST_CHECK(((CANInstance *)motor->can_rx.backend)->txconf.Identifier == 0x280u);
    TEST_CHECK(((CANInstance *)motor->can_rx.backend)->rx_id == 0x143u);
}

static void set_expected(uint8_t *frame, uint8_t motor_id, int16_t value)
{
    uint16_t wire_value = (uint16_t)value;
    uint8_t offset = (uint8_t)(motor_id - 1u) * 2u;
    frame[offset] = (uint8_t)(wire_value & 0xFFu);
    frame[offset + 1u] = (uint8_t)(wire_value >> 8);
}

static void test_routing(void)
{
    const uint8_t registration_order[LK_MOTOR_PER_BUS_CNT] = {4u, 2u, 3u, 1u};
    LKMotorInstance *motors[DEVICE_CAN_CNT][LK_MOTOR_PER_BUS_CNT] = {{NULL}};
    uint8_t expected[DEVICE_CAN_CNT][8] = {{0}};

    // 先在每条总线上登记4号，再交错登记其余编号，验证首实例不会决定其他总线去向。
    for (uint8_t order = 0u; order < LK_MOTOR_PER_BUS_CNT; ++order)
    {
        for (uint8_t bus = (uint8_t)(DEVICE_CAN_CNT - 1u);; --bus)
        {
            uint8_t motor_id = registration_order[order];
            Motor_Init_Config_s config = config_for(bus, motor_id);
            motors[bus][motor_id - 1u] = LKMotorInit(&config);
            TEST_CHECK(motors[bus][motor_id - 1u] != NULL);
            TEST_CHECK(((CANInstance *)motors[bus][motor_id - 1u]->can_rx.backend)->tx_id == 0x280u);
            TEST_CHECK(((CANInstance *)motors[bus][motor_id - 1u]->can_rx.backend)->rx_id == 0x140u + motor_id);
            if (bus == 0u)
                break;
        }
    }
    TEST_CHECK(can_registered == LK_MOTOR_MX_CNT);
    TEST_CHECK(can_attempts == LK_MOTOR_MX_CNT);

    // 首次只建立各 PID 时钟，按现有规则输出零；之后用1ms有效间隔检查实际槽位。
    now_us = 0u;
    LKMotorControl();
    TEST_CHECK(submission_count == DEVICE_CAN_CNT);
    for (unsigned i = 0u; i < submission_count; ++i)
        expect_empty_bus_frame(&submissions[i]);

    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        for (uint8_t motor_id = 1u; motor_id <= LK_MOTOR_PER_BUS_CNT; ++motor_id)
        {
            int16_t ref = (motor_id & 1u) ? (int16_t)(100 * bus + 100 * motor_id)
                                         : (int16_t)(-100 * bus - 100 * motor_id);
            LKMotorSetRef(motors[bus][motor_id - 1u], (float)ref);
            set_expected(expected[bus], motor_id, ref);
        }
    }

    now_us = 1000u;
    submission_count = 0u;
    LKMotorControl();
    TEST_CHECK(submission_count == DEVICE_CAN_CNT);
    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        const Submission_t *submission = &submissions[bus];
        TEST_CHECK(submission->bus == bus);
        TEST_CHECK(submission->tx_id == 0x280u && submission->header_id == 0x280u);
        TEST_CHECK(memcmp(submission->frame, expected[bus], sizeof(expected[bus])) == 0);
    }

    // 单条总线提交失败不挡住其他总线；下轮改值后提交新数据，不重放失败帧。
    can_fail_bus_mask = 1u << 1;
    now_us += 1000u;
    submission_count = 0u;
    LKMotorControl();
    TEST_CHECK(submission_count == DEVICE_CAN_CNT);
    TEST_CHECK(submissions[0].accepted == 1u && submissions[1].accepted == 0u &&
               submissions[2].accepted == 1u);
    LKMotorSetRef(motors[1][2], 777.0f);
    set_expected(expected[1], 3u, 777);
    can_fail_bus_mask = 0u;
    now_us += 1000u;
    submission_count = 0u;
    LKMotorControl();
    TEST_CHECK(submission_count == DEVICE_CAN_CNT);
    TEST_CHECK(memcmp(submissions[1].frame, expected[1], sizeof(expected[1])) == 0);

    // 停止只清对应编号的两字节，组内其他目标仍正常发送。
    LKMotorStop(motors[0][1]);
    expected[0][2] = 0u;
    expected[0][3] = 0u;
    now_us += 1000u;
    submission_count = 0u;
    LKMotorControl();
    TEST_CHECK(memcmp(submissions[0].frame, expected[0], sizeof(expected[0])) == 0);
    TEST_CHECK(submissions[0].frame[0] != 0u || submissions[0].frame[1] != 0u);

    // 非有限目标触发本轮计算失败，只清该编号槽位，不污染同总线其他输出。
    LKMotorSetRef(motors[2][0], INFINITY);
    expected[2][0] = 0u;
    expected[2][1] = 0u;
    now_us += 1000u;
    submission_count = 0u;
    LKMotorControl();
    TEST_CHECK(submission_count == DEVICE_CAN_CNT);
    TEST_CHECK(memcmp(submissions[2].frame, expected[2], sizeof(expected[2])) == 0);
    TEST_CHECK(submissions[2].frame[2] != 0u || submissions[2].frame[3] != 0u);

    // 有限但超出 int16_t 的命令不能直接窄化，保持对应槽位为零。
    LKMotorSetRef(motors[0][0], 32768.0f);
    expected[0][0] = 0u;
    expected[0][1] = 0u;
    now_us += 1000u;
    submission_count = 0u;
    LKMotorControl();
    TEST_CHECK(memcmp(submissions[0].frame, expected[0], sizeof(expected[0])) == 0);
    TEST_CHECK(motors[0][0]->pid_clock.last_status == PID_NUMERIC_ERROR);

    LKMotorSetRef(motors[0][2], -32769.0f);
    expected[0][4] = 0u;
    expected[0][5] = 0u;
    now_us += 1000u;
    submission_count = 0u;
    LKMotorControl();
    TEST_CHECK(memcmp(submissions[0].frame, expected[0], sizeof(expected[0])) == 0);
    TEST_CHECK(motors[0][2]->pid_clock.last_status == PID_NUMERIC_ERROR);
}

static void test_sparse_routing(void)
{
    Motor_Init_Config_s config = config_for(0u, 4u);
    LKMotorInstance *bus0_id4 = LKMotorInit(&config);
    TEST_CHECK(bus0_id4 != NULL);
    config = config_for(1u, 4u);
    LKMotorInstance *bus1_id4 = LKMotorInit(&config);
    TEST_CHECK(bus1_id4 != NULL);
    config = config_for(0u, 2u);
    LKMotorInstance *bus0_id2 = LKMotorInit(&config);
    TEST_CHECK(bus0_id2 != NULL);
    config = config_for(0u, 1u);
    config.output_axis.direction = MOTOR_AXIS_DIRECTION_REVERSE;
    LKMotorInstance *bus0_id1_reverse = LKMotorInit(&config);
    TEST_CHECK(bus0_id1_reverse != NULL);

    LKMotorSetRef(bus0_id4, 444.0f);
    LKMotorSetRef(bus0_id2, -222.0f);
    LKMotorSetRef(bus0_id1_reverse, 32768.0f);
    LKMotorSetRef(bus1_id4, -444.0f);
    now_us = 0u;
    LKMotorControl();
    TEST_CHECK(submission_count == 2u);

    now_us = 1000u;
    submission_count = 0u;
    LKMotorControl();
    TEST_CHECK(submission_count == 2u);
    static const uint8_t expected_bus0[8] = {0u, 0u, 0x22u, 0xFFu, 0u, 0u, 0xBCu, 0x01u};
    static const uint8_t expected_bus1[8] = {0u, 0u, 0u, 0u, 0u, 0u, 0x44u, 0xFEu};
    TEST_CHECK(submissions[0].bus == 0u && memcmp(submissions[0].frame, expected_bus0, 8u) == 0);
    TEST_CHECK(submissions[1].bus == 1u && memcmp(submissions[1].frame, expected_bus1, 8u) == 0);
    TEST_CHECK(bus0_id1_reverse->pid_clock.last_status == PID_NUMERIC_ERROR);
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "registration") == 0)
        test_registration();
    else if (strcmp(argv[1], "can_failure") == 0)
        test_can_failure_rollback();
    else if (strcmp(argv[1], "sparse") == 0)
        test_sparse_routing();
    else
    {
        TEST_CHECK(strcmp(argv[1], "routing") == 0);
        test_routing();
    }
    return EXIT_SUCCESS;
}
