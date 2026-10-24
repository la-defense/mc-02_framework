/**
 * @file dm_motor_test.c
 * @brief 用生产 DM、CAN 与 Daemon 验证注册回退、任务命令和力矩报文。
 * @note HAL、RTOS 和时间由主机替身提供；不连接或驱动真实电机。
 */
#include "dmmotor.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
#include "test_check.h"
#include <float.h>
#include <math.h>
#include <setjmp.h>
#include <stdlib.h>
#include <string.h>
#if defined(DM_TEST_THREADS)
#include <pthread.h>
#include <sched.h>
#endif

static FDCAN_GlobalTypeDef fdcan_registers[DEVICE_CAN_CNT];
FDCAN_HandleTypeDef hfdcan1 = {.Init = {.StdFiltersNbr = 14u}, .Instance = &fdcan_registers[0]};
FDCAN_HandleTypeDef hfdcan2 = {.Init = {.StdFiltersNbr = 14u}, .Instance = &fdcan_registers[1]};
FDCAN_HandleTypeDef hfdcan3 = {.Init = {.StdFiltersNbr = 14u}, .Instance = &fdcan_registers[2]};
volatile uint32_t dwt_cpu_freq_mhz = 480u;
volatile uint32_t test_dwt_cycles = 0u;

typedef struct
{
    const osThreadDef_t *definition;
    void *argument;
    uint8_t terminated;
} TestThread;

static TestThread threads[DM_MOTOR_CNT];
static unsigned thread_create_calls;
static unsigned fail_thread_create_call;
static unsigned fail_thread_terminate_call;
static unsigned thread_terminate_calls;
static unsigned task_iterations;
static uint32_t tick_ms;
static uint8_t fail_filter_once;
static uint8_t fail_tx_once;
static uint8_t busy_tx;
static unsigned tx_calls;
static unsigned tx_accepts;
static DMMotorInstance *stop_during_tx;
static uint32_t sent_id[128];
static uint8_t sent_frame[128][8];
static jmp_buf task_exit;
#if defined(DM_TEST_THREADS)
static pthread_mutex_t critical_mutex = PTHREAD_MUTEX_INITIALIZER;
static _Thread_local unsigned critical_depth;
#else
static unsigned critical_depth;
#endif

#if defined(DM_TEST_ALLOC_WRAP)
static unsigned allocation_calls;
static unsigned free_calls;
static unsigned fail_allocation_call;
void *__real_malloc(size_t size);
void __real_free(void *pointer);
void *__wrap_malloc(size_t size)
{
    ++allocation_calls;
    if (allocation_calls == fail_allocation_call)
        return NULL;
    return __real_malloc(size);
}
void __wrap_free(void *pointer)
{
    if (pointer != NULL)
        ++free_calls;
    __real_free(pointer);
}
#endif

static uint8_t bus_index(const FDCAN_HandleTypeDef *handle)
{
    if (handle == &hfdcan1) return 0u;
    if (handle == &hfdcan2) return 1u;
    if (handle == &hfdcan3) return 2u;
    return UINT8_MAX;
}

uint64_t DWT_GetTimeline_us(void) { return (uint64_t)tick_ms * 1000u; }
float DWT_GetDeltaT(uint32_t *counter) { (void)counter; return 0.002f; }
uint32_t HAL_GetTick(void) { return tick_ms; }

uint32_t HAL_FDCAN_ConfigRxFifoOverwrite(FDCAN_HandleTypeDef *handle, uint32_t fifo, uint32_t mode)
{ (void)handle; (void)fifo; (void)mode; return HAL_OK; }
uint32_t HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *handle, uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{ (void)handle; (void)a; (void)b; (void)c; (void)d; return HAL_OK; }
uint32_t HAL_FDCAN_Start(FDCAN_HandleTypeDef *handle)
{ handle->State = HAL_FDCAN_STATE_BUSY; handle->Instance->CCCR &= ~FDCAN_CCCR_INIT; return HAL_OK; }
uint32_t HAL_FDCAN_Stop(FDCAN_HandleTypeDef *handle)
{ handle->State = HAL_FDCAN_STATE_READY; handle->Instance->CCCR |= FDCAN_CCCR_INIT; return HAL_OK; }
uint32_t HAL_FDCAN_Init(FDCAN_HandleTypeDef *handle)
{ handle->State = HAL_FDCAN_STATE_READY; handle->Instance->CCCR |= FDCAN_CCCR_INIT; return HAL_OK; }
uint32_t HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *handle, uint32_t mask, uint32_t indexes)
{ (void)indexes; handle->Instance->IE = mask; return HAL_OK; }
uint32_t HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *handle, FDCAN_FilterTypeDef *filter)
{
    (void)filter;
    if (bus_index(handle) >= DEVICE_CAN_CNT || fail_filter_once != 0u)
    { fail_filter_once = 0u; return HAL_ERROR; }
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *handle)
{ return bus_index(handle) < DEVICE_CAN_CNT && busy_tx == 0u ? 1u : 0u; }
uint32_t HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *handle,
                                       FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    ++tx_calls;
    if (stop_during_tx != NULL)
    {
        DMMotorInstance *motor = stop_during_tx;
        stop_during_tx = NULL;
        TEST_CHECK(DMMotorStop(motor) == DMMOTOR_STATUS_OK);
    }
    if (bus_index(handle) >= DEVICE_CAN_CNT || fail_tx_once != 0u)
    { fail_tx_once = 0u; return HAL_ERROR; }
    if (tx_accepts < (sizeof(sent_frame) / sizeof(sent_frame[0])))
    {
        sent_id[tx_accepts] = header->Identifier;
        memcpy(sent_frame[tx_accepts], data, sizeof(sent_frame[tx_accepts]));
    }
    ++tx_accepts;
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *handle, FDCAN_ProtocolStatusTypeDef *status)
{ (void)handle; memset(status, 0, sizeof(*status)); return HAL_OK; }
uint32_t HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *handle, FDCAN_ErrorCountersTypeDef *counters)
{ (void)handle; memset(counters, 0, sizeof(*counters)); return HAL_OK; }
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *handle, uint32_t fifo)
{ (void)handle; (void)fifo; return 0u; }
uint32_t HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *handle, uint32_t fifo,
                                FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{ (void)handle; (void)fifo; (void)header; (void)data; return HAL_ERROR; }

void dm_test_critical_enter(void)
{
#if defined(DM_TEST_THREADS)
    if (critical_depth == 0u)
        TEST_CHECK(pthread_mutex_lock(&critical_mutex) == 0);
#endif
    ++critical_depth;
}
void dm_test_critical_exit(void)
{
    TEST_CHECK(critical_depth != 0u);
    --critical_depth;
#if defined(DM_TEST_THREADS)
    if (critical_depth == 0u)
        TEST_CHECK(pthread_mutex_unlock(&critical_mutex) == 0);
#endif
}

osThreadId osThreadCreate(const osThreadDef_t *definition, void *argument)
{
    ++thread_create_calls;
    if (thread_create_calls == fail_thread_create_call || thread_create_calls > DM_MOTOR_CNT)
        return NULL;
    unsigned index = thread_create_calls - 1u;
    TEST_CHECK(definition != NULL && definition->entry != NULL);
    TEST_CHECK(definition->priority == osPriorityNormal && definition->stack_size == 128u);
    threads[index].definition = definition;
    threads[index].argument = argument;
    threads[index].terminated = 0u;
    return &threads[index];
}
osStatus osThreadTerminate(osThreadId thread_id)
{
    ++thread_terminate_calls;
    if (thread_terminate_calls == fail_thread_terminate_call || thread_id == NULL)
        return osErrorOS;
    ((TestThread *)thread_id)->terminated = 1u;
    return osOK;
}
void osDelay(uint32_t milliseconds)
{
    tick_ms += milliseconds;
#if defined(DM_TEST_THREADS)
    (void)sched_yield();
#endif
    if (task_iterations != 0u && --task_iterations == 0u)
        longjmp(task_exit, 1);
}

static Motor_Init_Config_s config_for(FDCAN_HandleTypeDef *handle, uint32_t tx_id, uint32_t rx_id)
{
    Motor_Init_Config_s config = {0};
    config.can_init_config.can_handle = handle;
    config.can_init_config.tx_id = tx_id;
    config.can_init_config.rx_id = rx_id;
    config.controller_setting_init_config.close_loop_type = OPEN_LOOP;
    config.controller_setting_init_config.outer_loop_type = OPEN_LOOP;
    config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    config.controller_setting_init_config.feedforward_flag = FEEDFORWARD_NONE;
    return config;
}
static DMMotorInstance *register_motor(FDCAN_HandleTypeDef *handle, uint32_t tx_id, uint32_t rx_id)
{ Motor_Init_Config_s config = config_for(handle, tx_id, rx_id); return DMMotorInit(&config); }

static void run_task(unsigned index, unsigned iterations)
{
    TEST_CHECK(index < thread_create_calls && iterations != 0u);
    TEST_CHECK(threads[index].definition != NULL && threads[index].terminated == 0u);
    task_iterations = iterations;
    if (setjmp(task_exit) == 0)
    { threads[index].definition->entry(threads[index].argument); TEST_CHECK(0 && "DMMotorTask returned"); }
    TEST_CHECK(task_iterations == 0u);
}
static uint16_t torque_field(const uint8_t frame[8])
{ return (uint16_t)(((uint16_t)(frame[6] & 0x0fu) << 8u) | frame[7]); }
static void expect_mode_frame(unsigned index, DMMotor_Mode_e command)
{
    static const uint8_t prefix[7] = {0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu};
    TEST_CHECK(index < tx_accepts && memcmp(sent_frame[index], prefix, 7u) == 0);
    TEST_CHECK(sent_frame[index][7] == (uint8_t)command);
}
static void expect_zero_control_frame(unsigned index)
{
    static const uint8_t expected[8] = {0x7fu, 0xffu, 0x7fu, 0xf0u, 0x00u, 0x00u, 0x07u, 0xffu};
    TEST_CHECK(index < tx_accepts && memcmp(sent_frame[index], expected, 8u) == 0);
}

static void test_init(void)
{
    TEST_CHECK(DMMotorControlInit() == DMMOTOR_STATUS_NOT_REGISTERED);
    DMMotorState_t unchanged = {.task_ready = 7u};
    TEST_CHECK(DMMotorGetState(NULL, &unchanged) == DMMOTOR_STATUS_INVALID_PARAMETER);
    TEST_CHECK(unchanged.task_ready == 7u);
    DMMotorInstance *unknown = (DMMotorInstance *)(uintptr_t)0x1234u;
    TEST_CHECK(DMMotorGetState(unknown, &unchanged) == DMMOTOR_STATUS_NOT_REGISTERED);
    TEST_CHECK(unchanged.task_ready == 7u);
    Motor_Init_Config_s config = config_for(&hfdcan1, 0x123u, 0x321u), original = config;
    DMMotorInstance *motor = DMMotorInit(&config);
    TEST_CHECK(motor != NULL && memcmp(&config, &original, sizeof(config)) == 0 && tx_calls == 0u);
    DMMotorState_t state = {0};
    TEST_CHECK(DMMotorGetState(motor, &state) == DMMOTOR_STATUS_OK);
    TEST_CHECK(state.task_ready == 0u && state.running == 0u && state.reference_available == 0u);
    TEST_CHECK(DMMotorControlInit() == DMMOTOR_STATUS_OK && thread_create_calls == 1u);
    TEST_CHECK(DMMotorControlInit() == DMMOTOR_STATUS_OK && thread_create_calls == 1u);
    TEST_CHECK(DMMotorInit(&config) == NULL);
}

static void test_config(void)
{
    FDCAN_HandleTypeDef unknown_handle = {0};
    Motor_Init_Config_s config = config_for(&hfdcan1, 0u, 0u);
    config.can_init_config.can_handle = NULL; TEST_CHECK(DMMotorInit(&config) == NULL);
    config = config_for(&unknown_handle, 1u, 1u); TEST_CHECK(DMMotorInit(&config) == NULL);
    config = config_for(&hfdcan1, 0x800u, 1u); TEST_CHECK(DMMotorInit(&config) == NULL);
    config = config_for(&hfdcan1, 1u, 0x800u); TEST_CHECK(DMMotorInit(&config) == NULL);
    config = config_for(&hfdcan1, 1u, 1u);
    config.controller_setting_init_config.motor_reverse_flag = (Motor_Reverse_Flag_e)2;
    TEST_CHECK(DMMotorInit(&config) == NULL);
    config = config_for(&hfdcan1, 1u, 1u);
    config.controller_setting_init_config.close_loop_type = SPEED_LOOP;
    TEST_CHECK(DMMotorInit(&config) == NULL);
    config = config_for(&hfdcan1, 1u, 1u);
    config.controller_setting_init_config.outer_loop_type = SPEED_LOOP;
    TEST_CHECK(DMMotorInit(&config) == NULL);
    config = config_for(&hfdcan1, 1u, 1u);
    config.controller_setting_init_config.feedforward_flag = SPEED_FEEDFORWARD;
    TEST_CHECK(DMMotorInit(&config) == NULL);
    TEST_CHECK(register_motor(&hfdcan1, 0x7ffu, 0u) != NULL);
    config = config_for(&hfdcan1, 0x100u, 0u); TEST_CHECK(DMMotorInit(&config) == NULL);
    config = config_for(&hfdcan1, 0x7ffu, 0x201u); TEST_CHECK(DMMotorInit(&config) == NULL);
    TEST_CHECK(register_motor(&hfdcan2, 0x7ffu, 0u) != NULL);
    TEST_CHECK(register_motor(&hfdcan3, 0x7ffu, 0u) != NULL);
    TEST_CHECK(register_motor(&hfdcan1, 0x102u, 0x202u) != NULL);
    TEST_CHECK(register_motor(&hfdcan1, 0x103u, 0x203u) == NULL);
    TEST_CHECK(DMMotorControlInit() == DMMOTOR_STATUS_OK);
}

static void test_task_rollback(void)
{
    TEST_CHECK(register_motor(&hfdcan1, 1u, 2u) != NULL);
    TEST_CHECK(register_motor(&hfdcan2, 3u, 4u) != NULL);
    fail_thread_create_call = 2u;
    TEST_CHECK(DMMotorControlInit() == DMMOTOR_STATUS_RESOURCE_ERROR);
    TEST_CHECK(thread_create_calls == 2u && thread_terminate_calls == 1u && threads[0].terminated != 0u);
    fail_thread_create_call = 0u; thread_create_calls = 0u;
    TEST_CHECK(DMMotorControlInit() == DMMOTOR_STATUS_OK);
    TEST_CHECK(DMMotorControlInit() == DMMOTOR_STATUS_OK && thread_create_calls == 2u);
    DMMotorState_t state = {0};
    TEST_CHECK(DMMotorGetState((DMMotorInstance *)threads[0].argument, &state) == DMMOTOR_STATUS_OK);
    TEST_CHECK(state.task_ready != 0u);
}

static void test_frames(void)
{
    Motor_Init_Config_s config = config_for(&hfdcan1, 0x123u, 0x223u);
    DMMotorInstance *motor = DMMotorInit(&config);
    DMMotorState_t state = {0};
    TEST_CHECK(motor != NULL && DMMotorControlInit() == DMMOTOR_STATUS_OK);
    TEST_CHECK(DMMotorSetRef(motor, 9.0f) == DMMOTOR_STATUS_OK);
    run_task(0u, 1u); TEST_CHECK(tx_calls == 0u);
    TEST_CHECK(DMMotorCaliEncoder(motor) == DMMOTOR_STATUS_QUEUED);
    TEST_CHECK(DMMotorGetState(motor, &state) == DMMOTOR_STATUS_OK);
    TEST_CHECK(state.command_type == DMMOTOR_COMMAND_ZERO_POSITION &&
               state.command_result == DMMOTOR_COMMAND_PENDING && tx_calls == 0u);
    run_task(0u, 1u); expect_mode_frame(0u, DM_CMD_ZERO_POSITION);
    TEST_CHECK(DMMotorGetState(motor, &state) == DMMOTOR_STATUS_OK);
    TEST_CHECK(state.running == 0u && state.protocol_enabled == 0u);
    TEST_CHECK(DMMotorEnable(motor) == DMMOTOR_STATUS_QUEUED);
    TEST_CHECK(DMMotorGetState(motor, &state) == DMMOTOR_STATUS_OK);
    TEST_CHECK(state.command_result == DMMOTOR_COMMAND_PENDING && tx_accepts == 1u);
    TEST_CHECK(DMMotorCaliEncoder(motor) == DMMOTOR_STATUS_BUSY);
    run_task(0u, 1u); TEST_CHECK(tx_accepts == 2u); expect_mode_frame(1u, DM_CMD_MOTOR_MODE);
    TEST_CHECK(DMMotorGetState(motor, &state) == DMMOTOR_STATUS_OK);
    TEST_CHECK(state.running != 0u && state.command_result == DMMOTOR_COMMAND_ACCEPTED);
    TEST_CHECK(state.reference_available == 0u);
#if defined(DM_TEST_ALLOC_WRAP)
    unsigned allocations_before_runtime = allocation_calls;
#endif
    TEST_CHECK(DMMotorSetRef(motor, 9.0f) == DMMOTOR_STATUS_OK);
    run_task(0u, 1u); TEST_CHECK(tx_accepts == 3u && torque_field(sent_frame[2]) == 0xbffu);
    TEST_CHECK(sent_id[2] == 0x123u);
    TEST_CHECK(DMMotorStop(motor) == DMMOTOR_STATUS_OK);
    run_task(0u, 1u); expect_zero_control_frame(3u);
    TEST_CHECK(DMMotorCaliEncoder(motor) == DMMOTOR_STATUS_QUEUED);
    run_task(0u, 1u); expect_mode_frame(4u, DM_CMD_ZERO_POSITION);
    TEST_CHECK(DMMotorGetState(motor, &state) == DMMOTOR_STATUS_OK);
    TEST_CHECK(state.running == 0u && state.protocol_enabled != 0u);
#if defined(DM_TEST_ALLOC_WRAP)
    TEST_CHECK(allocation_calls == allocations_before_runtime);
#endif
}

static void test_reverse(void)
{
    Motor_Init_Config_s config = config_for(&hfdcan1, 0x124u, 0x224u);
    config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    DMMotorInstance *motor = DMMotorInit(&config);
    TEST_CHECK(motor != NULL && DMMotorControlInit() == DMMOTOR_STATUS_OK);
    TEST_CHECK(DMMotorEnable(motor) == DMMOTOR_STATUS_QUEUED); run_task(0u, 1u);
    TEST_CHECK(DMMotorSetRef(motor, 9.0f) == DMMOTOR_STATUS_OK); run_task(0u, 1u);
    TEST_CHECK(torque_field(sent_frame[1]) == 0x3ffu);
    TEST_CHECK(DMMotorSetRef(motor, FLT_MAX) == DMMOTOR_STATUS_OK); run_task(0u, 1u);
    TEST_CHECK(torque_field(sent_frame[2]) == 0x000u);
    TEST_CHECK(DMMotorSetRef(motor, -FLT_MAX) == DMMOTOR_STATUS_OK); run_task(0u, 1u);
    TEST_CHECK(torque_field(sent_frame[3]) == 0xfffu);
    TEST_CHECK(DMMotorSetRef(motor, NAN) == DMMOTOR_STATUS_INVALID_VALUE); run_task(0u, 1u);
    expect_zero_control_frame(4u);
    TEST_CHECK(DMMotorSetRef(motor, INFINITY) == DMMOTOR_STATUS_INVALID_VALUE); run_task(0u, 1u);
    expect_zero_control_frame(5u);
    TEST_CHECK(DMMotorSetRef(motor, -0.0f) == DMMOTOR_STATUS_OK); run_task(0u, 1u);
    expect_zero_control_frame(6u);
}

static void test_normal_edges(void)
{
    DMMotorInstance *motor = register_motor(&hfdcan1, 0x128u, 0x228u);
    TEST_CHECK(motor != NULL && DMMotorControlInit() == DMMOTOR_STATUS_OK);
    TEST_CHECK(DMMotorEnable(motor) == DMMOTOR_STATUS_QUEUED);
    run_task(0u, 1u);
    const float references[] = {9.0f, -9.0f, DM_T_MIN, DM_T_MAX, 0.0f};
    const uint16_t expected[] = {0xbffu, 0x3ffu, 0x000u, 0xfffu, 0x7ffu};
    for (unsigned i = 0u; i < sizeof(references) / sizeof(references[0]); ++i)
    {
        TEST_CHECK(DMMotorSetRef(motor, references[i]) == DMMOTOR_STATUS_OK);
        run_task(0u, 1u);
        TEST_CHECK(torque_field(sent_frame[i + 1u]) == expected[i]);
    }
}

static void test_commands(void)
{
    DMMotorInstance *motor = register_motor(&hfdcan1, 0x125u, 0x225u);
    TEST_CHECK(motor != NULL && DMMotorControlInit() == DMMOTOR_STATUS_OK);
    TEST_CHECK(DMMotorEnable(motor) == DMMOTOR_STATUS_QUEUED);
    TEST_CHECK(DMMotorStop(motor) == DMMOTOR_STATUS_CANCELLED);
    run_task(0u, 1u); TEST_CHECK(tx_calls == 0u);
    fail_tx_once = 1u;
    TEST_CHECK(DMMotorEnable(motor) == DMMOTOR_STATUS_QUEUED); run_task(0u, 1u);
    DMMotorState_t state = {0};
    TEST_CHECK(DMMotorGetState(motor, &state) == DMMOTOR_STATUS_OK);
    TEST_CHECK(state.running == 0u && state.command_result == DMMOTOR_COMMAND_REJECTED);
    TEST_CHECK(tx_calls == 1u && tx_accepts == 0u);
    TEST_CHECK(DMMotorEnable(motor) == DMMOTOR_STATUS_QUEUED); run_task(0u, 1u);
    TEST_CHECK(tx_calls == 2u && tx_accepts == 1u); expect_mode_frame(0u, DM_CMD_MOTOR_MODE);
    TEST_CHECK(DMMotorSetRef(motor, 4.0f) == DMMOTOR_STATUS_OK);
    fail_tx_once = 1u;
    run_task(0u, 1u);
    TEST_CHECK(tx_calls == 3u && tx_accepts == 1u);
    run_task(0u, 1u);
    TEST_CHECK(tx_calls == 4u && tx_accepts == 2u);
    TEST_CHECK(torque_field(sent_frame[1]) == 0x9c6u);
}

static void test_instance_routing(void)
{
    DMMotorInstance *first = register_motor(&hfdcan1, 0x141u, 0x241u);
    DMMotorInstance *second = register_motor(&hfdcan3, 0x142u, 0x242u);
    TEST_CHECK(first != NULL && second != NULL && DMMotorControlInit() == DMMOTOR_STATUS_OK);
    TEST_CHECK(DMMotorEnable(first) == DMMOTOR_STATUS_QUEUED);
    TEST_CHECK(DMMotorEnable(second) == DMMOTOR_STATUS_QUEUED);
    run_task(1u, 1u);
    run_task(0u, 1u);
    expect_mode_frame(0u, DM_CMD_MOTOR_MODE);
    expect_mode_frame(1u, DM_CMD_MOTOR_MODE);
    TEST_CHECK(sent_id[0] == 0x142u && sent_id[1] == 0x141u);
    TEST_CHECK(DMMotorSetRef(first, 9.0f) == DMMOTOR_STATUS_OK);
    TEST_CHECK(DMMotorSetRef(second, -9.0f) == DMMOTOR_STATUS_OK);
    run_task(0u, 1u);
    run_task(1u, 1u);
    TEST_CHECK(sent_id[2] == 0x141u && torque_field(sent_frame[2]) == 0xbffu);
    TEST_CHECK(sent_id[3] == 0x142u && torque_field(sent_frame[3]) == 0x3ffu);
}

static void test_quarantine(void)
{
    DMMotorInstance *motor = register_motor(&hfdcan1, 0x126u, 0x226u);
    TEST_CHECK(motor != NULL && register_motor(&hfdcan2, 0x127u, 0x227u) != NULL);
    fail_thread_create_call = 2u; fail_thread_terminate_call = 1u;
    TEST_CHECK(DMMotorControlInit() == DMMOTOR_STATUS_RESOURCE_ERROR);
    DMMotorState_t state = {0};
    TEST_CHECK(DMMotorGetState(motor, &state) == DMMOTOR_STATUS_OK && state.task_ready == 0u);
    TEST_CHECK(DMMotorControlInit() == DMMOTOR_STATUS_RESOURCE_ERROR);
    TEST_CHECK(DMMotorEnable(motor) == DMMOTOR_STATUS_NOT_READY);
}

static void test_stop_during_submit(void)
{
    DMMotorInstance *motor = register_motor(&hfdcan1, 0x129u, 0x229u);
    TEST_CHECK(motor != NULL && DMMotorControlInit() == DMMOTOR_STATUS_OK);
    TEST_CHECK(DMMotorEnable(motor) == DMMOTOR_STATUS_QUEUED);
    stop_during_tx = motor;
    run_task(0u, 1u);
    DMMotorState_t state = {0};
    TEST_CHECK(DMMotorGetState(motor, &state) == DMMOTOR_STATUS_OK);
    TEST_CHECK(state.protocol_enabled != 0u && state.running == 0u);
    expect_mode_frame(0u, DM_CMD_MOTOR_MODE);
    run_task(0u, 1u);
    expect_zero_control_frame(1u);
}

#if defined(DM_TEST_THREADS)
typedef struct { DMMotorInstance *motor; } ConcurrentInput;
static void *run_task_thread(void *argument)
{
    (void)argument;
    task_iterations = 2000u;
    if (setjmp(task_exit) == 0)
        threads[0].definition->entry(threads[0].argument);
    TEST_CHECK(task_iterations == 0u);
    return NULL;
}
static void *run_reference_writer(void *argument)
{
    ConcurrentInput *input = (ConcurrentInput *)argument;
    for (unsigned i = 0u; i < 2000u; ++i)
    {
        DMMotorState_t state = {0};
        TEST_CHECK(DMMotorSetRef(input->motor, (float)((int)(i % 73u) - 36)) == DMMOTOR_STATUS_OK);
        TEST_CHECK(DMMotorGetState(input->motor, &state) == DMMOTOR_STATUS_OK);
        if ((i % 17u) == 0u)
        {
            (void)DMMotorStop(input->motor);
            TEST_CHECK(DMMotorEnable(input->motor) == DMMOTOR_STATUS_OK);
        }
    }
    return NULL;
}
static void test_threads(void)
{
    DMMotorInstance *motor = register_motor(&hfdcan1, 0x12au, 0x22au);
    TEST_CHECK(motor != NULL && DMMotorControlInit() == DMMOTOR_STATUS_OK);
    TEST_CHECK(DMMotorEnable(motor) == DMMOTOR_STATUS_QUEUED);
    run_task(0u, 1u);
    TEST_CHECK(DMMotorSetRef(motor, 1.0f) == DMMOTOR_STATUS_OK);
    ConcurrentInput input = {.motor = motor};
    pthread_t task_thread, writer_thread;
    TEST_CHECK(pthread_create(&task_thread, NULL, run_task_thread, NULL) == 0);
    TEST_CHECK(pthread_create(&writer_thread, NULL, run_reference_writer, &input) == 0);
    TEST_CHECK(pthread_join(task_thread, NULL) == 0);
    TEST_CHECK(pthread_join(writer_thread, NULL) == 0);
    DMMotorState_t state = {0};
    TEST_CHECK(DMMotorGetState(motor, &state) == DMMOTOR_STATUS_OK);
    TEST_CHECK(state.protocol_enabled != 0u);
}
#endif

static void test_resources(void)
{
    Motor_Init_Config_s config = config_for(&hfdcan1, 0x130u, 0x230u);
#if defined(DM_TEST_ALLOC_WRAP)
    unsigned free_base = free_calls;
    fail_allocation_call = allocation_calls + 1u; TEST_CHECK(DMMotorInit(&config) == NULL);
    TEST_CHECK(free_calls == free_base);
    fail_allocation_call = allocation_calls + 2u; TEST_CHECK(DMMotorInit(&config) == NULL);
    TEST_CHECK(free_calls == free_base + 1u);
    fail_allocation_call = allocation_calls + 3u; TEST_CHECK(DMMotorInit(&config) == NULL);
    TEST_CHECK(free_calls == free_base + 3u);
    fail_allocation_call = 0u;
#endif
    Motor_Init_Config_s original = config;
    fail_filter_once = 1u; TEST_CHECK(DMMotorInit(&config) == NULL);
    TEST_CHECK(memcmp(&config, &original, sizeof(config)) == 0);
    TEST_CHECK(register_motor(&hfdcan1, 0x130u, 0x230u) != NULL);
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "init") == 0) test_init();
    else if (strcmp(argv[1], "config") == 0) test_config();
    else if (strcmp(argv[1], "rollback") == 0) test_task_rollback();
    else if (strcmp(argv[1], "frames") == 0) test_frames();
    else if (strcmp(argv[1], "reverse") == 0) test_reverse();
    else if (strcmp(argv[1], "edges") == 0) test_normal_edges();
    else if (strcmp(argv[1], "commands") == 0) test_commands();
    else if (strcmp(argv[1], "instances") == 0) test_instance_routing();
    else if (strcmp(argv[1], "quarantine") == 0) test_quarantine();
    else if (strcmp(argv[1], "submit-stop") == 0) test_stop_during_submit();
    else if (strcmp(argv[1], "resources") == 0) test_resources();
#if defined(DM_TEST_THREADS)
    else if (strcmp(argv[1], "threads") == 0) test_threads();
#endif
    else TEST_CHECK(0 && "unknown scenario");
    TEST_CHECK(critical_depth == 0u);
    return EXIT_SUCCESS;
}
