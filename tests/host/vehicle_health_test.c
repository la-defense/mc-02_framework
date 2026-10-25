/**
 * @file vehicle_health_test.c
 * @brief 检查生产整车健康快照的周期、复制一致性和失败保持。
 * @note 通过内部来源替身控制模块状态与时钟；不连接硬件。
 */
#include "test_check.h"
#include "vehicle_health.h"
#include "vehicle_health_source.h"
#include "bsp_log.h"
#include <stdlib.h>
#include <string.h>
#if defined(VEHICLE_HEALTH_TEST_THREADS)
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdatomic.h>
#endif

static uint32_t now_ms;
static uint32_t primask;
static uint8_t in_isr;
static uint8_t capture_ok = 1u;
static VehicleHealthSourceSnapshot_t source;
static BSPLogHealthSnapshotHandler_t registered_health_handler;
static BSPLogStatus_t health_registration_status = BSP_LOG_STATUS_OK;
static uint32_t allocator_calls;
static uint8_t track_allocator_calls;
#if defined(VEHICLE_HEALTH_TEST_THREADS)
static pthread_mutex_t critical_mutex = PTHREAD_MUTEX_INITIALIZER;
static atomic_bool reader_done;
static atomic_uint reader_errors;
static atomic_uint reader_samples;
#endif

#if defined(__linux__)
void *__real_malloc(size_t size);
void *__real_calloc(size_t count, size_t size);
void *__real_realloc(void *pointer, size_t size);
void __real_free(void *pointer);

void *__wrap_malloc(size_t size)
{
    if (track_allocator_calls != 0u)
        ++allocator_calls;
    return __real_malloc(size);
}

void *__wrap_calloc(size_t count, size_t size)
{
    if (track_allocator_calls != 0u)
        ++allocator_calls;
    return __real_calloc(count, size);
}

void *__wrap_realloc(void *pointer, size_t size)
{
    if (track_allocator_calls != 0u)
        ++allocator_calls;
    return __real_realloc(pointer, size);
}

void __wrap_free(void *pointer)
{
    if (track_allocator_calls != 0u)
        ++allocator_calls;
    __real_free(pointer);
}
#endif

uint8_t VehicleHealthSourceCapture(VehicleHealthSourceSnapshot_t *output)
{
    if ((output == NULL) || (capture_ok == 0u))
        return 0u;
    *output = source;
    return 1u;
}

uint32_t VehicleHealthSourceNowMs(void) { return now_ms; }
uint8_t VehicleHealthSourceInIsr(void) { return in_isr; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1u; }
void __set_PRIMASK(uint32_t value) { primask = value; }
uint32_t HAL_GetTick(void) { return now_ms; }
BSPLogStatus_t BSPLogRegisterHealthSnapshotHandler(BSPLogHealthSnapshotHandler_t handler)
{
    if (handler == NULL)
        return BSP_LOG_STATUS_BAD_ARGUMENT;
    if (health_registration_status != BSP_LOG_STATUS_OK)
        return health_registration_status;
    registered_health_handler = handler;
    return BSP_LOG_STATUS_OK;
}
uint32_t VehicleHealthSourceEnterCritical(void)
{
#if defined(VEHICLE_HEALTH_TEST_THREADS)
    (void)pthread_mutex_lock(&critical_mutex);
#endif
    uint32_t previous = primask;
    primask = 1u;
    return previous;
}
void VehicleHealthSourceExitCritical(uint32_t previous)
{
    primask = previous;
#if defined(VEHICLE_HEALTH_TEST_THREADS)
    (void)pthread_mutex_unlock(&critical_mutex);
#endif
}

static void TestSnapshotPeriodAndFailure(void)
{
    VehicleHealthSnapshot_t snapshot;
    memset(&snapshot, 0xA5, sizeof(snapshot));
    now_ms = 0u;
    source = (VehicleHealthSourceSnapshot_t){0};
    source.role = VEHICLE_HEALTH_ROLE_SINGLE;
    source.enabled_modules = VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_IMU) |
                             VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_HEATER);
    source.available_modules = VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_IMU);
    source.degraded_modules = VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_HEATER);
    source.imu_valid = 1u;
    source.imu_sample_ms = 0u;
    source.imu_sequence = 7u;
    source.heater_initialized = 1u;
    source.heater_compiled_enabled = 0u;
    source.heater_sensor_valid = 1u;
    source.heater_duty = 0u;
    source.temperature_c = 39.5f;
    capture_ok = 1u;

    TEST_CHECK(VehicleHealthGetSnapshot(&snapshot) == VEHICLE_HEALTH_ERROR_NOT_INITIALIZED);
    TEST_CHECK(snapshot.sequence == UINT32_C(0xA5A5A5A5));
    TEST_CHECK(VehicleHealthInit() == VEHICLE_HEALTH_OK);
    TEST_CHECK(VehicleHealthInit() == VEHICLE_HEALTH_ALREADY_INITIALIZED);
    health_registration_status = BSP_LOG_STATUS_NOT_INITIALIZED;
    TEST_CHECK(VehicleHealthDiagnosticInit() == VEHICLE_HEALTH_ERROR_NOT_INITIALIZED);
    health_registration_status = BSP_LOG_STATUS_BAD_CONTEXT;
    TEST_CHECK(VehicleHealthDiagnosticInit() == VEHICLE_HEALTH_ERROR_CONTEXT);
    health_registration_status = BSP_LOG_STATUS_HANDLER_ALREADY_REGISTERED;
    TEST_CHECK(VehicleHealthDiagnosticInit() == VEHICLE_HEALTH_ALREADY_INITIALIZED);
    health_registration_status = BSP_LOG_STATUS_BAD_ARGUMENT;
    TEST_CHECK(VehicleHealthDiagnosticInit() == VEHICLE_HEALTH_ERROR_ARGUMENT);
    health_registration_status = BSP_LOG_STATUS_OK;
    TEST_CHECK(VehicleHealthDiagnosticInit() == VEHICLE_HEALTH_OK);
    health_registration_status = BSP_LOG_STATUS_HANDLER_ALREADY_REGISTERED;
    TEST_CHECK(VehicleHealthDiagnosticInit() == VEHICLE_HEALTH_ALREADY_INITIALIZED);
    health_registration_status = BSP_LOG_STATUS_OK;
    TEST_CHECK(registered_health_handler != NULL);
    uint8_t reply[BSP_LOG_DIAGNOSTIC_REPLY_PAYLOAD_MAX] = {0u};
    uint8_t reply_length = 0u;
    TEST_CHECK(registered_health_handler(reply, sizeof(reply), &reply_length) == 1u);
    TEST_CHECK(reply_length == 2u && reply[0] == VEHICLE_HEALTH_ERROR_NOT_READY &&
               reply[1] == VEHICLE_HEALTH_DIAGNOSTIC_SCHEMA_VERSION);
    track_allocator_calls = 1u;
    VehicleHealthStatus_t process_status = VehicleHealthProcess();
    track_allocator_calls = 0u;
    TEST_CHECK(process_status == VEHICLE_HEALTH_OK && allocator_calls == 0u);
    TEST_CHECK(VehicleHealthGetSnapshot(&snapshot) == VEHICLE_HEALTH_OK);
    TEST_CHECK(snapshot.sequence == 1u && snapshot.captured_ms == 0u);
    TEST_CHECK(snapshot.imu_valid == 1u && snapshot.imu_sample_ms == 0u);
    TEST_CHECK(snapshot.thermal_stable_known == 0u && snapshot.system_reset_quota_known == 0u);
    TEST_CHECK(snapshot.degraded_modules == source.degraded_modules);
    TEST_CHECK(snapshot.temperature_c == 39.5f);

    allocator_calls = 0u;
    track_allocator_calls = 1u;
    uint8_t health_reply_accepted = registered_health_handler(reply, sizeof(reply), &reply_length);
    track_allocator_calls = 0u;
    TEST_CHECK(health_reply_accepted == 1u && allocator_calls == 0u);
    TEST_CHECK(reply_length == 120u);
    TEST_CHECK(reply[0] == VEHICLE_HEALTH_OK &&
               reply[1] == VEHICLE_HEALTH_DIAGNOSTIC_SCHEMA_VERSION &&
               reply[2] == VEHICLE_HEALTH_DIAGNOSTIC_SCHEMA_VERSION);
    TEST_CHECK(reply[3] == 1u && reply[4] == 0u && reply[5] == 0u && reply[6] == 0u);
    TEST_CHECK(reply[2u + 99u] == 0u); /* 系统重启额度未知，不能误报为已知零额度。 */
    TEST_CHECK(reply[2u + 100u] == 0xFFu && reply[2u + 101u] == 0xFFu &&
               reply[2u + 102u] == 0xFFu && reply[2u + 103u] == 0xFFu);
    TEST_CHECK(reply[117] == 0x6Eu && reply[118] == 0x0Fu);
    TEST_CHECK(reply[119] == 0u); /* 118字节快照的最后一个字节是保留位。 */
    uint8_t unchanged[BSP_LOG_DIAGNOSTIC_REPLY_PAYLOAD_MAX];
    memset(unchanged, 0x5A, sizeof(unchanged));
    uint8_t sentinel_length = 0xCCu;
    allocator_calls = 0u;
    track_allocator_calls = 1u;
    uint8_t too_small_accepted = registered_health_handler(unchanged, 119u, &sentinel_length);
    track_allocator_calls = 0u;
    TEST_CHECK(too_small_accepted == 0u && allocator_calls == 0u);
    TEST_CHECK(sentinel_length == 0xCCu && unchanged[0] == 0x5Au && unchanged[118] == 0x5Au);

    source.imu_sequence = 8u;
    now_ms = VEHICLE_HEALTH_UPDATE_PERIOD_MS - 1u;
    TEST_CHECK(VehicleHealthProcess() == VEHICLE_HEALTH_NOT_DUE);
    TEST_CHECK(VehicleHealthGetSnapshot(&snapshot) == VEHICLE_HEALTH_OK);
    TEST_CHECK(snapshot.sequence == 1u && snapshot.imu_sequence == 7u);

    capture_ok = 0u;
    now_ms = VEHICLE_HEALTH_UPDATE_PERIOD_MS;
    TEST_CHECK(VehicleHealthProcess() == VEHICLE_HEALTH_ERROR_CAPTURE);
    TEST_CHECK(VehicleHealthGetSnapshot(&snapshot) == VEHICLE_HEALTH_OK);
    TEST_CHECK(snapshot.sequence == 1u && snapshot.imu_sequence == 7u);

    capture_ok = 1u;
    now_ms += VEHICLE_HEALTH_UPDATE_PERIOD_MS;
    source.imu_sequence = 9u;
    TEST_CHECK(VehicleHealthProcess() == VEHICLE_HEALTH_OK);
    TEST_CHECK(VehicleHealthGetSnapshot(&snapshot) == VEHICLE_HEALTH_OK);
    TEST_CHECK(snapshot.sequence == 2u && snapshot.imu_sequence == 9u);
}

static void TestContextAndEvents(void)
{
    VehicleHealthSnapshot_t snapshot = {0};
    InitDiagnosticEventSnapshot_t events = {0};
    in_isr = 1u;
    TEST_CHECK(VehicleHealthProcess() == VEHICLE_HEALTH_ERROR_CONTEXT);
    TEST_CHECK(VehicleHealthGetSnapshot(&snapshot) == VEHICLE_HEALTH_ERROR_CONTEXT);
    in_isr = 0u;
    TEST_CHECK(VehicleHealthGetSnapshot(NULL) == VEHICLE_HEALTH_ERROR_ARGUMENT);
    TEST_CHECK(VehicleHealthGetInitEvents(NULL) == VEHICLE_HEALTH_ERROR_ARGUMENT);
    track_allocator_calls = 1u;
    VehicleHealthStatus_t event_status = VehicleHealthGetInitEvents(&events);
    track_allocator_calls = 0u;
    TEST_CHECK(event_status == VEHICLE_HEALTH_OK && allocator_calls == 0u);
}

static void TestBuildProfile(void)
{
    VehicleHealthBuildProfile_t profile = {0};
    uint32_t expected = VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_INIT) |
                        VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_SAFETY) |
                        VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_TASK_MONITOR) |
                        VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_IMU) |
                        VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_HEATER) |
                        VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_LOG);
    TEST_CHECK(VehicleHealthGetBuildProfile(&profile) == VEHICLE_HEALTH_OK);
#if defined(MC02_PROFILE_BENCH_SAFE)
    TEST_CHECK(profile.role == VEHICLE_HEALTH_ROLE_BENCH);
    expected |= VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_VISION);
    expected |= VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_USB);
#elif defined(CHASSIS_BOARD)
    TEST_CHECK(profile.role == VEHICLE_HEALTH_ROLE_CHASSIS);
    expected |= VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_CAN1) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_CAN2) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_CAN3) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_CHASSIS) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_REFEREE);
#elif defined(GIMBAL_BOARD)
    TEST_CHECK(profile.role == VEHICLE_HEALTH_ROLE_GIMBAL);
    expected |= VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_CAN1) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_CAN2) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_CAN3) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_GIMBAL) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_SHOOTER) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_REMOTE) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_VISION) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_USB);
#else
    TEST_CHECK(profile.role == VEHICLE_HEALTH_ROLE_SINGLE);
    expected |= VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_CAN1) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_CAN2) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_CAN3) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_CHASSIS) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_REFEREE) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_GIMBAL) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_SHOOTER) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_REMOTE) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_VISION) |
                VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_USB);
#endif
    TEST_CHECK(profile.enabled_modules == expected);
    TEST_CHECK(VehicleHealthGetBuildProfile(NULL) == VEHICLE_HEALTH_ERROR_ARGUMENT);
}

#if defined(VEHICLE_HEALTH_TEST_THREADS)
static void *ReadPublishedSnapshots(void *context)
{
    (void)context;
    VehicleHealthSnapshot_t snapshot;
    while (!atomic_load_explicit(&reader_done, memory_order_acquire))
    {
        if ((VehicleHealthGetSnapshot(&snapshot) == VEHICLE_HEALTH_OK) &&
            ((uint32_t)snapshot.temperature_c != snapshot.imu_sequence ||
             snapshot.remote_last_valid_ms != snapshot.imu_sequence))
            (void)atomic_fetch_add_explicit(&reader_errors, 1u, memory_order_relaxed);
        else
            (void)atomic_fetch_add_explicit(&reader_samples, 1u, memory_order_relaxed);
    }
    return NULL;
}

static void TestConcurrentReaders(void)
{
    pthread_t reader;
    atomic_store(&reader_done, false);
    atomic_store(&reader_errors, 0u);
    atomic_store(&reader_samples, 0u);
    now_ms = UINT32_MAX - 20u;
    source.imu_sequence = 0u;
    source.remote_last_valid_ms = 0u;
    source.temperature_c = 0.0f;
    TEST_CHECK(VehicleHealthProcess() == VEHICLE_HEALTH_OK);
    now_ms += VEHICLE_HEALTH_UPDATE_PERIOD_MS;
    TEST_CHECK(VehicleHealthProcess() == VEHICLE_HEALTH_OK);
    VehicleHealthSnapshot_t wrapped;
    TEST_CHECK(VehicleHealthGetSnapshot(&wrapped) == VEHICLE_HEALTH_OK);
    TEST_CHECK(wrapped.captured_ms == VEHICLE_HEALTH_UPDATE_PERIOD_MS - 21u);
    now_ms += VEHICLE_HEALTH_UPDATE_PERIOD_MS;
    source.imu_sequence = 0u;
    source.remote_last_valid_ms = 0u;
    source.temperature_c = 0.0f;
    TEST_CHECK(VehicleHealthProcess() == VEHICLE_HEALTH_OK);
    TEST_CHECK(pthread_create(&reader, NULL, ReadPublishedSnapshots, NULL) == 0);
    for (uint32_t sequence = 1u; sequence <= 5000u; ++sequence)
    {
        now_ms += VEHICLE_HEALTH_UPDATE_PERIOD_MS;
        source.imu_sequence = sequence;
        source.remote_last_valid_ms = sequence;
        source.temperature_c = (float)sequence;
        TEST_CHECK(VehicleHealthProcess() == VEHICLE_HEALTH_OK);
        if ((sequence & 31u) == 0u)
            (void)sched_yield();
    }
    atomic_store_explicit(&reader_done, true, memory_order_release);
    TEST_CHECK(pthread_join(reader, NULL) == 0);
    TEST_CHECK(atomic_load(&reader_errors) == 0u);
    TEST_CHECK(atomic_load(&reader_samples) > 0u);
}
#endif

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    TestSnapshotPeriodAndFailure();
    TestContextAndEvents();
    TestBuildProfile();
#if defined(VEHICLE_HEALTH_TEST_THREADS)
    TestConcurrentReaders();
#endif
    return 0;
}
