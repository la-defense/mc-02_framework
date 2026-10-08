#include "imu_heater.h"
#include "main.h"
#include "tim.h"
#include "bsp_adc.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ASSERT_TRUE(condition, message) \
    do { if (!(condition)) { fprintf(stderr, "%s\n", (message)); return EXIT_FAILURE; } } while (0)

static uint32_t now_ms;
static BSP_ADC_Sample_t adc_sample;
static BSP_ADC_Status_e adc_status;
TIM_HandleTypeDef htim3;

uint32_t HAL_GetTick(void) { return now_ms; }

BSP_ADC_Status_e BSP_ADCGetSample(BSP_ADC_Sample_t *sample)
{
    if (sample == NULL)
        return BSP_ADC_STATUS_INVALID_ARGUMENT;
    *sample = adc_sample;
    if (adc_status != BSP_ADC_STATUS_VALID)
        return adc_status;
    return (uint32_t)(now_ms - sample->timestamp_ms) <= BSP_ADC_SAMPLE_MAX_AGE_MS
               ? BSP_ADC_STATUS_VALID : BSP_ADC_STATUS_STALE;
}

void HostSetCompare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value)
{
    (void)channel;
    timer->compare = value;
}

int HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    (void)timer;
    (void)channel;
    return 0;
}

float DWT_GetDeltaT(uint32_t *last_count)
{
    if (last_count != NULL)
        ++*last_count;
    return 0.002f;
}

static void setup(uint32_t start_ms)
{
    now_ms = start_ms;
    htim3.compare = 0u;
    adc_sample = (BSP_ADC_Sample_t){
        .sequence = 1u,
        .timestamp_ms = start_ms,
        .vcc_in_volts = 24.0f,
        .voltage_config_valid = 1u,
    };
    adc_status = BSP_ADC_STATUS_VALID;
    IMUHeaterInit();
}

static BMI088_Data_t imu_sample(uint32_t sequence, uint32_t timestamp, float temperature)
{
    return (BMI088_Data_t){
        .temperature = temperature,
        .sequence = sequence,
        .timestamp_ms = timestamp,
        .valid = 1u,
    };
}

static void update_at(uint32_t tick, uint32_t sequence, float temperature)
{
    now_ms = tick;
    adc_sample.timestamp_ms = tick;
    BMI088_Data_t sample = imu_sample(sequence, tick, temperature);
    IMUHeaterUpdate(&sample, 0u);
}

static void get_status(IMUHeaterStatus_t *status)
{
    memset(status, 0, sizeof(*status));
    IMUHeaterGetStatus(status);
}

static int expect_fault(uint8_t expected, const char *message)
{
    IMUHeaterStatus_t status;
    get_status(&status);
    if (status.fault != expected)
    {
        fprintf(stderr, "%s (fault=%u)\n", message, status.fault);
        return 0;
    }
    return 1;
}

static int test_overtemp_and_manual_recovery(void)
{
    setup(100u);
    update_at(100u, 1u, 20.0f);
#if MC02_HEATER_ENABLED
    ASSERT_TRUE(htim3.compare > 0u, "enabled profile should produce a simulated nonzero PWM");
#else
    ASSERT_TRUE(htim3.compare == 0u, "disabled profile must keep simulated PWM at zero");
#endif
    update_at(102u, 2u, 50.0f);
    ASSERT_TRUE(expect_fault(1u, "50C must latch an overtemperature fault"), "fault assertion failed");
    ASSERT_TRUE(htim3.compare == 0u, "overtemperature must turn the heater off immediately");

    update_at(104u, 3u, 42.0f);
    ASSERT_TRUE(expect_fault(1u, "cooling below the trip point must not auto-clear the latch"), "fault cleared without a manual request");
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "42C must be rejected by the recovery API");
    ASSERT_TRUE(expect_fault(1u, "42C must not clear the overtemperature latch"), "42C recovery check failed");
    ASSERT_TRUE(htim3.compare == 0u, "a rejected clear must leave output off");

    update_at(106u, 4u, 41.0f);
    ASSERT_TRUE(IMUHeaterClearFault() == 1u, "a fresh sample below 42C should clear the fault");
    ASSERT_TRUE(expect_fault(0u, "a fresh sample below 42C may clear an overtemperature latch"), "cool recovery check failed");
    ASSERT_TRUE(htim3.compare == 0u, "a successful clear must keep output off for that update");

    BMI088_Data_t replay = imu_sample(4u, 106u, 20.0f);
    now_ms = 107u;
    adc_sample.timestamp_ms = now_ms;
    IMUHeaterUpdate(&replay, 0u);
    ASSERT_TRUE(htim3.compare == 0u, "replaying the clear sample must not resume PWM");
    update_at(108u, 5u, 20.0f);
#if MC02_HEATER_ENABLED
    ASSERT_TRUE(htim3.compare > 0u, "a new post-clear sample may resume normal control");
#else
    ASSERT_TRUE(htim3.compare == 0u, "bench-safe profile must remain heater-disabled after clearing");
#endif
    return EXIT_SUCCESS;
}

static int test_clear_rejects_invalid_temperature(void)
{
    setup(100u);
    update_at(100u, 1u, 50.0f);

    BMI088_Data_t invalid = imu_sample(2u, 102u, NAN);
    now_ms = 102u;
    adc_sample.timestamp_ms = now_ms;
    IMUHeaterUpdate(&invalid, 0u);
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "NaN must not clear a fault");

    invalid = imu_sample(3u, 104u, INFINITY);
    now_ms = 104u;
    adc_sample.timestamp_ms = now_ms;
    IMUHeaterUpdate(&invalid, 0u);
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "infinite temperature must not clear a fault");

    invalid = imu_sample(4u, 106u, 80.1f);
    now_ms = 106u;
    adc_sample.timestamp_ms = now_ms;
    IMUHeaterUpdate(&invalid, 0u);
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "out-of-range temperature must not clear a fault");

    invalid = imu_sample(5u, 108u, -20.1f);
    now_ms = 108u;
    adc_sample.timestamp_ms = now_ms;
    IMUHeaterUpdate(&invalid, 0u);
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "temperature below the valid range must not clear a fault");

    update_at(110u, 6u, -20.0f);
    ASSERT_TRUE(IMUHeaterClearFault() == 1u, "a later valid sample should remain eligible");
    return EXIT_SUCCESS;
}

static int test_clear_requires_recent_imu(uint32_t age_ms)
{
    setup(100u);
    update_at(100u, 1u, 50.0f);
    update_at(100u, 2u, 41.0f);
    now_ms = 100u + age_ms;
    adc_sample.timestamp_ms = now_ms;
    uint8_t cleared = IMUHeaterClearFault();
    if (age_ms == 5u)
    {
        ASSERT_TRUE(cleared == 1u, "an IMU sample exactly 5ms old remains eligible for recovery");
        ASSERT_TRUE(expect_fault(0u, "5ms IMU sample should clear the fault"), "5ms IMU sample was rejected");
    }
    else
    {
        ASSERT_TRUE(cleared == 0u, "an IMU sample older than 5ms must reject recovery");
        ASSERT_TRUE(expect_fault(1u, "6ms IMU sample must leave the fault latched"), "6ms IMU sample was accepted");
    }
    ASSERT_TRUE(htim3.compare == 0u, "clear attempt must keep output off");
    return EXIT_SUCCESS;
}

static int test_clear_requires_forward_sequence(void)
{
    setup(100u);
    update_at(100u, 100u, 50.0f);
    update_at(102u, 100u, 41.0f);
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "a repeated sequence must be rejected");
    ASSERT_TRUE(expect_fault(1u, "a repeated sequence cannot provide a cool recovery sample"), "repeated IMU sequence was accepted");

    BMI088_Data_t older = imu_sample(99u, 103u, 41.0f);
    now_ms = 103u;
    adc_sample.timestamp_ms = now_ms;
    IMUHeaterUpdate(&older, 0u);
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "a backward sequence must be rejected");
    ASSERT_TRUE(expect_fault(1u, "a sequence rollback cannot clear a fault"), "backward IMU sequence was accepted");

    update_at(104u, 101u, 41.0f);
    ASSERT_TRUE(IMUHeaterClearFault() == 1u, "a forward sequence should be accepted");
    ASSERT_TRUE(expect_fault(0u, "a fresh forward sequence may clear a fault"), "forward IMU sequence was rejected");
    return EXIT_SUCCESS;
}

static int test_clear_rejects_invalid_adc(void)
{
    setup(100u);
    update_at(100u, 1u, 50.0f);
    update_at(102u, 2u, 41.0f);

    adc_status = BSP_ADC_STATUS_NOT_READY;
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "ADC not-ready status must reject clear");
    adc_status = BSP_ADC_STATUS_ERROR;
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "ADC error status must reject clear");
    adc_status = BSP_ADC_STATUS_VALID;
    adc_sample.voltage_config_valid = 0u;
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "invalid voltage configuration must reject clear");
    adc_sample.voltage_config_valid = 1u;

    adc_sample.vcc_in_volts = NAN;
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "non-finite VIN must reject clear");
    adc_sample.vcc_in_volts = 9.99f;
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "low VIN must reject clear");
    adc_sample.vcc_in_volts = 30.01f;
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "high VIN must reject clear");

    adc_sample.vcc_in_volts = 24.0f;
    update_at(122u, 3u, 41.0f);
    adc_sample.timestamp_ms = 102u;
    ASSERT_TRUE(IMUHeaterClearFault() == 1u, "an ADC sample exactly 20ms old remains eligible");
    ASSERT_TRUE(expect_fault(0u, "20ms ADC sample should clear the fault"), "20ms ADC sample was rejected");

    update_at(123u, 4u, 41.0f);
    adc_sample.vcc_in_volts = 10.0f;
    ASSERT_TRUE(IMUHeaterClearFault() == 1u, "10V is an inclusive lower VIN boundary");
    update_at(124u, 5u, 41.0f);
    adc_sample.vcc_in_volts = 30.0f;
    ASSERT_TRUE(IMUHeaterClearFault() == 1u, "30V is an inclusive upper VIN boundary");
    return EXIT_SUCCESS;
}

static int test_adc_rejects_21ms_sample(void)
{
    setup(200u);
    update_at(200u, 1u, 50.0f);
    update_at(200u, 2u, 41.0f);
    update_at(221u, 3u, 41.0f);
    adc_sample.timestamp_ms = 200u;
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "an ADC sample older than 20ms must reject clear");
    ASSERT_TRUE(expect_fault(1u, "21ms ADC sample must leave the fault latched"), "21ms ADC sample was accepted");
    ASSERT_TRUE(htim3.compare == 0u, "invalid ADC clear must keep output off");
    return EXIT_SUCCESS;
}

static int test_sensor_loss_timeout_boundary(uint32_t start_ms)
{
    setup(start_ms);
    update_at(start_ms, 1u, 20.0f);
    BMI088_Data_t stale = imu_sample(1u, start_ms, 20.0f);
    uint32_t tick_before = start_ms + 499u;
    uint32_t tick_at = start_ms + 500u;
    now_ms = tick_before;
    adc_sample.timestamp_ms = now_ms;
    IMUHeaterUpdate(&stale, 0u);
    ASSERT_TRUE(expect_fault(0u, "sensor loss must not latch before 500ms"), "sensor timeout latched early");
    now_ms = tick_at;
    adc_sample.timestamp_ms = now_ms;
    IMUHeaterUpdate(&stale, 0u);
    ASSERT_TRUE(expect_fault(1u, "sensor loss must latch at 500ms"), "sensor timeout did not latch at boundary");
    ASSERT_TRUE(htim3.compare == 0u, "sensor timeout must leave output off");
    update_at(tick_at + 1u, 2u, 41.0f);
    ASSERT_TRUE(IMUHeaterClearFault() == 1u, "a new cool sample may clear sensor loss");
    ASSERT_TRUE(expect_fault(0u, "a new cool sample may clear sensor loss after the timeout latch"), "sensor timeout recovery sample was rejected");
    return EXIT_SUCCESS;
}

static int test_continuous_heat_timeout_boundary(uint32_t start_ms, uint8_t reaches_target)
{
    setup(start_ms);
    update_at(start_ms, 1u, 20.0f);
#if MC02_HEATER_ENABLED
    ASSERT_TRUE(htim3.compare > 0u, "continuous-heating test requires simulated PWM");
    update_at(start_ms + 299999u, 2u, 20.0f);
    ASSERT_TRUE(expect_fault(0u, "continuous heating must not latch before 300000ms"), "heater timeout latched early");
    update_at(start_ms + 300000u, 3u, reaches_target ? 42.0f : 20.0f);
    ASSERT_TRUE(expect_fault(1u, "continuous heating must latch at 300000ms"), "heater timeout did not latch at boundary");
    ASSERT_TRUE(htim3.compare == 0u, "heater timeout must leave output off");
    IMUHeaterStatus_t status;
    get_status(&status);
    ASSERT_TRUE(status.timeout_fault == 1u, "continuous timeout must report its fault cause");
#else
    update_at(start_ms + 299999u, 2u, 20.0f);
    update_at(start_ms + 300000u, 3u, reaches_target ? 42.0f : 20.0f);
    ASSERT_TRUE(expect_fault(0u, "a disabled heater cannot accumulate continuous-heating time"), "disabled profile latched a heat timeout");
    ASSERT_TRUE(htim3.compare == 0u, "disabled profile must keep output off");
#endif
    return EXIT_SUCCESS;
}

static int test_heat_timeout_restarts_after_zero(void)
{
    setup(0u);
    update_at(0u, 1u, 20.0f);
#if MC02_HEATER_ENABLED
    ASSERT_TRUE(htim3.compare > 0u, "first heating interval should produce nonzero PWM");
    update_at(299999u, 2u, 42.0f);
    ASSERT_TRUE(htim3.compare == 0u, "reaching the target must end the current heating interval");
    update_at(300000u, 3u, 20.0f);
    ASSERT_TRUE(htim3.compare > 0u, "a new heating interval may start after PWM returned to zero");
    update_at(599999u, 4u, 20.0f);
    ASSERT_TRUE(expect_fault(0u, "time before the new interval must not count toward its timeout"), "heat timer did not restart");
    update_at(600000u, 5u, 20.0f);
    ASSERT_TRUE(expect_fault(1u, "the restarted interval must latch at 300000ms"), "restarted heat interval did not time out");
#else
    update_at(299999u, 2u, 42.0f);
    update_at(300000u, 3u, 20.0f);
    update_at(599999u, 4u, 20.0f);
    update_at(600000u, 5u, 20.0f);
    ASSERT_TRUE(expect_fault(0u, "disabled profile has no nonzero heating interval"), "disabled profile accumulated heating time");
#endif
    return EXIT_SUCCESS;
}

static int test_force_off_and_reinitialization(void)
{
    setup(100u);
    update_at(100u, 1u, 50.0f);
    update_at(102u, 2u, 41.0f);
    BMI088_Data_t force_sample = imu_sample(3u, 103u, 41.0f);
    now_ms = 103u;
    adc_sample.timestamp_ms = now_ms;
    IMUHeaterUpdate(&force_sample, 1u);
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "the latest force-off request must reject clear");
    update_at(104u, 4u, 41.0f);
    IMUHeaterInit();
    ASSERT_TRUE(expect_fault(1u, "reinitialization must not silently erase a latched fault"), "reinitialization cleared the fault");
    return EXIT_SUCCESS;
}

static int test_long_operation_inhibits_clear(void)
{
    setup(100u);
    ASSERT_TRUE(IMUHeaterBeginLongOperation(IMU_HEATER_LONG_OPERATION_FLASH) == 1u,
                "flash operation should acquire its inhibit");
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "active long operation must reject clear");
    IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_FLASH, 0u);
    update_at(102u, 1u, 41.0f);
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "failed flash operation must reject clear");
    ASSERT_TRUE(expect_fault(1u, "failed flash operation must remain latched through fault clear"), "failed operation lock was cleared");
    ASSERT_TRUE(IMUHeaterBeginLongOperation(IMU_HEATER_LONG_OPERATION_FLASH) == 0u,
                "fault clear must not unlock a failed flash operation");
    ASSERT_TRUE(htim3.compare == 0u, "long-operation inhibit must leave output off");
    return EXIT_SUCCESS;
}

static int test_successful_long_operation_requires_new_sample(void)
{
    setup(100u);
    update_at(100u, 1u, 20.0f);
    ASSERT_TRUE(IMUHeaterBeginLongOperation(IMU_HEATER_LONG_OPERATION_CALIBRATION) == 1u,
                "calibration should acquire its inhibit");
    now_ms = 101u;
    adc_sample.timestamp_ms = now_ms;
    IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_CALIBRATION, 1u);
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "pending post-operation sample must reject clear");

    BMI088_Data_t replay = imu_sample(1u, 101u, 20.0f);
    IMUHeaterUpdate(&replay, 0u);
    ASSERT_TRUE(htim3.compare == 0u, "old pre-operation sequence must not restore PWM");
    update_at(102u, 2u, 20.0f);
#if MC02_HEATER_ENABLED
    ASSERT_TRUE(htim3.compare > 0u, "a new post-operation sample may restore normal control");
#else
    ASSERT_TRUE(htim3.compare == 0u, "disabled profile must remain off after a new sample");
#endif
    return EXIT_SUCCESS;
}

static int test_clear_requires_initialized_fresh_state(void)
{
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "uninitialized module must reject clear");
    setup(100u);
    ASSERT_TRUE(IMUHeaterClearFault() == 0u, "missing IMU sample must reject clear");
    update_at(100u, 1u, 41.0f);
    ASSERT_TRUE(IMUHeaterClearFault() == 1u, "a no-fault clear still checks and accepts valid fresh state");
    ASSERT_TRUE(htim3.compare == 0u, "no-fault clear still leaves output off");
    return EXIT_SUCCESS;
}

static int run(const char *name)
{
    if (strcmp(name, "recovery") == 0) return test_overtemp_and_manual_recovery();
    if (strcmp(name, "invalid-temperature") == 0) return test_clear_rejects_invalid_temperature();
    if (strcmp(name, "imu-age-5") == 0) return test_clear_requires_recent_imu(5u);
    if (strcmp(name, "imu-age-6") == 0) return test_clear_requires_recent_imu(6u);
    if (strcmp(name, "sequence") == 0) return test_clear_requires_forward_sequence();
    if (strcmp(name, "adc-invalid") == 0) return test_clear_rejects_invalid_adc();
    if (strcmp(name, "adc-age-21") == 0) return test_adc_rejects_21ms_sample();
    if (strcmp(name, "sensor-timeout") == 0) return test_sensor_loss_timeout_boundary(0u);
    if (strcmp(name, "sensor-timeout-wrap") == 0) return test_sensor_loss_timeout_boundary(UINT32_MAX - 100u);
    if (strcmp(name, "heat-timeout") == 0) return test_continuous_heat_timeout_boundary(0u, 0u);
    if (strcmp(name, "heat-timeout-wrap") == 0) return test_continuous_heat_timeout_boundary(UINT32_MAX - 100u, 0u);
    if (strcmp(name, "heat-timeout-at-target") == 0) return test_continuous_heat_timeout_boundary(0u, 1u);
    if (strcmp(name, "heat-timeout-restarts-after-zero") == 0) return test_heat_timeout_restarts_after_zero();
    if (strcmp(name, "force-off-reinit") == 0) return test_force_off_and_reinitialization();
    if (strcmp(name, "long-operation") == 0) return test_long_operation_inhibits_clear();
    if (strcmp(name, "post-operation-sample") == 0) return test_successful_long_operation_requires_new_sample();
    if (strcmp(name, "initialized-state") == 0) return test_clear_requires_initialized_fresh_state();
    fprintf(stderr, "unknown recovery test scenario: %s\n", name);
    return EXIT_FAILURE;
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "expected one recovery-test scenario name\n");
        return EXIT_FAILURE;
    }
    return run(argv[1]);
}
