#include "bmi088_host_fixture.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

extern volatile uint32_t cali_diag_acq_ok;
extern volatile uint32_t cali_diag_acq_fail;

static void fail(const char *message)
{
    fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
}

int main(void)
{
    BMI088HostReset();
    BMI088HostSetKernelRunning(1u);
    BMI088HostFailNextTransferAfterParamInit(HAL_ERROR);
    BMI088Instance *instance = BMI088HostRegisterOnline();
    if (instance == NULL)
        fail("BMI088Register should initialize and calibrate stable synthetic samples");
    if (bmi088_host_fixture.max_unyielded_busy_wait_us > 5000u)
        fail("BMI088 startup initialization and calibration must yield within the INS monitor deadline");
    if (bmi088_host_fixture.os_delay_call_count == 0u ||
        bmi088_host_fixture.ins_monitor_feed_count == 0u)
        fail("BMI088 startup initialization must yield to RTOS tasks and keep INS health monitoring alive");
    if (cali_diag_acq_fail != 1u || cali_diag_acq_ok != 6000u)
        fail("startup calibration must skip a failed partial transfer and collect 6000 successful samples");

    if (fabsf(instance->temperature - 22.875f) > 0.0001f)
        fail("calibration instance temperature must remain Celsius without rescaling");

    if (fabsf(bmi088_calib_temp - 22.875f) > 0.0001f)
        fail("published calibration temperature must remain Celsius without rescaling");

    if (!bmi088_host_fixture.calibration_meta_committed ||
        bmi088_host_fixture.committed_calibration_meta.result != PARAM_IMU_CALIB_OK)
        fail("successful startup calibration must commit metadata through the parameter API");

    if (bmi088_host_fixture.calibration_monitor_begin_count != 1u ||
        bmi088_host_fixture.calibration_monitor_end_count != 1u ||
        !bmi088_host_fixture.calibration_monitor_end_success)
        fail("startup calibration must use and close one bounded INS monitor window");
    if (bmi088_host_fixture.calibration_heater_begin_count != 1u ||
        bmi088_host_fixture.calibration_heater_end_count != 1u ||
        !bmi088_host_fixture.calibration_heater_end_success)
        fail("startup calibration must inhibit the heater until the operation succeeds");

    if (fabsf(bmi088_host_fixture.committed_calibration_meta.temperature - 22.875f) > 0.0001f)
        fail("persisted calibration temperature must remain Celsius without rescaling");

    if (!BMI088CalibRequest() || !BMI088CalibService())
        fail("on-demand calibration should execute through the bounded operation path");
    if (bmi088_host_fixture.calibration_monitor_begin_count != 2u ||
        bmi088_host_fixture.calibration_monitor_end_count != 2u ||
        bmi088_host_fixture.calibration_heater_begin_count != 2u ||
        bmi088_host_fixture.calibration_heater_end_count != 2u ||
        !bmi088_host_fixture.calibration_heater_end_success)
        fail("on-demand calibration must close its monitor and heater guards");

    bmi088_host_fixture.calibration_monitor_allow = 0u;
    uint32_t transfers_before_refusal = bmi088_host_fixture.transfer_count;
    if (!BMI088CalibRequest() || !BMI088CalibService())
        fail("a refused calibration request must be reported as completed with failure");
    if (bmi088_host_fixture.transfer_count != transfers_before_refusal ||
        BMI088CalibGetState() != BMI088_RECALIB_FAIL ||
        bmi088_host_fixture.calibration_heater_end_success != 0u)
        fail("guard refusal must skip calibration reads and leave the heater inhibited");

    return EXIT_SUCCESS;
}
