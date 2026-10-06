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
    BMI088HostFailNextTransferAfterParamInit(HAL_ERROR);
    BMI088Instance *instance = BMI088HostRegisterOnline();
    if (instance == NULL)
        fail("BMI088Register should initialize and calibrate stable synthetic samples");
    if (cali_diag_acq_fail != 1u || cali_diag_acq_ok != 6000u)
        fail("startup calibration must skip a failed partial transfer and collect 6000 successful samples");

    if (fabsf(instance->temperature - 22.875f) > 0.0001f)
        fail("calibration instance temperature must remain Celsius without rescaling");

    if (fabsf(bmi088_calib_temp - 22.875f) > 0.0001f)
        fail("published calibration temperature must remain Celsius without rescaling");

    if (!bmi088_host_fixture.calibration_meta_committed ||
        bmi088_host_fixture.committed_calibration_meta.result != PARAM_IMU_CALIB_OK)
        fail("successful startup calibration must commit metadata through the parameter API");

    if (fabsf(bmi088_host_fixture.committed_calibration_meta.temperature - 22.875f) > 0.0001f)
        fail("persisted calibration temperature must remain Celsius without rescaling");

    return EXIT_SUCCESS;
}
