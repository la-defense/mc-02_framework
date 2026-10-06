#ifndef MC02_BMI088_HOST_FIXTURE_H
#define MC02_BMI088_HOST_FIXTURE_H

#include "bmi088.h"
#include "bsp_param.h"

typedef struct
{
    uint32_t synthetic_time_us;
    uint32_t transfer_count;
    uint32_t fail_transfer_number;
    HAL_StatusTypeDef fail_status;
    uint8_t fail_next_transfer_after_param_init;
    HAL_StatusTypeDef param_init_failure_status;
    ParamImuCalibMeta_t committed_calibration_meta;
    uint8_t calibration_meta_committed;
} BMI088HostFixture_t;

extern BMI088HostFixture_t bmi088_host_fixture;

void BMI088HostReset(void);
void BMI088HostFailTransferAfter(uint32_t transfer_offset, HAL_StatusTypeDef status);
void BMI088HostFailNextTransferAfterParamInit(HAL_StatusTypeDef status);
BMI088Instance *BMI088HostCreateAcquireInstance(void);
BMI088Instance *BMI088HostRegisterOnline(void);

#endif
