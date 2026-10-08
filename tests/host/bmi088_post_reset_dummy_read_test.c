#include <stdio.h>
#include <stdlib.h>

#include "bmi088_host_fixture.h"
#include "bmi088_regNdef.h"

static void fail(const char *message)
{
    fprintf(stderr, "%s\n", message);
    exit(1);
}

int main(void)
{
    BMI088HostReset();
    BMI088HostSetKernelRunning(1u);
    BMI088HostModelPostResetDummyRead(1u);

    BMI088_Init_Config_s config = {0};
    config.work_mode = BMI088_BLOCK_PERIODIC_MODE;
    config.cali_mode = BMI088_LOAD_PRE_CALI_MODE;
    BMI088Instance *instance = BMI088Register(&config);

    if (instance == NULL)
        fail("BMI088Register must discard the first post-reset accelerometer read before checking chip ID");
    if (bmi088_init_error != BMI088_NO_ERROR || bmi088_init_retry_count != 1u)
        fail("a valid post-reset chip ID must initialize on the first attempt");
    if (bmi088_init_stage != BMI088_INIT_STAGE_READY)
        fail("successful registration must report the ready initialization stage");
    if (bmi088_init_failure_stage != BMI088_INIT_STAGE_IDLE)
        fail("successful registration must not retain a failed initialization stage");
    if (bmi088_init_acc_chip_id != BMI088_ACC_CHIP_ID_VALUE ||
        bmi088_init_gyro_chip_id != BMI088_GYRO_CHIP_ID_VALUE)
        fail("startup diagnostics must publish both decoded chip IDs");
    if (bmi088_host_fixture.accel_soft_reset_count != 1u ||
        bmi088_host_fixture.post_reset_dummy_reads_remaining != 0u ||
        bmi088_host_fixture.accel_chip_id_reads_after_reset != 2u)
        fail("startup must discard one dummy read and verify the next chip ID exactly once");

    puts("PASS: BMI088 discards the first post-reset accelerometer read");
    return 0;
}
