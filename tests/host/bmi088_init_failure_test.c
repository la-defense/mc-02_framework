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
    BMI088HostSetAccelChipId(0u);

    if (BMI088HostRegisterOnline() != NULL)
        fail("BMI088Register must reject an invalid accelerometer chip ID");
    if (bmi088_init_error != BMI088_NO_SENSOR)
        fail("BMI088 startup diagnostics must expose the final sensor initialization error");
    if (bmi088_init_retry_count != BMI088_INIT_MAX_RETRY)
        fail("BMI088 startup diagnostics must expose the retry count");
    if (bmi088_init_acc_status != HAL_OK || bmi088_init_gyro_status != HAL_OK ||
        bmi088_init_acc_chip_id != 0u || bmi088_init_gyro_chip_id != BMI088_GYRO_CHIP_ID_VALUE)
        fail("BMI088 startup diagnostics must distinguish a chip ID mismatch from an SPI transfer error");
    if (bmi088_host_fixture.os_delay_call_count == 0u ||
        bmi088_host_fixture.ins_monitor_feed_count == 0u)
        fail("BMI088 retries must yield to the scheduler and maintain INS monitoring");

    puts("PASS: BMI088 startup failure is bounded, observable, and cooperative");
    return 0;
}
