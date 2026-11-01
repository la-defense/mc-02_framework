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
    const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    const uint32_t transfer_offsets[] = {1u, 2u, 3u};
    for (size_t i = 0; i < sizeof(failures) / sizeof(failures[0]); ++i)
    {
        for (size_t j = 0; j < sizeof(transfer_offsets) / sizeof(transfer_offsets[0]); ++j)
        {
            BMI088HostReset();
            BMI088HostSetKernelRunning(1u);
            /* Transfers 1, 2 and 3 are probe, soft reset and post-reset dummy read. */
            BMI088HostFailTransferAfter(transfer_offsets[j], failures[i]);

            BMI088Instance *instance = BMI088HostRegisterOnline();
            if (instance == NULL)
                fail("a one-shot initialization SPI failure must be retried");
            if (bmi088_init_error != BMI088_NO_ERROR || bmi088_init_retry_count != 2u)
                fail("a failed initialization SPI operation must abort that attempt and retry");
            if (bmi088_host_fixture.failed_spi_transfer_count != 1u)
                fail("the injected initialization SPI failure must be observed once");

            uint32_t expected_resets = transfer_offsets[j] == 3u ? 2u : 1u;
            uint32_t expected_failed_writes = transfer_offsets[j] == 2u ? 1u : 0u;
            if (bmi088_host_fixture.failed_spi_write_transfer_count != expected_failed_writes ||
                bmi088_host_fixture.accel_soft_reset_count != expected_resets)
                fail("failed SPI writes must not reach the sensor model");

            free(instance);
        }
    }

    puts("PASS: BMI088 initialization SPI failures abort the current attempt");
    return 0;
}
