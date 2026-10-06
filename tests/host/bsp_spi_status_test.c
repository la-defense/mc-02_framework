#include "bsp_spi.h"

#include <stdio.h>
#include <stdlib.h>

extern volatile uint8_t SPIDeviceOnGoing[SPI_DEVICE_CNT];
extern volatile uint32_t spi_bus_timeout_cnt[SPI_DEVICE_CNT];

static SPI_HandleTypeDef spi_handle = {.Instance = SPI1};
static GPIO_TypeDef gpio;
static GPIO_PinState pin_state = GPIO_PIN_SET;
static HAL_StatusTypeDef transfer_result = HAL_OK;
static uint32_t transfer_calls;
static uint32_t transfer_timeout_ms;
static uint32_t synthetic_us;
uint32_t mc02_test_ipsr;
uint32_t mc02_test_primask;
uint32_t mc02_test_basepri;
uint32_t mc02_test_faultmask;

static void fail(const char *message)
{
    fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *handle, uint8_t *tx, uint8_t *rx,
                                          uint16_t len, uint32_t timeout)
{
    (void)handle; (void)tx; (void)rx; (void)len;
    transfer_calls++;
    transfer_timeout_ms = timeout;
    return transfer_result;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *handle, uint8_t *tx, uint8_t *rx, uint16_t len)
{ (void)handle; (void)tx; (void)rx; (void)len; transfer_calls++; return transfer_result; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *handle, uint8_t *tx, uint8_t *rx, uint16_t len)
{ (void)handle; (void)tx; (void)rx; (void)len; transfer_calls++; return transfer_result; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{ (void)port; (void)pin; pin_state = state; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{ (void)port; (void)pin; return pin_state; }
uint32_t DWT_ProbeStart(void) { return synthetic_us; }
uint32_t DWT_ProbeElapsedUs(uint32_t start) { return synthetic_us++ - start; }

int main(void)
{
    SPIInstance instance = {0};
    uint8_t rx[4] = {0};
    uint8_t tx[4] = {0};
    uint8_t cs_state = GPIO_PIN_SET;
    instance.spi_handle = &spi_handle;
    instance.GPIOx = &gpio;
    instance.cs_pin = 1u;
    instance.spi_work_mode = SPI_BLOCK_MODE;
    instance.cs_pin_state = &cs_state;

    const HAL_StatusTypeDef statuses[] = {HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    for (size_t i = 0; i < sizeof(statuses) / sizeof(statuses[0]); ++i)
    {
        transfer_result = statuses[i];
        HAL_StatusTypeDef actual = SPITransRecv(&instance, rx, tx, sizeof(rx));
        if (actual != statuses[i]) fail("SPITransRecv must propagate the blocking HAL result");
        if (pin_state != GPIO_PIN_SET || cs_state != GPIO_PIN_SET || SPIDeviceOnGoing[0] != 1u)
            fail("SPI block transfer must release chip select and bus after every HAL result");
        if (transfer_timeout_ms != 2u) fail("blocking SPI transfer must use the bounded 2ms timeout");
    }

    uint32_t calls_before_timeout = transfer_calls;
    SPIDeviceOnGoing[0] = 0u;
    pin_state = GPIO_PIN_SET;
    HAL_StatusTypeDef busy_status = SPITransRecv(&instance, rx, tx, sizeof(rx));
    if (busy_status != HAL_TIMEOUT) fail("bus ownership timeout must be returned to the caller");
    if (transfer_calls != calls_before_timeout) fail("SPI HAL must not start while another transfer owns the bus");
    if (SPIDeviceOnGoing[0] != 0u) fail("wait timeout must not steal an active transfer's bus ownership");
    if (pin_state != GPIO_PIN_SET) fail("wait timeout must not assert chip select");
    if (spi_bus_timeout_cnt[0] != 1u) fail("bus wait timeout counter must increase");

    SPIDeviceOnGoing[0] = 1u;
    const uint32_t calls_before_context_checks = transfer_calls;
    mc02_test_ipsr = 15u;
    if (SPITransRecv(&instance, rx, tx, sizeof(rx)) != HAL_BUSY)
        fail("blocking SPI must be rejected from interrupt context");
    mc02_test_ipsr = 0u;
    mc02_test_primask = 1u;
    if (SPITransRecv(&instance, rx, tx, sizeof(rx)) != HAL_BUSY)
        fail("blocking SPI must be rejected while interrupts are globally masked");
    mc02_test_primask = 0u;
    mc02_test_basepri = 0x40u;
    if (SPITransRecv(&instance, rx, tx, sizeof(rx)) != HAL_BUSY)
        fail("blocking SPI must be rejected inside a BASEPRI critical section");
    mc02_test_basepri = 0u;
    mc02_test_faultmask = 1u;
    if (SPITransRecv(&instance, rx, tx, sizeof(rx)) != HAL_BUSY)
        fail("blocking SPI must be rejected while FAULTMASK is set");
    mc02_test_faultmask = 0u;
    if (transfer_calls != calls_before_context_checks)
        fail("HAL SPI must not start from a context where its timeout tick may be masked");
    if (SPIDeviceOnGoing[0] != 1u || pin_state != GPIO_PIN_SET)
        fail("context rejection must leave the SPI bus and chip select untouched");
    return EXIT_SUCCESS;
}
