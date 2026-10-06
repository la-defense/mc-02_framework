#include "bsp_spi.h"

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdatomic.h>

extern volatile uint8_t SPIDeviceOnGoing[SPI_DEVICE_CNT];
extern volatile uint32_t spi_bus_timeout_cnt[SPI_DEVICE_CNT];
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

static SPI_HandleTypeDef spi_handle = {.Instance = SPI1};
static GPIO_TypeDef gpio;
static GPIO_PinState pin_state = GPIO_PIN_SET;
static HAL_StatusTypeDef transfer_result = HAL_OK;
static uint32_t transfer_calls;
static uint32_t transfer_timeout_ms;
static atomic_uint synthetic_us;
static uint8_t tick_wait_allowed = 1u;
static uint32_t tick_wait_attempts;
static uint32_t tick_wait_releases;
uint32_t mc02_test_ipsr;
_Thread_local uint32_t mc02_test_primask;
uint32_t mc02_test_basepri;
uint32_t mc02_test_faultmask;
static pthread_mutex_t irq_mask_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t start_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t start_condition = PTHREAD_COND_INITIALIZER;
static uint8_t start_ready_count;
static uint8_t start_contenders;

typedef struct
{
    SPIInstance *instance;
    uint8_t rx[4];
    uint8_t tx[4];
    HAL_StatusTypeDef result;
} SPIThreadCall;

void mc02_test_disable_irq(void)
{
    pthread_mutex_lock(&irq_mask_mutex);
    mc02_test_primask = 1u;
}

void mc02_test_set_primask(uint32_t value)
{
    mc02_test_primask = value;
    pthread_mutex_unlock(&irq_mask_mutex);
}

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
uint8_t BSP_HALTickTryAcquireBlockingWait(void)
{
    tick_wait_attempts++;
    return tick_wait_allowed;
}
void BSP_HALTickReleaseBlockingWait(void)
{
    tick_wait_releases++;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *handle, uint8_t *tx, uint8_t *rx, uint16_t len)
{ (void)handle; (void)tx; (void)rx; (void)len; transfer_calls++; return transfer_result; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *handle, uint8_t *tx, uint8_t *rx, uint16_t len)
{ (void)handle; (void)tx; (void)rx; (void)len; transfer_calls++; return transfer_result; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{ (void)port; (void)pin; pin_state = state; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{ (void)port; (void)pin; return pin_state; }
uint32_t DWT_ProbeStart(void) { return atomic_fetch_add(&synthetic_us, 1u); }
uint32_t DWT_ProbeElapsedUs(uint32_t start) { return atomic_fetch_add(&synthetic_us, 1u) - start; }

static void *SPIThreadContender(void *argument)
{
    SPIThreadCall *call = (SPIThreadCall *)argument;
    pthread_mutex_lock(&start_mutex);
    start_ready_count++;
    pthread_cond_broadcast(&start_condition);
    while (!start_contenders)
        pthread_cond_wait(&start_condition, &start_mutex);
    pthread_mutex_unlock(&start_mutex);

    call->result = SPITransRecv(call->instance, call->rx, call->tx, sizeof(call->rx));
    return NULL;
}

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
        uint32_t releases_before_transfer = tick_wait_releases;
        transfer_result = statuses[i];
        HAL_StatusTypeDef actual = SPITransRecv(&instance, rx, tx, sizeof(rx));
        if (actual != statuses[i]) fail("SPITransRecv must propagate the blocking HAL result");
        if (tick_wait_releases != releases_before_transfer + 1u)
            fail("blocking SPI must release its HAL tick lease for every transfer result");
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

    uint32_t calls_before_suspended_tick = transfer_calls;
    uint32_t attempts_before_suspended_tick = tick_wait_attempts;
    uint32_t releases_before_suspended_tick = tick_wait_releases;
    tick_wait_allowed = 0u;
    if (SPITransRecv(&instance, rx, tx, sizeof(rx)) != HAL_BUSY)
        fail("blocking SPI must be rejected while the HAL tick is suspended");
    tick_wait_allowed = 1u;
    if (transfer_calls != calls_before_suspended_tick)
        fail("HAL SPI must not start while its timeout tick is suspended");
    if (tick_wait_attempts != attempts_before_suspended_tick + 1u)
        fail("blocking SPI must acquire a HAL tick lease before transfer");
    if (tick_wait_releases != releases_before_suspended_tick)
        fail("a rejected HAL tick lease must not be released");
    if (SPIDeviceOnGoing[0] != 1u || pin_state != GPIO_PIN_SET)
        fail("suspended-tick rejection must release the bus without selecting a device");

    uint32_t releases_before_transfer = tick_wait_releases;
    if (SPITransRecv(&instance, rx, tx, sizeof(rx)) != transfer_result)
        fail("blocking SPI must preserve the HAL transfer status with a tick lease");
    if (tick_wait_releases != releases_before_transfer + 1u)
        fail("blocking SPI must release its HAL tick lease after transfer");

    SPI_Init_Config_s first_config = {
        .spi_handle = &spi_handle,
        .GPIOx = &gpio,
        .cs_pin = 2u,
        .spi_work_mode = SPI_DMA_MODE,
    };
    SPI_Init_Config_s second_config = first_config;
    second_config.cs_pin = 3u;
    SPIInstance *first_instance = SPIRegister(&first_config);
    SPIInstance *second_instance = SPIRegister(&second_config);
    uint8_t async_rx[4] = {0};
    uint8_t async_tx[4] = {0};
    transfer_result = HAL_OK;
    uint32_t tick_attempts_before_async = tick_wait_attempts;
    if (SPITransRecv(second_instance, async_rx, async_tx, sizeof(async_rx)) != HAL_OK)
        fail("registered asynchronous transfer should start successfully");
    if (tick_wait_attempts != tick_attempts_before_async)
        fail("asynchronous SPI modes must not reserve the HAL tick");
    if (SPIDeviceOnGoing[0] != 0u)
        fail("asynchronous transfer must retain bus ownership until completion or error");
    HAL_SPI_ErrorCallback(&spi_handle);
    if (SPIDeviceOnGoing[0] != 1u || second_instance->CS_State != GPIO_PIN_SET)
        fail("SPI error callback must release the active owner, not the first device on the bus");
    if (first_instance == second_instance)
        fail("SPI registration must keep distinct owner instances for one bus");

    SPI_HandleTypeDef concurrent_handle = {.Instance = SPI1};
    GPIO_TypeDef concurrent_gpio[2] = {{0}, {0}};
    volatile uint8_t bus_state[2] = {1u, 1u};
    SPIInstance concurrent_instances[2] = {0};
    SPIThreadCall calls[2] = {0};
    pthread_t threads[2];
    for (size_t i = 0; i < 2u; ++i)
    {
        concurrent_instances[i].spi_handle = &concurrent_handle;
        concurrent_instances[i].GPIOx = &concurrent_gpio[i];
        concurrent_instances[i].cs_pin = (uint16_t)(i + 2u);
        concurrent_instances[i].cs_pin_state = &bus_state[i];
        concurrent_instances[i].spi_work_mode = SPI_DMA_MODE;
        calls[i].instance = &concurrent_instances[i];
        if (pthread_create(&threads[i], NULL, SPIThreadContender, &calls[i]) != 0)
            fail("unable to create concurrent SPI claimant");
    }
    pthread_mutex_lock(&start_mutex);
    while (start_ready_count < 2u)
        pthread_cond_wait(&start_condition, &start_mutex);
    uint32_t calls_before_concurrent = transfer_calls;
    transfer_result = HAL_OK;
    start_contenders = 1u;
    pthread_cond_broadcast(&start_condition);
    pthread_mutex_unlock(&start_mutex);
    for (size_t i = 0; i < 2u; ++i)
        if (pthread_join(threads[i], NULL) != 0)
            fail("unable to join concurrent SPI claimant");

    uint8_t successful_claims = (uint8_t)((calls[0].result == HAL_OK) + (calls[1].result == HAL_OK));
    uint8_t timed_out_claims = (uint8_t)((calls[0].result == HAL_TIMEOUT) + (calls[1].result == HAL_TIMEOUT));
    if (successful_claims != 1u || timed_out_claims != 1u)
        fail("simultaneous SPI claimants must produce one owner and one timeout");
    if (transfer_calls != calls_before_concurrent + 1u)
        fail("only the atomic bus owner may start an SPI transfer");
    if (SPIDeviceOnGoing[0] != 0u)
        fail("the successful asynchronous transfer must retain bus ownership");
    return EXIT_SUCCESS;
}
