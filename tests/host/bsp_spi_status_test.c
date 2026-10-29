/** @file bsp_spi_status_test.c
 * @brief 生产 SPI 的注册、收发、回调、期限及隔离测试；只替换 HAL、时间和分配器。
 * @note 每项 CTest 独立启动，Release 仍检查；WSL 额外运行真实线程竞争。
 */
#include "bsp_spi.h"
#include "test_check.h"
#include <limits.h>
#include <string.h>
#ifdef SPI_TEST_THREADS
#include <pthread.h>
#include <stdatomic.h>
static pthread_mutex_t irq_mutex = PTHREAD_MUTEX_INITIALIZER;
static atomic_uint elapsed_us, tick_ms;
#else
static uint32_t elapsed_us, tick_ms;
#endif
SPI_TypeDef spi_registers[2];
DMA_Stream_TypeDef dma_registers[16];
_Thread_local uint32_t mc02_test_ipsr, mc02_test_primask, mc02_test_basepri, mc02_test_faultmask;
static SPI_HandleTypeDef handles[2];
static SPI_HandleTypeDef *ws2812_owned_handle;
static uint32_t ws2812_completions, ws2812_errors;
uint8_t WS2812_OnTxComplete(SPI_HandleTypeDef *handle)
{
    if (ws2812_owned_handle == NULL || handle != ws2812_owned_handle) return 0u;
    ++ws2812_completions;
    return 1u;
}
uint8_t WS2812_OnError(SPI_HandleTypeDef *handle)
{
    if (ws2812_owned_handle == NULL || handle != ws2812_owned_handle) return 0u;
    ++ws2812_errors;
    return 1u;
}
static DMA_HandleTypeDef dmas[4];
static GPIO_TypeDef gpio;
static uint8_t irq_enabled[18];
static uint32_t irq_clears[18], dma_clears, calls, abort_calls, leases, allocations, frees, callbacks;
static uint32_t hal_timeout;
static int last_operation, last_mode;
static uint8_t allocation_fail, tick_allowed = 1u, abort_fail, abort_not_stopped, dirty_failure;
static uint8_t early_complete, early_error, callback_checks, abort_late_callback;
static HAL_StatusTypeDef hal_result = HAL_OK;
static uint8_t tx[8], rx[8], other[8];

void mc02_test_disable_irq(void)
{
#ifdef SPI_TEST_THREADS
    if (!mc02_test_primask) TEST_CHECK(pthread_mutex_lock(&irq_mutex) == 0);
#endif
    mc02_test_primask = 1u;
}
void mc02_test_set_primask(uint32_t value)
{
    mc02_test_primask = value;
#ifdef SPI_TEST_THREADS
    if (!value) TEST_CHECK(pthread_mutex_unlock(&irq_mutex) == 0);
#endif
}
// 仅生产目标重命名 malloc/free；测试自身使用标准库，没有生产专用重置入口。
void *mc02_spi_malloc(size_t size)
{
    if (allocation_fail) return NULL;
    void *result = malloc(size);
    if (result != NULL) allocations++;
    return result;
}
void mc02_spi_free(void *pointer) { if (pointer != NULL) frees++; free(pointer); }
uint32_t NVIC_GetEnableIRQ(IRQn_Type irq) { TEST_CHECK(irq >= 0 && irq < 18); return irq_enabled[irq]; }
void NVIC_ClearPendingIRQ(IRQn_Type irq) { TEST_CHECK(irq >= 0 && irq < 18); irq_clears[irq]++; }
void mc02_dma_clear(DMA_HandleTypeDef *dma, uint32_t flags)
{ TEST_CHECK(dma != NULL && flags == 31u); dma_clears++; }
uint32_t HAL_GetTick(void) { return tick_ms; }
uint32_t DWT_ProbeStart(void)
{
#ifdef SPI_TEST_THREADS
    return atomic_fetch_add(&elapsed_us, 1u);
#else
    return elapsed_us++;
#endif
}
uint32_t DWT_ProbeElapsedUs(uint32_t start) { return DWT_ProbeStart() - start; }
uint8_t BSP_HALTickTryAcquireBlockingWait(void) { if (!tick_allowed) return 0u; leases++; return 1u; }
void BSP_HALTickReleaseBlockingWait(void) { TEST_CHECK(leases != 0u); leases--; }
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *handle) { return handle->State; }
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *dma) { return dma->State; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState s)
{ if (s == GPIO_PIN_SET) port->pins |= pin; else port->pins &= (uint16_t)~pin; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{ return (port->pins & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET; }
static void stopped(SPI_HandleTypeDef *h)
{
    h->State = HAL_SPI_STATE_READY; h->Instance->CR1 = h->Instance->CFG1 = h->Instance->IER = 0u;
    h->hdmatx->State = h->hdmarx->State = HAL_DMA_STATE_READY;
    h->hdmatx->Instance->CR = h->hdmarx->Instance->CR = 0u;
}
static void complete(SPI_HandleTypeDef *h, int op)
{
    stopped(h);
    if (op == 0) HAL_SPI_TxCpltCallback(h);
    else if (op == 1) HAL_SPI_RxCpltCallback(h);
    else HAL_SPI_TxRxCpltCallback(h);
}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *h)
{
    TEST_CHECK(!mc02_test_primask && !mc02_test_ipsr);
    abort_calls++;
    if (abort_late_callback) HAL_SPI_TxRxCpltCallback(h);
    if (!abort_fail && !abort_not_stopped) stopped(h);
    // HAL 返回 OK 和 SPI READY，DMA 仍启用时也不能释放数组。
    if (abort_not_stopped) { h->State = HAL_SPI_STATE_READY; h->Instance->CR1 = h->Instance->CFG1 = h->Instance->IER = 0u; }
    return abort_fail ? HAL_ERROR : HAL_OK;
}
static HAL_StatusTypeDef transfer(SPI_HandleTypeDef *h, int op, int mode, uint32_t timeout)
{
    TEST_CHECK(!mc02_test_primask && leases > 0u);
    calls++; last_operation = op; last_mode = mode; hal_timeout = timeout;
    h->State = HAL_SPI_STATE_BUSY; h->Instance->CR1 = SPI_CR1_SPE | SPI_CR1_CSTART;
    if (mode == 2)
    {
        if (op != 1) { h->hdmatx->State = HAL_DMA_STATE_BUSY; h->hdmatx->Instance->CR = 1u; }
        if (op != 0) { h->hdmarx->State = HAL_DMA_STATE_BUSY; h->hdmarx->Instance->CR = 1u; }
        h->Instance->CFG1 = 3u;
    }
    if ((!mode || hal_result != HAL_OK) && !dirty_failure) stopped(h);
    if (early_complete) complete(h, op);
    if (early_error) HAL_SPI_ErrorCallback(h);
    return hal_result;
}
#define DEFINE_HAL(name, op) \
HAL_StatusTypeDef HAL_SPI_##name(SPI_HandleTypeDef *h, uint8_t *p, uint16_t n, uint32_t t) \
{ (void)p; TEST_CHECK(n == 8u); return transfer(h, op, 0, t); } \
HAL_StatusTypeDef HAL_SPI_##name##_IT(SPI_HandleTypeDef *h, uint8_t *p, uint16_t n) \
{ (void)p; TEST_CHECK(n == 8u); return transfer(h, op, 1, 0u); } \
HAL_StatusTypeDef HAL_SPI_##name##_DMA(SPI_HandleTypeDef *h, uint8_t *p, uint16_t n) \
{ (void)p; TEST_CHECK(n == 8u); return transfer(h, op, 2, 0u); }
DEFINE_HAL(Transmit, 0)
DEFINE_HAL(Receive, 1)
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *t, uint8_t *r, uint16_t n, uint32_t timeout)
{ (void)t; (void)r; TEST_CHECK(n == 8u); return transfer(h, 2, 0, timeout); }
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *h, uint8_t *t, uint8_t *r, uint16_t n)
{ (void)t; (void)r; TEST_CHECK(n == 8u); return transfer(h, 2, 1, 0u); }
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *h, uint8_t *t, uint8_t *r, uint16_t n)
{ (void)t; (void)r; TEST_CHECK(n == 8u); return transfer(h, 2, 2, 0u); }
static SPITransferState_t state(SPIInstance *instance)
{
    SPITransferState_t result;
    TEST_CHECK(SPIGetTransferState(instance, &result) == HAL_OK);
    return result;
}
static void received(SPIInstance *instance)
{
    TEST_CHECK(!mc02_test_primask && HAL_GPIO_ReadPin(instance->GPIOx, instance->cs_pin) == GPIO_PIN_SET);
    TEST_CHECK(instance->rx_size == sizeof(rx) && instance->rx_buffer == rx);
    callbacks++;
    if (callback_checks)
    {
        TEST_CHECK(!state(instance).buffers_released && !SPIUnregister(instance));
        TEST_CHECK(SPISetMode(instance, SPI_BLOCK_MODE) == HAL_BUSY);
        TEST_CHECK(SPITransRecv(instance, other, tx, 8u) == HAL_BUSY);
        TEST_CHECK(SPIProcessTimeouts() == HAL_OK);
    }
}
static SPI_Init_Config_s config(unsigned bus, uint16_t pin, SPI_TXRX_MODE_e mode)
{
    SPI_Init_Config_s c = {.spi_handle = &handles[bus], .GPIOx = &gpio, .cs_pin = pin,
        .spi_work_mode = mode, .callback = received};
    return c;
}
static SPIInstance *device(unsigned bus, uint16_t pin, SPI_TXRX_MODE_e mode)
{
    SPI_Init_Config_s c = config(bus, pin, mode);
    SPIInstance *instance = SPIRegister(&c); TEST_CHECK(instance != NULL); return instance;
}
static HAL_StatusTypeDef submit(SPIInstance *instance, int op)
{
    if (!op) return SPITransmit(instance, tx, 8u);
    if (op == 1) return SPIRecv(instance, rx, 8u);
    return SPITransRecv(instance, rx, tx, 8u);
}
static void registration(void)
{
    SPI_Init_Config_s c = config(0u, 1u, SPI_BLOCK_MODE), original = c;
    TEST_CHECK(SPIRegister(NULL) == NULL);
    c.spi_handle = NULL; TEST_CHECK(SPIRegister(&c) == NULL); c = original;
    SPI_HandleTypeDef unknown = handles[0]; unknown.Instance = NULL;
    c.spi_handle = &unknown; TEST_CHECK(SPIRegister(&c) == NULL); c = original;
    c.GPIOx = NULL; TEST_CHECK(SPIRegister(&c) == NULL); c = original;
    c.cs_pin = 0u; TEST_CHECK(SPIRegister(&c) == NULL);
    c.cs_pin = 3u; TEST_CHECK(SPIRegister(&c) == NULL); c = original;
    c.spi_work_mode = (SPI_TXRX_MODE_e)9; TEST_CHECK(SPIRegister(&c) == NULL); c = original;
    c.async_timeout_ms = UINT32_MAX; TEST_CHECK(SPIRegister(&c) == NULL); c = original;
    allocation_fail = 1u; TEST_CHECK(SPIRegister(&c) == NULL); allocation_fail = 0u;
    SPIInstance *instances[4]; instances[0] = device(0u, 1u, SPI_BLOCK_MODE);
    TEST_CHECK(memcmp(&c, &original, sizeof(c)) == 0 && allocations == 1u);
    SPIInstance old = *instances[0];
    TEST_CHECK(SPIRegister(&c) == NULL && allocations == 1u);
    SPI_HandleTypeDef alias = handles[0]; c.spi_handle = &alias; c.cs_pin = 2u;
    TEST_CHECK(SPIRegister(&c) == NULL); c = original;
    for (unsigned i = 1u; i < 4u; ++i) instances[i] = device(i % 2u, (uint16_t)(1u << i), SPI_BLOCK_MODE);
    c.cs_pin = 16u; TEST_CHECK(SPIRegister(&c) == NULL && allocations == 4u);
    TEST_CHECK(memcmp(&old, instances[0], sizeof(old)) == 0 && calls == 0u);
    TEST_CHECK(instances[0]->rx_buffer == NULL && instances[0]->rx_size == 0u);
    TEST_CHECK(instances[0]->async_timeout_ms == 20u && state(instances[0]).buffers_released);
    for (unsigned i = 0u; i < 4u; ++i) TEST_CHECK(SPIUnregister(instances[i]));
    TEST_CHECK(allocations == frees && !SPIUnregister(NULL));
    c.async_timeout_ms = INT32_MAX;
    SPIInstance *again = SPIRegister(&c); TEST_CHECK(again != NULL && SPIUnregister(again));
}
static void operations(void)
{
    SPIInstance *instance = device(0u, 1u, SPI_BLOCK_MODE);
    const HAL_StatusTypeDef results[] = {HAL_OK, HAL_BUSY, HAL_ERROR, HAL_TIMEOUT};
    for (int mode = 0; mode < 3; ++mode)
        for (int op = 0; op < 3; ++op)
            for (unsigned i = 0u; i < 4u; ++i)
            {
                TEST_CHECK(SPISetMode(instance, (SPI_TXRX_MODE_e)mode) == HAL_OK);
                hal_result = results[i];
                TEST_CHECK(submit(instance, op) == results[i] && last_operation == op && last_mode == mode);
                if (!mode) TEST_CHECK(hal_timeout == 2u);
                if (mode && hal_result == HAL_OK)
                {
                    TEST_CHECK(!state(instance).buffers_released && leases == 1u);
                    uint32_t before = callbacks; complete(&handles[0], op);
                    TEST_CHECK(callbacks == before + (op != 0));
                }
                TEST_CHECK(state(instance).result == results[i] && state(instance).buffers_released && leases == 0u);
                TEST_CHECK(HAL_GPIO_ReadPin(&gpio, 1u) == GPIO_PIN_SET);
            }
    TEST_CHECK(allocations == 1u && SPIUnregister(instance));
}
static void invalid(void)
{
    SPIInstance *instance = device(0u, 1u, SPI_DMA_MODE);
    SPITransferState_t original = state(instance), output = original;
    TEST_CHECK(SPITransmit(NULL, tx, 8u) == HAL_ERROR);
    SPIInstance unregistered = *instance;
    TEST_CHECK(SPIRecv(&unregistered, rx, 8u) == HAL_ERROR);
    TEST_CHECK(SPIRecv(instance, NULL, 8u) == HAL_ERROR && SPITransmit(instance, tx, 0u) == HAL_ERROR);
    TEST_CHECK(SPITransRecv(instance, rx, rx + 1, 4u) == HAL_ERROR);
    TEST_CHECK(SPIGetTransferState(NULL, &output) == HAL_ERROR && memcmp(&output, &original, sizeof(output)) == 0);
    TEST_CHECK(SPIGetTransferState(instance, NULL) == HAL_ERROR);
    TEST_CHECK(SPISetMode(instance, (SPI_TXRX_MODE_e)99) == HAL_ERROR);
    irq_enabled[SPI1_IRQn] = 0u; TEST_CHECK(submit(instance, 2) == HAL_ERROR); irq_enabled[SPI1_IRQn] = 1u;
    handles[0].hdmatx = NULL; TEST_CHECK(submit(instance, 2) == HAL_ERROR); handles[0].hdmatx = &dmas[0];
    irq_enabled[0] = 0u; TEST_CHECK(submit(instance, 2) == HAL_ERROR); irq_enabled[0] = 1u;
    dmas[0].Init.Mode = 1u; TEST_CHECK(submit(instance, 2) == HAL_ERROR); dmas[0].Init.Mode = DMA_NORMAL;
    dmas[0].Init.MemDataAlignment = 2u; TEST_CHECK(submit(instance, 2) == HAL_ERROR); dmas[0].Init.MemDataAlignment = 0u;
    handles[0].Init.DataSize = 16u; TEST_CHECK(submit(instance, 2) == HAL_ERROR); handles[0].Init.DataSize = 8u;
    tick_allowed = 0u; TEST_CHECK(submit(instance, 2) == HAL_BUSY); tick_allowed = 1u;
    handles[0].State = HAL_SPI_STATE_BUSY; TEST_CHECK(submit(instance, 2) == HAL_BUSY); stopped(&handles[0]);
    uint32_t *flags[] = {&mc02_test_ipsr, &mc02_test_primask, &mc02_test_basepri, &mc02_test_faultmask};
    for (unsigned i = 0u; i < 4u; ++i)
    {
        *flags[i] = 1u;
        TEST_CHECK(submit(instance, 2) == HAL_BUSY && SPISetMode(instance, SPI_IT_MODE) == HAL_BUSY);
        TEST_CHECK(SPIProcessTimeouts() == HAL_BUSY && !SPIUnregister(instance));
        *flags[i] = 0u;
    }
    TEST_CHECK(calls == 0u && leases == 0u && abort_calls == 0u);
    output = state(instance); TEST_CHECK(memcmp(&output, &original, sizeof(output)) == 0);
    TEST_CHECK(SPIUnregister(instance));
}
static void ownership(void)
{
    // 先注册 b，再让 a 占用，验证回调不会误选表中第一个设备。
    SPIInstance *b = device(0u, 2u, SPI_DMA_MODE), *a = device(0u, 1u, SPI_DMA_MODE);
    SPIInstance *c = device(1u, 4u, SPI_IT_MODE);
    TEST_CHECK(submit(a, 2) == HAL_OK);
    SPIInstance before = *a;
    for (int op = 0; op < 3; ++op) TEST_CHECK(submit(b, op) == HAL_BUSY);
    TEST_CHECK(SPIRecv(a, other, 8u) == HAL_BUSY && memcmp(a, &before, sizeof(before)) == 0);
    TEST_CHECK(!SPIUnregister(a) && !SPIUnregister(b) && SPISetMode(a, SPI_BLOCK_MODE) == HAL_BUSY);
    TEST_CHECK(submit(c, 1) == HAL_OK && leases == 2u);
    SPI_HandleTypeDef alias = handles[0]; HAL_SPI_TxRxCpltCallback(&alias);
    HAL_SPI_TxCpltCallback(&handles[0]); TEST_CHECK(!state(a).buffers_released);
    complete(&handles[1], 1); TEST_CHECK(!state(a).buffers_released && leases == 1u);
    callback_checks = 1u; complete(&handles[0], 2); TEST_CHECK(leases == 0u);
    HAL_SPI_TxRxCpltCallback(&handles[0]); TEST_CHECK(callbacks == 2u);
    TEST_CHECK(submit(b, 2) == HAL_OK);
    HAL_SPI_ErrorCallback(&handles[0]);
    TEST_CHECK(state(a).phase == SPI_TRANSFER_SUCCEEDED && state(b).phase == SPI_TRANSFER_RECOVERING);
    TEST_CHECK(SPIProcessTimeouts() == HAL_OK && state(b).buffers_released && callbacks == 2u);
    TEST_CHECK(SPIUnregister(a) && SPIUnregister(b) && SPIUnregister(c));
}
static void wait_timeout(void)
{
    SPIInstance *a = device(0u, 1u, SPI_DMA_MODE), *b = device(0u, 2u, SPI_BLOCK_MODE);
    TEST_CHECK(submit(a, 2) == HAL_OK);
    SPIInstance before = *a;
    for (int op = 0; op < 3; ++op) TEST_CHECK(submit(b, op) == HAL_TIMEOUT);
    TEST_CHECK(calls == 1u && abort_calls == 0u && memcmp(a, &before, sizeof(before)) == 0);
    TEST_CHECK(HAL_GPIO_ReadPin(&gpio, 1u) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(&gpio, 2u) == GPIO_PIN_SET);
    complete(&handles[0], 2); TEST_CHECK(submit(b, 2) == HAL_OK);
    TEST_CHECK(SPIUnregister(a) && SPIUnregister(b));
}
static void timeout_test(uint8_t wrap)
{
    SPIInstance *a = device(0u, 1u, SPI_DMA_MODE), *b = device(1u, 2u, SPI_IT_MODE);
    uint32_t start = wrap ? UINT32_MAX - 10u : 0u; tick_ms = start;
    TEST_CHECK(submit(a, 2) == HAL_OK);
    tick_ms = start + 10u; TEST_CHECK(submit(b, 0) == HAL_OK);
    tick_ms = start + 19u; TEST_CHECK(SPIProcessTimeouts() == HAL_OK && abort_calls == 0u);
    tick_ms = start + 20u;
    TEST_CHECK(state(a).phase == SPI_TRANSFER_ACTIVE); // 查询不会替任务推进期限。
    abort_late_callback = 1u;
    TEST_CHECK(SPIProcessTimeouts() == HAL_OK && abort_calls == 1u && callbacks == 0u);
    TEST_CHECK(state(a).result == HAL_TIMEOUT && state(a).buffers_released && !state(b).buffers_released);
    TEST_CHECK(irq_clears[SPI1_IRQn] == 1u && irq_clears[0] == 1u && irq_clears[1] == 1u && dma_clears == 2u);
    complete(&handles[1], 0);
    TEST_CHECK(submit(a, 2) == HAL_OK); complete(&handles[0], 2);
    TEST_CHECK(callbacks == 1u && leases == 0u && SPIUnregister(a) && SPIUnregister(b));
}
static void failures(uint8_t isolate)
{
    SPIInstance *a = device(0u, 1u, SPI_DMA_MODE), *b = device(1u, 2u, SPI_IT_MODE);
    TEST_CHECK(submit(a, 2) == HAL_OK);
    HAL_SPI_ErrorCallback(&handles[0]);
    TEST_CHECK(state(a).phase == SPI_TRANSFER_RECOVERING && !state(a).buffers_released && abort_calls == 0u);
    HAL_SPI_TxRxCpltCallback(&handles[0]); TEST_CHECK(callbacks == 0u);
    abort_fail = isolate == 1u; abort_not_stopped = isolate == 2u;
    TEST_CHECK(SPIProcessTimeouts() == (isolate ? HAL_ERROR : HAL_OK));
    if (isolate)
    {
        TEST_CHECK(state(a).phase == SPI_TRANSFER_ISOLATED && !state(a).buffers_released);
        TEST_CHECK(submit(a, 2) == HAL_ERROR && !SPIUnregister(a));
        TEST_CHECK(SPIProcessTimeouts() == HAL_ERROR && abort_calls == 1u && leases == 1u);
        TEST_CHECK(submit(b, 2) == HAL_OK); complete(&handles[1], 2);
    }
    else
    {
        TEST_CHECK(state(a).result == HAL_ERROR && state(a).buffers_released && leases == 0u);
        TEST_CHECK(SPIUnregister(a));
    }
    TEST_CHECK(SPIUnregister(b));
}
static void early(void)
{
    SPIInstance *a = device(0u, 1u, SPI_DMA_MODE);
    early_complete = 1u;
    TEST_CHECK(submit(a, 2) == HAL_OK && callbacks == 1u && state(a).buffers_released);
    early_complete = 0u; early_error = 1u;
    TEST_CHECK(submit(a, 2) == HAL_ERROR && state(a).phase == SPI_TRANSFER_RECOVERING);
    early_error = 0u; TEST_CHECK(SPIProcessTimeouts() == HAL_OK);
    dirty_failure = 1u; hal_result = HAL_ERROR;
    TEST_CHECK(submit(a, 2) == HAL_ERROR && !state(a).buffers_released);
    TEST_CHECK(SPIProcessTimeouts() == HAL_OK);
    TEST_CHECK(SPISetMode(a, SPI_BLOCK_MODE) == HAL_OK);
    hal_result = HAL_TIMEOUT;
    TEST_CHECK(submit(a, 2) == HAL_TIMEOUT && state(a).buffers_released && leases == 0u);
    TEST_CHECK(abort_calls == 3u && SPIUnregister(a));
}
static void dirty_start(void)
{
    SPIInstance *a = device(0u, 1u, SPI_IT_MODE);
    const HAL_StatusTypeDef results[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    dirty_failure = 1u;
    // HAL 已启用部分硬件才失败时，所有操作都必须先停止才能复用缓冲区。
    for (int mode = 1; mode < 3; ++mode)
        for (int op = 0; op < 3; ++op)
            for (unsigned i = 0u; i < 3u; ++i)
            {
                TEST_CHECK(SPISetMode(a, (SPI_TXRX_MODE_e)mode) == HAL_OK);
                hal_result = results[i];
                TEST_CHECK(submit(a, op) == results[i] && !state(a).buffers_released);
                TEST_CHECK(state(a).phase == SPI_TRANSFER_RECOVERING);
                TEST_CHECK(SPIProcessTimeouts() == HAL_OK && state(a).buffers_released);
                TEST_CHECK(state(a).result == results[i] && leases == 0u && callbacks == 0u);
            }
    TEST_CHECK(abort_calls == 18u && allocations == 1u && SPIUnregister(a));
}
static void late_completion(void)
{
    SPI_Init_Config_s c = config(0u, 1u, SPI_IT_MODE);
    c.async_timeout_ms = 1u;
    SPIInstance *a = SPIRegister(&c); TEST_CHECK(a != NULL);
    TEST_CHECK(submit(a, 1) == HAL_OK && SPIProcessTimeouts() == HAL_OK && abort_calls == 0u);
    tick_ms = 1u; complete(&handles[0], 1);
    TEST_CHECK(callbacks == 0u && state(a).phase == SPI_TRANSFER_RECOVERING && state(a).result == HAL_TIMEOUT);
    TEST_CHECK(!state(a).buffers_released && SPIProcessTimeouts() == HAL_OK);
    TEST_CHECK(state(a).buffers_released && SPIUnregister(a));
    // 边界后一毫秒才收到的完成同样失败；新事务成功完成可恢复使用。
    a = device(0u, 1u, SPI_IT_MODE); tick_ms = 0u;
    TEST_CHECK(submit(a, 2) == HAL_OK); tick_ms = 21u; complete(&handles[0], 2);
    TEST_CHECK(callbacks == 0u && state(a).result == HAL_TIMEOUT);
    TEST_CHECK(SPIProcessTimeouts() == HAL_OK && submit(a, 2) == HAL_OK);
    complete(&handles[0], 2); TEST_CHECK(callbacks == 1u && SPIUnregister(a));
}
static void ws2812_dispatch(void)
{
    SPI_HandleTypeDef other = {0};
    ws2812_owned_handle = &other;
    HAL_SPI_TxCpltCallback(&other);
    HAL_SPI_ErrorCallback(&other);
    TEST_CHECK(ws2812_completions == 1u && ws2812_errors == 1u);
    // 同时验证 SPI1 的回调仍交给原事务，不被 SPI6 处理器吞掉。
    SPIInstance *a = device(0u, 1u, SPI_IT_MODE);
    TEST_CHECK(submit(a, 0) == HAL_OK);
    complete(&handles[0], 0);
    TEST_CHECK(state(a).phase == SPI_TRANSFER_SUCCEEDED && ws2812_completions == 1u);
    TEST_CHECK(SPIUnregister(a));
}
#ifdef SPI_TEST_THREADS
typedef struct { SPIInstance *instance; HAL_StatusTypeDef result; } ThreadCall;
static void *contend(void *argument)
{ ThreadCall *call = argument; call->result = submit(call->instance, 2); return NULL; }
static void *expire(void *argument) { (void)argument; (void)SPIProcessTimeouts(); return NULL; }
static void *finish_thread(void *argument) { HAL_SPI_TxRxCpltCallback(argument); return NULL; }
static void concurrency(void)
{
    SPIInstance *a = device(0u, 1u, SPI_IT_MODE), *b = device(0u, 2u, SPI_IT_MODE);
    // 多轮竞争；失败者不得改接收字段，完成与超时最多提交一次结果。
    for (unsigned i = 0u; i < 100u; ++i)
    {
        ThreadCall c[] = {{a, HAL_ERROR}, {b, HAL_ERROR}};
        pthread_t threads[2]; uint32_t before = calls;
        TEST_CHECK(pthread_create(&threads[0], NULL, contend, &c[0]) == 0);
        TEST_CHECK(pthread_create(&threads[1], NULL, contend, &c[1]) == 0);
        TEST_CHECK(pthread_join(threads[0], NULL) == 0 && pthread_join(threads[1], NULL) == 0);
        TEST_CHECK((c[0].result == HAL_OK) + (c[1].result == HAL_OK) == 1 && calls == before + 1u);
        TEST_CHECK(c[0].result == HAL_BUSY || c[1].result == HAL_BUSY);
        uint32_t before_callbacks = callbacks;
        tick_ms += i % 2u ? 20u : 19u; stopped(&handles[0]);
        TEST_CHECK(pthread_create(&threads[0], NULL, expire, NULL) == 0);
        TEST_CHECK(pthread_create(&threads[1], NULL, finish_thread, &handles[0]) == 0);
        TEST_CHECK(pthread_join(threads[0], NULL) == 0 && pthread_join(threads[1], NULL) == 0);
        TEST_CHECK(SPIProcessTimeouts() == HAL_OK && leases == 0u);
        TEST_CHECK(callbacks == before_callbacks + (i % 2u == 0u));
    }
    TEST_CHECK(allocations == 2u && SPIUnregister(a) && SPIUnregister(b));
}
#endif
int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    memset(irq_enabled, 1, sizeof(irq_enabled)); gpio.pins = UINT16_MAX;
    for (unsigned i = 0u; i < 2u; ++i)
    {
        handles[i].Instance = &spi_registers[i]; handles[i].Init.Mode = SPI_MODE_MASTER;
        handles[i].Init.DataSize = SPI_DATASIZE_8BIT;
        handles[i].hdmatx = &dmas[i * 2u]; handles[i].hdmarx = &dmas[i * 2u + 1u];
        for (unsigned j = 0u; j < 2u; ++j)
        {
            DMA_HandleTypeDef *dma = &dmas[i * 2u + j]; dma->Instance = &dma_registers[i * 2u + j];
            dma->Parent = &handles[i]; dma->Init.Direction = j ? DMA_PERIPH_TO_MEMORY : DMA_MEMORY_TO_PERIPH;
            dma->Init.MemInc = DMA_MINC_ENABLE;
        }
    }
    if (!strcmp(argv[1], "registration")) registration();
    else if (!strcmp(argv[1], "operations")) operations();
    else if (!strcmp(argv[1], "invalid")) invalid();
    else if (!strcmp(argv[1], "ownership")) ownership();
    else if (!strcmp(argv[1], "wait")) wait_timeout();
    else if (!strcmp(argv[1], "timeout")) timeout_test(0u);
    else if (!strcmp(argv[1], "wrap")) timeout_test(1u);
    else if (!strcmp(argv[1], "error")) failures(0u);
    else if (!strcmp(argv[1], "isolation")) failures(1u);
    else if (!strcmp(argv[1], "false-ready")) failures(2u);
    else if (!strcmp(argv[1], "early")) early();
    else if (!strcmp(argv[1], "dirty-start")) dirty_start();
    else if (!strcmp(argv[1], "late")) late_completion();
    else if (!strcmp(argv[1], "ws2812-dispatch")) ws2812_dispatch();
#ifdef SPI_TEST_THREADS
    else if (!strcmp(argv[1], "threads")) concurrency();
#endif
    else TEST_CHECK(0);
    return EXIT_SUCCESS;
}
