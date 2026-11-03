/** @file ws2812_pin_ownership_test.c
 * @brief 以 HAL 模型检查 SPI6／BDMA 的数据归属、完成和失败隔离。
 * @note 每个场景独立进程；不模拟 LED 电平、BDMA 物理时序或板上波形。
 */
#include "test_check.h"
#include "ws2812.h"
#include "led.h"
#include "robot_alarm_policy.h"
#include <stddef.h>
#include <string.h>
#ifdef WS2812_TEST_THREADS
#include <pthread.h>
static pthread_mutex_t irq_mutex = PTHREAD_MUTEX_INITIALIZER;
static _Thread_local uint32_t primask;
#else
static uint32_t primask;
#endif

GPIO_TypeDef fake_gpioa;
SPI_TypeDef fake_spi6;
BDMA_Channel_TypeDef fake_bdma0;
SCB_Type fake_scb;
static uint32_t tick_ms, ipsr, pins;
static unsigned spi_init_calls, dma_init_calls, starts, aborts, dma_aborts, dma_disables, leases, releases;
static unsigned enabled_irqs, disabled_irqs, cleared_irqs;
static uint8_t tick_available = 1u;
static uint8_t early_completion;
static uint8_t early_error;
static uint8_t dma_disable_stuck;
static HAL_StatusTypeDef start_result = HAL_OK, abort_result = HAL_OK, dma_abort_result = HAL_OK;
static SPI_HandleTypeDef *active_spi;
static const uint8_t *active_buffer;
static uint8_t captured[124];
static void Complete(void);
void BDMA_Channel0_IRQHandler(void);

uint8_t BSP_HALTickTryAcquireBlockingWait(void)
{
    if (tick_available == 0u) return 0u;
    ++leases;
    return 1u;
}
void BSP_HALTickReleaseBlockingWait(void) { ++releases; }
uint32_t HAL_GetTick(void) { return tick_ms; }
uint32_t __get_IPSR(void) { return ipsr; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void)
{
#ifdef WS2812_TEST_THREADS
    if (primask == 0u) TEST_CHECK(pthread_mutex_lock(&irq_mutex) == 0);
#endif
    primask = 1u;
}
void __set_PRIMASK(uint32_t value)
{
    const uint32_t previous = primask;
    primask = value;
#ifdef WS2812_TEST_THREADS
    if (previous != 0u && value == 0u) TEST_CHECK(pthread_mutex_unlock(&irq_mutex) == 0);
#else
    (void)previous;
#endif
}
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config)
{
    TEST_CHECK(config != NULL && config->PeriphClockSelection == RCC_PERIPHCLK_SPI6);
    TEST_CHECK(config->Spi6ClockSelection == RCC_SPI6CLKSOURCE_HSE);
    return HAL_OK;
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config)
{
    TEST_CHECK(port == GPIOA && config != NULL);
    TEST_CHECK(config->Mode == GPIO_MODE_AF_PP && config->Alternate == GPIO_AF8_SPI6);
    pins |= config->Pin;
}
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *handle)
{
    TEST_CHECK(handle != NULL && handle->Instance == SPI6);
    ++spi_init_calls;
    handle->State = HAL_SPI_STATE_READY;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *handle)
{
    TEST_CHECK(handle != NULL && handle->Instance == BDMA_Channel0);
    TEST_CHECK(handle->Init.Request == BDMA_REQUEST_SPI6_TX && handle->Init.Mode == DMA_NORMAL);
    TEST_CHECK(handle->Init.Direction == DMA_MEMORY_TO_PERIPH && handle->Init.MemInc == DMA_MINC_ENABLE);
    ++dma_init_calls;
    handle->State = HAL_DMA_STATE_READY;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *handle, const uint8_t *data, uint16_t size)
{
    TEST_CHECK(handle != NULL && handle->Instance == SPI6 && handle->hdmatx != NULL);
    TEST_CHECK(handle->hdmatx->Parent == handle && data != NULL && size == sizeof(captured));
    ++starts;
    if (start_result != HAL_OK) return start_result;
    active_spi = handle;
    active_buffer = data;
    memcpy(captured, data, size);
    handle->State = HAL_SPI_STATE_BUSY_TX;
    handle->hdmatx->State = HAL_DMA_STATE_BUSY;
    fake_spi6.CR1 = SPI_CR1_SPE | SPI_CR1_CSTART;
    fake_spi6.CFG1 = SPI_CFG1_TXDMAEN;
    fake_spi6.IER = SPI_IT_EOT;
    fake_bdma0.CCR = BDMA_CCR_EN;
    if (early_completion != 0u) Complete();
    if (early_error != 0u) TEST_CHECK(WS2812_OnError(handle) == 1u);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *handle)
{
    ++aborts;
    TEST_CHECK((fake_spi6.CR1 & SPI_CR1_CSTART) == 0u && fake_bdma0.CCR == 0u);
    TEST_CHECK(handle->hdmatx->State == HAL_DMA_STATE_READY);
    if (abort_result != HAL_OK) return abort_result;
    handle->State = HAL_SPI_STATE_READY;
    handle->hdmatx->State = HAL_DMA_STATE_READY;
    fake_spi6.CR1 = fake_spi6.CFG1 = fake_spi6.IER = fake_bdma0.CCR = 0u;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *handle)
{
    ++dma_aborts;
    TEST_CHECK(handle->XferAbortCallback == NULL);
    if (dma_abort_result != HAL_OK) return dma_abort_result;
    handle->State = HAL_DMA_STATE_READY;
    fake_bdma0.CCR = 0u;
    return HAL_OK;
}
void WS2812_TestDisableDMA(DMA_HandleTypeDef *handle)
{
    TEST_CHECK(handle->Instance == BDMA_Channel0);
    ++dma_disables;
    if (dma_disable_stuck == 0u) fake_bdma0.CCR &= ~BDMA_CCR_EN;
}
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *handle) { return handle->State; }
uint32_t HAL_DMA_GetState(DMA_HandleTypeDef *handle) { return handle->State; }
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *handle) { (void)handle; }
void HAL_SPI_IRQHandler(SPI_HandleTypeDef *handle) { (void)handle; }
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority)
{ TEST_CHECK((irq == SPI6_IRQn || irq == BDMA_Channel0_IRQn) && priority == 5u && subpriority == 0u); }
void HAL_NVIC_ClearPendingIRQ(int irq) { (void)irq; ++cleared_irqs; }
void HAL_NVIC_EnableIRQ(int irq) { (void)irq; ++enabled_irqs; }
void HAL_NVIC_DisableIRQ(int irq) { (void)irq; ++disabled_irqs; }

static void Complete(void)
{
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    TEST_CHECK(active_spi != NULL);
    active_spi->State = HAL_SPI_STATE_READY;
    active_spi->hdmatx->State = HAL_DMA_STATE_READY;
    fake_spi6.CR1 = fake_spi6.CFG1 = fake_spi6.IER = 0u;
    TEST_CHECK(WS2812_OnTxComplete(active_spi) == 1u);
    __set_PRIMASK(saved);
}

static void Init(void)
{
    TEST_CHECK(WS2812_Init() == HAL_OK && WS2812_Init() == HAL_OK);
    TEST_CHECK(spi_init_calls == 1u && dma_init_calls == 1u);
    TEST_CHECK(pins == GPIO_PIN_7 && (pins & GPIO_PIN_5) == 0u);
    TEST_CHECK(enabled_irqs == 2u && cleared_irqs == 2u);
}

static void TestSuccess(void)
{
    Init();
    tick_available = 0u;
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_BUSY && starts == 0u);
    tick_available = 1u;
    TEST_CHECK(WS2812_SetRGB(0x80u, 0u, 1u) == HAL_BUSY && starts == 1u);
    WS2812TransferState_t state;
    TEST_CHECK(WS2812_GetState(&state) == HAL_OK && state.phase == WS2812_TRANSFER_ACTIVE &&
               state.buffer_reusable == 0u);
    TEST_CHECK(((uintptr_t)active_buffer % 32u) == 0u);
    TEST_CHECK(captured[8] == 0x78u && captured[23] == 0x78u);
    for (uint16_t i = 24u; i < 124u; ++i) TEST_CHECK(captured[i] == 0u);
    TEST_CHECK(WS2812_SetRGB(3u, 2u, 1u) == HAL_BUSY && starts == 1u);
    TEST_CHECK(memcmp(active_buffer, captured, sizeof(captured)) == 0);
    // BDMA 搬运结束后 SPI 仍可能在发送最后一个字节，不能提前确认。
    active_spi->hdmatx->State = HAL_DMA_STATE_READY;
    BDMA_Channel0_IRQHandler();
    TEST_CHECK(WS2812_Process() == HAL_BUSY && releases == 0u);
    Complete();
    TEST_CHECK(WS2812_Process() == HAL_OK && leases == 1u && releases == 1u && dma_disables == 1u);
    TEST_CHECK(WS2812_GetState(&state) == HAL_OK && state.phase == WS2812_TRANSFER_IDLE &&
               state.last_result == HAL_OK && state.buffer_reusable == 1u);
    TEST_CHECK(WS2812_SetRGB(0x80u, 0u, 1u) == HAL_OK && starts == 1u);
    TEST_CHECK(WS2812_SetRGB(3u, 2u, 1u) == HAL_BUSY && starts == 2u);
    Complete();
    TEST_CHECK(WS2812_Process() == HAL_OK && leases == 2u && releases == 2u);
}

static void TestTimeout(uint8_t isolate)
{
    Init();
    tick_ms = UINT32_MAX - 9u;
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_BUSY);
    tick_ms += 19u;
    TEST_CHECK(WS2812_Process() == HAL_BUSY && aborts == 0u);
    ++tick_ms;
    TEST_CHECK(WS2812_Process() == HAL_BUSY && dma_aborts == 1u && aborts == 0u);
    if (isolate != 0u) tick_ms += 20u;
    else fake_spi6.CR1 &= ~SPI_CR1_CSTART;
    TEST_CHECK(WS2812_Process() == HAL_ERROR && aborts == (isolate != 0u ? 0u : 1u));
    WS2812TransferState_t state;
    TEST_CHECK(WS2812_GetState(&state) == HAL_OK && state.last_result == HAL_TIMEOUT);
    TEST_CHECK(state.phase == (isolate != 0u ? WS2812_TRANSFER_ISOLATED : WS2812_TRANSFER_FAILED));
    TEST_CHECK(state.buffer_reusable == (isolate != 0u ? 0u : 1u));
    TEST_CHECK(disabled_irqs == 2u && releases == 1u);
    TEST_CHECK(WS2812_OnTxComplete(active_spi) == 1u);
    TEST_CHECK(WS2812_SetRGB(4u, 5u, 6u) == HAL_ERROR && starts == 1u);
}

static void TestStartAndError(void)
{
    Init();
    start_result = HAL_BUSY;
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_BUSY);
    TEST_CHECK(leases == 1u && releases == 1u);
    start_result = HAL_OK;
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_BUSY);
    TEST_CHECK(WS2812_OnError(active_spi) == 1u);
    TEST_CHECK(WS2812_Process() == HAL_BUSY && aborts == 0u);
    fake_spi6.CR1 &= ~SPI_CR1_CSTART;
    TEST_CHECK(WS2812_Process() == HAL_ERROR && aborts == 1u && releases == 2u);
}

static void TestDirty(void)
{
    Init();
    fake_bdma0.CCR = BDMA_CCR_EN;
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_ERROR && starts == 0u && leases == 0u);
    WS2812TransferState_t state;
    TEST_CHECK(WS2812_GetState(&state) == HAL_OK && state.phase == WS2812_TRANSFER_ISOLATED &&
               state.buffer_reusable == 0u);
}

static void TestCacheDuringTransfer(void)
{
    Init();
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_BUSY);
    fake_scb.CCR = SCB_CCR_DC_Msk;
    TEST_CHECK(WS2812_Process() == HAL_BUSY && aborts == 0u);
    fake_spi6.CR1 &= ~SPI_CR1_CSTART;
    TEST_CHECK(WS2812_Process() == HAL_ERROR && aborts == 1u && releases == 1u);
}

static void TestEarlyCompletion(void)
{
    Init();
    early_completion = 1u;
    TEST_CHECK(WS2812_SetRGB(7u, 8u, 9u) == HAL_BUSY);
    TEST_CHECK(WS2812_Process() == HAL_OK && releases == 1u);
    TEST_CHECK(WS2812_SetRGB(7u, 8u, 9u) == HAL_OK && starts == 1u);
    TEST_CHECK(WS2812_OnTxComplete(active_spi) == 1u);
    TEST_CHECK(WS2812_Process() == HAL_OK && releases == 1u);
    SPI_HandleTypeDef other = {0};
    TEST_CHECK(WS2812_OnTxComplete(&other) == 0u && WS2812_OnError(&other) == 0u);
}

static void TestEarlyError(void)
{
    Init();
    early_error = 1u;
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_BUSY);
    TEST_CHECK(WS2812_Process() == HAL_BUSY);
    fake_spi6.CR1 &= ~SPI_CR1_CSTART;
    TEST_CHECK(WS2812_Process() == HAL_ERROR && releases == 1u);
    WS2812TransferState_t state;
    TEST_CHECK(WS2812_GetState(&state) == HAL_OK && state.last_result == HAL_ERROR);
}

static void TestLateCompletion(void)
{
    Init();
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_BUSY);
    tick_ms = 20u;
    Complete();
    TEST_CHECK(WS2812_Process() == HAL_ERROR && aborts == 1u);
    WS2812TransferState_t state;
    TEST_CHECK(WS2812_GetState(&state) == HAL_OK && state.last_result == HAL_TIMEOUT);
}

static void TestFalseReady(void)
{
    Init();
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_BUSY);
    Complete();
    dma_disable_stuck = 1u;
    TEST_CHECK(WS2812_Process() == HAL_BUSY && aborts == 0u);
    tick_ms = 20u;
    TEST_CHECK(WS2812_Process() == HAL_BUSY && aborts == 0u && releases == 0u);
    tick_ms = 40u;
    TEST_CHECK(WS2812_Process() == HAL_ERROR && aborts == 0u && releases == 1u);
    WS2812TransferState_t state;
    TEST_CHECK(WS2812_GetState(&state) == HAL_OK && state.last_result == HAL_ERROR);
}

static void TestStartError(void)
{
    Init();
    start_result = HAL_ERROR;
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_BUSY);
    WS2812TransferState_t state;
    TEST_CHECK(WS2812_GetState(&state) == HAL_OK && state.phase == WS2812_TRANSFER_RECOVERING);
    TEST_CHECK(WS2812_Process() == HAL_ERROR && aborts == 1u && releases == 1u);
    TEST_CHECK(WS2812_GetState(&state) == HAL_OK && state.last_result == HAL_ERROR);
}

static void TestErrorAfterCompletion(void)
{
    Init();
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_BUSY);
    Complete();
    TEST_CHECK(WS2812_OnError(active_spi) == 1u);
    TEST_CHECK(WS2812_Process() == HAL_ERROR && aborts == 1u && releases == 1u);
}

static void TestRecoveryFailure(uint8_t dma_failure)
{
    Init();
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_BUSY);
    TEST_CHECK(WS2812_OnError(active_spi) == 1u);
    if (dma_failure != 0u)
    {
        dma_abort_result = HAL_ERROR;
        TEST_CHECK(WS2812_Process() == HAL_ERROR && aborts == 0u);
    }
    else
    {
        TEST_CHECK(WS2812_Process() == HAL_BUSY);
        fake_spi6.CR1 &= ~SPI_CR1_CSTART;
        abort_result = HAL_ERROR;
        TEST_CHECK(WS2812_Process() == HAL_ERROR && aborts == 1u);
    }
    WS2812TransferState_t state;
    TEST_CHECK(WS2812_GetState(&state) == HAL_OK && state.phase == WS2812_TRANSFER_ISOLATED &&
               state.buffer_reusable == 0u && releases == 1u);
    TEST_CHECK(WS2812_SetRGB(2u, 3u, 4u) == HAL_ERROR && starts == 1u);
}

static RobotAlarmWriteResult_t PolicyLed(void *context, uint8_t r, uint8_t g, uint8_t b)
{
    (void)context;
    const LEDStatus_t result = LEDSetRGB(r, g, b);
    return result == LED_STATUS_OK ? ROBOT_ALARM_WRITE_APPLIED :
           result == LED_STATUS_BUSY ? ROBOT_ALARM_WRITE_BUSY : ROBOT_ALARM_WRITE_ERROR;
}

static RobotAlarmWriteResult_t PolicySound(void *context, BuzzerNote_t note, uint16_t duty)
{
    (void)context; (void)note; (void)duty;
    return ROBOT_ALARM_WRITE_APPLIED;
}

static void TestPolicyIntegration(void)
{
    Init();
    TEST_CHECK(LEDInit() == LED_STATUS_OK);
    RobotAlarmFaultRule_t faults[9] = {0};
    RobotAlarmStateRule_t states[ROBOT_ALARM_STATE_COUNT] = {0};
    for (uint8_t i = 0u; i < 9u; ++i)
    {
        faults[i].fault_bit = 1u << i;
        faults[i].mute_level = i < 8u ? i : 7u;
        faults[i].style.led.mode = ROBOT_ALARM_LED_OFF;
    }
    for (uint8_t i = 0u; i < ROBOT_ALARM_STATE_COUNT; ++i)
    {
        states[i].state = (RobotAlarmRobotState_t)i;
        states[i].style.led.mode = ROBOT_ALARM_LED_OFF;
    }
    states[ROBOT_ALARM_STATE_INIT].style.led = (RobotAlarmLedPattern_t){
        .mode = ROBOT_ALARM_LED_BLINK, .period_ms = 20u, .b = 100u};
    const RobotAlarmConfig_t config = {
        .fault_rule_count = 9u, .fault_rules = faults,
        .state_rule_count = ROBOT_ALARM_STATE_COUNT, .state_rules = states,
        .unknown_fault_style = {.led = {.mode = ROBOT_ALARM_LED_OFF}},
        .unknown_state_style = {.led = {.mode = ROBOT_ALARM_LED_OFF}},
        .max_mute_ms = 100u, .max_pattern_ms = 100u,
    };
    const RobotAlarmOutputOps_t output = {.write_led = PolicyLed, .write_buzzer = PolicySound};
    const RobotAlarmInput_t input = {.state = ROBOT_ALARM_STATE_INIT};
    RobotAlarmPolicy_t policy;
    TEST_CHECK(RobotAlarmPolicyInit(&policy, &config, &output, 1u, 1u) == ROBOT_ALARM_OK);
    TEST_CHECK(RobotAlarmPolicyUpdate(&policy, &input, 0u) == ROBOT_ALARM_DEGRADED);
    Complete(); LEDProcess();
    TEST_CHECK(RobotAlarmPolicyUpdate(&policy, &input, 1u) == ROBOT_ALARM_OK);
    tick_ms = 10u;
    TEST_CHECK(RobotAlarmPolicyUpdate(&policy, &input, 10u) == ROBOT_ALARM_DEGRADED);
    tick_ms = 20u;
    TEST_CHECK(RobotAlarmPolicyUpdate(&policy, &input, 20u) == ROBOT_ALARM_DEGRADED);
    Complete(); LEDProcess();
    TEST_CHECK(RobotAlarmPolicyUpdate(&policy, &input, 20u) == ROBOT_ALARM_DEGRADED);
    TEST_CHECK(starts == 3u);
    Complete(); LEDProcess();
    TEST_CHECK(RobotAlarmPolicyUpdate(&policy, &input, 21u) == ROBOT_ALARM_OK);
    TEST_CHECK(leases == 3u && releases == 3u);
}

#ifdef WS2812_TEST_THREADS
static void *CompleteThread(void *argument)
{
    (void)argument;
    Complete();
    return NULL;
}

static void TestConcurrentCompletion(void)
{
    Init();
    for (uint8_t i = 1u; i <= 100u; ++i)
    {
        TEST_CHECK(WS2812_SetRGB(i, 0u, 0u) == HAL_BUSY);
        pthread_t thread;
        TEST_CHECK(pthread_create(&thread, NULL, CompleteThread, NULL) == 0);
        const HAL_StatusTypeDef result = WS2812_Process();
        TEST_CHECK(result == HAL_OK || result == HAL_BUSY);
        TEST_CHECK(pthread_join(thread, NULL) == 0);
        TEST_CHECK(WS2812_Process() == HAL_OK);
        TEST_CHECK(WS2812_SetRGB(i, 0u, 0u) == HAL_OK);
        TEST_CHECK(leases == releases);
    }
}
#endif

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "success") == 0) TestSuccess();
    else if (strcmp(argv[1], "timeout") == 0) TestTimeout(0u);
    else if (strcmp(argv[1], "isolation") == 0) TestTimeout(1u);
    else if (strcmp(argv[1], "error") == 0) TestStartAndError();
    else if (strcmp(argv[1], "dirty") == 0) TestDirty();
    else if (strcmp(argv[1], "cache-active") == 0) TestCacheDuringTransfer();
    else if (strcmp(argv[1], "early") == 0) TestEarlyCompletion();
    else if (strcmp(argv[1], "early-error") == 0) TestEarlyError();
    else if (strcmp(argv[1], "late") == 0) TestLateCompletion();
    else if (strcmp(argv[1], "false-ready") == 0) TestFalseReady();
    else if (strcmp(argv[1], "start-error") == 0) TestStartError();
    else if (strcmp(argv[1], "complete-error") == 0) TestErrorAfterCompletion();
    else if (strcmp(argv[1], "dma-stop-error") == 0) TestRecoveryFailure(1u);
    else if (strcmp(argv[1], "spi-stop-error") == 0) TestRecoveryFailure(0u);
    else if (strcmp(argv[1], "policy") == 0) TestPolicyIntegration();
#ifdef WS2812_TEST_THREADS
    else if (strcmp(argv[1], "threads") == 0) TestConcurrentCompletion();
#endif
    else if (strcmp(argv[1], "cache") == 0)
    {
        WS2812TransferState_t state = {WS2812_TRANSFER_ISOLATED, HAL_TIMEOUT, 17u};
        fake_scb.CCR = SCB_CCR_DC_Msk;
        TEST_CHECK(LEDInit() == LED_STATUS_TRANSFER); // 平台接口保留初始化错误。
        TEST_CHECK(LEDSetRGB(1u, 2u, 3u) == LED_STATUS_UNINITIALIZED);
        TEST_CHECK(WS2812_Init() == HAL_ERROR && WS2812_GetState(&state) == HAL_ERROR);
        TEST_CHECK(state.phase == WS2812_TRANSFER_ISOLATED && state.last_result == HAL_TIMEOUT &&
                   state.buffer_reusable == 17u);
        TEST_CHECK(WS2812_GetState(NULL) == HAL_ERROR);
    }
    else if (strcmp(argv[1], "resource-busy") == 0)
    {
        fake_bdma0.CCR = BDMA_CCR_EN;
        TEST_CHECK(LEDInit() == LED_STATUS_BUSY); // 资源被占用时仍返回忙。
        TEST_CHECK(WS2812_Init() == HAL_BUSY && spi_init_calls == 0u && pins == 0u);
        fake_bdma0.CCR = 0u;
        fake_spi6.CR1 = SPI_CR1_SPE;
        TEST_CHECK(WS2812_Init() == HAL_BUSY && spi_init_calls == 0u && pins == 0u);
    }
    else TEST_CHECK(0);
    return 0;
}
