/** @file task_monitor_progress_test.c
 * @brief 用生产任务监控实现检查配置、任务进度、总期限及核心／辅助故障处理。
 * @note 测试替换 HAL 时钟、RTOS 等待和安全寄存器，不连接开发板。
 */
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "task_monitor.h"
#include "test_check.h"

GPIO_TypeDef mc02_test_gpiob;
GPIO_TypeDef mc02_test_gpioc;
TIM_TypeDef mc02_test_tim3;
RCC_TypeDef mc02_test_rcc;
uint32_t mc02_test_irq_disabled;
uint32_t mc02_test_primask;
uint32_t mc02_test_tick;
uint32_t mc02_test_ipsr;

static uint32_t watchdog_feed_count;
static uint32_t task_health_calls;
static uint8_t reported_task_health;
static uint32_t degrade_count;
static uint32_t degraded_id;
static uint32_t degraded_mask;
static uint32_t rtos_delay_count;
static uint8_t scheduler_running;

uint8_t osKernelRunning(void)
{
    return scheduler_running;
}

void osDelay(uint32_t milliseconds)
{
    rtos_delay_count++;
    mc02_test_tick += milliseconds;
}

void DWT_Delay(float seconds)
{
    mc02_test_tick += (uint32_t)(seconds * 1000.0f);
}

void BSP_WatchdogFeed(void)
{
    watchdog_feed_count++;
}

void RobotSafetySetTaskHealthy(uint8_t healthy)
{
    task_health_calls++;
    reported_task_health = healthy;
}

void MC02TestLogError(void)
{
}

static void DegradeDependency(TaskMonitor_Id_e id, uint32_t mask, void *context)
{
    uint32_t *callback_count = (uint32_t *)context;
    (*callback_count)++;
    degraded_id = (uint32_t)id;
    degraded_mask = mask;
}

static TaskMonitor_Result_e InitConfiguredForTest(const TaskMonitorTaskConfig_t *configs,
                                                  uint32_t count)
{
    // 生产初始化要求调度器尚未运行；测试在调用后再模拟任务运行期。
    scheduler_running = 0u;
    const TaskMonitor_Result_e result = TaskMonitorInitConfigured(configs, count);
    scheduler_running = 1u;
    return result;
}

static uint8_t OutputsAreOff(void)
{
    const uint32_t power_pins = POWER_24V_1_Pin | POWER_24V_2_Pin | POWER_5V_Pin;
    return (uint8_t)(mc02_test_gpiob.BSRR == (1u << 17u) &&
                     mc02_test_gpioc.BSRR == (power_pins << 16u) &&
                     mc02_test_tim3.CCR4 == 0u &&
                     (mc02_test_tim3.CCER & TIM_CCER_CC4E) == 0u &&
                     (mc02_test_tim3.CR1 & TIM_CR1_CEN) == 0u);
}

static const TaskMonitorTaskConfig_t core_and_event_configs[] = {
    {.id = TASK_MONITOR_INS,
     .name = "CORE_TEST",
     .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE,
     .period_ms = 5u,
     .deadline_ms = 10u,
     .startup_grace_ms = 5u},
    {.id = TASK_MONITOR_MOTOR,
     .name = "CORE_MOTOR_TEST",
     .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE,
     .period_ms = 5u,
     .deadline_ms = 10u,
     .startup_grace_ms = 5000u},
    {.id = TASK_MONITOR_OPTIONAL_0,
     .name = "EVENT_TEST",
     .mode = TASK_MONITOR_MODE_EVENT,
     .role = TASK_MONITOR_ROLE_AUXILIARY,
     .period_ms = 0u,
     .deadline_ms = 20u,
     .startup_grace_ms = 5u,
     .dependency_mask = 0x04u,
     .degrade_callback = DegradeDependency,
     .callback_context = &degrade_count},
};

static const TaskMonitorTaskConfig_t event_only_config = {
    .id = TASK_MONITOR_OPTIONAL_1,
    .name = "EVENT_ONLY",
    .mode = TASK_MONITOR_MODE_EVENT,
    .role = TASK_MONITOR_ROLE_AUXILIARY,
    .period_ms = 0u,
    .deadline_ms = 10u,
    .startup_grace_ms = 5u,
    .dependency_mask = 0x20u,
    .degrade_callback = DegradeDependency,
    .callback_context = &degrade_count,
};

static void ResetFakes(uint32_t now)
{
    mc02_test_gpiob = (GPIO_TypeDef){0};
    mc02_test_gpioc = (GPIO_TypeDef){0};
    mc02_test_tim3 = (TIM_TypeDef){0};
    mc02_test_rcc = (RCC_TypeDef){0};
    mc02_test_irq_disabled = 0u;
    mc02_test_primask = 0u;
    mc02_test_ipsr = 0u;
    mc02_test_tick = now;
    watchdog_feed_count = 0u;
    task_health_calls = 0u;
    reported_task_health = 1u;
    degrade_count = 0u;
    degraded_id = UINT32_MAX;
    degraded_mask = 0u;
    rtos_delay_count = 0u;
    scheduler_running = 0u;
}

int main(void)
{
    ResetFakes(100u);
    TaskMonitorProgressState_t before = {.begin_count = 77u};
    TEST_CHECK(TaskMonitorGetProgressState(TASK_MONITOR_INS, &before) ==
               TASK_MONITOR_RESULT_NOT_INITIALIZED);
    TEST_CHECK(before.begin_count == 77u);

    TEST_CHECK(InitConfiguredForTest(core_and_event_configs, 3u) == TASK_MONITOR_RESULT_OK);
    TaskMonitorProgressState_t core_state = {0};
    TEST_CHECK(TaskMonitorGetProgressState(TASK_MONITOR_INS, &core_state) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(core_state.mode == TASK_MONITOR_MODE_PERIODIC);
    TEST_CHECK(core_state.state == TASK_MONITOR_STATE_NOT_STARTED);
    TEST_CHECK(core_state.started == 0u && core_state.has_completed == 0u);

    // 调度器已运行时拒绝重置，不能借重初始化清除活动状态或锁存故障。
    TEST_CHECK(TaskMonitorInitConfigured(core_and_event_configs, 3u) ==
               TASK_MONITOR_RESULT_SCHEDULER_RUNNING);
    TaskMonitorTaskConfig_t invalid_duplicate[2] = {
        core_and_event_configs[0], core_and_event_configs[0]};
    TEST_CHECK(InitConfiguredForTest(invalid_duplicate, 2u) ==
               TASK_MONITOR_RESULT_BAD_CONFIGURATION);
    TaskMonitorProgressState_t after_invalid_init = {0};
    TEST_CHECK(TaskMonitorGetProgressState(TASK_MONITOR_INS, &after_invalid_init) ==
               TASK_MONITOR_RESULT_OK);
    TEST_CHECK(after_invalid_init.state == TASK_MONITOR_STATE_NOT_STARTED);

    mc02_test_ipsr = 1u;
    TEST_CHECK(TaskMonitorBeginWork(TASK_MONITOR_INS) == TASK_MONITOR_RESULT_CONTEXT);
    TEST_CHECK(TaskMonitorReportProgress(TASK_MONITOR_INS, 1u) == TASK_MONITOR_RESULT_CONTEXT);
    TEST_CHECK(TaskMonitorSetWaiting(TASK_MONITOR_INS, TASK_MONITOR_WAIT_EVENT) ==
               TASK_MONITOR_RESULT_CONTEXT);
    TEST_CHECK(TaskMonitorCompleteWork(TASK_MONITOR_INS) == TASK_MONITOR_RESULT_CONTEXT);
    mc02_test_ipsr = 0u;

    // 事件任务已经进入等待状态；等待很久也不会因没有新事件而超时。
    TEST_CHECK(TaskMonitorSetWaiting(TASK_MONITOR_OPTIONAL_0,
                                     TASK_MONITOR_WAIT_EVENT) == TASK_MONITOR_RESULT_OK);
    TaskMonitorProgressState_t waiting_state = {0};
    TEST_CHECK(TaskMonitorGetProgressState(TASK_MONITOR_OPTIONAL_0, &waiting_state) ==
               TASK_MONITOR_RESULT_OK);
    TEST_CHECK(waiting_state.wait_reason == TASK_MONITOR_WAIT_EVENT);
    TEST_CHECK(waiting_state.waiting_since_ms == 100u);
    mc02_test_tick = 105u;
    TaskMonitorTick();
    TEST_CHECK(TaskMonitorAllAlive() == 1u);
    TEST_CHECK(watchdog_feed_count == 1u);
    TEST_CHECK(degrade_count == 0u);
    mc02_test_tick = 106u;
    TaskMonitorTick();
    TEST_CHECK(TaskMonitorAllAlive() == 0u);
    TEST_CHECK(watchdog_feed_count == 1u);
    TEST_CHECK(reported_task_health == 0u);
    TEST_CHECK(OutputsAreOff() != 0u);

    // 工作中的重复进度只记录阶段；固定总期限仍从 BeginWork 时计算。
    ResetFakes(200u);
    TEST_CHECK(InitConfiguredForTest(core_and_event_configs, 3u) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(TaskMonitorSetWaiting(TASK_MONITOR_OPTIONAL_0,
                                     TASK_MONITOR_WAIT_EVENT) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(TaskMonitorBeginWork(TASK_MONITOR_INS) == TASK_MONITOR_RESULT_OK);
    for (uint32_t step = 1u; step <= 6u; ++step)
    {
        mc02_test_tick += 2u;
        TEST_CHECK(TaskMonitorReportProgress(TASK_MONITOR_INS, step) == TASK_MONITOR_RESULT_OK);
    }
    TaskMonitorTick();
    TEST_CHECK(TaskMonitorAllAlive() == 0u);
    TEST_CHECK(watchdog_feed_count == 0u);
    TEST_CHECK(reported_task_health == 0u);
    TEST_CHECK(OutputsAreOff() != 0u);
    TEST_CHECK(TaskMonitorBeginWork(TASK_MONITOR_INS) == TASK_MONITOR_RESULT_BUSY);
    TEST_CHECK(TaskMonitorBeginWork(TASK_MONITOR_MOTOR) == TASK_MONITOR_RESULT_BUSY);
    TEST_CHECK(TaskMonitorCompleteWork(TASK_MONITOR_INS) == TASK_MONITOR_RESULT_NOT_RUNNING);

    // 恰好到期限仍接受完成；周期任务随后必须按期限开始下一轮。
    ResetFakes(300u);
    TEST_CHECK(InitConfiguredForTest(core_and_event_configs, 3u) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(TaskMonitorSetWaiting(TASK_MONITOR_OPTIONAL_0,
                                     TASK_MONITOR_WAIT_EVENT) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(TaskMonitorBeginWork(TASK_MONITOR_INS) == TASK_MONITOR_RESULT_OK);
    mc02_test_tick = 310u;
    TEST_CHECK(TaskMonitorCompleteWork(TASK_MONITOR_INS) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(TaskMonitorGetProgressState(TASK_MONITOR_INS, &core_state) ==
               TASK_MONITOR_RESULT_OK);
    TEST_CHECK(core_state.job_started_ms == 300u);
    TEST_CHECK(core_state.last_completed_ms == 310u);
    TEST_CHECK(core_state.complete_count == 1u);
    TEST_CHECK(core_state.has_completed != 0u);
    mc02_test_tick = 320u;
    TaskMonitorTick();
    TEST_CHECK(TaskMonitorAllAlive() == 1u);
    const uint32_t completed_count_before_query = core_state.complete_count;
    TEST_CHECK(TaskMonitorGetProgressState(TASK_MONITOR_INS, &core_state) ==
               TASK_MONITOR_RESULT_OK);
    TEST_CHECK(core_state.complete_count == completed_count_before_query);
    mc02_test_tick = 321u;
    TaskMonitorTick();
    TEST_CHECK(TaskMonitorAllAlive() == 0u);

    // 辅助事件空闲无期限；事件开始后期限不随进度续期，故障只降级配置依赖。
    ResetFakes(400u);
    TEST_CHECK(InitConfiguredForTest(&event_only_config, 1u) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(TaskMonitorSetWaiting(TASK_MONITOR_OPTIONAL_1,
                                     TASK_MONITOR_WAIT_EVENT) == TASK_MONITOR_RESULT_OK);
    mc02_test_tick = 4400u;
    TaskMonitorTick();
    TEST_CHECK(TaskMonitorAllAlive() == 1u);
    TEST_CHECK(watchdog_feed_count == 1u);
    TEST_CHECK(TaskMonitorBeginWork(TASK_MONITOR_OPTIONAL_1) == TASK_MONITOR_RESULT_OK);
    for (uint32_t step = 0u; step < 4u; ++step)
    {
        mc02_test_tick += 3u;
        TEST_CHECK(TaskMonitorReportProgress(TASK_MONITOR_OPTIONAL_1, step + 1u) ==
                   TASK_MONITOR_RESULT_OK);
    }
    TaskMonitorTick();
    TEST_CHECK(TaskMonitorAllAlive() == 1u);
    TEST_CHECK(watchdog_feed_count == 2u);
    TEST_CHECK(degrade_count == 1u);
    TEST_CHECK(degraded_id == TASK_MONITOR_OPTIONAL_1);
    TEST_CHECK(degraded_mask == 0x20u);
    uint32_t degraded = 0u;
    TEST_CHECK(TaskMonitorGetDegradedDependencies(&degraded) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(degraded == 0x20u);
    TaskMonitorProgressState_t auxiliary_state = {0};
    TEST_CHECK(TaskMonitorGetProgressState(TASK_MONITOR_OPTIONAL_1, &auxiliary_state) ==
               TASK_MONITOR_RESULT_OK);
    TEST_CHECK(auxiliary_state.state == TASK_MONITOR_STATE_FAULTED);
    TEST_CHECK(auxiliary_state.fault_reason == TASK_MONITOR_FAULT_WORK_DEADLINE);

    // 无符号时差同时覆盖 tick 回绕；完成迟到一毫秒会立即关断核心输出。
    ResetFakes(UINT32_MAX - 5u);
    TaskMonitorTaskConfig_t wrap_config = core_and_event_configs[0];
    wrap_config.startup_grace_ms = 10u;
    TEST_CHECK(InitConfiguredForTest(&wrap_config, 1u) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(TaskMonitorBeginWork(TASK_MONITOR_INS) == TASK_MONITOR_RESULT_OK);
    mc02_test_tick = 4u;
    TEST_CHECK(TaskMonitorCompleteWork(TASK_MONITOR_INS) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(TaskMonitorBeginWork(TASK_MONITOR_INS) == TASK_MONITOR_RESULT_OK);
    mc02_test_tick = 15u;
    TEST_CHECK(TaskMonitorCompleteWork(TASK_MONITOR_INS) == TASK_MONITOR_RESULT_BUSY);
    TEST_CHECK(TaskMonitorAllAlive() == 0u);
    TEST_CHECK(reported_task_health == 0u);
    TEST_CHECK(OutputsAreOff() != 0u);

    // 配置到调度器启动之间的耗时不应消耗任务自己的启动宽限。
    ResetFakes(600u);
    const TaskMonitorTaskConfig_t startup_configs[] = {
        {.id = TASK_MONITOR_INS,
         .name = "STARTED_CORE",
         .mode = TASK_MONITOR_MODE_PERIODIC,
         .role = TASK_MONITOR_ROLE_CORE,
         .period_ms = 5u,
         .deadline_ms = 10u,
         .startup_grace_ms = 5u},
        {.id = TASK_MONITOR_MOTOR,
         .name = "LATE_CORE",
         .mode = TASK_MONITOR_MODE_PERIODIC,
         .role = TASK_MONITOR_ROLE_CORE,
         .period_ms = 5u,
         .deadline_ms = 10u,
         .startup_grace_ms = 5u},
    };
    TEST_CHECK(TaskMonitorInitConfigured(startup_configs, 2u) == TASK_MONITOR_RESULT_OK);
    mc02_test_tick += 1000u;
    scheduler_running = 1u;
    TEST_CHECK(TaskMonitorBeginWork(TASK_MONITOR_INS) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(TaskMonitorCompleteWork(TASK_MONITOR_INS) == TASK_MONITOR_RESULT_OK);
    mc02_test_tick += 5u;
    TaskMonitorTick();
    TEST_CHECK(TaskMonitorAllAlive() == 1u);
    mc02_test_tick += 1u;
    TaskMonitorTick();
    TEST_CHECK(TaskMonitorAllAlive() == 0u);
    TEST_CHECK(OutputsAreOff() != 0u);

    // 多次有界等待会产生进度记录，但不能把仍未完成的周期工作续期。
    ResetFakes(500u);
    TaskMonitorTaskConfig_t delay_config = core_and_event_configs[0];
    delay_config.startup_grace_ms = 100u;
    TEST_CHECK(InitConfiguredForTest(&delay_config, 1u) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(TaskMonitorBeginWork(TASK_MONITOR_INS) == TASK_MONITOR_RESULT_OK);
    TaskMonitorDelayMs(TASK_MONITOR_INS, 11u);
    TEST_CHECK(rtos_delay_count == 11u);
    TEST_CHECK(TaskMonitorGetProgressState(TASK_MONITOR_INS, &core_state) ==
               TASK_MONITOR_RESULT_OK);
    TEST_CHECK(core_state.job_started_ms == 500u);
    TEST_CHECK(core_state.progress_count == 11u);
    TaskMonitorTick();
    TEST_CHECK(TaskMonitorAllAlive() == 0u);

    puts("PASS: configured task progress, fixed deadlines, event idle, and role-specific faults");
    return 0;
}
