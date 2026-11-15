/**
 * @file lk_motor_bsp_can_test.c
 * @brief 通过生产 LK 和 BSP CAN 验证注册回退、真实 CAN 帧头及跨总线发送。
 * @note HAL、时间与日志由测试替身提供；每种场景单独启动，测试不连接设备。
 */
#include "LK9025.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "bsp_can.h"
#include "../../bsp/can/platform_can.h"
#include "can_topic_transport.h"
#include "can_topic_link.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "test_check.h"
#include "motor_feedback_test_support.h"
#include "module_plan.h"
#include "robot_startup.h"
#include "ins_startup.h"
#include "task_monitor.h"
#include "init_diagnostic.h"
#include "robot_safety.h"
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* 测试按生产任务的顺序计算目标，再由唯一调度入口推进一次发送。 */
#define LKMotorControl() (LKMotorControl(), CANProcessTx())

static FDCAN_GlobalTypeDef fdcan_registers[DEVICE_CAN_CNT];
FDCAN_HandleTypeDef hfdcan1 = {.Init = {.StdFiltersNbr = 14u}, .Instance = &fdcan_registers[0]};
FDCAN_HandleTypeDef hfdcan2 = {.Init = {.StdFiltersNbr = 14u}, .Instance = &fdcan_registers[1]};
FDCAN_HandleTypeDef hfdcan3 = {.Init = {.StdFiltersNbr = 14u}, .Instance = &fdcan_registers[2]};
volatile uint32_t dwt_cpu_freq_mhz = 480u;
volatile uint32_t test_dwt_cycles;
volatile uint32_t test_ipsr;
uint32_t can_test_irq_mask, can_test_irq_sections;
static uint8_t verify_filter_interrupts;
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *handle);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *handle, uint32_t flags);
extern volatile DWT_Probe_t can_prof_rx_dispatch;

static uint64_t now_us;
static uint32_t filter_fail_once;
static uint32_t tx_busy_mask;
static uint32_t tx_error_mask;
static uint32_t bus_start_count[DEVICE_CAN_CNT];
static uint32_t bus_stop_count[DEVICE_CAN_CNT]; // 记录各总线的独立退出次数。
static uint32_t bus_init_count[DEVICE_CAN_CNT]; // 记录是否切换过控制器模式。
static uint32_t filter_count[DEVICE_CAN_CNT];
static uint32_t filter_rx_id[DEVICE_CAN_CNT];
static uint32_t filter_last_index[DEVICE_CAN_CNT]; // 记录实际使用的过滤器槽。
static uint32_t filter_last_type[DEVICE_CAN_CNT]; // 区分标准和扩展过滤器。
static uint32_t tx_attempt_count[DEVICE_CAN_CNT];
static uint32_t tx_accept_count[DEVICE_CAN_CNT];
static uint32_t tx_header_id[DEVICE_CAN_CNT];
static uint32_t tx_latest_buffer[DEVICE_CAN_CNT];
static uint8_t tx_latest_unknown; // 模拟 HAL 已接受但未返回可用槽位。
static uint8_t tx_next_buffer_index[DEVICE_CAN_CNT];
static uint8_t tx_frame[DEVICE_CAN_CNT][8];
static uint32_t tx_id_history[DEVICE_CAN_CNT][64];
static uint8_t tx_id_type_history[DEVICE_CAN_CNT][64];
static uint8_t tx_length_history[DEVICE_CAN_CNT][64];
static uint8_t tx_data_history[DEVICE_CAN_CNT][64][8];
typedef struct
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64]; // 模拟 HAL 最多复制一帧 CAN FD 的负载，验证目标缓冲容量。
} TestRxFrame_t;
static TestRxFrame_t rx_frames[DEVICE_CAN_CNT][64];
static uint8_t rx_frame_count[DEVICE_CAN_CNT];
static uint8_t rx_frame_read[DEVICE_CAN_CNT];
static TestRxFrame_t rx_fifo1_frames[DEVICE_CAN_CNT][64]; // 分开模拟每条总线的 FIFO1。
static uint8_t rx_fifo1_count[DEVICE_CAN_CNT], rx_fifo1_read[DEVICE_CAN_CNT];
static uint8_t inject_after_filter, pending_registration_irq;
static uint8_t rx_read_failures; // 受控注入接收失败，避免测试因错误循环无限运行。
static uint8_t rx_fifo1_failure; // 单独注入 FIFO1 读取失败。
static uint8_t rx_fifo_stuck; // 模拟成功读取但 FIFO 占用不下降。
static uint32_t rx_read_calls; // 记录 HAL 实际读取次数。
static uint32_t burst_rx_count[DEVICE_CAN_CNT];
static uint32_t burst_rx_bad[DEVICE_CAN_CNT];
static uint32_t burst_last_cycle[DEVICE_CAN_CNT];
static uint8_t status_busoff[DEVICE_CAN_CNT]; // 模拟各总线独立的关闭状态。
static uint8_t suppress_busoff_init; // 模拟总线关闭标志与 INIT 状态尚不一致。
static uint8_t status_error[DEVICE_CAN_CNT]; // 模拟各总线独立的错误状态。
static uint8_t status_read_failure; // 模拟 HAL 查询失败。
static uint8_t inject_submit_on_stop; // 在控制器停止期间模拟其他任务提交。
static CAN_TxSubmitStatus_t stop_submit_status; // 记录并发提交结果。
static uint8_t fail_recovery_stop, fail_recovery_start; // 注入停止或重新启动失败。
static uint8_t selftest_loopback_enabled, selftest_hook_busy; // 只在指定自测场景模拟中断。
static uint32_t selftest_completed[DEVICE_CAN_CNT]; // 记录每条总线上已模拟完成的测试帧数。

void CANTestProbeHook(void)
{
    if (!selftest_loopback_enabled || selftest_hook_busy) return; // 普通测试及中断重入不触发模拟。
    selftest_hook_busy = 1u; // 防止接收中断中的探针再次进入本钩子。
    FDCAN_HandleTypeDef *handles[DEVICE_CAN_CNT] = {&hfdcan1, &hfdcan2, &hfdcan3}; // 三条总线的测试句柄。
    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        if (selftest_completed[bus] >= tx_accept_count[bus]) continue; // 本总线没有新提交帧。
        const uint32_t index = selftest_completed[bus]++; // 先标记完成，避免中断递归重复处理。
        TEST_CHECK(index < 64u && tx_id_history[bus][index] == 0x123u); // 仅接受本测试的回环帧。
        HAL_FDCAN_TxBufferCompleteCallback(handles[bus], tx_latest_buffer[bus]); // 模拟硬件发送完成。
        TEST_CHECK(rx_frame_count[bus] < 64u); // 模拟 FIFO 保持在固定容量内。
        TestRxFrame_t *frame = &rx_frames[bus][rx_frame_count[bus]++]; // 添加同一帧的回环接收。
        frame->header.Identifier = tx_id_history[bus][index]; // 接收 ID 与发送 ID 一致。
        frame->header.DataLength = 8u << 16u; // 回环测试使用八字节经典帧。
        memcpy(frame->data, tx_data_history[bus][index], 8u); // 保留本次发送内容。
        HAL_FDCAN_RxFifo0Callback(handles[bus], FDCAN_IT_RX_FIFO0_NEW_MESSAGE); // 模拟硬件接收中断。
    }
    selftest_hook_busy = 0u; // 本轮中断模拟完成。
}

void CANTestRestoreIrq(uint32_t value)
{
    const uint32_t was_masked = can_test_irq_mask; // 保留恢复前的中断状态。
    can_test_irq_mask = value; // 先恢复调用方要求的状态。
    if (was_masked && !value && pending_registration_irq) // 注册发布后模拟等待中的接收中断。
    {
        pending_registration_irq = 0u; // 先清除标记，避免中断处理再次触发自身。
        HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); // 使用生产中断入口。
    }
}

#if defined(LK_TEST_ALLOC_WRAP)
static unsigned malloc_calls;
static unsigned free_calls;
void *__real_malloc(size_t size);
void *__real_calloc(size_t count, size_t size);
void __real_free(void *pointer);
void *__wrap_malloc(size_t size)
{
    ++malloc_calls;
    return __real_malloc(size);
}
void *__wrap_calloc(size_t count, size_t size)
{
    ++malloc_calls;
    return __real_calloc(count, size);
}
void __wrap_free(void *pointer)
{
    if (pointer != NULL)
        ++free_calls;
    __real_free(pointer);
}
#endif

uint64_t DWT_GetTimeline_us(void) { return now_us; }
float DWT_GetDeltaT(uint32_t *counter)
{
    if (counter != NULL)
        ++*counter;
    return 0.001f;
}
uint32_t HAL_GetTick(void) { return (uint32_t)(now_us / 1000u); }

static uint8_t bus_for_handle(const FDCAN_HandleTypeDef *handle)
{
    if (handle == &hfdcan1) return 0u;
    if (handle == &hfdcan2) return 1u;
    if (handle == &hfdcan3) return 2u;
    return UINT8_MAX;
}

static uint8_t test_dlc_to_length(uint32_t encoded_dlc)
{
    for (uint8_t length = 0u; length <= 8u; ++length)
        if (DLC_LookUp_Table[length] == encoded_dlc)
            return length;
    return UINT8_MAX;
}

uint32_t HAL_FDCAN_ConfigRxFifoOverwrite(FDCAN_HandleTypeDef *handle, uint32_t fifo, uint32_t mode)
{
    TEST_CHECK(handle != NULL && handle->Instance != NULL); // HAL 访问寄存器前句柄必须已准备。
    (void)fifo; (void)mode;
    return HAL_OK;
}
uint32_t HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *handle, uint32_t nonmatching_std,
                                      uint32_t nonmatching_ext, uint32_t remote_std, uint32_t remote_ext)
{
    (void)handle; (void)nonmatching_std; (void)nonmatching_ext; (void)remote_std; (void)remote_ext;
    return HAL_OK;
}
uint32_t HAL_FDCAN_Start(FDCAN_HandleTypeDef *handle)
{
    uint8_t bus = bus_for_handle(handle);
    if (bus < DEVICE_CAN_CNT)
        ++bus_start_count[bus];
    if (fail_recovery_start != 0u) return HAL_ERROR; // 模拟恢复期间启动失败。
    handle->State = 2u;
    handle->Instance->CCCR &= ~FDCAN_CCCR_INIT;
    return HAL_OK;
}
uint32_t HAL_FDCAN_Stop(FDCAN_HandleTypeDef *handle)
{
    const uint8_t bus = bus_for_handle(handle); // 找出本次停止的总线。
    if (bus < DEVICE_CAN_CNT) ++bus_stop_count[bus]; // 检查故障回退是否影响其他总线。
    if (fail_recovery_stop != 0u) return HAL_ERROR; // 停止失败时控制器状态保持不变。
    if (inject_submit_on_stop != 0u)
    {
        static const uint8_t data = 0x77u; // 模拟故障期间试图提交的旧目标。
        const CAN_TxFrame_t frame = {.identifier = 0x321u, .identifier_type = CAN_TX_ID_STANDARD,
                                     .length = 1u, .data = &data};
        uint32_t request = 0u; // 故障期间不能分配请求编号。
        stop_submit_status = CANSubmitOneShot(handle, &data, &frame, CAN_TX_PRIORITY_NORMAL, 500u, &request);
        TEST_CHECK(request == 0u); // 被拒绝的请求不能进入队列。
    }
    handle->State = 1u;
    handle->Instance->CCCR |= FDCAN_CCCR_INIT;
    return HAL_OK;
}
uint32_t HAL_FDCAN_Init(FDCAN_HandleTypeDef *handle)
{
    const uint8_t bus = bus_for_handle(handle); // 确认本次初始化所属总线。
    if (bus < DEVICE_CAN_CNT) ++bus_init_count[bus]; // 记录诊断是否触及控制器。
    handle->Instance->CCCR |= FDCAN_CCCR_INIT;
    handle->State = 1u;
    return HAL_OK;
}
uint32_t HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *handle, uint32_t mask, uint32_t buffer_indexes)
{
    (void)buffer_indexes;
    handle->Instance->IE = mask;
    return HAL_OK;
}
uint32_t HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *handle, FDCAN_FilterTypeDef *filter)
{
    if (verify_filter_interrupts) TEST_CHECK(can_test_irq_mask == 0u); // 硬件配置不应被登记临界区包围。
    uint8_t bus = bus_for_handle(handle);
    if (filter_fail_once != 0u)
    {
        filter_fail_once = 0u;
        return HAL_ERROR;
    }
    if (bus >= DEVICE_CAN_CNT)
        return HAL_ERROR;
    filter_last_index[bus] = filter->FilterIndex; // 保存 HAL 实际收到的槽位。
    filter_last_type[bus] = filter->IdType; // 保存硬件过滤器类型。
    if (filter->FilterConfig == FDCAN_FILTER_DISABLE) return HAL_OK; // 禁用不计作新增过滤器。
    if (inject_after_filter) pending_registration_irq = 1u; // 让下一次恢复中断时立刻接收。
    filter_rx_id[bus] = filter->FilterID1;
    ++filter_count[bus];
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *handle)
{
    uint8_t bus = bus_for_handle(handle);
    return bus < DEVICE_CAN_CNT && (tx_busy_mask & (1u << bus)) == 0u ? 1u : 0u;
}
uint32_t HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *handle,
                                       FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    uint8_t bus = bus_for_handle(handle);
    if (bus >= DEVICE_CAN_CNT)
        return HAL_ERROR;
    ++tx_attempt_count[bus];
    if ((tx_error_mask & (1u << bus)) != 0u)
        return HAL_ERROR;
    ++tx_accept_count[bus];
    const uint32_t history_index = tx_accept_count[bus] - 1u;
    if (history_index < 64u)
    {
        tx_id_history[bus][history_index] = header->Identifier;
        tx_id_type_history[bus][history_index] = header->IdType;
        tx_length_history[bus][history_index] = test_dlc_to_length(header->DataLength);
        memcpy(tx_data_history[bus][history_index], data, 8u);
    }
    tx_latest_buffer[bus] = 1u << tx_next_buffer_index[bus];
    tx_next_buffer_index[bus] = (uint8_t)((tx_next_buffer_index[bus] + 1u) % 16u);
    tx_header_id[bus] = header->Identifier;
    memcpy(tx_frame[bus], data, sizeof(tx_frame[bus]));
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(FDCAN_HandleTypeDef *handle)
{
    const uint8_t bus = bus_for_handle(handle);
    if (tx_latest_unknown != 0u) return 0u; // 保留硬件可能持有帧的不确定状态。
    return bus < DEVICE_CAN_CNT ? tx_latest_buffer[bus] : 0u;
}
uint32_t HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *handle, FDCAN_ProtocolStatusTypeDef *status)
{
    memset(status, 0, sizeof(*status));
    if (status_read_failure) return HAL_ERROR;
    uint8_t bus = bus_for_handle(handle);
    if (bus < DEVICE_CAN_CNT) {
        status->BusOff = status_busoff[bus];
        status->ErrorPassive = status_error[bus];
        if (status->BusOff && !suppress_busoff_init) handle->Instance->CCCR |= FDCAN_CCCR_INIT; // 通常 BusOff 后进入 INIT。
    }
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *handle, FDCAN_ErrorCountersTypeDef *counters)
{
    (void)handle;
    memset(counters, 0, sizeof(*counters));
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *handle, uint32_t fifo)
{
    uint8_t bus = bus_for_handle(handle);
    if (rx_fifo_stuck && bus == 0u && fifo == FDCAN_RX_FIFO0) return 1u; // 模拟异常时始终不为空。
    if (bus < DEVICE_CAN_CNT && fifo == FDCAN_RX_FIFO1)
        return (uint32_t)(rx_fifo1_count[bus] - rx_fifo1_read[bus]); // FIFO1 独立于 FIFO0。
    return bus < DEVICE_CAN_CNT ? (uint32_t)(rx_frame_count[bus] - rx_frame_read[bus]) : 0u;
}
uint32_t HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *handle, uint32_t fifo,
                                FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    ++rx_read_calls; // 每次调用都计数，验证失败后不会继续读取。
    if (rx_read_failures != 0u)
    {
        --rx_read_failures; // 本次失败保持 FIFO 中的原帧。
        return HAL_ERROR;
    }
    uint8_t bus = bus_for_handle(handle);
    if (rx_fifo_stuck && bus == 0u && fifo == FDCAN_RX_FIFO0)
    {
        memset(header, 0, sizeof(*header)); // 模拟 HAL 报告读取成功。
        memset(data, 0, 64u); // 清空输出，保留 FIFO 不下降的故障。
        return HAL_OK;
    }
    if (bus < DEVICE_CAN_CNT && fifo == FDCAN_RX_FIFO1)
    {
        if (rx_fifo1_failure != 0u) return HAL_ERROR; // FIFO0 成功后也可能在 FIFO1 失败。
        if (rx_fifo1_read[bus] >= rx_fifo1_count[bus]) return HAL_ERROR; // 空 FIFO 不可读取。
        TestRxFrame_t *frame = &rx_fifo1_frames[bus][rx_fifo1_read[bus]++]; // 只消费 FIFO1 的帧。
        *header = frame->header; // 复制被丢弃的帧头。
        memcpy(data, frame->data, sizeof(frame->data)); // 和实际 HAL 一样允许最大负载。
        return HAL_OK;
    }
    if (bus >= DEVICE_CAN_CNT || rx_frame_read[bus] >= rx_frame_count[bus])
        return HAL_ERROR;
    TestRxFrame_t *frame = &rx_frames[bus][rx_frame_read[bus]++];
    test_dwt_cycles += 480000u;
    *header = frame->header;
    memcpy(data, frame->data, sizeof(frame->data));
    return HAL_OK;
}

static uint32_t valid_rx_count;
static uint8_t valid_rx_length;
static void validate_rx_callback(CANInstance *instance)
{
    ++valid_rx_count; // 只统计实际交给模块的合法帧。
    valid_rx_length = instance->rx_len; // 记录零长度等有效边界。
}

static void test_registration_irq_state(void)
{
    CAN_Init_Config_s config = {0}; // 使用默认标准帧类型。
    config.can_handle = &hfdcan1; // 在第一条总线上登记。
    config.tx_id = 0x300u; // 选择测试发送 ID。
    config.rx_id = 0x321u; // 选择测试接收 ID。
    config.can_module_callback = validate_rx_callback; // 收到测试帧后可派发。
    can_test_irq_sections = 0u; // 单独统计本次注册的临界区。
    verify_filter_interrupts = 1u; // 检查硬件过滤器配置发生在中断开启时。
    inject_after_filter = 1u; // 过滤器配置后安排一次接收中断。
    rx_frames[0][0].header.Identifier = config.rx_id; // 构造属于新端点的帧。
    rx_frame_count[0] = 1u; // 中断可读取这一帧。
    TEST_CHECK(CANRegister(&config) != NULL); // 注册生产端点。
    TEST_CHECK(can_test_irq_sections > 0u && can_test_irq_mask == 0u); // 发布时使用临界区并恢复中断。
    CANProcessRx(); // 在普通任务中派发中断接收的数据。
    TEST_CHECK(valid_rx_count == 1u); // 发布完成时到达的帧必须找到新实例。
    inject_after_filter = 0u; // 第二次注册不重复注入。
    verify_filter_interrupts = 0u; // 下一次模拟调用前已关中断，不要求开启硬件配置。
    can_test_irq_mask = 1u; // 模拟外层已经屏蔽中断。
    config.rx_id = 0x322u; // 使用另一个端点避免重复登记。
    TEST_CHECK(CANRegister(&config) != NULL); // 在调用方已屏蔽时仍可登记。
    TEST_CHECK(can_test_irq_mask == 1u); // 函数不得意外开启外层中断。
}

static void test_bus_start_and_rollback_isolation(void)
{
    CAN_Init_Config_s config = {0}; // 从默认标准帧配置开始。
    config.can_handle = &hfdcan1; // 第一条总线先启用。
    config.tx_id = 0x300u; // 测试发送号。
    config.rx_id = 0x321u; // 测试接收号。
    config.can_module_callback = validate_rx_callback; // 接收时计数。
    TEST_CHECK(CANRegister(&config) != NULL); // 第一条总线应成功启动。
    TEST_CHECK(bus_start_count[0] == 1u && bus_start_count[1] == 0u && bus_start_count[2] == 0u);
    config.can_handle = &hfdcan2; // 第二条总线稍后才启用。
    hfdcan2.Instance = NULL; // 模拟车型未准备或初始化失败的控制器。
    TEST_CHECK(!CANStartBus(&hfdcan2)); // 不能把空硬件地址交给 HAL。
    TEST_CHECK(CANRegister(&config) == NULL); // 注册失败不能影响已有总线。
    TEST_CHECK(bus_start_count[1] == 0u && bus_stop_count[1] == 0u);
    TEST_CHECK(hfdcan1.State == HAL_FDCAN_STATE_BUSY); // 第一条总线保持运行。
    hfdcan2.Instance = &fdcan_registers[1]; // 模拟重新准备总线供后续回退场景使用。
    filter_fail_once = 1u; // 注入第二条总线的过滤器失败。
    TEST_CHECK(CANRegister(&config) == NULL); // 本次端点必须回退。
    TEST_CHECK(bus_stop_count[0] == 0u && bus_stop_count[1] == 1u && bus_stop_count[2] == 0u);
    TEST_CHECK(hfdcan1.State == HAL_FDCAN_STATE_BUSY); // 第一条总线仍在运行。
    TEST_CHECK(CANRegister(&config) != NULL); // 修复后可重新注册第二条总线。
    TEST_CHECK(bus_start_count[0] == 1u && bus_start_count[1] == 2u && bus_start_count[2] == 0u);
    rx_frames[0][0].header.Identifier = config.rx_id; // 从第一条总线投递一帧。
    rx_frame_count[0] = 1u; // 只启用一条接收记录。
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); // 模拟接收中断。
    CANProcessRx(); // 在任务上下文派发。
    TEST_CHECK(valid_rx_count == 1u); // 失败回退后原端点仍可正常接收。
}

static void test_platform_filter_capacity(void)
{
    PlatformCanFilterCapacity_t result = {.standard = 0xa5u, .extended = 0x5au}; // 失败时检查输出未被改写。
    TEST_CHECK(PlatformCanEndpointCapacity() == CAN_MX_REGISTER_CNT); // 平台上限与驱动登记表一致。
    TEST_CHECK(!PlatformCanGetFilterCapacity(PLATFORM_CAN_INVALID, &result));
    TEST_CHECK(result.standard == 0xa5u && result.extended == 0x5au);
    TEST_CHECK(!PlatformCanGetFilterCapacity(PLATFORM_CAN_BUS_1, NULL));
    hfdcan1.Instance = NULL; // 模拟板级初始化失败。
    TEST_CHECK(!PlatformCanGetFilterCapacity(PLATFORM_CAN_BUS_1, &result));
    TEST_CHECK(result.standard == 0xa5u && result.extended == 0x5au);
    hfdcan1.Instance = &fdcan_registers[0]; // 恢复已准备控制器。
    hfdcan1.Init.StdFiltersNbr = 7u; hfdcan1.Init.ExtFiltersNbr = 3u;
    TEST_CHECK(PlatformCanGetFilterCapacity(PLATFORM_CAN_BUS_1, &result));
    TEST_CHECK(result.standard == 7u && result.extended == 3u); // 两类槽分别读取。
    hfdcan1.Init.StdFiltersNbr = 100u; hfdcan1.Init.ExtFiltersNbr = 100u;
    TEST_CHECK(PlatformCanGetFilterCapacity(PLATFORM_CAN_BUS_1, &result));
    TEST_CHECK(result.standard == CAN_MX_REGISTER_CNT && result.extended == CAN_MX_REGISTER_CNT); // 不超过全局登记上限。
    TEST_CHECK(bus_start_count[0] == 0u && filter_count[0] == 0u); // 查询不启动总线或配置过滤器。
}

static void test_rx_failure_and_invalid_frames(void)
{
    CAN_Init_Config_s config = {0}; // 默认标准 ID，与现有调用保持兼容。
    config.can_handle = &hfdcan1; // 测试第一条总线。
    config.tx_id = 0x300u; // 发送 ID 不影响接收校验。
    config.rx_id = 0x321u; // 仅接收此 ID。
    config.can_module_callback = validate_rx_callback; // 检查实际派发。
    TEST_CHECK(CANRegister(&config) != NULL);
    rx_frame_count[0] = 1u; // 第一条测试帧保持在硬件 FIFO 中。
    rx_frames[0][0].header.Identifier = config.rx_id;
    rx_read_failures = 2u; // 若失败后继续读取，就会触发第二次失败。
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    TEST_CHECK(rx_read_calls == 1u); // HAL 失败必须立即结束本轮中断处理。
    TEST_CHECK(rx_frame_read[0] == 0u); // 失败不应消费或派发旧数据。
    CANProcessRx();
    TEST_CHECK(valid_rx_count == 0u);

    rx_read_failures = 0u; // 后续中断恢复读取。
    rx_frame_count[0] = 3u; // 两帧无效，最后一帧是合法的零长度帧。
    rx_frames[0][0].header.DataLength = 9u << 16u; // 超过经典 CAN 的 8 字节。
    rx_frames[0][1].header.Identifier = config.rx_id;
    rx_frames[0][1].header.FDFormat = FDCAN_FD_CAN; // FD 帧不属于经典 CAN 入口。
    rx_frames[0][2].header.Identifier = config.rx_id;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    CANProcessRx();
    TEST_CHECK(valid_rx_count == 1u); // 无效帧不能被伪装成零长度数据。
    TEST_CHECK(valid_rx_length == 0u); // 真正的零长度经典数据帧仍可交付。
}

typedef struct { uint32_t count, time_ms; uint8_t marker; } ExtendedRxState;
static void extended_rx_callback(void *context, const PlatformCanRxFrame_t *frame)
{
    ExtendedRxState *state = context; // 使用端点自身的测试状态。
    ++state->count; // 按端点分别统计收到的帧。
    state->time_ms = frame->received_ms; // 保存中断入队时的原始时间。
    state->marker = frame->length == 0u ? 0u : frame->data[0]; // 读取本帧有效载荷。
}

static void test_classic_extended_rx_and_filter_reuse(void)
{
    hfdcan1.Init.StdFiltersNbr = 1u; // 第一条总线只分配一个标准过滤器。
    hfdcan1.Init.ExtFiltersNbr = 1u; // 第一条总线只分配一个扩展过滤器。
    hfdcan2.Init.ExtFiltersNbr = 1u; // 第二条总线独立拥有一个扩展过滤器。
    PlatformCanRxPort_t standard = {0}, extended = {0}, other = {0}; // 分别保存三个接收端点。
    ExtendedRxState std_state = {0}, ext_state = {0}, other_state = {0}; // 分别记录三个回调。
    PlatformCanEndpoint_t endpoint = {.can_bus = PLATFORM_CAN_BUS_1, .tx_id = 0x123u, .rx_id = 0x123u};
    TEST_CHECK(PlatformCanRxRegister(&standard, &endpoint, extended_rx_callback, &std_state)); // 零初始化类型保持标准帧。
    endpoint.identifier_type = CAN_TX_ID_EXTENDED; // 相同数值改为扩展类型。
    TEST_CHECK(PlatformCanRxRegister(&extended, &endpoint, extended_rx_callback, &ext_state)); // 允许同数值的扩展端点。
    TEST_CHECK(filter_last_type[0] == FDCAN_EXTENDED_ID && filter_last_index[0] == 0u);
    PlatformCanRxPort_t overflow = {0}; // 额外端点用于验证容量不足。
    endpoint.rx_id = 0x124u; // 改为不同 ID，避免重复注册检查掩盖容量问题。
    TEST_CHECK(!PlatformCanRxRegister(&overflow, &endpoint, extended_rx_callback, &ext_state));
    TEST_CHECK(overflow.backend == NULL); // 容量不足不能发布半成品端点。
    endpoint.rx_id = 0x123u; // 恢复待检查的相同 ID。
    endpoint.can_bus = PLATFORM_CAN_BUS_2; // 相同扩展 ID 可用于另一条总线。
    TEST_CHECK(PlatformCanRxRegister(&other, &endpoint, extended_rx_callback, &other_state));
    rx_frames[0][0].header.Identifier = 0x123u; // 第一帧为标准帧。
    rx_frames[0][0].header.DataLength = 1u << 16u;
    rx_frames[0][0].data[0] = 0x11u;
    rx_frames[0][1] = rx_frames[0][0]; // 第二帧数值相同，类型不同。
    rx_frames[0][1].header.IdType = FDCAN_EXTENDED_ID;
    rx_frames[0][1].data[0] = 0x22u;
    rx_frames[1][0] = rx_frames[0][1]; // 第三帧来自另一条总线。
    rx_frames[1][0].data[0] = 0x33u;
    rx_frame_count[0] = 2u; rx_frame_count[1] = 1u; // 供中断依次读取。
    now_us = 123000u; // 使用固定接收时间，检查派发不会刷新时间。
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    HAL_FDCAN_RxFifo0Callback(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    now_us = 125000u; CANProcessRx(); // 稍后在任务中派发已入队帧。
    TEST_CHECK(std_state.count == 1u && std_state.marker == 0x11u && std_state.time_ms == 123u);
    TEST_CHECK(ext_state.count == 1u && ext_state.marker == 0x22u && ext_state.time_ms == 123u);
    TEST_CHECK(other_state.count == 1u && other_state.marker == 0x33u && other_state.time_ms == 123u);
    TEST_CHECK(PlatformCanRxRollback(&extended)); // 回退时确认第一条总线的扩展槽关闭。
    endpoint.can_bus = PLATFORM_CAN_BUS_1; // 转回第一条总线。
    endpoint.rx_id = 0x1fffffffu; endpoint.tx_id = 0x1fffffffu; // 验证 29 位最大合法 ID。
    filter_fail_once = 1u; // 注入重新配置硬件过滤器失败。
    TEST_CHECK(!PlatformCanRxRegister(&extended, &endpoint, extended_rx_callback, &ext_state));
    TEST_CHECK(extended.backend == NULL); // 失败后端点保持空闲，槽位也可重新申请。
    TEST_CHECK(PlatformCanRxRegister(&extended, &endpoint, extended_rx_callback, &ext_state));
    TEST_CHECK(filter_last_index[0] == 0u && filter_last_type[0] == FDCAN_EXTENDED_ID); // 复用释放槽。
    filter_fail_once = 1u; // 注入关闭硬件过滤器失败。
    TEST_CHECK(!PlatformCanRxRollback(&extended)); // 硬件关闭失败必须向调用者报告。
    TEST_CHECK(extended.backend == NULL); // 即使硬件失败也撤销软件端点与旧队列引用。
    TEST_CHECK(!PlatformCanRxRegister(&extended, &endpoint, extended_rx_callback, &ext_state));
    TEST_CHECK(extended.backend == NULL); // 未确认关闭的槽不得直接复用。
    const uint32_t other_filter_count = filter_count[1]; // 第二条总线的硬件不应被重新配置。
    TEST_CHECK(CANReapplyFilters(&hfdcan1)); // 只按第一条总线的剩余端点重建过滤器。
    TEST_CHECK(filter_count[1] == other_filter_count); // 重建没有修改其他控制器。
    TEST_CHECK(PlatformCanRxRegister(&extended, &endpoint, extended_rx_callback, &ext_state));
    rx_frames[1][1] = rx_frames[1][0]; // 重新投递另一条总线上的旧端点帧。
    rx_frame_count[1] = 2u; // 包含刚添加的一帧。
    HAL_FDCAN_RxFifo0Callback(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    CANProcessRx(); // 过滤器失败和重建不能破坏其他总线端点。
    TEST_CHECK(other_state.count == 2u && other_state.marker == 0x33u);
    PlatformCanRxPort_t invalid = {0}; // 错误配置不能占用端点。
    endpoint.rx_id = 0x20000000u; // 超过 29 位上限。
    TEST_CHECK(!PlatformCanRxRegister(&invalid, &endpoint, extended_rx_callback, &ext_state));
    endpoint.identifier_type = CAN_TX_ID_STANDARD; endpoint.rx_id = 0x800u; endpoint.tx_id = 0u;
    TEST_CHECK(!PlatformCanRxRegister(&invalid, &endpoint, extended_rx_callback, &ext_state)); // 标准 ID 也要验证范围。
    endpoint.identifier_type = (CAN_TxIdType_t)-1; endpoint.rx_id = 0u; // 未定义的标识符类型。
    TEST_CHECK(!PlatformCanRxRegister(&invalid, &endpoint, extended_rx_callback, &ext_state));
    endpoint.identifier_type = CAN_TX_ID_EXTENDED; endpoint.can_bus = PLATFORM_CAN_BUS_3;
    TEST_CHECK(!PlatformCanRxRegister(&invalid, &endpoint, extended_rx_callback, &ext_state)); // 第三条总线没有扩展过滤器。
    TEST_CHECK(bus_stop_count[0] == 0u && bus_stop_count[1] == 0u); // 容量失败不关闭已经运行的总线。
}

static void test_filter_restore_failure_isolates_one_bus(void)
{
    PlatformCanRxPort_t first = {0}, second = {0}, extra = {0}; // 三个独立接收端点。
    ExtendedRxState first_state = {0}, second_state = {0}; // 分别观察两条总线。
    PlatformCanEndpoint_t endpoint = {.can_bus = PLATFORM_CAN_BUS_1, .tx_id = 0x101u, .rx_id = 0x101u};
    TEST_CHECK(PlatformCanRxRegister(&first, &endpoint, extended_rx_callback, &first_state));
    endpoint.can_bus = PLATFORM_CAN_BUS_2; endpoint.rx_id = 0x102u; endpoint.tx_id = 0x102u;
    TEST_CHECK(PlatformCanRxRegister(&second, &endpoint, extended_rx_callback, &second_state));
    const uint32_t other_filters = filter_count[1]; // 重装不能触碰第二条总线。
    const uint32_t other_interrupts = hfdcan2.Instance->IE; // 隔离不能关闭其他控制器的中断。
    const uint8_t payload = 0x42u;
    const CAN_TxFrame_t frame = {.identifier = 0x101u, .identifier_type = CAN_TX_ID_STANDARD,
                                 .data = &payload, .length = 1u};
    uint32_t request = 0u, in_flight = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL, 20u, &in_flight) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx(); // 第一帧交给模拟硬件，尚未报告完成。
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL, 20u, &request) == CAN_TX_SUBMIT_ACCEPTED);
    rx_frames[0][0].header.Identifier = 0x101u; rx_frames[0][0].header.DataLength = 1u << 16u;
    rx_frames[0][0].data[0] = payload; rx_frame_count[0] = 1u;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); // 先排入旧反馈。
    filter_fail_once = 1u; // 第一条总线的重装失败。
    TEST_CHECK(!CANReapplyFilters(&hfdcan1));
    TEST_CHECK(PlatformCanBusIsQuarantined(PLATFORM_CAN_BUS_1)); // 板级接口暴露真实隔离状态。
    TEST_CHECK(!PlatformCanBusIsQuarantined(PLATFORM_CAN_BUS_2)); // 其他总线没有被标记隔离。
    TEST_CHECK(PlatformCanBusIsQuarantined(PLATFORM_CAN_INVALID)); // 未绑定编号按不可用处理。
    TEST_CHECK(bus_stop_count[0] == 1u && bus_stop_count[1] == 0u && filter_count[1] == other_filters);
    TEST_CHECK(hfdcan1.Instance->IE == 0u && hfdcan2.Instance->IE == other_interrupts);
    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(request, &result) && result.state == CAN_TX_RESULT_CANCELLED);
    TEST_CHECK(CANGetTxRequestState(in_flight, &result) && result.state == CAN_TX_RESULT_CANCELLED && result.pending_frames == 1u);
    CAN_TxSchedulerStats_t stats = {0};
    TEST_CHECK(CANGetTxSchedulerStats(&hfdcan1, &stats) && stats.cancelled == 2u); // 未使用的结果槽不能计入取消。
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL, 20u, &request) == CAN_TX_SUBMIT_NOT_READY);
    TEST_CHECK(!CANForceMode(&hfdcan1, FDCAN_MODE_NORMAL, 0u)); // 诊断模式切换不能绕过隔离。
    CANProcessRx();
    TEST_CHECK(first_state.count == 0u); // 故障前排队的反馈不再交付。
    endpoint.can_bus = PLATFORM_CAN_BUS_1; endpoint.rx_id = 0x103u; endpoint.tx_id = 0x103u;
    TEST_CHECK(!PlatformCanRxRegister(&extra, &endpoint, extended_rx_callback, &first_state)); // 隔离后不能隐式重启。
    rx_frames[1][0].header.Identifier = 0x102u; rx_frames[1][0].header.DataLength = 1u << 16u;
    rx_frames[1][0].data[0] = payload; rx_frame_count[1] = 1u;
    HAL_FDCAN_RxFifo0Callback(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    CANProcessRx();
    TEST_CHECK(second_state.count == 1u && second_state.marker == payload); // 第二条总线仍正常派发。
}

static void burst_rx_callback(CANInstance *instance)
{
    uint8_t bus = bus_for_handle(instance->can_handle);
    if (bus >= DEVICE_CAN_CNT || instance->rx_len != 8u || instance->rx_buff[0] != burst_rx_count[bus] ||
        instance->rx_timestamp_cycles == 0u ||
        (burst_rx_count[bus] != 0u &&
         (uint32_t)(instance->rx_timestamp_cycles - burst_last_cycle[bus]) != 480000u))
    {
        if (bus < DEVICE_CAN_CNT)
            ++burst_rx_bad[bus];
        return;
    }
    burst_last_cycle[bus] = instance->rx_timestamp_cycles;
    ++burst_rx_count[bus];
}

static void test_production_can_receive_burst(void)
{
    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        CAN_Init_Config_s config = {0};
        config.can_handle = bus == 0u ? &hfdcan1 : bus == 1u ? &hfdcan2 : &hfdcan3;
        config.tx_id = 0x300u + bus;
        config.rx_id = 0x320u + bus;
        config.can_module_callback = burst_rx_callback;
        TEST_CHECK(CANRegister(&config) != NULL);

        rx_frame_count[bus] = 32u;
        rx_frame_read[bus] = 0u;
        for (uint8_t frame = 0u; frame < rx_frame_count[bus]; ++frame)
        {
            TestRxFrame_t *item = &rx_frames[bus][frame];
            memset(item, 0, sizeof(*item));
            item->header.Identifier = config.rx_id;
            item->header.DataLength = 8u << 16u;
            item->header.RxFrameType = FDCAN_DATA_FRAME;
            item->header.IdType = FDCAN_STANDARD_ID;
            item->data[0] = frame;
        }
        HAL_FDCAN_RxFifo0Callback(config.can_handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        TEST_CHECK(rx_frame_read[bus] == 32u);
        TEST_CHECK(CANGetRxQueueDrop(config.can_handle) == 1u);
    }

    now_us = 0u;
    CANProcessRx();
    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        TEST_CHECK(burst_rx_count[bus] == 31u);
        TEST_CHECK(burst_rx_bad[bus] == 0u);
    }
    TEST_CHECK(can_prof_rx_dispatch.calls == 1u);
    TEST_CHECK(can_prof_rx_dispatch.max_us >= can_prof_rx_dispatch.last_us);
}

/* 通过生产 CAN 派发器调用真实 DJI 解码回调，连续压入三条总线的满队列。 */
static void test_production_dji_receive_burst(void)
{
    DJIMotorInstance *motors[DEVICE_CAN_CNT] = {0};
    FDCAN_HandleTypeDef *handles[DEVICE_CAN_CNT] = {&hfdcan1, &hfdcan2, &hfdcan3};
    uint32_t expected_accepted[DEVICE_CAN_CNT] = {0u};

    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        Motor_Init_Config_s config = {0};
        config.motor_type = M3508;
        config.output_axis = (Motor_Output_Axis_Config_s){1.0f, MOTOR_AXIS_DIRECTION_NORMAL};
        config.can_init_config.can_bus = (PlatformCanBus_t)(PLATFORM_CAN_BUS_1 + bus);
        config.can_init_config.tx_id = 1u;
        config.controller_setting_init_config.close_loop_type = OPEN_LOOP;
        motors[bus] = DJIMotorInit(&config);
        TEST_CHECK(motors[bus] != NULL);
        TEST_CHECK(motors[bus]->motor_can_endpoint.rx_id == 0x201u);
    }

    for (uint8_t batch = 0u; batch < 24u; ++batch)
    {
        for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
        {
            CANInstance *can = (CANInstance *)motors[bus]->motor_can_port.backend; // 测试底层实例读取接收状态。
            rx_frame_count[bus] = 32u;
            rx_frame_read[bus] = 0u;
            for (uint8_t frame = 0u; frame < rx_frame_count[bus]; ++frame)
            {
                TestRxFrame_t *item = &rx_frames[bus][frame];
                uint16_t ecd = (uint16_t)((7000u + batch * 73u + frame) % 8192u);
                int16_t rpm = (int16_t)((int16_t)(batch * 31u) - (int16_t)(frame * 3u));
                int16_t current = (int16_t)((int16_t)(frame * 17u) - (int16_t)(batch * 11u));
                memset(item, 0, sizeof(*item));
                item->header.Identifier = can->rx_id;
                item->header.DataLength = 8u << 16u;
                item->header.RxFrameType = FDCAN_DATA_FRAME;
                item->header.IdType = FDCAN_STANDARD_ID;
                item->data[0] = (uint8_t)(ecd >> 8u);
                item->data[1] = (uint8_t)ecd;
                item->data[2] = (uint8_t)((uint16_t)rpm >> 8u);
                item->data[3] = (uint8_t)rpm;
                item->data[4] = (uint8_t)((uint16_t)current >> 8u);
                item->data[5] = (uint8_t)current;
                item->data[6] = (uint8_t)(batch + frame);
            }
            HAL_FDCAN_RxFifo0Callback(handles[bus], FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
            TEST_CHECK(rx_frame_read[bus] == 32u);
            expected_accepted[bus] += 31u;
        }

        now_us = (uint64_t)batch * 1000u;
        CANProcessRx();
    }

    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        DJIMotorInstance *motor = motors[bus];
        uint16_t expected_ecd = (uint16_t)((7000u + 23u * 73u + 30u) % 8192u);
        TEST_CHECK(motor->feedback_sample_count == expected_accepted[bus]);
        TEST_CHECK(motor->measure.ecd == expected_ecd);
        TEST_CHECK(motor->measure.temperature == (uint8_t)(23u + 30u));
        TEST_CHECK(isfinite(motor->measure.angle_single_round));
        TEST_CHECK(isfinite(motor->measure.speed_aps));
        TEST_CHECK(isfinite(motor->measure.total_angle));
        TEST_CHECK(CANGetRxQueueDrop(handles[bus]) == 24u);
    }
    TEST_CHECK(can_prof_rx_dispatch.calls == 24u);
}

static Motor_Init_Config_s config_for(uint8_t bus, uint32_t motor_id)
{
    Motor_Init_Config_s config = {0};
    config.motor_type = LK9025;
    config.output_axis = (Motor_Output_Axis_Config_s){1.0f, MOTOR_AXIS_DIRECTION_NORMAL};
    config.can_init_config.can_bus = (PlatformCanBus_t)(PLATFORM_CAN_BUS_1 + bus);
    config.can_init_config.tx_id = motor_id;
    config.controller_setting_init_config.close_loop_type = OPEN_LOOP;
    config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    return config;
}

static PID_Init_Config_s pid_config(float kp, float max_out)
{
    PID_Init_Config_s config = {0};
    config.Kp = kp;
    config.MaxOut = max_out;
    config.DtMin_s = MOTOR_PID_DT_MIN_S;
    config.DtMax_s = MOTOR_PID_DT_MAX_S;
    return config;
}

// 真实 CAN 注册失败后，DJI 不得留下守护对象或占用反馈 ID。
static void test_dji_receive_registration_rollback(void)
{
    Motor_Init_Config_s config = {0};
    config.motor_type = M3508;
    config.output_axis = (Motor_Output_Axis_Config_s){1.0f, MOTOR_AXIS_DIRECTION_NORMAL};
    config.can_init_config.can_bus = PLATFORM_CAN_BUS_1;
    config.can_init_config.tx_id = 1u;
    config.controller_setting_init_config.close_loop_type = OPEN_LOOP;
#if defined(LK_TEST_ALLOC_WRAP)
    const unsigned alloc_before = malloc_calls, free_before = free_calls;
#endif
    config.can_init_config.can_bus = PLATFORM_CAN_INVALID; // 未声明的逻辑总线不得申请电机资源。
    TEST_CHECK(DJIMotorInit(&config) == NULL); // 缺少板级绑定时拒绝注册。
#if defined(LK_TEST_ALLOC_WRAP)
    TEST_CHECK(malloc_calls == alloc_before); // 绑定检查发生在首次分配之前。
#endif
    config.can_init_config.can_bus = PLATFORM_CAN_BUS_1; // 恢复可用总线再注入注册错误。
    filter_fail_once = 1u; // 注入底层过滤器配置失败。
    TEST_CHECK(DJIMotorInit(&config) == NULL);
#if defined(LK_TEST_ALLOC_WRAP)
    TEST_CHECK(malloc_calls - alloc_before == free_calls - free_before); // 本次分配均已回收。
#endif
    TEST_CHECK(filter_count[0] == 0u);
    DJIMotorInstance *motor = DJIMotorInit(&config); // 同一端点可在失败后重新注册。
    TEST_CHECK(motor != NULL && motor->motor_can_endpoint.rx_id == 0x201u);
    TEST_CHECK(PlatformCanRxIsRegistered(&motor->motor_can_port));
    TEST_CHECK(DJIMotorInit(&config) == NULL); // 同总线同反馈 ID 必须拒绝重复。
    config.can_init_config.can_bus = PLATFORM_CAN_BUS_2;
    TEST_CHECK(DJIMotorInit(&config) != NULL); // 不同总线允许同一反馈 ID。
}

// 启动失败时只撤销指定电机，保留同组及其他总线的电机。
static uint8_t start_module_with_topic_failure(void *context, unsigned module)
{
    (void)context; // 本用例使用进程内固定状态，不需要外部对象。
    Motor_Init_Config_s config = {0}; // 明确标记为主机测试用电机参数。
    config.motor_type = M3508; // 使用生产 DJI 编码和分组逻辑。
    config.output_axis = (Motor_Output_Axis_Config_s){1.0f, MOTOR_AXIS_DIRECTION_NORMAL}; // 测试输出轴没有减速。
    config.controller_setting_init_config.close_loop_type = OPEN_LOOP; // 不依赖 PID 参数。
    config.can_init_config.tx_id = 1u; // 两条总线允许相同设备编号。
    config.can_init_config.can_bus = module == ROBOT_MODULE_CHASSIS ? PLATFORM_CAN_BUS_2 : PLATFORM_CAN_BUS_1; // 核心实例与待回退实例使用不同总线。
    DJIMotorInstance *motor = DJIMotorInit(&config); // 真实登记电机、来源监控和 CAN 端点。
    TEST_CHECK(motor != NULL);
    if (module == ROBOT_MODULE_CHASSIS) return 1u; // 保留另一条总线的成功实例。
    TEST_CHECK(module == ROBOT_MODULE_GIMBAL); // 下游模块不能越过失败依赖执行。
    MessagePublisher_t *publisher = NULL; // 成对登记失败不得发布半个端点。
    MessageSubscriber_t *subscriber = NULL; // 输出句柄在失败时保持为空。
    TEST_CHECK(MessageEndpointPairRegister(77u, 77u, 9u, &publisher, &subscriber) == MESSAGE_ERR_CAPACITY);
    TEST_CHECK(publisher == NULL && subscriber == NULL);
    TEST_CHECK(DJIMotorRollbackStartup(motor)); // 话题失败后撤销已取得的真实驱动资源。
    return 0u; // 交由生产依赖计划判断核心或辅助失败。
}

static RobotModulePlan_t integration_plan;
static RobotModuleStartup_t integration_state;
static unsigned integration_tasks, integration_ready, integration_faults, integration_disabled, integration_shutdowns;
static uint8_t integration_failed;
uint8_t PlatformSchedulerIsRunning(void) { return 1u; } // 模拟协调任务运行条件。
uint8_t RobotInitializeApplications(void)
{
    return (uint8_t)(RobotModuleStart(&integration_plan, start_module_with_topic_failure, NULL,
                                    &integration_state) == ROBOT_MODULE_OK); // 运行生产依赖计划与真实驱动。
}
RobotInsStartupResult_e RobotInitializeIns(void) { return ROBOT_INS_STARTUP_READY; } // 本用例不验证姿态设备。
void RobotRunStartupDiagnostics(void) {} // 不启动硬件自测。
uint8_t RobotRecheckCanAfterDiagnostics(void) { return 1u; } // CAN 隔离另有生产专项。
uint8_t PlatformTaskBatchBegin(void) { return 1u; } // 任务调度由计数替身观察。
uint8_t PlatformTaskBatchEnd(void) { return 1u; }
uint8_t OSTaskInit(void) { ++integration_tasks; return 1u; } // 记录是否允许进入任务创建。
uint8_t BSP_WatchdogInitChecked(uint32_t ms) { TEST_CHECK(ms == 200u); return 1u; }
TaskMonitor_Result_e TaskMonitorReleaseRuntime(void) { return TASK_MONITOR_RESULT_OK; }
void InitDiagnosticSetStage(InitDiagnosticStage_e stage) { (void)stage; }
uint8_t InitDiagnosticIsFailed(void) { return integration_failed; }
void InitDiagnosticReportFailure(InitDiagnosticStage_e stage, uint8_t module, InitDiagnosticReason_e reason, uint16_t resource)
{ (void)stage; (void)module; (void)reason; (void)resource; integration_failed = 1u; }
void InitDiagnosticReportDegraded(InitDiagnosticStage_e stage, uint8_t module, InitDiagnosticReason_e reason, uint16_t resource)
{ (void)stage; (void)module; (void)reason; (void)resource; }
void InitDiagnosticMarkReady(void) { ++integration_ready; }
void RobotFaultReport(Robot_Fault_e fault) { TEST_CHECK(fault == ROBOT_FAULT_INIT_FAILURE); ++integration_faults; }
void RobotSafetyRequestDisable(void) { ++integration_disabled; }
void BSP_SafetyStartupLatchOutputsOff(void) { ++integration_shutdowns; } // 主机记录关断调用，不驱动引脚。

static void test_module_driver_topic_failure(uint8_t core)
{
    const MessageTopicDescriptor_t descriptor = {
        .topic_id = 77u, // 本联合测试独占的话题编号。
        .payload_size = MESSAGE_CENTER_MAX_PAYLOAD_BYTES, // 每个订阅使用最大负载。
        .max_age_ms = 100u, // 使用合法的固定有效时间。
        .capacity = MESSAGE_CENTER_MAX_QUEUE_DEPTH, // 每个订阅需要四个存储槽。
        .delivery = MESSAGE_DELIVERY_EVENT_FIFO, // 有序队列消耗完整存储区。
        .overflow = MESSAGE_OVERFLOW_REJECT_NEWEST, // 满容量时明确拒绝。
        .source_clock = MESSAGE_CLOCK_LOCAL, // 使用主机模拟的本地时钟。
        .publisher_mask = 0x1ffu, // 允许测试中的九个端点身份发布。
        .subscriber_mask = 0x1ffu, // 第九个身份合法，失败原因应为容量不足。
    };
    TEST_CHECK(MessageCenterInit(&descriptor, 1u) == MESSAGE_OK);
    for (uint8_t endpoint = 1u; endpoint <= 8u; ++endpoint) // 耗尽生产消息中心的订阅数据区。
        TEST_CHECK(MessageSubscriberRegister(77u, endpoint) != NULL);
    RobotModulePlan_t plan = {
        .enabled = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS) | ROBOT_MODULE_BIT(ROBOT_MODULE_GIMBAL) | ROBOT_MODULE_BIT(ROBOT_MODULE_SHOOTER), // 只运行本测试声明的三个模块。
        .required = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS), // 底盘始终是核心。
    };
    plan.dependencies[ROBOT_MODULE_SHOOTER] = ROBOT_MODULE_BIT(ROBOT_MODULE_GIMBAL); // 发射依赖失败云台，不得调用其初始化。
    if (core) plan.required |= ROBOT_MODULE_BIT(ROBOT_MODULE_GIMBAL); // 同样的资源失败按不同车型分类。
    integration_plan = plan; // 将本次配置交给协调器的设备入口。
    TEST_CHECK(RobotStartupRun() == (uint8_t)!core); // 真实协调器决定是否创建和放行任务。
    const RobotModuleStartup_t state = integration_state; // 检查同一次依赖启动结果。
    TEST_CHECK(integration_tasks == (unsigned)!core && integration_ready == (unsigned)!core);
    TEST_CHECK(integration_faults == core && integration_disabled == core && integration_shutdowns == core); // 核心失败必须调用全部保护。
    TEST_CHECK(state.started == ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS)); // 成功实例保留，失败实例未发布。
    TEST_CHECK(state.failed == ROBOT_MODULE_BIT(ROBOT_MODULE_GIMBAL) && state.core_failed == core);
    if (!core) TEST_CHECK(state.blocked == ROBOT_MODULE_BIT(ROBOT_MODULE_SHOOTER)); // 辅助失败只阻塞下游。
    if (!core) // 只有辅助失败的计划允许进入后续周期处理。
    {
        DJIMotorControl(); // 统一电机循环只应遍历保留实例。
        CANProcessTx(); // 使用生产发送队列推进硬件替身。
        TEST_CHECK(tx_accept_count[0] == 0u && tx_accept_count[1] == 1u); // 回退总线不发送，另一条总线保留。
    }
    else TEST_CHECK(tx_accept_count[0] == 0u && tx_accept_count[1] == 0u); // 初始化流程本身不得发送控制帧。
}

static void test_dji_startup_rollback(void)
{
    Motor_Init_Config_s config = {0}; // 使用不依赖 PID 的测试电机配置。
    config.motor_type = M3508; // 选择常用底盘电调。
    config.output_axis = (Motor_Output_Axis_Config_s){1.0f, MOTOR_AXIS_DIRECTION_NORMAL}; // 保持输出轴比例和方向。
    config.controller_setting_init_config.close_loop_type = OPEN_LOOP; // 不需要闭环参数。
    config.can_init_config.can_bus = PLATFORM_CAN_BUS_1; // 前两个电机共享总线和发送组。
    config.can_init_config.tx_id = 1u; // 注册第一台电机。
    DJIMotorInstance *first = DJIMotorInit(&config); // 记录稍后需要回退的实例。
    config.can_init_config.tx_id = 2u; // 注册相同发送组的第二台电机。
    DJIMotorInstance *second = DJIMotorInit(&config); // 保留实例验证同组隔离。
    config.can_init_config.can_bus = PLATFORM_CAN_BUS_2; // 第三台放到另一条总线。
    config.can_init_config.tx_id = 1u; // 不同总线允许重复编号。
    DJIMotorInstance *other = DJIMotorInit(&config); // 用于验证跨总线隔离。
    TEST_CHECK(first != NULL && second != NULL && other != NULL);
    TEST_CHECK(!DJIMotorRollbackStartup(NULL)); // 空指针不能影响现有实例。
    TEST_CHECK(DJIMotorRollbackStartup(first)); // 回退第一台电机。
    TEST_CHECK(PlatformCanRxIsRegistered(&second->motor_can_port)); // 同组电机的接收端点仍有效。
    TEST_CHECK(PlatformCanRxIsRegistered(&other->motor_can_port)); // 其他总线的接收端点仍有效。
    config.can_init_config.can_bus = PLATFORM_CAN_BUS_1; // 重新使用已释放的设备编号。
    DJIMotorInstance *replacement = DJIMotorInit(&config); // 证明电机表、监控及接收端点均可复用。
    TEST_CHECK(replacement != NULL);
    TEST_CHECK(DJIMotorRollbackStartup(second)); // 继续撤销同组剩余电机。
    TEST_CHECK(DJIMotorRollbackStartup(replacement)); // 撤销替代电机，释放整个发送组。
    DJIMotorControl(); // 只有另一条总线的电机仍在发送组内。
    CANProcessTx(); // 处理测试产生的周期帧。
    TEST_CHECK(tx_accept_count[0] == 0u && tx_accept_count[1] == 1u); // 空组不发送，其他组不受影响。
    TEST_CHECK(DJIMotorInit(&config) != NULL); // 已释放的第一条总线仍允许注册。
}

// 守护任务开始后不能用仅适用于启动阶段的回退接口释放电机。
static void test_dji_startup_rollback_after_daemon(void)
{
    Motor_Init_Config_s config = {0}; // 构造有效的单电机配置。
    config.motor_type = M3508; // 使用 DJI 底盘电调。
    config.output_axis = (Motor_Output_Axis_Config_s){1.0f, MOTOR_AXIS_DIRECTION_NORMAL}; // 输出轴比例和方向。
    config.controller_setting_init_config.close_loop_type = OPEN_LOOP; // 不启用闭环。
    config.can_init_config.can_bus = PLATFORM_CAN_BUS_1; // 电机位于第一条总线。
    config.can_init_config.tx_id = 1u; // 使用第一个电调编号。
    DJIMotorInstance *motor = DJIMotorInit(&config); // 先完成初始化。
    TEST_CHECK(motor != NULL);
    DaemonTask(); // 模拟守护监控已经开始运行。
    TEST_CHECK(!DJIMotorRollbackStartup(motor)); // 越过启动阶段时必须拒绝回退。
    TEST_CHECK(PlatformCanRxIsRegistered(&motor->motor_can_port)); // 失败后电机端点仍然有效。
    TEST_CHECK(DJIMotorInit(&config) == NULL); // 同一地址仍被原电机占用。
}

// 删除同组电机时，剩余电机的发送组仍须保留。
static void test_dji_startup_rollback_keeps_group(void)
{
    Motor_Init_Config_s config = {0}; // 配置两台共用发送帧的电机。
    config.motor_type = M3508; // 两台电调使用相同型号。
    config.output_axis = (Motor_Output_Axis_Config_s){1.0f, MOTOR_AXIS_DIRECTION_NORMAL}; // 输出轴不减速。
    config.controller_setting_init_config.close_loop_type = OPEN_LOOP; // 保持默认停止输出。
    config.can_init_config.can_bus = PLATFORM_CAN_BUS_1; // 两台电机都在第一条总线。
    config.can_init_config.tx_id = 1u; // 第一台占帧内第一组字节。
    DJIMotorInstance *first = DJIMotorInit(&config); // 登记将被回退的实例。
    config.can_init_config.tx_id = 2u; // 第二台占同一帧的第二组字节。
    DJIMotorInstance *second = DJIMotorInit(&config); // 保留该实例。
    TEST_CHECK(first != NULL && second != NULL);
    TEST_CHECK(DJIMotorRollbackStartup(first)); // 只回退第一台电机。
    DJIMotorControl(); // 让保留实例生成组帧。
    CANProcessTx(); // 提交至第一条总线。
    TEST_CHECK(tx_accept_count[0] == 1u && tx_id_history[0][0] == 0x200u); // 同组电机仍能发送。
}

// 临时诊断资源不足时，回环不能先切换已有总线模式。
static void test_selftest_capacity_skip(uint8_t scenario)
{
    CAN_Init_Config_s config = {.can_handle = &hfdcan1, .tx_id = 1u,
                                .rx_id = 0x301u, .can_module_callback = validate_rx_callback}; // 默认端点属于第一条总线。
    if (scenario == 1u) // 测试全局注册表全部占用。
    {
        for (uint8_t i = 0u; i < 16u; ++i)
        {
            config.can_handle = i < 8u ? &hfdcan1 : &hfdcan2; // 两条总线分别使用八个端点。
            config.rx_id = 0x300u + (uint32_t)(i % 8u); // 各总线内保持 ID 唯一。
            TEST_CHECK(CANRegister(&config) != NULL);
        }
        TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_3)); // 第三条总线也满足基础启动条件。
    }
    else
    {
        hfdcan1.Init.StdFiltersNbr = scenario == 2u ? 1u : 14u; // 场景二只提供一个标准槽。
        hfdcan2.Init.StdFiltersNbr = 0u; // 第二条总线没有临时标准槽。
        hfdcan3.Init.StdFiltersNbr = 0u; // 第三条总线没有临时标准槽。
        if (scenario == 3u) config.rx_id = 0x123u; // 场景三让业务占用回环测试 ID。
        TEST_CHECK(CANRegister(&config) != NULL); // 记录本总线的业务端点。
        TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_2)); // 其余总线处于启动状态。
        TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_3));
    }
    uint32_t started[DEVICE_CAN_CNT], stopped[DEVICE_CAN_CNT], initialized[DEVICE_CAN_CNT]; // 记录进入诊断前的硬件调用数。
    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        started[bus] = bus_start_count[bus]; // 保存启动次数。
        stopped[bus] = bus_stop_count[bus]; // 保存停止次数。
        initialized[bus] = bus_init_count[bus]; // 保存控制器初始化次数。
    }
    TEST_CHECK(CANRunLoopbackSelfTest() == 0u); // 没有资源的三条总线均不能报告通过。
    TEST_CHECK(can_selftest_skip_mask == 7u && can_selftest_pass_mask == 0u); // 分别报告全部跳过。
    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
    {
        TEST_CHECK(bus_start_count[bus] == started[bus] && bus_stop_count[bus] == stopped[bus] &&
                   bus_init_count[bus] == initialized[bus]); // 跳过时不得停止、重启或重新初始化。
        TEST_CHECK(tx_accept_count[bus] == 0u); // 不能发出诊断帧。
    }
}

// DWT 计数停止且回环没有反馈时，诊断必须有限退出并恢复控制器。
static void test_selftest_stopped_clock(void)
{
    hfdcan2.Init.StdFiltersNbr = 0u; // 后两条总线不申请诊断端点。
    hfdcan3.Init.StdFiltersNbr = 0u; // 防止无关总线进入回环。
    TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_1)); // 第一条总线用于验证有限等待。
    TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_2)); // 其余总线保持已准备状态。
    TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_3));
    const uint8_t payload[1] = {0x5au}; // 另一条总线的独立业务数据。
    const CAN_TxFrame_t other_frame = {.identifier = 0x321u, .identifier_type = CAN_TX_ID_STANDARD,
                                       .data = payload, .length = 1u}; // 测试帧仅在主机模型中排队。
    uint32_t other_id = 0u; // 记录第二条总线请求以检查队列状态。
    TEST_CHECK(CANSubmitOneShot(&hfdcan2, &hfdcan2, &other_frame, CAN_TX_PRIORITY_NORMAL, 100u, &other_id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANRunLoopbackSelfTest() == 0u); // 固定 DWT 替身且无反馈时不可能通过。
    TEST_CHECK(can_selftest_done && can_selftest_skip_mask == 6u && can_selftest_pass_mask == 0u); // 一条失败，两条因容量跳过。
    TEST_CHECK(bus_init_count[0] == 1u && CANBusIsQuarantined(&hfdcan1)); // 未确认完成的发送使本总线隔离，不再重启。
    TEST_CHECK(bus_init_count[1] == 0u && bus_init_count[2] == 0u); // 跳过的总线未切换。
    TEST_CHECK(tx_accept_count[0] <= 1u); // 时基异常后不得继续发送剩余测试帧。
    CAN_TxResult_t other_result = {0}; // 查询其他总线的业务请求。
    TEST_CHECK(CANGetTxRequestState(other_id, &other_result) && other_result.state == CAN_TX_RESULT_QUEUED && tx_accept_count[1] == 0u); // 回环不能推进另一条总线。
}

// 模拟发送完成和接收中断，验证正常回环后恢复已有业务端点。
static void test_selftest_loopback_complete(void)
{
    CAN_Init_Config_s config = {.can_handle = &hfdcan1, .tx_id = 1u,
                                .rx_id = 0x301u, .can_module_callback = validate_rx_callback}; // 保留第一条总线的业务端点。
    CANInstance *business = CANRegister(&config); // 诊断前先注册业务接收。
    TEST_CHECK(business != NULL);
    TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_2)); // 启动第二条诊断总线。
    TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_3)); // 启动第三条诊断总线。
    selftest_loopback_enabled = 1u; // 只在本场景产生模拟收发完成中断。
    TEST_CHECK(CANRunLoopbackSelfTest() == 3u); // 三条总线均应完成回环和恢复。
    selftest_loopback_enabled = 0u; // 之后业务帧不再由测试钩子处理。
    TEST_CHECK(can_selftest_done && can_selftest_pass_mask == 7u && can_selftest_skip_mask == 0u); // 通过和跳过状态必须一致。
    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus)
        TEST_CHECK(tx_accept_count[bus] == 8u && !CANBusIsQuarantined(bus == 0u ? &hfdcan1 : bus == 1u ? &hfdcan2 : &hfdcan3)); // 每条总线发送八帧并保持可用。
    rx_frames[0][rx_frame_count[0]].header.Identifier = config.rx_id; // 诊断后投递业务反馈。
    rx_frames[0][rx_frame_count[0]].header.DataLength = 1u << 16u; // 单字节反馈用于测试派发。
    rx_frames[0][rx_frame_count[0]++].data[0] = 0x42u; // 写入测试数据并增加 FIFO 数量。
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); // 进入生产接收中断。
    CANProcessRx(); // 在任务上下文派发业务帧。
    TEST_CHECK(valid_rx_count == 1u && valid_rx_length == 1u); // 已注册端点在恢复后仍接收反馈。
}

// 发送尚未进入硬件时，时基停滞应撤销排队请求并恢复该总线。
static void test_selftest_stopped_clock_queued(void)
{
    hfdcan2.Init.StdFiltersNbr = 0u; // 后两条总线没有诊断过滤器名额。
    hfdcan3.Init.StdFiltersNbr = 0u; // 仅测试第一条总线的排队路径。
    TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_1)); // 启动回环目标总线。
    TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_2)); // 其他总线按资源不足跳过。
    TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_3));
    tx_busy_mask = 1u; // 第一条总线硬件始终忙，请求保持软件排队。
    TEST_CHECK(CANRunLoopbackSelfTest() == 0u); // 时基不前进时诊断必须失败并退出。
    CAN_TxSchedulerStats_t stats = {0}; // 查询该总线发送队列。
    TEST_CHECK(CANGetTxSchedulerStats(&hfdcan1, &stats) && stats.queued == 0u); // 测试请求已从软件队列撤销。
    TEST_CHECK(!CANBusIsQuarantined(&hfdcan1) && bus_init_count[0] == 2u); // 未进入硬件的请求撤销后可恢复正常模式。
    TEST_CHECK(tx_accept_count[0] == 0u && can_selftest_skip_mask == 6u); // 没有测试帧进入硬件，其他两条总线跳过。
}

// 使用生产电容和 CAN 驱动检查启动回退及旧队列帧。
static void test_supercap_startup_rollback(void)
{
    Motor_Init_Config_s motor_config = {0}; // 同一总线保留一台无关电机。
    motor_config.motor_type = M3508; // 使用常见 DJI 电调。
    motor_config.output_axis = (Motor_Output_Axis_Config_s){1.0f, MOTOR_AXIS_DIRECTION_NORMAL}; // 不改变输出轴比例。
    motor_config.can_init_config.can_bus = PLATFORM_CAN_BUS_1; // 与电容共用第一条总线。
    motor_config.can_init_config.tx_id = 1u; // 电机使用独立的反馈编号。
    motor_config.controller_setting_init_config.close_loop_type = OPEN_LOOP; // 测试不依赖 PID。
    DJIMotorInstance *motor = DJIMotorInit(&motor_config); // 登记需要保留的端点。
    TEST_CHECK(motor != NULL);
    SuperCap_Init_Config_s config = {.can_config = {.can_handle = &hfdcan1, .tx_id = 0x302u, .rx_id = 0x301u}}; // 使用旧电容的两个标准 ID。
    SuperCapInstance *cap = SuperCapInit(&config); // 登记生产电容实例。
    TEST_CHECK(cap != NULL);
    rx_frames[0][0].header.Identifier = 0x301u; // 构造将要过期的电容反馈。
    rx_frames[0][0].header.DataLength = 6u << 16u; // 旧协议至少需要六字节。
    rx_frames[0][0].data[0] = 0x12u; // 写入可识别的旧帧内容。
    rx_frame_count[0] = 1u; // 让模拟硬件提供一帧。
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); // 先把旧帧排入生产队列。
    TEST_CHECK(SuperCapRollbackStartup(cap)); // 在接收任务启动前撤销电容。
    TEST_CHECK(PlatformCanRxIsRegistered(&motor->motor_can_port)); // 同总线的电机端点保持有效。
    cap = SuperCapInit(&config); // 使用相同 ID 重新注册电容。
    TEST_CHECK(cap != NULL);
    CANProcessRx(); // 派发之前排队的旧帧。
    SuperCapFeedback_t feedback = {0}; // 查询新实例的数据状态。
    TEST_CHECK(SuperCapGet(cap, 0u, 100u, &feedback) == SUPERCAP_OK && feedback.state == SUPERCAP_FEEDBACK_UNSEEN); // 旧帧不能转交新实例。
    rx_frame_read[0] = 0u; // 重新提供一帧当前反馈。
    rx_frames[0][0].data[0] = 0x34u; // 新帧使用不同内容。
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); // 接收新帧。
    CANProcessRx(); // 按普通任务路径派发。
    TEST_CHECK(SuperCapGet(cap, 0u, 100u, &feedback) == SUPERCAP_OK && feedback.state == SUPERCAP_FEEDBACK_FRESH && feedback.value.vol == 0x3400u); // 新端点只接受新数据。
}

// 电机和电容必须把硬件过滤器关闭失败传给启动协调者。
static void test_device_filter_close_failure(void)
{
    Motor_Init_Config_s motor_config = {0}; // 准备使用第一条总线的电机。
    motor_config.motor_type = M3508; // 使用 DJI 电调。
    motor_config.output_axis = (Motor_Output_Axis_Config_s){1.0f, MOTOR_AXIS_DIRECTION_NORMAL}; // 不改变轴比例。
    motor_config.can_init_config.can_bus = PLATFORM_CAN_BUS_1; // 选择第一条总线。
    motor_config.can_init_config.tx_id = 1u; // 电机反馈使用独立 ID。
    motor_config.controller_setting_init_config.close_loop_type = OPEN_LOOP; // 不启用闭环。
    DJIMotorInstance *motor = DJIMotorInit(&motor_config); // 注册待回退电机。
    TEST_CHECK(motor != NULL);
    filter_fail_once = 1u; // 模拟硬件拒绝关闭电机过滤器。
    TEST_CHECK(!DJIMotorRollbackStartup(motor)); // 软件已撤销，但返回失败供整车失能。
    TEST_CHECK(CANReapplyFilters(&hfdcan1)); // 恢复硬件过滤表以继续测试。
    SuperCap_Init_Config_s config = {.can_config = {.can_handle = &hfdcan1, .tx_id = 0x302u, .rx_id = 0x301u}}; // 电容使用独立的标准 ID。
    SuperCapInstance *cap = SuperCapInit(&config); // 注册待回退电容。
    TEST_CHECK(cap != NULL);
    filter_fail_once = 1u; // 模拟硬件拒绝关闭电容过滤器。
    TEST_CHECK(!SuperCapRollbackStartup(cap)); // 软件实例释放后仍必须报告失败。
    TEST_CHECK(CANReapplyFilters(&hfdcan1)); // 恢复后槽位才能重新使用。
    TEST_CHECK(SuperCapInit(&config) != NULL); // 恢复后允许再次登记同一电容。
}

// 真实队列验证三条总线上三类 DJI 分组均使用对应的标准帧 ID。
static void test_dji_logical_send_groups(void)
{
    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus) // 逐条总线创建相同的三组电机。
    {
        Motor_Init_Config_s config = {0}; // 每条总线使用独立的初始配置。
        config.output_axis = (Motor_Output_Axis_Config_s){1.0f, MOTOR_AXIS_DIRECTION_NORMAL}; // 输出轴不减速、不反向。
        config.can_init_config.can_bus = (PlatformCanBus_t)(PLATFORM_CAN_BUS_1 + bus); // 对应逻辑总线。
        config.controller_setting_init_config.close_loop_type = OPEN_LOOP; // 测试组帧，不依赖闭环参数。
        config.motor_type = M3508; // 3508 的 ID 1 属于 0x200 组。
        config.can_init_config.tx_id = 1u; // 设备编号从 1 开始。
        TEST_CHECK(DJIMotorInit(&config) != NULL); // 注册第一组。
        config.can_init_config.tx_id = 5u; // 3508 的 ID 5 属于 0x1ff 组。
        TEST_CHECK(DJIMotorInit(&config) != NULL); // 注册第二组。
        config.motor_type = GM6020; // 6020 的 ID 5 属于 0x2ff 组。
        TEST_CHECK(DJIMotorInit(&config) != NULL); // 注册第三组。
    }
    DJIMotorControl(); // 分组提交周期帧，默认停止输出。
    for (uint8_t round = 0u; round < 3u; ++round) CANProcessTx(); // 每条总线推进三帧。
    for (uint8_t bus = 0u; bus < DEVICE_CAN_CNT; ++bus) // 检查没有跨总线误发。
    {
        TEST_CHECK(tx_accept_count[bus] == 3u); // 每条总线恰好收到三组。
        TEST_CHECK(tx_id_history[bus][0] == 0x1ffu); // 第一组为 0x1ff。
        TEST_CHECK(tx_id_history[bus][1] == 0x200u); // 第二组为 0x200。
        TEST_CHECK(tx_id_history[bus][2] == 0x2ffu); // 第三组为 0x2ff。
        for (uint8_t group = 0u; group < 3u; ++group) // 默认停止的电机不得输出非零值。
            for (uint8_t byte = 0u; byte < 8u; ++byte) // 检查组帧中的每个电机字节。
                TEST_CHECK(tx_data_history[bus][group][byte] == 0u); // 未启用时输出保持零。
    }
}

static void test_production_registration_rollback(void)
{
    Motor_Init_Config_s config = config_for(2u, 4u);
#if defined(LK_TEST_ALLOC_WRAP)
    unsigned alloc_before = malloc_calls;
    unsigned free_before = free_calls;
#endif
    filter_fail_once = 1u;
    TEST_CHECK(LKMotorInit(&config) == NULL);
#if defined(LK_TEST_ALLOC_WRAP)
    TEST_CHECK(malloc_calls == alloc_before + 3u);
    TEST_CHECK(free_calls == free_before + 3u);
#endif
    TEST_CHECK(filter_count[2] == 0u);

    LKMotorInstance *motor = LKMotorInit(&config);
    TEST_CHECK(motor != NULL);
    TEST_CHECK(motor->motor_id == 4u && motor->can_bus_index == 2u);
    TEST_CHECK(((CANInstance *)motor->can_rx.backend)->rx_id == 0x144u);
    TEST_CHECK(((CANInstance *)motor->can_rx.backend)->tx_id == 0x280u);
    TEST_CHECK(((CANInstance *)motor->can_rx.backend)->txconf.Identifier == 0x280u);
    TEST_CHECK(filter_count[2] == 1u && filter_rx_id[2] == 0x144u);
}

static void test_standard_motor_rejects_extended_id(void)
{
    Motor_Init_Config_s lk = config_for(0u, 1u); // 使用已有的有效 LK 电机配置。
    lk.can_init_config.identifier_type = CAN_TX_ID_EXTENDED; // 模拟误选扩展帧。
    TEST_CHECK(LKMotorInit(&lk) == NULL); // LK 当前协议只发送标准帧，必须拒绝不一致配置。
    Motor_Init_Config_s dji = {0}; // 单独构造 DJI 电机配置。
    dji.motor_type = M3508; // 选择已支持的标准帧电机。
    dji.output_axis = (Motor_Output_Axis_Config_s){1.0f, MOTOR_AXIS_DIRECTION_NORMAL};
    dji.can_init_config.can_bus = PLATFORM_CAN_BUS_1;
    dji.can_init_config.tx_id = 1u;
    dji.can_init_config.identifier_type = CAN_TX_ID_EXTENDED; // 模拟同样的类型错误。
    dji.controller_setting_init_config.close_loop_type = OPEN_LOOP;
    TEST_CHECK(DJIMotorInit(&dji) == NULL); // 未登记端点也不能创建电机实例。
    TEST_CHECK(bus_start_count[0] == 0u); // 配置错误不能启动总线。
}

// 使用生产 CAN 入队和端点派发验证 LK 反馈原始时间及错误长度。
static void test_lk_logical_receive(void)
{
    Motor_Init_Config_s config = config_for(0u, 1u); // 将一号电机绑定到逻辑总线一。
    LKMotorInstance *motor = LKMotorInit(&config); // 注册真实平台适配及过滤器。
    TEST_CHECK(motor != NULL);
    TestRxFrame_t *item = &rx_frames[0][0]; // 构造真实 CAN 接收替身的第一帧。
    memset(item, 0, sizeof(*item));
    item->header.Identifier = 0x141u;
    item->header.RxFrameType = FDCAN_DATA_FRAME;
    item->header.IdType = FDCAN_STANDARD_ID;
    item->header.DataLength = 8u << 16u;
    item->data[1] = 42u; // 温度用于证明正确数据到达电机对象。
    item->data[6] = 0x34u;
    item->data[7] = 0x12u;
    rx_frame_count[0] = 1u;
    now_us = 123000u; // 中断记录的时间早于任务派发时间。
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    TEST_CHECK(motor->feedback_sequence == 0u); // 中断只入队，不运行解码。
    now_us = 125000u;
    CANProcessRx(); // 接收任务调用逻辑端点回调。
    TEST_CHECK(motor->feedback_sequence == 1u && motor->feedback_timestamp_ms == 123u);
    TEST_CHECK(motor->measure.ecd == 0x1234u && motor->measure.temperature == 42u);
    rx_frames[0][1] = *item; // 短帧不得刷新上一次有效反馈。
    rx_frames[0][1].header.DataLength = 7u << 16u;
    rx_frame_count[0] = 2u;
    now_us = 130000u;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    CANProcessRx();
    TEST_CHECK(motor->feedback_sequence == 1u && motor->feedback_timestamp_ms == 123u);
}

static void test_production_can_routing(void)
{
    Motor_Init_Config_s config_a = config_for(2u, 4u);
    Motor_Init_Config_s config_b = config_for(0u, 2u);
    LKMotorInstance *motor_a = LKMotorInit(&config_a);
    LKMotorInstance *motor_b = LKMotorInit(&config_b);
    TEST_CHECK(motor_a != NULL && motor_b != NULL);
    TEST_CHECK(filter_count[2] == 1u && filter_rx_id[2] == 0x144u);
    TEST_CHECK(filter_count[0] == 1u && filter_rx_id[0] == 0x142u);
    TEST_CHECK(bus_start_count[0] == 1u && bus_start_count[1] == 0u && bus_start_count[2] == 1u);

    LKMotorSetRef(motor_a, 1234.0f);
    LKMotorSetRef(motor_b, -321.0f);
    now_us = 0u;
    LKMotorControl();
    TEST_CHECK(tx_accept_count[0] == 1u && tx_accept_count[2] == 1u);
    TEST_CHECK(tx_header_id[0] == 0x280u && tx_header_id[2] == 0x280u);
    TEST_CHECK(memcmp(tx_frame[0], (uint8_t[8]){0}, 8u) == 0);
    TEST_CHECK(memcmp(tx_frame[2], (uint8_t[8]){0}, 8u) == 0);

    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(tx_accept_count[0] == 2u && tx_accept_count[2] == 2u);
    TEST_CHECK(tx_frame[0][2] == 0xBFu && tx_frame[0][3] == 0xFEu);
    TEST_CHECK(tx_frame[2][6] == 0xD2u && tx_frame[2][7] == 0x04u);

    // FIFO 忙时该总线只累计丢弃，其他总线仍成功；恢复后发送当周期的新值。
    tx_busy_mask = 1u << 2;
    LKMotorSetRef(motor_a, 777.0f);
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_accept_count[0] == 3u && tx_accept_count[2] == 2u);
    CAN_TxStats_t stats = {0};
    CANGetTxStats(&hfdcan3, &stats);
    TEST_CHECK(stats.tx_drop == 1u && stats.tx_ok == 2u);

    tx_busy_mask = 0u;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_accept_count[2] == 3u && tx_frame[2][6] == 0x09u && tx_frame[2][7] == 0x03u);

    // HAL 拒绝一条总线的提交不影响另一条；下轮仍使用最新计算值。
    tx_error_mask = 1u;
    LKMotorSetRef(motor_b, 456.0f);
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_accept_count[0] == 4u && tx_accept_count[2] == 4u);
    CANGetTxStats(&hfdcan1, &stats);
    TEST_CHECK(stats.tx_error == 1u && stats.tx_ok == 4u);

    tx_error_mask = 0u;
    LKMotorSetRef(motor_b, -654.0f);
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_accept_count[0] == 5u && tx_frame[0][2] == 0x72u && tx_frame[0][3] == 0xFDu);

    CAN_ObservationCounters_t observation = {0};
    TEST_CHECK(CANGetObservationCounters(2u, &observation) == 1u);
    TEST_CHECK(observation.active == 1u && observation.tx_ok == 5u && observation.tx_drop == 1u);
    TEST_CHECK(CANGetObservationCounters(0u, &observation) == 1u);
    TEST_CHECK(observation.tx_error == 1u && observation.tx_ok == 5u);
    CAN_ObservationCounters_t unchanged = {.tx_ok = 0xA5A5A5A5u};
    TEST_CHECK(CANGetObservationCounters(DEVICE_CAN_CNT, &unchanged) == 0u);
    TEST_CHECK(unchanged.tx_ok == 0xA5A5A5A5u);
}

static void test_closed_loop_feedback_selection(void)
{
    float external_angle = 0.5f;
    float external_speed = 0.5f;
    LKMotorInstance *motors[LK_MOTOR_PER_BUS_CNT] = {NULL};
    static const int16_t expected_current[LK_MOTOR_PER_BUS_CNT] = {5, 2, 4, 1};
    static const uint8_t expected_frame[8] = {5u, 0u, 2u, 0u, 4u, 0u, 1u, 0u};

    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
    {
        Motor_Init_Config_s config = config_for(0u, (uint32_t)index + 1u);
        config.controller_setting_init_config.close_loop_type = ALL_THREE_LOOP;
        config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
        config.controller_setting_init_config.angle_feedback_source =
            (index & 1u) != 0u ? OTHER_FEED : MOTOR_FEED;
        config.controller_setting_init_config.speed_feedback_source =
            (index & 2u) != 0u ? OTHER_FEED : MOTOR_FEED;
        config.controller_param_init_config.other_angle_feedback = MotorFeedbackTestMakeSource(
            &external_angle, MotorFeedbackTestReadRadExternal, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL);
        config.controller_param_init_config.other_speed_feedback = MotorFeedbackTestMakeSource(
            &external_speed, MotorFeedbackTestReadRadSExternal, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_EXTERNAL);
        config.controller_param_init_config.angle_PID = pid_config(2.0f, 1000.0f);
        config.controller_param_init_config.speed_PID = pid_config(3.0f, 1000.0f);
        config.controller_param_init_config.current_PID = pid_config(1.0f, 1000.0f);

        motors[index] = LKMotorInit(&config);
        TEST_CHECK(motors[index] != NULL);
        motors[index]->measure.total_angle = 0.0f;
        motors[index]->measure.speed_rads = 0.25f;
        motors[index]->measure.real_current = 0;
        LKMotorSetRef(motors[index], 1.0f);
    }

    now_us = 0u;
    LKMotorControl();
    TEST_CHECK(memcmp(tx_frame[0], (uint8_t[8]){0}, 8u) == 0);

    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(tx_header_id[0] == 0x280u);
    TEST_CHECK(memcmp(tx_frame[0], expected_frame, sizeof(expected_frame)) == 0);
    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
        TEST_CHECK((int16_t)((uint16_t)tx_frame[0][index * 2u] |
                            ((uint16_t)tx_frame[0][index * 2u + 1u] << 8u)) == expected_current[index]);
}

static void test_direction_flags(uint8_t speed_outer)
{
    LKMotorInstance *motors[LK_MOTOR_PER_BUS_CNT] = {NULL};

    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
    {
        Motor_Init_Config_s config = config_for(0u, (uint32_t)index + 1u);
        Motor_Control_Setting_s *setting = &config.controller_setting_init_config;
        setting->outer_loop_type = speed_outer ? SPEED_LOOP : ANGLE_LOOP;
        setting->close_loop_type = speed_outer ? SPEED_LOOP : (ANGLE_LOOP | SPEED_LOOP);
        config.output_axis.direction = (index & 1u) != 0u ? MOTOR_AXIS_DIRECTION_REVERSE :
                                                            MOTOR_AXIS_DIRECTION_NORMAL;
        config.controller_param_init_config.angle_PID = pid_config(4.0f, 1000.0f);
        config.controller_param_init_config.speed_PID = pid_config(4.0f, 1000.0f);

        motors[index] = LKMotorInit(&config);
        TEST_CHECK(motors[index] != NULL);
        motors[index]->measure.total_angle = 0.0f;
        motors[index]->measure.speed_rads = 0.25f;
        LKMotorSetRef(motors[index], 1.0f);
    }

    now_us = 0u;
    LKMotorControl();
    now_us = 1000u;
    LKMotorControl();
    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
    {
        int16_t actual = (int16_t)((uint16_t)tx_frame[0][index * 2u] |
                                   ((uint16_t)tx_frame[0][index * 2u + 1u] << 8u));
        const int16_t expected_positive = speed_outer ?
            ((index & 1u) != 0u ? -5 : 3) : ((index & 1u) != 0u ? -17 : 15);
        TEST_CHECK(actual == expected_positive);
        LKMotorSetRef(motors[index], -1.0f);
    }

    now_us = 2000u;
    LKMotorControl();
    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
    {
        int16_t actual = (int16_t)((uint16_t)tx_frame[0][index * 2u] |
                                   ((uint16_t)tx_frame[0][index * 2u + 1u] << 8u));
        const int16_t expected_negative = speed_outer ?
            ((index & 1u) != 0u ? 3 : -5) : ((index & 1u) != 0u ? 15 : -17);
        TEST_CHECK(actual == expected_negative);
        LKMotorSetRef(motors[index], 0.0f);
        motors[index]->measure.total_angle = 0.0f;
        motors[index]->measure.speed_rads = 0.0f;
    }

    now_us = 3000u;
    LKMotorControl();
    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
        TEST_CHECK(tx_frame[0][index * 2u] == 0u && tx_frame[0][index * 2u + 1u] == 0u);
}

static void test_current_loop_ignores_axis_direction_flags(void)
{
    Motor_Init_Config_s config = config_for(0u, 1u);
    config.controller_setting_init_config.outer_loop_type = CURRENT_LOOP;
    config.controller_setting_init_config.close_loop_type = CURRENT_LOOP;
    config.output_axis.direction = MOTOR_AXIS_DIRECTION_REVERSE;
    config.controller_param_init_config.current_PID = pid_config(0.25f, 1000.0f);
    LKMotorInstance *motor = LKMotorInit(&config);
    TEST_CHECK(motor != NULL);
    LKMotorSetRef(motor, 30.0f);

    now_us = 0u;
    LKMotorControl();
    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 7u && tx_frame[0][1] == 0u);
}

static void test_open_loop_keeps_motor_direction(void)
{
    Motor_Init_Config_s config = config_for(0u, 1u);
    config.controller_setting_init_config.outer_loop_type = OPEN_LOOP;
    config.controller_setting_init_config.close_loop_type = OPEN_LOOP;
    config.output_axis.direction = MOTOR_AXIS_DIRECTION_REVERSE;
    LKMotorInstance *motor = LKMotorInit(&config);
    TEST_CHECK(motor != NULL);
    LKMotorSetRef(motor, 9.0f);

    now_us = 0u;
    LKMotorControl();
    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 9u && tx_frame[0][1] == 0u);

    LKMotorSetRef(motor, -9.0f);
    now_us = 2000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 0xF7u && tx_frame[0][1] == 0xFFu);

    LKMotorSetRef(motor, 0.0f);
    now_us = 3000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 0u && tx_frame[0][1] == 0u);
}

static void test_invalid_direction_flags_rejected(void)
{
    Motor_Init_Config_s config = config_for(0u, 1u);
    config.controller_setting_init_config.motor_reverse_flag = (Motor_Reverse_Flag_e)2;
    TEST_CHECK(LKMotorInit(&config) == NULL);
    config = config_for(0u, 1u);
    config.controller_setting_init_config.feedback_reverse_flag = (Feedback_Reverse_Flag_e)2;
    TEST_CHECK(LKMotorInit(&config) == NULL);
    config = config_for(0u, 1u);
    config.output_axis.direction = (Motor_Axis_Direction_e)0;
    TEST_CHECK(LKMotorInit(&config) == NULL);
    TEST_CHECK(filter_count[0] == 0u && bus_start_count[0] == 0u);
}

static void test_feedforward_paths_and_recovery(void)
{
    float external_angle = 0.5f;
    float external_speed = 0.5f;
    float speed_feedforward = 0.25f;
    float current_feedforward = 0.5f;
    static const int16_t expected_current[LK_MOTOR_PER_BUS_CNT] = {5, 3, 5, 2};
    static const uint8_t expected_frame[8] = {5u, 0u, 3u, 0u, 5u, 0u, 2u, 0u};

    Motor_Init_Config_s invalid = config_for(0u, 1u);
    invalid.controller_setting_init_config.close_loop_type = ALL_THREE_LOOP;
    invalid.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    invalid.controller_setting_init_config.feedforward_flag = SPEED_FEEDFORWARD;
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    invalid.controller_setting_init_config.feedforward_flag = CURRENT_FEEDFORWARD;
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    invalid.controller_setting_init_config.feedforward_flag = FEEDFORWARD_NONE;
    invalid.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    invalid.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    invalid.controller_setting_init_config.speed_feedback_source = (Feedback_Source_e)2;
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    TEST_CHECK(filter_count[0] == 0u && bus_start_count[0] == 0u);

    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
    {
        Motor_Init_Config_s config = config_for(0u, (uint32_t)index + 1u);
        config.controller_setting_init_config.close_loop_type = ALL_THREE_LOOP;
        config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
        config.controller_setting_init_config.angle_feedback_source =
            (index == 1u || index == 3u) ? OTHER_FEED : MOTOR_FEED;
        config.controller_setting_init_config.speed_feedback_source =
            (index == 2u || index == 3u) ? OTHER_FEED : MOTOR_FEED;
        config.controller_setting_init_config.feedforward_flag =
            index == 1u ? SPEED_FEEDFORWARD : index == 2u ? CURRENT_FEEDFORWARD :
            index == 3u ? CURRENT_AND_SPEED_FEEDFORWARD : FEEDFORWARD_NONE;
        config.controller_param_init_config.other_angle_feedback =
            (index == 1u || index == 3u) ? MotorFeedbackTestMakeSource(&external_angle,
                MotorFeedbackTestReadRadExternal, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL) :
                (Motor_Signal_Source_s){0};
        config.controller_param_init_config.other_speed_feedback =
            (index == 2u || index == 3u) ? MotorFeedbackTestMakeSource(&external_speed,
                MotorFeedbackTestReadRadSExternal, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_EXTERNAL) :
                (Motor_Signal_Source_s){0};
        config.controller_param_init_config.speed_feedforward =
            (index == 1u || index == 3u) ? MotorFeedbackTestMakeSource(&speed_feedforward,
                MotorFeedbackTestReadRadSOutput, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_OUTPUT_AXIS) :
                (Motor_Signal_Source_s){0};
        config.controller_param_init_config.current_feedforward =
            (index == 2u || index == 3u) ? MotorFeedbackTestMakeSource(&current_feedforward,
                MotorFeedbackTestReadDeviceCurrentOutput, MOTOR_UNIT_DEVICE_CURRENT,
                MOTOR_REFERENCE_OUTPUT_AXIS) : (Motor_Signal_Source_s){0};
        config.controller_param_init_config.angle_PID = pid_config(2.0f, 1000.0f);
        config.controller_param_init_config.speed_PID = pid_config(3.0f, 1000.0f);
        config.controller_param_init_config.current_PID = pid_config(1.0f, 1000.0f);

        LKMotorInstance *motor = LKMotorInit(&config);
        TEST_CHECK(motor != NULL);
        TEST_CHECK(motor->speed_feedforward.context == config.controller_param_init_config.speed_feedforward.context);
        TEST_CHECK(motor->current_feedforward.context == config.controller_param_init_config.current_feedforward.context);
        motor->measure.total_angle = 0.0f;
        motor->measure.speed_rads = 0.25f;
        motor->measure.real_current = 0;
        LKMotorSetRef(motor, 1.0f);
    }

    now_us = 0u;
    LKMotorControl();
    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(memcmp(tx_frame[0], expected_frame, sizeof(expected_frame)) == 0);
    for (uint8_t index = 0u; index < LK_MOTOR_PER_BUS_CNT; ++index)
        TEST_CHECK((int16_t)((uint16_t)tx_frame[0][index * 2u] |
                            ((uint16_t)tx_frame[0][index * 2u + 1u] << 8u)) == expected_current[index]);

    // 非有限前馈只清本实例槽位；恢复有限输入后新一轮可重新计算。
    speed_feedforward = NAN;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][2] == 0u && tx_frame[0][3] == 0u);
    TEST_CHECK(tx_frame[0][0] == 5u && tx_frame[0][4] == 5u);
    TEST_CHECK(tx_frame[0][6] == 0u && tx_frame[0][7] == 0u);
    speed_feedforward = 0.25f;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][2] == 3u && tx_frame[0][3] == 0u);
    TEST_CHECK(tx_frame[0][6] == 2u && tx_frame[0][7] == 0u);
}

static void test_feedback_failure_recovery(void)
{
    float external_angle = 0.5f;
    Motor_Init_Config_s invalid = config_for(0u, 1u);
    invalid.controller_setting_init_config.close_loop_type = ANGLE_LOOP;
    invalid.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    invalid.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    invalid.controller_setting_init_config.angle_feedback_source = (Feedback_Source_e)2;
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    invalid.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    invalid.controller_setting_init_config.close_loop_type = ALL_THREE_LOOP;
    invalid.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
    invalid.controller_param_init_config.other_angle_feedback = MotorFeedbackTestMakeSource(
        &external_angle, MotorFeedbackTestReadRadExternal, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL);
    TEST_CHECK(LKMotorInit(&invalid) == NULL);
    TEST_CHECK(filter_count[0] == 0u && bus_start_count[0] == 0u);

    Motor_Init_Config_s config = config_for(0u, 1u);
    config.controller_setting_init_config.close_loop_type = ANGLE_LOOP;
    config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
    config.controller_param_init_config.other_angle_feedback = MotorFeedbackTestMakeSource(
        &external_angle, MotorFeedbackTestReadRadExternal, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL);
    config.controller_param_init_config.angle_PID = pid_config(2.0f, 1000.0f);
    LKMotorInstance *motor = LKMotorInit(&config);
    TEST_CHECK(motor != NULL);
    TEST_CHECK(motor->other_speed_feedback.read == NULL);
    motor->measure.total_angle = 0.5f * (float)RAD_2_DEGREE;
    LKMotorSetRef(motor, 1.0f);

    now_us = 0u;
    LKMotorControl();
    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 1u && tx_frame[0][1] == 0u);

    external_angle = NAN;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(motor->pid_clock.last_status == PID_NUMERIC_ERROR);
    TEST_CHECK(tx_frame[0][0] == 0u && tx_frame[0][1] == 0u);
    PID_State_t state = {0};
    TEST_CHECK(PIDGetState(&motor->angle_PID, &state) == PID_OK);
    TEST_CHECK(state.Err == 0.0f && state.Last_Err == 0.0f && state.ITerm == 0.0f &&
               state.Iout == 0.0f && state.Pout == 0.0f && state.Dout == 0.0f &&
               state.Output == 0.0f);

    external_angle = 0.25f;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 1u && tx_frame[0][1] == 0u);

    motor->motor_settings.angle_feedback_source = (Feedback_Source_e)2;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(motor->pid_clock.last_status == PID_BAD_ARGUMENT);
    TEST_CHECK(tx_frame[0][0] == 0u && tx_frame[0][1] == 0u);
    motor->motor_settings.angle_feedback_source = MOTOR_FEED;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 1u && tx_frame[0][1] == 0u);

    motor->motor_settings.close_loop_type = ANGLE_LOOP | SPEED_LOOP;
    motor->motor_settings.outer_loop_type = SPEED_LOOP;
    motor->motor_settings.speed_feedback_source = MOTOR_FEED;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(motor->pid_clock.last_status == PID_NOT_INITIALIZED);
    TEST_CHECK(tx_frame[0][0] == 0u && tx_frame[0][1] == 0u);
    motor->motor_settings.close_loop_type = ANGLE_LOOP;
    motor->motor_settings.outer_loop_type = ANGLE_LOOP;
    now_us += 1000u;
    LKMotorControl();
    TEST_CHECK(tx_frame[0][0] == 1u && tx_frame[0][1] == 0u);
}

static void test_feedforward_overflow(void)
{
    float feedforward = FLT_MAX;
    Motor_Init_Config_s config = config_for(0u, 1u);
    config.controller_setting_init_config.close_loop_type = ANGLE_LOOP;
    config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    config.controller_setting_init_config.feedforward_flag = SPEED_FEEDFORWARD;
    config.controller_param_init_config.speed_feedforward = MotorFeedbackTestMakeSource(
        &feedforward, MotorFeedbackTestReadRadSOutput, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_OUTPUT_AXIS);
    config.controller_param_init_config.angle_PID = pid_config(1.0f, FLT_MAX);
    LKMotorInstance *motor = LKMotorInit(&config);
    TEST_CHECK(motor != NULL);
    motor->measure.total_angle = 0.0f;
    LKMotorSetRef(motor, FLT_MAX);
    now_us = 0u;
    LKMotorControl();
    now_us = 1000u;
    LKMotorControl();
    TEST_CHECK(motor->pid_clock.last_status == PID_NUMERIC_ERROR);
    TEST_CHECK(tx_frame[0][0] == 0u && tx_frame[0][1] == 0u);
}

static void scheduler_rx_callback(CANInstance *instance)
{
    (void)instance;
}

static void scheduler_register(void)
{
    CAN_Init_Config_s config = {0};
    config.can_handle = &hfdcan1;
    config.tx_id = 0x280u;
    config.rx_id = 0x500u;
    config.can_module_callback = scheduler_rx_callback;
    TEST_CHECK(CANRegister(&config) != NULL);
}

static void scheduler_complete_latest(FDCAN_HandleTypeDef *handle)
{
    const uint8_t bus = bus_for_handle(handle);
    TEST_CHECK(bus < DEVICE_CAN_CNT && tx_latest_buffer[bus] != 0u);
    const uint32_t completed = tx_latest_buffer[bus];
    HAL_FDCAN_TxBufferCompleteCallback(handle, completed);
    for (uint8_t index = 0u; index < 16u; ++index)
    {
        if ((completed & (1u << index)) != 0u)
        {
            tx_next_buffer_index[bus] = index;
            break;
        }
    }
}

static CAN_TxFrame_t scheduler_frame(uint32_t id, CAN_TxIdType_t id_type,
                                     const uint8_t *data, uint8_t length)
{
    CAN_TxFrame_t frame = {
        .identifier = id,
        .identifier_type = id_type,
        .data = data,
        .length = length,
    };
    return frame;
}

static void scheduler_periodic_guard(const void *source, uint32_t now_ms,
                                     uint8_t *payload, uint8_t length)
{
    (void)now_ms;
    const uint8_t *allowed = (const uint8_t *)source;
    if (allowed == NULL || *allowed == 0u)
        memset(payload, 0, length);
}

static void test_can_tx_periodic_guard_after_rejected_replacement(void)
{
    scheduler_register();
    static const uint8_t payload_nonzero = 0xA5u;
    static const uint8_t payload_zero = 0u;
    static uint8_t allow_nonzero = 1u;
    CAN_TxFrame_t frame = scheduler_frame(0x356u, CAN_TX_ID_STANDARD,
                                          &payload_nonzero, 1u);
    uint32_t request_id = 0u;

    // 先占满所有 HAL 结果槽和硬件缓冲，随后让普通请求填满软件等待队列。
    for (uint8_t i = 0u; i < 16u; ++i)
    {
        const CAN_TxFrame_t filler = scheduler_frame(0x400u + i,
            CAN_TX_ID_STANDARD, &payload_nonzero, 1u);
        TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &filler, CAN_TX_PRIORITY_NORMAL,
                                    100u, &request_id) == CAN_TX_SUBMIT_ACCEPTED);
        CANProcessTx();
    }
    tx_busy_mask = 1u;
    for (uint8_t i = 0u; i < 15u; ++i)
    {
        const CAN_TxFrame_t waiting = scheduler_frame(0x500u + i,
            CAN_TX_ID_STANDARD, &payload_nonzero, 1u);
        TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &waiting, CAN_TX_PRIORITY_BACKGROUND,
                                    100u, &request_id) == CAN_TX_SUBMIT_ACCEPTED);
    }

    request_id = 0xA5A5A5A5u; // 之前的填充请求已使用此变量，重新设置以检查失败不改写。
    TEST_CHECK(PlatformCanSubmitPeriodic(PLATFORM_CAN_INVALID, &allow_nonzero, 1u, &frame,
        CAN_TX_PRIORITY_CONTROL, 100u, 1000u, scheduler_periodic_guard,
        &request_id) == CAN_TX_SUBMIT_NOT_READY);
    TEST_CHECK(request_id == 0xA5A5A5A5u); // 无绑定时不能改写请求编号。
    TEST_CHECK(PlatformCanSubmitPeriodic(PLATFORM_CAN_BUS_1, &allow_nonzero, 1u, &frame,
        CAN_TX_PRIORITY_CONTROL, 100u, 1000u, scheduler_periodic_guard,
        &request_id) == CAN_TX_SUBMIT_ACCEPTED);
    const uint32_t guarded_request_id = request_id;

    // 结果记录已满时，新零值提交失败，旧非零周期项仍留在队列中。
    frame.data = &payload_zero;
    uint32_t rejected_id = 0xA5A5A5A5u;
    TEST_CHECK(PlatformCanSubmitPeriodic(PLATFORM_CAN_BUS_1, &allow_nonzero, 1u, &frame,
        CAN_TX_PRIORITY_CONTROL, 100u, 1000u, scheduler_periodic_guard,
        &rejected_id) == CAN_TX_SUBMIT_FULL);
    TEST_CHECK(rejected_id == 0xA5A5A5A5u);

    // 撤销许可后即使队列内仍是旧非零副本，真正交给 HAL 前也会被改为零。
    allow_nonzero = 0u;
    scheduler_complete_latest(&hfdcan1);
    tx_busy_mask = 0u;
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 17u);
    TEST_CHECK(tx_id_history[0][16] == 0x356u);
    TEST_CHECK(tx_data_history[0][16][0] == 0u);
    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(guarded_request_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_IN_FLIGHT);
}

static void test_can_tx_arguments_and_copy(void)
{
    TEST_CHECK(CANClassicFrameWorstCaseBits(CAN_TX_ID_STANDARD, 8u) == 136u);
    TEST_CHECK(CANClassicFrameWorstCaseBits(CAN_TX_ID_EXTENDED, 8u) == 161u);
    TEST_CHECK(CANClassicFrameWorstCaseBits((CAN_TxIdType_t)2, 8u) == 0u);
    TEST_CHECK(CANClassicFrameWorstCaseBits(CAN_TX_ID_STANDARD, 9u) == 0u);

    CAN_TxBusConfig_t bus_config = {
        .queue_capacity = 4u,
        .utilization_limit_permille = 500u,
        .nominal_bitrate_bps = 1000000u,
    };
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, NULL) == CAN_TX_SUBMIT_INVALID);
    bus_config.queue_capacity = 0u;
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, &bus_config) == CAN_TX_SUBMIT_INVALID);
    bus_config.queue_capacity = 4u;
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, &bus_config) == CAN_TX_SUBMIT_ACCEPTED);

    uint8_t payload[8] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};
    CAN_TxFrame_t frame = scheduler_frame(0x123u, CAN_TX_ID_STANDARD, payload, sizeof(payload));
    uint32_t request_id = 0xA5A5A5A5u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &request_id) == CAN_TX_SUBMIT_NOT_READY);
    TEST_CHECK(request_id == 0xA5A5A5A5u);
    scheduler_register();
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, &bus_config) == CAN_TX_SUBMIT_NOT_READY);

    frame = scheduler_frame(0x800u, CAN_TX_ID_STANDARD, payload, sizeof(payload));
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &request_id) == CAN_TX_SUBMIT_INVALID);
    frame = scheduler_frame(0x20000000u, CAN_TX_ID_EXTENDED, payload, sizeof(payload));
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &request_id) == CAN_TX_SUBMIT_INVALID);
    frame = scheduler_frame(0x123u, CAN_TX_ID_STANDARD, payload, 9u);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &request_id) == CAN_TX_SUBMIT_INVALID);
    frame = scheduler_frame(0x123u, CAN_TX_ID_STANDARD, NULL, 1u);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &request_id) == CAN_TX_SUBMIT_INVALID);
    frame = scheduler_frame(0x123u, CAN_TX_ID_STANDARD, payload, sizeof(payload));
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, (CAN_TxPriority_t)4,
                                20u, &request_id) == CAN_TX_SUBMIT_INVALID);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, (CAN_TxPriority_t)-1,
                                20u, &request_id) == CAN_TX_SUBMIT_INVALID);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                0u, &request_id) == CAN_TX_SUBMIT_INVALID);
    TEST_CHECK(request_id == 0xA5A5A5A5u);

    test_ipsr = 1u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &request_id) == CAN_TX_SUBMIT_CONTEXT);
    test_ipsr = 0u;

    static const uint8_t expected_payload[8] = {0x10u, 0x21u, 0x32u, 0x43u,
                                                0x54u, 0x65u, 0x76u, 0x87u};
    for (uint8_t length = 0u; length <= 8u; ++length)
    {
        uint8_t source_data[8];
        memcpy(source_data, expected_payload, sizeof(source_data));
        const CAN_TxIdType_t id_type = (length & 1u) != 0u ?
            CAN_TX_ID_EXTENDED : CAN_TX_ID_STANDARD;
        const uint32_t id = id_type == CAN_TX_ID_STANDARD ?
            (length == 0u ? 0u : length == 2u ? 0x7ffu : 0x321u) :
            (length == 1u ? 0u : length == 3u ? 0x1fffffffu : 0x1234567u);
        frame = scheduler_frame(id, id_type, length == 0u ? NULL : source_data, length);
        now_us = (uint64_t)length * 1000u;
        request_id = 0u;
        TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                    20u, &request_id) == CAN_TX_SUBMIT_ACCEPTED);
        memset(source_data, 0xEE, sizeof(source_data));
        CANProcessTx();

        const uint32_t history_index = length;
        TEST_CHECK(tx_id_history[0][history_index] == id);
        TEST_CHECK(tx_id_type_history[0][history_index] == (uint8_t)id_type);
        TEST_CHECK(tx_length_history[0][history_index] == length);
        TEST_CHECK(memcmp(tx_data_history[0][history_index], expected_payload, length) == 0);
        scheduler_complete_latest(&hfdcan1);

        CAN_TxResult_t result = {0};
        TEST_CHECK(CANGetTxRequestState(request_id, &result) == 1u);
        TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);
        TEST_CHECK(result.accepted_frames == 1u && result.completed_frames == 1u);
    }

    CAN_TxSchedulerStats_t stats = {0};
    TEST_CHECK(CANGetTxSchedulerStats(&hfdcan1, &stats) == 1u);
    TEST_CHECK(stats.completed == 9u && stats.queued == 0u);
}

static void test_can_tx_queue_batch_and_buses(void)
{
    const CAN_TxBusConfig_t bus0_config = {
        .queue_capacity = 2u,
        .utilization_limit_permille = 500u,
        .nominal_bitrate_bps = 1000000u,
    };
    const CAN_TxBusConfig_t bus1_config = {
        .queue_capacity = 1u,
        .utilization_limit_permille = 400u,
        .nominal_bitrate_bps = 500000u,
    };
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, &bus0_config) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANConfigureTxBus(&hfdcan2, &bus1_config) == CAN_TX_SUBMIT_ACCEPTED);
    scheduler_register();
    TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_2)); // 纯发送总线由启动计划单独启用。

    const uint8_t a_data[2] = {0xA1u, 0xA2u};
    const uint8_t b_data[2] = {0xB1u, 0xB2u};
    const uint8_t c_data[1] = {0xC1u};
    const CAN_TxFrame_t batch[2] = {
        {0x222u, CAN_TX_ID_STANDARD, a_data, sizeof(a_data)},
        {0x223u, CAN_TX_ID_STANDARD, b_data, sizeof(b_data)},
    };
    const CAN_TxFrame_t urgent = scheduler_frame(0x111u, CAN_TX_ID_STANDARD,
                                                  c_data, sizeof(c_data));
    const CAN_TxFrame_t other_bus = scheduler_frame(0x400u, CAN_TX_ID_STANDARD,
                                                     c_data, sizeof(c_data));
    uint32_t batch_id = 0u;
    uint32_t urgent_id = 0u;
    uint32_t other_bus_id = 0u;
    TEST_CHECK(CANSubmitOneShotBatch(&hfdcan1, NULL, batch, 2u, CAN_TX_PRIORITY_NORMAL,
                                     30u, &batch_id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &urgent, CAN_TX_PRIORITY_URGENT,
                                30u, &urgent_id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANSubmitOneShot(&hfdcan2, NULL, &other_bus, CAN_TX_PRIORITY_NORMAL,
                                30u, &other_bus_id) == CAN_TX_SUBMIT_ACCEPTED);

    uint32_t rejected_id = 0x5A5A5A5Au;
    TEST_CHECK(CANSubmitOneShotBatch(&hfdcan1, NULL, batch, 2u, CAN_TX_PRIORITY_NORMAL,
                                     30u, &rejected_id) == CAN_TX_SUBMIT_FULL);
    TEST_CHECK(rejected_id == 0x5A5A5A5Au);
    CAN_TxSchedulerStats_t stats = {0};
    TEST_CHECK(CANGetTxSchedulerStats(&hfdcan1, &stats) == 1u);
    TEST_CHECK(stats.queued == 2u && stats.queue_peak == 2u);
    TEST_CHECK(CANGetTxSchedulerStats(&hfdcan2, &stats) == 1u && stats.queued == 1u);

    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 1u && tx_id_history[0][0] == 0x111u);
    TEST_CHECK(tx_accept_count[1] == 1u && tx_id_history[1][0] == 0x400u);
    scheduler_complete_latest(&hfdcan1);
    scheduler_complete_latest(&hfdcan2);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 2u && tx_id_history[0][1] == 0x222u);
    scheduler_complete_latest(&hfdcan1);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 3u && tx_id_history[0][2] == 0x223u);
    scheduler_complete_latest(&hfdcan1);

    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(batch_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE && result.completed_frames == 2u);
    TEST_CHECK(CANGetTxRequestState(urgent_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);
    TEST_CHECK(CANGetTxRequestState(other_bus_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);
    TEST_CHECK(CANCancelTxRequest(urgent_id) == 0u);

    CAN_TxFrame_t invalid_batch[2] = {batch[0], batch[1]};
    invalid_batch[1].identifier = 0x800u;
    rejected_id = 0x5A5A5A5Au;
    TEST_CHECK(CANSubmitOneShotBatch(&hfdcan1, NULL, invalid_batch, 2u,
                                     CAN_TX_PRIORITY_NORMAL, 30u,
                                     &rejected_id) == CAN_TX_SUBMIT_INVALID);
    TEST_CHECK(rejected_id == 0x5A5A5A5Au);
    CAN_TxFrame_t too_many[CAN_TX_MAX_BATCH_FRAMES + 1u];
    for (size_t i = 0u; i < CAN_TX_MAX_BATCH_FRAMES + 1u; ++i)
        too_many[i] = batch[0];
    TEST_CHECK(CANSubmitOneShotBatch(&hfdcan1, NULL, too_many,
                                     CAN_TX_MAX_BATCH_FRAMES + 1u,
                                     CAN_TX_PRIORITY_NORMAL, 30u,
                                     &rejected_id) == CAN_TX_SUBMIT_INVALID);

    // 64字节话题连同时间和校验按新版格式需要13帧，必须整组提交。
    TEST_CHECK(CAN_TX_MAX_BATCH_FRAMES >= 13u);
    CAN_TxFrame_t maximum_batch[CAN_TX_MAX_BATCH_FRAMES];
    for (size_t i = 0u; i < CAN_TX_MAX_BATCH_FRAMES; ++i)
    {
        maximum_batch[i] = batch[0];
        maximum_batch[i].identifier = 0x600u + (uint32_t)i;
    }
    uint32_t maximum_batch_id = 0u;
    TEST_CHECK(CANSubmitOneShotBatch(&hfdcan1, NULL, maximum_batch,
                                     CAN_TX_MAX_BATCH_FRAMES,
                                     CAN_TX_PRIORITY_NORMAL, 30u,
                                     &maximum_batch_id) == CAN_TX_SUBMIT_ACCEPTED);
    for (size_t i = 0u; i < CAN_TX_MAX_BATCH_FRAMES; ++i)
    {
        CANProcessTx();
        TEST_CHECK(tx_id_history[0][3u + i] == 0x600u + i);
        scheduler_complete_latest(&hfdcan1);
    }
    TEST_CHECK(CANGetTxRequestState(maximum_batch_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE &&
               result.completed_frames == CAN_TX_MAX_BATCH_FRAMES);

    uint32_t later_deadline_id = 0u;
    uint32_t earlier_deadline_id = 0u;
    const CAN_TxFrame_t later_deadline = scheduler_frame(0x501u, CAN_TX_ID_STANDARD,
                                                          c_data, sizeof(c_data));
    const CAN_TxFrame_t earlier_deadline = scheduler_frame(0x500u, CAN_TX_ID_STANDARD,
                                                            c_data, sizeof(c_data));
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &later_deadline,
                                CAN_TX_PRIORITY_NORMAL, 20u,
                                &later_deadline_id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &earlier_deadline,
                                CAN_TX_PRIORITY_NORMAL, 10u,
                                &earlier_deadline_id) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(tx_id_history[0][3u + CAN_TX_MAX_BATCH_FRAMES] == 0x500u);
    scheduler_complete_latest(&hfdcan1);
    CANProcessTx();
    TEST_CHECK(tx_id_history[0][4u + CAN_TX_MAX_BATCH_FRAMES] == 0x501u);
    scheduler_complete_latest(&hfdcan1);
    TEST_CHECK(CANGetTxRequestState(later_deadline_id, &result) == 1u &&
               result.state == CAN_TX_RESULT_COMPLETE);
    TEST_CHECK(CANGetTxRequestState(earlier_deadline_id, &result) == 1u &&
               result.state == CAN_TX_RESULT_COMPLETE);
}

// 生产调度器按组发送完整话题，并将内容从实际提交的帧中还原。
static void test_can_tx_topic_group(void)
{
    CAN_Init_Config_s config={0};
    config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config);
    TEST_CHECK(can != NULL);
    CanTopicTransport_t transport;
    TEST_CHECK(CanTopicTransportInit(&transport,PLATFORM_CAN_BUS_1,can->tx_id,can));
    CanTopicMessage_t input={.topic_id=7u,.length=64u,.source_ms=25u};
    for (unsigned i=0u;i<64u;++i) input.payload[i]=(uint8_t)i;
    uint32_t topic_id=0u;
    TEST_CHECK(CanTopicTransportSend(&transport,&input,30u,&topic_id)==CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx(); scheduler_complete_latest(&hfdcan1);
    uint8_t normal_byte=0u; uint32_t normal_id=0u;
    CAN_TxFrame_t normal=scheduler_frame(0x102u,CAN_TX_ID_STANDARD,&normal_byte,1u);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1,NULL,&normal,CAN_TX_PRIORITY_NORMAL,30u,&normal_id)==CAN_TX_SUBMIT_ACCEPTED);
    for (size_t i=1u;i<CAN_TOPIC_MAX_FRAMES;++i) {
        CANProcessTx(); scheduler_complete_latest(&hfdcan1);
        TEST_CHECK(tx_id_history[0][i]==0x321u);
    }
    CANProcessTx(); scheduler_complete_latest(&hfdcan1);
    TEST_CHECK(tx_id_history[0][CAN_TOPIC_MAX_FRAMES]==0x102u);
    CAN_TxResult_t result;
    TEST_CHECK(CANGetTxRequestState(topic_id,&result) && result.state==CAN_TX_RESULT_COMPLETE && result.completed_frames==13u);
    CanTopicReceiver_t receiver={0}; CanTopicMessage_t output={0};
    for (size_t i=0u;i<CAN_TOPIC_MAX_FRAMES;++i) {
        CanTopicFrame_t frame={.length=tx_length_history[0][i]};
        memcpy(frame.bytes,tx_data_history[0][i],frame.length);
        CanTopicWireStatus_t status=CanTopicReceive(&receiver,&frame,(uint32_t)i,(uint32_t)i,50u,&output);
        TEST_CHECK(status==(i+1u==CAN_TOPIC_MAX_FRAMES ? CAN_TOPIC_OK : CAN_TOPIC_INCOMPLETE));
    }
    TEST_CHECK(output.source_ms==25u && output.first_rx_ms==0u && output.length==64u && output.payload[63]==63u);
}

// 控制请求到达后放弃可中断的旧话题组，不让剩余分片占用控制期限。
static void test_can_tx_topic_interrupt(void)
{
    CAN_Init_Config_s config={0};
    config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config); TEST_CHECK(can != NULL);
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,PLATFORM_CAN_BUS_1,can->tx_id,can));
    CanTopicMessage_t input={.topic_id=7u,.length=64u,.source_ms=25u};
    uint32_t topic_id=0u;
    TEST_CHECK(CanTopicTransportSend(&transport,&input,30u,&topic_id)==CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx(); scheduler_complete_latest(&hfdcan1);
    uint8_t byte=0u; uint32_t control_id=0u;
    CAN_TxFrame_t control=scheduler_frame(0x101u,CAN_TX_ID_STANDARD,&byte,1u);
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1,&byte,1u,&control,CAN_TX_PRIORITY_CONTROL,4u,1000u,&control_id)==CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx(); scheduler_complete_latest(&hfdcan1);
    TEST_CHECK(tx_id_history[0][1u]==0x101u);
    CAN_TxResult_t result;
    TEST_CHECK(CANGetTxRequestState(topic_id,&result) && result.state==CAN_TX_RESULT_FAILED && result.accepted_frames==1u);
    CANProcessTx(); TEST_CHECK(tx_accept_count[0]==2u);
}

// 来源和本机时钟在此测试中相同；真实双板必须由对时模块提供映射。
static uint8_t test_topic_map(void *context,uint32_t remote,uint32_t now,uint32_t *local,uint32_t *uncertainty)
{
    (void)now;
    if (!*(uint8_t *)context) return 0u;
    *local=remote; *uncertainty=0u; return 1u;
}
static uint8_t test_topic_copy(void *context,const void *input,size_t input_size,void *output,size_t output_size)
{
    (void)context;
    if (input_size!=4u || output_size!=4u) return 0u;
    memcpy(output,input,4u); return 1u;
}
// 从生产中断队列进入CAN派发、拼包、映射和生产消息中心。
static void test_can_tx_topic_rx(void)
{
    const MessageTopicDescriptor_t descriptor={19u,4u,50u,1u,MESSAGE_DELIVERY_SNAPSHOT,
        MESSAGE_OVERFLOW_REPLACE_OLDEST,MESSAGE_CLOCK_LOCAL_MAPPED,1u,2u,0u};
    TEST_CHECK(MessageCenterInit(&descriptor,1u)==MESSAGE_OK);
    MessagePublisher_t *publisher=MessagePublisherRegister(19u,1u);
    MessageSubscriber_t *subscriber=MessageSubscriberRegister(19u,2u);
    TEST_CHECK(publisher && subscriber);
    CanTopicLink_t link={0}; uint8_t synchronized=1u;
    const CanTopicCodec_t codec={test_topic_copy,NULL,4u};
    PlatformCanEndpoint_t config={.can_bus=PLATFORM_CAN_BUS_1,.tx_id=0x321u,.rx_id=0x501u};
    TEST_CHECK(CanTopicLinkInit(&link,&config,NULL,publisher,NULL,&codec,test_topic_map,&synchronized,50u));
    CanTopicMessage_t message={.topic_id=19u,.length=4u,.source_ms=100u,.payload={1u,2u,3u,4u}};
    CanTopicFrame_t frames[CAN_TOPIC_MAX_FRAMES]; size_t count=0u;
    TEST_CHECK(CanTopicEncode(&message,1u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    for (size_t i=0u;i<count;++i) {
        TestRxFrame_t *item=&rx_frames[0][i]; memset(item,0,sizeof(*item));
        item->header.Identifier=0x501u; item->header.DataLength=(uint32_t)frames[i].length<<16u;
        item->header.RxFrameType=FDCAN_DATA_FRAME; item->header.IdType=FDCAN_STANDARD_ID;
        memcpy(item->data,frames[i].bytes,frames[i].length);
    }
    rx_frame_count[0]=(uint8_t)count; rx_frame_read[0]=0u; now_us=100000u;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    CANProcessRx();
    uint8_t output[4]={0}; MessageFreshness_t freshness;
    TEST_CHECK(link.last_receive==CAN_BRIDGE_OK);
    TEST_CHECK(MessageReadSnapshot(subscriber,output,sizeof(output),100u,&freshness)==MESSAGE_OK &&
               memcmp(output,message.payload,4u)==0 && freshness.received_time_ms==100u);
    message.source_ms=101u;
    TEST_CHECK(CanTopicEncode(&message,2u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    frames[count-1u].bytes[frames[count-1u].length-1u]^=1u;
    for (size_t i=0u;i<count;++i) {
        rx_frames[0][i].header.DataLength=(uint32_t)frames[i].length<<16u;
        memcpy(rx_frames[0][i].data,frames[i].bytes,frames[i].length);
    }
    rx_frame_count[0]=(uint8_t)count; rx_frame_read[0]=0u; now_us=101000u;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    CANProcessRx();
    TEST_CHECK(link.last_receive==CAN_BRIDGE_WIRE);
    TEST_CHECK(MessageReadSnapshot(subscriber,output,sizeof(output),101u,&freshness)==MESSAGE_OK &&
               freshness.received_time_ms==100u && freshness.source_time_ms==100u);
    synchronized=0u; uint32_t id=0u; CanTopicBridgeStatus_t sync;
    TEST_CHECK(CanTopicLinkService(&link,101u,&id,&sync)==CAN_BRIDGE_UNSYNCED);
    TEST_CHECK(MessageReadSnapshot(subscriber,output,sizeof(output),101u,&freshness)==MESSAGE_ERR_EMPTY);
}

// 新版本到达时停止旧组，已经进入硬件的帧结束前不能提交新组。
static void test_can_tx_topic_cancel(void)
{
    const MessageTopicDescriptor_t descriptor={19u,4u,50u,1u,MESSAGE_DELIVERY_SNAPSHOT,
        MESSAGE_OVERFLOW_REPLACE_OLDEST,MESSAGE_CLOCK_LOCAL,1u,2u,0u};
    TEST_CHECK(MessageCenterInit(&descriptor,1u)==MESSAGE_OK);
    MessagePublisher_t *publisher=MessagePublisherRegister(19u,1u);
    MessageSubscriber_t *subscriber=MessageSubscriberRegister(19u,2u);
    TEST_CHECK(publisher && subscriber);
    CAN_Init_Config_s config={0}; config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x501u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config); TEST_CHECK(can!=NULL);
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,PLATFORM_CAN_BUS_1,can->tx_id,can));
    const CanTopicCodec_t codec={test_topic_copy,NULL,4u};
    CanTopicBridge_t bridge;
    TEST_CHECK(CanTopicBridgeInit(&bridge,&transport,subscriber,NULL,&codec,NULL,NULL,NULL)==CAN_BRIDGE_OK);
    uint32_t value=1u, first_id=0u, second_id=0xabcdu;
    TEST_CHECK(MessagePublishAt(publisher,&value,0u,0u,0u,MESSAGE_CLOCK_LOCAL)==MESSAGE_OK);
    TEST_CHECK(CanTopicBridgeSend(&bridge,0u,&first_id)==CAN_BRIDGE_OK);
    CANProcessTx(); uint32_t first_mask=tx_latest_buffer[0];
    value=2u; now_us=1000u;
    TEST_CHECK(MessagePublishAt(publisher,&value,1u,1u,1u,MESSAGE_CLOCK_LOCAL)==MESSAGE_OK);
    TEST_CHECK(CanTopicBridgeSend(&bridge,1u,&second_id)==CAN_BRIDGE_SEND && second_id==0xabcdu);
    CAN_TxResult_t result;
    TEST_CHECK(CANGetTxRequestState(first_id,&result) && result.state==CAN_TX_RESULT_CANCELLED && result.pending_frames==1u);
    HAL_FDCAN_TxBufferCompleteCallback(&hfdcan1,first_mask);
    TEST_CHECK(CanTopicBridgeSend(&bridge,1u,&second_id)==CAN_BRIDGE_OK && second_id!=0xabcdu);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0]==2u && tx_data_history[0][1u][0]==0xa0u && tx_data_history[0][1u][1]==1u);
}

// 同总线每轮都有控制帧时，话题期限耗尽必须报告过期，不能伪称已发送。
static void test_can_tx_topic_contention(void)
{
    CAN_Init_Config_s config={0}; config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config); TEST_CHECK(can!=NULL);
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,PLATFORM_CAN_BUS_1,can->tx_id,can));
    CanTopicMessage_t message={.topic_id=7u,.length=64u}; uint32_t topic_id=0u;
    TEST_CHECK(CanTopicTransportSend(&transport,&message,5u,&topic_id)==CAN_TX_SUBMIT_ACCEPTED);
    uint8_t data=0u; CAN_TxFrame_t control=scheduler_frame(0x101u,CAN_TX_ID_STANDARD,&data,1u);
    for (unsigned i=0u;i<5u;++i) {
        now_us=(uint64_t)i*1000u; uint32_t control_id=0u;
        TEST_CHECK(CANSubmitPeriodic(&hfdcan1,&data,1u,&control,CAN_TX_PRIORITY_CONTROL,4u,1000u,
                                     &control_id)==CAN_TX_SUBMIT_ACCEPTED);
        CANProcessTx(); scheduler_complete_latest(&hfdcan1);
        TEST_CHECK(tx_id_history[0][i]==0x101u);
    }
    now_us=5000u; CANProcessTx();
    CAN_TxResult_t result;
    TEST_CHECK(CANGetTxRequestState(topic_id,&result) && result.state==CAN_TX_RESULT_EXPIRED &&
               result.accepted_frames==0u);
}

// 专用桥接总线每5ms产生5帧，并故意漏掉每组首轮服务，验证有界补充推进。
static void test_can_tx_topic_drain(void)
{
    CAN_Init_Config_s config={0}; config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config); TEST_CHECK(can!=NULL);
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,PLATFORM_CAN_BUS_1,can->tx_id,can));
    CanTopicMessage_t message={.topic_id=7u,.length=21u}; uint32_t ids[6]={0};
    for (unsigned tick=0u;tick<30u;++tick) {
        now_us=(uint64_t)tick*1000u;
        if (tick%5u==0u) {
            message.source_ms=tick;
            TEST_CHECK(CanTopicTransportSend(&transport,&message,50u,&ids[tick/5u])==CAN_TX_SUBMIT_ACCEPTED);
            if (tick==0u) {
                CANProcessTxBus(NULL);
                test_ipsr=1u; CANProcessTxBus(&hfdcan1); test_ipsr=0u;
                TEST_CHECK(tx_accept_count[0]==0u);
            }
            continue; // 模拟服务任务本轮没有得到调度。
        }
        for (unsigned attempt=0u;attempt<2u;++attempt) {
            uint32_t before=tx_accept_count[0];
            CANProcessTxBus(&hfdcan1);
            if (tx_accept_count[0]!=before) scheduler_complete_latest(&hfdcan1);
        }
    }
    for (unsigned i=0u;i<6u;++i) {
        CAN_TxResult_t result;
        TEST_CHECK(CANGetTxRequestState(ids[i],&result) && result.state==CAN_TX_RESULT_COMPLETE && result.completed_frames==5u);
    }
    TEST_CHECK(tx_accept_count[0]==30u && tx_accept_count[1]==0u && tx_accept_count[2]==0u);
}

// 任务在计算期限后被抢占，入队时应保留原来的绝对截止时刻。
static void test_can_tx_topic_absolute_deadline(void)
{
    CAN_Init_Config_s config={0};
    config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    TEST_CHECK(CANRegister(&config)!=NULL);
    const uint8_t data=0x11u;
    const CAN_TxFrame_t frame=scheduler_frame(0x321u,CAN_TX_ID_STANDARD,&data,1u);
    uint32_t id=99u;
    now_us=115000u;
    TEST_CHECK(CANSubmitInterruptibleBatchUntil(&hfdcan1,&data,&frame,1u,
        CAN_TX_PRIORITY_NORMAL,150u,&id)==CAN_TX_SUBMIT_ACCEPTED);
    now_us=150000u;
    CANProcessTx();
    CAN_TxResult_t result;
    TEST_CHECK(CANGetTxRequestState(id,&result) && result.state==CAN_TX_RESULT_EXPIRED);
    id=99u;
    TEST_CHECK(CANSubmitInterruptibleBatchUntil(&hfdcan1,&data,&frame,1u,
        CAN_TX_PRIORITY_NORMAL,150u,&id)==CAN_TX_SUBMIT_INVALID && id==99u);
    now_us=(uint64_t)(UINT32_MAX-2u)*1000u;
    TEST_CHECK(CANSubmitInterruptibleBatchUntil(&hfdcan1,&data,&frame,1u,
        CAN_TX_PRIORITY_NORMAL,2u,&id)==CAN_TX_SUBMIT_ACCEPTED);
    now_us=(uint64_t)2u*1000u;
    CANProcessTx();
    TEST_CHECK(CANGetTxRequestState(id,&result) && result.state==CAN_TX_RESULT_EXPIRED);
}

// 模拟专用桥接总线同时发送底盘话题、对时和可选电容命令。
static void test_can_tx_topic_sync_budget(void)
{
    CAN_Init_Config_s config={0}; config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config); TEST_CHECK(can!=NULL);
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,PLATFORM_CAN_BUS_1,can->tx_id,can));
    CanTimeSync_t sync={0}; TEST_CHECK(CanTimeSyncInit(&sync,PLATFORM_CAN_BUS_1,can->tx_id,can));
    CanTopicMessage_t message={.topic_id=7u,.length=21u};
    uint32_t ids[6]={0u};
    uint8_t cap_data[8]={0u};
    CAN_TxFrame_t cap=scheduler_frame(0x302u,CAN_TX_ID_STANDARD,cap_data,8u);
    // 固定 CRC8/0x31 向量：请求编号0x1234，发送板时间1000ms。
    const uint8_t peer_probe[8]={0xc1u,0x34u,0x12u,0xe8u,0x03u,0u,0u,0x84u};
    for (unsigned tick=0u;tick<30u;++tick) {
        now_us=(uint64_t)tick*1000u;
        if (tick%5u==0u) {
            if (tick>0u) {
                CAN_TxResult_t previous;
                TEST_CHECK(CANGetTxRequestState(ids[tick/5u-1u],&previous) &&
                           previous.state==CAN_TX_RESULT_COMPLETE && previous.completed_frames==5u);
            }
            if (tick==0u) TEST_CHECK(CanTimeSyncService(&sync,0u)==CAN_TIME_SYNC_OK);
            if (tick==10u)
                TEST_CHECK(CanTimeSyncReceive(&sync,peer_probe,8u,10u,10u)==CAN_TIME_SYNC_OK);
            if (tick%10u==0u) {
                uint32_t cap_id=0u;
                TEST_CHECK(CANSubmitPeriodic(&hfdcan1,&cap_data,1u,&cap,CAN_TX_PRIORITY_NORMAL,
                                             20u,10000u,&cap_id)==CAN_TX_SUBMIT_ACCEPTED);
            }
            message.source_ms=tick;
            TEST_CHECK(CanTopicTransportSend(&transport,&message,50u,&ids[tick/5u])==CAN_TX_SUBMIT_ACCEPTED);
            continue; // 每组首轮不推进，给任务调度抖动留出一轮余量。
        }
        for (unsigned attempt=0u;attempt<2u;++attempt) {
            const uint32_t before=tx_accept_count[0];
            CANProcessTxBus(&hfdcan1);
            if (tx_accept_count[0]!=before) scheduler_complete_latest(&hfdcan1);
        }
    }
    CAN_TxResult_t final;
    TEST_CHECK(CANGetTxRequestState(ids[5],&final) && final.state==CAN_TX_RESULT_COMPLETE &&
               final.completed_frames==5u);
    TEST_CHECK(tx_accept_count[0]==36u && tx_accept_count[1]==0u && tx_accept_count[2]==0u);
}

// 即使有额外推进机会，共用总线上的新控制帧仍会中止未发送完的话题。
static void test_can_tx_topic_shared(void)
{
    CAN_Init_Config_s config={0}; config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config); TEST_CHECK(can!=NULL);
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,PLATFORM_CAN_BUS_1,can->tx_id,can));
    CanTopicMessage_t message={.topic_id=7u,.length=21u}; uint32_t topic_id=0u;
    TEST_CHECK(CanTopicTransportSend(&transport,&message,50u,&topic_id)==CAN_TX_SUBMIT_ACCEPTED);
    uint8_t data=0u; CAN_TxFrame_t control=scheduler_frame(0x101u,CAN_TX_ID_STANDARD,&data,1u);
    uint32_t control_id=0u;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1,&data,1u,&control,CAN_TX_PRIORITY_CONTROL,4u,1000u,&control_id)==CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx(); scheduler_complete_latest(&hfdcan1);
    CANProcessTxBus(&hfdcan1); scheduler_complete_latest(&hfdcan1);
    TEST_CHECK(tx_id_history[0][0u]==0x101u && tx_id_history[0][1u]==0x321u);
    now_us=1000u;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1,&data,1u,&control,CAN_TX_PRIORITY_CONTROL,4u,1000u,&control_id)==CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx(); scheduler_complete_latest(&hfdcan1);
    CANProcessTxBus(&hfdcan1);
    CAN_TxResult_t result;
    TEST_CHECK(tx_accept_count[0]==3u && tx_id_history[0][2u]==0x101u);
    TEST_CHECK(CANGetTxRequestState(topic_id,&result) && result.state==CAN_TX_RESULT_FAILED && result.accepted_frames==1u);
}

// 同一轮两帧可同时被HAL接受，完成回调逆序时仍要等待整组结束。
static void test_can_tx_topic_pending(void)
{
    CAN_Init_Config_s config={0}; config.can_handle=&hfdcan1; config.tx_id=0x321u; config.rx_id=0x500u;
    config.can_module_callback=scheduler_rx_callback;
    CANInstance *can=CANRegister(&config); TEST_CHECK(can!=NULL);
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,PLATFORM_CAN_BUS_1,can->tx_id,can));
    CanTopicMessage_t message={.topic_id=7u,.length=21u}; uint32_t topic_id=0u;
    TEST_CHECK(CanTopicTransportSend(&transport,&message,50u,&topic_id)==CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx(); uint32_t first=tx_latest_buffer[0];
    CANProcessTxBus(&hfdcan1); uint32_t second=tx_latest_buffer[0];
    TEST_CHECK(first!=second && tx_accept_count[0]==2u);
    CAN_TxResult_t result;
    TEST_CHECK(CANGetTxRequestState(topic_id,&result) && result.pending_frames==2u);
    HAL_FDCAN_TxBufferCompleteCallback(&hfdcan1,second);
    HAL_FDCAN_TxBufferCompleteCallback(&hfdcan1,first);
    TEST_CHECK(CANGetTxRequestState(topic_id,&result) && result.pending_frames==0u &&
               result.completed_frames==2u && result.state==CAN_TX_RESULT_ACTIVE);
    for (unsigned i=2u;i<5u;++i) { CANProcessTxBus(&hfdcan1); scheduler_complete_latest(&hfdcan1); }
    TEST_CHECK(CANGetTxRequestState(topic_id,&result) && result.completed_frames==5u && result.state==CAN_TX_RESULT_COMPLETE);
}

static void test_can_tx_periodic_latest_and_budget(void)
{
    const CAN_TxBusConfig_t bus_config = {
        .queue_capacity = 4u,
        .utilization_limit_permille = 250u,
        .nominal_bitrate_bps = 1000000u,
    };
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, &bus_config) == CAN_TX_SUBMIT_ACCEPTED);
    scheduler_register();

    static const uint8_t first[8] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};
    static const uint8_t latest[8] = {8u, 7u, 6u, 5u, 4u, 3u, 2u, 1u};
    static const uint8_t extended[8] = {9u, 8u, 7u, 6u, 5u, 4u, 3u, 2u};
    static int source_a;
    static int source_b;
    uint32_t first_id = 0u;
    uint32_t latest_id = 0u;
    uint32_t second_stream_id = 0xABABABABu;
    CAN_TxFrame_t frame_a = scheduler_frame(0x300u, CAN_TX_ID_STANDARD, first, 8u);
    CAN_TxFrame_t frame_b = scheduler_frame(0x1234567u, CAN_TX_ID_EXTENDED, extended, 8u);

    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_a, 1u, &frame_a,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 1000u,
                                 &first_id) == CAN_TX_SUBMIT_ACCEPTED);
    frame_a.data = latest;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_a, 1u, &frame_a,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 1000u,
                                 &latest_id) == CAN_TX_SUBMIT_COALESCED);
    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(first_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_SUPERSEDED);

    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_b, 2u, &frame_b,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 1000u,
                                 &second_stream_id) == CAN_TX_SUBMIT_BANDWIDTH);
    TEST_CHECK(second_stream_id == 0xABABABABu);
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_b, 2u, &frame_b,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 2000u,
                                 &second_stream_id) == CAN_TX_SUBMIT_ACCEPTED);

    CAN_TxSchedulerStats_t stats = {0};
    TEST_CHECK(CANGetTxSchedulerStats(&hfdcan1, &stats) == 1u);
    TEST_CHECK(stats.periodic_reserved_permille == 217u && stats.queued == 2u);
    CANProcessTx();
    TEST_CHECK(tx_id_history[0][0] == 0x300u);
    TEST_CHECK(memcmp(tx_data_history[0][0], latest, 8u) == 0);
    scheduler_complete_latest(&hfdcan1);
    CANProcessTx();
    TEST_CHECK(tx_id_history[0][1] == 0x1234567u);
    TEST_CHECK(tx_id_type_history[0][1] == FDCAN_EXTENDED_ID);
    scheduler_complete_latest(&hfdcan1);
    TEST_CHECK(CANGetTxRequestState(latest_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);
    TEST_CHECK(CANGetTxRequestState(second_stream_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);

    static const uint8_t busy_value[8] = {0xB0u, 0xB1u, 0xB2u, 0xB3u,
                                          0xB4u, 0xB5u, 0xB6u, 0xB7u};
    static const uint8_t recovered_value[8] = {0xC0u, 0xC1u, 0xC2u, 0xC3u,
                                               0xC4u, 0xC5u, 0xC6u, 0xC7u};
    uint32_t busy_id = 0u;
    tx_busy_mask = 1u;
    frame_a.data = busy_value;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_a, 1u, &frame_a,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 1000u,
                                 &busy_id) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(CANGetTxRequestState(busy_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_FAILED);
    TEST_CHECK(tx_accept_count[0] == 2u);

    tx_busy_mask = 0u;
    uint32_t recovered_id = 0u;
    frame_a.data = recovered_value;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_a, 1u, &frame_a,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 1000u,
                                 &recovered_id) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 3u);
    TEST_CHECK(memcmp(tx_data_history[0][2], recovered_value, 8u) == 0);
    scheduler_complete_latest(&hfdcan1);

    static const uint8_t rejected_value[8] = {0xD0u, 0xD1u, 0xD2u, 0xD3u,
                                              0xD4u, 0xD5u, 0xD6u, 0xD7u};
    static const uint8_t error_recovery_value[8] = {0xE0u, 0xE1u, 0xE2u, 0xE3u,
                                                    0xE4u, 0xE5u, 0xE6u, 0xE7u};
    uint32_t rejected_id = 0u;
    tx_error_mask = 1u;
    frame_a.data = rejected_value;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_a, 1u, &frame_a,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 1000u,
                                 &rejected_id) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(CANGetTxRequestState(rejected_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_FAILED && tx_accept_count[0] == 3u);

    tx_error_mask = 0u;
    uint32_t error_recovery_id = 0u;
    frame_a.data = error_recovery_value;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source_a, 1u, &frame_a,
                                 CAN_TX_PRIORITY_CONTROL, 30u, 1000u,
                                 &error_recovery_id) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 4u);
    TEST_CHECK(memcmp(tx_data_history[0][3], error_recovery_value, 8u) == 0);
    scheduler_complete_latest(&hfdcan1);
}

static void test_can_tx_expiry_cancel_and_wrap(void)
{
    const CAN_TxBusConfig_t bus_config = {
        .queue_capacity = 2u,
        .utilization_limit_permille = 500u,
        .nominal_bitrate_bps = 1000000u,
    };
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, &bus_config) == CAN_TX_SUBMIT_ACCEPTED);
    scheduler_register();
    const uint8_t data[1] = {0x42u};
    const CAN_TxFrame_t frame = scheduler_frame(0x123u, CAN_TX_ID_STANDARD, data, 1u);
    uint32_t expired_id = 0u;
    uint32_t cancelled_id = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                5u, &expired_id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &cancelled_id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANCancelTxRequest(cancelled_id) == 1u);
    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(cancelled_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_CANCELLED);
    now_us = 5000u;
    uint32_t fresh_id = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                20u, &fresh_id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANGetTxRequestState(expired_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_EXPIRED);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 1u);
    TEST_CHECK(CANGetTxRequestState(fresh_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_IN_FLIGHT);
    scheduler_complete_latest(&hfdcan1);

    now_us = (uint64_t)(UINT32_MAX - 2u) * 1000u;
    uint32_t wrap_id = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                5u, &wrap_id) == CAN_TX_SUBMIT_ACCEPTED);
    now_us = ((uint64_t)UINT32_MAX + 1u) * 1000u;
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 2u);
    TEST_CHECK(CANGetTxRequestState(wrap_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_IN_FLIGHT);
    scheduler_complete_latest(&hfdcan1);
    TEST_CHECK(CANGetTxRequestState(wrap_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);

    now_us = (uint64_t)(UINT32_MAX - 2u) * 1000u;
    uint32_t wrapped_expiry_id = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_NORMAL,
                                5u, &wrapped_expiry_id) == CAN_TX_SUBMIT_ACCEPTED);
    now_us = 2000u;
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 2u);
    TEST_CHECK(CANGetTxRequestState(wrapped_expiry_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_EXPIRED);
}

static void test_can_tx_budget_large_arithmetic(void)
{
    const CAN_TxBusConfig_t bus_config = {
        .queue_capacity = 2u,
        .utilization_limit_permille = 1u,
        .nominal_bitrate_bps = UINT32_MAX,
    };
    TEST_CHECK(CANConfigureTxBus(&hfdcan1, &bus_config) == CAN_TX_SUBMIT_ACCEPTED);
    scheduler_register();

    static const uint8_t data[8] = {0u};
    const CAN_TxFrame_t frame = scheduler_frame(0x456u, CAN_TX_ID_STANDARD, data, 8u);
    static int source;
    uint32_t request_id = 0u;
    TEST_CHECK(CANSubmitPeriodic(&hfdcan1, &source, 1u, &frame,
                                 CAN_TX_PRIORITY_CONTROL, INT32_MAX, UINT32_MAX,
                                 &request_id) == CAN_TX_SUBMIT_ACCEPTED);
    CAN_TxSchedulerStats_t stats = {0};
    TEST_CHECK(CANGetTxSchedulerStats(&hfdcan1, &stats) == 1u);
    TEST_CHECK(stats.periodic_reserved_permille == 1u);
}

static void test_can_tx_retry_boundary_and_abort(void)
{
    scheduler_register();
    const uint8_t data[2] = {0xAAu, 0x55u};
    const CAN_TxFrame_t frame = scheduler_frame(0x321u, CAN_TX_ID_STANDARD, data, 2u);
    uint32_t busy_id = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_URGENT,
                                100u, &busy_id) == CAN_TX_SUBMIT_ACCEPTED);
    tx_busy_mask = 1u;
    for (uint8_t attempt = 0u; attempt < 4u; ++attempt)
        CANProcessTx();
    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(busy_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_FAILED && result.accepted_frames == 0u);

    CAN_TxSchedulerStats_t stats = {0};
    TEST_CHECK(CANGetTxSchedulerStats(&hfdcan1, &stats) == 1u);
    TEST_CHECK(stats.fifo_busy == 4u && stats.retries == 3u && stats.failed == 1u);
    tx_busy_mask = 0u;
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 0u);

    uint32_t hal_error_id = 0u;
    tx_error_mask = 1u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_URGENT,
                                100u, &hal_error_id) == CAN_TX_SUBMIT_ACCEPTED);
    for (uint8_t attempt = 0u; attempt < 4u; ++attempt)
        CANProcessTx();
    TEST_CHECK(CANGetTxRequestState(hal_error_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_FAILED && result.accepted_frames == 0u);
    TEST_CHECK(result.last_hal_error == HAL_ERROR);
    tx_error_mask = 0u;

    uint32_t accepted_id = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &frame, CAN_TX_PRIORITY_URGENT,
                                100u, &accepted_id) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 1u);
    HAL_FDCAN_TxBufferAbortCallback(&hfdcan1, tx_latest_buffer[0]);
    TEST_CHECK(CANGetTxRequestState(accepted_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_FAILED && result.accepted_frames == 1u);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0] == 1u);

    // 多帧组中任一已接受帧中止后，未提交的后续分片不得继续发送。
    CAN_TxFrame_t group[3]={frame,frame,frame};
    uint32_t group_id=0u;
    TEST_CHECK(CANSubmitOneShotBatch(&hfdcan1,NULL,group,3u,CAN_TX_PRIORITY_NORMAL,100u,&group_id)==CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0]==2u);
    HAL_FDCAN_TxBufferAbortCallback(&hfdcan1,tx_latest_buffer[0]);
    // 结果历史回绕时，尚未清理的失败组仍持有自己的结果槽。
    for (unsigned i=0u;i<40u;++i) {
        uint32_t temporary_id=0u;
        TEST_CHECK(CANSubmitOneShot(&hfdcan1,NULL,&frame,CAN_TX_PRIORITY_NORMAL,100u,&temporary_id)==CAN_TX_SUBMIT_ACCEPTED);
        TEST_CHECK(CANCancelTxRequest(temporary_id));
    }
    TEST_CHECK(CANGetTxRequestState(group_id,&result) && result.state==CAN_TX_RESULT_FAILED);
    CANProcessTx();
    TEST_CHECK(tx_accept_count[0]==2u);
    TEST_CHECK(CANGetTxRequestState(group_id,&result) && result.state==CAN_TX_RESULT_FAILED);
}

static void test_can_tx_retry_yields_for_priority(void)
{
    scheduler_register();
    const uint8_t data = 0x5Au;
    const CAN_TxFrame_t normal = scheduler_frame(0x321u, CAN_TX_ID_STANDARD, &data, 1u);
    const CAN_TxFrame_t urgent = scheduler_frame(0x111u, CAN_TX_ID_STANDARD, &data, 1u);
    uint32_t normal_id = 0u;
    uint32_t urgent_id = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &normal, CAN_TX_PRIORITY_NORMAL,
                                100u, &normal_id) == CAN_TX_SUBMIT_ACCEPTED);

    tx_busy_mask = 1u;
    CANProcessTx();
    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(normal_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_QUEUED);

    tx_busy_mask = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, NULL, &urgent, CAN_TX_PRIORITY_URGENT,
                                100u, &urgent_id) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTx();
    TEST_CHECK(tx_id_history[0][0] == urgent.identifier);
    scheduler_complete_latest(&hfdcan1);
    CANProcessTx();
    TEST_CHECK(tx_id_history[0][1] == normal.identifier);
    scheduler_complete_latest(&hfdcan1);

    TEST_CHECK(CANGetTxRequestState(urgent_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);
    TEST_CHECK(CANGetTxRequestState(normal_id, &result) == 1u);
    TEST_CHECK(result.state == CAN_TX_RESULT_COMPLETE);
}

typedef struct {
    unsigned calls;
    PlatformCanRxFrame_t last;
    uint8_t first_bytes[4];
} PlatformCanRxTestState;

static void platform_can_test_receive(void *context, const PlatformCanRxFrame_t *frame)
{
    PlatformCanRxTestState *state = context;
    if (state->calls < sizeof(state->first_bytes))
        state->first_bytes[state->calls] = frame->data[0]; // 保存前几帧用于核对顺序。
    ++state->calls; // 记录生产接收派发实际调用的次数。
    state->last = *frame; // 复制回调期间有效的帧，便于随后检查。
}

// 接收注册和回退连接生产 CAN 实现，硬件中断由主机模型提供。
static void test_platform_can_receive(void)
{
    PlatformCanRxPort_t port = {0}, duplicate = {0};
    PlatformCanRxTestState state = {0};
    PlatformCanEndpoint_t endpoint = {.can_bus=PLATFORM_CAN_BUS_1, .tx_id=0x321u, .rx_id=0x500u};
    TEST_CHECK(PlatformCanBusIsBound(PLATFORM_CAN_BUS_1));
    TEST_CHECK(!PlatformCanBusIsBound(PLATFORM_CAN_INVALID));
    endpoint.can_bus = PLATFORM_CAN_INVALID; // 未绑定总线不能触发底层注册。
    TEST_CHECK(!PlatformCanRxRegister(&port, &endpoint, platform_can_test_receive, &state));
    TEST_CHECK(port.backend == NULL && port.callback == NULL);
    endpoint.can_bus = PLATFORM_CAN_BUS_1;
    endpoint.tx_id = 0x800u; // 非法标准帧 ID 也必须保持端点为空。
    TEST_CHECK(!PlatformCanRxRegister(&port, &endpoint, platform_can_test_receive, &state));
    TEST_CHECK(port.backend == NULL);
    endpoint.tx_id = 0x321u;
    filter_fail_once = 1u; // 过滤器失败后不得留下部分注册。
    TEST_CHECK(!PlatformCanRxRegister(&port, &endpoint, platform_can_test_receive, &state));
    TEST_CHECK(port.backend == NULL && port.callback == NULL);
    TEST_CHECK(PlatformCanRxRegister(&port, &endpoint, platform_can_test_receive, &state));
    TEST_CHECK(!PlatformCanRxRegister(&port, &endpoint, platform_can_test_receive, &state));
    TEST_CHECK(!PlatformCanRxRegister(&duplicate, &endpoint, platform_can_test_receive, &state));
    TEST_CHECK(duplicate.backend == NULL); // 同一总线与接收 ID 不能重复注册。
    TestRxFrame_t *item = &rx_frames[0][0];
    memset(item, 0, sizeof(*item));
    item->header.Identifier = endpoint.rx_id;
    item->header.DataLength = 4u << 16u;
    item->header.RxFrameType = FDCAN_DATA_FRAME;
    item->header.IdType = FDCAN_STANDARD_ID;
    item->data[0] = 0x12u;
    item->data[3] = 0x34u;
    rx_frame_count[0] = 1u;
    rx_frame_read[0] = 0u;
    now_us = 123000u; // 中断入队时间独立于后续任务派发时间。
    test_dwt_cycles = 123456u; // 周期计数也应反映入队时刻。
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    const uint32_t queued_cycles = test_dwt_cycles; // 模型读取帧时会推进计数，记录实际入队值。
    TEST_CHECK(state.calls == 0u); // 回调必须留在接收任务执行。
    now_us = 125000u;
    test_dwt_cycles = 999999u; // 改变派发前计数，验证不会覆盖入队值。
    CANProcessRx();
    TEST_CHECK(state.calls == 1u && state.last.length == 4u && state.last.received_ms == 123u);
    TEST_CHECK(state.last.received_ticks == queued_cycles);
    TEST_CHECK(state.last.data[0] == 0x12u && state.last.data[3] == 0x34u);
    PlatformCanRxTestState other_state = {0};
    PlatformCanEndpoint_t other_endpoint = endpoint;
    other_endpoint.rx_id = 0x501u;
    TEST_CHECK(PlatformCanRxRegister(&duplicate, &other_endpoint, platform_can_test_receive, &other_state));
    // 在任务派发前排入两个端点的帧，再回退第一个端点。
    rx_frames[0][1] = rx_frames[0][0];
    rx_frames[0][1].header.Identifier = other_endpoint.rx_id;
    rx_frames[0][1].data[0] = 0xa0u;
    rx_frames[0][2] = rx_frames[0][1];
    rx_frames[0][2].data[0] = 0xa1u;
    rx_frame_count[0] = 3u;
    rx_frame_read[0] = 0u;
    now_us = 126000u;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    PlatformCanRxRollback(&port); // 旧端点的排队帧必须被撤销。
    TEST_CHECK(port.backend == NULL && port.callback == NULL && port.context == NULL);
    TEST_CHECK(PlatformCanRxRegister(&port, &endpoint, platform_can_test_receive, &state));
    now_us = 127000u;
    CANProcessRx();
    TEST_CHECK(state.calls == 1u); // 旧帧不能交给新注册的同 ID 端点。
    TEST_CHECK(other_state.calls == 2u && other_state.last.received_ms == 126u);
    TEST_CHECK(other_state.first_bytes[0] == 0xa0u && other_state.first_bytes[1] == 0xa1u);
    PlatformCanRxRollback(&port);
    PlatformCanRxRollback(&duplicate);
    PlatformCanRxRollback(&port); // 重复回退保持无操作。
}

// 直接连接生产 CAN 队列，检查逻辑总线适配不会改变期限和结果。
static void test_platform_can_queue(void)
{
    CAN_Init_Config_s config = {0};
    config.can_handle = &hfdcan1; // 为测试总线启动生产 CAN 服务。
    config.tx_id = 0x321u;
    config.rx_id = 0x500u;
    config.can_module_callback = scheduler_rx_callback;
    TEST_CHECK(CANRegister(&config) != NULL);
    const uint8_t data = 0x11u;
    const CAN_TxFrame_t frame = scheduler_frame(0x321u, CAN_TX_ID_STANDARD, &data, 1u);
    uint32_t id = 99u;
    // 未绑定总线不能提交，也不能改写调用者保留的请求编号。
    TEST_CHECK(PlatformCanSubmitBatch(PLATFORM_CAN_INVALID, &data, &frame, 1u,
        CAN_TX_PRIORITY_NORMAL, 50u, &id) == CAN_TX_SUBMIT_NOT_READY && id == 99u);
    TEST_CHECK(PlatformCanProcessTxBus(PLATFORM_CAN_INVALID) == 0u);
    now_us = 115000u;
    TEST_CHECK(PlatformCanSubmitBatchUntil(PLATFORM_CAN_BUS_1, &data, &frame, 1u,
        CAN_TX_PRIORITY_NORMAL, 150u, &id) == CAN_TX_SUBMIT_ACCEPTED);
    now_us = 150000u; // 到达原始截止时刻后，队列必须过期而不能发送。
    TEST_CHECK(PlatformCanProcessTxBus(PLATFORM_CAN_BUS_1) == 1u);
    CAN_TxResult_t result;
    TEST_CHECK(PlatformCanGetRequest(id, &result) && result.state == CAN_TX_RESULT_EXPIRED);
    TEST_CHECK(tx_accept_count[0] == 0u);
    id = 0u;
    TEST_CHECK(PlatformCanSubmitBatch(PLATFORM_CAN_BUS_1, &data, &frame, 1u,
        CAN_TX_PRIORITY_NORMAL, 50u, &id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(PlatformCanCancelRequest(id) == 1u);
    TEST_CHECK(PlatformCanGetRequest(id, &result) && result.state == CAN_TX_RESULT_CANCELLED);
    id = 99u; // 一次性命令也必须拒绝未绑定总线且不改编号。
    TEST_CHECK(PlatformCanSubmitOneShot(PLATFORM_CAN_INVALID, &data, &frame,
        CAN_TX_PRIORITY_URGENT, 50u, &id) == CAN_TX_SUBMIT_NOT_READY && id == 99u);
    TEST_CHECK(PlatformCanSubmitOneShot(PLATFORM_CAN_BUS_1, &data, &frame,
        CAN_TX_PRIORITY_URGENT, 50u, &id) == CAN_TX_SUBMIT_ACCEPTED);
    tx_busy_mask = 1u; // 保持硬件繁忙，核对一次性命令沿用有限重试。
    for (unsigned attempt = 0u; attempt < 4u; ++attempt) CANProcessTx();
    TEST_CHECK(PlatformCanGetRequest(id, &result) && result.state == CAN_TX_RESULT_FAILED);
}

// 通过真实板级绑定及驱动检查三条总线和不可用状态。
static void test_platform_can_status(void)
{
    TEST_CHECK(PlatformCanGetStatus(PLATFORM_CAN_INVALID) == PLATFORM_CAN_STATUS_UNKNOWN);
    TEST_CHECK(PlatformCanGetStatus(PLATFORM_CAN_BUS_1) == PLATFORM_CAN_STATUS_UNKNOWN); // 尚未启动不查询 HAL。
    TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_1)); // 启动各总线后才查询硬件状态。
    TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_2));
    TEST_CHECK(PlatformCanStartBus(PLATFORM_CAN_BUS_3));
    status_error[1] = 1u; // 第二条总线报告错误。
    status_busoff[2] = 1u; // 第三条总线进入总线关闭。
    TEST_CHECK(PlatformCanGetStatus(PLATFORM_CAN_BUS_1) == PLATFORM_CAN_STATUS_OK);
    TEST_CHECK(PlatformCanGetStatus(PLATFORM_CAN_BUS_2) == PLATFORM_CAN_STATUS_ERROR);
    TEST_CHECK(PlatformCanGetStatus(PLATFORM_CAN_BUS_3) == PLATFORM_CAN_STATUS_BUSOFF);
    status_read_failure = 1u; // 硬件查询失败时不显示为正常。
    TEST_CHECK(PlatformCanGetStatus(PLATFORM_CAN_BUS_1) == PLATFORM_CAN_STATUS_UNKNOWN);
}

// 故障恢复前仅撤销该总线尚在软件队列的请求；另一条总线继续调度。
static void test_busoff_cancels_queued(void)
{
    TEST_CHECK(CANStartBus(&hfdcan1)); // 启动故障注入总线。
    TEST_CHECK(CANStartBus(&hfdcan2)); // 启动不受影响的总线。
    const uint8_t payload = 0x42u; // 两条总线使用相同的测试数据。
    const CAN_TxFrame_t frame = scheduler_frame(0x321u, CAN_TX_ID_STANDARD, &payload, 1u);
    uint32_t fault_id = 0u, other_id = 0u; // 分别记录两个请求。
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, &payload, &frame, CAN_TX_PRIORITY_NORMAL, 500u, &fault_id) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANSubmitOneShot(&hfdcan2, &payload, &frame, CAN_TX_PRIORITY_NORMAL, 500u, &other_id) == CAN_TX_SUBMIT_ACCEPTED);
    status_busoff[0] = 1u; // 在发送任务推进前让第一条总线发生 BusOff。
    inject_submit_on_stop = 1u; // 在 HAL 停止操作中插入一次并发提交。
    now_us = 100000u; // 到达健康检查周期，两个请求均尚未过期。
    CANHealthMonitor(); // 使用生产故障检查和软件队列撤销入口。
    TEST_CHECK(stop_submit_status == CAN_TX_SUBMIT_NOT_READY); // 恢复期间不能提交新帧。
    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(fault_id, &result) && result.state == CAN_TX_RESULT_CANCELLED);
    TEST_CHECK(CANGetTxRequestState(other_id, &result) && result.state == CAN_TX_RESULT_QUEUED);
    CANProcessTx(); // 恢复后的本轮只能提交另一条总线的请求。
    TEST_CHECK(tx_accept_count[0] == 0u && tx_accept_count[1] == 1u);
}

static void test_busoff_discards_queued_feedback(void)
{
    PlatformCanRxPort_t first = {0}, other = {0}; // 两条总线分别保存接收端点。
    ExtendedRxState first_state = {0}, other_state = {0}; // 独立记录两路派发。
    PlatformCanEndpoint_t endpoint = {.can_bus = PLATFORM_CAN_BUS_1, .tx_id = 0x321u, .rx_id = 0x123u};
    TEST_CHECK(PlatformCanRxRegister(&first, &endpoint, extended_rx_callback, &first_state));
    endpoint.can_bus = PLATFORM_CAN_BUS_2; // 相同 ID 属于不同总线。
    TEST_CHECK(PlatformCanRxRegister(&other, &endpoint, extended_rx_callback, &other_state));
    for (uint8_t bus = 0u; bus < 2u; ++bus)
    {
        rx_frames[bus][0].header.Identifier = 0x123u; // 两帧在故障前入队。
        rx_frames[bus][0].header.DataLength = 1u << 16u; // 每帧包含一个标记字节。
        rx_frames[bus][0].data[0] = (uint8_t)(0x10u + bus); // 区分两条总线的数据。
        rx_frame_count[bus] = 1u; // 模拟各控制器各收到一帧。
    }
    now_us = 99000u; // 两帧在故障前 1 ms 到达，尚未因时间过期。
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    HAL_FDCAN_RxFifo0Callback(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    status_busoff[0] = 1u; // 只有第一条总线发生 BusOff。
    now_us = 100000u;
    CANHealthMonitor(); // 清除旧代际并尝试重启。
    status_busoff[0] = 0u; // 模拟该次重启后总线恢复。
    rx_frames[0][1] = rx_frames[0][0]; // 在故障总线上排入恢复后的新帧。
    rx_frames[0][1].data[0] = 0x55u;
    rx_frame_count[0] = 2u;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    CANProcessRx(); // 故障前第一条总线的帧不能交给设备。
    TEST_CHECK(first_state.count == 1u && first_state.marker == 0x55u);
    TEST_CHECK(other_state.count == 1u && other_state.marker == 0x11u); // 其他总线的旧队列不变。
}

static void busoff_during_rx_callback(void *context, const PlatformCanRxFrame_t *frame)
{
    ExtendedRxState *state = context; // 记录故障时已经开始的设备回调。
    ++state->count; // 该回调已经取得旧帧，不能假装它未执行。
    state->marker = frame->data[0]; // 保存本次回调接收的内容。
    status_busoff[0] = 1u; // 模拟回调运行期间总线发生故障。
    now_us = 100000u; // 推进到健康检查时刻。
    CANHealthMonitor(); // 在回调尚未返回时运行生产故障处理。
}

static void test_busoff_during_rx_callback(void)
{
    PlatformCanRxPort_t first = {0}; // 故障总线上的接收端点。
    ExtendedRxState state = {0}; // 回调实际执行情况。
    PlatformCanEndpoint_t endpoint = {.can_bus = PLATFORM_CAN_BUS_1, .tx_id = 0x321u, .rx_id = 0x123u};
    TEST_CHECK(PlatformCanRxRegister(&first, &endpoint, busoff_during_rx_callback, &state));
    TEST_CHECK(CANStartBus(&hfdcan2)); // 第二条总线不应因本次故障隔离。
    rx_frames[0][0].header.Identifier = endpoint.rx_id; // 排入故障前的设备反馈。
    rx_frames[0][0].header.DataLength = 1u << 16u;
    rx_frames[0][0].data[0] = 0x51u;
    rx_frame_count[0] = 1u;
    now_us = 99000u;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    const uint32_t starts_before = bus_start_count[0]; // 回调期间不能重启该总线。
    CANProcessRx();
    TEST_CHECK(state.count == 1u && state.marker == 0x51u); // 已开始的回调可返回。
    TEST_CHECK(CANBusIsQuarantined(&hfdcan1)); // 故障处理发现回调尚未结束时隔离。
    TEST_CHECK(!CANBusIsQuarantined(&hfdcan2) && bus_start_count[0] == starts_before);
}

static void test_busoff_discards_hardware_fifo(uint8_t read_failure)
{
    PlatformCanRxPort_t first = {0}, other = {0}; // 两路端点用于检查隔离范围。
    ExtendedRxState first_state = {0}, other_state = {0}; // 记录最终派发次数。
    PlatformCanEndpoint_t endpoint = {.can_bus = PLATFORM_CAN_BUS_1, .tx_id = 0x321u, .rx_id = 0x123u};
    TEST_CHECK(PlatformCanRxRegister(&first, &endpoint, extended_rx_callback, &first_state));
    endpoint.can_bus = PLATFORM_CAN_BUS_2;
    TEST_CHECK(PlatformCanRxRegister(&other, &endpoint, extended_rx_callback, &other_state));
    rx_frames[0][0].header.Identifier = 0x123u; // 故障前的帧仍留在 FIFO0。
    rx_frames[0][0].header.DataLength = 1u << 16u;
    rx_frames[0][0].data[0] = 0x11u;
    rx_frame_count[0] = 1u;
    rx_fifo1_frames[0][0] = rx_frames[0][0]; // FIFO1 也留有旧帧。
    rx_fifo1_count[0] = 1u;
    rx_frames[1][0] = rx_frames[0][0]; // 另一条总线的帧不能被清除。
    rx_frame_count[1] = 1u;
    rx_read_failures = (uint8_t)(read_failure == 1u); // 模拟 FIFO0 读取失败。
    rx_fifo1_failure = (uint8_t)(read_failure == 2u); // 另一个场景模拟 FIFO1 读取失败。
    status_busoff[0] = 1u;
    now_us = 100000u;
    CANHealthMonitor(); // 只清理暂停总线的硬件 FIFO。
    if (read_failure != 0u)
    {
        TEST_CHECK(CANBusIsQuarantined(&hfdcan1)); // 未确认清空时禁止重启。
        TEST_CHECK(rx_frame_read[0] == (uint8_t)(read_failure == 2u)); // FIFO1 失败前允许 FIFO0 已清空。
        TEST_CHECK(rx_fifo1_read[0] == 0u); // 失败不能伪装成 FIFO1 已清空。
    }
    else
    {
        TEST_CHECK(!CANBusIsQuarantined(&hfdcan1));
        TEST_CHECK(rx_frame_read[0] == 1u && rx_fifo1_read[0] == 1u); // 两个 FIFO 均已清空。
        HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        HAL_FDCAN_RxFifo1Callback(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    }
    HAL_FDCAN_RxFifo0Callback(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); // 其他总线仍正常读取。
    CANProcessRx();
    TEST_CHECK(first_state.count == 0u && other_state.count == 1u);
}

static void test_busoff_recovery_failure(uint8_t stop_failure)
{
    TEST_CHECK(CANStartBus(&hfdcan1)); // 在启用故障注入前启动总线。
    TEST_CHECK(CANStartBus(&hfdcan2)); // 第二条总线检查隔离范围。
    fail_recovery_stop = stop_failure; // 按场景选择停止失败。
    fail_recovery_start = (uint8_t)!stop_failure; // 另一场景选择重新启动失败。
    status_busoff[0] = 1u; // 第一条总线进入 BusOff。
    now_us = 100000u; // 推进到健康检查时刻。
    CANHealthMonitor(); // 恢复失败应隔离本总线。
    TEST_CHECK(CANBusIsQuarantined(&hfdcan1));
    TEST_CHECK(!CANBusIsQuarantined(&hfdcan2));
    TEST_CHECK(!CANStartBus(&hfdcan1)); // 不能靠重新注册绕过隔离。
}

static void test_busoff_without_init(void)
{
    TEST_CHECK(CANStartBus(&hfdcan1)); // 正常启动后再注入不一致状态。
    suppress_busoff_init = 1u; // 不模拟硬件进入 INIT。
    status_busoff[0] = 1u;
    now_us = 100000u;
    CANHealthMonitor();
    TEST_CHECK(CANBusIsQuarantined(&hfdcan1)); // 无法证明接收已停止时不能清空后重启。
}

static void test_busoff_fifo_stuck(void)
{
    TEST_CHECK(CANStartBus(&hfdcan1)); // 准备故障总线。
    rx_fifo_stuck = 1u; // 读取后占用仍不下降。
    status_busoff[0] = 1u; // 驱动进入 FIFO 清理路径。
    now_us = 100000u;
    CANHealthMonitor();
    TEST_CHECK(rx_read_calls == 64u); // 读操作必须在固定次数后停止。
    TEST_CHECK(CANBusIsQuarantined(&hfdcan1)); // 未能清空时保持隔离。
}

static void test_busoff_inflight_quarantine(uint8_t unknown_slot)
{
    TEST_CHECK(CANStartBus(&hfdcan1)); // 启动待注入故障的总线。
    TEST_CHECK(CANStartBus(&hfdcan2)); // 第二条总线用于检查隔离范围。
    const uint8_t payload = 0x34u; // 使用固定负载检查已提交帧。
    const CAN_TxFrame_t frame = scheduler_frame(0x321u, CAN_TX_ID_STANDARD, &payload, 1u);
    uint32_t request = 0u; // 保存硬件已接受的请求编号。
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, &payload, &frame, CAN_TX_PRIORITY_NORMAL, 500u, &request) == CAN_TX_SUBMIT_ACCEPTED);
    tx_latest_unknown = unknown_slot; // 两个场景分别覆盖已知和无法关联的硬件槽。
    CANProcessTxBus(&hfdcan1); // 模拟 HAL 已接受但尚未报告完成。
    TEST_CHECK(tx_accept_count[0] == 1u);
    if (unknown_slot != 0u)
    {
        TEST_CHECK(CANBusIsQuarantined(&hfdcan1)); // 槽位不明不能等到后续 BusOff 才隔离。
        TEST_CHECK(!CANBusIsQuarantined(&hfdcan2)); // 不应影响其他总线。
        uint32_t rejected = 0u;
        TEST_CHECK(CANSubmitOneShot(&hfdcan1, &payload, &frame, CAN_TX_PRIORITY_NORMAL, 500u, &rejected) == CAN_TX_SUBMIT_NOT_READY);
        TEST_CHECK(rejected == 0u); // 未接受的提交不能分配编号。
    }
    const uint32_t starts_before = bus_start_count[0]; // 对比故障后是否再次启动。
    status_busoff[0] = 1u; // 发送结果尚不明确时发生 BusOff。
    now_us = 100000u;
    CANHealthMonitor(); // 不能把硬件仍持有的旧帧当作已取消。
    TEST_CHECK(CANBusIsQuarantined(&hfdcan1));
    TEST_CHECK(!CANBusIsQuarantined(&hfdcan2));
    TEST_CHECK(bus_start_count[0] == starts_before); // 故障总线不能自动重启。
    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(request, &result));
    TEST_CHECK(result.state == (unknown_slot != 0u ? CAN_TX_RESULT_FAILED : CAN_TX_RESULT_CANCELLED));
    if (unknown_slot == 0u)
        TEST_CHECK(result.pending_frames == 1u); // 状态取消不等于硬件确认帧已撤销。
    TEST_CHECK(!CANStartBus(&hfdcan1)); // 重复启动不能绕过隔离。
}

static void test_tx_slot_collision_quarantine(void)
{
    TEST_CHECK(CANStartBus(&hfdcan1)); // 模拟两个发送请求共用一个硬件槽。
    TEST_CHECK(CANStartBus(&hfdcan2)); // 保留另一条独立总线。
    const uint8_t payload = 0x34u;
    const CAN_TxFrame_t frame = scheduler_frame(0x321u, CAN_TX_ID_STANDARD, &payload, 1u);
    uint32_t first = 0u, second = 0u, waiting = 0u;
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, &payload, &frame, CAN_TX_PRIORITY_NORMAL, 500u, &first) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTxBus(&hfdcan1); // 第一帧已接受，模拟尚未完成。
    tx_next_buffer_index[0] = 0u; // 让 HAL 错误地把下一帧也关联到同一个槽。
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, &payload, &frame, CAN_TX_PRIORITY_NORMAL, 500u, &second) == CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(CANSubmitOneShot(&hfdcan1, &payload, &frame, CAN_TX_PRIORITY_BACKGROUND, 500u, &waiting) == CAN_TX_SUBMIT_ACCEPTED);
    CANProcessTxBus(&hfdcan1); // 发现槽位冲突后立即隔离。
    TEST_CHECK(CANBusIsQuarantined(&hfdcan1) && !CANBusIsQuarantined(&hfdcan2));
    CAN_TxResult_t result = {0};
    TEST_CHECK(CANGetTxRequestState(second, &result) && result.state == CAN_TX_RESULT_FAILED);
    TEST_CHECK(CANGetTxRequestState(waiting, &result) && result.state == CAN_TX_RESULT_CANCELLED);
    TEST_CHECK(CANGetTxRequestState(first, &result) && result.state == CAN_TX_RESULT_CANCELLED && result.pending_frames == 1u);
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "interrupt-before-init") == 0)
    {
        HAL_FDCAN_TxFifoEmptyCallback(&hfdcan1); // 工作区尚未准备时模拟发送中断。
        HAL_FDCAN_TxBufferCompleteCallback(&hfdcan1, 1u); // 完成中断不能隐式初始化工作区。
        HAL_FDCAN_TxBufferAbortCallback(&hfdcan1, 1u); // 中止中断也不能操作未发布状态。
        HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); // 模拟早期 FIFO0 中断。
        HAL_FDCAN_RxFifo1Callback(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE); // 模拟早期 FIFO1 中断。
        TEST_CHECK(rx_read_calls == 0u); // 早期中断不能读取尚未初始化的队列。
        CANInitWorkspace(); // 正常启动仍可初始化工作区。
        test_registration_irq_state(); // 随后注册和接收仍能工作。
        return 0; // 此场景不执行第二次工作区初始化。
    }
    CANInitWorkspace();
    if (strcmp(argv[1], "registration-irq") == 0)
        test_registration_irq_state();
    else if (strcmp(argv[1], "lk-logical-receive") == 0)
        test_lk_logical_receive();
    else if (strcmp(argv[1], "bus-isolation") == 0)
        test_bus_start_and_rollback_isolation();
    else if (strcmp(argv[1], "filter-capacity") == 0)
        test_platform_filter_capacity();
    else if (strcmp(argv[1], "rx-extended") == 0)
        test_classic_extended_rx_and_filter_reuse();
    else if (strcmp(argv[1], "filter-restore-failure") == 0)
        test_filter_restore_failure_isolates_one_bus();
    else if (strcmp(argv[1], "motor-id-type") == 0)
        test_standard_motor_rejects_extended_id();
    else if (strcmp(argv[1], "rx-invalid") == 0)
        test_rx_failure_and_invalid_frames();
    else if (strcmp(argv[1], "platform-can-status") == 0)
        test_platform_can_status();
    else if (strcmp(argv[1], "busoff-cancel-queued") == 0)
        test_busoff_cancels_queued();
    else if (strcmp(argv[1], "busoff-rx-generation") == 0)
        test_busoff_discards_queued_feedback();
    else if (strcmp(argv[1], "busoff-rx-active") == 0)
        test_busoff_during_rx_callback();
    else if (strcmp(argv[1], "busoff-rx-hardware") == 0)
        test_busoff_discards_hardware_fifo(0u);
    else if (strcmp(argv[1], "busoff-rx-read-fails") == 0)
        test_busoff_discards_hardware_fifo(1u);
    else if (strcmp(argv[1], "busoff-rx-fifo1-fails") == 0)
        test_busoff_discards_hardware_fifo(2u);
    else if (strcmp(argv[1], "busoff-stop-fails") == 0)
        test_busoff_recovery_failure(1u);
    else if (strcmp(argv[1], "busoff-start-fails") == 0)
        test_busoff_recovery_failure(0u);
    else if (strcmp(argv[1], "busoff-without-init") == 0)
        test_busoff_without_init();
    else if (strcmp(argv[1], "busoff-fifo-stuck") == 0)
        test_busoff_fifo_stuck();
    else if (strcmp(argv[1], "busoff-inflight") == 0)
        test_busoff_inflight_quarantine(0u);
    else if (strcmp(argv[1], "busoff-unknown-slot") == 0)
        test_busoff_inflight_quarantine(1u);
    else if (strcmp(argv[1], "tx-slot-collision") == 0)
        test_tx_slot_collision_quarantine();
    else if (strcmp(argv[1], "platform-can-rx") == 0)
        test_platform_can_receive();
    else if (strcmp(argv[1], "dji-rx-rollback") == 0)
        test_dji_receive_registration_rollback();
    else if (strcmp(argv[1], "module-topic-aux") == 0)
        test_module_driver_topic_failure(0u);
    else if (strcmp(argv[1], "module-topic-core") == 0)
        test_module_driver_topic_failure(1u);
    else if (strcmp(argv[1], "dji-startup-rollback") == 0)
        test_dji_startup_rollback();
    else if (strcmp(argv[1], "dji-startup-rollback-late") == 0)
        test_dji_startup_rollback_after_daemon();
    else if (strcmp(argv[1], "dji-startup-rollback-group") == 0)
        test_dji_startup_rollback_keeps_group();
    else if (strcmp(argv[1], "supercap-startup-rollback") == 0)
        test_supercap_startup_rollback();
    else if (strcmp(argv[1], "device-filter-close-failure") == 0)
        test_device_filter_close_failure();
    else if (strcmp(argv[1], "selftest-global-full") == 0)
        test_selftest_capacity_skip(1u);
    else if (strcmp(argv[1], "selftest-filter-full") == 0)
        test_selftest_capacity_skip(2u);
    else if (strcmp(argv[1], "selftest-id-conflict") == 0)
        test_selftest_capacity_skip(3u);
    else if (strcmp(argv[1], "selftest-stopped-clock") == 0)
        test_selftest_stopped_clock();
    else if (strcmp(argv[1], "selftest-loopback-complete") == 0)
        test_selftest_loopback_complete();
    else if (strcmp(argv[1], "selftest-stopped-clock-queued") == 0)
        test_selftest_stopped_clock_queued();
    else if (strcmp(argv[1], "dji-send-groups") == 0)
        test_dji_logical_send_groups();
    else if (strcmp(argv[1], "platform-can") == 0)
        test_platform_can_queue();
    else if (strcmp(argv[1], "rollback") == 0)
        test_production_registration_rollback();
    else if (strcmp(argv[1], "routing") == 0)
        test_production_can_routing();
    else if (strcmp(argv[1], "feedforward") == 0)
        test_feedforward_paths_and_recovery();
    else if (strcmp(argv[1], "feedback_failure") == 0)
        test_feedback_failure_recovery();
    else if (strcmp(argv[1], "direction-angle") == 0)
        test_direction_flags(0u);
    else if (strcmp(argv[1], "direction-speed") == 0)
        test_direction_flags(1u);
    else if (strcmp(argv[1], "direction-current") == 0)
        test_current_loop_ignores_axis_direction_flags();
    else if (strcmp(argv[1], "direction-open") == 0)
        test_open_loop_keeps_motor_direction();
    else if (strcmp(argv[1], "direction-invalid") == 0)
        test_invalid_direction_flags_rejected();
    else if (strcmp(argv[1], "rx-burst") == 0)
        test_production_can_receive_burst();
    else if (strcmp(argv[1], "dji-rx-burst") == 0)
        test_production_dji_receive_burst();
    else if (strcmp(argv[1], "tx-arguments-copy") == 0)
        test_can_tx_arguments_and_copy();
    else if (strcmp(argv[1], "tx-queue-batch-buses") == 0)
        test_can_tx_queue_batch_and_buses();
    else if (strcmp(argv[1], "tx-topic-group") == 0)
        test_can_tx_topic_group();
    else if (strcmp(argv[1], "tx-topic-interrupt") == 0)
        test_can_tx_topic_interrupt();
    else if (strcmp(argv[1], "tx-topic-rx") == 0)
        test_can_tx_topic_rx();
    else if (strcmp(argv[1], "tx-topic-cancel") == 0)
        test_can_tx_topic_cancel();
    else if (strcmp(argv[1], "tx-topic-contention") == 0)
        test_can_tx_topic_contention();
    else if (strcmp(argv[1], "tx-topic-drain") == 0)
        test_can_tx_topic_drain();
    else if (strcmp(argv[1], "tx-topic-sync-budget") == 0)
        test_can_tx_topic_sync_budget();
    else if (strcmp(argv[1], "tx-topic-absolute-deadline") == 0)
        test_can_tx_topic_absolute_deadline();
    else if (strcmp(argv[1], "tx-topic-shared") == 0)
        test_can_tx_topic_shared();
    else if (strcmp(argv[1], "tx-topic-pending") == 0)
        test_can_tx_topic_pending();
    else if (strcmp(argv[1], "tx-periodic-budget") == 0)
        test_can_tx_periodic_latest_and_budget();
    else if (strcmp(argv[1], "tx-periodic-guarded-revoke") == 0)
        test_can_tx_periodic_guard_after_rejected_replacement();
    else if (strcmp(argv[1], "tx-expiry-cancel-wrap") == 0)
        test_can_tx_expiry_cancel_and_wrap();
    else if (strcmp(argv[1], "tx-budget-large-arithmetic") == 0)
        test_can_tx_budget_large_arithmetic();
    else if (strcmp(argv[1], "tx-retry-abort") == 0)
        test_can_tx_retry_boundary_and_abort();
    else if (strcmp(argv[1], "tx-retry-priority") == 0)
        test_can_tx_retry_yields_for_priority();
    else
    {
        if (strcmp(argv[1], "closed_loops") == 0)
            test_closed_loop_feedback_selection();
        else
        {
            TEST_CHECK(strcmp(argv[1], "feedforward_overflow") == 0);
            test_feedforward_overflow();
        }
    }
    return EXIT_SUCCESS;
}
