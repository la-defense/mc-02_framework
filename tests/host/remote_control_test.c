/** @file remote_control_test.c
 * @brief 用真实UART、DBUS和安全逻辑检查长度、字段、来源时间与键鼠租约。
 * @note HAL与毫秒时钟由测试提供，无硬件操作，检查在Release仍生效。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef MC02_REMOTE_THREAD_TEST
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#endif
#include "remote_control.h"
#include "daemon.h"
#include "robot_safety.h"
#include "can_recovery.h"
#include "bsp_usart.h"
#include "remote_safety.h"
#if defined(MC02_STD_CMD_ADAPTER_TEST)
#include "std_cmd_remote.h"
#endif
#include "../../bsp/board/board_uart_stm32.h"
#if MC02_BENCH_RC_DIAGNOSTICS
#include "remote_bench.h"
#endif
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); exit(1); } } while (0)
UART_HandleTypeDef huart5, huart9;
static DMA_HandleTypeDef huart5_dma, huart9_dma;
static uint32_t now;
static unsigned tick_reads_before_delay; // 测试指定在第几次取时后插入一次调度延迟。
static uint32_t tick_after_delay; // 延迟结束后的本机时刻。
static uint8_t *dma_buffer;
#if defined(MC02_REMOTE_CAPTURE_RESET_ESTOP)
uint8_t remote_test_reset_estop_latched;
uint32_t remote_test_reset_estop_writes;
#endif
static RemoteControlDiagnostics_s remote_diag_snapshot;
#define DIAG(field) (RemoteControlGetDiagnostics(&remote_diag_snapshot), remote_diag_snapshot.field)
static RemoteControlSnapshot_s remote_input_snapshot;
#define RC_INPUT (RemoteControlGetSnapshot(&remote_input_snapshot), remote_input_snapshot.input)
extern volatile uint32_t usart_rx_oversize_total;
uint32_t HAL_GetTick(void)
{
    const uint32_t snapshot = now;
    if (tick_reads_before_delay != 0u && --tick_reads_before_delay == 0u)
        now = tick_after_delay; // 本次返回旧时刻，下一次取时模拟任务恢复执行。
    return snapshot;
}

static void daemon_cycle(void)
{
    uint8_t processed = 0u;
    DaemonTask();
    (void)DaemonProcessTaskEvents(DAEMON_CONSUMER_ROBOT_TASK,
                                  DAEMON_EVENT_MAX_DEPTH, &processed);
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ (void)n; if (h == &huart5) dma_buffer = p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *p, uint16_t n, uint32_t t)
{ (void)h; (void)p; (void)n; (void)t; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ return HAL_UART_Transmit(h,p,n,0); }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ return HAL_UART_Transmit(h,p,n,0); }
static void encode(uint8_t b[18], const uint16_t ch[5], uint8_t left, uint8_t right)
{
    memset(b, 0, 18);
    b[0] = ch[0]; b[1] = (ch[0] >> 8) | (ch[1] << 3);
    b[2] = (ch[1] >> 5) | (ch[2] << 6); b[3] = ch[2] >> 2;
    b[4] = (ch[2] >> 10) | (ch[3] << 1);
    b[5] = (ch[3] >> 7) | (right << 4) | (left << 6);
    b[16] = ch[4]; b[17] = ch[4] >> 8;
}
static void receive(const uint8_t b[18], uint16_t size)
{
    memcpy(dma_buffer, b, 18);
    HAL_UARTEx_RxEventCallback(&huart5, size);
    USARTProcessRx();
}
static void pc_frame(uint16_t heartbeat, uint16_t keys)
{
    uint8_t b[18]; const uint16_t ch[5]={1024,1024,1024,1024,1024};
    encode(b,ch,1,3); b[10]=(uint8_t)heartbeat; b[11]=(uint8_t)(heartbeat>>8);
    b[14]=(uint8_t)keys; b[15]=(uint8_t)(keys>>8); receive(b,18);
}
static void set_heartbeat(uint8_t b[18], uint16_t heartbeat)
{ b[10]=(uint8_t)heartbeat; b[11]=(uint8_t)(heartbeat>>8); }

static int CheckManualPermitRevocation(void)
{
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    const RobotControlPolicy_t policy = {
        .enabled_functions = ROBOT_CONTROL_CHASSIS, // 使用受限调试的底盘许可。
        .configured = 1u, // 采用分功能许可。
        .commissioning = 1u, // 只接受人工来源。
    };
    CHECK(RobotSafetyConfigureControl(&policy));
    uint8_t frame[18];
    const uint16_t center[5] = {1024u, 1024u, 1024u, 1024u, 1024u};
    encode(frame, center, RC_SW_DOWN, RC_SW_MID); // 保持在遥控摇杆档。
    now = 1u; receive(frame, sizeof(frame)); // 先建立有效无线来源。
    RobotSafetyRequestEnable();
    RobotSafetyUpdate(); // 等待当前核心条件建立。
    now = 2u; receive(frame, sizeof(frame));
    RobotSafetyControlInputs_t inputs = {
        .mode = ROBOT_CONTROL_MANUAL, // 由测试代替命令任务提交人工来源。
        .manual = {2u, REMOTE_RADIO_LEASE_MS, 1u}, // 来源保留接收时间。
        .feedback = {{2u, REMOTE_RADIO_LEASE_MS, 1u}}, // 四轮反馈视为在本场景内有效。
        .feedback_valid_functions = ROBOT_CONTROL_CHASSIS, // 只提供底盘反馈。
    };
    CHECK(RobotSafetyPublishControlInputs(&inputs));
    RobotSafetyUpdate();
    RobotSafetySnapshot_t before;
    CHECK(RobotSafetyGetSnapshot(&before) && RobotSafetyIsReady());
    now = 3u; receive(frame, sizeof(frame)); // 同档新帧和后台零心跳不会打断摇杆控制。
    CHECK(RobotSafetyIsReadyForGeneration(now, before.control_generation));
    encode(frame, center, RC_SW_MID, RC_SW_MID);
    now = 4u; receive(frame, sizeof(frame)); // 离开摇杆档，命令任务尚未轮询。
    CHECK(!RobotSafetyIsReadyForGeneration(now, before.control_generation));
    encode(frame, center, RC_SW_DOWN, RC_SW_MID);
    now = 5u; receive(frame, sizeof(frame)); // 在命令任务运行前重新切回摇杆档。
    CHECK(!RobotSafetyIsReadyForGeneration(now, before.control_generation));
    CHECK(RobotSafetyPublishControlInputs(&inputs)); // 模拟被抢占的命令任务迟到提交旧来源。
    CHECK(!RobotSafetyIsReadyAt(now)); // 此处旧来源尚未超过自己的期限。
    RobotSafetyUpdate();
    now = 6u; receive(frame, sizeof(frame));
    inputs.manual.source_time_ms = now;
    CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate();
    CHECK(RobotSafetyIsReady()); // 新无线输入可恢复，原使能请求仍保留。

    now = 7u; pc_frame(1u, 0u); // 切到键鼠档并开始全松开验证。
    RobotSafetyUpdate();
    now = 8u; pc_frame(2u, 0u); // 第二帧授权键鼠。
    inputs.manual = (RobotControlLease_t){now, REMOTE_PC_LEASE_MS, 1u};
    CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate();
    CHECK(RobotSafetyGetSnapshot(&before) && RobotSafetyIsReady());
    now = 9u; pc_frame(3u, 0u); // 连续合法键鼠心跳不撤销已发布目标。
    CHECK(RobotSafetyIsReadyForGeneration(now, before.control_generation));
    now = 10u; pc_frame(0u, 0u); // 键鼠端撤销，左开关仍在同一档。
    CHECK(!RobotSafetyIsReadyForGeneration(now, before.control_generation));
    now = 11u; pc_frame(1u, 0u); now = 12u; pc_frame(2u, 0u); // 命令轮询前重新握手。
    CHECK(!RobotSafetyIsReadyForGeneration(now, before.control_generation));
    RobotSafetyUpdate();
    now = 13u; pc_frame(3u, 0u);
    inputs.manual.source_time_ms = now;
    CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate();
    CHECK(RobotSafetyIsReady());
    return 0;
}

#ifdef MC02_REMOTE_THREAD_TEST
static atomic_int remote_thread_start;
static atomic_int remote_thread_done;
static atomic_int remote_thread_failed;
static atomic_int daemon_thread_done;

static void *PublishRemoteFrames(void *unused)
{
    (void)unused;
    while (atomic_load(&remote_thread_start) == 0)
        atomic_signal_fence(memory_order_acquire);
    for (uint32_t sequence = 0u; sequence < 50000u; ++sequence)
    {
        const int16_t offset = (int16_t)((int32_t)(sequence % 301u) - 150);
        const uint16_t channels[5] = {
            (uint16_t)(1024 + offset), (uint16_t)(1024 - offset),
            1024u, 1024u, 1024u
        };
        uint8_t frame[18];
        encode(frame, channels, 3u, 3u);
        set_heartbeat(frame, (uint16_t)(sequence % 65534u + 1u));
        receive(frame, sizeof(frame));
    }
    atomic_store(&remote_thread_done, 1);
    return NULL;
}

static void *ReadRemoteSnapshots(void *unused)
{
    (void)unused;
    atomic_store(&remote_thread_start, 1);
    uint32_t previous_count = 0u;
    do
    {
        RemoteControlSnapshot_s input = {0};
        RemoteControlDiagnostics_s diagnostics = {0};
        if (RemoteControlGetDiagnostics(&diagnostics) == 0u)
            atomic_store(&remote_thread_failed, 1);
        RemoteControlGetSnapshot(&input);
        if ((int32_t)input.input.rc.rocker_r_ + input.input.rc.rocker_r1 != 0 ||
            diagnostics.accepted_frames < previous_count ||
            (diagnostics.online != 0u && diagnostics.has_valid_frame == 0u))
            atomic_store(&remote_thread_failed, 1);
        previous_count = diagnostics.accepted_frames;
    } while (atomic_load(&remote_thread_done) == 0);
    return NULL;
}

static void *AdvanceDaemon(void *unused)
{
    (void)unused;
    for (uint32_t iteration = 0u; iteration < 5000u; ++iteration)
    {
        daemon_cycle();
        if (iteration > 1000u && atomic_load(&remote_thread_done) != 0)
            break;
        (void)sched_yield();
    }
    atomic_store(&daemon_thread_done, 1);
    return NULL;
}

static int ConcurrentRemoteSnapshots(void)
{
    pthread_t producer;
    pthread_t reader;
    pthread_t daemon;
    CHECK(pthread_create(&producer, NULL, PublishRemoteFrames, NULL) == 0);
    CHECK(pthread_create(&reader, NULL, ReadRemoteSnapshots, NULL) == 0);
    CHECK(pthread_create(&daemon, NULL, AdvanceDaemon, NULL) == 0);
    CHECK(pthread_join(producer, NULL) == 0);
    CHECK(pthread_join(reader, NULL) == 0);
    CHECK(pthread_join(daemon, NULL) == 0);
    CHECK(atomic_load(&daemon_thread_done) != 0);
    CHECK(atomic_load(&remote_thread_failed) == 0);
    CHECK(DIAG(accepted_frames) == 50000u);
    return 0;
}
#endif

int main(int argc, char **argv)
{
    huart5.hdmarx = &huart5_dma;
    huart9.hdmarx = &huart9_dma;
    CHECK(BoardUartResolve(PLATFORM_UART_1) == &huart1); // 逻辑 1 对应 USART1。
    CHECK(BoardUartResolve(PLATFORM_UART_2) == &huart2); // 逻辑 2 对应 USART2。
    CHECK(BoardUartResolve(PLATFORM_UART_3) == &huart3); // 逻辑 3 对应 USART3。
    CHECK(BoardUartResolve(PLATFORM_UART_5) == &huart5); // 板级映射保留当前遥控接线。
    CHECK(BoardUartResolve(PLATFORM_UART_7) == &huart7); // 逻辑 7 对应 UART7。
    CHECK(BoardUartResolve(PLATFORM_UART_8) == &huart8); // 逻辑 8 对应 UART8。
    CHECK(BoardUartResolve(PLATFORM_UART_9) == &huart9); // 上位机串口仍映射到 UART9。
    CHECK(BoardUartResolve(PLATFORM_UART_10) == &huart10); // 逻辑 10 对应 USART10。
    CHECK(BoardUartResolve(PLATFORM_UART_INVALID) == NULL); // 未声明编号不能回退到现有串口。
    CHECK(BoardUartResolve((PlatformUartPort_t)4) == NULL); // 没有声明 UART4，不能映射到其他外设。
    CHECK(RemoteControlInit(PLATFORM_UART_INVALID) == NULL && dma_buffer == NULL); // 失败不启动 DMA。
    if (argc > 1 && (!strcmp(argv[1], "source-time-wrap") ||
                     !strcmp(argv[1], "pc-age-wrap")))
        now = UINT32_MAX - 50u;
    RobotSafetyInit();
#if MC02_BENCH_RC_DIAGNOSTICS
    RemoteBenchInit(PLATFORM_UART_5);
    RemoteBenchTask(); CHECK(!RemoteControlIsOnline() && EstopIsLatched());
    RobotSafetySetImuValid(1); RobotSafetySetCalibValid(1);
    uint8_t frame[18]; const uint16_t center[5]={1024,1024,1024,1024,1024};
    encode(frame,center,3,3);
    for(now=0;now<=1000;now+=10) { receive(frame,18); RemoteBenchTask(); daemon_cycle(); }
    CHECK(RemoteControlIsOnline() && !EstopIsLatched() && RobotSafetyGetState()==ROBOT_SAFE);
    encode(frame,center,2,2);
    for(now=1010;now<=2020;now+=10) {
        RobotSafetyRequestEnable(); receive(frame,18); RemoteBenchTask(); daemon_cycle();
        CHECK(!RobotSafetyIsReady());
    }
    const Robot_Fault_e faults[]={ROBOT_FAULT_MOTOR_OFFLINE,ROBOT_FAULT_CAN_BUSOFF,ROBOT_FAULT_TILT};
    for(unsigned i=0;i<3;++i) {
        RobotSafetyRequestEnable(); RobotFaultReport(faults[i]);
        now+=10; receive(frame,18); RemoteBenchTask(); daemon_cycle();
        CHECK(RobotSafetyGetState()==ROBOT_FAULT && !RobotSafetyIsReady());
        RobotFaultClear(faults[i]); now+=10; receive(frame,18); RemoteBenchTask(); daemon_cycle();
        CHECK(RobotSafetyGetState()==ROBOT_SAFE && !RobotSafetyIsReady());
    }
    for(unsigned i=0;i<11;++i) { now+=10; daemon_cycle(); RemoteBenchTask(); }
    CHECK(!RemoteControlIsOnline() && EstopIsLatched());
    receive(frame,18); RemoteBenchTask(); CHECK(RemoteControlIsOnline() && EstopIsLatched());
    CHECK(!RobotSafetyIsReady());
    return 0;
#endif
    const RC_ctrl_t *rc = RemoteControlInit(PLATFORM_UART_5);
    CHECK(rc != NULL && dma_buffer != NULL);
    CHECK(!RemoteControlIsOnline());
    if (argc > 1 && !strcmp(argv[1], "estop-single-frame")) {
        uint8_t frame[18];
        uint16_t channels[5] = {1024u, 1024u, 1024u, 1024u, 1024u}; // 双杆和拨轮起初都在中位。
        RemoteSafetyGestures_s holds;
        RemoteSafetyGesturesInit(&holds);
        RemoteControlSnapshot_s sample;
        RobotSafetySetImuValid(1u);
        RobotSafetySetCalibValid(1u);
        encode(frame, channels, RC_SW_MID, RC_SW_MID);
        now = 1u;
        receive(frame, sizeof(frame)); // 先让手势任务开始累计两个开关居中。
        RemoteControlGetSnapshot(&sample);
        RemoteSafetyGesturesUpdate(&holds, &sample.input, sample.remote_online, now, 1u, sample.gesture_switch_generation);
        for (now = 51u; now <= 1001u; now += 50u)
            receive(frame, sizeof(frame)); // 合法帧持续到达，手势任务尚未再次运行。
        now = 1001u;
        RemoteControlGetSnapshot(&sample); // 命令任务读取后被抢占，此时中位计时已满一秒。
        channels[4] = 1325u; // 原始读数对应拨轮 +301。
        encode(frame, channels, RC_SW_MID, RC_SW_MID);
        receive(frame, 17u); // 短帧无效，不能提交拨轮动作。
        CHECK(!EstopIsLatched());
        receive(frame, sizeof(frame)); // 同一时钟刻度内收到一份合法的急停帧。
        CHECK(EstopIsLatched() && EstopGetReason() == ESTOP_REASON_MANUAL);
#if defined(MC02_REMOTE_CAPTURE_RESET_ESTOP)
        CHECK(remote_test_reset_estop_latched == 1u && remote_test_reset_estop_writes == 1u); // 接收返回前同步跨复位标志。
#endif
        channels[4] = 1024u;
        encode(frame, channels, RC_SW_MID, RC_SW_MID);
        receive(frame, sizeof(frame)); // 拨轮随下一帧恢复中位，命令任务仍未恢复。
        RemoteSafetyGesturesUpdate(&holds, &sample.input, sample.remote_online, now, 1u, sample.gesture_switch_generation);
        CHECK(EstopIsLatched()); // 被抢占前的旧居中证据不能清除新急停。
#if defined(MC02_REMOTE_CAPTURE_RESET_ESTOP)
        CHECK(remote_test_reset_estop_latched == 1u && remote_test_reset_estop_writes == 1u); // 旧的清除尝试不能重写记录。
#endif
        RemoteControlGetSnapshot(&sample);
        RemoteSafetyGesturesUpdate(&holds, &sample.input, sample.remote_online, now, 1u, sample.gesture_switch_generation);

        for (now = 1051u; now <= 1951u; now += 50u)
            receive(frame, sizeof(frame)); // 保持开关居中且无线持续在线。
        now = 2000u;
        receive(frame, sizeof(frame));
        RemoteControlGetSnapshot(&sample);
        RemoteSafetyGesturesUpdate(&holds, &sample.input, sample.remote_online, now, 1u, sample.gesture_switch_generation);
        CHECK(EstopIsLatched()); // 新的居中保持只有 999 ms。
        now = 2001u;
        receive(frame, sizeof(frame));
        RemoteControlGetSnapshot(&sample);
        RemoteSafetyGesturesUpdate(&holds, &sample.input, sample.remote_online, now, 1u, sample.gesture_switch_generation);
        CHECK(!EstopIsLatched() && RobotSafetyGetState() == ROBOT_SAFE);
#if defined(MC02_REMOTE_CAPTURE_RESET_ESTOP)
        CHECK(remote_test_reset_estop_latched == 0u && remote_test_reset_estop_writes == 2u); // 仅在有效解除时清除一次。
#endif
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "estop-dial-priority")) {
        uint8_t frame[18];
        uint16_t channels[5] = {1024u, 1024u, 1024u, 1024u, 1325u}; // 摇杆居中，拨轮原始读数对应 +301。
        RemoteSafetyGestures_s holds;
        RemoteSafetyGesturesInit(&holds);
        RobotSafetySetImuValid(1u);
        RobotSafetySetCalibValid(1u);
        RemoteControlSnapshot_s sample;
        encode(frame, channels, RC_SW_MID, RC_SW_MID); // 双开关居中也是解除急停手势。
        now = 1u;
        receive(frame, sizeof(frame));
        RemoteControlGetSnapshot(&sample);
        CHECK(sample.remote_online && sample.input.rc.dial == 301);
        RemoteSafetyGesturesUpdate(&holds, &sample.input, sample.remote_online, now, 1u, sample.gesture_switch_generation);
        RobotSafetySnapshot_t locked;
        CHECK(RobotSafetyGetSnapshot(&locked) && locked.estop_latched);
        for (now = 51u; now <= 1001u; now += 50u) {
            receive(frame, sizeof(frame)); // 保持无线输入在线，同时维持相冲突的拨轮和开关位置。
            RemoteControlGetSnapshot(&sample);
            RemoteSafetyGesturesUpdate(&holds, &sample.input, sample.remote_online, now, 1u, sample.gesture_switch_generation);
        }
        RobotSafetySnapshot_t continued;
        CHECK(RobotSafetyGetSnapshot(&continued) && continued.estop_latched);
        CHECK(continued.control_generation == locked.control_generation); // 若暗中先解除再锁回，急停会使控制代次再次变化。

        channels[4] = 1324u; // +300 刚好退出拨轮急停范围，释放拨轮本身不能解除锁存。
        encode(frame, channels, RC_SW_MID, RC_SW_MID);
        for (now = 1002u; now <= 1952u; now += 50u) {
            receive(frame, sizeof(frame)); // 保持两个开关原位，重新累计有效的居中时间。
            RemoteControlGetSnapshot(&sample);
            RemoteSafetyGesturesUpdate(&holds, &sample.input, sample.remote_online, now, 1u, sample.gesture_switch_generation);
        }
        now = 2001u;
        receive(frame, sizeof(frame));
        RemoteControlGetSnapshot(&sample);
        RemoteSafetyGesturesUpdate(&holds, &sample.input, sample.remote_online, now, 1u, sample.gesture_switch_generation);
        CHECK(EstopIsLatched()); // 释放拨轮后只过了 999 ms，急停仍须保持。
        now = 2002u;
        receive(frame, sizeof(frame));
        RemoteControlGetSnapshot(&sample);
        RemoteSafetyGesturesUpdate(&holds, &sample.input, sample.remote_online, now, 1u, sample.gesture_switch_generation);
        CHECK(!EstopIsLatched()); // 连续保持满一秒，才允许按原规则回到 SAFE。
        CHECK(RobotSafetyGetState() == ROBOT_SAFE);
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "gesture-right-edge")) {
        uint8_t frame[18];
        const uint16_t center[5] = {1024u, 1024u, 1024u, 1024u, 1024u}; // 合法无线帧，两个摇杆居中。
        RemoteSafetyGestures_s holds;
        RemoteSafetyGesturesInit(&holds);
        RobotSafetySetImuValid(1u);
        RobotSafetySetCalibValid(1u);
        EstopRequest(ESTOP_REASON_MANUAL); // 只用模拟手势验证急停锁存。

        RemoteControlSnapshot_s sample;
        encode(frame, center, RC_SW_MID, RC_SW_MID);
        now = 1u;
        receive(frame, sizeof(frame));
        RemoteControlGetSnapshot(&sample);
        const uint32_t left_generation = sample.source_selection_generation;
        const uint32_t gesture_generation = sample.gesture_switch_generation; // 首次读取对应的双开关计数。
        RemoteSafetyGesturesUpdate(&holds, &sample.input, sample.remote_online, now, 1u, gesture_generation);
        for (now = 51u; now < 951u; now += 50u) receive(frame, sizeof(frame)); // 期间无线链路持续有效。
        encode(frame, center, RC_SW_MID, RC_SW_UP);
        receive(frame, sizeof(frame)); // 右开关先移开，命令任务未运行。
        now = 1000u;
        encode(frame, center, RC_SW_MID, RC_SW_MID);
        receive(frame, sizeof(frame)); // 同一轮命令任务读取前，右开关又拨回。
        now = 1001u;
        RemoteControlGetSnapshot(&sample);
        CHECK(sample.remote_online && sample.source_selection_generation == left_generation); // 右开关不改变人工平移来源。
        CHECK(sample.gesture_switch_generation != gesture_generation); // 两次合法变化留下记录。
        RemoteSafetyGesturesUpdate(&holds, &sample.input, sample.remote_online, now, 1u, sample.gesture_switch_generation);
        CHECK(EstopIsLatched()); // 两次看到相同组合，仍不能把中间的断开算入一秒。

        for (now = 1051u; now <= 2001u; now += 50u) receive(frame, sizeof(frame)); // 新组合持续保持一秒。
        now = 2001u;
        RemoteControlGetSnapshot(&sample);
        RemoteSafetyGesturesUpdate(&holds, &sample.input, sample.remote_online, now, 1u, sample.gesture_switch_generation);
        CHECK(!EstopIsLatched());
        RobotSafetyUpdate();
        CHECK(RobotSafetyGetState() == ROBOT_SAFE); // 解除急停仍遵守原调试状态。
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "formal-autonomous-gap")) {
        RobotSafetySetImuValid(1u); RobotSafetySetCalibValid(1u);
        const RobotControlPolicy_t policy = {ROBOT_CONTROL_CHASSIS | ROBOT_CONTROL_GIMBAL, 1u, 1u, 0u};
        CHECK(RobotSafetyConfigureControl(&policy)); RobotSafetyUpdate();
        now = 1u;
        const RobotSafetyControlInputs_t inputs = {
            .mode = ROBOT_CONTROL_AUTONOMOUS, // 本场景选择正式自主。
            .match = {1u, 5000u, 1u}, // 比赛阶段覆盖整个测试。
            .feedback = {{1u, 5000u, 1u}, {1u, 5000u, 1u}, {0u, 0u, 0u}}, // 两组核心反馈有效。
            .feedback_valid_functions = ROBOT_CONTROL_CHASSIS | ROBOT_CONTROL_GIMBAL,
            .match_running = 1u,
        };
        CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate(); CHECK(RobotSafetyIsReady());
        pc_frame(1u,0u); pc_frame(2u,0u); pc_frame(3u,1u); // 已授权键鼠，模拟无线正常。
        now+=101u; pc_frame(4u,0u); // 离线事件未派发前先收到新无线帧。
        RemoteControlSnapshot_s snapshot; RemoteControlGetSnapshot(&snapshot); RobotSafetyUpdate();
        CHECK(!snapshot.pc_valid && !EstopIsLatched() && RobotSafetyIsReady()); // 正式自主不因遥控间断急停，键鼠仍要重新授权。
        CHECK(DIAG(offline_events)==1u);
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "formal-autonomous-offline")) {
        RobotSafetySetImuValid(1u); RobotSafetySetCalibValid(1u); // 真实安全接口报告已检查的核心条件。
        const RobotControlPolicy_t policy = {ROBOT_CONTROL_CHASSIS | ROBOT_CONTROL_GIMBAL, 1u, 1u, 0u};
        CHECK(RobotSafetyConfigureControl(&policy)); RobotSafetyUpdate();
        now = 1u;
        const RobotSafetyControlInputs_t inputs = {
            .mode = ROBOT_CONTROL_AUTONOMOUS, // 场景明确使用正式自主。
            .match = {1u, 5000u, 1u}, // 后续无线离线期间比赛阶段仍有效。
            .feedback = {{1u, 5000u, 1u}, {1u, 5000u, 1u}, {0u, 0u, 0u}}, // 两组测试反馈在本场景保持有效。
            .feedback_valid_functions = ROBOT_CONTROL_CHASSIS | ROBOT_CONTROL_GIMBAL, // 由测试模拟已检查的反馈。
            .match_running = 1u, // 正在比赛；没有人工连续目标或击发请求。
        };
        CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate(); CHECK(RobotSafetyIsReady());
        now = 2u; pc_frame(1u, 0u); now = 3u; pc_frame(2u, 0u); // 先建立真实无线接收记录。
        now = 104u; daemon_cycle(); // 真实来源监控和离线回调处理超过100ms的无线间隔。
        RemoteControlSnapshot_s value; RemoteControlGetSnapshot(&value);
        RobotSafetyUpdate();
        CHECK(!value.remote_online && !value.pc_valid && !EstopIsLatched());
        CHECK(RobotSafetyIsReady()); // 正式自主继续按比赛阶段与核心状态运行。
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "manual-permit-revoke"))
        return CheckManualPermitRevocation();
    if (argc > 1 && !strcmp(argv[1], "snapshot-metadata")) {
        RemoteControlSnapshot_s value;
        RemoteControlGetSnapshot(&value); // 首帧前没有可用来源或编号。
        CHECK(!value.remote_online && !value.pc_valid && value.radio_sequence == 0u && value.pc_sample_sequence == 0u);
        now = 10u; pc_frame(1u, 0u); // 第一份全松开输入只建立恢复计数。
        RemoteControlGetSnapshot(&value);
        CHECK(value.radio_sequence == 1u && value.pc_sample_sequence == 1u && !value.pc_valid);
        now = 11u; pc_frame(2u, 0u); // 第二份全松开数据使键鼠可用。
        RemoteControlGetSnapshot(&value);
        CHECK(value.pc_valid && value.radio_source_time_ms == 11u && value.pc_source_time_ms == 11u);
        CHECK(value.radio_sequence == 2u && value.pc_sample_sequence == 2u);
        now = 15u; RemoteControlGetSnapshot(&value); // 查询不能替输入增加序号或续期。
        CHECK(value.radio_source_time_ms == 11u && value.pc_source_time_ms == 11u && value.pc_sample_sequence == 2u);
        now = 20u; pc_frame(2u, 1u); // 重复键鼠心跳仍是合法无线帧，但键鼠输入不更新。
        RemoteControlGetSnapshot(&value);
        CHECK(value.radio_sequence == 3u && value.radio_source_time_ms == 20u);
        CHECK(value.pc_sample_sequence == 2u && value.pc_source_time_ms == 11u && value.input.key[0].keys == 0u);
        uint8_t frame[18]; const uint16_t ch[5] = {1024,1024,1024,1024,1024};
        encode(frame, ch, 1u, 3u); set_heartbeat(frame, 3u); frame[14] = 1u;
        now = 30u; memcpy(dma_buffer, frame, sizeof(frame));
        HAL_UARTEx_RxEventCallback(&huart5, sizeof(frame)); // 先记录接收，再推迟任务派发。
        now = 35u; USARTProcessRx();
        RemoteControlGetSnapshot(&value);
        CHECK(value.radio_sequence == 4u && value.pc_sample_sequence == 3u && value.input.key[0].w);
        CHECK(value.radio_source_time_ms == 30u && value.pc_source_time_ms == 30u);
#if defined(MC02_STD_CMD_ADAPTER_TEST)
        StdCmdRemoteMap_t mapping = {0}; // 使用只属于主机的测试比例。
        mapping.remote_map.max_age_ms = 100u; mapping.keyboard_map.max_age_ms = 150u;
        mapping.remote_map.allowed_frames = mapping.keyboard_map.allowed_frames = 1u << STD_CMD_FRAME_CHASSIS;
        for (unsigned axis = 0u; axis < STD_CMD_AXIS_COUNT; ++axis)
        {
            mapping.remote_map.axis[axis] = (StdCmdAxisMap_t){660.0f, 0.01f, 10.0f};
            mapping.keyboard_map.axis[axis] = (StdCmdAxisMap_t){1.0f, 1.0f, 1.0f};
        }
        mapping.keyboard_positive[STD_CMD_VX] = (uint16_t)(1u << Key_W); // 将真实接收的 W 映射为测试前进。
        StdCmdManualIntent_t intent = {0};
        CHECK(StdCmdAdaptRemote(&mapping, &value, now, &intent) == STD_CMD_OK);
        CHECK(intent.source == STD_CMD_KEY_MOUSE && intent.axis[STD_CMD_VX] == 1.0f &&
              intent.sequence == 3u && intent.source_time_ms == 30u); // 真实接收时间不能被任务派发时间替代。
#endif
        frame[12] = 2u; now = 40u; receive(frame, sizeof(frame)); // 非法鼠标字段不得更新任何来源信息。
        RemoteControlGetSnapshot(&value);
        CHECK(value.radio_sequence == 4u && value.pc_sample_sequence == 3u && value.radio_source_time_ms == 30u);

        encode(frame, ch, RC_SW_DOWN, RC_SW_MID); // 一帧合法切档，若过期则不能撤销当前来源。
        set_heartbeat(frame, 4u);
        now = 45u; memcpy(dma_buffer, frame, sizeof(frame));
        HAL_UARTEx_RxEventCallback(&huart5, sizeof(frame)); // 在 45 ms 保存原始接收时刻。
        const uint32_t previous_selection = value.source_selection_generation;
        const uint32_t before_drop = DIAG(rejected_length);
        now = 95u; // UART 派发与遥控解码入口均恰好位于 50 ms 边界。
        tick_reads_before_delay = 2u; // 越过 UART 检查与遥控入口检查后，模拟被抢占。
        tick_after_delay = 96u;
        USARTProcessRx();
        CHECK(tick_reads_before_delay == 0u && now == 96u);
        RemoteControlGetSnapshot(&value);
        CHECK(value.remote_online && value.pc_valid && value.input.key[0].w);
        CHECK(value.radio_sequence == 4u && value.pc_sample_sequence == 3u);
        CHECK(value.input.rc.switch_left == RC_SW_UP && value.source_selection_generation == previous_selection);
        CHECK(DIAG(rejected_length) == before_drop + 1u);

        now = 100u; memcpy(dma_buffer, frame, sizeof(frame));
        HAL_UARTEx_RxEventCallback(&huart5, sizeof(frame));
        now = 150u; USARTProcessRx(); // 提交检查也处于 50 ms，仍可正常切档。
        RemoteControlGetSnapshot(&value);
        CHECK(value.remote_online && value.radio_source_time_ms == 100u && value.radio_sequence == 5u);
        CHECK(value.input.rc.switch_left == RC_SW_DOWN && value.source_selection_generation == previous_selection + 1u);
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "source-epoch")) {
        uint8_t frame[18]; const uint16_t ch[5] = {1024,1024,1024,1024,1024};
        RemoteControlSnapshot_s before = {0}, after = {0};
        encode(frame, ch, RC_SW_DOWN, RC_SW_MID); now = 1u; receive(frame, sizeof(frame));
        RemoteControlGetSnapshot(&before);
        CHECK(before.source_selection_generation != 0u); // 首次合法来源建立选择代次。
        CHECK(RemoteControlRecoveryStillValid(&before, now)); // 当前摇杆可供后续恢复检查。
        CanRecoveryTimed_t recovery = {0}; // 用真实无线帧的来源信息驱动恢复检查。
        const CanRecoveryConfig_t config = {1u, 10u}; // 测试反馈稳定等待 10 ms。
        CHECK(CanRecoveryTimedInit(&recovery, &config));
        CanRecoveryTimedInput_t recovery_input = {
            .core = {.now_ms = now, .valid_feedback_mask = 1u, .feedback_sequence = {1u},
                     .command_sequence = before.radio_sequence, .bus_ok = 1u, .permit_ok = 1u},
            .command_source_id = 1u, .command_source_epoch = before.source_selection_generation,
            .command_source_ms = before.radio_source_time_ms, .command_max_age_ms = REMOTE_RADIO_LEASE_MS};
        CHECK(!CanRecoveryTimedStep(&recovery, &recovery_input));
        now = 2u; receive(frame, sizeof(frame)); RemoteControlGetSnapshot(&after);
        CHECK(after.source_selection_generation == before.source_selection_generation); // 同档新帧不能误报切换。
        CHECK(!RemoteControlRecoveryStillValid(&before, now)); // 已有更新帧时不能用旧输入提交最终恢复。
        recovery_input.core.now_ms = now; recovery_input.core.feedback_sequence[0] = 2u;
        recovery_input.core.command_sequence = after.radio_sequence;
        recovery_input.command_source_ms = after.radio_source_time_ms;
        CHECK(!CanRecoveryTimedStep(&recovery, &recovery_input) &&
              recovery.recovery.phase == CAN_RECOVERY_STABILIZING); // 模拟新电机反馈开始稳定等待。
        encode(frame, ch, RC_SW_UP, RC_SW_MID); now = 3u; receive(frame, sizeof(frame));
        encode(frame, ch, RC_SW_DOWN, RC_SW_MID); now = 4u; receive(frame, sizeof(frame));
        RemoteControlGetSnapshot(&after);
        CHECK(after.source_selection_generation == before.source_selection_generation + 2u); // 两次读取之间切出又切回仍会留下变化。
        recovery_input.core.now_ms = now; recovery_input.core.command_sequence = after.radio_sequence;
        recovery_input.command_source_ms = after.radio_source_time_ms;
        recovery_input.command_source_epoch = after.source_selection_generation;
        CHECK(!CanRecoveryTimedStep(&recovery, &recovery_input) &&
              recovery.recovery.phase == CAN_RECOVERY_WAIT_FEEDBACK); // 短暂切换后必须重新验证反馈。
        before = after; frame[12] = 2u; now = 5u; receive(frame, sizeof(frame)); // 非法帧不能改变选择代次。
        RemoteControlGetSnapshot(&after); CHECK(after.source_selection_generation == before.source_selection_generation);
        frame[12] = 0u; now = 105u; daemon_cycle(); // 超过最近合法帧的无线期限。
        now = 106u; receive(frame, sizeof(frame)); RemoteControlGetSnapshot(&after);
        CHECK(after.source_selection_generation != before.source_selection_generation); // 即使回到同档，断线重连也须重新建立来源。
        CHECK(RemoteControlRecoveryStillValid(&after, now));
        CHECK(!RemoteControlRecoveryStillValid(&after, now + REMOTE_RADIO_LEASE_MS + 1u)); // 不依赖后台任务，也按原始时间撤销过期输入。
        return 0;
    }
#ifdef MC02_REMOTE_THREAD_TEST
    if (argc > 1 && !strcmp(argv[1], "threads")) return ConcurrentRemoteSnapshots();
#endif
    RobotSafetyUpdate();
    CHECK(EstopIsLatched());
    CHECK(RobotSafetyGetState() == ROBOT_ESTOP);
    if (argc > 1 && (!strcmp(argv[1], "source-time-zero") ||
                     !strcmp(argv[1], "source-time-wrap"))) {
        uint8_t b[18]; const uint16_t ch[5]={1024,1024,1024,1024,1024};
        RemoteControlSnapshot_s state;
        const uint32_t start = now;
        encode(b,ch,1,3); set_heartbeat(b,1);
        memcpy(dma_buffer,b,18); HAL_UARTEx_RxEventCallback(&huart5,18);
        now+=50u; USARTProcessRx();
        CHECK(DIAG(last_valid_tick)==start);
        CHECK(DIAG(pc_last_advance_tick)==start);
        now=start+100u; RemoteControlGetSnapshot(&state); CHECK(state.remote_online);
        now=start+101u; daemon_cycle(); RemoteControlGetSnapshot(&state);
        CHECK(!state.remote_online && !state.pc_valid && !state.input.rc.switch_left);
        CHECK(DIAG(offline_tick)==now && DIAG(offline_last_valid_tick)==start);
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "pc-boundaries")) {
        RemoteControlSnapshot_s s;
        pc_frame(65534,0); pc_frame(65535,0); pc_frame(1,1);
        RemoteControlGetSnapshot(&s); CHECK(s.pc_valid && s.input.key[0].w);
        pc_frame(0x8001,0); RemoteControlGetSnapshot(&s); CHECK(s.pc_valid && s.input.key[0].w);
        uint8_t b[18]; const uint16_t ch[5]={1024,1024,1024,1024,1024};
        now=75; pc_frame(0x8001,0); // 无线仍有帧，PC 心跳不推进。
        encode(b,ch,1,3); set_heartbeat(b,2); b[12]=2; now=100; receive(b,18);
        CHECK(DIAG(pc_last_advance_tick)==0 && DIAG(raw_mouse_z)==0x8001);
        // 无线帧继续到达，但重复键鼠心跳不能延长150ms键鼠期限。
        now=150; pc_frame(0x8001,0); RemoteControlGetSnapshot(&s); CHECK(s.pc_valid);
        now=151; RemoteControlGetSnapshot(&s); CHECK(!s.pc_valid);
        b[12]=0; b[6]=1; set_heartbeat(b,3); receive(b,18);
        b[6]=0; set_heartbeat(b,4); receive(b,18); RemoteControlGetSnapshot(&s); CHECK(!s.pc_valid);
        now+=151; set_heartbeat(b,5); receive(b,18); RemoteControlGetSnapshot(&s); CHECK(!s.pc_valid);
        set_heartbeat(b,6); receive(b,18); RemoteControlGetSnapshot(&s); CHECK(s.pc_valid);
        for(unsigned i=0;i<12;++i) { now+=10; daemon_cycle(); }
        RemoteControlGetSnapshot(&s); CHECK(!s.remote_online && !s.pc_valid && EstopIsLatched());
        pc_frame(7,1); RemoteControlGetSnapshot(&s); CHECK(s.remote_online && !s.pc_valid && EstopIsLatched());
        pc_frame(8,0); pc_frame(9,0); RemoteControlGetSnapshot(&s); CHECK(s.pc_valid && EstopIsLatched());
    }
    if (argc > 1 && (!strcmp(argv[1], "pc-age-zero") ||
                     !strcmp(argv[1], "pc-age-wrap"))) {
        pc_frame(0,0); pc_frame(1,0);
        now+=33u; pc_frame(2,0);
        now+=33u; pc_frame(3,(1u<<Key_E)|(1u<<Key_F));
        const uint32_t last=now;
        RemoteControlSnapshot_s s; RemoteControlGetSnapshot(&s);
        CHECK(s.pc_valid && s.input.key_count[0][Key_E]==1);
        for(unsigned age=10;age<=150;age+=10) {
            now=last+age; pc_frame(3,(1u<<Key_E)|(1u<<Key_F));
            RemoteControlGetSnapshot(&s); CHECK(s.pc_valid && s.remote_online);
        }
        now=last+151u; RemoteControlGetSnapshot(&s);
        CHECK(!s.pc_valid && s.remote_online);
        CHECK(!s.input.key[0].keys && !s.input.key_count[0][Key_E] && !s.input.key_count[0][Key_F]);
        CHECK(DIAG(pc_expire_tick)==now && DIAG(pc_last_advance_tick)==last);
        pc_frame(4,1); RemoteControlGetSnapshot(&s); CHECK(!s.pc_valid);
        pc_frame(5,0); pc_frame(6,0); pc_frame(7,1);
        RemoteControlGetSnapshot(&s); CHECK(s.pc_valid && s.input.key[0].keys==1);
    }
    if (argc > 1 && !strcmp(argv[1], "pc-handshake")) {
        pc_frame(1,1); CHECK(RC_INPUT.key[0].keys==0);
        pc_frame(2,0); pc_frame(3,1); CHECK(RC_INPUT.key[0].keys==0);
        pc_frame(4,0); pc_frame(4,0); CHECK(RC_INPUT.key[0].keys==0);
        pc_frame(5,0); pc_frame(6,1); CHECK(RC_INPUT.key[0].keys==1);
        CHECK(RC_INPUT.key_count[0][Key_W]==1);
        pc_frame(6,0); CHECK(RC_INPUT.key[0].keys==1); /* Repeated seq cannot change input. */
        pc_frame(5,0); CHECK(RC_INPUT.key[0].keys==1);
        pc_frame(7,0); CHECK(RC_INPUT.key[0].keys==0);
        pc_frame(8,1); CHECK(RC_INPUT.key_count[0][Key_W]==2);
        pc_frame(0,1); CHECK(RC_INPUT.key[0].keys==0 && RC_INPUT.key_count[0][Key_W]==0);
    }
    if (argc > 1 && !strcmp(argv[1], "pc-startup")) {
        uint8_t b[18]; const uint16_t ch[5]={1024,1024,1024,1024,1024};
        encode(b,ch,1,3); b[14]=1; b[12]=1; receive(b,18);
        CHECK(RemoteControlIsOnline());
        CHECK(RC_INPUT.key[0].keys==0 && RC_INPUT.mouse.press_l==0);
        CHECK(RC_INPUT.key_count[KEY_PRESS][Key_W]==0);
    }
    if (argc > 1 && !strcmp(argv[1], "mouse-z")) {
        /* DBUS byte offsets 10/11; compare literal wire bit patterns. */
        uint8_t b[18]; const uint16_t ch[5]={1024,1024,1024,1024,1024};
        const uint16_t words[]={0,1,256,0x7fff,0x8000,0xffff};
        for (unsigned i=0;i<sizeof(words)/sizeof(words[0]);++i) {
            encode(b,ch,1,3); b[10]=(uint8_t)words[i]; b[11]=(uint8_t)(words[i]>>8);
            memcpy(dma_buffer,b,18); HAL_UARTEx_RxEventCallback(&huart5,18);
            memset(dma_buffer,0xee,18); USARTProcessRx();
            CHECK(DIAG(raw_mouse_z)==words[i]);
            CHECK(!RC_INPUT.key[0].keys && !RC_INPUT.mouse.x && !RC_INPUT.mouse.y);
            CHECK(RemoteControlIsOnline());
            receive(b,18); CHECK(DIAG(raw_mouse_z)==words[i]);
        }
        b[10]=0; b[11]=0; receive(b,17);
        CHECK(DIAG(raw_mouse_z)==0xffff);
        b[12]=2; receive(b,18); CHECK(DIAG(raw_mouse_z)==0xffff);
        CHECK(DIAG(accepted_frames)==12);
        now=1000; /* No new frame: diagnostics retain the last accepted bits. */
        CHECK(DIAG(raw_mouse_z)==0xffff);
    }
    if (argc > 1 && !strcmp(argv[1], "keyboard-map")) {
        /* Official 5.0.2.37 sender and DT7 hardware: A=0x0004, D=0x0008. */
        uint8_t b[18]; const uint16_t ch[5]={1024,1024,1024,1024,1024};
        pc_frame(1,0); pc_frame(2,0);
        encode(b,ch,1,3); set_heartbeat(b,3); b[14]=0x04; receive(b,18);
        CHECK(RC_INPUT.key[KEY_PRESS].a && !RC_INPUT.key[KEY_PRESS].d);
        CHECK(RC_INPUT.key_count[KEY_PRESS][Key_A]==1);
        b[14]=0; set_heartbeat(b,4); receive(b,18); b[14]=0x08; set_heartbeat(b,5); receive(b,18);
        CHECK(RC_INPUT.key[KEY_PRESS].d && !RC_INPUT.key[KEY_PRESS].a);
        CHECK(RC_INPUT.key_count[KEY_PRESS][Key_D]==1);
        b[14]=0; set_heartbeat(b,6); receive(b,18);
        CHECK(!RC_INPUT.key[KEY_PRESS].keys);
    }
    if (argc > 1 && !strcmp(argv[1], "length"))
    {
        uint8_t b[18]; const uint16_t center[5] = {1024,1024,1024,1024,1024};
        encode(b, center, 3, 3);
        RC_ctrl_t before[2]; memcpy(before, rc, sizeof(before));
        for (unsigned n=0; n<18; ++n) {
            receive(b, n);
            CHECK(!RemoteControlIsOnline());
            CHECK(!memcmp(before, rc, sizeof(before)));
        }
        receive(b, 19);
        receive(b, 65535);
        CHECK(!RemoteControlIsOnline());
        CHECK(usart_rx_oversize_total == 2);
        CHECK(DIAG(rejected_length) == 17);
        CHECK(!memcmp(before, rc, sizeof(before)));
        receive(b, 18);
        CHECK(RemoteControlIsOnline());
        CHECK(EstopIsLatched());
    }
    if (argc > 1 && !strcmp(argv[1], "fields"))
    {
        uint8_t b[18]; uint16_t ch[5] = {1024,1024,1024,1024,1024};
        encode(b, ch, 3, 3); receive(b, 18);
        RC_ctrl_t before[2]; memcpy(before, rc, sizeof(before));
        for (unsigned i=0; i<5; ++i) {
            ch[i]=363; encode(b,ch,3,3); b[14]=1; receive(b,18);
            CHECK(!memcmp(before,rc,sizeof(before)));
            ch[i]=1685; encode(b,ch,3,3); receive(b,18);
            CHECK(!memcmp(before,rc,sizeof(before)));
            ch[i]=1024;
        }
        encode(b,ch,0,3); receive(b,18); CHECK(!memcmp(before,rc,sizeof(before)));
        encode(b,ch,3,0); receive(b,18); CHECK(!memcmp(before,rc,sizeof(before)));
        for (unsigned i=12; i<=13; ++i) {
            encode(b,ch,3,3); b[i]=2; receive(b,18); CHECK(!memcmp(before,rc,sizeof(before)));
        }
        for (unsigned i=0; i<11; ++i) { now += 10; receive(b,18); daemon_cycle(); }
        CHECK(!RemoteControlIsOnline()); CHECK(EstopIsLatched());
        RC_ctrl_t empty[2] = {0}; CHECK(!memcmp(empty,rc,sizeof(empty)));
        receive(b,18); CHECK(!RemoteControlIsOnline());
        CHECK(DIAG(offline_events) == 1);
        CHECK(DIAG(offline_tick) - DIAG(offline_last_valid_tick) == 110);
        for (unsigned edge=0; edge<2; ++edge) {
            for(unsigned i=0; i<5; ++i) ch[i]=edge ? 1684 : 364;
            encode(b,ch,1,2); receive(b,18);
            CHECK(RemoteControlIsOnline()); CHECK(RC_INPUT.rc.dial == (edge ? 660 : -660));
            CHECK(RC_INPUT.rc.rocker_l_ == (edge ? 660 : -660));
        }
    }
    if (argc > 1 && !strcmp(argv[1], "queue-keys")) {
        pc_frame(1,0); pc_frame(2,0);
        uint8_t b[18]; uint16_t ch[5]={1100,900,1200,800,1024};
        encode(b,ch,1,2); b[6]=0xfe; b[7]=0xff; b[8]=0x23; b[9]=1; b[12]=b[13]=1;
        set_heartbeat(b,3);
        b[14]=1; memcpy(dma_buffer,b,18); HAL_UARTEx_RxEventCallback(&huart5,18);
        memset(dma_buffer,0xff,18); USARTProcessRx();
        CHECK(RemoteControlIsOnline()); CHECK(RC_INPUT.rc.rocker_r_==76);
        CHECK(RC_INPUT.rc.rocker_r1==-124 && RC_INPUT.rc.rocker_l_==176 && RC_INPUT.rc.rocker_l1==-224);
        CHECK(RC_INPUT.mouse.x==-2 && RC_INPUT.mouse.y==291 && RC_INPUT.mouse.press_r==1);
        CHECK(RC_INPUT.key_count[KEY_PRESS][Key_W]==1);
        receive(b,18); CHECK(RC_INPUT.key_count[KEY_PRESS][Key_W]==1);
        b[14]=0; b[12]=2; receive(b,18); /* Invalid release doesn't alter edges. */
        b[14]=1; b[12]=1; receive(b,18); CHECK(RC_INPUT.key_count[KEY_PRESS][Key_W]==1);
        b[14]=0; set_heartbeat(b,4); receive(b,18); b[14]=1|(1<<Key_Ctrl); set_heartbeat(b,5); receive(b,18);
        CHECK(RC_INPUT.key_count[KEY_PRESS_WITH_CTRL][Key_W]==1);
        b[14]=0; set_heartbeat(b,6); receive(b,18); b[14]=1|(1<<Key_Shift); set_heartbeat(b,7); receive(b,18);
        CHECK(RC_INPUT.key_count[KEY_PRESS_WITH_SHIFT][Key_W]==1);
        CHECK(DIAG(accepted_frames)==9 && DIAG(rejected_fields)==1);
    }
    if (argc > 1 && !strcmp(argv[1], "gestures")) {
        RemoteSafetyGestures_s holds; RemoteSafetyGesturesInit(&holds);
        RC_ctrl_t input={0}; input.rc.switch_left=input.rc.switch_right=RC_SW_MID;
        RobotSafetySetRcOnline(1); RobotSafetySetImuValid(1); RobotSafetySetCalibValid(1);
        const uint32_t starts[]={0,UINT32_MAX-500};
        for(unsigned i=0;i<2;++i) {
            EstopRequest(ESTOP_REASON_MANUAL); RemoteSafetyGesturesInit(&holds);
            RemoteSafetyGesturesUpdate(&holds,&input,1,starts[i],1,0u); // 单元夹具直接提供每次开关读数。
            RemoteSafetyGesturesUpdate(&holds,&input,1,starts[i]+999u,1,0u); CHECK(EstopIsLatched());
            RemoteSafetyGesturesUpdate(&holds,&input,1,starts[i]+1000u,1,0u); CHECK(!EstopIsLatched());
            RobotSafetyUpdate(); CHECK(RobotSafetyGetState()==ROBOT_SAFE);
        }
        input.rc.switch_left=input.rc.switch_right=RC_SW_DOWN;
        RemoteSafetyGesturesInit(&holds);
        RobotSafetyRequestEnable(); /* Diagnostic must revoke even an earlier request. */
        RemoteSafetyGesturesUpdate(&holds,&input,1,0,0,0u);
        RemoteSafetyGesturesUpdate(&holds,&input,1,1000,0,0u);
        RobotSafetyUpdate(); CHECK(!RobotSafetyIsReady());
        RemoteSafetyGesturesUpdate(&holds,&input,1,1001,1,0u);
        RemoteSafetyGesturesUpdate(&holds,&input,1,2001,1,0u); RobotSafetyUpdate(); CHECK(RobotSafetyIsReady());
        input.rc.switch_left=input.rc.switch_right=RC_SW_UP;
        RemoteSafetyGesturesUpdate(&holds,&input,1,2100,1,0u);
        RemoteSafetyGesturesUpdate(&holds,&input,1,3100,1,0u); RobotSafetyUpdate(); CHECK(!RobotSafetyIsReady());
        EstopRequest(ESTOP_REASON_MANUAL); input.rc.switch_left=input.rc.switch_right=RC_SW_MID;
        RemoteSafetyGesturesUpdate(&holds,&input,1,4000,1,0u);
        RemoteSafetyGesturesUpdate(&holds,&input,0,4500,1,0u);
        RemoteSafetyGesturesUpdate(&holds,&input,1,5000,1,0u); CHECK(EstopIsLatched());
        RemoteSafetyGesturesUpdate(&holds,&input,1,5999,1,0u); CHECK(EstopIsLatched());
        RemoteSafetyGesturesUpdate(&holds,&input,1,6000,1,0u); CHECK(!EstopIsLatched());
        input.rc.dial=301; RemoteSafetyGesturesUpdate(&holds,&input,1,6100,1,0u); CHECK(EstopIsLatched());
    }
    return 0;
}
