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
#include "bsp_usart.h"
#include "remote_safety.h"
#include "../../bsp/board/board_uart_stm32.h"
#if MC02_BENCH_RC_DIAGNOSTICS
#include "remote_bench.h"
#endif
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); exit(1); } } while (0)
UART_HandleTypeDef huart5, huart9;
static DMA_HandleTypeDef huart5_dma, huart9_dma;
static uint32_t now;
static uint8_t *dma_buffer;
static RemoteControlDiagnostics_s remote_diag_snapshot;
#define DIAG(field) (RemoteControlGetDiagnostics(&remote_diag_snapshot), remote_diag_snapshot.field)
static RemoteControlSnapshot_s remote_input_snapshot;
#define RC_INPUT (RemoteControlGetSnapshot(&remote_input_snapshot), remote_input_snapshot.input)
extern volatile uint32_t usart_rx_oversize_total;
uint32_t HAL_GetTick(void) { return now; }

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
            RemoteSafetyGesturesUpdate(&holds,&input,1,starts[i],1);
            RemoteSafetyGesturesUpdate(&holds,&input,1,starts[i]+999u,1); CHECK(EstopIsLatched());
            RemoteSafetyGesturesUpdate(&holds,&input,1,starts[i]+1000u,1); CHECK(!EstopIsLatched());
            RobotSafetyUpdate(); CHECK(RobotSafetyGetState()==ROBOT_SAFE);
        }
        input.rc.switch_left=input.rc.switch_right=RC_SW_DOWN;
        RemoteSafetyGesturesInit(&holds);
        RobotSafetyRequestEnable(); /* Diagnostic must revoke even an earlier request. */
        RemoteSafetyGesturesUpdate(&holds,&input,1,0,0);
        RemoteSafetyGesturesUpdate(&holds,&input,1,1000,0);
        RobotSafetyUpdate(); CHECK(!RobotSafetyIsReady());
        RemoteSafetyGesturesUpdate(&holds,&input,1,1001,1);
        RemoteSafetyGesturesUpdate(&holds,&input,1,2001,1); RobotSafetyUpdate(); CHECK(RobotSafetyIsReady());
        input.rc.switch_left=input.rc.switch_right=RC_SW_UP;
        RemoteSafetyGesturesUpdate(&holds,&input,1,2100,1);
        RemoteSafetyGesturesUpdate(&holds,&input,1,3100,1); RobotSafetyUpdate(); CHECK(!RobotSafetyIsReady());
        EstopRequest(ESTOP_REASON_MANUAL); input.rc.switch_left=input.rc.switch_right=RC_SW_MID;
        RemoteSafetyGesturesUpdate(&holds,&input,1,4000,1);
        RemoteSafetyGesturesUpdate(&holds,&input,0,4500,1);
        RemoteSafetyGesturesUpdate(&holds,&input,1,5000,1); CHECK(EstopIsLatched());
        RemoteSafetyGesturesUpdate(&holds,&input,1,5999,1); CHECK(EstopIsLatched());
        RemoteSafetyGesturesUpdate(&holds,&input,1,6000,1); CHECK(!EstopIsLatched());
        input.rc.dial=301; RemoteSafetyGesturesUpdate(&holds,&input,1,6100,1); CHECK(EstopIsLatched());
    }
    return 0;
}
