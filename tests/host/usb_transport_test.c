/** @file usb_transport_test.c
 * @brief 生产USB核心的容量、缓冲归属、超时、连接变化及有限恢复回归。
 * @note 每个场景独立进程；替换端点和时钟，测试不接触实际USB或复位开发板。
 */
#define _GNU_SOURCE
#include "usb_transport_port.h"
#include "test_check.h"
#include <pthread.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

static pthread_mutex_t critical = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
static atomic_uint now_ms;
static _Thread_local uint8_t in_isr;
static unsigned leases, starts, stops, submits, completions, links, received;
static USBStatus_t start_status = USB_OK, stop_status = USB_OK, submit_status = USB_ACCEPTED;
static uint8_t tick_available = 1u;
static const uint8_t *borrowed;
static size_t borrowed_length;
static USBRxBlock_t metadata[32];
static uint8_t first_bytes[32];
static USBTxEvent_t completed_event;

// 生产核心若新增了分配调用，此目标立即失败；主机线程库的内部存储不属于核心。
void *__wrap_malloc(size_t size) { (void)size; TEST_CHECK(0); return NULL; }
void *__wrap_calloc(size_t count,size_t size) { (void)count; (void)size; TEST_CHECK(0); return NULL; }
void __wrap_free(void *pointer) { (void)pointer; TEST_CHECK(0); }

uint32_t USBPortNowMs(void) { return atomic_load(&now_ms); }
uint32_t USBPortEnter(void) { TEST_CHECK(pthread_mutex_lock(&critical) == 0); return 0u; }
void USBPortExit(uint32_t saved) { (void)saved; TEST_CHECK(pthread_mutex_unlock(&critical) == 0); }
uint8_t USBPortInISR(void) { return in_isr; }
uint8_t USBPortAcquireTick(void) { if (!tick_available) return 0u; ++leases; return 1u; }
void USBPortReleaseTick(void) { TEST_CHECK(leases > 0u); --leases; }
USBStatus_t USBPortStart(void) { ++starts; return start_status; }
USBStatus_t USBPortStop(void) { ++stops; if (stop_status == USB_OK) borrowed = NULL; return stop_status; }
USBStatus_t USBPortSubmit(const uint8_t *data, size_t length)
{
    ++submits;
    if (submit_status == USB_ACCEPTED || submit_status == USB_SEND_ERROR)
    { borrowed = data; borrowed_length = length; }
    return submit_status;
}

static USBTransportState_t State(void)
{
    USBTransportState_t result;
    TEST_CHECK(USBTransportGetState(&result) == USB_OK);
    return result;
}
static void Complete(void *context, const USBTxEvent_t *event)
{
    TEST_CHECK(context == &received && !in_isr);
    completed_event = *event;
    ++completions;
    TEST_CHECK(USBTransportProcess() == USB_BUSY);
}
static void Link(void *context, uint32_t generation, USBLinkReason_t reason)
{
    TEST_CHECK(context == &received && !in_isr && generation != 0u);
    (void)reason;
    ++links;
}
static void Receive(void *context, const USBRxBlock_t *block)
{
    TEST_CHECK(context == &received && !in_isr && received < 32u);
    metadata[received] = *block;
    first_bytes[received++] = block->data[0];
    TEST_CHECK(USBTransportProcessRx() == USB_BUSY);
}
static void Init(void)
{
    USBTransportConfig_t config = {Receive, Complete, Link, &received};
    TEST_CHECK(USBTransportInit(&config) == USB_OK);
    // 配置表不是长期借用；回调声明和context由生产模块按值保存。
    memset(&config, 0, sizeof(config));
    TEST_CHECK(USBTransportProcess() == USB_OK);
    TEST_CHECK(starts == 1u && State().phase == USB_PHASE_WAIT_CONFIGURATION && leases == 0u);
    USBTransportOnLink(USB_LINK_CONFIGURED, 1u);
    TEST_CHECK(USBTransportProcess() == USB_OK && State().phase == USB_PHASE_READY);
}
static uint32_t Send(void)
{
    uint32_t id = 0u;
    const uint8_t bytes[] = {1u,2u,3u,4u};
    TEST_CHECK(USBTransportSubmit(bytes, sizeof(bytes), &id) == USB_ACCEPTED && id != 0u);
    return id;
}
static void Tick(uint32_t value) { atomic_store(&now_ms, value); }
static void Timeout(void)
{
    USBTransportState_t before = State();
    Tick(before.last_tx.submitted_ms + USB_TX_TIMEOUT_MS);
    TEST_CHECK(USBTransportProcess() == USB_OK);
    TEST_CHECK(State().last_tx.result == USB_TX_TIMEOUT);
}
static void ReconfigureAfterRestart(void)
{
    TEST_CHECK(State().phase == USB_PHASE_DISCONNECT_WAIT);
    uint32_t stopped_at = USBPortNowMs();
    unsigned old_starts = starts;
    Tick(stopped_at + 249u);
    TEST_CHECK(USBTransportProcess() == USB_OK && starts == old_starts);
    Tick(stopped_at + 250u);
    TEST_CHECK(USBTransportProcess() == USB_OK && starts == old_starts + 1u);
    TEST_CHECK(State().phase == USB_PHASE_WAIT_CONFIGURATION);
    USBTransportOnLink(USB_LINK_CONFIGURED, 1u);
    TEST_CHECK(USBTransportProcess() == USB_OK && leases == 0u);
}

static void Arguments(void)
{
    TEST_CHECK(USBTransportNextNonzero(0u)==1u && USBTransportNextNonzero(UINT32_MAX)==1u);
    TEST_CHECK(USBTransportNextNonzero(9u)==10u);
    USBTransportState_t output, previous;
    memset(&output, 0xa5, sizeof(output)); previous = output;
    TEST_CHECK(USBTransportGetState(&output) == USB_NOT_INITIALIZED && memcmp(&output,&previous,sizeof(output)) == 0);
    TEST_CHECK(USBTransportGetState(NULL) == USB_BAD_ARGUMENT);
    in_isr = 1u;
    TEST_CHECK(USBTransportInit(NULL) == USB_CONTEXT_ERROR);
    TEST_CHECK(USBTransportProcess() == USB_CONTEXT_ERROR);
    TEST_CHECK(USBTransportProcessRx() == USB_CONTEXT_ERROR);
    TEST_CHECK(USBTransportGetState(&output) == USB_CONTEXT_ERROR);
    in_isr = 0u;
    Init();
    previous = State();
    TEST_CHECK(USBTransportInit(NULL) == USB_ALREADY_INITIALIZED);
    output = State(); TEST_CHECK(memcmp(&output,&previous,sizeof(output)) == 0);
    uint8_t data[129] = {0}; uint32_t id = 91u;
    TEST_CHECK(USBTransportSubmit(NULL,1u,&id) == USB_BAD_ARGUMENT);
    TEST_CHECK(USBTransportSubmit(data,0u,&id) == USB_BAD_ARGUMENT);
    TEST_CHECK(USBTransportSubmit(data,sizeof(data),&id) == USB_BAD_ARGUMENT);
    TEST_CHECK(USBTransportSubmit(data,1u,NULL) == USB_BAD_ARGUMENT && id == 91u);
    tick_available = 0u;
    TEST_CHECK(USBTransportSubmit(data,1u,&id) == USB_TIMEBASE_ERROR && id == 91u);
    TEST_CHECK(submits == 0u && leases == 0u);
}
static void Queue(void)
{
    Init();
    uint8_t data[129]; memset(data, 0x53, sizeof(data));
    for (unsigned i=0u;i<8u;++i) { data[0]=(uint8_t)i; USBTransportOnReceive(data,128u); }
    data[0]=9u; USBTransportOnReceive(data,1u);
    memset(data,0xff,sizeof(data));
    TEST_CHECK(State().rx_used == 8u && State().rx_dropped == 1u);
    TEST_CHECK(USBTransportProcessRx() == USB_OK && received == 4u && State().rx_used == 4u);
    TEST_CHECK(USBTransportProcessRx() == USB_OK && received == 8u && State().rx_used == 0u);
    for (unsigned i=0u;i<8u;++i)
    { TEST_CHECK(first_bytes[i] == i && metadata[i].sequence == i+1u && metadata[i].length == 128u); }
    USBTransportOnReceive(data,129u);
    USBTransportOnReceive(data,1u);
    TEST_CHECK(USBTransportProcessRx() == USB_OK && received == 9u);
    TEST_CHECK(metadata[8].sequence == 11u && State().rx_oversized == 1u);
}
static void TimeAndGeneration(void)
{
    Init(); uint8_t byte = 7u;
    USBTransportOnReceive(&byte,1u); Tick(50u); USBTransportProcessRx();
    TEST_CHECK(received == 1u && metadata[0].received_ms == 0u);
    Tick(100u); USBTransportOnReceive(&byte,1u); Tick(151u); USBTransportProcessRx();
    TEST_CHECK(received == 1u && State().rx_expired == 1u);
    Tick(UINT32_MAX-20u); USBTransportOnReceive(&byte,1u); Tick(29u); USBTransportProcessRx();
    TEST_CHECK(received == 2u);
    uint32_t generation = State().generation;
    USBTransportOnReceive(&byte,1u);
    USBTransportOnLink(USB_LINK_RESET,1u);
    TEST_CHECK(!USBTransportGenerationIsCurrent(generation) && State().rx_used == 0u);
    USBTransportOnLink(USB_LINK_CONFIGURED,1u); USBTransportProcessRx();
    TEST_CHECK(received == 2u && State().generation != generation);
}
static void Sending(void)
{
    Init(); uint8_t source[128]; memset(source,0x53,sizeof(source)); uint32_t id;
    TEST_CHECK(USBTransportSubmit(source,sizeof(source),&id) == USB_ACCEPTED);
    memset(source,0xee,sizeof(source));
    TEST_CHECK(borrowed != source && borrowed_length == 128u && borrowed[0] == 0x53u && leases == 1u);
    uint32_t untouched = 91u;
    TEST_CHECK(USBTransportSubmit(source,1u,&untouched) == USB_BUSY && untouched == 91u);
    TEST_CHECK(borrowed[0] == 0x53u);
    Tick(99u); in_isr=1u; USBTransportOnTxComplete(); USBTransportOnTxComplete(); in_isr=0u;
    TEST_CHECK(completions == 0u && leases == 0u);
    USBTransportProcess();
    TEST_CHECK(completions == 1u && completed_event.id == id && completed_event.result == USB_TX_COMPLETE);
    USBTransportProcess(); TEST_CHECK(completions == 1u);
    submit_status=USB_BUSY; USBTxEvent_t old = State().last_tx;
    TEST_CHECK(USBTransportSubmit(source,1u,&untouched) == USB_BUSY && leases == 0u);
    USBTxEvent_t after = State().last_tx;
    TEST_CHECK(memcmp(&old,&after,sizeof(old)) == 0);
}
static void TimeoutRecovery(void)
{
    Init(); uint32_t id=Send();
    Tick(99u); USBTransportProcess(); TEST_CHECK(stops == 0u);
    Tick(100u); USBTransportProcess();
    TEST_CHECK(stops == 1u && State().restart_attempts == 1u && completions == 1u);
    TEST_CHECK(completed_event.id == id && completed_event.result == USB_TX_TIMEOUT);
    TEST_CHECK(leases == 1u && !State().tx_buffer_held);
    USBTransportOnTxComplete(); USBTransportProcess(); TEST_CHECK(completions == 1u);
    ReconfigureAfterRestart();
    TEST_CHECK(State().restart_attempts == 1u && State().phase == USB_PHASE_READY);
}
static void StopFailure(void)
{
    Init(); Send(); stop_status=USB_SEND_ERROR;
    Tick(100u); TEST_CHECK(USBTransportProcess() == USB_SEND_ERROR);
    TEST_CHECK(State().phase == USB_PHASE_ISOLATED && State().tx_buffer_held && leases == 2u);
    USBTransportOnLink(USB_LINK_CONFIGURED,1u); USBTransportOnTxComplete();
    Tick(10000u); USBTransportProcess();
    TEST_CHECK(State().phase == USB_PHASE_ISOLATED && leases == 2u && completions == 1u && starts == 1u);
}
static void StartError(void)
{
    Init(); uint8_t byte=0x53u; uint32_t id=91u;
    submit_status=USB_SEND_ERROR;
    TEST_CHECK(USBTransportSubmit(&byte,1u,&id)==USB_SEND_ERROR && id==91u);
    // 报错时底层也可能已经取得数组；确认停止前不能立即复用，更不能通知未接受的编号。
    TEST_CHECK(State().tx_buffer_held && leases==1u && State().accepted==0u);
    byte=0xeeu; TEST_CHECK(borrowed!=NULL && borrowed[0]==0x53u);
    TEST_CHECK(USBTransportProcess()==USB_OK);
    TEST_CHECK(!State().tx_buffer_held && leases==1u && completions==0u);
    ReconfigureAfterRestart();
    TEST_CHECK(completions==0u && State().accepted==0u);
}
static void Exhaustion(void)
{
    Init();
    for(unsigned attempt=1u;attempt<=3u;++attempt)
    {
        Tick(attempt*2000u); Send(); Timeout();
        TEST_CHECK(State().restart_attempts == attempt);
        ReconfigureAfterRestart();
    }
    Tick(8000u); Send(); Timeout();
    TEST_CHECK(State().phase == USB_PHASE_EXHAUSTED && State().restart_attempts == 3u && stops == 3u);
    TEST_CHECK(State().tx_buffer_held && leases == 1u);
    USBTransportOnLink(USB_LINK_CONFIGURED,1u); USBTransportProcess();
    TEST_CHECK(State().phase == USB_PHASE_READY && State().restart_attempts == 3u && leases == 0u);
    uint32_t base=USBPortNowMs();
    for(uint32_t elapsed=0u;elapsed<=60000u;elapsed+=20u)
    {
        Tick(base+elapsed); Send(); USBTransportOnTxComplete();
        USBTransportReportValidCommand(State().generation,USBPortNowMs()); USBTransportProcess();
        if(elapsed<60000u) TEST_CHECK(State().restart_attempts == 3u);
    }
    TEST_CHECK(State().restart_attempts == 0u && State().restart_total == 3u);
}
static void WaitAndRetry(void)
{
    Init(); Tick(100000u); USBTransportProcess(); TEST_CHECK(stops == 0u);
    Send(); Timeout(); ReconfigureAfterRestart();
    Send(); Timeout();
    TEST_CHECK(State().phase == USB_PHASE_RETRY_WAIT && State().restart_attempts == 1u);
    Tick(101099u); USBTransportProcess(); TEST_CHECK(stops == 1u);
    Tick(101100u); USBTransportProcess(); TEST_CHECK(stops == 2u);
}
static void LateCompletion(void)
{
    Init(); Send(); Tick(100u); USBTransportOnTxComplete();
    TEST_CHECK(State().last_tx.result==USB_TX_TIMEOUT && completions==0u && leases==0u);
    USBTransportProcess(); TEST_CHECK(completions==1u && completed_event.result==USB_TX_TIMEOUT);
}
static void CancelAndResume(void)
{
    Init(); uint32_t id=Send();
    USBTransportOnLink(USB_LINK_RESET,1u);
    TEST_CHECK(!State().tx_buffer_held && leases==0u);
    USBTransportOnTxComplete(); USBTransportProcess();
    TEST_CHECK(completions==1u && completed_event.id==id && completed_event.result==USB_TX_CANCELLED);
    TEST_CHECK(State().restart_attempts==0u);
    USBTransportOnLink(USB_LINK_CONFIGURED,1u); USBTransportProcess();
    uint32_t generation=State().generation;
    USBTransportOnLink(USB_LINK_SUSPENDED,1u);
    TEST_CHECK(!USBTransportGenerationIsCurrent(generation));
    USBTransportOnLink(USB_LINK_RESUMED,1u); USBTransportProcess();
    TEST_CHECK(State().phase==USB_PHASE_READY && State().generation!=generation && leases==0u);
}
static void TransferError(void)
{
    Init(); uint32_t id=Send();
    USBTransportOnLink(USB_LINK_TRANSFER_ERROR,0u);
    TEST_CHECK(State().tx_buffer_held && leases==1u);
    USBTransportProcess();
    TEST_CHECK(completions==1u && completed_event.id==id && completed_event.result==USB_TX_ERROR);
    TEST_CHECK(State().tx_errors==1u && State().cancelled==0u && State().restart_attempts==1u);
}
static void InitFailure(void)
{
    start_status=USB_SEND_ERROR;
    TEST_CHECK(USBTransportInit(NULL)==USB_OK && USBTransportProcess()==USB_SEND_ERROR);
    TEST_CHECK(State().phase==USB_PHASE_EXHAUSTED && leases==0u);
    Tick(100000u); USBTransportProcess(); TEST_CHECK(starts==1u && stops==0u);
}
static void Enumeration(void)
{
    Init(); Send(); Timeout();
    for(unsigned attempt=1u;attempt<=3u;++attempt)
    {
        TEST_CHECK(State().phase==USB_PHASE_DISCONNECT_WAIT);
        Tick(USBPortNowMs()+250u); USBTransportProcess();
        TEST_CHECK(State().phase==USB_PHASE_WAIT_CONFIGURATION);
        Tick(USBPortNowMs()+4999u); USBTransportProcess();
        TEST_CHECK(State().phase==USB_PHASE_WAIT_CONFIGURATION);
        Tick(USBPortNowMs()+1u); USBTransportProcess();
        TEST_CHECK(State().phase==USB_PHASE_RETRY_WAIT);
        Tick(USBPortNowMs()+1000u); USBTransportProcess();
    }
    TEST_CHECK(State().phase==USB_PHASE_EXHAUSTED && State().restart_attempts==3u &&
               stops==3u && leases==0u);
    USBTransportOnLink(USB_LINK_CONFIGURED,1u); USBTransportProcess();
    TEST_CHECK(State().phase==USB_PHASE_READY && State().restart_attempts==3u);
}
static void TimeWrap(void)
{
    Init(); Tick(UINT32_MAX-50u); Send(); Tick(48u); USBTransportProcess();
    TEST_CHECK(stops==0u);
    Tick(49u); USBTransportProcess(); TEST_CHECK(stops==1u && State().timeouts==1u);
    ReconfigureAfterRestart();
}
static void StableNeedsBoth(uint8_t transmit_only)
{
    Init(); Send(); Timeout(); ReconfigureAfterRestart();
    uint32_t base=USBPortNowMs();
    for(uint32_t elapsed=0u;elapsed<=70000u;elapsed+=20u)
    {
        Tick(base+elapsed);
        if(transmit_only) { Send(); USBTransportOnTxComplete(); }
        else USBTransportReportValidCommand(State().generation,USBPortNowMs());
        USBTransportProcess();
    }
    TEST_CHECK(State().restart_attempts==1u && !State().stable_tracking);
}
static void StableGap(void)
{
    Init(); Send(); Timeout(); ReconfigureAfterRestart();
    uint32_t base=USBPortNowMs();
    for(uint32_t elapsed=0u;elapsed<=90100u;elapsed+=20u)
    {
        if(elapsed>30000u && elapsed<30080u) continue;
        Tick(base+elapsed); Send(); USBTransportOnTxComplete();
        USBTransportReportValidCommand(State().generation,USBPortNowMs()); USBTransportProcess();
        if(elapsed<90080u) TEST_CHECK(State().restart_attempts==1u);
    }
    TEST_CHECK(State().restart_attempts==0u);
}
static void *Producer(void *unused)
{
    (void)unused; uint8_t data[8];
    for(unsigned i=0u;i<20000u;++i) { memset(data,(int)(i&0xffu),sizeof(data)); USBTransportOnReceive(data,sizeof(data)); }
    return NULL;
}
static void ConcurrentReceive(void *context,const USBRxBlock_t *block)
{
    (void)context;
    for(size_t i=1u;i<block->length;++i) TEST_CHECK(block->data[i] == block->data[0]);
}
static void Threads(void)
{
    USBTransportConfig_t config={ConcurrentReceive,NULL,NULL,NULL};
    TEST_CHECK(USBTransportInit(&config) == USB_OK); USBTransportProcess();
    USBTransportOnLink(USB_LINK_CONFIGURED,1u);
    pthread_t producer; TEST_CHECK(pthread_create(&producer,NULL,Producer,NULL) == 0);
    for(unsigned i=0u;i<20000u;++i)
    { USBTransportProcessRx(); USBTransportState_t state=State(); TEST_CHECK(state.rx_used<=8u); }
    TEST_CHECK(pthread_join(producer,NULL) == 0);
    USBTransportProcessRx(); USBTransportProcessRx(); TEST_CHECK(State().rx_used == 0u);
}
int main(int argc,char **argv)
{
    TEST_CHECK(argc == 2);
    if(strcmp(argv[1],"arguments")==0) Arguments();
    else if(strcmp(argv[1],"queue")==0) Queue();
    else if(strcmp(argv[1],"time-generation")==0) TimeAndGeneration();
    else if(strcmp(argv[1],"sending")==0) Sending();
    else if(strcmp(argv[1],"timeout")==0) TimeoutRecovery();
    else if(strcmp(argv[1],"stop-failure")==0) StopFailure();
    else if(strcmp(argv[1],"start-error")==0) StartError();
    else if(strcmp(argv[1],"exhaustion")==0) Exhaustion();
    else if(strcmp(argv[1],"retry")==0) WaitAndRetry();
    else if(strcmp(argv[1],"late-completion")==0) LateCompletion();
    else if(strcmp(argv[1],"cancel-resume")==0) CancelAndResume();
    else if(strcmp(argv[1],"transfer-error")==0) TransferError();
    else if(strcmp(argv[1],"init-failure")==0) InitFailure();
    else if(strcmp(argv[1],"enumeration")==0) Enumeration();
    else if(strcmp(argv[1],"wrap")==0) TimeWrap();
    else if(strcmp(argv[1],"transmit-only")==0) StableNeedsBoth(1u);
    else if(strcmp(argv[1],"receive-only")==0) StableNeedsBoth(0u);
    else if(strcmp(argv[1],"stable-gap")==0) StableGap();
    else if(strcmp(argv[1],"threads")==0) Threads();
    else TEST_CHECK(0);
    return 0;
}
