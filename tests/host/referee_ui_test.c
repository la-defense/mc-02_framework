/** @file referee_ui_test.c
 * @brief 通过生产构图、发送和任务入口验证边界、协议字节、失败及在途缓冲寿命。
 * @note CTest选择场景；本目标使用发送模型及选定格式化/CRC失败，真实队列另由UI联测覆盖。
 */
#include "referee_UI.h"
#include "referee_task.h"
#include "test_check.h"
#include "crc.h"
#include "crc_injection.h"
#include "daemon.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <math.h>

// 序号由生产共享入口维护；测试通过只读副本查询。
static uint8_t current_sequence(void)
{
    RefereeTxState_t value;
    return RefereeGetTxState(&value) == USART_SEND_ACCEPTED ? value.next_sequence : 0u;
}
static uint8_t rx[255], sent[128];
static uint16_t sent_length, inflight_length;
static const uint8_t *inflight;
static unsigned sends, delays, reloads, unregisters;
static int ready = 1, complete_on_delay = 1, format_fail, registration_fail;
static USARTSendStatus_t start_status = USART_SEND_ACCEPTED;
static USARTInstance instance = {0};
static DaemonInstance daemon_instance;
static usart_module_callback receive_callback;
static void *receive_context;
static uint32_t now_ms;
static uint32_t rx_sequence;

uint32_t HAL_GetTick(void) { return now_ms; }
DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *c) { (void)c; return &daemon_instance; }
uint8_t DaemonUnregister(DaemonInstance *d) { (void)d; ++unregisters; return 1; }
void DaemonReload(DaemonInstance *d) { (void)d; ++reloads; }
DaemonStatus_t DaemonReportAt(DaemonInstance *d, uint32_t received_ms, uint8_t flags)
{ (void)d; (void)received_ms; if ((flags & DAEMON_REPORT_TRANSPORT_VALID) != 0u) ++reloads; return DAEMON_OK; }
uint8_t DaemonIsOnline(DaemonInstance *d) { (void)d; return 1; }
USARTInstance *USARTRegister(USART_Init_Config_s *c)
{ receive_callback=c->module_callback;receive_context=c->context; return registration_fail?NULL:&instance; }
uint8_t USARTServiceInit(USARTInstance *i) { (void)i; return 1u; }
static USARTTxRequest_t pending_request;
static uint32_t next_id,pending_id;
static uint8_t tx_buffer[128];
static void notify_result(USARTTxResult_t result)
{
    TEST_CHECK(pending_id!=0u);
    USARTTxEvent_t event={.id=pending_id,.result=result,.started=1u,
        .started_ms=now_ms,.completed_ms=now_ms};
    USARTTxRequest_t request=pending_request;pending_id=0u;ready=1;
    request.complete(request.context,&event);
}
USARTSendStatus_t USARTSubmitTx(USARTInstance *i,const USARTTxRequest_t *request,uint32_t *id)
{
    TEST_CHECK(i==&instance && request->mode==USART_TRANSFER_DMA && request->length<=sizeof(sent));
    if(start_status!=USART_SEND_ACCEPTED)return start_status;
    if(!ready)return USART_SEND_BUSY;
    memcpy(tx_buffer,request->data,request->length);memcpy(sent,request->data,request->length);
    sent_length=(uint16_t)request->length;inflight=tx_buffer;inflight_length=sent_length;
    pending_request=*request;pending_id=++next_id;*id=pending_id;++sends;ready=0;
    if(complete_on_delay)notify_result(USART_TX_COMPLETE);
    return USART_SEND_ACCEPTED;
}
// 该目标只检查编码与状态选择；真实发送统计由uart_ui_integration_test覆盖。
USARTSendStatus_t USARTGetTxState(const USARTInstance *instance, USARTTxState_t *output)
{
    (void)instance; memset(output,0,sizeof(*output));
    return USART_SEND_ACCEPTED;
}
USARTSendStatus_t USARTCancelTx(USARTInstance *i,uint32_t id)
{
    TEST_CHECK(i==&instance);
    if(pending_id!=id)return USART_SEND_NOT_FOUND;
    notify_result(USART_TX_CANCELLED);return USART_SEND_ACCEPTED;
}
void osDelay(uint32_t milliseconds)
{ (void)milliseconds;++delays;TEST_CHECK(0); }
static void ignore_result(void *context,const USARTTxEvent_t *event)
{ (void)context;(void)event; }
// MinGW的C99标准库使用此实际符号；包装目标必须与生产对象的未定义引用一致。
#ifdef _WIN32
#define __real_vsnprintf __real___mingw_vsnprintf
#define __wrap_vsnprintf __wrap___mingw_vsnprintf
#endif
int __real_vsnprintf(char *out,size_t size,const char *fmt,va_list args);
/** @brief 包装选定的一次合法格式化失败；其他调用执行真实标准库。 */
int __wrap_vsnprintf(char *out,size_t size,const char *fmt,va_list args)
{
    if(format_fail) { format_fail=0; return -1; }
    return __real_vsnprintf(out,size,fmt,args);
}
#ifdef __linux__
int __real___vsnprintf_chk(char *out,size_t size,int flag,size_t capacity,const char *fmt,va_list args);
/** @brief 保留glibc的加固检查；优化构建实际调用_chk时也可注入一次失败。 */
int __wrap___vsnprintf_chk(char *out,size_t size,int flag,size_t capacity,const char *fmt,va_list args)
{
    if(format_fail) { format_fail=0; return -1; }
    return __real___vsnprintf_chk(out,size,flag,capacity,fmt,args);
}
#endif

static const referee_id_t id={.Robot_Color=Robot_Red,.Robot_ID=1u,.Cilent_ID=0x101u};
static uint16_t reference_crc(const uint8_t *p,size_t n,uint16_t poly,uint16_t seed)
{
    for(size_t i=0;i<n;++i) {
        seed^=p[i];
        for(unsigned bit=0;bit<8;++bit) seed=(uint16_t)((seed>>1)^((seed&1u)?poly:0u));
    }
    return seed;
}
static size_t make_referee_frame(uint8_t *frame,uint16_t command,const uint8_t *payload,size_t length)
{
    memset(frame,0,length+9u);frame[0]=0xa5u;frame[1]=(uint8_t)length;
    frame[2]=(uint8_t)(length>>8u);frame[3]=(uint8_t)rx_sequence;
    frame[4]=(uint8_t)reference_crc(frame,4u,0x8cu,0xffu);
    frame[5]=(uint8_t)command;frame[6]=(uint8_t)(command>>8u);memcpy(frame+7u,payload,length);
    uint16_t crc=reference_crc(frame,length+7u,0x8408u,0xffffu);
    frame[length+7u]=(uint8_t)crc;frame[length+8u]=(uint8_t)(crc>>8u);return length+9u;
}
static void feed_referee(uint16_t command,const uint8_t *payload,size_t length)
{
    size_t total=make_referee_frame(rx,command,payload,length);
    const USART_RxBlock_t block={rx,total,now_ms,++rx_sequence};
    receive_callback(receive_context,&block);
}
static void publish_robot_state(uint8_t robot_id)
{
    uint8_t payload[17]={0};payload[0]=robot_id;
    feed_referee(ID_game_robot_state,payload,sizeof(payload));
}
static void check_frame(uint16_t content,size_t length,uint8_t sequence)
{
    TEST_CHECK(sent_length==15u+length && sent[0]==0xa5u && sent[3]==sequence);
    TEST_CHECK(sent[1]==6u+length && sent[2]==0u);
    TEST_CHECK(sent[4]==reference_crc(sent,4u,0x8cu,0xffu));
    TEST_CHECK(sent[5]==1u && sent[6]==3u && sent[7]==(uint8_t)content && sent[8]==(uint8_t)(content>>8));
    TEST_CHECK(reference_crc(sent,sent_length-2u,0x8408u,0xffffu)==
               (uint16_t)(sent[sent_length-2u]|((uint16_t)sent[sent_length-1u]<<8)));
}

static void drawing(void)
{
    struct { uint32_t before; Graph_Data_t value; uint32_t after; } box;
    memset(&box,0xa5,sizeof(box));
    Graph_Data_t *g=&box.value;
    TEST_CHECK(UILineDraw(g,"abc",1,2,8,3,100,200,300,400)==UI_OK);
    TEST_CHECK(g->graphic_name[0]=='c' && g->graphic_name[2]=='a');
    const char short_name[3]={'q',0,0};
    TEST_CHECK(UILineDraw(g,short_name,0,0,0,0,0,0,1920,1080)==UI_OK);
    TEST_CHECK(g->graphic_name[0]==0 && g->graphic_name[1]==0 && g->graphic_name[2]=='q');
    Graph_Data_t saved=*g;
#define REJECT(call,status) do { TEST_CHECK((call)==(status)); TEST_CHECK(memcmp(g,&saved,sizeof(saved))==0); } while(0)
    REJECT(UILineDraw(g,NULL,1,0,0,0,0,0,0,0),UI_ERROR_ARGUMENT);
    REJECT(UILineDraw(g,"abc",4,0,0,0,0,0,0,0),UI_ERROR_RANGE);
    REJECT(UILineDraw(g,"abc",1,10,0,0,0,0,0,0),UI_ERROR_RANGE);
    REJECT(UILineDraw(g,"abc",1,0,9,0,0,0,0,0),UI_ERROR_RANGE);
    REJECT(UILineDraw(g,"abc",1,0,0,1024,0,0,0,0),UI_ERROR_RANGE);
    REJECT(UILineDraw(g,"abc",1,0,0,0,1921,0,0,0),UI_ERROR_RANGE);
    REJECT(UILineDraw(g,"abc",1,0,0,0,0,1081,0,0),UI_ERROR_RANGE);
    REJECT(UILineDraw(g,"abc",1,0,0,0,0,0,UINT32_MAX,0),UI_ERROR_RANGE);
    REJECT(UIRectangleDraw(g,"abc",1,0,0,0,0,0,0,1081),UI_ERROR_RANGE);
    REJECT(UICircleDraw(g,"abc",1,0,0,0,0,0,1024),UI_ERROR_RANGE);
    REJECT(UIOvalDraw(g,"abc",1,0,0,0,0,0,2048,0),UI_ERROR_RANGE);
    REJECT(UIArcDraw(g,"abc",1,0,0,361,0,0,0,0,0,0),UI_ERROR_RANGE);
    REJECT(UIArcDraw(g,"abc",1,0,0,0,361,0,0,0,0,0),UI_ERROR_RANGE);
    REJECT(UIIntDraw(g,"abc",1,0,0,512,0,0,0,0),UI_ERROR_RANGE);
    REJECT(UIFloatDraw(g,"abc",1,0,0,1,512,0,0,0,0),UI_ERROR_RANGE);
#undef REJECT
    TEST_CHECK(UILineDraw(NULL,"abc",0,0,0,0,0,0,0,0)==UI_ERROR_ARGUMENT);
    TEST_CHECK(UIRectangleDraw(NULL,"abc",0,0,0,0,0,0,0,0)==UI_ERROR_ARGUMENT);
    TEST_CHECK(UICircleDraw(NULL,"abc",0,0,0,0,0,0,0)==UI_ERROR_ARGUMENT);
    TEST_CHECK(UIOvalDraw(NULL,"abc",0,0,0,0,0,0,0,0)==UI_ERROR_ARGUMENT);
    TEST_CHECK(UIArcDraw(NULL,"abc",0,0,0,0,0,0,0,0,0,0)==UI_ERROR_ARGUMENT);
    TEST_CHECK(UIIntDraw(NULL,"abc",0,0,0,0,0,0,0,0)==UI_ERROR_ARGUMENT);
    TEST_CHECK(UIFloatDraw(NULL,"abc",0,0,0,0,0,0,0,0,0)==UI_ERROR_ARGUMENT);
    TEST_CHECK(UIRectangleDraw(g,"abc",3,9,8,1023,1920,1080,0,0)==UI_OK);
    TEST_CHECK(UICircleDraw(g,"abc",3,9,8,1023,1920,1080,1023)==UI_OK);
    TEST_CHECK(UIOvalDraw(g,"abc",3,9,8,1023,1920,1080,2047,2047)==UI_OK);
    TEST_CHECK(UIArcDraw(g,"abc",3,9,8,360,0,1023,1920,1080,2047,2047)==UI_OK);
    int32_t values[]={INT32_MIN,-1234,-1,0,1234,INT32_MAX};
    for(size_t i=0;i<sizeof(values)/sizeof(values[0]);++i) {
        TEST_CHECK(UIIntDraw(g,"abc",3,9,8,511,1023,1920,1080,values[i])==UI_OK);
        uint32_t bits=g->radius|((uint32_t)g->end_x<<10)|((uint32_t)g->end_y<<21);
        TEST_CHECK(bits==(uint32_t)values[i]);
        TEST_CHECK(UIFloatDraw(g,"abc",3,9,8,511,511,1023,1920,1080,values[i])==UI_OK);
        TEST_CHECK(g->end_angle==0u && g->graphic_tpye==UI_Graph_Float);
    }
    TEST_CHECK(box.before==UINT32_C(0xa5a5a5a5) && box.after==UINT32_C(0xa5a5a5a5));
}

static void text_tests(void)
{
    struct { uint32_t before; String_Data_t value; uint32_t after; } box;
    memset(&box,0xa5,sizeof(box));
    String_Data_t *text=&box.value;
    char source[256];memset(source,'x',255);source[255]=0;
    size_t lengths[]={0,1,29,30};
    for(size_t i=0;i<4;++i) {
        source[lengths[i]]=0;
        TEST_CHECK(UICharDraw(text,"txt",1,9,8,511,1023,1920,1080,"%s",source)==UI_OK);
        TEST_CHECK(text->Graph_Control.end_angle==lengths[i]);
        for(size_t j=lengths[i];j<30;++j) TEST_CHECK(text->show_Data[j]==0);
        source[lengths[i]]='x';
    }
    String_Data_t saved=*text;
    source[31]=0;
    TEST_CHECK(UICharDraw(text,"txt",1,0,0,1,1,0,0,"%s",source)==UI_ERROR_LENGTH);
    TEST_CHECK(memcmp(text,&saved,sizeof(saved))==0);
    source[31]='x';
    TEST_CHECK(UICharDraw(text,"txt",1,0,0,1,1,0,0,"%s",source)==UI_ERROR_LENGTH);
    TEST_CHECK(memcmp(text,&saved,sizeof(saved))==0);
    format_fail=1;
    TEST_CHECK(UICharDraw(text,"txt",1,0,0,1,1,0,0,"%d",7)==UI_ERROR_FORMAT);
    TEST_CHECK(memcmp(text,&saved,sizeof(saved))==0);
    TEST_CHECK(UICharDraw(text,"txt",1,0,0,1,1,0,0,NULL)==UI_ERROR_ARGUMENT);
    TEST_CHECK(UICharDraw(NULL,"txt",1,0,0,1,1,0,0,"x")==UI_ERROR_ARGUMENT);
    TEST_CHECK(UICharDraw(text,"txt",1,0,0,512,1,0,0,"x")==UI_ERROR_RANGE);
    TEST_CHECK(memcmp(text,&saved,sizeof(saved))==0);
    TEST_CHECK(UICharDraw(text,"txt",1,0,0,1,1,0,0,"%d/%s",-7,"ok")==UI_OK);
    TEST_CHECK(text->Graph_Control.end_angle==5 && memcmp(text->show_Data,"-7/ok",5)==0);
    TEST_CHECK(box.before==UINT32_C(0xa5a5a5a5) && box.after==UINT32_C(0xa5a5a5a5));
}

static void frames(void)
{
    // 独立手工位段与逐位CRC核对向量，来源见REF-04票据；不调用生产编码器生成期望。
    const uint8_t golden[]={0xa5,0x15,0,0,0xbc,1,3,1,1,1,0,1,1,0x63,0x62,0x61,0x81,0x20,0,0,3,0x90,1,0x19,0,0xb0,4,0x32,0xd9,0xdf};
    Graph_Data_t graphs[7];
    for(size_t i=0;i<7;++i) TEST_CHECK(UILineDraw(&graphs[i],"abc",1,2,8,3,100,200,300,400)==UI_OK);
    TEST_CHECK(UIGraphRefresh(&id,graphs,7,1,NULL)==UI_NOT_INITIALIZED && sends==0);
    UART_HandleTypeDef handle={0};TEST_CHECK(RefereeInit(&handle,NULL)==REFEREE_OK);
    now_ms=10u;publish_robot_state(1u);

    TEST_CHECK(UIGraphRefresh(&id,graphs,7,1,NULL)==UI_SUBMITTED);
    TEST_CHECK(sent_length==sizeof(golden) && memcmp(sent,golden,sizeof(golden))==0 && current_sequence()==1);
    const size_t counts[]={1,2,5,7};
    for(size_t i=0;i<4;++i) {
        uint8_t sequence=current_sequence();
        TEST_CHECK(UIGraphRefresh(&id,graphs,7,counts[i],NULL)==UI_SUBMITTED);
        check_frame((uint16_t)(0x101+i),counts[i]*15u,sequence);
    }
    TEST_CHECK(UILineDraw(&graphs[0],"abc",0,0,0,0,0,0,0,0)==UI_OK);
    TEST_CHECK(UIRectangleDraw(&graphs[1],"abc",1,0,0,1,0,0,1920,1080)==UI_OK);
    TEST_CHECK(UICircleDraw(&graphs[2],"abc",1,0,0,1,0,0,1023)==UI_OK);
    TEST_CHECK(UIOvalDraw(&graphs[3],"abc",1,0,0,1,0,0,2047,2047)==UI_OK);
    TEST_CHECK(UIArcDraw(&graphs[4],"abc",1,0,0,360,0,1,0,0,2047,2047)==UI_OK);
    TEST_CHECK(UIFloatDraw(&graphs[5],"abc",1,0,0,15,2,1,0,0,-1234)==UI_OK);
    TEST_CHECK(UIIntDraw(&graphs[6],"abc",1,0,0,15,1,0,0,INT32_MIN)==UI_OK);
    TEST_CHECK(UIGraphRefresh(&id,graphs,7,7,NULL)==UI_SUBMITTED);
    const uint8_t negative_float[]={0x2e,0xfb,0xff,0xff}, min_integer[]={0,0,0,0x80};
    TEST_CHECK(memcmp(sent+13+5*15+11,negative_float,4)==0);
    TEST_CHECK(memcmp(sent+13+6*15+11,min_integer,4)==0);
    unsigned previous=sends;uint8_t sequence=current_sequence();
    size_t bad[]={0,3,4,6,8,256,SIZE_MAX};
    for(size_t i=0;i<7;++i) TEST_CHECK(UIGraphRefresh(&id,graphs,7,bad[i],NULL)==UI_ERROR_LENGTH);
    TEST_CHECK(UIGraphRefresh(&id,graphs,1,2,NULL)==UI_ERROR_LENGTH);
    TEST_CHECK(UIGraphRefresh(&id,NULL,7,1,NULL)==UI_ERROR_ARGUMENT);
    TEST_CHECK(UIGraphRefresh(NULL,graphs,7,1,NULL)==UI_ERROR_ARGUMENT);
    graphs[1].color=9;
    TEST_CHECK(UIGraphRefresh(&id,graphs,7,2,NULL)==UI_ERROR_RANGE);
    graphs[1]=graphs[0];
    TEST_CHECK(sends==previous && sequence==current_sequence());
    String_Data_t text;
    TEST_CHECK(UICharDraw(&text,"txt",1,0,0,1,1,0,0,"123456789012345678901234567890")==UI_OK);
    TEST_CHECK(UICharRefresh(&id,&text,NULL)==UI_SUBMITTED);
    check_frame(0x110,45,sequence);
    TEST_CHECK(memcmp(sent+28,text.show_Data,30)==0);
    text.Graph_Control.end_angle=31;
    TEST_CHECK(UICharRefresh(&id,&text,NULL)==UI_ERROR_RANGE);
    TEST_CHECK(UICharRefresh(&id,NULL,NULL)==UI_ERROR_ARGUMENT);
    TEST_CHECK(UIGraphRefresh(&id,&text.Graph_Control,1,1,NULL)==UI_ERROR_RANGE);
    TEST_CHECK(UIDelete(&id,3,0,NULL)==UI_ERROR_RANGE && UIDelete(&id,1,10,NULL)==UI_ERROR_RANGE);
    TEST_CHECK(UIDelete(NULL,1,0,NULL)==UI_ERROR_ARGUMENT);
    referee_id_t wrong=id;wrong.Cilent_ID=0x107;
    TEST_CHECK(UIDelete(&wrong,1,0,NULL)==UI_ERROR_RANGE);
    wrong.Cilent_ID=0x165;
    TEST_CHECK(UIDelete(&wrong,1,0,NULL)==UI_ERROR_RANGE);
    wrong.Robot_ID=101;
    wrong.Robot_Color=Robot_Blue;
    TEST_CHECK(UIDelete(&wrong,2,9,NULL)==UI_NOT_INITIALIZED);
    wrong.Robot_ID=109;
    TEST_CHECK(UIDelete(&wrong,0,0,NULL)==UI_NOT_INITIALIZED);
    while(current_sequence()!=255u) TEST_CHECK(UIDelete(&id,0,0,NULL)==UI_SUBMITTED);
    TEST_CHECK(UIDelete(&id,1,0,NULL)==UI_SUBMITTED && current_sequence()==0);
    check_frame(0x100,2,255);
}

static void transport(void)
{
    uint8_t bytes[128]={1,2,3};uint32_t ticket=0u;
    RefereeTxRequest_t request={2u,0x0200u,bytes,3u,100u,ignore_result,NULL,NULL};
    TEST_CHECK(RefereeSubmit(&request,&ticket)==REFEREE_SEND_NOT_INITIALIZED);
    TEST_CHECK(RefereeSubmit(NULL,&ticket)==REFEREE_SEND_BAD_ARGUMENT);
    request.timeout_ms=0u;TEST_CHECK(RefereeSubmit(&request,&ticket)==REFEREE_SEND_BAD_ARGUMENT);request.timeout_ms=100u;
    request.length=113u;TEST_CHECK(RefereeSubmit(&request,&ticket)==REFEREE_SEND_BAD_ARGUMENT);request.length=3u;
    // 掩码仅用于测试发送流程，不作为比赛规则。
    RefereeConfig_t config={.timeout_10hz_ms=300u,.timeout_3hz_ms=1000u,.timeout_1hz_ms=3000u,.timeout_map_ms=3000u,
        .interaction={REFEREE_COMPETITION_RMUC_2026,REFEREE_PROTOCOL_2026_V2,1u,0x01ffu,0x01ffu}};
    UART_HandleTypeDef h={0};TEST_CHECK(RefereeInit(&h,&config)==REFEREE_OK);
    now_ms=10u;publish_robot_state(1u);
    USARTSendStatus_t statuses[]={USART_SEND_BUSY,USART_SEND_ERROR,USART_SEND_NOT_READY,USART_SEND_BAD_ARGUMENT};
    UIStatus_t expected[]={UI_SEND_BUSY,UI_SEND_ERROR,UI_NOT_INITIALIZED,UI_SEND_ERROR};
    for(size_t i=0;i<4u;++i) {
        start_status=statuses[i];unsigned previous=sends;
        TEST_CHECK(UIDelete(&id,1,0,NULL)==expected[i]);
        TEST_CHECK(sends==previous && current_sequence()==0u && delays==0u);
    }
    start_status=USART_SEND_ACCEPTED;
    for(int failure=0;failure<2;++failure) {
        crc_fail_after=failure;TEST_CHECK(UIDelete(&id,1,0,NULL)==UI_ERROR_CRC && sends==0u && current_sequence()==0u);
    }
    complete_on_delay=0;
    TEST_CHECK(UIDelete(&id,1,0,NULL)==UI_SUBMITTED && current_sequence()==1u);
    uint8_t saved[128];memcpy(saved,inflight,inflight_length);uint16_t length=inflight_length;
    TEST_CHECK(inflight!=sent && delays==0u);
    TEST_CHECK(UIDelete(&id,2,9,NULL)==UI_SEND_BUSY);
    TEST_CHECK(inflight_length==length && memcmp(inflight,saved,length)==0 && current_sequence()==1u);
    notify_result(USART_TX_FAILED);
    TEST_CHECK(current_sequence()==1u); // 入队后的失败不回退已消费的线上序号。
    TEST_CHECK(RefereeSubmit(&request,&ticket)==REFEREE_SEND_ACCEPTED);
    memset(bytes,0xff,sizeof(bytes));TEST_CHECK(inflight[13]==1u && inflight[14]==2u && inflight[15]==3u);
    notify_result(USART_TX_COMPLETE);
}
static void advance_ui(void)
{ now_ms+=115u;publish_robot_state(1u); }
static UIState_t ui_state(void)
{ UIState_t result;TEST_CHECK(UIGetState(&result)==UI_OK);return result; }
static void initialize_ui(void)
{
    TEST_CHECK(MyUIInit()==UI_SUBMITTED);
    for(unsigned i=0u;i<15u;++i) {
        advance_ui();unsigned previous=sends;
        TEST_CHECK(UIRefresh()==UI_SUBMITTED && sends==previous+1u);
        TEST_CHECK(ui_state().phase==UI_PHASE_INITIALIZING);
        TEST_CHECK(UIRefresh()==UI_OK);
    }
    TEST_CHECK(ui_state().phase==UI_PHASE_RUNNING && ui_state().init_step==15u);
}
static void task_tests(void)
{
    Referee_Interactive_info_t data={0};UART_HandleTypeDef h={0};
    TEST_CHECK(UITask()==UI_NOT_INITIALIZED && MyUIInit()==UI_NOT_INITIALIZED);
    TEST_CHECK(UITaskInit(NULL)==UI_ERROR_ARGUMENT);
    TEST_CHECK(RefereeInit(&h,NULL)==REFEREE_OK && UITaskInit(&data)==UI_OK);
    TEST_CHECK(MyUIInit()==UI_SUBMITTED && UIRefresh()==UI_OK && sends==0u);
    TEST_CHECK(ui_state().phase==UI_PHASE_WAIT_ID);
    now_ms=10u;publish_robot_state(1u);start_status=USART_SEND_ERROR;
    TEST_CHECK(UIRefresh()==UI_SEND_ERROR && ui_state().phase==UI_PHASE_ERROR && sends==0u);
    // UI失败不撤销接收，仍可发布独立的比赛结果。
    uint8_t result=2u;feed_referee(ID_game_result,&result,1u);
    RefereeSnapshot_t snapshot;TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK &&
        snapshot.command[REF_CMD_GAME_RESULT].data.game_result.winner==2u);
    start_status=USART_SEND_ACCEPTED;initialize_ui();
    for(unsigned i=0u;i<6u;++i) { advance_ui();TEST_CHECK(UIRefresh()==UI_SUBMITTED);TEST_CHECK(UIRefresh()==UI_OK); }
    data.chassis_mode=CHASSIS_ROTATE;advance_ui();start_status=USART_SEND_BUSY;
    unsigned previous=sends;uint8_t seq=current_sequence();
    TEST_CHECK(UIRefresh()==UI_SEND_BUSY && sends==previous && current_sequence()==seq && data.Referee_Interactive_Flag.chassis_flag);
    start_status=USART_SEND_ACCEPTED;complete_on_delay=0;
    TEST_CHECK(UIRefresh()==UI_SUBMITTED);TEST_CHECK(data.Referee_Interactive_Flag.chassis_flag);
    data.chassis_mode=CHASSIS_NO_FOLLOW;TEST_CHECK(UIRefresh()==UI_OK);
    notify_result(USART_TX_COMPLETE);TEST_CHECK(UIRefresh()==UI_OK && data.Referee_Interactive_Flag.chassis_flag);
    complete_on_delay=1;advance_ui();TEST_CHECK(UIRefresh()==UI_SUBMITTED);TEST_CHECK(UIRefresh()==UI_OK);
    TEST_CHECK(!data.Referee_Interactive_Flag.chassis_flag);
    data.Chassis_Power_Data.chassis_power_mx=NAN;advance_ui();previous=sends;
    TEST_CHECK(UIRefresh()==UI_ERROR_RANGE && sends==previous && data.Referee_Interactive_Flag.Power_flag);
    data.Chassis_Power_Data.chassis_power_mx=3.5f;initialize_ui();
    now_ms+=301u;TEST_CHECK(UIRefresh()==UI_NOT_INITIALIZED && ui_state().phase==UI_PHASE_NEED_INIT);
    publish_robot_state(2u);previous=sends;TEST_CHECK(UIRefresh()==UI_NOT_INITIALIZED && sends==previous);
    TEST_CHECK(MyUIInit()==UI_SUBMITTED);now_ms+=115u;publish_robot_state(2u);
    TEST_CHECK(UIRefresh()==UI_SUBMITTED && sent[9]==2u && sent[10]==0u);
    TEST_CHECK(delays==0u);
}
static void initialization_failure(void)
{
    Referee_Interactive_info_t data={0};UART_HandleTypeDef h={0};registration_fail=1;
    TEST_CHECK(RefereeInit(&h,NULL)==REFEREE_ERROR_RESOURCE && unregisters==1u);
    TEST_CHECK(UITaskInit(&data)==UI_OK && MyUIInit()==UI_SUBMITTED);
    TEST_CHECK(UIRefresh()==UI_OK && ui_state().phase==UI_PHASE_WAIT_ID && sends==0u);
    registration_fail=0;TEST_CHECK(RefereeInit(&h,NULL)==REFEREE_OK);
    initialize_ui();TEST_CHECK(RefereeInit(&h,NULL)==REFEREE_ERROR_ALREADY_INITIALIZED);
}

int main(int argc,char **argv)
{
    TEST_CHECK(argc==2);
    if(strcmp(argv[1],"drawing")==0)drawing();
    else if(strcmp(argv[1],"text")==0)text_tests();
    else if(strcmp(argv[1],"frames")==0)frames();
    else if(strcmp(argv[1],"transport")==0)transport();
    else if(strcmp(argv[1],"task")==0)task_tests();
    else if(strcmp(argv[1],"init-failure")==0)initialization_failure();
    else TEST_CHECK(0);
    return 0;
}
