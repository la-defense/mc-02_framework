/** @file referee_ui_test.c
 * @brief 通过生产构图、发送和任务入口验证边界、协议字节、失败及在途缓冲寿命。
 * @note 参数选择由CTest传入；仅替代硬件、时钟和选定的一次格式化/CRC失败。
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

uint32_t HAL_GetTick(void) { return now_ms; }
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *c) { (void)c; return &daemon_instance; }
uint8_t DaemonUnregister(DaemonInstance *d) { (void)d; ++unregisters; return 1; }
void DaemonReload(DaemonInstance *d) { (void)d; ++reloads; }
uint8_t DaemonIsOnline(DaemonInstance *d) { (void)d; return 1; }
USARTInstance *USARTRegister(USART_Init_Config_s *c)
{ receive_callback=c->module_callback;receive_context=c->context; return registration_fail?NULL:&instance; }
uint8_t USARTServiceInit(USARTInstance *i) { (void)i; return 1u; }
uint8_t USARTIsReady(USARTInstance *i) { TEST_CHECK(i==&instance); return (uint8_t)ready; }
USARTSendStatus_t USARTSendChecked(USARTInstance *i,const uint8_t *data,uint16_t length,int mode)
{
    TEST_CHECK(i==&instance && mode==USART_TRANSFER_DMA && length<=sizeof(sent));
    if(start_status!=USART_SEND_ACCEPTED) return start_status;
    memcpy(sent,data,length); sent_length=length;
    inflight=data; inflight_length=length; ++sends; ready=0;
    return USART_SEND_ACCEPTED;
}
void osDelay(uint32_t milliseconds)
{
    TEST_CHECK(milliseconds==115u); ++delays;
    if(complete_on_delay) ready=1;
}
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

static const referee_id_t id={.Robot_ID=1u,.Cilent_ID=0x101u};
static uint16_t reference_crc(const uint8_t *p,size_t n,uint16_t poly,uint16_t seed)
{
    for(size_t i=0;i<n;++i) {
        seed^=p[i];
        for(unsigned bit=0;bit<8;++bit) seed=(uint16_t)((seed>>1)^((seed&1u)?poly:0u));
    }
    return seed;
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
    TEST_CHECK(UIGraphRefresh(&id,graphs,7,1)==UI_NOT_INITIALIZED && sends==0);
    UART_HandleTypeDef handle={0};TEST_CHECK(RefereeInit(&handle)!=NULL);
    UI_Seq=0;
    TEST_CHECK(UIGraphRefresh(&id,graphs,7,1)==UI_SUBMITTED);
    TEST_CHECK(sent_length==sizeof(golden) && memcmp(sent,golden,sizeof(golden))==0 && UI_Seq==1);
    const size_t counts[]={1,2,5,7};
    for(size_t i=0;i<4;++i) {
        uint8_t sequence=UI_Seq;
        TEST_CHECK(UIGraphRefresh(&id,graphs,7,counts[i])==UI_SUBMITTED);
        check_frame((uint16_t)(0x101+i),counts[i]*15u,sequence);
    }
    TEST_CHECK(UILineDraw(&graphs[0],"abc",0,0,0,0,0,0,0,0)==UI_OK);
    TEST_CHECK(UIRectangleDraw(&graphs[1],"abc",1,0,0,1,0,0,1920,1080)==UI_OK);
    TEST_CHECK(UICircleDraw(&graphs[2],"abc",1,0,0,1,0,0,1023)==UI_OK);
    TEST_CHECK(UIOvalDraw(&graphs[3],"abc",1,0,0,1,0,0,2047,2047)==UI_OK);
    TEST_CHECK(UIArcDraw(&graphs[4],"abc",1,0,0,360,0,1,0,0,2047,2047)==UI_OK);
    TEST_CHECK(UIFloatDraw(&graphs[5],"abc",1,0,0,15,2,1,0,0,-1234)==UI_OK);
    TEST_CHECK(UIIntDraw(&graphs[6],"abc",1,0,0,15,1,0,0,INT32_MIN)==UI_OK);
    TEST_CHECK(UIGraphRefresh(&id,graphs,7,7)==UI_SUBMITTED);
    const uint8_t negative_float[]={0x2e,0xfb,0xff,0xff}, min_integer[]={0,0,0,0x80};
    TEST_CHECK(memcmp(sent+13+5*15+11,negative_float,4)==0);
    TEST_CHECK(memcmp(sent+13+6*15+11,min_integer,4)==0);
    unsigned previous=sends;uint8_t sequence=UI_Seq;
    size_t bad[]={0,3,4,6,8,256,SIZE_MAX};
    for(size_t i=0;i<7;++i) TEST_CHECK(UIGraphRefresh(&id,graphs,7,bad[i])==UI_ERROR_LENGTH);
    TEST_CHECK(UIGraphRefresh(&id,graphs,1,2)==UI_ERROR_LENGTH);
    TEST_CHECK(UIGraphRefresh(&id,NULL,7,1)==UI_ERROR_ARGUMENT);
    TEST_CHECK(UIGraphRefresh(NULL,graphs,7,1)==UI_ERROR_ARGUMENT);
    graphs[1].color=9;
    TEST_CHECK(UIGraphRefresh(&id,graphs,7,2)==UI_ERROR_RANGE);
    graphs[1]=graphs[0];
    TEST_CHECK(sends==previous && sequence==UI_Seq);
    String_Data_t text;
    TEST_CHECK(UICharDraw(&text,"txt",1,0,0,1,1,0,0,"123456789012345678901234567890")==UI_OK);
    TEST_CHECK(UICharRefresh(&id,&text)==UI_SUBMITTED);
    check_frame(0x110,45,sequence);
    TEST_CHECK(memcmp(sent+28,text.show_Data,30)==0);
    text.Graph_Control.end_angle=31;
    TEST_CHECK(UICharRefresh(&id,&text)==UI_ERROR_RANGE);
    TEST_CHECK(UICharRefresh(&id,NULL)==UI_ERROR_ARGUMENT);
    TEST_CHECK(UIGraphRefresh(&id,&text.Graph_Control,1,1)==UI_ERROR_RANGE);
    TEST_CHECK(UIDelete(&id,3,0)==UI_ERROR_RANGE && UIDelete(&id,1,10)==UI_ERROR_RANGE);
    TEST_CHECK(UIDelete(NULL,1,0)==UI_ERROR_ARGUMENT);
    referee_id_t wrong=id;wrong.Cilent_ID=0x107;
    TEST_CHECK(UIDelete(&wrong,1,0)==UI_ERROR_RANGE);
    wrong.Cilent_ID=0x165;
    TEST_CHECK(UIDelete(&wrong,1,0)==UI_ERROR_RANGE);
    wrong.Robot_ID=101;
    TEST_CHECK(UIDelete(&wrong,2,9)==UI_SUBMITTED);
    wrong.Robot_ID=109;
    TEST_CHECK(UIDelete(&wrong,0,0)==UI_SUBMITTED);
    UI_Seq=255;
    TEST_CHECK(UIDelete(&id,1,0)==UI_SUBMITTED && UI_Seq==0);
    check_frame(0x100,2,255);
}

static void transport(void)
{
    uint8_t bytes[128]={1,2,3};
    TEST_CHECK(RefereeSend(bytes,3)==REFEREE_SEND_NOT_INITIALIZED);
    TEST_CHECK(RefereeSend(NULL,3)==REFEREE_SEND_BAD_ARGUMENT);
    TEST_CHECK(RefereeSend(bytes,0)==REFEREE_SEND_BAD_ARGUMENT);
    TEST_CHECK(RefereeSend(bytes,129)==REFEREE_SEND_BAD_ARGUMENT);
    UART_HandleTypeDef h={0};TEST_CHECK(RefereeInit(&h)!=NULL);
    USARTSendStatus_t statuses[]={USART_SEND_BUSY,USART_SEND_ERROR,USART_SEND_TIMEOUT,USART_SEND_BAD_ARGUMENT};
    UIStatus_t expected[]={UI_SEND_BUSY,UI_SEND_ERROR,UI_SEND_TIMEOUT,UI_SEND_ERROR};
    for(size_t i=0;i<4;++i) {
        start_status=statuses[i];unsigned previous=sends;
        TEST_CHECK(UIDelete(&id,1,0)==expected[i]);
        TEST_CHECK(sends==previous && UI_Seq==0 && delays==0);
    }
    start_status=USART_SEND_ACCEPTED;
    for(int failure=0;failure<2;++failure) {
        crc_fail_after=failure;
        TEST_CHECK(UIDelete(&id,1,0)==UI_ERROR_CRC && sends==0 && UI_Seq==0);
    }
    complete_on_delay=0;
    TEST_CHECK(UIDelete(&id,1,0)==UI_SUBMITTED && UI_Seq==1);
    uint8_t saved[128];memcpy(saved,inflight,inflight_length);uint16_t length=inflight_length;
    TEST_CHECK(inflight!=sent && delays==1);
    TEST_CHECK(UIDelete(&id,2,9)==UI_SEND_BUSY);
    TEST_CHECK(inflight_length==length && memcmp(inflight,saved,length)==0 && UI_Seq==1 && sends==1);
    // 固定延时已返回而HAL仍忙，旧帧不得被覆盖；显式模拟完成后才接受下一帧。
    ready=1;complete_on_delay=1;
    TEST_CHECK(RefereeSend(bytes,sizeof(bytes))==REFEREE_SEND_ACCEPTED);
    memset(bytes,0xff,sizeof(bytes));TEST_CHECK(inflight[0]==1 && inflight[1]==2 && inflight[2]==3);
}

static void task_tests(void)
{
    Referee_Interactive_info_t data={0};UART_HandleTypeDef h={0};
    TEST_CHECK(UITask()==UI_NOT_INITIALIZED && MyUIInit()==UI_NOT_INITIALIZED);
    TEST_CHECK(UITaskInit(&h,NULL)==NULL);
    TEST_CHECK(UITaskInit(NULL,&data)==NULL);
    referee_info_t *info=UITaskInit(&h,&data);TEST_CHECK(info!=NULL);
    TEST_CHECK(MyUIInit()==UI_NOT_INITIALIZED);
    info->GameRobotState.robot_id=1;
    start_status=USART_SEND_ERROR;
    TEST_CHECK(MyUIInit()==UI_SEND_ERROR && UITask()==UI_NOT_INITIALIZED && sends==0);
    TEST_CHECK(RefereeGetInfo()==info && info->init_flag);
    // 初始化失败后仍通过生产接收回调接受合法帧，接收租约正常更新。
    uint8_t frame[]={0xa5,1,0,7,0,2,0,5,0,0};
    frame[4]=(uint8_t)reference_crc(frame,4,0x8c,0xff);
    uint16_t tail=reference_crc(frame,8,0x8408,0xffff);frame[8]=(uint8_t)tail;frame[9]=(uint8_t)(tail>>8);
    memcpy(rx,frame,sizeof(frame));now_ms=17;
    const USART_RxBlock_t block={rx,sizeof(frame),now_ms,1u};
    receive_callback(receive_context,&block);
    TEST_CHECK(info->GameResult.winner==5 && RefereeGetLastRxMs()==17 && reloads==1);
    start_status=USART_SEND_ACCEPTED;
    crc_fail_after=3;unsigned partial=sends;
    TEST_CHECK(MyUIInit()==UI_ERROR_CRC && sends==partial+1 && UITask()==UI_NOT_INITIALIZED);
    TEST_CHECK(MyUIInit()==UI_SUBMITTED);
    TEST_CHECK(UIRefresh()==UI_OK);
    data.chassis_mode=CHASSIS_ROTATE;
    start_status=USART_SEND_BUSY;unsigned previous=sends;uint8_t seq=UI_Seq;
    TEST_CHECK(UIRefresh()==UI_SEND_BUSY && data.Referee_Interactive_Flag.chassis_flag);
    TEST_CHECK(sends==previous && UI_Seq==seq);
    start_status=USART_SEND_ACCEPTED;
    TEST_CHECK(UIRefresh()==UI_OK && !data.Referee_Interactive_Flag.chassis_flag && sends==previous+1);
    data.chassis_mode=(chassis_mode_e)255;previous=sends;
    TEST_CHECK(UIRefresh()==UI_ERROR_RANGE && data.Referee_Interactive_Flag.chassis_flag && sends==previous);
    data.chassis_mode=CHASSIS_ROTATE;TEST_CHECK(UIRefresh()==UI_OK);
    data.Chassis_Power_Data.chassis_power_mx=NAN;previous=sends;
    TEST_CHECK(UIRefresh()==UI_ERROR_RANGE && data.Referee_Interactive_Flag.Power_flag && sends==previous);
    data.Chassis_Power_Data.chassis_power_mx=1e30f;
    TEST_CHECK(UIRefresh()==UI_ERROR_RANGE && sends==previous);
    data.Chassis_Power_Data.chassis_power_mx=3.5f;
    TEST_CHECK(UIRefresh()==UI_OK && !data.Referee_Interactive_Flag.Power_flag && sends==previous+1);
    TEST_CHECK(UITask()==UI_OK && data.Chassis_Power_Data.chassis_power_mx==7.0f);
    start_status=USART_SEND_ERROR;
    TEST_CHECK(MyUIInit()==UI_SEND_ERROR);
    previous=sends;
    TEST_CHECK(UITask()==UI_NOT_INITIALIZED && sends==previous);
}

static void initialization_failure(void)
{
    Referee_Interactive_info_t data={0};UART_HandleTypeDef h={0};
    registration_fail=1;
    TEST_CHECK(UITaskInit(&h,&data)==NULL && RefereeGetInfo()==NULL && unregisters==1);
    TEST_CHECK(MyUIInit()==UI_NOT_INITIALIZED && UIRefresh()==UI_NOT_INITIALIZED);
    registration_fail=0;
    referee_info_t *info=UITaskInit(&h,&data);TEST_CHECK(info!=NULL);
    info->GameRobotState.robot_id=1;
    TEST_CHECK(MyUIInit()==UI_SUBMITTED);
    TEST_CHECK(UITaskInit(&h,&data)==NULL && RefereeGetInfo()==info);
    TEST_CHECK(UIRefresh()==UI_OK);
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
