/**
 * @file referee_snapshot_test.c
 * @brief 通过真实UART队列、CRC和裁判解析器验证命令解码及一致快照。
 * 主机只替代HAL、时间、daemon和临界区；Linux额外用真实线程交错读取。
 * @author kidneygood (you@domain.com)
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include "bsp_usart.h"
#include "crc.h"
#include "daemon.h"
#include "referee_protocol.h"
#include "rm_referee.h"
#include "test_check.h"
#if defined(REFEREE_TEST_THREADS)
#include <pthread.h>
pthread_mutex_t referee_test_critical_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

static UART_HandleTypeDef handle;
static DMA_HandleTypeDef dma;
static uint8_t *dma_bytes;
static uint16_t dma_capacity;
static Daemon_Init_Config_s daemon_config;
static DaemonInstance daemon_instance;
#if defined(REFEREE_TEST_THREADS)
static _Atomic uint32_t now_ms;
static _Atomic unsigned reload_count;
static _Atomic unsigned reader_failures;
static _Atomic uint8_t producer_done;
#else
static uint32_t now_ms;
static unsigned reload_count;
#endif

uint32_t HAL_GetTick(void)
{
#if defined(REFEREE_TEST_THREADS)
    return atomic_load_explicit(&now_ms, memory_order_relaxed);
#else
    return now_ms;
#endif
}
static void set_tick(uint32_t value)
{
#if defined(REFEREE_TEST_THREADS)
    atomic_store_explicit(&now_ms,value,memory_order_relaxed);
#else
    now_ms=value;
#endif
}
DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config) { daemon_config=*config;return &daemon_instance; }
uint8_t DaemonUnregister(DaemonInstance *instance) { (void)instance;return 1u; }
void DaemonReload(DaemonInstance *instance)
{
    (void)instance;
#if defined(REFEREE_TEST_THREADS)
    atomic_fetch_add_explicit(&reload_count,1u,memory_order_relaxed);
#else
    ++reload_count;
#endif
}
DaemonStatus_t DaemonReportAt(DaemonInstance *instance, uint32_t received_ms, uint8_t flags)
{
    (void)instance;
    (void)received_ms;
    if ((flags & DAEMON_REPORT_TRANSPORT_VALID) == 0u)
        return DAEMON_OK;
#if defined(REFEREE_TEST_THREADS)
    atomic_fetch_add_explicit(&reload_count,1u,memory_order_relaxed);
#else
    ++reload_count;
#endif
    return DAEMON_OK;
}
uint8_t DaemonIsOnline(DaemonInstance *instance) { (void)instance;return 1u; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{ (void)h;dma_bytes=p;dma_capacity=n;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h,uint8_t *p,uint16_t n,uint32_t t)
{ (void)h;(void)p;(void)n;(void)t;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{ return HAL_UART_Transmit(h,p,n,0u); }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{ return HAL_UART_Transmit(h,p,n,0u); }
void osDelay(uint32_t milliseconds) { (void)milliseconds; }

// 独立逐位算法只用于生成帧，不调用生产CRC实现计算期望值。
static uint32_t reference_crc(const uint8_t *p,size_t n,uint32_t poly,uint32_t seed)
{
    for(size_t i=0u;i<n;++i) {
        seed^=p[i];
        for(unsigned bit=0u;bit<8u;++bit) seed=(seed>>1u)^((seed&1u)?poly:0u);
    }
    return seed;
}

static void write_u32le(uint8_t *p,uint32_t value)
{ p[0]=(uint8_t)value;p[1]=(uint8_t)(value>>8u);p[2]=(uint8_t)(value>>16u);p[3]=(uint8_t)(value>>24u); }
static void write_u16le(uint8_t *p,uint16_t value)
{ p[0]=(uint8_t)value;p[1]=(uint8_t)(value>>8u); }
static void write_u64le(uint8_t *p,uint64_t value)
{ for(size_t i=0u;i<8u;++i)p[i]=(uint8_t)(value>>(8u*i)); }
static void write_float(uint8_t *p,float value)
{ uint32_t bits;memcpy(&bits,&value,sizeof(bits));write_u32le(p,bits); }

static size_t make_frame(uint8_t *frame,uint16_t command,const uint8_t *payload,size_t length,uint8_t sequence)
{
    memset(frame,0,length+9u);frame[0]=REFEREE_SOF;frame[1]=(uint8_t)length;
    frame[2]=(uint8_t)(length>>8u);frame[3]=sequence;
    frame[4]=(uint8_t)reference_crc(frame,4u,0x8cu,0xffu);
    frame[5]=(uint8_t)command;frame[6]=(uint8_t)(command>>8u);memcpy(frame+7u,payload,length);
    uint16_t tail=(uint16_t)reference_crc(frame,length+7u,0x8408u,0xffffu);
    frame[length+7u]=(uint8_t)tail;frame[length+8u]=(uint8_t)(tail>>8u);return length+9u;
}

static void feed(const uint8_t *bytes,size_t length)
{
    TEST_CHECK(length<=dma_capacity);
    memcpy(dma_bytes,bytes,length);
    HAL_UARTEx_RxEventCallback(&handle,(uint16_t)length);
    USARTProcessRx();
}
static void feed_frame(uint16_t command,const uint8_t *payload,size_t length,uint8_t sequence)
{
    uint8_t frame[128];size_t n=make_frame(frame,command,payload,length,sequence);feed(frame,n);
}
static void make_valid_payload(uint16_t command,uint8_t *payload,size_t length)
{
    memset(payload,0,length);
    if(command==ID_game_result) payload[0]=2u;
    if(command==ID_game_robot_state) { payload[0]=1u;write_float(payload+12u,0.0f); }
    if(command==ID_game_robot_pos) { write_float(payload,1.25f);write_float(payload+4u,-2.5f);write_float(payload+8u,90.0f); }
    if(command==ID_shoot_data) write_float(payload+3u,3.5f);
    if(command==ID_ground_robot_position) for(size_t i=0u;i<8u;++i)write_float(payload+4u*i,(float)i);
    if(command==ID_map_command) { write_float(payload,2.0f);write_float(payload+4u,3.0f);payload[9]=1u; }
    if(command==ID_power_heat_data) memset(payload,0xff,8u);
    if(command==ID_ground_robot_position) memset(payload+32u,0xff,8u);
    if(command==ID_student_interactive) {
        payload[0]=0x00u;payload[1]=0x02u;payload[2]=3u;payload[4]=1u;
    }
}

static const struct { uint16_t id; uint8_t length; RefereeCommandIndex_t index; } commands[] = {
    {ID_game_state,11u,REF_CMD_GAME_STATE},
    {ID_game_result,1u,REF_CMD_GAME_RESULT},
    {ID_game_robot_survivors,20u,REF_CMD_ROBOT_HP},
    {ID_event_data,4u,REF_CMD_EVENT_DATA},
    {ID_referee_warning,3u,REF_CMD_WARNING},
    {ID_dart_shoot_data,3u,REF_CMD_DART_INFO},
    {ID_game_robot_state,17u,REF_CMD_ROBOT_STATE},
    {ID_power_heat_data,14u,REF_CMD_POWER_HEAT},
    {ID_game_robot_pos,12u,REF_CMD_ROBOT_POSITION},
    {ID_buff_musk,8u,REF_CMD_BUFF},
    {ID_robot_hurt,1u,REF_CMD_ROBOT_HURT},
    {ID_shoot_data,7u,REF_CMD_SHOOT_DATA},
    {ID_projectile_allowance,8u,REF_CMD_PROJECTILE_ALLOWANCE},
    {ID_rfid_status,5u,REF_CMD_RFID},
    {ID_dart_client_cmd,6u,REF_CMD_DART_CLIENT},
    {ID_ground_robot_position,40u,REF_CMD_GROUND_POSITION},
    {ID_radar_mark_data,2u,REF_CMD_RADAR_MARK},
    {ID_sentry_info,14u,REF_CMD_SENTRY_INFO},
    {ID_radar_info,1u,REF_CMD_RADAR_INFO},
    {ID_map_command,12u,REF_CMD_MAP_COMMAND},
    {ID_student_interactive,11u,REF_CMD_INTERACTIVE}
};

static void check_protocol_and_lengths(void)
{
    RefereeSnapshot_t snapshot;uint8_t payload[120],frame[128];uint8_t sequence=0u;
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK && !snapshot.link_online);
    for(size_t i=0u;i<sizeof(commands)/sizeof(commands[0]);++i) {
        make_valid_payload(commands[i].id,payload,commands[i].length);
        feed_frame(commands[i].id,payload,commands[i].length,sequence++);
        TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK);
        TEST_CHECK(snapshot.command[commands[i].index].receive_count==1u);
        TEST_CHECK(snapshot.command[commands[i].index].source_rx_ms==HAL_GetTick());
        size_t n=make_frame(frame,commands[i].id,payload,commands[i].length-1u,sequence++);
        feed(frame,n);
        TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK && snapshot.command[commands[i].index].receive_count==1u);
        n=make_frame(frame,commands[i].id,payload,commands[i].length+1u,sequence++);
        feed(frame,n);
        TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK && snapshot.command[commands[i].index].receive_count==1u);
    }
    // 官方详细表和示例结构都给出12字节；总表的16/15字节变体在严格长度检查中拒绝。
    make_valid_payload(ID_game_robot_pos,payload,16u);feed_frame(ID_game_robot_pos,payload,16u,sequence++);
    make_valid_payload(ID_map_command,payload,15u);feed_frame(ID_map_command,payload,15u,sequence++);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK);
    TEST_CHECK(snapshot.command[REF_CMD_ROBOT_POSITION].receive_count==1u &&
               snapshot.command[REF_CMD_MAP_COMMAND].receive_count==1u);
    // 线上固定向量由协议字段及CRC逐位参考核对。
    const uint8_t result_payload[]={2u};
    size_t n=make_frame(frame,ID_game_result,result_payload,sizeof(result_payload),0u);
    const uint8_t fixed[]={0xa5,1,0,0,0x68,2,0,2,0xbc,0xe3};
    TEST_CHECK(n==sizeof(fixed) && memcmp(frame,fixed,sizeof(fixed))==0);
    // 保留字段可呈现任意位形，包括浮点NaN的字节，但不会被解释成测量值。
    TEST_CHECK(snapshot.command[REF_CMD_POWER_HEAT].data.power_heat.reserved[0]==0xffu);
    TEST_CHECK(snapshot.command[REF_CMD_GROUND_POSITION].data.ground_position.reserved[0]==0xffu);
    TEST_CHECK(fabsf(snapshot.command[REF_CMD_ROBOT_POSITION].data.robot_position.x_m-1.25f)<1e-6f);
    TEST_CHECK(snapshot.command[REF_CMD_SHOOT_DATA].data.shoot_data.initial_speed_mps==3.5f);
    TEST_CHECK(snapshot.command[REF_CMD_INTERACTIVE].state==REFEREE_DATA_EVENT_RECORDED);
}

static void check_fixed_field_decoding(void)
{
    RefereeSnapshot_t snapshot;uint8_t payload[LEN_ground_robot_position];
    memset(payload,0,LEN_game_state);payload[0]=0xa3u;write_u16le(payload+1u,0x1234u);
    write_u64le(payload+3u,UINT64_C(0x0123456789abcdef));
    feed_frame(ID_game_state,payload,LEN_game_state,100u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK);
    TEST_CHECK(snapshot.command[REF_CMD_GAME_STATE].data.game_state.game_type==3u &&
               snapshot.command[REF_CMD_GAME_STATE].data.game_state.game_progress==10u &&
               snapshot.command[REF_CMD_GAME_STATE].data.game_state.stage_remain_s==0x1234u &&
               snapshot.command[REF_CMD_GAME_STATE].data.game_state.sync_timestamp==UINT64_C(0x0123456789abcdef));

    memset(payload,0,LEN_game_robot_HP);
    for(size_t i=0u;i<4u;++i)write_u16le(payload+2u*i,(uint16_t)(100u+i));
    write_u16le(payload+8u,(uint16_t)(int16_t)-1234);
    for(size_t i=0u;i<5u;++i)write_u16le(payload+10u+2u*i,(uint16_t)(200u+i));
    feed_frame(ID_game_robot_survivors,payload,LEN_game_robot_HP,101u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK);
    TEST_CHECK(snapshot.command[REF_CMD_ROBOT_HP].data.robot_hp.hp[0]==100u &&
               snapshot.command[REF_CMD_ROBOT_HP].data.robot_hp.hp[8]==204u &&
               snapshot.command[REF_CMD_ROBOT_HP].data.robot_hp.damage_difference==-1234);

    write_u32le(payload,UINT32_C(0xa5123456));
    feed_frame(ID_event_data,payload,LEN_event_data,102u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK &&
               snapshot.command[REF_CMD_EVENT_DATA].data.event_data.event_bits==UINT32_C(0xa5123456));

    memset(payload,0,LEN_dart_shoot_data);payload[0]=9u;write_u16le(payload+1u,0x02d5u);
    feed_frame(ID_dart_shoot_data,payload,LEN_dart_shoot_data,103u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK &&
               snapshot.command[REF_CMD_DART_INFO].data.dart_info.info_bits==0x02d5u);

    memset(payload,0,LEN_game_robot_state);payload[0]=1u;payload[1]=3u;
    write_u16le(payload+2u,456u);write_u16le(payload+4u,600u);
    write_u16le(payload+6u,40u);write_u16le(payload+8u,200u);write_u16le(payload+10u,80u);
    write_float(payload+12u,17.25f);payload[16]=0x05u;
    feed_frame(ID_game_robot_state,payload,LEN_game_robot_state,104u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK);
    TEST_CHECK(snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.current_hp==456u &&
               snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.chassis_power_limit_w==80u &&
               snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.bullet_speed_limit_mps==17.25f &&
               snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.power_output_bits==0x05u);

    memset(payload,0,LEN_power_heat_data);for(size_t i=0u;i<8u;++i)payload[i]=(uint8_t)(0xf0u+i);
    write_u16le(payload+8u,0x1234u);write_u16le(payload+10u,0x2345u);write_u16le(payload+12u,0x3456u);
    feed_frame(ID_power_heat_data,payload,LEN_power_heat_data,105u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK);
    TEST_CHECK(snapshot.command[REF_CMD_POWER_HEAT].data.power_heat.reserved[7]==0xf7u &&
               snapshot.command[REF_CMD_POWER_HEAT].data.power_heat.buffer_energy_j==0x1234u &&
               snapshot.command[REF_CMD_POWER_HEAT].data.power_heat.heat_42mm==0x3456u);

    // 固定帧字节由协议字段和独立逐位CRC核对，避免测试与组帧辅助函数共用期望值。
    const uint8_t position_frame[]={0xa5,0x0c,0x00,0x2a,0x25,0x03,0x02,0x00,0x00,
                                    0xa0,0x3f,0x00,0x00,0x20,0xc0,0x00,0x00,0xb4,0x42,0x18,0x84};
    feed(position_frame,sizeof(position_frame));
    memset(payload,0,LEN_buff_musk);payload[0]=10u;write_u16le(payload+1u,0x1234u);
    payload[3]=50u;payload[4]=30u;write_u16le(payload+5u,0x2345u);payload[7]=0x55u;
    feed_frame(ID_buff_musk,payload,LEN_buff_musk,107u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK);
    TEST_CHECK(snapshot.command[REF_CMD_ROBOT_POSITION].data.robot_position.x_m==1.25f &&
               snapshot.command[REF_CMD_ROBOT_POSITION].data.robot_position.y_m==-2.5f &&
               snapshot.command[REF_CMD_BUFF].data.buff.cooling_buff==0x1234u &&
               snapshot.command[REF_CMD_BUFF].data.buff.remaining_energy_bits==0x55u);

    memset(payload,0,LEN_robot_hurt);payload[0]=0x51u;
    feed_frame(ID_robot_hurt,payload,LEN_robot_hurt,108u);
    memset(payload,0,LEN_shoot_data);payload[0]=0x06u;payload[1]=3u;payload[2]=9u;write_float(payload+3u,4.5f);
    feed_frame(ID_shoot_data,payload,LEN_shoot_data,109u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK);
    TEST_CHECK(snapshot.command[REF_CMD_ROBOT_HURT].data.robot_hurt.armor_id==1u &&
               snapshot.command[REF_CMD_ROBOT_HURT].data.robot_hurt.hurt_type==5u &&
               snapshot.command[REF_CMD_SHOOT_DATA].data.shoot_data.shooter_number==3u &&
               snapshot.command[REF_CMD_SHOOT_DATA].data.shoot_data.frequency_hz==9u &&
               snapshot.command[REF_CMD_SHOOT_DATA].data.shoot_data.initial_speed_mps==4.5f);

    write_u16le(payload,0x1234u);write_u16le(payload+2u,0x2345u);
    write_u16le(payload+4u,0x3456u);write_u16le(payload+6u,0x4567u);
    feed_frame(ID_projectile_allowance,payload,LEN_projectile_allowance,110u);
    write_u32le(payload,UINT32_C(0x89abcdef));payload[4]=0x35u;
    feed_frame(ID_rfid_status,payload,LEN_rfid_status,111u);
    memset(payload,0,LEN_dart_client_cmd);payload[0]=2u;write_u16le(payload+2u,0x1234u);write_u16le(payload+4u,0x5678u);
    feed_frame(ID_dart_client_cmd,payload,LEN_dart_client_cmd,112u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK);
    TEST_CHECK(snapshot.command[REF_CMD_PROJECTILE_ALLOWANCE].data.projectile_allowance.allowance_fortress==0x4567u &&
               snapshot.command[REF_CMD_RFID].data.rfid.status_bits==UINT32_C(0x89abcdef) &&
               snapshot.command[REF_CMD_RFID].data.rfid.extra_status==0x35u &&
               snapshot.command[REF_CMD_DART_CLIENT].data.dart_client.latest_launch_time==0x5678u);

    for(size_t i=0u;i<8u;++i)write_float(payload+4u*i,(float)i+0.5f);
    for(size_t i=32u;i<40u;++i)payload[i]=(uint8_t)(0xa0u+i-32u);
    feed_frame(ID_ground_robot_position,payload,LEN_ground_robot_position,113u);
    write_u16le(payload,0xa55au);feed_frame(ID_radar_mark_data,payload,LEN_radar_mark_data,114u);
    write_u32le(payload,UINT32_C(0x89abcdef));write_u16le(payload+4u,0x1357u);
    write_u64le(payload+6u,UINT64_C(0x0123456789abcdef));feed_frame(ID_sentry_info,payload,LEN_sentry_info,115u);
    payload[0]=0x35u;feed_frame(ID_radar_info,payload,LEN_radar_info,116u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK);
    TEST_CHECK(snapshot.command[REF_CMD_GROUND_POSITION].data.ground_position.position_m[7]==7.5f &&
               snapshot.command[REF_CMD_GROUND_POSITION].data.ground_position.reserved[7]==0xa7u &&
               snapshot.command[REF_CMD_RADAR_MARK].data.radar_mark.mark_bits==0xa55au &&
               snapshot.command[REF_CMD_SENTRY_INFO].data.sentry_info.info_bits_3==UINT64_C(0x0123456789abcdef) &&
               snapshot.command[REF_CMD_RADAR_INFO].data.radar_info.info_bits==0x35u);

    memset(payload,0,LEN_map_command);write_float(payload,8.5f);write_float(payload+4u,-1.25f);
    payload[8]=0x21u;payload[9]=101u;write_u16le(payload+10u,0x4321u);
    feed_frame(ID_map_command,payload,LEN_map_command,117u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK);
    TEST_CHECK(snapshot.command[REF_CMD_MAP_COMMAND].data.map_command.target_x_m==8.5f &&
               snapshot.command[REF_CMD_MAP_COMMAND].data.map_command.target_y_m==-1.25f &&
               snapshot.command[REF_CMD_MAP_COMMAND].data.map_command.keyboard_bits==0x21u &&
               snapshot.command[REF_CMD_MAP_COMMAND].data.map_command.target_robot_id==101u &&
               snapshot.command[REF_CMD_MAP_COMMAND].data.map_command.source_id==0x4321u);
}

static void check_failures_and_stream(void)
{
    RefereeSnapshot_t before,after;uint8_t payload[120],frame[128];
    TEST_CHECK(RefereeGetSnapshot(&before)==REFEREE_OK);
    uint32_t current=before.command[REF_CMD_ROBOT_POSITION].receive_count;
    make_valid_payload(ID_game_robot_pos,payload,LEN_game_robot_pos);write_u32le(payload,0x7fc00000u);
    feed_frame(ID_game_robot_pos,payload,LEN_game_robot_pos,77u);
    make_valid_payload(ID_shoot_data,payload,LEN_shoot_data);write_u32le(payload+3u,0x7f800000u);
    feed_frame(ID_shoot_data,payload,LEN_shoot_data,78u);
    make_valid_payload(ID_game_robot_state,payload,LEN_game_robot_state);payload[0]=0u;
    write_float(payload+12u,INFINITY);feed_frame(ID_game_robot_state,payload,LEN_game_robot_state,79u);
    TEST_CHECK(RefereeGetSnapshot(&after)==REFEREE_OK &&
               after.command[REF_CMD_ROBOT_POSITION].receive_count==current);
    uint32_t shot_count=after.command[REF_CMD_SHOOT_DATA].receive_count;
    TEST_CHECK(shot_count==2u && after.command[REF_CMD_ROBOT_STATE].data.robot_state.robot_id==1u);
    make_valid_payload(ID_student_interactive,payload,LEN_receive_data);payload[0]=1u;
    feed_frame(ID_student_interactive,payload,LEN_receive_data,80u);
    make_valid_payload(ID_student_interactive,payload,LEN_receive_data);payload[2]=101u;
    feed_frame(ID_student_interactive,payload,LEN_receive_data,81u);
    make_valid_payload(ID_student_interactive,payload,LEN_receive_data);payload[4]=101u;
    feed_frame(ID_student_interactive,payload,LEN_receive_data,82u);
    TEST_CHECK(RefereeGetSnapshot(&after)==REFEREE_OK && after.command[REF_CMD_INTERACTIVE].receive_count==1u);
    // 本机身份来源超过10Hz期限后，即使交互包本身正确也不发布。
    set_tick(301u);make_valid_payload(ID_student_interactive,payload,LEN_receive_data);
    feed_frame(ID_student_interactive,payload,LEN_receive_data,83u);
    TEST_CHECK(RefereeGetSnapshot(&after)==REFEREE_OK && after.command[REF_CMD_INTERACTIVE].receive_count==1u);
    set_tick(0u);
    make_valid_payload(ID_game_result,payload,1u);size_t n=make_frame(frame,ID_game_result,payload,1u,83u);
    frame[n-1u]^=1u;feed(frame,n);
    frame[n-1u]^=1u;frame[4]^=1u;feed(frame,n);
    n=make_frame(frame,0x7fffu,payload,0u,84u);feed(frame,n);
    TEST_CHECK(RefereeGetSnapshot(&after)==REFEREE_OK &&
               after.command[REF_CMD_GAME_RESULT].receive_count==1u && after.command[REF_CMD_GAME_RESULT].source_rx_ms==before.command[REF_CMD_GAME_RESULT].source_rx_ms);

    // 每个分割点都保留首块的原时间，查询不会延长这条命令的期限。
    make_valid_payload(ID_game_result,payload,1u);n=make_frame(frame,ID_game_result,payload,1u,84u);
    for(size_t split=1u;split<n;++split) {
        uint32_t first=HAL_GetTick();feed(frame,split);
        set_tick(first+1u);feed(frame+split,n-split);
        TEST_CHECK(RefereeGetSnapshot(&after)==REFEREE_OK);
        TEST_CHECK(after.command[REF_CMD_GAME_RESULT].source_rx_ms==first);
        set_tick(first+4000u);
    }
}

static void check_expiry(void)
{
    RefereeSnapshot_t snapshot;uint8_t payload[120]={0};
    set_tick(0u);
    feed_frame(ID_game_state,payload,LEN_game_state,1u);
    set_tick(3000u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK && snapshot.command[REF_CMD_GAME_STATE].state==REFEREE_DATA_FRESH);
    set_tick(3001u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK && snapshot.command[REF_CMD_GAME_STATE].state==REFEREE_DATA_EXPIRED);
    TEST_CHECK(snapshot.command[REF_CMD_GAME_STATE].age_ms==3001u && !snapshot.link_online);
    // 更新10Hz组不能让停止更新的3Hz数据继续显示为新鲜。
    make_valid_payload(ID_game_robot_state,payload,LEN_game_robot_state);
    feed_frame(ID_game_robot_state,payload,LEN_game_robot_state,2u);
    set_tick(2000u);
    memset(payload,0,LEN_game_robot_HP);feed_frame(ID_game_robot_survivors,payload,LEN_game_robot_HP,3u);
    set_tick(3000u);
    make_valid_payload(ID_game_robot_state,payload,LEN_game_robot_state);
    feed_frame(ID_game_robot_state,payload,LEN_game_robot_state,4u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK && snapshot.command[REF_CMD_ROBOT_HP].state==REFEREE_DATA_FRESH);
    set_tick(3001u);
    make_valid_payload(ID_game_robot_state,payload,LEN_game_robot_state);
    feed_frame(ID_game_robot_state,payload,LEN_game_robot_state,5u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK && snapshot.command[REF_CMD_ROBOT_HP].state==REFEREE_DATA_EXPIRED);
    // 10Hz数据到300ms边界仍可用，超过1ms后过期；其它命令不能替它续期。
    set_tick(10000u);make_valid_payload(ID_power_heat_data,payload,LEN_power_heat_data);
    feed_frame(ID_power_heat_data,payload,LEN_power_heat_data,6u);
    set_tick(10300u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK && snapshot.command[REF_CMD_POWER_HEAT].state==REFEREE_DATA_FRESH);
    set_tick(10301u);make_valid_payload(ID_game_robot_state,payload,LEN_game_robot_state);
    feed_frame(ID_game_robot_state,payload,LEN_game_robot_state,7u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK && snapshot.command[REF_CMD_POWER_HEAT].state==REFEREE_DATA_EXPIRED &&
               snapshot.command[REF_CMD_POWER_HEAT].age_ms==301u);
    set_tick(10302u);make_valid_payload(ID_power_heat_data,payload,LEN_power_heat_data);
    feed_frame(ID_power_heat_data,payload,LEN_power_heat_data,70u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK);
    TEST_CHECK(snapshot.command[REF_CMD_POWER_HEAT].state==REFEREE_DATA_FRESH);
    TEST_CHECK(snapshot.command[REF_CMD_POWER_HEAT].receive_count==4u);
    // 过期3Hz血量记录只由同一命令的新合法帧恢复。
    set_tick(12002u);make_valid_payload(ID_game_robot_survivors,payload,LEN_game_robot_HP);
    feed_frame(ID_game_robot_survivors,payload,LEN_game_robot_HP,71u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK &&
               snapshot.command[REF_CMD_ROBOT_HP].state==REFEREE_DATA_FRESH &&
               snapshot.command[REF_CMD_ROBOT_HP].receive_count==4u);
    // 0x0303重复包只更新最近内容记录，不生成持续有效的用户操作状态。
    set_tick(20000u);make_valid_payload(ID_map_command,payload,LEN_map_command);
    feed_frame(ID_map_command,payload,LEN_map_command,8u);set_tick(20100u);
    feed_frame(ID_map_command,payload,LEN_map_command,9u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK &&
               snapshot.command[REF_CMD_MAP_COMMAND].state==REFEREE_DATA_FRESH &&
               snapshot.command[REF_CMD_MAP_COMMAND].receive_count==4u &&
               snapshot.command[REF_CMD_MAP_COMMAND].source_rx_ms==20100u);
    set_tick(23100u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK && snapshot.command[REF_CMD_MAP_COMMAND].state==REFEREE_DATA_FRESH);
    set_tick(23101u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK && snapshot.command[REF_CMD_MAP_COMMAND].state==REFEREE_DATA_EXPIRED);
    // 事件只保留最近一条及年龄，时间再久也不变成周期数据。
    uint8_t winner=2u;set_tick(24000u);feed_frame(ID_game_result,&winner,1u,10u);set_tick(90000u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK &&
               snapshot.command[REF_CMD_GAME_RESULT].state==REFEREE_DATA_EVENT_RECORDED &&
               snapshot.command[REF_CMD_GAME_RESULT].age_ms==66000u);
    // 单次计时回绕下仍按无符号差值判断期限，0毫秒也保留为合法来源时间。
    set_tick(UINT32_MAX-100u);
    memset(payload,0,LEN_game_state);feed_frame(ID_game_state,payload,LEN_game_state,11u);
    set_tick(2899u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK && snapshot.command[REF_CMD_GAME_STATE].state==REFEREE_DATA_FRESH);
    set_tick(2900u);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK && snapshot.command[REF_CMD_GAME_STATE].state==REFEREE_DATA_EXPIRED);
    const DaemonEvent_t offline_event = {.state = DAEMON_SOURCE_OFFLINE};
    daemon_config.event_handler(&offline_event, daemon_config.owner_id);
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK && !snapshot.link_online &&
               snapshot.command[REF_CMD_GAME_STATE].state==REFEREE_DATA_EXPIRED);
}

#if defined(REFEREE_TEST_THREADS)
static void *writer_thread(void *argument)
{
    (void)argument;uint8_t payload[LEN_game_state]={0};uint8_t frame[32];
    for(uint32_t i=1u;i<=20000u;++i) {
        payload[1]=(uint8_t)i;payload[2]=(uint8_t)(i>>8u);
        uint64_t timestamp=i;for(size_t b=0u;b<8u;++b)payload[3u+b]=(uint8_t)(timestamp>>(8u*b));
        size_t n=make_frame(frame,ID_game_state,payload,sizeof(payload),(uint8_t)i);feed(frame,n);
    }
    atomic_store_explicit(&producer_done,1u,memory_order_release);return NULL;
}
static void *reader_thread(void *argument)
{
    (void)argument;RefereeSnapshot_t snapshot;
    while(!atomic_load_explicit(&producer_done,memory_order_acquire)) {
        if(RefereeGetSnapshot(&snapshot)!=REFEREE_OK) { atomic_fetch_add(&reader_failures,1u);continue; }
        const RefereeCommandSnapshot_t *entry=&snapshot.command[REF_CMD_GAME_STATE];
        if(entry->receive_count!=0u && (uint16_t)entry->data.game_state.sync_timestamp!=entry->data.game_state.stage_remain_s)
            atomic_fetch_add(&reader_failures,1u);
        if(entry->receive_count!=0u && (uint8_t)entry->data.game_state.sync_timestamp!=entry->wire_sequence)
            atomic_fetch_add(&reader_failures,1u);
    }
    return NULL;
}
static void check_concurrent_snapshots(void)
{
    uint8_t initial[LEN_game_state]={0};feed_frame(ID_game_state,initial,sizeof(initial),0u);
    pthread_t writer,reader;TEST_CHECK(pthread_create(&writer,NULL,writer_thread,NULL)==0);
    TEST_CHECK(pthread_create(&reader,NULL,reader_thread,NULL)==0);
    TEST_CHECK(pthread_join(writer,NULL)==0 && pthread_join(reader,NULL)==0);
    TEST_CHECK(atomic_load(&reader_failures)==0u);
}
#else
static void check_concurrent_snapshots(void)
{
    RefereeSnapshot_t snapshot;uint8_t payload[LEN_game_state]={0};
    for(uint32_t i=1u;i<=1000u;++i) {
        payload[1]=(uint8_t)i;payload[2]=(uint8_t)(i>>8u);
        uint64_t timestamp=i;for(size_t b=0u;b<8u;++b)payload[3u+b]=(uint8_t)(timestamp>>(8u*b));
        feed_frame(ID_game_state,payload,sizeof(payload),(uint8_t)i);
        TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_OK);
        TEST_CHECK((uint16_t)snapshot.command[REF_CMD_GAME_STATE].data.game_state.sync_timestamp==
                   snapshot.command[REF_CMD_GAME_STATE].data.game_state.stage_remain_s);
        TEST_CHECK((uint8_t)snapshot.command[REF_CMD_GAME_STATE].data.game_state.sync_timestamp==
                   snapshot.command[REF_CMD_GAME_STATE].wire_sequence);
    }
}
#endif

int main(void)
{
    handle.hdmarx=&dma;
    RefereeSnapshot_t snapshot;
    TEST_CHECK(RefereeGetSnapshot(&snapshot)==REFEREE_ERROR_NOT_INITIALIZED);
    TEST_CHECK(memcmp(&snapshot,&(RefereeSnapshot_t){0},sizeof(snapshot))==0);
    UART_HandleTypeDef *handle_pointer=&handle;
    RefereeConfig_t invalid={0u,1000u,3000u,3000u};
    TEST_CHECK(RefereeInit(handle_pointer,&invalid)==REFEREE_ERROR_CONFIG);
    invalid=(RefereeConfig_t){UINT32_MAX,1000u,3000u,3000u};
    TEST_CHECK(RefereeInit(handle_pointer,&invalid)==REFEREE_ERROR_CONFIG);
    RefereeConfig_t config={300u,1000u,3000u,3000u};
    TEST_CHECK(RefereeInit(handle_pointer,&config)==REFEREE_OK);
    config.timeout_10hz_ms=1u; // 初始化后修改源对象不能改变模块保存的期限。
    TEST_CHECK(dma_bytes!=NULL && dma_capacity==255u);
    TEST_CHECK(RefereeInit(handle_pointer,NULL)==REFEREE_ERROR_ALREADY_INITIALIZED);
    check_protocol_and_lengths();
    check_fixed_field_decoding();
    check_failures_and_stream();
    check_expiry();
    check_concurrent_snapshots();
    return 0;
}
