/** @file dji_can_diagnostic_stm32.c
 * @brief 对已注册 DJI 电机执行 STM32 CAN 内部回环诊断。
 */
#include "dji_motor_diagnostic_internal.h"
#if CAN_DJI_INJECT_TEST >= 1
#include "board_can_stm32.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "../dwt/platform_probe.h"
#include <math.h>
#include <stddef.h>
#endif

/* ============================================================================
   DJI 假反馈注入自测(默认关闭, 开关见 dji_motor.h 的 CAN_DJI_INJECT_TEST)
   ========================================================================== */
#if CAN_DJI_INJECT_TEST >= 1

volatile uint32_t can_dji_inject_pass = 0;
volatile uint32_t can_dji_inject_fail = 0;
volatile uint32_t can_dji_inject_multi_turn_err = 0;
volatile uint32_t can_dji_inject_frame_cnt = 0;

static uint8_t s_inject_source;                     /* 固定来源身份，不持有发送缓存 */
static uint8_t s_inject_frame[8];
static DJIMotorInstance *s_inject_target = NULL;    /* 被注入的电机 */
static DJI_Motor_Measure_s s_inject_backup;         /* 测试前的 measure, 结束后还原 */
static FDCAN_HandleTypeDef *s_inject_h = NULL;

/* 按 DJI 反馈帧格式填好 tx_buff 并发出(不等待接收) */
static void DJIInjectSend(uint16_t ecd, int16_t speed_rpm, int16_t current_raw, uint8_t temp)
{
    if (s_inject_h == NULL)
        return;
    s_inject_frame[0] = (uint8_t)(ecd >> 8);
    s_inject_frame[1] = (uint8_t)(ecd & 0xFFu);
    s_inject_frame[2] = (uint8_t)(((uint16_t)speed_rpm) >> 8);
    s_inject_frame[3] = (uint8_t)(((uint16_t)speed_rpm) & 0xFFu);
    s_inject_frame[4] = (uint8_t)(((uint16_t)current_raw) >> 8);
    s_inject_frame[5] = (uint8_t)(((uint16_t)current_raw) & 0xFFu);
    s_inject_frame[6] = temp;
    s_inject_frame[7] = 0u;
    can_dji_inject_frame_cnt++;
    const CAN_TxFrame_t frame = {
        .identifier = s_inject_target->motor_can_endpoint.rx_id,
        .identifier_type = CAN_TX_ID_STANDARD,
        .data = s_inject_frame,
        .length = 8u,
    };
    uint32_t request_id = 0u;
    if (CANSubmitOneShot(s_inject_h, &s_inject_source, &frame,
                         CAN_TX_PRIORITY_URGENT, 3u, &request_id) != CAN_TX_SUBMIT_ACCEPTED)
        can_dji_inject_fail++;
}

/* 边消费接收队列边等, 直到目标电机的 ecd 变成期望值; 返回 1=等到 */
static uint8_t DJIInjectWaitEcd(uint16_t expect, uint32_t timeout_us)
{
    uint32_t t0 = PlatformProbeStart();
    while (DWT_ProbeElapsedUs(t0) < timeout_us)
    {
        CANProcessRx();
        CANProcessTx();
        if (s_inject_target != NULL && s_inject_target->measure.ecd == expect)
            return 1u;
    }
    return 0u;
}

uint8_t CANRunDjiFeedbackInjectTest(void)
{
    can_dji_inject_pass = 0;
    can_dji_inject_fail = 0;
    can_dji_inject_multi_turn_err = 0;
    can_dji_inject_frame_cnt = 0;

    DJIMotorInstance *first_motor = DJIMotorDiagnosticFirst(); // 从普通驱动取得长期有效的注册对象。
    if (first_motor == NULL)
    {
        BSP_LOG_EVENT_0(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_05);
        return 0u;
    }

    s_inject_target = first_motor;
    s_inject_h = BoardCanResolve(s_inject_target->motor_can_endpoint.can_bus);
    uint32_t target_rx_id = s_inject_target->motor_can_endpoint.rx_id;
    s_inject_backup = s_inject_target->measure;

    /* 1. 切内部回环: 帧不会真的上线, 但自己发自己收, 不会干扰总线上的其它设备 */
    if (CANForceMode(s_inject_h, FDCAN_MODE_INTERNAL_LOOPBACK, 0u) == 0u)
    {
        BSP_LOG_EVENT_0(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_06);
        /* CANForceMode 失败时可能已经把总线 Stop 掉了, 这里必须恢复一次,
           否则这条总线上线不了, 后面的电机控制帧全都发不出去。 */
        (void)CANForceMode(s_inject_h, FDCAN_MODE_NORMAL, 0u);
        CANReapplyFilters();
        return 0u;
    }

    /* 关键: CANForceMode 内部的 HAL_FDCAN_Init() 会重配消息 RAM, 把各电机实例
       原先注册的接收滤镜一起清掉。不重装的话, 我们发出去的 0x201 根本没有滤镜
       去接, 注入帧会被硬件直接丢弃(实测: 5 项全失败、只发出 4 帧就是这个原因)。
       回环自测不需要这一步, 因为它注册测试实例时自己会加滤镜。 */
    CANReapplyFilters();

    /* 发送接口按已运行总线句柄提交复制帧，无需制造无接收用途的注册实例。 */
    (void)target_rx_id;

    /* ---- 测试 1: 单帧解码(ecd / 温度) ---- */
    DJIInjectSend(1000u, 100, 50, 42u);
    if (DJIInjectWaitEcd(1000u, 3000u) && s_inject_target->measure.temperature == 42u)
        can_dji_inject_pass++;
    else
        can_dji_inject_fail++;

    /* ---- 测试 2: 正向跨圈。ecd 1000→8100, 差 +7100 > 4096(半圈) → total_round 应 -1 ---- */
    int32_t r0 = s_inject_target->measure.total_round;
    DJIInjectSend(8100u, 0, 0, 42u);
    if (DJIInjectWaitEcd(8100u, 3000u) && s_inject_target->measure.total_round == r0 - 1)
        can_dji_inject_pass++;
    else
    {
        can_dji_inject_fail++;
        can_dji_inject_multi_turn_err++;
    }

    /* ---- 测试 3: 反向跨圈。ecd 8100→1000, 差 -7100 < -4096 → total_round 应回到 r0 ---- */
    DJIInjectSend(1000u, 0, 0, 42u);
    if (DJIInjectWaitEcd(1000u, 3000u) && s_inject_target->measure.total_round == r0)
        can_dji_inject_pass++;
    else
    {
        can_dji_inject_fail++;
        can_dji_inject_multi_turn_err++;
    }

    /* ---- 测试 4: total_angle 与 (total_round, ecd) 一致 ---- */
    float expect_angle = (float)(r0 * 360) + ECD_ANGLE_COEF_DJI * 1000.0f;
    if (fabsf(s_inject_target->measure.total_angle - expect_angle) < 0.5f)
        can_dji_inject_pass++;
    else
        can_dji_inject_fail++;

    /* ---- 测试 5: 连续多帧的 ecd 序列(验证逐帧、按到达顺序被处理) ---- */
    uint8_t seq_ok = 1u;
    for (uint16_t k = 0; k < 20u; ++k)
    {
        uint16_t e = (uint16_t)((1000u + k * 13u) & 0x1FFFu);
        DJIInjectSend(e, 0, 0, 42u);
        if (!DJIInjectWaitEcd(e, 2000u))
        {
            seq_ok = 0u;
            break;
        }
    }
    if (seq_ok)
        can_dji_inject_pass++;
    else
        can_dji_inject_fail++;

#if CAN_DJI_INJECT_TEST >= 2
    /* 压测模式: 保留回环与注入实例, 由 daemon 循环持续注入, 用来观察队列深度与丢帧 */
    BSP_LOG_EVENT_2(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_08, BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_pass), BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_fail));
#else
    /* 收尾: 还原 measure、切回正常模式并重装滤镜 */
    s_inject_target->measure = s_inject_backup;
    (void)CANForceMode(s_inject_h, FDCAN_MODE_NORMAL, 0u);
    CANReapplyFilters();
    s_inject_h = NULL;
#endif

    BSP_LOG_EVENT_4(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_09, BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_pass), BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_fail), BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_multi_turn_err), BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_frame_cnt));
    return (can_dji_inject_fail == 0u) ? 1u : 0u;
}

#if CAN_DJI_INJECT_TEST >= 2
void DJIInjectStressTick(void)
{
    if (s_inject_h == NULL)
        return;
    /* 每轮塞一小批帧(不等待消费), 制造"任务还没来得及消费"的场景,
       从而观察 rxq_peak(队列最大占用)与 rxq_drop(队列满丢帧)。 */
    static uint16_t ecd = 0u;
    for (uint8_t n = 0; n < 10u; ++n)
    {
        ecd = (uint16_t)((ecd + 7u) & 0x1FFFu);
        DJIInjectSend(ecd, 50, 20, 40u);
    }
}
#endif

#endif /* CAN_DJI_INJECT_TEST >= 1 */
