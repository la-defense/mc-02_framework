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

// 按 DJI 反馈帧格式打包并异步提交，接收由后续等待步骤处理。
static void DJIInjectSend(uint16_t ecd, int16_t speed_rpm, int16_t current_raw, uint8_t temp)
{
    if (s_inject_h == NULL)
        return; // 没有已选择的总线时不能提交。
    s_inject_frame[0] = (uint8_t)(ecd >> 8); // 编码器值高字节。
    s_inject_frame[1] = (uint8_t)(ecd & 0xFFu); // 编码器值低字节。
    s_inject_frame[2] = (uint8_t)(((uint16_t)speed_rpm) >> 8); // 有符号转速的高字节。
    s_inject_frame[3] = (uint8_t)(((uint16_t)speed_rpm) & 0xFFu); // 转速低字节。
    s_inject_frame[4] = (uint8_t)(((uint16_t)current_raw) >> 8); // 原始电流高字节。
    s_inject_frame[5] = (uint8_t)(((uint16_t)current_raw) & 0xFFu); // 原始电流低字节。
    s_inject_frame[6] = temp; // 温度字节。
    s_inject_frame[7] = 0u; // 保留字节固定为零。
    can_dji_inject_frame_cnt++; // 记录尝试注入的帧数。
    const CAN_TxFrame_t frame = {
        .identifier = s_inject_target->motor_can_endpoint.rx_id, // 使用目标电机的反馈 ID。
        .identifier_type = CAN_TX_ID_STANDARD, // DJI 反馈使用标准 ID。
        .data = s_inject_frame, // 提交队列会复制这八个字节。
        .length = 8u, // 反馈帧固定八字节。
    };
    uint32_t request_id = 0u; // 保存发送队列生成的请求编号。
    if (CANSubmitOneShot(s_inject_h, &s_inject_source, &frame,
                         CAN_TX_PRIORITY_URGENT, 3u, &request_id) != CAN_TX_SUBMIT_ACCEPTED)
        can_dji_inject_fail++; // 队列未接受时计入诊断失败。
}

// 消费收发队列直到编码器达到期望值或等待超时，成功返回 1。
static uint8_t DJIInjectWaitEcd(uint16_t expect, uint32_t timeout_us)
{
    uint32_t t0 = PlatformProbeStart(); // 记录等待开始的周期计数。
    while (DWT_ProbeElapsedUs(t0) < timeout_us)
    {
        CANProcessRx(); // 将接收队列中的反馈交给电机解码。
        CANProcessTx(); // 推进尚未发送的注入帧。
        if (s_inject_target != NULL && s_inject_target->measure.ecd == expect)
            return 1u; // 已收到本轮期望的编码器值。
    }
    return 0u; // 到达等待时限仍未收到目标值。
}

uint8_t CANRunDjiFeedbackInjectTest(void)
{
    can_dji_inject_pass = 0; // 清除上次通过项数。
    can_dji_inject_fail = 0; // 清除上次失败项数。
    can_dji_inject_multi_turn_err = 0; // 清除跨圈错误数。
    can_dji_inject_frame_cnt = 0; // 清除尝试注入的帧数。

    DJIMotorInstance *first_motor = DJIMotorDiagnosticFirst(); // 从普通驱动取得长期有效的注册对象。
    if (first_motor == NULL)
    {
        BSP_LOG_EVENT_0(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_05); // 记录没有注册电机。
        return 0u; // 无目标时无法注入反馈。
    }

    s_inject_target = first_motor; // 保留选中的长期电机对象。
    s_inject_h = BoardCanResolve(s_inject_target->motor_can_endpoint.can_bus); // 查询电机所在总线。
    uint32_t target_rx_id = s_inject_target->motor_can_endpoint.rx_id; // 读取反馈 ID。
    s_inject_backup = s_inject_target->measure; // 保存测试前的反馈供结束时恢复。

    // 切内部回环，发送帧只回到本机，不进入外部总线。
    if (CANForceMode(s_inject_h, FDCAN_MODE_INTERNAL_LOOPBACK, 0u) == 0u)
    {
        BSP_LOG_EVENT_0(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_06); // 记录切换回环失败。
        // 模式切换失败时总线可能已停止，先尝试恢复模式和滤镜。
        (void)CANForceMode(s_inject_h, FDCAN_MODE_NORMAL, 0u); // 尝试恢复正常总线模式。
        CANReapplyFilters(); // 重装切换过程中可能清除的滤镜。
        return 0u; // 回环不可用时结束诊断。
    }

    // 模式切换会重配消息 RAM 并清除接收滤镜，必须重新安装才能接收注入帧。
    CANReapplyFilters(); // 为回环反馈重装电机接收滤镜。

    // 使用已运行总线提交复制帧，不额外注册无接收用途的实例。
    (void)target_rx_id; // 发送时直接从电机端点读取该 ID。

    // 测试单帧编码器与温度解码。
    DJIInjectSend(1000u, 100, 50, 42u);
    if (DJIInjectWaitEcd(1000u, 3000u) && s_inject_target->measure.temperature == 42u)
        can_dji_inject_pass++;
    else
        can_dji_inject_fail++;

    // 编码器从 1000 到 8100，跨过半圈时圈数应减一。
    int32_t r0 = s_inject_target->measure.total_round;
    DJIInjectSend(8100u, 0, 0, 42u);
    if (DJIInjectWaitEcd(8100u, 3000u) && s_inject_target->measure.total_round == r0 - 1)
        can_dji_inject_pass++;
    else
    {
        can_dji_inject_fail++;
        can_dji_inject_multi_turn_err++;
    }

    // 编码器反向从 8100 回到 1000，圈数应恢复。
    DJIInjectSend(1000u, 0, 0, 42u);
    if (DJIInjectWaitEcd(1000u, 3000u) && s_inject_target->measure.total_round == r0)
        can_dji_inject_pass++;
    else
    {
        can_dji_inject_fail++;
        can_dji_inject_multi_turn_err++;
    }

    // 检查累计角度与圈数、编码器读数一致。
    float expect_angle = (float)(r0 * 360) + ECD_ANGLE_COEF_DJI * 1000.0f; // 从圈数和单圈角度计算期望值。
    if (fabsf(s_inject_target->measure.total_angle - expect_angle) < 0.5f)
        can_dji_inject_pass++; // 角度误差在允许范围内。
    else
        can_dji_inject_fail++; // 角度与期望不一致。

    // 连续注入多帧并逐帧检查接收顺序。
    uint8_t seq_ok = 1u; // 先假定连续反馈均按顺序到达。
    for (uint16_t k = 0; k < 20u; ++k)
    {
        uint16_t e = (uint16_t)((1000u + k * 13u) & 0x1FFFu); // 生成单圈范围内的下一个编码器值。
        DJIInjectSend(e, 0, 0, 42u); // 提交这一帧反馈。
        if (!DJIInjectWaitEcd(e, 2000u))
        {
            seq_ok = 0u; // 超时后标记序列失败。
            break; // 不再发送后续序列帧。
        }
    }
    if (seq_ok)
        can_dji_inject_pass++; // 所有帧都在限时内收到。
    else
        can_dji_inject_fail++; // 至少一帧未按期收到。

#if CAN_DJI_INJECT_TEST >= 2
    // 压测模式保留回环，由 daemon 持续注入以观察队列。
    BSP_LOG_EVENT_2(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_08, BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_pass), BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_fail));
#else
    // 还原反馈、恢复正常模式并重装滤镜。
    s_inject_target->measure = s_inject_backup; // 恢复测试前的电机反馈。
    (void)CANForceMode(s_inject_h, FDCAN_MODE_NORMAL, 0u); // 尝试切回正常总线模式。
    CANReapplyFilters(); // 重装模式切换可能清除的接收滤镜。
    s_inject_h = NULL; // 结束后不再允许继续注入。
#endif

    BSP_LOG_EVENT_4(EV_MODULES_MOTOR_DJIMOTOR_DJI_MOTOR_C_09, BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_pass), BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_fail), BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_multi_turn_err), BSP_LOG_FIELD_U32((unsigned long)can_dji_inject_frame_cnt));
    return (can_dji_inject_fail == 0u) ? 1u : 0u;
}

#if CAN_DJI_INJECT_TEST >= 2
void DJIInjectStressTick(void)
{
    if (s_inject_h == NULL)
        return;
    // 每轮连续提交一小批帧，观察接收任务尚未消费时的占用与丢帧。
    static uint16_t ecd = 0u;
    for (uint8_t n = 0; n < 10u; ++n)
    {
        ecd = (uint16_t)((ecd + 7u) & 0x1FFFu);
        DJIInjectSend(ecd, 50, 20, 40u);
    }
}
#endif

#endif /* CAN_DJI_INJECT_TEST >= 1 */
