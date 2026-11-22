/** @file chassis_motion_diag.c
 * @brief 将受限底盘的速度与位姿编码为定长的只读串口应答。
 */
#include "chassis_motion_diag.h"
#include "bsp_log_diag.h"
#include "chassis_feedback.h"
#include <float.h>
#include <string.h>

_Static_assert(sizeof(float) == sizeof(uint32_t) && FLT_RADIX == 2 &&
               FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128, "motion diagnostic uses binary32");

static void WriteU32(uint8_t *out, uint32_t value)
{
    for (unsigned i = 0u; i < sizeof(value); ++i)
        out[i] = (uint8_t)(value >> (8u * i)); // 按最低字节到最高字节逐个写入。
}

static void WriteFloat(uint8_t *out, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits)); // 保留 binary32 位模式，不做数值转换。
    WriteU32(out, bits);
}

static uint8_t BuildMotionReply(uint8_t *out, uint8_t capacity, uint8_t *length)
{
    if (out == NULL || length == NULL || capacity < BSP_LOG_DIAG_CHASSIS_MOTION_REPLY_SIZE)
        return 0u; // 空指针或空间不足时不读反馈，也不改调用者的输出。
    ChassisVelocityFeedback_t speed;
    ChassisOdometryFeedback_t odometry;
    const uint8_t valid = Chassis_GetMotionFeedback(&speed, &odometry); // 同一次读取，缺测项已清零。
    out[0] = BSP_LOG_DIAG_CHASSIS_MOTION_SCHEMA;
    out[1] = valid; // bit 0 标记速度，bit 1 标记里程计；零值也可以是有效测量。
    WriteU32(out + 2u, speed.source_time_ms); // 最早轮速接收时刻，ms。
    WriteFloat(out + 6u, speed.twist.vx_m_s); // 底盘前向速度，m/s。
    WriteFloat(out + 10u, speed.twist.vy_m_s); // 底盘左向速度，m/s。
    WriteFloat(out + 14u, speed.twist.wz_rad_s); // 绕向上竖轴的角速度，rad/s。
    WriteU32(out + 18u, odometry.source_time_ms); // 最近参与积分的最早轮速时刻，ms。
    WriteU32(out + 22u, odometry.segment); // 轮速断流后用新段号表达原点变化。
    WriteFloat(out + 26u, odometry.pose.x_m); // 段起点坐标系内的前向位移，m。
    WriteFloat(out + 30u, odometry.pose.y_m); // 段起点坐标系内的左向位移，m。
    WriteFloat(out + 34u, odometry.pose.yaw_rad); // 相对段起始方向的角度，rad。
    *length = BSP_LOG_DIAG_CHASSIS_MOTION_REPLY_SIZE;
    return 1u;
}

BSPLogStatus_t ChassisMotionDiagnosticInit(void)
{
    return BSPLogRegisterChassisMotionHandler(BuildMotionReply);
}
