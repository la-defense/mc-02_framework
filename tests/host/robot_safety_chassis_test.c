#include "robot_safety.h"
#include <stdio.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL:%d %s\n", __LINE__, #condition); return 1; } } while (0)

static uint32_t safety_now_ms;

uint32_t HAL_GetTick(void)
{
    return safety_now_ms;
}

static uint8_t grant_current_remote_command(void)
{
    RobotSafetySnapshot_t snapshot = {0};
    if (!RobotSafetyGetSnapshot(&snapshot))
        return 0u;
    return RobotSafetyTryGrantRemoteChassisPermit(snapshot.chassis_inhibit_generation,
        safety_now_ms, 50u, safety_now_ms);
}

int main(void)
{
    RobotSafetySnapshot_t before = {0};
    RobotSafetySnapshot_t after = {0};
    safety_now_ms = 0u;
    RobotSafetyInit();
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    CHECK(grant_current_remote_command());
    RobotSafetyUpdate();
    CHECK(RobotSafetyGetState() == ROBOT_READY);
    CHECK(RobotSafetyIsReadyAt(safety_now_ms));

    // 来源租约在期限边界仍有效，越界后立即失能并在状态更新中清除许可。
    safety_now_ms = 50u;
    CHECK(RobotSafetyIsReadyAt(safety_now_ms));
    safety_now_ms = 51u;
    CHECK(!RobotSafetyIsReadyAt(safety_now_ms));
    RobotSafetyUpdate();
    CHECK(RobotSafetyGetState() == ROBOT_SAFE);
    CHECK(RobotSafetyGetSnapshot(&after) && after.remote_chassis_permit == 0u);
    CHECK(!RobotSafetyTryGrantRemoteChassisPermit(after.chassis_inhibit_generation,
        0u, 50u, safety_now_ms));
    CHECK(!RobotSafetyTryGrantRemoteChassisPermit(after.chassis_inhibit_generation,
        safety_now_ms, 0u, safety_now_ms));
    CHECK(grant_current_remote_command());
    RobotSafetyUpdate();
    CHECK(RobotSafetyIsReadyAt(safety_now_ms));

    // 故障代际变化与许可授予原子比较，读取旧代际的控制任务不能重新开闸。
    CHECK(RobotSafetyGetSnapshot(&before));
    RobotFaultReport(ROBOT_FAULT_CAN_BUSOFF);
    CHECK(!RobotSafetyIsReadyAt(safety_now_ms));
    RobotSafetyUpdate();
    CHECK(RobotSafetyGetState() == ROBOT_FAULT);
    CHECK(RobotSafetyGetSnapshot(&after));
    CHECK(after.chassis_inhibit_generation != before.chassis_inhibit_generation);
    RobotFaultClear(ROBOT_FAULT_CAN_BUSOFF);
    RobotSafetyUpdate();
    CHECK(!RobotSafetyTryGrantRemoteChassisPermit(before.chassis_inhibit_generation,
        safety_now_ms, 50u, safety_now_ms));
    CHECK(grant_current_remote_command());
    RobotSafetyUpdate();
    CHECK(RobotSafetyIsReadyAt(safety_now_ms));

    EstopRequest(ESTOP_REASON_MANUAL);
    CHECK(!RobotSafetyIsReadyAt(safety_now_ms));
    RobotSafetyUpdate();
    CHECK(RobotSafetyGetState() == ROBOT_ESTOP);
    CHECK(EstopClear());
    CHECK(!RobotSafetyIsReadyAt(safety_now_ms));
    CHECK(grant_current_remote_command());
    RobotSafetyUpdate();
    CHECK(RobotSafetyIsReadyAt(safety_now_ms));

    CHECK(RobotSafetyGetSnapshot(&before));
    RobotSafetyRevokeRemoteChassisPermit();
    CHECK(!RobotSafetyIsReadyAt(safety_now_ms));
    CHECK(RobotSafetyGetSnapshot(&after));
    CHECK(after.remote_chassis_permit == 0u);
    CHECK(after.chassis_inhibit_generation != before.chassis_inhibit_generation);

    // 本地姿态条件失败时，即使命令仍在期限内也不能授予底盘许可。
    RobotSafetySetImuValid(0u);
    CHECK(!grant_current_remote_command());
    RobotSafetyUpdate();
    CHECK(!RobotSafetyIsReadyAt(safety_now_ms));
    CHECK(!RobotSafetyIsChassisLocalReady());

    RobotSafetyInit();
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    CHECK(grant_current_remote_command());
    RobotSafetyUpdate();
    CHECK(RobotSafetyIsReadyAt(safety_now_ms));
    RobotSafetyRequestDisable();
    CHECK(!RobotSafetyIsReadyAt(safety_now_ms));
    CHECK(RobotSafetyGetSnapshot(&after));
    CHECK(after.remote_chassis_permit == 0u);
    CHECK(grant_current_remote_command());
    RobotSafetyUpdate();
    CHECK(RobotSafetyIsReadyAt(safety_now_ms));

    RobotSafetySetImuValid(0u);
    RobotSafetySetImuValid(1u);
    CHECK(!RobotSafetyIsReadyAt(safety_now_ms));
    RobotSafetyUpdate();
    CHECK(!RobotSafetyIsReadyAt(safety_now_ms));
    return 0;
}
