/** @file robot_safety_control_test.c
 * @brief 使用真实安全管理验证来源接管、立即禁止和正式自主的遥控条件。
 */
#include "robot_safety.h"
#include <stdio.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); return 1; } } while (0)
static uint32_t now;
uint32_t HAL_GetTick(void) { return now; }

// 准备核心事实；遥控是否有效由各场景决定，不直接伪造READY。
static void Healthy(uint8_t remote)
{
    RobotSafetyInit();
    RobotSafetySetRcOnline(remote);
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
}

static RobotSafetyControlInputs_t Inputs(RobotControlMode_t mode)
{
    const RobotSafetyControlInputs_t inputs = {
        .mode = mode, // 场景显式选择模式，不推断遥控开关含义。
        .manual = {now, 100u, 1u}, // 本时刻产生的新人工来源。
        .aim = {now, 20u, 1u}, // 瞄准期限独立于人工和导航。
        .navigation = {now, 50u, 1u}, // 导航期限独立。
        .match = {now, 100u, 1u}, // 本时刻报告比赛运行阶段。
        .feedback_valid_functions = ROBOT_CONTROL_ALL, // 本场景全部反馈已检查。
        .match_running = 1u, // 比赛运行事实仍受match期限检查。
        .manual_fire_requested = 1u, // 用于验证辅助人工击发。
        .vision_fire_requested = 1u, // 用于验证自主视觉击发。
    };
    return inputs;
}

static int ManualAndPulses(void)
{
    now = 100u; Healthy(1u);
    const RobotControlPolicy_t policy = {ROBOT_CONTROL_ALL, 1u, 0u, 0u};
    CHECK(RobotSafetyConfigureControl(&policy));
    CHECK(!RobotSafetyConfigureControl(&policy)); // 配置不能在运行途中再次改写。
    RobotSafetyRequestEnable(); RobotSafetyUpdate(); // 使能本身不让旧来源取得许可。
    CHECK(!RobotSafetyIsReady());
    now = 101u; RobotSafetyControlInputs_t inputs = Inputs(ROBOT_CONTROL_MANUAL);
    CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate();
    CHECK(RobotSafetyIsReady());
    RobotControlPermits_t permits;
    CHECK(RobotSafetyGetFunctionPermitsAt(now, &permits));
    CHECK(permits.fire_source == ROBOT_CONTROL_SOURCE_MANUAL);
    RobotSafetySnapshot_t before, after;
    CHECK(RobotSafetyGetSnapshot(&before));
    now = 102u; RobotFaultReport(ROBOT_FAULT_CAN_BUSOFF); // 在两个控制周期之间制造短暂禁止。
    CHECK(!RobotSafetyIsReady());
    RobotFaultClear(ROBOT_FAULT_CAN_BUSOFF);
    CHECK(!RobotSafetyIsReady()); // 即使核心恢复，旧来源仍不能重新接管。
    CHECK(RobotSafetyGetSnapshot(&after) && after.control_generation != before.control_generation);
    CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate(); // 再次提交同一旧来源不能续期。
    CHECK(!RobotSafetyIsReady());
    now = 103u; inputs = Inputs(ROBOT_CONTROL_ASSIST);
    CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate();
    CHECK(RobotSafetyGetFunctionPermitsAt(now, &permits));
    CHECK(permits.gimbal_source == ROBOT_CONTROL_SOURCE_VISION && permits.fire_source == ROBOT_CONTROL_SOURCE_MANUAL);
    now = 104u; inputs.feedback_valid_functions &= ~(uint32_t)ROBOT_CONTROL_CHASSIS; // 局部底盘反馈失效。
    CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate();
    CHECK(RobotSafetyGetFunctionPermitsAt(now, &permits));
    CHECK(!(permits.functions & ROBOT_CONTROL_CHASSIS) && (permits.functions & ROBOT_CONTROL_GIMBAL));
    now = 105u; inputs.feedback_valid_functions |= ROBOT_CONTROL_CHASSIS; // 恢复反馈，但目标仍来自103ms。
    CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate();
    CHECK(RobotSafetyGetFunctionPermitsAt(now, &permits));
    CHECK(!(permits.functions & ROBOT_CONTROL_CHASSIS) && (permits.functions & ROBOT_CONTROL_GIMBAL));
    now = 106u; inputs = Inputs(ROBOT_CONTROL_MANUAL); // 新人工输入只在恢复之后产生。
    CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate(); CHECK(RobotSafetyIsReady());
    CHECK(RobotSafetyGetFunctionPermitsAt(now, &permits) && (permits.functions & ROBOT_CONTROL_CHASSIS));
    now = 107u; inputs.blocked_functions = ROBOT_CONTROL_SHOOTER; // 局部卡弹只停止发射。
    CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate();
    CHECK(RobotSafetyGetFunctionPermitsAt(now, &permits) && !permits.fire_allowed && (permits.functions & ROBOT_CONTROL_CHASSIS));
    now = 108u; inputs.blocked_functions = 0u; // 旧请求不能随着卡弹解除重新执行。
    CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate();
    CHECK(RobotSafetyGetFunctionPermitsAt(now, &permits) && !(permits.functions & ROBOT_CONTROL_SHOOTER) && (permits.functions & ROBOT_CONTROL_GIMBAL));
    now = 207u; CHECK(!RobotSafetyIsReady()); // 106ms的新人工来源超过100ms后，查询时即撤销。
    CHECK(RobotSafetyGetFunctionPermitsAt(now, &permits) && permits.functions == 0u);
    now = 208u; inputs = Inputs(ROBOT_CONTROL_MANUAL);
    CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate();
    CHECK(RobotSafetyIsReady());
    CHECK(!RobotSafetyPublishControlInputs(NULL)); // 错误提交必须撤销旧事实。
    CHECK(!RobotSafetyIsReady());
    return 0;
}

static int Autonomous(void)
{
    now = 300u; Healthy(0u); // 正式自主没有遥控在线条件。
    const RobotControlPolicy_t policy = {ROBOT_CONTROL_ALL, 1u, 1u, 0u};
    CHECK(RobotSafetyConfigureControl(&policy)); RobotSafetyUpdate();
    CHECK(!EstopIsLatched() && !RobotSafetyIsReady()); // 缺少比赛阶段时只等待，不凭资格启动。
    now = 301u; RobotSafetyControlInputs_t inputs = Inputs(ROBOT_CONTROL_AUTONOMOUS);
    inputs.manual.valid = 0u;
    CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate();
    CHECK(RobotSafetyIsReady() && RobotSafetyIsFormalAutonomous());
    RobotControlPermits_t permits;
    CHECK(RobotSafetyGetFunctionPermitsAt(now, &permits) && permits.fire_source == ROBOT_CONTROL_SOURCE_VISION);
    RobotSafetySetRcOnline(0u); RobotSafetyUpdate(); // 重复遥控离线不能撤销正式自主来源。
    CHECK(!EstopIsLatched() && RobotSafetyIsReady());
    now = 322u; CHECK(RobotSafetyGetFunctionPermitsAt(now, &permits));
    CHECK(permits.chassis_source == ROBOT_CONTROL_SOURCE_NAVIGATION && permits.gimbal_source == ROBOT_CONTROL_SOURCE_LOCAL && !permits.fire_allowed);
    EstopRequest(ESTOP_REASON_MANUAL); RobotSafetyUpdate();
    CHECK(!RobotSafetyIsReady());
    CHECK(EstopClear()); // 正式自主可以在没有遥控时明确解除人工急停。
    RobotSafetyUpdate();
    CHECK(RobotSafetyIsReady()); // 仍须比赛与核心有效；恢复后只能采用本地策略等待新目标。
    CHECK(RobotSafetyGetFunctionPermitsAt(now, &permits) && permits.chassis_source == ROBOT_CONTROL_SOURCE_LOCAL && !permits.fire_allowed);
    RobotSafetyRequestDisable(); RobotSafetyUpdate();
    CHECK(!RobotSafetyIsReady()); // 人工显式失能不能被自主资格绕过。
    now = 323u; inputs = Inputs(ROBOT_CONTROL_AUTONOMOUS);
    CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate(); CHECK(!RobotSafetyIsReady());
    RobotSafetyRequestEnable(); RobotSafetyUpdate();
    CHECK(RobotSafetyIsReady());
    now = 424u; CHECK(!RobotSafetyIsReady()); // 比赛阶段到期撤销全部功能。
    return 0;
}

int main(void)
{
    CHECK(ManualAndPulses() == 0);
    CHECK(Autonomous() == 0);
    return 0;
}
