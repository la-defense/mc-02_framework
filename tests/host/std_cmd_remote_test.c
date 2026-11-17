/** @file std_cmd_remote_test.c
 * @brief 使用隔离测试配置检查已解析遥控与键鼠数据的人工意图换算。
 */
#include "std_cmd_remote.h"
#include "test_check.h"
#include <math.h>
#include <string.h>

int main(void)
{
    StdCmdRemoteMap_t map = {0}; // 本文件中的参数仅用于主机计算。
    map.remote_map.max_age_ms = 100u; // 无线输入不得超过自己的期限。
    map.keyboard_map.max_age_ms = 150u; // 键盘输入拥有独立期限。
    map.remote_map.allowed_frames = 1u << STD_CMD_FRAME_CHASSIS;
    map.keyboard_map.allowed_frames = 1u << STD_CMD_FRAME_BIG_YAW;
    map.remote_frame = STD_CMD_FRAME_CHASSIS;
    map.keyboard_frame = STD_CMD_FRAME_BIG_YAW;
    for (unsigned axis = 0u; axis < STD_CMD_AXIS_COUNT; ++axis)
    {
        map.remote_map.axis[axis] = (StdCmdAxisMap_t){660.0f, 0.01f, 10.0f}; // 测试摇杆范围与比例。
        map.keyboard_map.axis[axis] = (StdCmdAxisMap_t){1.0f, 2.0f, 2.0f}; // 测试键盘方向到物理量的比例。
    }
    map.remote_axis[STD_CMD_VX] = STD_CMD_RC_RIGHT_Y; // 前进由测试右摇杆竖直通道提供。
    map.remote_axis[STD_CMD_VY] = STD_CMD_RC_RIGHT_X; // 平移由测试右摇杆水平通道提供。
    map.keyboard_positive[STD_CMD_VX] = (uint16_t)(1u << Key_W); // W 为测试正向。
    map.keyboard_negative[STD_CMD_VX] = (uint16_t)(1u << Key_S); // S 为测试负向。
    RemoteControlSnapshot_s remote = {0};
    remote.remote_online = 1u; remote.radio_sequence = 7u; remote.radio_source_time_ms = 100u;
    remote.input.rc.switch_left = RC_SW_DOWN; remote.input.rc.rocker_r1 = 100; remote.input.rc.rocker_r_ = -50;
    StdCmdManualIntent_t output = {0};
    TEST_CHECK(StdCmdAdaptRemote(&map, &remote, 110u, &output) == STD_CMD_OK);
    TEST_CHECK(fabsf(output.axis[STD_CMD_VX] - 1.0f) < 0.0001f &&
               fabsf(output.axis[STD_CMD_VY] + 0.5f) < 0.0001f); // 检查符号和通道映射。
    TEST_CHECK(output.source == STD_CMD_REMOTE && output.sequence == 7u && output.source_time_ms == 100u &&
               output.frame == STD_CMD_FRAME_CHASSIS && output.max_age_ms == 100u); // 原始时间与来源不被续期。
    const StdCmdManualIntent_t prior = output; // 失败时旧结果必须保持不变。
    TEST_CHECK(StdCmdAdaptRemote(&map, &remote, 201u, &output) == STD_CMD_INVALID_INPUT);
    TEST_CHECK(memcmp(&output, &prior, sizeof(output)) == 0); // 超期不能交付新目标。
    remote.input.rc.switch_left = RC_SW_UP; remote.pc_valid = 1u;
    remote.pc_source_time_ms = 95u; remote.pc_sample_sequence = 3u;
    remote.input.key[KEY_PRESS].keys = (uint16_t)(1u << Key_W);
    TEST_CHECK(StdCmdAdaptRemote(&map, &remote, 120u, &output) == STD_CMD_OK);
    TEST_CHECK(output.source == STD_CMD_KEY_MOUSE && output.axis[STD_CMD_VX] == 2.0f &&
               output.sequence == 3u && output.source_time_ms == 95u && output.frame == STD_CMD_FRAME_BIG_YAW &&
               output.max_age_ms == 105u); // 键盘和无线较早到期者共同限制意图。
    TEST_CHECK(StdCmdAdaptRemote(&map, &remote, 190u, &output) == STD_CMD_OK &&
               output.max_age_ms == 105u); // 重复读取不能延长较早来源的截止时间。
    remote.input.key[KEY_PRESS].keys |= (uint16_t)(1u << Key_S); // 同时按下相反方向。
    TEST_CHECK(StdCmdAdaptRemote(&map, &remote, 120u, &output) == STD_CMD_OK && output.axis[STD_CMD_VX] == 0.0f);
    remote.radio_source_time_ms = 200u; // 新无线帧不能替旧键盘续期。
    TEST_CHECK(StdCmdAdaptRemote(&map, &remote, 246u, &output) == STD_CMD_EXPIRED);
    remote.pc_valid = 0u;
    TEST_CHECK(StdCmdAdaptRemote(&map, &remote, 200u, &output) == STD_CMD_INVALID_INPUT);
    remote.input.rc.switch_left = RC_SW_MID;
    TEST_CHECK(StdCmdAdaptRemote(&map, &remote, 200u, &output) == STD_CMD_INVALID_INPUT);
    map.remote_axis[0] = (StdCmdRemoteAxis_t)STD_CMD_RC_AXIS_COUNT;
    TEST_CHECK(StdCmdAdaptRemote(&map, &remote, 200u, &output) == STD_CMD_BAD_CONFIG);
    map.remote_axis[0] = STD_CMD_RC_ZERO; map.keyboard_negative[0] = map.keyboard_positive[0];
    TEST_CHECK(StdCmdAdaptRemote(&map, &remote, 200u, &output) == STD_CMD_BAD_CONFIG);
    map.keyboard_negative[0] = (uint16_t)(1u << Key_S); // 恢复有效测试映射。
    remote.input.rc.switch_left = RC_SW_DOWN; remote.radio_source_time_ms = UINT32_MAX - 4u;
    TEST_CHECK(StdCmdAdaptRemote(&map, &remote, 5u, &output) == STD_CMD_OK &&
               output.source_time_ms == UINT32_MAX - 4u); // 跨越计时回绕仍保留原始时间。
    return 0;
}
