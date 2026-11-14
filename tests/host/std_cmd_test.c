/** @file std_cmd_test.c
 * @brief 使用隔离输入检查生产适配接口；数据不用于实车，不需要HAL或RTOS替身。
 */
#include "std_cmd.h"
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { fprintf(stderr, "line %d: %s\n", __LINE__, #condition); return 1; } } while (0)

// 测试每个通道都使用明确范围，数值只用于检查转换和限幅。
static StdCmdManualMap_t TestMap(void)
{
    const StdCmdManualMap_t map = {
        .axis = {
            {660.0f, 0.01f, 2.0f}, // 前进：输入计数换m/s，测试正向限幅。
            {660.0f, -0.01f, 2.0f}, // 左移：反向输入映射，测试方向转换。
            {660.0f, 0.001f, 1.0f}, // 底盘旋转：输入计数换rad/s。
            {660.0f, 0.002f, 2.0f}, // 枪口yaw：输入计数换rad/s。
            {660.0f, 0.003f, 2.0f}, // 枪口pitch：输入计数换rad/s。
        },
        .max_age_ms = 100u, // 测试配置最多使用100ms，不代表实车参数。
        .allowed_frames = 1u << STD_CMD_FRAME_CHASSIS, // 本测试仅允许底盘参照。
    };
    return map;
}

// 本来源比映射多允许50ms，用于证明换算不会延长期限。
static StdCmdManualInput_t TestInput(void)
{
    const StdCmdManualInput_t input = {
        .axis = {300.0f, 300.0f, 100.0f, -100.0f, 100.0f}, // 五通道合法测试计数。
        .source_time_ms = 1000u, // 采用独立于硬件的固定原始时刻。
        .sequence = 7u, // 重复读取必须保持此序号。
        .max_age_ms = 150u, // 来源自身期限，与配置取较小值。
        .source = STD_CMD_REMOTE, // 测试遥控来源。
        .frame = STD_CMD_FRAME_CHASSIS, // 与测试配置允许位一致。
        .valid = 1u, // 表示上游协议校验已通过。
    };
    return input;
}

// 故障返回必须保留完整旧输出；调用者还必须根据失败状态撤销来源。
static int FailurePreserves(const StdCmdManualMap_t *map, const StdCmdManualInput_t *input,
                             uint32_t now, StdCmdStatus_t expected)
{
    StdCmdManualIntent_t output;
    unsigned char previous[sizeof(output)];
    memset(&output, 0xa5, sizeof(output)); // 填非零字节，使部分写入也能被发现。
    memcpy(previous, &output, sizeof(output)); // 连同填充字节保留原输出。
    CHECK(StdCmdAdaptManual(map, input, now, &output) == expected);
    CHECK(memcmp(previous, &output, sizeof(output)) == 0);
    return 0;
}

static int Manual(void)
{
    StdCmdManualMap_t map = TestMap();
    StdCmdManualInput_t input = TestInput();
    StdCmdManualIntent_t output;
    CHECK(StdCmdAdaptManual(&map, &input, 1100u, &output) == STD_CMD_OK);
    CHECK(output.axis[STD_CMD_VX] == 2.0f && output.axis[STD_CMD_VY] == -2.0f);
    CHECK(fabsf(output.axis[STD_CMD_WZ] - 0.1f) < 0.00001f);
    CHECK(fabsf(output.axis[STD_CMD_YAW_RATE] + 0.2f) < 0.00001f);
    CHECK(fabsf(output.axis[STD_CMD_PITCH_RATE] - 0.3f) < 0.00001f);
    CHECK(output.source_time_ms == 1000u && output.sequence == 7u && output.max_age_ms == 100u);
    CHECK(FailurePreserves(&map, &input, 1101u, STD_CMD_EXPIRED) == 0);
    CHECK(FailurePreserves(&map, &input, 999u, STD_CMD_EXPIRED) == 0);
    input.max_age_ms = 20u; // 来源期限更短时，不能被配置中的100ms替代。
    CHECK(FailurePreserves(&map, &input, 1021u, STD_CMD_EXPIRED) == 0);
    CHECK(StdCmdAdaptManual(&map, &input, 1020u, &output) == STD_CMD_OK);
    CHECK(output.max_age_ms == 20u && output.source_time_ms == 1000u);
    input.source_time_ms = UINT32_MAX - 4u; // 从最大值跨零，实际年龄为10ms。
    CHECK(StdCmdAdaptManual(&map, &input, 5u, &output) == STD_CMD_OK);
    input = TestInput();
    input.source = STD_CMD_KEY_MOUSE; // 换来源不改变单位换算，也不产生新时间。
    CHECK(StdCmdAdaptManual(&map, &input, 1000u, &output) == STD_CMD_OK);
    CHECK(output.source == STD_CMD_KEY_MOUSE && output.source_time_ms == input.source_time_ms);
    return 0;
}

static int Invalid(void)
{
    StdCmdManualMap_t map = TestMap();
    StdCmdManualInput_t input = TestInput();
    CHECK(FailurePreserves(NULL, &input, 1000u, STD_CMD_BAD_ARGUMENT) == 0);
    CHECK(FailurePreserves(&map, NULL, 1000u, STD_CMD_BAD_ARGUMENT) == 0);
    CHECK(StdCmdAdaptManual(&map, &input, 1000u, NULL) == STD_CMD_BAD_ARGUMENT);
    input.axis[4] = NAN; // 最后一个通道失败也不能写出前四个新值。
    CHECK(FailurePreserves(&map, &input, 1000u, STD_CMD_INVALID_INPUT) == 0);
    input = TestInput();
    input.axis[0] = 661.0f; // 越界输入不能先夹到660再接收。
    CHECK(FailurePreserves(&map, &input, 1000u, STD_CMD_INVALID_INPUT) == 0);
    input = TestInput();
    input.frame = (StdCmdFrame_t)-1;
    CHECK(FailurePreserves(&map, &input, 1000u, STD_CMD_INVALID_INPUT) == 0);
    input.frame = STD_CMD_FRAME_FIELD; // 合法枚举但车型未允许。
    CHECK(FailurePreserves(&map, &input, 1000u, STD_CMD_INVALID_INPUT) == 0);
    input = TestInput();
    input.source = (StdCmdManualSource_t)99;
    CHECK(FailurePreserves(&map, &input, 1000u, STD_CMD_INVALID_INPUT) == 0);
    input = TestInput();
    input.sequence = 0u;
    CHECK(FailurePreserves(&map, &input, 1000u, STD_CMD_INVALID_INPUT) == 0);
    input = TestInput();
    input.valid = 0u;
    CHECK(FailurePreserves(&map, &input, 1000u, STD_CMD_INVALID_INPUT) == 0);
    input = TestInput();
    map.max_age_ms = 0u;
    CHECK(FailurePreserves(&map, &input, 1000u, STD_CMD_BAD_CONFIG) == 0);
    map = TestMap();
    map.allowed_frames = 1u << STD_CMD_FRAME_COUNT;
    CHECK(FailurePreserves(&map, &input, 1000u, STD_CMD_BAD_CONFIG) == 0);
    map = TestMap();
    map.axis[1].gain = INFINITY;
    CHECK(FailurePreserves(&map, &input, 1000u, STD_CMD_BAD_CONFIG) == 0);
    map = TestMap();
    map.axis[0].input_limit = 0.0f;
    CHECK(FailurePreserves(&map, &input, 1000u, STD_CMD_BAD_CONFIG) == 0);
    map = TestMap();
    map.axis[0].output_limit = -1.0f;
    CHECK(FailurePreserves(&map, &input, 1000u, STD_CMD_BAD_CONFIG) == 0);
    map = TestMap();
    map.axis[0].gain = FLT_MAX; // 所有输入有限，但乘法溢出仍必须拒绝。
    CHECK(FailurePreserves(&map, &input, 1000u, STD_CMD_INVALID_INPUT) == 0);
    return 0;
}

static int Events(void)
{
    StdCmdEventCursor_t cursor = {0}; // 两来源序号均从未消费开始。
    StdCmdEvent_t event = {UINT32_MAX, 1000u, 50u, STD_CMD_REMOTE}; // 测试回绕前的最后编号。
    CHECK(StdCmdConsumeEvent(&cursor, &event, 1050u) == STD_CMD_OK);
    CHECK(StdCmdConsumeEvent(&cursor, &event, 1050u) == STD_CMD_DUPLICATE);
    event.sequence = 1u; // 序号跳过保留零后，仍是合法向前。
    CHECK(StdCmdConsumeEvent(&cursor, &event, 1050u) == STD_CMD_OK);
    event.sequence = UINT32_MAX; // 回退到旧序号不能重新执行。
    CHECK(StdCmdConsumeEvent(&cursor, &event, 1050u) == STD_CMD_DUPLICATE);
    event.sequence = 2u;
    CHECK(StdCmdConsumeEvent(&cursor, &event, 1051u) == STD_CMD_EXPIRED);
    CHECK(cursor.sequence[STD_CMD_REMOTE] == 1u);
    event.source = STD_CMD_KEY_MOUSE; // 不同来源允许使用相同编号。
    event.sequence = 1u;
    CHECK(StdCmdConsumeEvent(&cursor, &event, 1050u) == STD_CMD_OK);
    event.sequence = 0u;
    CHECK(StdCmdConsumeEvent(&cursor, &event, 1050u) == STD_CMD_INVALID_INPUT);
    event.source = (StdCmdManualSource_t)-1;
    CHECK(StdCmdConsumeEvent(&cursor, &event, 1050u) == STD_CMD_INVALID_INPUT);
    CHECK(StdCmdConsumeEvent(NULL, &event, 1050u) == STD_CMD_BAD_ARGUMENT);
    CHECK(StdCmdConsumeEvent(&cursor, NULL, 1050u) == STD_CMD_BAD_ARGUMENT);
    return 0;
}

int main(void)
{
    CHECK(Manual() == 0);
    CHECK(Invalid() == 0);
    CHECK(Events() == 0);
    return 0;
}
