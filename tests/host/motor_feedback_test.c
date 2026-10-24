/* @file motor_feedback_test.c
 * @brief 测试生产电机能力查询和输出轴单位换算的固定结果及失败保持。
 */
#include "test_check.h"
#include "motor_feedback.h"

#include <float.h>
#include <math.h>
#include <string.h>

static void Conversion(void)
{
    const Motor_Output_Axis_Config_s config = {2.0f, MOTOR_AXIS_DIRECTION_REVERSE};
    float position = 77.0f;
    float speed = 88.0f;

    TEST_CHECK(MotorFeedbackConvertOutputAxis(&config, 6.283185307179586f,
        12.566370614359172f, &position, &speed) == MOTOR_FEEDBACK_OK);
    TEST_CHECK(fabsf(position + 3.141592653589793f) < 2e-6f);
    TEST_CHECK(fabsf(speed + 6.283185307179586f) < 4e-6f);

    position = 0.0f;
    speed = 0.0f;
    TEST_CHECK(MotorFeedbackConvertDegreesToOutputAxis(&config, 360.0f, 720.0f,
        &position, &speed) == MOTOR_FEEDBACK_OK);
    TEST_CHECK(fabsf(position + 3.141592653589793f) < 2e-6f);
    TEST_CHECK(fabsf(speed + 6.283185307179586f) < 4e-6f);

    position = 77.0f;
    speed = 88.0f;
    TEST_CHECK(MotorFeedbackConvertOutputAxis(&config, NAN, 1.0f,
        &position, &speed) == MOTOR_FEEDBACK_NONFINITE);
    TEST_CHECK(position == 77.0f && speed == 88.0f);

    const Motor_Output_Axis_Config_s tiny_ratio = {FLT_MIN, MOTOR_AXIS_DIRECTION_NORMAL};
    TEST_CHECK(MotorFeedbackConvertOutputAxis(&tiny_ratio, FLT_MAX, 1.0f,
        &position, &speed) == MOTOR_FEEDBACK_OVERFLOW);
    TEST_CHECK(position == 77.0f && speed == 88.0f);

    const Motor_Output_Axis_Config_s bad_direction = {1.0f, (Motor_Axis_Direction_e)0};
    TEST_CHECK(MotorFeedbackConvertOutputAxis(&bad_direction, 1.0f, 1.0f,
        &position, &speed) == MOTOR_FEEDBACK_BAD_CONFIG);
    TEST_CHECK(position == 77.0f && speed == 88.0f);
}

static void Capabilities(void)
{
    Motor_Capabilities_s capabilities;
    memset(&capabilities, 0xa5, sizeof(capabilities));
    TEST_CHECK(MotorFeedbackGetCapabilities(M3508, &capabilities) == MOTOR_FEEDBACK_OK);
    TEST_CHECK(capabilities.measurement_mask ==
        (MOTOR_CAPABILITY_POSITION | MOTOR_CAPABILITY_SPEED | MOTOR_CAPABILITY_CURRENT));
    TEST_CHECK(capabilities.position_unit == MOTOR_UNIT_RAD);
    TEST_CHECK(capabilities.speed_unit == MOTOR_UNIT_RAD_S);
    TEST_CHECK(capabilities.current_unit == MOTOR_UNIT_DEVICE_CURRENT);
    TEST_CHECK(capabilities.torque_unit == MOTOR_UNIT_UNSPECIFIED);
    TEST_CHECK(capabilities.voltage_unit == MOTOR_UNIT_UNSPECIFIED);

    TEST_CHECK(MotorFeedbackGetCapabilities(HT04, &capabilities) == MOTOR_FEEDBACK_OK);
    TEST_CHECK(capabilities.measurement_mask ==
        (MOTOR_CAPABILITY_POSITION | MOTOR_CAPABILITY_SPEED | MOTOR_CAPABILITY_TORQUE));
    TEST_CHECK(capabilities.current_unit == MOTOR_UNIT_UNSPECIFIED);
    TEST_CHECK(capabilities.torque_unit == MOTOR_UNIT_DEVICE_TORQUE);
    TEST_CHECK(capabilities.voltage_unit == MOTOR_UNIT_UNSPECIFIED);

    TEST_CHECK(MotorFeedbackGetCapabilities(XMCY, &capabilities) == MOTOR_FEEDBACK_UNSUPPORTED);
    TEST_CHECK(capabilities.measurement_mask ==
        (MOTOR_CAPABILITY_POSITION | MOTOR_CAPABILITY_SPEED | MOTOR_CAPABILITY_TORQUE));
    TEST_CHECK(MotorFeedbackGetCapabilities(MOTOR_TYPE_NONE, NULL) == MOTOR_FEEDBACK_BAD_ARGUMENT);
}

typedef struct
{
    Motor_Signal_Snapshot_s value;
    Motor_Feedback_Status_e status;
    uint32_t *calls;
} SignalTestSource_s;

static Motor_Feedback_Status_e ReadSignal(const void *context, Motor_Signal_Snapshot_s *output)
{
    const SignalTestSource_s *source = (const SignalTestSource_s *)context;
    if (source == NULL || output == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;
    (*source->calls)++;
    if (source->status != MOTOR_FEEDBACK_OK)
        return source->status;
    *output = source->value;
    return MOTOR_FEEDBACK_OK;
}

static uint8_t SignalEqual(const Motor_Signal_Snapshot_s *left,
                           const Motor_Signal_Snapshot_s *right)
{
    return (uint8_t)(left->value == right->value && left->unit == right->unit &&
                     left->reference == right->reference &&
                     left->source_time_ms == right->source_time_ms &&
                     left->sequence == right->sequence && left->valid == right->valid);
}

static void SignalSnapshot(void)
{
    uint32_t calls = 0u;
    SignalTestSource_s context = {
        .value = {
            .value = 1.25f,
            .unit = MOTOR_UNIT_RAD,
            .reference = MOTOR_REFERENCE_EXTERNAL,
            .source_time_ms = 100u,
            .sequence = 1u,
            .valid = 1u,
        },
        .status = MOTOR_FEEDBACK_OK,
        .calls = &calls,
    };
    Motor_Signal_Source_s source = {
        .read = ReadSignal,
        .context = &context,
        .max_age_ms = 50u,
        .unit = MOTOR_UNIT_RAD,
        .reference = MOTOR_REFERENCE_EXTERNAL,
    };
    Motor_Signal_Snapshot_s output = {
        .value = 9.0f,
        .unit = MOTOR_UNIT_VOLT,
        .reference = MOTOR_REFERENCE_ROTOR,
        .source_time_ms = 0xabcdef01u,
        .valid = 0u,
    };

    TEST_CHECK(MotorFeedbackReadSignal(&source, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL,
        150u, &output) == MOTOR_FEEDBACK_OK);
    TEST_CHECK(calls == 1u);
    TEST_CHECK(output.value == 1.25f && output.unit == MOTOR_UNIT_RAD);
    TEST_CHECK(output.reference == MOTOR_REFERENCE_EXTERNAL && output.source_time_ms == 100u);
    TEST_CHECK(output.valid == 1u);

    const Motor_Signal_Snapshot_s previous = output;
    TEST_CHECK(MotorFeedbackReadSignal(&source, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL,
        151u, &output) == MOTOR_FEEDBACK_STALE);
    TEST_CHECK(calls == 2u);
    TEST_CHECK(SignalEqual(&output, &previous));

    context.value.source_time_ms = 0u;
    TEST_CHECK(MotorFeedbackReadSignal(&source, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL,
        0u, &output) == MOTOR_FEEDBACK_OK);
    TEST_CHECK(calls == 3u && output.source_time_ms == 0u);
    output = previous;

    context.value.source_time_ms = 0xfffffff0u;
    TEST_CHECK(MotorFeedbackReadSignal(&source, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL,
        0x10u, &output) == MOTOR_FEEDBACK_OK);
    TEST_CHECK(calls == 4u && output.source_time_ms == 0xfffffff0u);
    output = previous;

    context.value.source_time_ms = 101u;
    TEST_CHECK(MotorFeedbackReadSignal(&source, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL,
        100u, &output) == MOTOR_FEEDBACK_STALE);
    TEST_CHECK(calls == 5u && SignalEqual(&output, &previous));

    context.value.source_time_ms = 100u;
    context.value.unit = MOTOR_UNIT_DEG;
    TEST_CHECK(MotorFeedbackReadSignal(&source, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL,
        100u, &output) == MOTOR_FEEDBACK_UNIT_MISMATCH);
    TEST_CHECK(calls == 6u && SignalEqual(&output, &previous));

    context.value.unit = MOTOR_UNIT_RAD;
    context.value.reference = MOTOR_REFERENCE_OUTPUT_AXIS;
    TEST_CHECK(MotorFeedbackReadSignal(&source, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL,
        100u, &output) == MOTOR_FEEDBACK_REFERENCE_MISMATCH);
    TEST_CHECK(calls == 7u && SignalEqual(&output, &previous));

    context.value.reference = MOTOR_REFERENCE_EXTERNAL;
    context.value.valid = 0u;
    TEST_CHECK(MotorFeedbackReadSignal(&source, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL,
        100u, &output) == MOTOR_FEEDBACK_INVALID_SNAPSHOT);
    TEST_CHECK(calls == 8u && SignalEqual(&output, &previous));

    context.value.valid = 1u;
    context.value.value = INFINITY;
    TEST_CHECK(MotorFeedbackReadSignal(&source, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL,
        100u, &output) == MOTOR_FEEDBACK_NONFINITE);
    TEST_CHECK(calls == 9u && SignalEqual(&output, &previous));

    context.value.value = 2.0f;
    context.status = MOTOR_FEEDBACK_BAD_CONFIG;
    TEST_CHECK(MotorFeedbackReadSignal(&source, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL,
        100u, &output) == MOTOR_FEEDBACK_SOURCE_ERROR);
    TEST_CHECK(calls == 10u && SignalEqual(&output, &previous));

    context.status = MOTOR_FEEDBACK_OK;
    source.max_age_ms = 0u;
    TEST_CHECK(MotorFeedbackReadSignal(&source, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL,
        100u, &output) == MOTOR_FEEDBACK_BAD_CONFIG);
    TEST_CHECK(calls == 10u && SignalEqual(&output, &previous));
    source.max_age_ms = 50u;
    TEST_CHECK(MotorFeedbackReadSignal(&source, MOTOR_UNIT_UNSPECIFIED,
        MOTOR_REFERENCE_EXTERNAL, 100u, &output) == MOTOR_FEEDBACK_BAD_CONFIG);
    TEST_CHECK(calls == 10u && SignalEqual(&output, &previous));
    source.max_age_ms = UINT32_MAX;
    TEST_CHECK(MotorFeedbackValidateSignalSource(&source) == MOTOR_FEEDBACK_BAD_CONFIG);
    TEST_CHECK(calls == 10u && SignalEqual(&output, &previous));
    source.max_age_ms = 50u;
    TEST_CHECK(calls == 10u && SignalEqual(&output, &previous));
    TEST_CHECK(MotorFeedbackReadSignal(NULL, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL,
        100u, &output) == MOTOR_FEEDBACK_BAD_ARGUMENT);
    TEST_CHECK(MotorFeedbackReadSignal(&source, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL,
        100u, NULL) == MOTOR_FEEDBACK_BAD_ARGUMENT);
    TEST_CHECK(SignalEqual(&output, &previous));
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "conversion") == 0)
        Conversion();
    else if (strcmp(argv[1], "capabilities") == 0)
        Capabilities();
    else if (strcmp(argv[1], "signal") == 0)
        SignalSnapshot();
    else
        TEST_CHECK(0);
    return 0;
}
