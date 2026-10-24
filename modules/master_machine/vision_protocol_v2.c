/** @file vision_protocol_v2.c
 * @brief V2业务帧的字节编解码、CRC和有界流解析。
 * @note 线上数值显式按小端处理；解析器只在完整帧校验通过后调用业务回调。
 */
#include "vision_protocol_v2.h"

#include <math.h>
#include <float.h>
#include <string.h>
#include "crc.h"

#define VISION_V2_AIM_PAYLOAD_SIZE 25u
#define VISION_V2_NAV_PAYLOAD_SIZE 13u
#define VISION_V2_POSTURE_PAYLOAD_SIZE 34u
#define VISION_V2_REFEREE_PAYLOAD_SIZE 36u

_Static_assert(sizeof(float) == 4u && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
               FLT_MAX_EXP == 128, "V2 wire format requires IEEE-754 binary32");

static uint16_t ReadU16LE(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t ReadU32LE(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void WriteU16LE(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void WriteU32LE(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static float ReadF32LE(const uint8_t *data)
{
    const uint32_t bits = ReadU32LE(data);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void WriteF32LE(uint8_t *data, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    WriteU32LE(data, bits);
}

VisionV2Status_t VisionV2GetPayloadSize(uint8_t type, size_t *length)
{
    if (length == NULL)
        return VISION_V2_ERR_ARGUMENT;
    size_t candidate;
    switch (type)
    {
    case VISION_V2_AIM_COMMAND: candidate = VISION_V2_AIM_PAYLOAD_SIZE; break;
    case VISION_V2_NAVIGATION_COMMAND: candidate = VISION_V2_NAV_PAYLOAD_SIZE; break;
    case VISION_V2_POSTURE_STATUS: candidate = VISION_V2_POSTURE_PAYLOAD_SIZE; break;
    case VISION_V2_REFEREE_SUMMARY: candidate = VISION_V2_REFEREE_PAYLOAD_SIZE; break;
    default: return VISION_V2_ERR_TYPE;
    }
    *length = candidate;
    return VISION_V2_OK;
}

VisionV2Status_t VisionV2EncodeFrame(uint8_t type, uint16_t sequence,
                                    uint32_t source_time_ms, const uint8_t *payload,
                                    size_t payload_length, uint8_t *output,
                                    size_t *output_length)
{
    if (payload == NULL || output == NULL || output_length == NULL)
        return VISION_V2_ERR_ARGUMENT;
    size_t expected_length = 0u;
    VisionV2Status_t status = VisionV2GetPayloadSize(type, &expected_length);
    if (status != VISION_V2_OK)
        return status;
    if (payload_length != expected_length || payload_length > VISION_V2_MAX_PAYLOAD)
        return VISION_V2_ERR_LENGTH;

    uint8_t candidate[VISION_V2_MAX_FRAME];
    candidate[0] = (uint8_t)'M';
    candidate[1] = (uint8_t)'C';
    candidate[2] = VISION_V2_VERSION;
    candidate[3] = type;
    WriteU16LE(&candidate[4], (uint16_t)payload_length);
    WriteU16LE(&candidate[6], sequence);
    WriteU32LE(&candidate[8], source_time_ms);
    memcpy(&candidate[VISION_V2_HEADER_SIZE], payload, payload_length);
    const size_t crc_offset = VISION_V2_HEADER_SIZE + payload_length;
    uint32_t crc = 0u;
    if (CRCCompute(CRC16_MODBUS_A5, candidate, crc_offset, &crc) != CRC_OK)
        return VISION_V2_ERR_CRC;
    WriteU16LE(&candidate[crc_offset], (uint16_t)crc);
    memcpy(output, candidate, crc_offset + VISION_V2_CRC_SIZE);
    *output_length = crc_offset + VISION_V2_CRC_SIZE;
    return VISION_V2_OK;
}

VisionV2Status_t VisionV2DecodeAim(const uint8_t *payload, size_t length,
                                   VisionV2AimCommand_t *command)
{
    if (payload == NULL || command == NULL)
        return VISION_V2_ERR_ARGUMENT;
    if (length != VISION_V2_AIM_PAYLOAD_SIZE)
        return VISION_V2_ERR_LENGTH;
    VisionV2AimCommand_t candidate = {
        .mode = payload[0],
        .yaw_rad = ReadF32LE(&payload[1]),
        .yaw_rate_rad_s = ReadF32LE(&payload[5]),
        .yaw_accel_rad_s2 = ReadF32LE(&payload[9]),
        .pitch_rad = ReadF32LE(&payload[13]),
        .pitch_rate_rad_s = ReadF32LE(&payload[17]),
        .pitch_accel_rad_s2 = ReadF32LE(&payload[21]),
    };
    if (candidate.mode > 1u || !isfinite(candidate.yaw_rad) ||
        !isfinite(candidate.yaw_rate_rad_s) || !isfinite(candidate.yaw_accel_rad_s2) ||
        !isfinite(candidate.pitch_rad) || !isfinite(candidate.pitch_rate_rad_s) ||
        !isfinite(candidate.pitch_accel_rad_s2))
        return VISION_V2_ERR_VALUE;
    *command = candidate;
    return VISION_V2_OK;
}

VisionV2Status_t VisionV2DecodeNavigation(const uint8_t *payload, size_t length,
                                          VisionV2NavigationCommand_t *command)
{
    if (payload == NULL || command == NULL)
        return VISION_V2_ERR_ARGUMENT;
    if (length != VISION_V2_NAV_PAYLOAD_SIZE)
        return VISION_V2_ERR_LENGTH;
    VisionV2NavigationCommand_t candidate = {
        .valid_flags = payload[0],
        .vx_m_s = ReadF32LE(&payload[1]),
        .vy_m_s = ReadF32LE(&payload[5]),
        .yaw_rate_rad_s = ReadF32LE(&payload[9]),
    };
    if ((candidate.valid_flags & (uint8_t)~(VISION_V2_NAV_TRANSLATION_VALID |
                                           VISION_V2_NAV_YAW_RATE_VALID)) != 0u ||
        !isfinite(candidate.vx_m_s) || !isfinite(candidate.vy_m_s) ||
        !isfinite(candidate.yaw_rate_rad_s) || fabsf(candidate.vx_m_s) > 2.0f ||
        fabsf(candidate.vy_m_s) > 2.0f || fabsf(candidate.yaw_rate_rad_s) > 10.0f)
        return VISION_V2_ERR_VALUE;
    *command = candidate;
    return VISION_V2_OK;
}

VisionV2Status_t VisionV2EncodePosture(const VisionV2PostureStatus_t *status,
                                       uint8_t *payload, size_t capacity)
{
    if (status == NULL || payload == NULL)
        return VISION_V2_ERR_ARGUMENT;
    if (capacity < VISION_V2_POSTURE_PAYLOAD_SIZE)
        return VISION_V2_ERR_LENGTH;
    if ((status->valid_flags & (uint8_t)~(VISION_V2_POSTURE_ORIENTATION_VALID |
                                          VISION_V2_POSTURE_RATE_VALID)) != 0u ||
        status->robot_state > 1u)
        return VISION_V2_ERR_VALUE;
    for (size_t i = 0u; i < 4u; ++i)
        if (!isfinite(status->quaternion_wxyz[i]))
            return VISION_V2_ERR_VALUE;
    if (!isfinite(status->yaw_rad) || !isfinite(status->yaw_rate_rad_s) ||
        !isfinite(status->pitch_rad) || !isfinite(status->pitch_rate_rad_s))
        return VISION_V2_ERR_VALUE;

    uint8_t candidate[VISION_V2_POSTURE_PAYLOAD_SIZE];
    candidate[0] = status->valid_flags;
    candidate[1] = status->robot_state;
    size_t offset = 2u;
    for (size_t i = 0u; i < 4u; ++i, offset += 4u)
        WriteF32LE(&candidate[offset], status->quaternion_wxyz[i]);
    WriteF32LE(&candidate[offset], status->yaw_rad); offset += 4u;
    WriteF32LE(&candidate[offset], status->yaw_rate_rad_s); offset += 4u;
    WriteF32LE(&candidate[offset], status->pitch_rad); offset += 4u;
    WriteF32LE(&candidate[offset], status->pitch_rate_rad_s);
    memcpy(payload, candidate, sizeof(candidate));
    return VISION_V2_OK;
}

VisionV2Status_t VisionV2EncodeRefereeSummary(const VisionV2RefereeSummary_t *summary,
                                              uint8_t *payload, size_t capacity)
{
    if (summary == NULL || payload == NULL)
        return VISION_V2_ERR_ARGUMENT;
    if (capacity < VISION_V2_REFEREE_PAYLOAD_SIZE)
        return VISION_V2_ERR_LENGTH;
    if ((summary->valid_flags & (uint8_t)~(VISION_V2_REF_ROBOT_STATE_VALID |
                                           VISION_V2_REF_POWER_HEAT_VALID |
                                           VISION_V2_REF_SHOOT_RECORD_VALID)) != 0u ||
        (summary->power_output_flags & (uint8_t)~0x07u) != 0u)
        return VISION_V2_ERR_VALUE;
    if (!isfinite(summary->bullet_speed_limit_m_s) ||
        !isfinite(summary->last_shot_speed_m_s))
        return VISION_V2_ERR_VALUE;

    uint8_t candidate[VISION_V2_REFEREE_PAYLOAD_SIZE];
    size_t offset = 0u;
    candidate[offset++] = summary->valid_flags;
    candidate[offset++] = summary->robot_id;
    WriteU16LE(&candidate[offset], summary->current_hp); offset += 2u;
    WriteU16LE(&candidate[offset], summary->maximum_hp); offset += 2u;
    WriteU16LE(&candidate[offset], summary->chassis_power_limit_w); offset += 2u;
    WriteF32LE(&candidate[offset], summary->bullet_speed_limit_m_s); offset += 4u;
    candidate[offset++] = summary->power_output_flags;
    WriteU16LE(&candidate[offset], summary->buffer_energy_j); offset += 2u;
    WriteU16LE(&candidate[offset], summary->heat_17mm); offset += 2u;
    WriteU16LE(&candidate[offset], summary->heat_42mm); offset += 2u;
    candidate[offset++] = summary->last_shot_frequency_hz;
    WriteF32LE(&candidate[offset], summary->last_shot_speed_m_s); offset += 4u;
    WriteU32LE(&candidate[offset], summary->robot_state_source_ms); offset += 4u;
    WriteU32LE(&candidate[offset], summary->power_heat_source_ms); offset += 4u;
    WriteU32LE(&candidate[offset], summary->shoot_record_source_ms);
    memcpy(payload, candidate, sizeof(candidate));
    return VISION_V2_OK;
}

void VisionV2StreamReset(VisionV2Stream_t *stream)
{
    if (stream != NULL)
        memset(stream, 0, sizeof(*stream));
}

static void RemovePrefix(VisionV2Stream_t *stream, size_t count)
{
    if (count >= stream->used)
    {
        stream->used = 0u;
        return;
    }
    const size_t remaining = stream->used - count;
    memmove(stream->bytes, &stream->bytes[count], remaining);
    memmove(stream->byte_time_ms, &stream->byte_time_ms[count],
            remaining * sizeof(stream->byte_time_ms[0]));
    stream->used = remaining;
}

static void ProcessAvailable(VisionV2Stream_t *stream, uint32_t generation,
                             VisionV2FrameCallback_t complete, void *context,
                             uint32_t *errors)
{
    while (stream->used > 0u)
    {
        if (stream->bytes[0] != (uint8_t)'M')
        {
            RemovePrefix(stream, 1u);
            continue;
        }
        if (stream->used < 2u)
            return;
        if (stream->bytes[1] != (uint8_t)'C')
        {
            RemovePrefix(stream, 1u);
            continue;
        }
        if (stream->used < VISION_V2_HEADER_SIZE)
            return;

        size_t expected_payload = 0u;
        if (stream->bytes[2] != VISION_V2_VERSION ||
            VisionV2GetPayloadSize(stream->bytes[3], &expected_payload) != VISION_V2_OK ||
            ReadU16LE(&stream->bytes[4]) != expected_payload)
        {
            (*errors)++;
            RemovePrefix(stream, 1u);
            continue;
        }
        const size_t frame_length = VISION_V2_HEADER_SIZE + expected_payload + VISION_V2_CRC_SIZE;
        if (stream->used < frame_length)
            return;

        uint32_t computed_crc = 0u;
        const uint16_t received_crc = ReadU16LE(&stream->bytes[frame_length - VISION_V2_CRC_SIZE]);
        if (CRCCompute(CRC16_MODBUS_A5, stream->bytes,
                       frame_length - VISION_V2_CRC_SIZE, &computed_crc) != CRC_OK ||
            (uint16_t)computed_crc != received_crc)
        {
            (*errors)++;
            RemovePrefix(stream, 1u);
            continue;
        }

        VisionV2Frame_t frame;
        memset(&frame, 0, sizeof(frame));
        frame.type = stream->bytes[3];
        frame.payload_length = (uint16_t)expected_payload;
        frame.sequence = ReadU16LE(&stream->bytes[6]);
        frame.source_time_ms = ReadU32LE(&stream->bytes[8]);
        frame.received_time_ms = stream->byte_time_ms[0];
        frame.generation = generation;
        memcpy(frame.payload, &stream->bytes[VISION_V2_HEADER_SIZE], expected_payload);
        complete(context, &frame);
        RemovePrefix(stream, frame_length);
    }
}

VisionV2Status_t VisionV2StreamFeed(VisionV2Stream_t *stream, const uint8_t *data,
                                   size_t length, uint32_t received_ms,
                                   uint32_t block_sequence, uint32_t generation,
                                   uint32_t now_ms, VisionV2FrameCallback_t complete,
                                   void *context, uint32_t *frame_errors)
{
    if (stream == NULL || data == NULL || complete == NULL || length == 0u ||
        length > VISION_V2_STREAM_BLOCK_MAX)
        return VISION_V2_ERR_ARGUMENT;

    uint32_t errors = 0u;
    if (stream->used > 0u &&
        (stream->generation != generation ||
         (uint32_t)(block_sequence - stream->previous_block_sequence) != 1u ||
         (uint32_t)(now_ms - stream->byte_time_ms[0]) > VISION_V2_ASSEMBLY_TIMEOUT_MS))
        stream->used = 0u;

    stream->generation = generation;
    stream->previous_block_sequence = block_sequence;
    stream->have_previous_block = 1u;
    for (size_t i = 0u; i < length; ++i)
    {
        if (stream->used == VISION_V2_MAX_FRAME)
        {
            errors++;
            RemovePrefix(stream, 1u);
        }
        stream->bytes[stream->used] = data[i];
        stream->byte_time_ms[stream->used] = received_ms;
        stream->used++;
        ProcessAvailable(stream, generation, complete, context, &errors);
    }
    if (frame_errors != NULL)
        *frame_errors = errors;
    return VISION_V2_OK;
}
