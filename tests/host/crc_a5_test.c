/** @file crc_a5_test.c
 * @brief 用完整 A5 帧验证归档协议组包／解析和 CRC 错误拒绝；无硬件依赖。
 */
#include "seasky_protocol.h"
#include "test_check.h"
#include "crc_injection.h"
#include <string.h>

int main(void)
{
    float input = 1.0f, output = 0.0f;
    uint8_t frame[32] = {0};
    uint16_t length = 99u, flags = 0u;
    TEST_CHECK(get_protocol_send_data(0x2345u, 0x1234u, &input, 1u, frame, &length) == CRC_OK);
    TEST_CHECK(length == 14u);
    const uint8_t legacy[] = {0xa5, 0x06, 0x00, 0x99, 0x45, 0x23, 0x34, 0x12, 0x00, 0x00, 0x80, 0x3f, 0x78, 0x7b};
    TEST_CHECK(memcmp(frame, legacy, sizeof(legacy)) == 0);
    // 头部和尾部计算分别失败时，长度都为0，调用方不能发送部分帧。
    for (int failure = 0; failure < 2; ++failure)
    {
        crc_fail_after = failure;
        TEST_CHECK(get_protocol_send_data(0x2345u, 0x1234u, &input, 1u, frame, &length) != CRC_OK);
        TEST_CHECK(length == 0u);
    }
    TEST_CHECK(get_protocol_send_data(0x2345u, 0x1234u, &input, 1u, frame, &length) == CRC_OK);
    TEST_CHECK(get_protocol_info(frame, &flags, (uint8_t *)&output) == 0x2345u);
    TEST_CHECK(flags == 0x1234u && output == input);
    // 损坏数据后保持调用者上一份有效值，不发布未通过检查的内容。
    frame[9] ^= 1u;
    flags = 0x4567u; output = 5.0f;
    TEST_CHECK(get_protocol_info(frame, &flags, (uint8_t *)&output) == 0u);
    TEST_CHECK(flags == 0x4567u && output == 5.0f);
    TEST_CHECK(get_protocol_send_data(1u, 0u, NULL, 1u, frame, &length) == CRC_ERROR_ARGUMENT);
    TEST_CHECK(length == 0u);
    return 0;
}
