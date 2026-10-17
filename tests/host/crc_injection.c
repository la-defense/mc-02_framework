/** @file crc_injection.c
 * @brief 用 GNU 链接包装注入一次 CRC 错误，其余调用仍执行生产算法。
 */
#include "crc.h"
#include "crc_injection.h"

int crc_fail_after = -1;
CRCStatus_t __real_CRCCompute(CRCAlgorithm_t algorithm, const uint8_t *data,
                             size_t length, uint32_t *output);

// 不改输出，让调用方测试确认它没有误用上一次的计算值。
CRCStatus_t __wrap_CRCCompute(CRCAlgorithm_t algorithm, const uint8_t *data,
                             size_t length, uint32_t *output)
{
    if (crc_fail_after == 0)
    {
        crc_fail_after = -1;
        return CRC_ERROR_STATE;
    }
    if (crc_fail_after > 0) --crc_fail_after;
    return __real_CRCCompute(algorithm, data, length, output);
}
