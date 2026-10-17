/** @file bsp_crash_crc.h
 * @brief 异常入口使用的独立 CRC32 逐位计算；不访问普通 CRC 表和运行时服务。
 * @note 输入长度必须由固定记录布局得到，不能采用损坏记录声明的长度。
 * 使用本工程 GCC 的 always_inline，避免在异常入口增加外部计算调用。
 */
#ifndef MC02_CRASH_CRC_H
#define MC02_CRASH_CRC_H
#include <stddef.h>
#include <stdint.h>

/** @brief 按低字节在前处理每个32位字，与旧异常记录保持一致。
 * @param[in] words 可读且4字节对齐的记录数据；仅 count 为0时允许 NULL。
 * @param[in] count 固定布局包含的32位字数，每字执行32次位处理。
 * @param[out] output 接收 CRC32；失败保持原值，不得与输入重叠。
 * @return 1表示成功，0表示空参数；无初始化、分配、日志或查表依赖。
 */
static inline __attribute__((always_inline)) uint8_t CrashCRC32Words(
    const volatile uint32_t *words, size_t count, uint32_t *output)
{
    if (output == NULL || (words == NULL && count != 0u)) return 0u;
    uint32_t crc = 0xffffffffu;
    // 只读取调用者按固定记录长度指定的字，每字依次处理四个低位字节。
    for (size_t index = 0u; index < count; ++index)
    {
        uint32_t word = words[index];
        for (unsigned byte = 0u; byte < 4u; ++byte)
        {
            crc ^= word & 0xffu;
            word >>= 8u;
            for (unsigned bit = 0u; bit < 8u; ++bit)
                crc = (crc >> 1u) ^ ((crc & 1u) ? 0xedb88320u : 0u);
        }
    }
    *output = ~crc;
    return 1u;
}
#endif
