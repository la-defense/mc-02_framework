/** @file message_freshness.h
 * @brief 保存消息是否有效、发布序号及发布时间，供各控制模块统一检查旧数据。
 */
#ifndef MESSAGE_FRESHNESS_H
#define MESSAGE_FRESHNESS_H

#include <stdint.h>

#define MESSAGE_DEFAULT_MAX_AGE_MS 50u

typedef struct
{
    uint8_t valid; // 非 0 表示已发布过可用数据。
    uint32_t sequence; // 发布时加 1；检查函数不要求序号非 0，也不与上次读取比较。
    uint32_t timestamp_ms; // 数据产生或接收时的毫秒值，读取不能更新它。
} MessageFreshness_t;

/** @brief 检查已标记有效的消息是否仍在允许时间内。
 * @param[in] freshness 消息时间记录，NULL 时返回 0。
 * @param[in] now_ms 当前毫秒计数，须与 timestamp_ms 使用同一时间来源。
 * @param[in] max_age_ms 允许过去的毫秒数，常用默认值为 50；等于时限也允许。
 * @return 1 为可用，0 为未发布、指针为空或已到时；只检查标记和时间，不检查数据内容。
 */
static inline uint8_t MessageFreshnessIsValid(const MessageFreshness_t *freshness,
                                             uint32_t now_ms,
                                             uint32_t max_age_ms)
{
    return (uint8_t)(freshness != 0 && freshness->valid != 0u &&
                     (uint32_t)(now_ms - freshness->timestamp_ms) <= max_age_ms);
}

#endif
