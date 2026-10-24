/** @file message_freshness.h
 * @brief 保存消息来源、首次接收和话题发布时间；读取不延长有效期限。
 */
#ifndef MESSAGE_FRESHNESS_H
#define MESSAGE_FRESHNESS_H

#include <stdint.h>

#define MESSAGE_DEFAULT_MAX_AGE_MS 50u

typedef struct
{
    uint8_t valid; /* 非0表示已发布过数据。 */
    uint32_t sequence; /* 每次成功发布递增，0保留为未发布。 */
    union
    {
        uint32_t source_time_ms; /* 来源产生时刻；REMOTE_RAW时仅记录，不与本机比较。 */
        uint32_t timestamp_ms; /* 兼容名称，与source_time_ms共用存储。 */
    };
    uint32_t received_time_ms; /* 本地首次接收时刻，转发不能改写。 */
    uint32_t published_time_ms; /* 本次写入话题时刻，查询不能改写。 */
    uint8_t source_clock; /* MESSAGE_CLOCK_*，不包含message_center.h以保持轻依赖。 */
} MessageFreshness_t;

/** @brief 检查消息是否仍在允许时间内。
 * @param[in] freshness 消息时间记录，NULL时返回0。
 * @param[in] now_ms 本机毫秒时刻，须与timestamp_ms使用同一时钟域。
 * @param[in] max_age_ms 允许毫秒数；等于期限时仍有效。
 * @return 已发布且未过时返回1，否则返回0；本函数不检查数据内容。
 * @note 远端原始时钟须先由调用方转换为本机时间；消息中心读取接口会按接收时刻判定REMOTE_RAW。
 */
static inline uint8_t MessageFreshnessIsValid(const MessageFreshness_t *freshness,
                                             uint32_t now_ms,
                                             uint32_t max_age_ms)
{
    return (uint8_t)(freshness != 0 && freshness->valid != 0u &&
                     (uint32_t)(now_ms - freshness->timestamp_ms) <= max_age_ms);
}

#endif
