#ifndef MESSAGE_FRESHNESS_H
#define MESSAGE_FRESHNESS_H

#include <stdint.h>

#define MESSAGE_DEFAULT_MAX_AGE_MS 50u

typedef struct
{
    uint8_t valid;
    uint32_t sequence;
    uint32_t timestamp_ms;
} MessageFreshness_t;

static inline uint8_t MessageFreshnessIsValid(const MessageFreshness_t *freshness,
                                             uint32_t now_ms,
                                             uint32_t max_age_ms)
{
    return (uint8_t)(freshness != 0 && freshness->valid != 0u &&
                     (uint32_t)(now_ms - freshness->timestamp_ms) <= max_age_ms);
}

#endif
