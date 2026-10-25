/**
 * @file bsp_reset_recovery.c
 * @brief 将恢复策略记录放入崩溃保留区，并根据复位来源执行有限启动。
 * 记录位于NOLOAD AXI SRAM；提交时刷新H7 ECC的64位写缓存，系统复位后再校验CRC。
 */
#include "bsp_reset_recovery.h"

#include "bsp_crash_crc.h"
#include "main.h"

#include <stddef.h>

#define RESET_RECOVERY_SLOT_COUNT 2u
#define RESET_RECOVERY_CRC_WORDS 7u

static volatile ResetRecoveryRecord_t recovery_slots[RESET_RECOVERY_SLOT_COUNT]
    __attribute__((section(".crash_record.recovery"), used, aligned(8)));
static ResetRecoveryRecord_t recovery_state;
static uint8_t recovery_active_slot;
static uint8_t recovery_initialized;
static uint8_t recovery_locked_out = 1u;

_Static_assert(sizeof(ResetRecoveryRecord_t) == 32u, "恢复记录必须保持32字节固定布局");
_Static_assert((sizeof(ResetRecoveryRecord_t) % 8u) == 0u,
               "H723 AXI SRAM恢复记录必须按64位ECC字对齐");
_Static_assert(offsetof(ResetRecoveryRecord_t, header) == 0u &&
                   offsetof(ResetRecoveryRecord_t, sequence) == 8u &&
                   offsetof(ResetRecoveryRecord_t, consecutive_attempts) == 12u &&
                   offsetof(ResetRecoveryRecord_t, flags) == 16u &&
                   offsetof(ResetRecoveryRecord_t, last_cause) == 20u &&
                   offsetof(ResetRecoveryRecord_t, tail) == 24u,
               "恢复记录各字段必须位于对齐的ECC字边界");

static void BSPResetRecoveryGetCRCWords(const ResetRecoveryRecord_t *record,
                                        uint32_t words[RESET_RECOVERY_CRC_WORDS])
{
    words[0] = record->header.fields.magic;
    words[1] = record->header.fields.version;
    words[2] = record->sequence;
    words[3] = record->consecutive_attempts;
    words[4] = record->flags;
    words[5] = record->last_cause;
    words[6] = record->tail.fields.stable_since_ms;
}

static uint32_t BSPResetRecoveryEnterCritical(void)
{
    const uint32_t previous = __get_PRIMASK();
    __disable_irq();
    return previous;
}

static void BSPResetRecoveryExitCritical(uint32_t previous)
{
    __set_PRIMASK(previous);
}

static uint8_t BSPResetRecoverySlotValid(uint8_t index, ResetRecoveryRecord_t *output)
{
    if (index >= RESET_RECOVERY_SLOT_COUNT || output == NULL)
        return 0u;

    const volatile ResetRecoveryRecord_t *slot = &recovery_slots[index];
    ResetRecoveryRecord_t candidate = {
        .header = {.fields = {
            .magic = slot->header.fields.magic,
            .version = slot->header.fields.version,
        }},
        .sequence = slot->sequence,
        .consecutive_attempts = slot->consecutive_attempts,
        .flags = slot->flags,
        .last_cause = slot->last_cause,
        .tail = {.fields = {
            .stable_since_ms = slot->tail.fields.stable_since_ms,
            .crc32 = slot->tail.fields.crc32,
        }},
    };
    uint32_t words[RESET_RECOVERY_CRC_WORDS];
    BSPResetRecoveryGetCRCWords(&candidate, words);
    uint32_t actual_crc = 0u;
    if (candidate.header.fields.magic != RESET_RECOVERY_MAGIC ||
        candidate.header.fields.version != RESET_RECOVERY_VERSION ||
        !CrashCRC32Words(words, RESET_RECOVERY_CRC_WORDS, &actual_crc) ||
        actual_crc != candidate.tail.fields.crc32)
        return 0u;

    *output = candidate;
    return 1u;
}

static uint8_t BSPResetRecoverySlotBlank(uint8_t index)
{
    if (index >= RESET_RECOVERY_SLOT_COUNT)
        return 0u;

    const volatile ResetRecoveryRecord_t *slot = &recovery_slots[index];
    return slot->header.ecc_flush_word == 0u && slot->sequence == 0u &&
                   slot->consecutive_attempts == 0u && slot->flags == 0u &&
                   slot->last_cause == 0u && slot->tail.ecc_flush_word == 0u
               ? 1u
               : 0u;
}

static void BSPResetRecoverySeal(ResetRecoveryRecord_t *record)
{
    uint32_t words[RESET_RECOVERY_CRC_WORDS];
    BSPResetRecoveryGetCRCWords(record, words);
    uint32_t crc = 0u;
    (void)CrashCRC32Words(words, RESET_RECOVERY_CRC_WORDS, &crc);
    record->tail.fields.crc32 = crc;
}

static void BSPResetRecoveryStore(const ResetRecoveryRecord_t *record)
{
    const uint8_t target = (uint8_t)(recovery_active_slot ^ 1u);
    volatile ResetRecoveryRecord_t *slot = &recovery_slots[target];

    // 先让目标槽失效；另一槽保留上一份完整记录，以便掉电或复位时回退。
    slot->header.ecc_flush_word = (uint64_t)record->header.fields.version << 32u;
    slot->sequence = record->sequence;
    slot->consecutive_attempts = record->consecutive_attempts;
    slot->flags = record->flags;
    slot->last_cause = record->last_cause;
    slot->tail.ecc_flush_word = (uint64_t)record->tail.fields.stable_since_ms |
                                ((uint64_t)record->tail.fields.crc32 << 32u);
    __DSB();
    // 标记有效时直接写完整64位头，避免复位截断最后一个32位ECC写。
    slot->header.ecc_flush_word = (uint64_t)RESET_RECOVERY_MAGIC |
                                  ((uint64_t)record->header.fields.version << 32u);
    __DSB();

    recovery_active_slot = target;
    recovery_state = *record;
}

static uint8_t BSPResetRecoverySelectLatest(ResetRecoveryRecord_t *output,
                                            uint8_t *active_slot)
{
    ResetRecoveryRecord_t first;
    ResetRecoveryRecord_t second;
    const uint8_t first_valid = BSPResetRecoverySlotValid(0u, &first);
    const uint8_t second_valid = BSPResetRecoverySlotValid(1u, &second);

    if (first_valid == 0u && second_valid == 0u)
        return 0u;
    // 只有冷上电清零留下的空槽可以与有效槽并存；非空无效槽可能是
    // 上次故障记录被复位打断，不能静默回退并丢掉刚发生的故障。
    if ((first_valid == 0u && BSPResetRecoverySlotBlank(0u) == 0u) ||
        (second_valid == 0u && BSPResetRecoverySlotBlank(1u) == 0u))
        return 0u;
    if (second_valid == 0u ||
        (first_valid != 0u && (int32_t)(first.sequence - second.sequence) > 0))
    {
        *output = first;
        *active_slot = 0u;
    }
    else
    {
        *output = second;
        *active_slot = 1u;
    }
    return 1u;
}

uint8_t BSP_ResetRecoveryInit(uint32_t reset_flags)
{
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u)
    {
        // D-cache目前关闭；若后续打开，需先实现缓存清理再信任跨复位记录。
        recovery_initialized = 0u;
        recovery_locked_out = 1u;
        return 0u;
    }

    ResetRecoveryResetCause_t cause = {0u, 0u, 0u};
    cause.power_on = (reset_flags & RCC_RSR_PORRSTF) != 0u ? 1u : 0u;
    cause.brownout = (reset_flags & RCC_RSR_BORRSTF) != 0u ? 1u : 0u;
    cause.watchdog = (reset_flags & RCC_RSR_IWDG1RSTF) != 0u ? 1u : 0u;
#ifdef RCC_RSR_WWDG1RSTF
    if ((reset_flags & RCC_RSR_WWDG1RSTF) != 0u)
        cause.watchdog = 1u;
#endif

    uint8_t active_slot = 0u;
    ResetRecoveryRecord_t candidate = {0u};
    uint8_t record_valid = 0u;
    if (cause.power_on == 0u)
        record_valid = BSPResetRecoverySelectLatest(&candidate, &active_slot);
    if (cause.power_on != 0u)
    {
        // POR表示供电重新建立；作废旧槽，符合人工急停不跨断电保留的约定。
        for (uint8_t index = 0u; index < RESET_RECOVERY_SLOT_COUNT; ++index)
        {
            volatile ResetRecoveryRecord_t *slot = &recovery_slots[index];
            slot->header.ecc_flush_word = 0u;
            slot->sequence = 0u;
            slot->consecutive_attempts = 0u;
            slot->flags = 0u;
            slot->last_cause = 0u;
            slot->tail.ecc_flush_word = 0u;
            __DSB();
        }
    }

    const ResetRecoveryStatus_e result =
        ResetRecoveryPolicyOnBoot(&candidate, cause, record_valid);
    recovery_state = candidate;
    recovery_active_slot = active_slot;
    recovery_initialized = 1u;
    recovery_locked_out = result == RESET_RECOVERY_STATUS_LOCKED ? 1u : 0u;
    BSPResetRecoverySeal(&recovery_state);
    BSPResetRecoveryStore(&recovery_state);
    return 1u;
}

void BSP_ResetRecoveryMarkFault(ResetRecoveryCause_e cause)
{
    if (recovery_initialized == 0u || recovery_locked_out != 0u)
        return;

    const uint32_t previous = BSPResetRecoveryEnterCritical();
    ResetRecoveryPolicyMarkFault(&recovery_state, cause);
    BSPResetRecoverySeal(&recovery_state);
    BSPResetRecoveryStore(&recovery_state);
    BSPResetRecoveryExitCritical(previous);
}

void BSP_ResetRecoverySetManualEstop(uint8_t latched)
{
    if (recovery_initialized == 0u)
        return;

    const uint32_t previous = BSPResetRecoveryEnterCritical();
    ResetRecoveryPolicySetManualEstop(&recovery_state, latched);
    BSPResetRecoverySeal(&recovery_state);
    BSPResetRecoveryStore(&recovery_state);
    BSPResetRecoveryExitCritical(previous);
}

uint8_t BSP_ResetRecoveryGetState(BSP_ResetRecoveryState_t *output)
{
    if (output == NULL || recovery_initialized == 0u)
        return 0u;

    BSP_ResetRecoveryState_t candidate;
    const uint32_t previous = BSPResetRecoveryEnterCritical();
    candidate.consecutive_attempts = recovery_state.consecutive_attempts;
    candidate.last_cause = (ResetRecoveryCause_e)recovery_state.last_cause;
    candidate.locked_out = recovery_locked_out;
    candidate.manual_estop_latched =
        ResetRecoveryPolicyHasManualEstop(&recovery_state);
    BSPResetRecoveryExitCritical(previous);
    *output = candidate;
    return 1u;
}

uint8_t BSP_ResetRecoveryIsLockedOut(void)
{
    return recovery_locked_out;
}

uint8_t BSP_ResetRecoveryObserveControl(uint32_t now_ms,
                                        uint8_t control_ready,
                                        uint8_t all_core_healthy)
{
    if (recovery_initialized == 0u || recovery_locked_out != 0u)
        return 0u;

    const uint32_t previous = BSPResetRecoveryEnterCritical();
    const uint8_t changed = ResetRecoveryPolicyObserveControl(&recovery_state, now_ms,
                                                               control_ready,
                                                               all_core_healthy);
    if (changed != 0u)
    {
        BSPResetRecoverySeal(&recovery_state);
        BSPResetRecoveryStore(&recovery_state);
    }
    BSPResetRecoveryExitCritical(previous);
    return changed;
}

#ifdef MC02_RESET_RECOVERY_TESTING
/* 主机专用故障注入：模拟备用槽刚失效、尚未提交完整记录时发生复位。 */
void BSP_ResetRecoveryTestTearInactiveSlot(void)
{
    const uint8_t target = (uint8_t)(recovery_active_slot ^ 1u);
    recovery_slots[target].header.ecc_flush_word =
        (uint64_t)RESET_RECOVERY_VERSION << 32u;
    __DSB();
}
#endif
