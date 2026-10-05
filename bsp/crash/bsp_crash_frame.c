#include "bsp_crash_frame.h"

#include <stddef.h>

#define CRASH_LOG_SNAPSHOT_WORDS 32u
#define CRASH_LOG_BASIC_FRAME_WORDS 8u
#define CRASH_LOG_FP_FRAME_PREFIX_WORDS 18u

/* The ranges are kept in sync with STM32H723VGTx_FLASH.ld. */
static uint8_t CrashLogGetStackRegionEnd(uintptr_t address, uintptr_t *region_end)
{
    if (address >= (uintptr_t)0x20000000u && address <= (uintptr_t)0x20020000u)
    {
        *region_end = (uintptr_t)0x20020000u;
        return 1u;
    }
    if (address >= (uintptr_t)0x24000000u && address <= (uintptr_t)0x24020000u)
    {
        *region_end = (uintptr_t)0x24020000u;
        return 1u;
    }
    if (address >= (uintptr_t)0x30000000u && address <= (uintptr_t)0x30008000u)
    {
        *region_end = (uintptr_t)0x30008000u;
        return 1u;
    }
    if (address >= (uintptr_t)0x38000000u && address <= (uintptr_t)0x38004000u)
    {
        *region_end = (uintptr_t)0x38004000u;
        return 1u;
    }
    return 0u;
}

static uint8_t CrashLogExceptionReturnValid(uint32_t exception_return)
{
    switch (exception_return)
    {
    case 0xFFFFFFF1u:
    case 0xFFFFFFF9u:
    case 0xFFFFFFFDu:
    case 0xFFFFFFE1u:
    case 0xFFFFFFE9u:
    case 0xFFFFFFEDu:
        return 1u;
    default:
        return 0u;
    }
}

uint32_t CrashLogReadableStackWords(uintptr_t stack_pointer)
{
    uintptr_t region_end;

    if ((stack_pointer & (sizeof(uint32_t) - 1u)) != 0u ||
        !CrashLogGetStackRegionEnd(stack_pointer, &region_end))
        return 0u;

    uintptr_t available_words = (region_end - stack_pointer) / sizeof(uint32_t);
    if (available_words > CRASH_LOG_SNAPSHOT_WORDS)
        return CRASH_LOG_SNAPSHOT_WORDS;
    return (uint32_t)available_words;
}

uint8_t CrashLogFrameResolve(uintptr_t stack_pointer, uint32_t exception_return,
                             uint32_t cfsr, CrashLogFrameView_t *view)
{
    const uint32_t frame_error_mask = (1u << 3u) |  /* MMFSR.MUNSTKERR */
                                      (1u << 4u) |  /* MMFSR.MSTKERR */
                                      (1u << 5u) |  /* MMFSR.MLSPERR */
                                      (1u << 11u) | /* BFSR.UNSTKERR */
                                      (1u << 12u) | /* BFSR.STKERR */
                                      (1u << 13u);  /* BFSR.LSPERR */
    const uint32_t fp_extended = (exception_return & (1u << 4u)) == 0u;
    const uint32_t prefix_words = fp_extended ? CRASH_LOG_FP_FRAME_PREFIX_WORDS : 0u;
    const uintptr_t frame_bytes = (uintptr_t)(prefix_words + CRASH_LOG_BASIC_FRAME_WORDS) *
                                  sizeof(uint32_t);
    uintptr_t region_end;

    if (view == NULL)
        return 0u;

    view->core_frame_address = 0u;
    view->snapshot_words = 0u;
    if ((stack_pointer & 7u) != 0u ||
        !CrashLogExceptionReturnValid(exception_return) ||
        (cfsr & frame_error_mask) != 0u ||
        !CrashLogGetStackRegionEnd(stack_pointer, &region_end) ||
        stack_pointer > region_end ||
        frame_bytes > (region_end - stack_pointer))
        return 0u;

    view->core_frame_address = stack_pointer + ((uintptr_t)prefix_words * sizeof(uint32_t));
    view->snapshot_words = CrashLogReadableStackWords(stack_pointer);
    return 1u;
}
