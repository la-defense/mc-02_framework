#ifndef BSP_CRASH_FRAME_H
#define BSP_CRASH_FRAME_H

#include <stdint.h>

typedef struct
{
    uintptr_t core_frame_address;
    uint32_t snapshot_words;
} CrashLogFrameView_t;

/* Resolve an exception stack frame only when its full basic or FP-extended
 * frame lies in linker-configured on-chip SRAM and CFSR reports no frame error.
 */
uint8_t CrashLogFrameResolve(uintptr_t stack_pointer, uint32_t exception_return,
                             uint32_t cfsr, CrashLogFrameView_t *view);

/* Number of snapshot words readable before reaching the end of on-chip SRAM. */
uint32_t CrashLogReadableStackWords(uintptr_t stack_pointer);

#endif
