/**
 * @file bsp_crash_frame_test.c
 * @brief 用链接范围模型检查异常寄存器帧和栈读取边界，不访问目标硬件。
 * @author MC-02 contributors
 * @license BSD-3-Clause; see LICENSE.
 */
#include <stdint.h>
#include <stdio.h>

#include "bsp_crash_frame.h"

static const CrashMemoryRegion_t kRegions[CRASH_MEMORY_REGION_CAPACITY] = {
    {0x08000000u, 0x080C0000u, 0u},
    {0x20000000u, 0x20020000u, 1u},
    {0x24000000u, 0x24020000u, 1u},
    {0x30000000u, 0x30008000u, 1u},
    {0x38000000u, 0x38004000u, 1u},
};
static const CrashMemoryMap_t kMap = {kRegions, CRASH_MEMORY_REGION_CAPACITY};

#define CHECK(condition)                                                         \
    do                                                                           \
    {                                                                            \
        if (!(condition))                                                        \
        {                                                                        \
            fprintf(stderr, "FAIL:%d: %s\n", __LINE__, #condition);             \
            return 1;                                                            \
        }                                                                        \
    } while (0)

static int TestBasicFrameAndBoundedSnapshot(void)
{
    CrashLogFrameView_t view = {0};
    CHECK(CrashLogFrameResolve(&kMap, 0x20001000u, 0xFFFFFFF9u, 0u, &view));
    CHECK(view.core_frame_address == 0x20001000u);
    CHECK(view.snapshot_words == 32u);
    CHECK(CrashLogReadableStackWords(&kMap, 0x2001FFF0u) == 4u);
    return 0;
}

static int TestExtendedFloatingPointFrame(void)
{
    CrashLogFrameView_t view = {0};
    CHECK(CrashLogFrameResolve(&kMap, 0x24001000u, 0xFFFFFFEDu, 0u, &view));
    CHECK(view.core_frame_address == 0x24001048u);
    CHECK(view.snapshot_words == 32u);
    return 0;
}

static int TestInvalidOrFaultedFrameIsRejected(void)
{
    CrashLogFrameView_t view = {0};
    static const uint32_t stacking_faults[] = {
        1u << 3u,  /* MMFSR.MUNSTKERR */
        1u << 4u,  /* MMFSR.MSTKERR */
        1u << 5u,  /* MMFSR.MLSPERR */
        1u << 11u, /* BFSR.UNSTKERR */
        1u << 12u, /* BFSR.STKERR */
        1u << 13u, /* BFSR.LSPERR */
    };

    CHECK(!CrashLogFrameResolve(&kMap, 0x20020000u, 0xFFFFFFF9u, 0u, &view));
    CHECK(!CrashLogFrameResolve(&kMap, 0x2001FFF0u, 0xFFFFFFF9u, 0u, &view));
    CHECK(!CrashLogFrameResolve(&kMap, 0x20001004u, 0xFFFFFFF9u, 0u, &view));
    CHECK(!CrashLogFrameResolve(&kMap, 0x08001000u, 0xFFFFFFF9u, 0u, &view));
    CHECK(!CrashLogFrameResolve(&kMap, 0x20001002u, 0xFFFFFFF9u, 0u, &view));
    CHECK(!CrashLogFrameResolve(&kMap, 0x20001000u, 0x00000010u, 0u, &view));
    CHECK(!CrashLogFrameResolve(&kMap, UINTPTR_MAX - 3u, 0u, 0u, &view));
    for (uint32_t i = 0u; i < (sizeof(stacking_faults) / sizeof(stacking_faults[0])); ++i)
        CHECK(!CrashLogFrameResolve(&kMap, 0x20001000u, 0xFFFFFFF9u, stacking_faults[i], &view));
    CHECK(!CrashLogFrameResolve(&kMap, 0x38003FC0u, 0xFFFFFFEDu, 0u, &view));
    CHECK(CrashLogReadableStackWords(&kMap, 0x24020000u) == 0u);
    return 0;
}

static int TestLinkerSuppliedRangeChecks(void)
{
    const CrashMemoryRegion_t overlap[] = {{0x1000u, 0x2000u}, {0x1F00u, 0x3000u}};
    const CrashMemoryRegion_t invalid_stack_flag[] = {{0x1000u, 0x2000u, 2u}};
    const CrashMemoryMap_t invalid_map = {overlap, 2u};
    const CrashMemoryMap_t invalid_flag_map = {invalid_stack_flag, 1u};

    CHECK(CrashLogRangeReadable(&kMap, 0x08000000u, 4u, 4u));
    CHECK(CrashLogReadableStackWords(&kMap, 0x08001000u) == 0u);
    CHECK(CrashLogRangeReadable(&kMap, 0x2001FFF0u, 16u, 4u));
    CHECK(!CrashLogRangeReadable(&kMap, 0x2001FFF0u, 20u, 4u));
    CHECK(!CrashLogRangeReadable(&kMap, 0x080BFFF0u, 32u, 4u));
    CHECK(!CrashLogRangeReadable(&kMap, 0x20000001u, 4u, 4u));
    CHECK(!CrashLogRangeReadable(&kMap, UINTPTR_MAX - 3u, 8u, 4u));
    CHECK(!CrashLogRangeReadable(&invalid_map, 0x1100u, 4u, 4u));
    CHECK(!CrashLogRangeReadable(&invalid_flag_map, 0x1100u, 4u, 4u));
    CHECK(!CrashLogRangeReadable(NULL, 0x1100u, 4u, 4u));
    CHECK(CrashLogReadableStackWords(&kMap, 0x38003FC0u) == 16u);
    return 0;
}

int main(void)
{
    if (TestBasicFrameAndBoundedSnapshot() != 0 ||
        TestExtendedFloatingPointFrame() != 0 ||
        TestInvalidOrFaultedFrameIsRejected() != 0 ||
        TestLinkerSuppliedRangeChecks() != 0)
        return 1;

    puts("PASS: exception frames and stack snapshots are range checked");
    return 0;
}
