#include <stdint.h>
#include <stdio.h>

#include "bsp_crash_frame.h"

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
    CHECK(CrashLogFrameResolve(0x20001000u, 0xFFFFFFF9u, 0u, &view));
    CHECK(view.core_frame_address == 0x20001000u);
    CHECK(view.snapshot_words == 32u);
    CHECK(CrashLogReadableStackWords(0x2001FFF0u) == 4u);
    return 0;
}

static int TestExtendedFloatingPointFrame(void)
{
    CrashLogFrameView_t view = {0};
    CHECK(CrashLogFrameResolve(0x24001000u, 0xFFFFFFEDu, 0u, &view));
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

    CHECK(!CrashLogFrameResolve(0x20020000u, 0xFFFFFFF9u, 0u, &view));
    CHECK(!CrashLogFrameResolve(0x2001FFF0u, 0xFFFFFFF9u, 0u, &view));
    CHECK(!CrashLogFrameResolve(0x20001004u, 0xFFFFFFF9u, 0u, &view));
    CHECK(!CrashLogFrameResolve(0x20001002u, 0xFFFFFFF9u, 0u, &view));
    CHECK(!CrashLogFrameResolve(0x20001000u, 0x00000010u, 0u, &view));
    CHECK(!CrashLogFrameResolve(UINTPTR_MAX - 3u, 0u, 0u, &view));
    for (uint32_t i = 0u; i < (sizeof(stacking_faults) / sizeof(stacking_faults[0])); ++i)
        CHECK(!CrashLogFrameResolve(0x20001000u, 0xFFFFFFF9u, stacking_faults[i], &view));
    CHECK(!CrashLogFrameResolve(0x38003FC0u, 0xFFFFFFEDu, 0u, &view));
    CHECK(CrashLogReadableStackWords(0x24020000u) == 0u);
    return 0;
}

int main(void)
{
    if (TestBasicFrameAndBoundedSnapshot() != 0 ||
        TestExtendedFloatingPointFrame() != 0 ||
        TestInvalidOrFaultedFrameIsRejected() != 0)
        return 1;

    puts("PASS: exception frames and stack snapshots are range checked");
    return 0;
}
