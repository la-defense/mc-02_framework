/**
 * @file bsp_crash_frame.c
 * @brief 为 bsp_crash.c 检查栈地址和寄存器保存区域，接口用法见 bsp_crash_frame.h。
 * 这里只计算读取位置和数量，检查失败时让调用方跳过对应内存，避免记录故障时再次读错地址。
 */
#include "bsp_crash_frame.h"

#include <stddef.h>

// 以下数量均以 uint32_t 计，每项 4 字节。
#define CRASH_LOG_SNAPSHOT_WORDS 32u        // 崩溃记录最多保存的栈数据个数
#define CRASH_LOG_BASIC_FRAME_WORDS 8u      // R0～R3、R12、LR、PC、xPSR
#define CRASH_LOG_FP_FRAME_PREFIX_WORDS 18u // 基本寄存器前的浮点相关区域

static uint8_t CrashLogMemoryMapValid(const CrashMemoryMap_t *map)
{
    if (map == NULL || map->regions == NULL || map->region_count == 0u ||
        map->region_count > CRASH_MEMORY_REGION_CAPACITY)
        return 0u;

    for (uint32_t index = 0u; index < map->region_count; ++index)
    {
        const CrashMemoryRegion_t *region = &map->regions[index];
        if (region->start >= region->end || region->is_stack_memory > 1u ||
            (index > 0u && map->regions[index - 1u].end > region->start))
            return 0u;
    }
    return 1u;
}

// 通用读取可以使用只读Flash；异常栈帧还必须落在标记为RAM的区域。
static uint8_t CrashLogRangeReadableInRegions(const CrashMemoryMap_t *map, uintptr_t address,
                                             size_t length, size_t alignment,
                                             uint8_t stack_only)
{
    uintptr_t requested_end;

    if (!CrashLogMemoryMapValid(map) || length == 0u ||
        (alignment != 1u && alignment != 2u && alignment != 4u && alignment != 8u) ||
        (address % alignment) != 0u || length > UINTPTR_MAX - address)
        return 0u;

    requested_end = address + length;
    for (uint32_t index = 0u; index < map->region_count; ++index)
    {
        const CrashMemoryRegion_t *region = &map->regions[index];

        if (address >= region->start && requested_end <= region->end &&
            (!stack_only || region->is_stack_memory != 0u))
            return 1u;
    }
    return 0u;
}

uint8_t CrashLogRangeReadable(const CrashMemoryMap_t *map, uintptr_t address,
                              size_t length, size_t alignment)
{
    return CrashLogRangeReadableInRegions(map, address, length, alignment, 0u);
}

static uint8_t CrashLogFindReadableRegion(const CrashMemoryMap_t *map, uintptr_t address,
                                          uint8_t stack_only, uintptr_t *region_end)
{
    if (!CrashLogMemoryMapValid(map) || region_end == NULL)
        return 0u;

    for (uint32_t index = 0u; index < map->region_count; ++index)
    {
        const CrashMemoryRegion_t *region = &map->regions[index];
        if (address >= region->start && address <= region->end &&
            (!stack_only || region->is_stack_memory != 0u))
        {
            *region_end = region->end;
            return 1u;
        }
    }
    return 0u;
}

// 只接受 Cortex-M7 使用的六种 EXC_RETURN 标记，其他值无法确定保存区域的布局。
static uint8_t CrashLogExceptionReturnValid(uint32_t exception_return)
{
    switch (exception_return)
    {
    // 这三种标记表示只有 8 项基本寄存器数据。
    case 0xFFFFFFF1u:
    case 0xFFFFFFF9u:
    case 0xFFFFFFFDu:
    // 这三种标记表示基本寄存器前面还留有浮点相关区域。
    case 0xFFFFFFE1u:
    case 0xFFFFFFE9u:
    case 0xFFFFFFEDu:
        return 1u;
    default:
        // 不认识的标记不用于推算寄存器地址。
        return 0u;
    }
}

uint32_t CrashLogReadableStackWords(const CrashMemoryMap_t *map, uintptr_t stack_pointer)
{
    uintptr_t region_end;

    // 起点须是 4 的倍数，并位于允许的 RAM 范围内，才能按 uint32_t 计算数量。
    if ((stack_pointer & (sizeof(uint32_t) - 1u)) != 0u ||
        !CrashLogFindReadableRegion(map, stack_pointer, 1u, &region_end))
        return 0u;

    uintptr_t available_words = (region_end - stack_pointer) / sizeof(uint32_t);
    // 崩溃记录只预留 32 项，剩余内存再多也只读取这些。
    if (available_words > CRASH_LOG_SNAPSHOT_WORDS)
        return CRASH_LOG_SNAPSHOT_WORDS;
    return (uint32_t)available_words;
}

uint8_t CrashLogFrameResolve(const CrashMemoryMap_t *map, uintptr_t stack_pointer,
                             uint32_t exception_return,
                             uint32_t cfsr, CrashLogFrameView_t *view)
{
    // 保存、恢复寄存器或保存浮点数据时出错，栈上的相关数据可能不完整。
    const uint32_t frame_error_mask = (1u << 3u) |  /* MMFSR.MUNSTKERR */
                                      (1u << 4u) |  /* MMFSR.MSTKERR */
                                      (1u << 5u) |  /* MMFSR.MLSPERR */
                                      (1u << 11u) | /* BFSR.UNSTKERR */
                                      (1u << 12u) | /* BFSR.STKERR */
                                      (1u << 13u);  /* BFSR.LSPERR */
    // bit4 为 0 时，8 项基本寄存器前面还有 18 项浮点相关区域，检查长度时也要算上。
    const uint32_t fp_extended = (exception_return & (1u << 4u)) == 0u;
    const uint32_t prefix_words = fp_extended ? CRASH_LOG_FP_FRAME_PREFIX_WORDS : 0u;
    const uintptr_t frame_bytes = (uintptr_t)(prefix_words + CRASH_LOG_BASIC_FRAME_WORDS) *
                                  sizeof(uint32_t);

    // 没有接收结果的位置时直接失败。
    if (view == NULL)
        return 0u;

    // 先清除输出，后续检查失败时不会留下上次的寄存器地址和数据个数。
    view->core_frame_address = 0u;
    view->snapshot_words = 0u;
    // 起点须是 8 的倍数，返回标记和错误状态也须符合要求，才能继续检查整个区域。
    if ((stack_pointer & 7u) != 0u ||
        !CrashLogExceptionReturnValid(exception_return) ||
        (cfsr & frame_error_mask) != 0u ||
        !CrashLogRangeReadableInRegions(map, stack_pointer, (size_t)frame_bytes, 8u, 1u))
        return 0u;

    // 跳过可能存在的浮点区域得到 R0 的位置；可复制个数仍从原始栈起点计算。
    view->core_frame_address = stack_pointer + ((uintptr_t)prefix_words * sizeof(uint32_t));
    view->snapshot_words = CrashLogReadableStackWords(map, stack_pointer);
    return 1u;
}
