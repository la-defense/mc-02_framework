/**
 * @file bsp_crash_frame.h
 * @brief 检查异常时保存寄存器的栈地址，并计算最多可复制的数据个数。
 * bsp_crash.c 先调用这里的接口，再读取对应内存；本模块只计算地址和长度。
 * 实现中的 Flash 常量区及四段 RAM 范围须与 STM32H723VGTx_FLASH.ld 保持一致。
 */
#ifndef BSP_CRASH_FRAME_H
#define BSP_CRASH_FRAME_H

#include <stddef.h>
#include <stdint.h>

#define CRASH_MEMORY_REGION_CAPACITY 5u

/** @brief 由链接脚本给出的连续可读地址范围，结束地址不包含在内。 */
typedef struct
{
    uintptr_t start;
    uintptr_t end;
    uint8_t is_stack_memory; ///< RAM区域填1；只读Flash等其他区域填0
} CrashMemoryRegion_t;

/** @brief 当前固件允许检查的 Flash 常量区和 RAM 区。 */
typedef struct
{
    const CrashMemoryRegion_t *regions;
    uint32_t region_count;
} CrashMemoryMap_t;

/** @brief 异常栈地址检查通过后，提供给崩溃记录代码的读取位置和数量。 */
typedef struct
{
    uintptr_t core_frame_address; ///< 8 项基本寄存器数据的起点，第一项为 R0，地址按字节计
    uint32_t snapshot_words;      ///< 从原始 stack_pointer 起可复制的 uint32_t 个数，每项 4 字节，最多 32 项
} CrashLogFrameView_t;

/**
 * @brief 检查寄存器保存区域是否完整位于同一段允许的 RAM 内。
 * @param[in] map 本固件由链接符号建立的地址范围表；表和范围数组须在整个调用期间可读。
 * @param[in] stack_pointer 异常入口传入的栈起始地址，按字节计，必须是 8 的倍数。
 * @param[in] exception_return 进入异常时 LR 中的 EXC_RETURN 标记，用于判断保存区域的布局。
 * @param[in] cfsr 当时的 CFSR 故障状态，用于检查保存或恢复寄存器时是否出错。
 * @param[out] view 接收读取位置和数量；为 NULL 时返回 0，非空时先将两个字段清零。
 * @return 1 表示地址、返回标记、相关错误位和整个区域长度均通过检查；0 表示失败。
 * @note 基本寄存器数据共 8 项（32 字节）；包含浮点区域时，前面另有 18 项（72 字节）。
 *       失败时非空 view 的两个字段保持为 0，调用方跳过读取。
 */
uint8_t CrashLogFrameResolve(const CrashMemoryMap_t *map, uintptr_t stack_pointer,
                             uint32_t exception_return,
                             uint32_t cfsr, CrashLogFrameView_t *view);

/**
 * @brief 计算从指定栈地址起，到同段 RAM 末尾还能完整复制多少个 uint32_t。
 * @param[in] map 当前固件的地址范围表。
 * @param[in] stack_pointer 读取起点，按字节计，必须是 4 的倍数。
 * @return 可复制的数据个数，每项 4 字节，最多 32 项；地址不符合要求或没有剩余空间时返回 0。
 * @note 只检查实现中列出的四段 RAM 和剩余长度；异常寄存器区域还须由 CrashLogFrameResolve() 检查。
 */
uint32_t CrashLogReadableStackWords(const CrashMemoryMap_t *map, uintptr_t stack_pointer);

/**
 * @brief 检查一段数据是否完整落在一个允许读取的区域内。
 * @param[in] map 地址范围表。
 * @param[in] address 数据起点，按字节计。
 * @param[in] length 数据字节数，必须大于0。
 * @param[in] alignment 起点对齐字节数，只允许1、2、4或8。
 * @return 1表示整段数据通过检查；0表示参数、对齐、溢出或范围不符合要求。
 */
uint8_t CrashLogRangeReadable(const CrashMemoryMap_t *map, uintptr_t address,
                              size_t length, size_t alignment);

#endif
