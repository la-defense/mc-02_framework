/** @file bsp_flash.h
 * @brief 保留的 F1/F4 风格 Flash 接口及地址表，当前 H723 固件已在 CMake 中排除此实现。
 * H723 参数保存使用 bsp_param.c；本表大小及地址不能当作 H723 的实际扇区布局。
 * 以下接口均不检查地址、容量或空指针，也不负责擦写前的系统准备，仅供后续移植参考。
 */
#ifndef _BSP_FLASH_H
#define _BSP_FLASH_H
#include "main.h"

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000)  /* Base address of Sector 0, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08004000)  /* Base address of Sector 1, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08008000)  /* Base address of Sector 2, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x0800C000)  /* Base address of Sector 3, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08010000)  /* Base address of Sector 4, 64 Kbytes   */
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08020000)  /* Base address of Sector 5, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08040000)  /* Base address of Sector 6, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x08060000)  /* Base address of Sector 7, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_8 ((uint32_t)0x08080000)  /* Base address of Sector 8, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_9 ((uint32_t)0x080A0000)  /* Base address of Sector 9, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_10 ((uint32_t)0x080C0000) /* Base address of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11 ((uint32_t)0x080E0000) /* Base address of Sector 11, 128 Kbytes */

#define FLASH_END_ADDR ((uint32_t)0x08100000)       /* 旧表第一段区域的末地址，不表示输入地址一定有效。 */


#define ADDR_FLASH_SECTOR_12 ((uint32_t)0x08100000) /* Base address of Sector 12, 16 Kbytes  */
#define ADDR_FLASH_SECTOR_13 ((uint32_t)0x08104000) /* Base address of Sector 13, 16 Kbytes  */
#define ADDR_FLASH_SECTOR_14 ((uint32_t)0x08108000) /* Base address of Sector 14, 16 Kbytes  */
#define ADDR_FLASH_SECTOR_15 ((uint32_t)0x0810C000) /* Base address of Sector 15, 16 Kbytes  */
#define ADDR_FLASH_SECTOR_16 ((uint32_t)0x08110000) /* Base address of Sector 16, 64 Kbytes  */
#define ADDR_FLASH_SECTOR_17 ((uint32_t)0x08120000) /* Base address of Sector 17, 128 Kbytes */
#define ADDR_FLASH_SECTOR_18 ((uint32_t)0x08140000) /* Base address of Sector 18, 128 Kbytes */
#define ADDR_FLASH_SECTOR_19 ((uint32_t)0x08160000) /* Base address of Sector 19, 128 Kbytes */
#define ADDR_FLASH_SECTOR_20 ((uint32_t)0x08180000) /* Base address of Sector 20, 128 Kbytes */
#define ADDR_FLASH_SECTOR_21 ((uint32_t)0x081A0000) /* Base address of Sector 21, 128 Kbytes */
#define ADDR_FLASH_SECTOR_22 ((uint32_t)0x081C0000) /* Base address of Sector 22, 128 Kbytes */
#define ADDR_FLASH_SECTOR_23 ((uint32_t)0x081E0000) /* Base address of Sector 23, 128 Kbytes */



/** @brief 从旧地址表对应扇区起擦除指定数量，不返回 HAL 结果。
 * @param[in] address 旧芯片表支持的地址；超范围查找会落到扇区 11，不报告错误。
 * @param[in] len 连续扇区个数，不是字节数，调用方须检查实际芯片范围。
 */
void flash_erase_address(uint32_t address, uint16_t len);

/** @brief 按四字节逐项写入，从 start_address 到下一扇区起点之间尝试写入。
 * @param[in] start_address 已擦除区域中四字节对齐的地址，须使用旧芯片对应的表。
 * @param[in] buf 至少可读 len 个 uint32_t，不能为 NULL。
 * @param[in] len uint32_t 元素个数，须大于 0 且不超过本扇区剩余空间。
 * @return -1 为 HAL 写入失败，前面部分可能已写；0 只表示未遇到失败，不保证写足 len 项。
 * @note 循环包含下一扇区起点，且 len 为 0 也会尝试写入；调用方必须先检查长度，不可并发调用。
 */
int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len);


/** @brief 按四字节递增地址写入，达到指定个数或越过 end_address 后结束。
 * @param[in] start_address 已擦除且四字节对齐的起始地址。
 * @param[in] end_address 允许尝试写入的末地址，包含此地址，调用方须检查芯片和扇区范围。
 * @param[in] buf 至少可读 len 个 uint32_t，不能为 NULL。
 * @param[in] len uint32_t 元素个数，须大于 0。
 * @return -1 为 HAL 失败，0 为未遇到失败；范围提前用完也返回 0，已写部分不回退。
 */
int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len);

/** @brief 通过内存复制读取 Flash，不检查地址或容量。
 * @param[in] address 至少可读 len*4 字节的起始地址。
 * @param[out] buf 至少可写 len 个 uint32_t 的区域，不能与源区重叠。
 * @param[in] len uint32_t 元素个数，不是字节数。
 */
void flash_read(uint32_t address, uint32_t *buf, uint32_t len);

/** @brief 按旧地址表查下一扇区的起点。
 * @param[in] address 当前地址。
 * @return 表中下一扇区地址；未命中前面分支时返回 FLASH_END_ADDR，不表示输入一定合法。
 */
uint32_t get_next_flash_address(uint32_t address);
#endif
