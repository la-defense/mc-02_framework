/**
 * @file bsp_crash_guard.h
 * @brief 检查是否已进入崩溃记录，重复进入时请求系统复位。
 * bsp_crash.c 的各崩溃入口共用一个标志，通过检查后再继续保存现场。
 */
#ifndef BSP_CRASH_GUARD_H
#define BSP_CRASH_GUARD_H

#include <stdint.h>

/**
 * @brief 标志为 0 时改为 1 并返回，允许调用方继续记录。
 * @param[in,out] active 各入口共用的可写标志，启动时初始化为 0，并在本次启动中保持有效。
 * @note active 为 NULL 或标志非 0 时，立即请求系统复位，不再返回，也不改写标志。
 *       首次进入后保留标志为 1，本函数不会清除它；当前调用方使用启动时清零的普通全局变量。
 *       本函数只处理标志和复位，输出关断及现场保存由调用方所在的崩溃处理流程负责。
 */
void CrashLogGuardEnter(volatile uint32_t *active);

#endif
