/** @file platform_task_stats.h
 * @brief 提供不暴露 RTOS 类型的有限任务运行统计读取接口。
 */
#ifndef MC02_PLATFORM_TASK_STATS_H
#define MC02_PLATFORM_TASK_STATS_H

#include <stdint.h>

#define PLATFORM_TASK_STATS_MAX_COUNT 12u /**< 单次最多保留的任务数量。 */
#define PLATFORM_TASK_STATS_NAME_SIZE 12u /**< 名称空间为11字节和1个结束符。 */

/** @brief 一条任务统计记录，单位沿用平台运行计数器及栈字数。 */
typedef struct
{
    char name[PLATFORM_TASK_STATS_NAME_SIZE]; /**< 任务名，最多11个字符并以\0结束。 */
    uint32_t runtime_counter;                 /**< FreeRTOS累计运行计数，32位回绕。 */
    uint32_t priority;                        /**< 当前任务优先级。 */
    uint32_t stack_high_water_words;          /**< 历史最小剩余栈字数，每字大小由后端决定。 */
} PlatformTaskStatsEntry_t;

/** @brief 本次任务统计读取的计数信息。 */
typedef struct
{
    uint32_t captured_count;     /**< 最近一次捕获写入的记录数量。 */
    uint32_t total_task_count;   /**< 读取前后观测到的最大任务总数。 */
    uint32_t total_runtime_count;/**< RTOS 返回的系统运行计数。 */
} PlatformTaskStatsInfo_t;

/** @brief 捕获有限数量的任务运行及栈统计。
 * @param[out] info 统计数量信息；失败时保持不变。
 * @return 1 表示读取成功，0 表示参数无效或后端统计接口不可用。
 * @note 固定最多捕获 PLATFORM_TASK_STATS_MAX_COUNT 条；捕获和后续读取必须串行完成。
 */
uint8_t PlatformTaskStatsCapture(PlatformTaskStatsInfo_t *info);

/** @brief 读取最近一次捕获中的一条任务记录。
 * @param[in] index 从0开始的记录编号，须小于 captured_count。
 * @param[out] entry 接收记录的对象；失败时保持原值。
 * @return 1 表示写入成功，0 表示参数无效或编号超出最近一次捕获范围。
 * @note 必须在下一次 PlatformTaskStatsCapture 前完成读取；任务名最多11个字符。
 */
uint8_t PlatformTaskStatsGetEntry(uint32_t index, PlatformTaskStatsEntry_t *entry);

#endif
