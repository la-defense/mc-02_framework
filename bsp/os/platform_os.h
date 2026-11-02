/** @file platform_os.h
 * @brief 框架任务创建与毫秒等待接口；不暴露芯片或 RTOS 类型。
 */
#ifndef MC02_PLATFORM_OS_H
#define MC02_PLATFORM_OS_H
#include <stdint.h>

typedef void (*PlatformTaskEntry_t)(void const *argument); /**< 任务入口及其只读参数。 */
typedef enum {
    PLATFORM_TASK_BELOW_NORMAL = 0, /**< 低于普通任务，适合后台事件处理。 */
    PLATFORM_TASK_NORMAL,           /**< 普通任务优先级。 */
    PLATFORM_TASK_ABOVE_NORMAL,    /**< 高于普通任务的优先级。 */
    PLATFORM_TASK_LOW              /**< 低优先级，沿用现有 LCD 任务设置。 */
} PlatformTaskPriority_t;

typedef void *PlatformTaskHandle_t; /**< 仅用于保存和查询任务身份，不暴露 RTOS 类型。 */

typedef struct {
    const char *name;                 /**< 任务名称；存储至少持续到创建调用返回。 */
    PlatformTaskEntry_t entry;        /**< 任务入口，不能为空。 */
    void *argument;                   /**< 原样传给入口；对象须在任务使用期间有效。 */
    uint32_t stack_words;             /**< 栈容量，单位为后端栈字；当前后端接受 1～65535。 */
    PlatformTaskPriority_t priority;  /**< 框架优先级，必须为已声明值。 */
} PlatformTaskConfig_t;

/** @brief 创建一个任务；可在调度器启动前或任务上下文调用。
 * @param[in] config 本次配置，调用期间必须有效且不可为空。
 * @return 非零表示创建成功，零表示配置无效或后端创建失败。
 * @note 创建后任务可能在本函数返回前运行；本接口不提供删除或任务句柄。
 */
uint8_t PlatformTaskCreate(const PlatformTaskConfig_t *config);

/** @brief 创建任务并返回供诊断使用的句柄。
 * @param[in] config 本次任务配置，调用期间有效且不可为空。
 * @param[out] handle 成功时写入不透明句柄；失败时保持原值，不可为空。
 * @return 非零表示创建成功，零表示参数或后端创建失败。
 * @note 不提供删除或运行期改变优先级；任务可能在函数返回前运行。
 */
uint8_t PlatformTaskCreateTracked(const PlatformTaskConfig_t *config, PlatformTaskHandle_t *handle);

/** @brief 在任务上下文等待指定毫秒数，沿用后端的时钟分辨率。
 * @param[in] milliseconds 等待时间，单位毫秒；零的行为由当前后端决定。
 * @note 不得从中断调用；不用于亚毫秒任务唤醒。
 */
void PlatformTaskDelayMs(uint32_t milliseconds);

/** @brief 查询任务调度器是否正在运行。
 * @return 非零表示运行中，零表示尚未运行。
 * @note H723 后端沿用 CMSIS osKernelRunning 的判断。
 */
uint8_t PlatformSchedulerIsRunning(void);

/** @brief 在调度器启动前沿用平台原有的忙等实现。
 * @param[in] milliseconds 等待时长，单位毫秒。
 * @note 仅用于保留现有启动前行为；调用者须确保时基已初始化，期间不让出 CPU。
 */
void PlatformDelayBeforeSchedulerMs(uint32_t milliseconds);
#endif
