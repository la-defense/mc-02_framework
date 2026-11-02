/** @file platform_os_freertos.c
 * @brief 将框架任务接口转换为本工程的 CMSIS-RTOS／FreeRTOS 调用。
 */
#include "platform_os.h"
#include "cmsis_os.h"
#include <stddef.h>

uint8_t PlatformTaskCreateTracked(const PlatformTaskConfig_t *config, PlatformTaskHandle_t *handle)
{
    // 先检查输入，避免把不完整的任务定义交给后端。
    if (config == NULL || handle == NULL || config->name == NULL || config->entry == NULL ||
        config->stack_words == 0u || config->stack_words > UINT16_MAX)
        return 0u;
    osPriority priority; // 只接受已声明的框架优先级，避免意外映射。
    switch (config->priority)
    {
    case PLATFORM_TASK_BELOW_NORMAL: priority = osPriorityBelowNormal; break; // 后台任务。
    case PLATFORM_TASK_NORMAL: priority = osPriorityNormal; break; // 普通任务。
    case PLATFORM_TASK_ABOVE_NORMAL: priority = osPriorityAboveNormal; break; // 较高优先级。
    case PLATFORM_TASK_LOW: priority = osPriorityLow; break; // 沿用 LCD 的低优先级。
    default: return 0u; // 不认识的配置不创建任务。
    }
    // 当前 CMSIS 适配器在创建调用内读取定义，FreeRTOS 复制名称并保存入口和参数。
    osThreadDef_t definition = {0};
    definition.name = (char *)config->name; // CMSIS 声明未使用 const，当前后端仅读取名称。
    definition.pthread = config->entry; // 直接保存同签名入口，避免转换函数指针。
    definition.tpriority = priority; // 将框架优先级映射到现有 CMSIS 优先级。
    definition.stacksize = config->stack_words; // 本工程 CMSIS 后端把此值作为 FreeRTOS 栈字数。
    osThreadId created = osThreadCreate(&definition, config->argument); // 在后端创建并取得不透明句柄。
    if (created == NULL) return 0u; // 创建失败时不改调用者旧句柄。
    *handle = created; // 成功后才公布句柄供诊断读取。
    return 1u;
}

uint8_t PlatformTaskCreate(const PlatformTaskConfig_t *config)
{
    PlatformTaskHandle_t ignored = NULL; // 不需要保存句柄的调用者仍共用相同检查。
    return PlatformTaskCreateTracked(config, &ignored);
}

void PlatformTaskDelayMs(uint32_t milliseconds)
{
    (void)osDelay(milliseconds); // 沿用原先的 CMSIS 毫秒延时和调度语义。
}
