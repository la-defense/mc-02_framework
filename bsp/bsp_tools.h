/** @file bsp_tools.h
 * @brief 将回调交给独立 RTOS 任务执行，注册信息在 bsp_tools.c 中长期保存。
 */
#include "cmsis_os.h"
#include "bsp_log.h"

/** @brief 注册一个回调任务，任务启动后先执行一次回调，此后每次等到信号再执行。
 * @param[in] name 以 0 结尾的任务名，创建时须可读。
 * @param[in] cbk 有效回调地址，实际按 void (*)(void const *) 调用，不能为 NULL。
 * @param[in] ins 传给回调的地址，须在任务使用期间有效，可由回调自行约定是否允许 NULL。
 * @param[in] priority RTOS 任务优先级。
 * @return 对应任务的信号位；接口未检查任务创建失败，也不返回任务句柄。
 * @note 最多注册 32 项，超过后持续打印错误并停在循环中；需要另有对应任务句柄才能发送信号。
 */
uint32_t CreateCallbackTask(char *name, void *cbk, void *ins, osPriority priority);

