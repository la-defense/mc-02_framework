#ifndef WS2812_H
#define WS2812_H

#include "main.h"

/** @file ws2812.h
* @brief MC02 板载彩灯，使用 SPI6 从 PA7 发送 GRB 编码。
* 来源：达妙 MC02 例程（资料/例程/CtrBoard-H7_WS2812），保留本工程移植实现。
* 每个颜色位用一个 SPI 字节发送；代码把 0xC0、0xF0 右移一位后分别作为 0、1 的编码。
* PA5 保留给 ADC1_INP19 按键采样；SPI6 时钟留在外设内部，不占用该引脚。
* 代码说明当前编码，不代替实际灯珠波形和时序测量。
 */

/** @brief 配置 SPI6、BDMA 通道0和数据引脚；重复调用保持已初始化状态。
 * @return HAL_OK 表示成功，资源正在使用时返回 HAL_BUSY，其他初始化失败返回 HAL_ERROR。
 * @note 调度器启动前串行调用；要求 DCache 关闭，使用 SRAM4 和 SPI6／BDMA0 中断，不占用 PA5。
 */
HAL_StatusTypeDef WS2812_Init(void);
/** @brief 复制颜色并尝试启动异步发送；未初始化或通道隔离时返回 HAL_ERROR。
* @param[in] r,g,b 红、绿、蓝分量，各为 0～255，实际按绿、红、蓝顺序发送，高位在前。
* @return HAL_OK 表示该颜色的本机发送已完成；HAL_BUSY 表示等待提交或完成；HAL_ERROR 表示通道不可用。
* @note 普通任务单调用者串行使用，调用方须定期调用 WS2812_Process()；灯珠实际波形仍需测量。
 */
HAL_StatusTypeDef WS2812_SetRGB(uint8_t r, uint8_t g, uint8_t b);

/** @brief 在普通任务处理完成、20ms 超时和分轮停止；查询或中断不会推进恢复。
 * @return HAL_OK 表示空闲或完成，HAL_BUSY 表示在途，HAL_ERROR 表示失败或隔离。
 * @note 预计由 Daemon 每10ms调用；停止检查再给20ms，协作检查不保证精确截止。
 */
HAL_StatusTypeDef WS2812_Process(void);

typedef enum
{
    WS2812_TRANSFER_IDLE = 0,
    WS2812_TRANSFER_ACTIVE,
    WS2812_TRANSFER_RECOVERING,
    WS2812_TRANSFER_FAILED,
    WS2812_TRANSFER_ISOLATED,
} WS2812TransferPhase_t;

/** @brief 只读事务信息；buffer_reusable 为1表示硬件已停止，失败通道仍不接受新提交。 */
typedef struct
{
    WS2812TransferPhase_t phase;
    HAL_StatusTypeDef last_result; /**< 最近事务结果；期限到达为 HAL_TIMEOUT。 */
    uint8_t buffer_reusable;
} WS2812TransferState_t;

/** @brief 复制发送状态，不推进事务；未初始化或输出为空时返回 HAL_ERROR 并保留输出。
 * @param[out] state 调用方保存的状态副本。
 * @return 成功 HAL_OK，否则 HAL_ERROR。
 */
HAL_StatusTypeDef WS2812_GetState(WS2812TransferState_t *state);

/** @brief 由共享 HAL 回调分发 SPI 完成；只记录属于 SPI6 的事件。
 * @param[in] handle HAL 回调的 SPI 句柄。
 * @return 1 为 SPI6 句柄，0 为其他句柄；可在中断上下文调用。
 */
uint8_t WS2812_OnTxComplete(SPI_HandleTypeDef *handle);
/** @brief 由共享 HAL 回调分发 SPI 错误；中断中只登记，不执行停止。
 * @param[in] handle HAL 回调的 SPI 句柄。
 * @return 1 为 SPI6 句柄，0 为其他句柄；可在中断上下文调用。
 */
uint8_t WS2812_OnError(SPI_HandleTypeDef *handle);

#endif // !WS2812_H
