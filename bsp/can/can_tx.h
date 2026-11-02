/** @file can_tx.h
 * @brief 与芯片无关的经典 CAN 队列提交和结果类型。
 * @note 帧数据在提交时复制；硬件完成状态由具体平台的队列实现报告。
 */
#ifndef MC02_CAN_TX_H
#define MC02_CAN_TX_H

#include <stddef.h>
#include <stdint.h>

#define CAN_TX_MAX_QUEUE_ITEMS 16u
// 允许新版桥接一次提交64字节内容和时间、校验字段所需的13个分片。
#define CAN_TX_MAX_BATCH_FRAMES 13u

/** @brief CAN 帧标识符类型；经典帧数据区最多 8 字节。 */
typedef enum
{
    CAN_TX_ID_STANDARD = 0,
    CAN_TX_ID_EXTENDED = 1
} CAN_TxIdType_t;

/** @brief 调度器帧副本的输入格式。 */
typedef struct
{
    uint32_t identifier;          /**< 标准 ID 为 0～0x7FF，扩展 ID 为 0～0x1FFFFFFF。 */
    CAN_TxIdType_t identifier_type;
    const uint8_t *data;          /**< 长度大于零时须指向可读存储；提交返回后可立即复用。 */
    uint8_t length;               /**< 数据字节数，允许 0～8。 */
} CAN_TxFrame_t;

/** @brief 每条总线独立的队列与周期流量预算。 */
typedef struct
{
    uint8_t queue_capacity;       /**< 有界待发组容量，范围 1～16。 */
    uint16_t utilization_limit_permille; /**< 周期流量上限，范围 1～1000‰。 */
    uint32_t nominal_bitrate_bps; /**< 经典 CAN 标称位速率，单位 bit/s。 */
} CAN_TxBusConfig_t;

/** @brief 提交请求的排序优先级，数值越小越先调度。 */
typedef enum
{
    CAN_TX_PRIORITY_URGENT = 0,
    CAN_TX_PRIORITY_CONTROL = 1,
    CAN_TX_PRIORITY_NORMAL = 2,
    CAN_TX_PRIORITY_BACKGROUND = 3
} CAN_TxPriority_t;

/** @brief 调度请求的提交结果；成功只代表调度器已复制并接收请求。 */
typedef enum
{
    CAN_TX_SUBMIT_ACCEPTED = 0,
    CAN_TX_SUBMIT_COALESCED,
    CAN_TX_SUBMIT_INVALID,
    CAN_TX_SUBMIT_NOT_READY,
    CAN_TX_SUBMIT_FULL,
    CAN_TX_SUBMIT_BANDWIDTH,
    CAN_TX_SUBMIT_CONTEXT
} CAN_TxSubmitStatus_t;

/** @brief 在周期帧交给硬件前，使用当前时间与可变副本做有界的最终准备／许可检查。
 * @note 在普通 CAN 服务任务的短临界区内调用；不得阻塞、分配内存或再次提交 CAN 请求。
 */
typedef void (*CAN_TxPeriodicPrepareFn_t)(const void *source, uint32_t now_ms,
                                          uint8_t *payload, uint8_t length);

/** @brief 单次请求的异步处理状态，硬件接受与总线完成分开报告。 */
typedef enum
{
    CAN_TX_RESULT_QUEUED = 0,
    CAN_TX_RESULT_ACTIVE,
    CAN_TX_RESULT_IN_FLIGHT,
    CAN_TX_RESULT_COMPLETE,
    CAN_TX_RESULT_FAILED,
    CAN_TX_RESULT_EXPIRED,
    CAN_TX_RESULT_CANCELLED,
    CAN_TX_RESULT_SUPERSEDED
} CAN_TxResultState_t;

typedef struct
{
    uint32_t request_id;          /**< 非零请求编号；仅在本机调度器生命周期内唯一。 */
    CAN_TxResultState_t state;
    uint8_t frame_count;
    uint8_t accepted_frames;      /**< 平台已接受的帧数，不代表已在线上发送。 */
    uint8_t completed_frames;     /**< 收到发送完成通知的帧数。 */
    uint8_t pending_frames;       /**< 平台已接受且尚未收到完成或中止通知的帧数。 */
    uint32_t last_hal_error;      /**< 后端原始错误值；名称为兼容现有接口而保留。 */
} CAN_TxResult_t;

#endif
