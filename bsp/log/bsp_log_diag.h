/** @file bsp_log_diag.h
 * @brief 声明USART10诊断串口使用的短帧格式和命令编号。
 * @note 帧不包含控制或安全命令；等级修改仅在本次上电期间有效，日志文本仍走RTT。
 */
#ifndef MC02_BSP_LOG_DIAG_H
#define MC02_BSP_LOG_DIAG_H

#include <stdint.h>

#define BSP_LOG_DIAG_SYNC_0 0xD3u
#define BSP_LOG_DIAG_SYNC_1 0x6Cu
#define BSP_LOG_DIAG_VERSION 1u
#define BSP_LOG_DIAG_HEADER_SIZE 6u
#define BSP_LOG_DIAG_CRC_SIZE 2u
#define BSP_LOG_DIAG_REQUEST_MAX 10u
#define BSP_LOG_DIAG_REPLY_BIT 0x80u
#define BSP_LOG_DIAG_TOPIC_SCHEMA_VERSION 1u
#define BSP_LOG_DIAG_TOPIC_HEADER_SIZE 4u
#define BSP_LOG_DIAG_TOPIC_RECORD_SIZE 54u
#define BSP_LOG_DIAG_TOPIC_REPLY_SIZE 58u

/** @brief 主机发给MCU的诊断命令；不包含日志文本流。 */
typedef enum
{
    BSP_LOG_DIAG_SET_LEVEL = 1u,
    BSP_LOG_DIAG_GET_LEVEL = 2u,
    BSP_LOG_DIAG_GET_HEALTH = 3u,
    BSP_LOG_DIAG_GET_TOPIC_DIAGNOSTICS = 4u
} BSPLogDiagnosticCommand_t;

/**
 * 请求帧按字节排列为：同步字节D3 6C、版本、命令、序号、负载长度、负载、CRC低字节、CRC高字节。
 * CRC采用CRC16_MODBUS_A5配置，覆盖同步字节至负载末尾。SET负载为来源编号和等级各1字节；
 * GET_LEVEL负载为1字节来源编号。GET_HEALTH无负载；GET_TOPIC_DIAGNOSTICS负载为零起始话题序号。
 * 两种查询的回复均由已注册处理器生成，最多120字节。此串口仅承载等级命令和按需只读状态；日志记录只经RTT输出。
 * 话题统计应答前4字节为状态、schema、序号和启用话题数，随后是54字节固定字段；整次请求只读取一个话题。
 */

#endif /* MC02_BSP_LOG_DIAG_H */
