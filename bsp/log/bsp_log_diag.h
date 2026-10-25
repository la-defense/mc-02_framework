/** @file bsp_log_diag.h
 * @brief 声明独立日志串口使用的短帧格式和命令编号。
 * @note 帧不包含控制或安全命令；等级修改仅在本次上电期间有效。
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

/** @brief 主机发给MCU的日志等级命令。 */
typedef enum
{
    BSP_LOG_DIAG_SET_LEVEL = 1u,
    BSP_LOG_DIAG_GET_LEVEL = 2u
} BSPLogDiagnosticCommand_t;

/**
 * 请求帧按字节排列为：同步字节D3 6C、版本、命令、序号、负载长度、负载、CRC低字节、CRC高字节。
 * CRC采用CRC16_MODBUS_A5配置，覆盖同步字节至负载末尾。SET负载为来源编号和等级各1字节；
 * GET_LEVEL负载为1字节来源编号。命令回复负载为状态、来源、当前等级和固件编译上限。
 * 此串口仅用于等级命令和回复，日志记录只经RTT输出。
 */

#endif /* MC02_BSP_LOG_DIAG_H */
