/** @file daemon.h
 * @brief 为 CANComm 主机测试保留来源状态接口所需的最小类型。
 * @note 仅由 can_comm_freshness_test 使用；生产 Daemon 头文件不受影响。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_CAN_COMM_TEST_DAEMON_H
#define MC02_CAN_COMM_TEST_DAEMON_H

#include <stdint.h>

typedef struct { int unused; } DaemonInstance;
typedef enum { DAEMON_OK = 0, DAEMON_ERR_ARGUMENT, DAEMON_ERR_NOT_REGISTERED } DaemonStatus_t;
typedef enum { DAEMON_SOURCE_WAITING_FIRST_FRAME = 0, DAEMON_SOURCE_ONLINE_DATA_VALID,
               DAEMON_SOURCE_ONLINE_DATA_INVALID, DAEMON_SOURCE_OFFLINE } DaemonSourceState_t;
typedef enum { DAEMON_EVENT_FIFO = 1, DAEMON_EVENT_COUNT, DAEMON_EVENT_COALESCED_STATE }
    DaemonEventPolicy_t;
typedef enum { DAEMON_CONSUMER_BOUND_TASK = 0, DAEMON_CONSUMER_INDEPENDENT_TASK }
    DaemonConsumerMode_t;
typedef struct daemon_event
{
    DaemonSourceState_t state;
} DaemonEvent_t;
typedef struct
{
    uint32_t transport_timeout_ms;
    uint32_t data_timeout_ms;
    uint32_t startup_grace_ms;
    uint32_t event_max_age_ms;
    uint16_t source_id;
    uint8_t event_capacity;
    uint8_t consumer_id;
    DaemonEventPolicy_t event_policy;
    DaemonConsumerMode_t consumer_mode;
    void (*event_handler)(const DaemonEvent_t *, void *);
    void *owner_id;
} Daemon_Init_Config_s;

#define DAEMON_CONSUMER_ROBOT_TASK 2u
#define DAEMON_CONSUMER_MOTOR_TASK 1u
#define DAEMON_REPORT_TRANSPORT_VALID (1u << 0)
#define DAEMON_REPORT_DATA_PRESENT    (1u << 1)
#define DAEMON_REPORT_DATA_VALID      (1u << 2)

DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config);
uint8_t DaemonUnregister(DaemonInstance *instance);
void DaemonReload(DaemonInstance *instance);
uint8_t DaemonIsOnline(DaemonInstance *instance);
DaemonStatus_t DaemonReportAt(DaemonInstance *instance, uint32_t received_ms, uint8_t flags);

#endif
