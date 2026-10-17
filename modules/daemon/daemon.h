/** @file daemon.h
* @brief 按周期调用次数检查模块是否仍在更新，计数耗尽后报告一次离线。
 */
#ifndef MONITOR_H
#define MONITOR_H

#include "stdint.h"
#include "string.h"

#define DAEMON_MX_CNT 64

/* 模块离线处理函数指针 */
typedef void (*offline_callback)(void *);

/* daemon结构体定义 */
typedef struct daemon_ins
{
    uint16_t reload_count;     // 重载值
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用

    uint16_t temp_count; // 当前值,减为零说明模块离线或异常
    uint8_t offline_flag; // 0 表示本次尚未报告离线，1 表示已报告；重新收到数据后清零。
    void *owner_id;      // 模块地址，离线回调据此找到对应电机或通信对象。
} DaemonInstance;

/* daemon初始化配置 */
typedef struct
{
    uint16_t reload_count;     // 实际上这是app唯一需要设置的值?
    uint16_t init_count;       // 上线等待时间,有些模块需要收到主控的指令才会反馈报文,或pc等需要开机时间
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用

    void *owner_id;            // id取拥有daemon的实例的地址,如DJIMotorInstance*,cast成void*类型
} Daemon_Init_Config_s;

/** @brief 保存模块离线计数配置，启动初始等待。
* @param[in] config 有效配置；reload_count 和 init_count 为任务调用次数，为 0 时各用默认 100。
* @return 长期有效对象；NULL、超过 64 项或分配失败返回 NULL。
* @note 当前 DaemonTask 约每 10 ms 调用一次，初始计数未用完也视为在线，不代表已收过有效数据。
 */
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config);
/** @brief 移出实例表并释放对象。
* @param[in] instance 确保没有回调仍在使用的实例，成功后不可再访问。
* @return 1 为已释放，0 为 NULL 或未注册。
 */
uint8_t DaemonUnregister(DaemonInstance *instance);

/** @brief 将当前计数恢复为 reload_count，并允许下次离线再次报告。
* @param[in,out] instance 模块计数对象，NULL 时不处理。
 */
void DaemonReload(DaemonInstance *instance);

/** @brief 读取模块当前计数是否大于 0，不更新计时。
* @param[in] instance 模块计数对象，NULL 返回 0。
* @return 1 为计数仍大于 0，0 为已耗尽或指针为空；数据是否有效还须由业务层检查。
 */
uint8_t DaemonIsOnline(DaemonInstance *instance);

/** @brief 遍历已注册模块，每次将非零计数减一；本次进入时已为 0 且尚未报告时调用一次回调。
* @note 由 Daemon 任务约每 10 ms 调用，时间由调用频率决定，不能将次数直接当作毫秒。
 */
void DaemonTask();

#endif // !MONITOR_H
