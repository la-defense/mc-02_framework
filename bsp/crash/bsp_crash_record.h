/**
 * @file bsp_crash_record.h
 * @brief 定义复位后保留的崩溃记录格式和纯数据检查接口。
 * BSP 异常入口只写固定长度 RAM；正常启动和主机工具按版本、长度和 CRC 检查。
 * @author MC-02 contributors
 * @license BSD-3-Clause; see LICENSE.
 */
#ifndef BSP_CRASH_RECORD_H
#define BSP_CRASH_RECORD_H

#include <stddef.h>
#include <stdint.h>

#define CRASH_LOG_VERSION 4u
#define CRASH_LOG_V3_VERSION 3u
#define CRASH_LOG_MAGIC_BUSY 0x34495342u
#define CRASH_LOG_MAGIC_OK 0x344F5243u
#define CRASH_TEST_PERMIT_MAGIC 0x4D524550u
#define CRASH_TEST_PERMIT_CONSUMED 0x4D4F4E45u
#define CRASH_TASK_NAME_LEN 16u
#define CRASH_SOURCE_NAME_LEN 32u
#define CRASH_STACK_SNAPSHOT_WORDS 32u
#define CRASH_RECORD_CAPACITY 512u

typedef enum
{
    CRASH_TYPE_NONE = 0,
    CRASH_TYPE_HARDFAULT,
    CRASH_TYPE_MEMMANAGE,
    CRASH_TYPE_BUSFAULT,
    CRASH_TYPE_USAGEFAULT,
    CRASH_TYPE_NMI,
    CRASH_TYPE_ASSERT,
    CRASH_TYPE_STACK_OVF,
} CrashType_e;

/** @brief v4 崩溃记录；所有地址字段以字节计，文本长度不含结尾零。 */
typedef struct
{
    uint32_t magic;         ///< 写入时为 BUSY，完整校验后为 OK
    uint32_t len;           ///< 完整记录字节数
    uint32_t version;       ///< 当前格式为 4
    uint32_t progress;      ///< 1～4 表示记录写入阶段
    uint32_t count;          ///< 连续有效记录递增，无有效旧记录时从 1 开始
    uint32_t type;           ///< CrashType_e 故障类型
    uint32_t arg0;           ///< 原始附加值；断言时保留源文件原地址
    uint32_t arg1;           ///< 原始附加值；断言时为源码行号
    uint32_t build_id[5];    ///< GNU Build ID 的20个原始字节，按本机小端字存放
    uint32_t identity_flags; ///< bit0 Build ID可读，bit1编译配置身份可读
    uint32_t profile;        ///< BSP 观测身份中的profile编号
    uint32_t board_role;     ///< BSP 观测身份中的板卡角色编号
    uint32_t module_mask;    ///< 编译时选入模块位图
    uint32_t config_version; ///< 编译配置身份布局编号
    uint32_t task_name_len;  ///< task_name中的有效字节数
    char task_name[CRASH_TASK_NAME_LEN]; ///< 有界复制的任务名，始终以零结束
    uint32_t source_name_len; ///< source_name中的有效字节数
    char source_name[CRASH_SOURCE_NAME_LEN]; ///< 有界复制的断言文件名
    uint32_t cfsr;            ///< Cortex-M 故障状态寄存器
    uint32_t hfsr;
    uint32_t bfar;
    uint32_t mmfar;
    uint32_t dfsr;
    uint32_t r0, r1, r2, r3, r12, lr, pc, xpsr;
    uint32_t frame_status; ///< 0无硬件帧，1有效，2地址或保存状态无效
    uint32_t exception_return; ///< 异常入口LR中的 EXC_RETURN 原值
    uint32_t sp;            ///< 栈读取起点
    uint32_t psp;
    uint32_t msp;
    uint32_t stack_words;   ///< 实际从sp读取的字数，最多32
    uint32_t stack_snapshot[CRASH_STACK_SNAPSHOT_WORDS];
    uint32_t crc32;         ///< CRC32-IEEE，覆盖len到stack_snapshot末尾
} CrashLog_t;

typedef enum
{
    CRASH_RECORD_VALID = 0,
    CRASH_RECORD_VALID_BUSY_FINAL,
    CRASH_RECORD_EMPTY,
    CRASH_RECORD_WRITING,
    CRASH_RECORD_BAD_ARGUMENT,
    CRASH_RECORD_BAD_MAGIC,
    CRASH_RECORD_BAD_LENGTH,
    CRASH_RECORD_BAD_VERSION,
    CRASH_RECORD_BAD_CRC,
    CRASH_RECORD_LEGACY_V3,
} CrashRecordStatus_t;

/** @brief 记录检查结果；actual_length在可读取头部时报告记录声明长度。 */
typedef struct
{
    CrashRecordStatus_t status;
    uint32_t actual_length;
    uint32_t expected_length;
    uint32_t version;
    uint32_t progress;
} CrashRecordInfo_t;

/**
 * @brief 检查一段最多512字节的崩溃记录，不读取声明长度以外的数据。
 * @param[in] bytes 从固定保留区复制出的原始字节；capacity为0时才允许NULL。
 * @param[in] capacity 实际可读字节数，范围0～512。
 * @param[out] info 接收状态及记录声明长度；为NULL时返回参数错误。
 * @return 记录状态；v3只报告旧版，需由主机离线工具按旧布局解析。
 * @note 失败时不改写调用方的记录结构；info用于说明失败原因，允许被更新。
 */
CrashRecordStatus_t CrashLogRecordInspect(const uint8_t *bytes, size_t capacity,
                                          CrashRecordInfo_t *info);

/**
 * @brief 将NOLOAD许可区中的无效magic归一为空闲状态。
 * @param[in] stored_magic 当前32位标记。
 * @return 有效待执行或已消费标记原样保留；其他值返回0。
 * @note 纯计算，不访问硬件或修改输入；只能在启动时用于许可标记检查。
 */
uint32_t CrashTestPermitMagicNormalize(uint32_t stored_magic);

#endif
