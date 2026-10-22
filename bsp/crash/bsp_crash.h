/**
 * @file bsp_crash.h
 * @brief 保存异常现场、检查复位保留记录并供日志和 LCD 查询。
 * 异常入口先关闭已知输出，再写固定 RAM 记录并请求系统复位；详细字段见 bsp_crash_record.h。
 * @author MC-02 contributors
 * @license BSD-3-Clause; see LICENSE.
 */
#ifndef BSP_CRASH_H
#define BSP_CRASH_H

#include <stdint.h>

#include "bsp_crash_record.h"

#ifndef MC02_BENCH_CRASH_DIAGNOSTICS
#define MC02_BENCH_CRASH_DIAGNOSTICS 0
#endif

typedef enum
{
    CRASH_TEST_CASE_NONE = 0,
    CRASH_TEST_CASE_UNDEFINED_INSTRUCTION = 1,
    CRASH_TEST_CASE_ASSERT = 2,
    CRASH_TEST_CASE_STACK_HOOK = 3,
} CrashTestCase_e;

/**
 * @brief 固定地址的一次性自测许可，共10个小端32位字。
 * @note CRC覆盖version到nonce；magic最后写入，目标先将magic改为CONSUMED再触发异常。
 */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t test_case;
    uint32_t build_id[5];
    uint32_t nonce;
    uint32_t crc32;
} CrashTestPermit_t;

/**
 * @brief 检查上次崩溃记录并输出通过检查的字段，不清除记录。
 * @note 在日志初始化后由普通任务调用；v3记录交给主机工具按旧格式读取。
 */
void CrashLogInit(void);

/**
 * @brief 检查保留记录的格式、声明长度和CRC。
 * @param[out] info 接收状态及实际声明长度；为NULL时返回参数错误。
 * @return 记录检查状态；失败原因和长度写入info。
 */
CrashRecordStatus_t CrashLogGetStatus(CrashRecordInfo_t *info);

/**
 * @brief 检查记录并复制给LCD或查询方，不清除原记录。
 * @param[out] out 接收完整v4记录；为NULL或记录无效时保持原内容。
 * @return 1表示复制成功，0表示失败。
 * @note 不要求调用方关闭中断；若出现新异常，处理程序会立即保存并复位。
 */
uint8_t CrashLogGetLast(CrashLog_t *out);

/** @brief 清除当前记录的magic，使后续查询忽略它。 */
void CrashLogClear(void);

/**
 * @brief 将记录类型转换为静态英文名称。
 * @param[in] type CrashType_e数值；未知值返回"NONE"。
 * @return 指向只读静态字符串的指针；无需释放。
 */
const char *CrashLogTypeName(uint32_t type);

/**
 * @brief 将记录类型转换为短静态标记。
 * @param[in] type CrashType_e数值；未知值返回"---"。
 * @return 指向只读静态字符串的指针；无需释放。
 */
const char *CrashLogTypeShort(uint32_t type);

/**
 * @brief 关闭已知输出、保存指定现场并请求复位，不返回。
 * @param[in] type 记录类型，使用CrashType_e的数值。
 * @param[in] frame 8项基本寄存器起点；NULL时使用当前栈和caller_pc。
 * @param[in] arg0 随记录保存的附加数据。
 * @param[in] arg1 随记录保存的附加数据。
 * @param[in] caller_pc 无硬件异常帧时保存的调用地址，按字节计。
 */
void CrashLogRecord(uint32_t type, uint32_t *frame, uint32_t arg0, uint32_t arg1,
                    uint32_t caller_pc);

/**
 * @brief 保存configASSERT的文件名和行号，然后请求复位。
 * @param[in] file 程序内可读的源文件字符串；异常入口最多复制31字节。
 * @param[in] line 源码行号。
 */
void CrashLogAssertFail(const char *file, uint32_t line);

/**
 * @brief 有界复制栈溢出任务名，保存现场后请求复位。
 * @param[in] task_name 可读任务名；最多复制15字节，NULL表示空名。
 */
void CrashLogStackOverflow(const char *task_name);

/**
 * @brief 供异常汇编入口保存现场并请求复位。
 * @param[in] stack_frame 原始栈起点；NULL表示没有硬件寄存器帧。
 * @param[in] type 异常类型编号。
 * @param[in] exception_return 进入异常时LR中的EXC_RETURN原值。
 */
void CrashLogHandlerC(uint32_t *stack_frame, uint32_t type, uint32_t exception_return);

#if MC02_BENCH_CRASH_DIAGNOSTICS
/** @brief 仅专用bench_safe诊断固件导出的固定许可记录。 */
extern volatile CrashTestPermit_t bsp_crash_test_permit;
/** @brief 验证并消费一次性许可，再执行许可指定的单次记录路径。 */
void CrashLogSelfTest(void);
#endif

#endif
