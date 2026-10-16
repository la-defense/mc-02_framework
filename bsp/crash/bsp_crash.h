/**
 * @file bsp_crash.h
 * @brief 保存异常、断言失败和任务栈溢出的现场，复位后供日志和 LCD 查询。
 * 异常入口先关闭已知输出，再记录并请求复位；BSPInit() 在日志初始化后调用 CrashLogInit()。
 * 记录放在 .noinit 中，可跨系统复位保留；掉电后不能保证保留。
 */
#ifndef BSP_CRASH_H
#define BSP_CRASH_H

#include <stdint.h>

/** @brief 记录格式版本；改变 CrashLog_t 的字段或布局时须增加版本号。 */
#define CRASH_LOG_VERSION 3u

// magic 表示写入状态：BUSY 为正在写，OK 为写完；BUSY 且进度为 4 时还会继续核对整份记录。
#define CRASH_LOG_MAGIC_BUSY 0x59535542u
#define CRASH_LOG_MAGIC_OK 0x4B4F5243u

/** @brief 任务名数组的字节数，含结尾的 '\0'；编译时检查能否容纳 FreeRTOS 的任务名。 */
#define CRASH_TASK_NAME_LEN 16u

/** @brief 触发现场记录的原因。 */
typedef enum
{
    CRASH_TYPE_NONE = 0,
    CRASH_TYPE_HARDFAULT,   ///< HardFault 异常
    CRASH_TYPE_MEMMANAGE,   ///< 内存访问保护异常
    CRASH_TYPE_BUSFAULT,    ///< 总线访问异常
    CRASH_TYPE_USAGEFAULT,  ///< 指令或处理器使用错误
    CRASH_TYPE_NMI,         ///< 普通关中断操作不会屏蔽的中断
    CRASH_TYPE_ASSERT,     ///< configASSERT 失败
    CRASH_TYPE_STACK_OVF,  ///< 任务栈溢出
} CrashType_e;

/** @brief 最多保存的 uint32_t 个数，每项 4 字节，用于事后查找调用位置。 */
#define CRASH_STACK_SNAPSHOT_WORDS 32u

/** @brief 崩溃记录；查询成功表示记录通过检查，各寄存器是否读到还要看 frame_status。 */
typedef struct
{
    // 头部用于判断记录格式、编译编号和写入进度。
    uint32_t magic;        ///< 开始写时设为 BUSY，写完后设为 OK
    uint32_t len;          ///< 记录字节数，须等于 sizeof(CrashLog_t)
    uint32_t version;      ///< 保存时的 CRASH_LOG_VERSION
    uint32_t fw_id;        ///< 由 bsp_crash.c 的编译日期、时间和配置名计算的编号
    uint32_t crc32;        ///< 从 task_name 到结构体末尾的 CRC32-IEEE 校验值
    uint32_t progress;     ///< 写入进度：1 头部，2 故障寄存器，3 基本寄存器，4 栈上数据
    uint32_t count;        ///< 旧 magic 为 OK/BUSY 时加 1，否则从 1 开始；超过 UINT32_MAX 后回到 0
    uint32_t type;         ///< CrashType_e 中的记录类型
    uint32_t arg0;         ///< 附加数据；断言入口保存文件名地址，栈溢出入口填 0
    uint32_t arg1;         ///< 附加数据；断言入口保存行号，栈溢出入口填 0

    // 从这里到结构体末尾参与 CRC；任务名直接复制字符，供复位后读取。
    char task_name[CRASH_TASK_NAME_LEN];

    // 直接读取的故障状态；地址寄存器是否可用还要看相应状态位。
    uint32_t cfsr;         ///< 内存访问、总线和指令相关的故障状态
    uint32_t hfsr;         ///< HardFault 状态
    uint32_t bfar;         ///< 总线错误地址，需检查 CFSR 中对应的地址有效位
    uint32_t mmfar;        ///< 内存访问错误地址，需检查 CFSR 中对应的地址有效位
    uint32_t dfsr;         ///< 调试相关的故障状态

    // 硬件保存的 8 项基本寄存器，每项 4 字节；检查失败时填 0。
    // 无异常寄存器数据时，PC 使用 caller_pc，其余填 0。
    uint32_t r0, r1, r2, r3, r12, lr, pc, xpsr;
    uint32_t frame_status; ///< 0：无异常寄存器数据；1：检查通过；2：地址、返回标记或保存状态不合要求

    // 以下地址均按字节计。
    uint32_t sp;           ///< 栈上数据的读取起点；无异常寄存器数据时优先取非零 PSP，否则取 MSP
    uint32_t psp;          ///< 任务使用的栈指针 PSP
    uint32_t msp;          ///< 异常处理使用的主栈指针 MSP

    // 从 sp 起复制的 uint32_t 数据；无法读取的剩余位置填 0。
    uint32_t stack_snapshot[CRASH_STACK_SNAPSHOT_WORDS];
} CrashLog_t;

/**
 * @brief 检查上次崩溃记录，通过后打印现场，不清除记录。
 * @note 在日志初始化后调用。未通过检查时返回；其中 magic 为 BUSY 时会报告已保存的进度。
 */
void CrashLogInit(void);

/**
 * @brief 检查记录并复制给 LCD 或其他查询代码，不清除原记录。
 * @param[out] out 接收完整记录的结构体；为 NULL 时返回 0。
 * @return 1 表示检查通过且已复制；0 表示空指针或记录未通过检查，out 原有内容保持不变。
 * @note 复制时没有关闭中断，调用方须避免同时改写或清除记录。
 */
uint8_t CrashLogGetLast(CrashLog_t *out);

/**
 * @brief 将 magic 清零，使后续查询和启动打印忽略这份记录。
 * @note 其他字段及正在记录的标志保持原样；下一次写记录时，次数从 1 开始。
 */
void CrashLogClear(void);

/**
 * @brief 获取日志用的完整类型名。
 * @param[in] type CrashType_e 中的类型编号。
 * @return 程序内的只读字符串，无须释放；未识别的编号返回 "NONE"。
 */
const char *CrashLogTypeName(uint32_t type);
/**
 * @brief 获取 LCD 用的类型简称，最多 4 个字符。
 * @param[in] type CrashType_e 中的类型编号。
 * @return 程序内的只读字符串，无须释放；未识别的编号返回 "---"。
 */
const char *CrashLogTypeShort(uint32_t type);

/**
 * @brief 关闭已知输出，保存指定现场后请求系统复位，不返回。
 * @param[in] type 记录类型，使用 CrashType_e 中的值。
 * @param[in] frame 8 项基本寄存器数据的起点；为 NULL 时使用当前栈地址及 caller_pc。
 * @param[in] arg0 随记录保存的附加数据；断言入口用作文件名地址。
 * @param[in] arg1 随记录保存的附加数据；断言入口用作行号。
 * @param[in] caller_pc 无异常寄存器数据时保存的调用位置，按字节计；frame 非空时忽略。
 * @note 本入口按普通的 8 项布局检查 frame；含浮点区域的异常须由汇编入口传递真实返回标记。
 */
void CrashLogRecord(uint32_t type, uint32_t *frame, uint32_t arg0, uint32_t arg1,
                    uint32_t caller_pc);

/**
 * @brief 记录 configASSERT 失败的文件和行号，然后请求复位，不返回。
 * @param[in] file 文件名字符串的地址，须保持到复位后仍指向同一可读字符串；NULL 时省略文件名输出。
 * @param[in] line 源码行号。
 * @note 保存文件名地址，使用 __FILE__ 这样的常量字符串；打印前还会检查内容是否可打印。
 */
void CrashLogAssertFail(const char *file, uint32_t line);
/**
 * @brief 复制发生栈溢出的任务名，保存现场后请求复位，不返回。
 * @param[in] task_name 可读的任务名，NULL 时保存为空串；最多复制 15 个字符并补 '\0'。
 * @note 不可打印的字符替换为 '.'，任务名直接存入记录，复位后无需访问原来的字符数组。
 */
void CrashLogStackOverflow(const char *task_name);

/**
 * @brief 供异常汇编入口保存现场并请求复位，不返回。
 * @param[in] stack_frame 原始栈起点；为 NULL 时按无异常寄存器数据处理。
 * @param[in] type 异常类型，使用 CrashType_e 中相应的值。
 * @param[in] exception_return 进入异常时 LR 中的 EXC_RETURN 标记，用于区分普通及含浮点区域的布局。
 * @note 汇编入口须先关闭普通中断和已知输出；本函数检查是否重复进入，再记录，避免重复写关断寄存器。
 */
void CrashLogHandlerC(uint32_t *stack_frame, uint32_t type, uint32_t exception_return);

/** @brief 崩溃自测的编译开关；默认 0 不编译，设为 1 后轮流触发三种故障。 */
#ifndef CRASH_TEST
#define CRASH_TEST 0
#endif
#if CRASH_TEST
/** @brief 调用自测前等待的 daemon 循环次数；当前每次 10ms，默认约 3s，供启动日志先输出。 */
#ifndef CRASH_TEST_DELAY_LOOPS
#define CRASH_TEST_DELAY_LOOPS 300u
#endif
/**
 * @brief 按保留的顺序选择 BusFault、断言失败或任务栈溢出记录路径。
 * @note 故障正常触发后会记录并复位，下次启动选下一项；未触发故障时打印错误并返回。
 *       仅在 CRASH_TEST 为 1 时编译，由 daemon 等待指定循环次数后调用。
 */
void CrashLogSelfTest(void);
#endif

#endif // BSP_CRASH_H
