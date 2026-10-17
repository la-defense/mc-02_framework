/**
 * @file bsp_crash.c
 * @brief 保存崩溃现场、复位后检查并输出记录，接口用法见 bsp_crash.h。
 * 故障入口先关闭已知输出，按检查结果读取寄存器和栈，再请求复位；日志只在下次启动时输出。
 */
#include "bsp_crash.h"
#include "bsp_crash_crc.h"
#include "bsp_crash_frame.h"
#include "bsp_crash_guard.h"
#include "bsp_safety.h"
#include "main.h"
#include "bsp_log.h"

#include <stddef.h> /* offsetof */

// FreeRTOS 头文件提供栈溢出回调的参数类型及任务名长度。
#include "FreeRTOS.h"
#include "task.h"

// .noinit 不由启动代码清零，供复位后读取；记录过程中只写 RAM，不写 Flash。
__attribute__((section(".noinit"))) static CrashLog_t g_crash_log;
// 这个标志在每次启动时清零，同一次记录中再次进入就请求复位。
static volatile uint32_t g_crash_handler_active;

// 编号使用本文件的编译日期、时间和下方配置名；其他文件单独改变时，编号未必改变。
static const char kCrashBuildStamp[] = __DATE__ " " __TIME__;

// 分别记录诊断裸板、普通裸板和整机配置，便于查询并区分保存记录时的配置。
#if MC02_BENCH_RC_DIAGNOSTICS
const char mc02_build_profile[] = "rcdiag_bench_safe";
#elif defined(MC02_PROFILE_BENCH_SAFE)
const char mc02_build_profile[] = "bench_safe";
#else
const char mc02_build_profile[] = "robot";
#endif

// FNV-1a 计算 32 位编号：先逐字符处理配置名，再处理编译时间，各自遇到 '\0' 停止。
// 用宏在当前位置计算，减少记录过程中的函数调用。
#define CRASH_FW_ID_COMPUTE(out)                                          \
    do {                                                                  \
        const char *_cs = mc02_build_profile;                             \
        (out) = 2166136261u;                                              \
        while (*_cs != '\0')                                              \
        {                                                                 \
            (out) = ((out) ^ (uint32_t)(uint8_t)(*_cs)) * 16777619u;      \
            _cs++;                                                        \
        }                                                                 \
        _cs = kCrashBuildStamp;                                           \
        while (*_cs != '\0')                                              \
        {                                                                 \
            (out) = ((out) ^ (uint32_t)(uint8_t)(*_cs)) * 16777619u;      \
            _cs++;                                                        \
        }                                                                 \
    } while (0)

// CRC 从 task_name 算到结构体末尾，前面的头部字段不参与计算。
#define CRASH_PAYLOAD_OFFSET ((uint32_t)offsetof(CrashLog_t, task_name))
#define CRASH_PAYLOAD_WORDS \
    (((uint32_t)sizeof(CrashLog_t) - CRASH_PAYLOAD_OFFSET) / 4u)

// 写完记录后、请求复位前的等待次数，给最后的内存写入留出时间。
#define CRASH_RESET_DRAIN_LOOPS 20000u

// 编译时检查记录长度及 CRC 起点是否为 4 的倍数，以及任务名数组是否足够大。
_Static_assert(sizeof(CrashLog_t) % 4u == 0u,
               "崩溃记录长度必须是 4 的整数倍(逐字校验和的前提)");
_Static_assert(CRASH_PAYLOAD_OFFSET % 4u == 0u, "payload 必须 4 字节对齐");
_Static_assert(CRASH_TASK_NAME_LEN >= configMAX_TASK_NAME_LEN,
               "CRASH_TASK_NAME_LEN 放不下 FreeRTOS 的任务名");

// 按下面列出的 Flash、DTCM 和 AXI SRAM 范围检查字符串起始地址，不检查整串数据的范围。
static uint8_t CrashPtrPlausible(uint32_t p)
{
    // 文件名通常在 Flash，RAM 中的字符串也允许进入后面的内容检查。
    return ((p >= 0x08000000u && p < 0x08200000u) /* Flash */
            || (p >= 0x20000000u && p < 0x20020000u) /* DTCM */
            || (p >= 0x24000000u && p < 0x24020000u) /* AXI SRAM */
            )
               ? 1u
               : 0u;
}

// 最多读 192 字节，检查是否为非空、可打印并以 '\0' 结束的字符串。
static uint8_t CrashLogStringPlausible(uint32_t p)
{
    const char *s;

    // 起点不在允许范围内就跳过内容读取。
    if (!CrashPtrPlausible(p))
        return 0u;

    s = (const char *)(uintptr_t)p;
    // 文件名可能包含完整路径，逐字符检查，遇到结尾或不可打印字符时提前结束。
    for (uint32_t i = 0u; i < 192u; ++i)
    {
        char c = s[i];

        if (c == '\0')
            return (i > 0u) ? 1u : 0u; // 空串不作为文件名输出
        if (c < 0x20 || c > 0x7E)
            return 0u; // 避免把其他数据当作字符串打印
    }
    return 0u; // 192 字节内没有结尾，不继续向后读取
}

// 依次核对写入状态、长度、版本、编译编号及 CRC，决定记录能否使用。
static uint8_t CrashLogValid(void)
{
    uint32_t fw_id;
    uint32_t crc;

    // 除 OK 外，BUSY 且进度为 4 的记录也继续检查，兼容最后一次状态写入未保留的情况。
    if (g_crash_log.magic != CRASH_LOG_MAGIC_OK)
    {
        if ((g_crash_log.magic != CRASH_LOG_MAGIC_BUSY) ||
            (g_crash_log.progress != 4u))
            return 0u;
    }
    if (g_crash_log.len != (uint32_t)sizeof(CrashLog_t))
        return 0u; // 长度不匹配，后面的字段不能按当前结构使用
    if (g_crash_log.version != CRASH_LOG_VERSION)
        return 0u; // 记录格式版本不匹配

    CRASH_FW_ID_COMPUTE(fw_id);
    if (g_crash_log.fw_id != fw_id)
        return 0u; // 保存时的编译编号与当前编号不匹配

    if (!CrashCRC32Words((const volatile uint32_t *)((const volatile uint8_t *)&g_crash_log +
                            CRASH_PAYLOAD_OFFSET), CRASH_PAYLOAD_WORDS, &crc))
        return 0u;
    return (crc == g_crash_log.crc32) ? 1u : 0u;
}

// 先检查是否重复进入，再关闭普通中断和已知输出，供主动报告故障的入口使用。
static void CrashLogBegin(void)
{
    CrashLogGuardEnter(&g_crash_handler_active);
    BSP_SafetyLatchOutputsOff();
}

// 调用前须已关闭普通中断和已知输出，并通过重复进入检查；写现场期间不使用日志、HAL 或 RTOS。
static void CrashLogWriteRecord(uint32_t type, uint32_t *frame, uint32_t exception_return,
                                uint32_t arg0, uint32_t arg1, uint32_t caller_pc)
{
    // 先标记为正在写；途中再次故障时，下次启动可以报告已保存的进度。
    {
        uint32_t prev_magic = g_crash_log.magic;

        g_crash_log.magic = CRASH_LOG_MAGIC_BUSY;
        g_crash_log.len = (uint32_t)sizeof(CrashLog_t);
        g_crash_log.version = CRASH_LOG_VERSION;
        CRASH_FW_ID_COMPUTE(g_crash_log.fw_id);

        // 这里只看旧 magic：认得 OK/BUSY 就累加，否则从 1 开始，避免直接使用未初始化的次数。
        if (prev_magic == CRASH_LOG_MAGIC_OK || prev_magic == CRASH_LOG_MAGIC_BUSY)
            g_crash_log.count++;
        else
            g_crash_log.count = 1u;
    }
    g_crash_log.type = type;
    g_crash_log.arg0 = arg0;
    g_crash_log.arg1 = arg1;
    g_crash_log.progress = 1u;

    // 栈溢出入口已经复制任务名；其他类型清空数组，避免留下旧任务名。
    if (type != CRASH_TYPE_STACK_OVF)
    {
        for (uint32_t i = 0u; i < CRASH_TASK_NAME_LEN; ++i)
            g_crash_log.task_name[i] = '\0';
    }

    // 直接读取故障状态寄存器，供复位后判断原因。
    g_crash_log.cfsr = SCB->CFSR;
    g_crash_log.hfsr = SCB->HFSR;
    g_crash_log.bfar = SCB->BFAR;
    g_crash_log.mmfar = SCB->MMFAR;
    g_crash_log.dfsr = SCB->DFSR;
    g_crash_log.progress = 2u;

    g_crash_log.psp = __get_PSP();
    g_crash_log.msp = __get_MSP();
    uint32_t snapshot_words = 0u;

    // 异常入口提供了原始栈地址时，先检查整个寄存器区域再读取。
    if (frame != NULL)
    {
        CrashLogFrameView_t frame_view;
        g_crash_log.sp = (uint32_t)(uintptr_t)frame;
        // 检查通过后，core_frame_address 已跳过可能存在的浮点区域。
        if (CrashLogFrameResolve((uintptr_t)frame, exception_return, g_crash_log.cfsr,
                                 &frame_view))
        {
            const volatile uint32_t *core_frame =
                (const volatile uint32_t *)frame_view.core_frame_address;
            g_crash_log.r0 = core_frame[0];
            g_crash_log.r1 = core_frame[1];
            g_crash_log.r2 = core_frame[2];
            g_crash_log.r3 = core_frame[3];
            g_crash_log.r12 = core_frame[4];
            g_crash_log.lr = core_frame[5];
            g_crash_log.pc = core_frame[6];
            g_crash_log.xpsr = core_frame[7];
            g_crash_log.frame_status = 1u;
            snapshot_words = frame_view.snapshot_words;
        }
        else
        {
            // 地址、返回标记或保存状态不合要求，寄存器填 0，并保持栈读取数量为 0。
            g_crash_log.r0 = g_crash_log.r1 = g_crash_log.r2 = g_crash_log.r3 = 0u;
            g_crash_log.r12 = g_crash_log.lr = g_crash_log.pc = g_crash_log.xpsr = 0u;
            g_crash_log.frame_status = 2u;
        }
    }
    else
    {
        // 主动报告断言或栈溢出时，使用当前的栈地址，优先取非零 PSP，否则取 MSP。
        g_crash_log.r0 = g_crash_log.r1 = g_crash_log.r2 = g_crash_log.r3 = 0u;
        g_crash_log.r12 = g_crash_log.lr = g_crash_log.xpsr = 0u;
        g_crash_log.frame_status = 0u;
        g_crash_log.sp = (g_crash_log.psp != 0u) ? g_crash_log.psp : g_crash_log.msp;
        snapshot_words = CrashLogReadableStackWords((uintptr_t)g_crash_log.sp);
        // 这条路径没有硬件保存的 PC，使用调用方传入的位置帮助查找报告入口。
        g_crash_log.pc = caller_pc;
    }
    g_crash_log.progress = 3u;

    // 最多复制检查允许的 uint32_t 个数，其余位置填 0，避免读出允许的 RAM 范围。
    {
        const volatile uint32_t *p = (const volatile uint32_t *)(uintptr_t)g_crash_log.sp;
        for (uint32_t i = 0; i < CRASH_STACK_SNAPSHOT_WORDS; ++i)
            g_crash_log.stack_snapshot[i] = (i < snapshot_words) ? p[i] : 0u;
    }
    g_crash_log.progress = 4u;

    // 先为 task_name 及其后的字段计算 CRC，再写 OK，供下次启动检查。
    {
        uint32_t crc;

        if (CrashCRC32Words((const volatile uint32_t *)((const volatile uint8_t *)&g_crash_log +
                                CRASH_PAYLOAD_OFFSET), CRASH_PAYLOAD_WORDS, &crc))
        {
            g_crash_log.crc32 = crc;
            g_crash_log.magic = CRASH_LOG_MAGIC_OK;
        }
        // 未算出 CRC 时保留 BUSY，仍按原路径复位，不发布完成标记。
    }

    // 请求复位前等待固定次数，给最后的写入留出时间；读取时仍兼容 BUSY 且进度为 4 的记录。
    for (volatile uint32_t _drain = 0u; _drain < CRASH_RESET_DRAIN_LOOPS; ++_drain)
        ;

    // 写完后主动请求复位，下一次启动再输出日志。
    NVIC_SystemReset();

    // 复位函数正常不会返回；意外返回时停在这里，避免继续执行。
    for (;;)
        ;
}

void CrashLogRecord(uint32_t type, uint32_t *frame, uint32_t arg0, uint32_t arg1,
                    uint32_t caller_pc)
{
    CrashLogBegin();
    CrashLogWriteRecord(type, frame, 0xFFFFFFF9u, arg0, arg1, caller_pc);
}

// 保留独立函数调用，让 __builtin_return_address(0) 指向调用本函数的位置。
__attribute__((noinline)) void CrashLogAssertFail(const char *file, uint32_t line)
{
    CrashLogRecord(CRASH_TYPE_ASSERT, NULL, (uint32_t)(uintptr_t)file, line,
                   (uint32_t)(uintptr_t)__builtin_return_address(0));
}

__attribute__((noinline)) void CrashLogStackOverflow(const char *task_name)
{
    // 先检查重复进入并关闭输出，再读取任务名；读取时再次故障会进入复位路径。
    CrashLogBegin();

    // 直接保存任务名字符，避免复位后访问已被重新使用的任务内存；最后一个位置留给 '\0'。
    for (uint32_t i = 0u; i < CRASH_TASK_NAME_LEN; ++i)
    {
        char c = '\0';

        if ((task_name != NULL) && (i < (CRASH_TASK_NAME_LEN - 1u)))
            c = task_name[i];
        // 非空且不能打印的字符替换为 '.'，供日志和 LCD 正常显示。
        if ((c != '\0') && ((c < 0x20) || (c > 0x7E)))
            c = '.';

        g_crash_log.task_name[i] = c;
        // 遇到原有结尾、空指针或已用完字符位置时停止复制。
        if (c == '\0')
            break;
    }

    CrashLogWriteRecord(CRASH_TYPE_STACK_OVF, NULL, 0u, 0u, 0u,
                        (uint32_t)(uintptr_t)__builtin_return_address(0));
}

/**
 * @brief FreeRTOS 检测到任务栈溢出时调用，复制任务名、保存现场并请求复位，不返回。
 * @param[in] xTask 发生溢出的任务，本实现不使用该参数。
 * @param[in] pcTaskName 任务名字符数组，允许 NULL；交给 CrashLogStackOverflow() 复制。
 * @note 由 configCHECK_FOR_STACK_OVERFLOW=2 启用检测。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    CrashLogStackOverflow((const char *)pcTaskName);
}

void CrashLogHandlerC(uint32_t *stack_frame, uint32_t type, uint32_t exception_return)
{
    // 汇编入口已关闭中断和已知输出，这里直接检查重复进入，避免再写一遍关断寄存器。
    CrashLogGuardEnter(&g_crash_handler_active);
    CrashLogWriteRecord(type, stack_frame, exception_return, 0u, 0u, 0u);
}

void CrashLogInit(void)
{
    // 未通过检查时跳过完整现场输出；若仍是 BUSY，保留一行进度信息供排查。
    if (!CrashLogValid())
    {
        if (g_crash_log.magic == CRASH_LOG_MAGIC_BUSY)
            LOGERROR("[crash] 上次崩溃记录写到一半就中断了(第 %lu 步): type=%lu count=%lu, "
                     "现场不完整已忽略 —— 1=头部 2=寄存器 3=栈帧 4=栈快照",
                     (unsigned long)g_crash_log.progress,
                     (unsigned long)g_crash_log.type, (unsigned long)g_crash_log.count);
        return;
    }

    LOGERROR("==== 上次运行崩溃现场(累计第 %lu 次) ====",
             (unsigned long)g_crash_log.count);
    LOGERROR("[crash] type=%s frame=%s pc=0x%08lX lr=0x%08lX sp=0x%08lX",
             CrashLogTypeName(g_crash_log.type),
             (g_crash_log.frame_status == 1u) ? "valid" :
             (g_crash_log.frame_status == 2u) ? "invalid" : "none",
             (unsigned long)g_crash_log.pc, (unsigned long)g_crash_log.lr,
             (unsigned long)g_crash_log.sp);
    LOGERROR("[crash] CFSR=0x%08lX HFSR=0x%08lX BFAR=0x%08lX MMFAR=0x%08lX DFSR=0x%08lX",
             (unsigned long)g_crash_log.cfsr, (unsigned long)g_crash_log.hfsr,
             (unsigned long)g_crash_log.bfar, (unsigned long)g_crash_log.mmfar,
             (unsigned long)g_crash_log.dfsr);
    LOGERROR("[crash] R0=%08lX R1=%08lX R2=%08lX R3=%08lX R12=%08lX xPSR=%08lX",
             (unsigned long)g_crash_log.r0, (unsigned long)g_crash_log.r1,
             (unsigned long)g_crash_log.r2, (unsigned long)g_crash_log.r3,
             (unsigned long)g_crash_log.r12, (unsigned long)g_crash_log.xpsr);
    LOGERROR("[crash] MSP=0x%08lX PSP=0x%08lX",
             (unsigned long)g_crash_log.msp, (unsigned long)g_crash_log.psp);

    // 文件名保存的是地址，输出前再次检查起点和字符，避免直接把旧内容当字符串。
    if (g_crash_log.type == CRASH_TYPE_ASSERT &&
        CrashLogStringPlausible(g_crash_log.arg0))
        LOGERROR("[crash] assert: %s:%lu", (const char *)g_crash_log.arg0,
                 (unsigned long)g_crash_log.arg1);
    // 任务名已直接存入记录，栈溢出类型才输出这一项。
    if (g_crash_log.type == CRASH_TYPE_STACK_OVF)
        LOGERROR("[crash] 任务 '%s' 栈溢出", g_crash_log.task_name);

    // 每行输出 8 项 uint32_t，行首为相对 sp 的字节数，便于对照 map 文件查找调用位置。
    for (uint32_t row = 0; row < (CRASH_STACK_SNAPSHOT_WORDS / 8u); ++row)
    {
        const uint32_t *s = &g_crash_log.stack_snapshot[row * 8u];
        LOGINFO("[crash] sp+%02lu: %08lX %08lX %08lX %08lX %08lX %08lX %08lX %08lX",
                (unsigned long)(row * 8u * 4u),
                (unsigned long)s[0], (unsigned long)s[1], (unsigned long)s[2],
                (unsigned long)s[3], (unsigned long)s[4], (unsigned long)s[5],
                (unsigned long)s[6], (unsigned long)s[7]);
    }
    LOGERROR("==== 现场结束(pc 用 arm-none-eabi-addr2line 反查) ====");
}

uint8_t CrashLogGetLast(CrashLog_t *out)
{
    // 空指针或记录未通过检查时直接返回，保留调用方原有的输出内容。
    if (out == NULL || !CrashLogValid())
        return 0u;
    *out = g_crash_log;
    return 1u;
}

void CrashLogClear(void)
{
    g_crash_log.magic = 0u; // 只使当前记录失效，其他字段保留
}

const char *CrashLogTypeName(uint32_t type)
{
    // 按类型选择日志名称，未知编号走 default。
    switch (type)
    {
    case CRASH_TYPE_HARDFAULT:
        return "HARDFAULT";
    case CRASH_TYPE_MEMMANAGE:
        return "MEMMANAGE";
    case CRASH_TYPE_BUSFAULT:
        return "BUSFAULT";
    case CRASH_TYPE_USAGEFAULT:
        return "USAGEFAULT";
    case CRASH_TYPE_NMI:
        return "NMI";
    case CRASH_TYPE_ASSERT:
        return "ASSERT";
    case CRASH_TYPE_STACK_OVF:
        return "STACKOVF";
    default:
        return "NONE";
    }
}

const char *CrashLogTypeShort(uint32_t type)
{
    // LCD 只显示较短的名称，未知编号用 "---" 占位。
    switch (type)
    {
    case CRASH_TYPE_HARDFAULT:
        return "HF";
    case CRASH_TYPE_MEMMANAGE:
        return "MM";
    case CRASH_TYPE_BUSFAULT:
        return "BUS";
    case CRASH_TYPE_USAGEFAULT:
        return "UDF";
    case CRASH_TYPE_NMI:
        return "NMI";
    case CRASH_TYPE_ASSERT:
        return "ASRT";
    case CRASH_TYPE_STACK_OVF:
        return "STK";
    default:
        return "---";
    }
}

#if CRASH_TEST
// 保存已选过的自测次数，复位后从下一种故障继续。
__attribute__((section(".noinit"))) static uint32_t s_crash_test_stage;

void CrashLogSelfTest(void)
{
    uint32_t stage = s_crash_test_stage % 3u;
    s_crash_test_stage++;
    LOGWARNING("[crash] 自测阶段 %lu/3", (unsigned long)stage + 1u);

    // 每次启动依次选择三种路径中的一种，第三项后重新从第一项开始。
    switch (stage)
    {
    case 0u: // 尝试通过写未连接的 FMC 区域触发 BusFault
        *(volatile uint32_t *)0xC0000000u = 0xDEADBEEFu;
        break;
    case 1u: // 通过 configASSERT 进入断言失败记录
        configASSERT(0);
        break;
    default: // 直接调用栈溢出记录入口，检查这条记录路径
        CrashLogStackOverflow("selftest");
        break;
    }

    LOGERROR("[crash] 自测阶段 %lu 竟然没触发崩溃, 说明记录链路有问题",
             (unsigned long)stage + 1u);
}
#endif /* CRASH_TEST */
