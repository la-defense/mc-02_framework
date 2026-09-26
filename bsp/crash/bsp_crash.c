#include "bsp_crash.h"
#include "main.h"
#include "bsp_log.h"

/* FreeRTOS 的头在这里是为了实现 vApplicationStackOverflowHook(栈溢出钩子),
   它由 configCHECK_FOR_STACK_OVERFLOW=2 触发。 */
#include "FreeRTOS.h"
#include "task.h"

/* "CRAS" 的小端写法: 用来判断 .noinit 里那份记录是不是有效的 */
#define CRASH_LOG_MAGIC 0x53415243u

/* 崩溃现场: 放在 .noinit 段 —— 启动代码的 FillZerobss 只清 .bss, 所以这一块
   能跨系统复位保留, 下次启动时由 CrashLogInit() 读出来。
   (掉电后内容丢失, 这是有意为之: handler 里不写 Flash, 避免二次故障。) */
__attribute__((section(".noinit"))) static CrashLog_t g_crash_log;

/* 判断一个指针像不像指向本固件的 Flash(字符串字面量都在那儿)。
   重新烧录过固件之后, 老的 __FILE__ 指针可能已经失效, 打印前必须挡一下。 */
static uint8_t CrashPtrPlausible(uint32_t p)
{
    /* 允许的指针范围: Flash(字符串字面量, 如断言的文件名) + 内部 RAM
       (DTCM 0x2000_0000 / AXI SRAM 0x2400_0000, 如 FreeRTOS TCB 里的任务名)。
       2026-09-26: 原来只认 Flash, 于是"栈溢出"记录里的任务名(在 TCB 里, 属于
       AXI SRAM)被判为不可信, 那一行"任务 '%s' 栈溢出"永远打不出来 —— 实测
       栈溢出时只能看到一堆寄存器, 定位不了是哪个任务。 */
    return ((p >= 0x08000000u && p < 0x08200000u) /* Flash */
            || (p >= 0x20000000u && p < 0x20020000u) /* DTCM */
            || (p >= 0x24000000u && p < 0x24020000u) /* AXI SRAM */
            )
               ? 1u
               : 0u;
}

void CrashLogRecord(uint32_t type, uint32_t *frame, uint32_t arg0, uint32_t arg1,
                    uint32_t caller_pc)
{
    /* !! 本函数只在异常上下文执行 !!
       异常可能由内存/总线错误引起, 此时任何函数调用、外设访问、日志输出都可能
       二次触发异常(CPU 进 Lockup, 连现场都保不住)。所以这里只做:
       "读寄存器 -> 写 RAM -> 复位"。顺序上先写数据、最后写 magic ——
       下次启动看到 magic 有效, 就说明这份记录是完整写下去的。 */

    g_crash_log.count++; /* 累加, 不清零: 用来知道"一共崩了几次" */
    g_crash_log.type = type;
    g_crash_log.arg0 = arg0;
    g_crash_log.arg1 = arg1;
    g_crash_log.progress = 1u;

    /* 故障状态寄存器(直接读, 不需要外设时钟) */
    g_crash_log.cfsr = SCB->CFSR;
    g_crash_log.hfsr = SCB->HFSR;
    g_crash_log.bfar = SCB->BFAR;
    g_crash_log.mmfar = SCB->MMFAR;
    g_crash_log.dfsr = SCB->DFSR;
    g_crash_log.progress = 2u;

    g_crash_log.psp = __get_PSP();
    g_crash_log.msp = __get_MSP();

    if (frame != NULL)
    {
        /* 异常压栈的 8 个字, 顺序由 ARM 硬件固定 */
        g_crash_log.r0 = frame[0];
        g_crash_log.r1 = frame[1];
        g_crash_log.r2 = frame[2];
        g_crash_log.r3 = frame[3];
        g_crash_log.r12 = frame[4];
        g_crash_log.lr = frame[5];
        g_crash_log.pc = frame[6];
        g_crash_log.xpsr = frame[7];
        g_crash_log.sp = (uint32_t)(uintptr_t)frame;
    }
    else
    {
        /* 断言/栈溢出这类"主动调用"没有异常帧: 直接取当前栈指针 */
        g_crash_log.r0 = g_crash_log.r1 = g_crash_log.r2 = g_crash_log.r3 = 0u;
        g_crash_log.r12 = g_crash_log.lr = g_crash_log.xpsr = 0u;
        g_crash_log.sp = (g_crash_log.psp != 0u) ? g_crash_log.psp : g_crash_log.msp;
        /* 没有异常帧就没有"出错 PC"; 用调用者给的返回地址代替 ——
           至少能看出断言/溢出是从哪一行代码报上来的, 否则 PC 只能是 0。 */
        g_crash_log.pc = caller_pc;
    }
    g_crash_log.progress = 3u;

    /* 栈快照: 从 sp 起连续拷若干字, 事后可以手工回溯调用链。
       纯内存读; 万一 sp 本身已经不可访问会再次异常, 但那时关键字段已经写好了。 */
    {
        const volatile uint32_t *p = (const volatile uint32_t *)(uintptr_t)g_crash_log.sp;
        for (uint32_t i = 0; i < CRASH_STACK_SNAPSHOT_WORDS; ++i)
            g_crash_log.stack_snapshot[i] = p[i];
    }
    g_crash_log.progress = 4u;

    /* 最后发布 magic: 保证"读到有效 magic" == "整份记录已写完" */
    g_crash_log.magic = CRASH_LOG_MAGIC;

    /* 主动复位, 不再死循环等看门狗 */
    NVIC_SystemReset();

    for (;;) /* 理论上到不了这里 */
        ;
}

/* noinline: __builtin_return_address(0) 要取"调用本函数的那一句"的地址,
   如果本函数被内联掉, 这个地址就没有意义了。 */
__attribute__((noinline)) void CrashLogAssertFail(const char *file, uint32_t line)
{
    CrashLogRecord(CRASH_TYPE_ASSERT, NULL, (uint32_t)(uintptr_t)file, line,
                   (uint32_t)(uintptr_t)__builtin_return_address(0));
}

__attribute__((noinline)) void CrashLogStackOverflow(const char *task_name)
{
    CrashLogRecord(CRASH_TYPE_STACK_OVF, NULL, (uint32_t)(uintptr_t)task_name, 0u,
                   (uint32_t)(uintptr_t)__builtin_return_address(0));
}

/**
 * @brief FreeRTOS 栈溢出钩子(由 configCHECK_FOR_STACK_OVERFLOW=2 触发)
 * @note  任务名指向 Flash 里的字符串字面量, 复位后地址依然有效,
 *        所以现场里只需要存这个指针, 下次启动时再打印出来。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    CrashLogStackOverflow((const char *)pcTaskName);
}

/* 异常入口的统一落点: 由 stm32h7xx_it.c 里的 naked handler 用
   "b CrashLogHandlerC" 跳进来(r0 = 异常栈帧指针, r1 = 异常类型)。 */
void CrashLogHandlerC(uint32_t *frame, uint32_t type)
{
    CrashLogRecord(type, frame, 0u, 0u, 0u); /* 有异常帧, PC 从帧里取 */
}

void CrashLogInit(void)
{
    if (g_crash_log.magic != CRASH_LOG_MAGIC)
    {
        /* 没有完整记录。但如果 progress 不为 0, 说明"写到一半就死了" ——
           这本身就是很有价值的线索(比如 4 = 栈快照那一步又触发了异常)。 */
        if (g_crash_log.progress != 0u && g_crash_log.progress <= 4u)
            LOGERROR("[crash] 上次崩溃记录不完整: 写到第 %lu 步就中断了 "
                     "(type=%lu count=%lu) —— 1=头部 2=寄存器 3=栈帧 4=栈快照",
                     (unsigned long)g_crash_log.progress,
                     (unsigned long)g_crash_log.type, (unsigned long)g_crash_log.count);
        return;
    }

    LOGERROR("==== 上次运行崩溃现场(累计第 %lu 次) ====",
             (unsigned long)g_crash_log.count);
    LOGERROR("[crash] type=%s pc=0x%08lX lr=0x%08lX sp=0x%08lX",
             CrashLogTypeName(g_crash_log.type),
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

    if (g_crash_log.type == CRASH_TYPE_ASSERT && CrashPtrPlausible(g_crash_log.arg0))
        LOGERROR("[crash] assert: %s:%lu", (const char *)g_crash_log.arg0,
                 (unsigned long)g_crash_log.arg1);
    if (g_crash_log.type == CRASH_TYPE_STACK_OVF && CrashPtrPlausible(g_crash_log.arg0))
        LOGERROR("[crash] 任务 '%s' 栈溢出", (const char *)g_crash_log.arg0);

    /* 栈快照: 每行 8 个字, 方便事后对照 map 文件手工回溯 */
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
    if (out == NULL || g_crash_log.magic != CRASH_LOG_MAGIC)
        return 0u;
    *out = g_crash_log;
    return 1u;
}

void CrashLogClear(void)
{
    g_crash_log.magic = 0u; /* 只清有效标志, 下次崩溃会重新写 */
}

const char *CrashLogTypeName(uint32_t type)
{
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
/* 自测游标也放 .noinit: 复位后继续, 于是能一次烧录连着验证三种崩溃 */
__attribute__((section(".noinit"))) static uint32_t s_crash_test_stage;

void CrashLogSelfTest(void)
{
    uint32_t stage = s_crash_test_stage % 3u;
    s_crash_test_stage++;
    LOGWARNING("[crash] 自测阶段 %lu/3", (unsigned long)stage + 1u);

    switch (stage)
    {
    case 0u: /* BusFault: 写一片没有外设的地址(FMC 区, 本板没接) */
        *(volatile uint32_t *)0xC0000000u = 0xDEADBEEFu;
        break;
    case 1u: /* 断言失败: 走 configASSERT -> CrashLogAssertFail */
        configASSERT(0);
        break;
    default: /* 栈溢出钩子的记录路径(直接调用, 避免真的踩坏内存) */
        CrashLogStackOverflow("selftest");
        break;
    }

    LOGERROR("[crash] 自测阶段 %lu 竟然没触发崩溃, 说明记录链路有问题",
             (unsigned long)stage + 1u);
}
#endif /* CRASH_TEST */
