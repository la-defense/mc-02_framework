#ifndef BSP_CRASH_H
#define BSP_CRASH_H

#include <stdint.h>

/* ============================================================================
   崩溃现场记录(2026-09-23, LOG-03)
   ----------------------------------------------------------------------------
   把所有"会让板子停在那里"的入口接到同一套现场记录上:
     - Cortex-M 的 5 个异常入口: HardFault / MemManage / BusFault / UsageFault / NMI
     - FreeRTOS 的 configASSERT 失败
     - FreeRTOS 的栈溢出钩子 (vApplicationStackOverflowHook)
   现场存在 **.noinit 段**(链接脚本里新增): 启动代码只清 .bss, 所以它能跨
   系统复位保留(掉电丢失), 下次启动由 CrashLogInit() 读出来打印 + 供 LCD 显示。

   设计约束(为什么 handler 里写得那么"素"):
     异常可能由**内存/总线错误**引起, 此时任何函数调用、外设访问、日志输出都可能
     二次触发异常(CPU 会进 Lockup, 连现场都保不住)。所以记录过程只做:
       "读寄存器 → 写 RAM → 复位", 不调用函数、不写日志、不碰外设。
   ========================================================================== */

/* 崩溃入口类型 */
typedef enum
{
    CRASH_TYPE_NONE = 0,
    CRASH_TYPE_HARDFAULT,
    CRASH_TYPE_MEMMANAGE,
    CRASH_TYPE_BUSFAULT,
    CRASH_TYPE_USAGEFAULT,
    CRASH_TYPE_NMI,
    CRASH_TYPE_ASSERT,    /* configASSERT 失败 */
    CRASH_TYPE_STACK_OVF, /* 任务栈溢出 */
} CrashType_e;

/* 保存多少个栈上数据(用于事后手工回溯调用链) */
#define CRASH_STACK_SNAPSHOT_WORDS 32u

typedef struct
{
    /* ---- 头部 ---- */
    uint32_t magic; /* = CRASH_LOG_MAGIC 表示这份记录有效(最后写, 保证"看到它就说明写完了") */
    uint32_t progress; /* 写入进度: 1=已写头部 2=已写故障寄存器 3=已写栈帧 4=已写栈快照
                          用来诊断"记录写到一半就死了"(magic 无效但 progress 有值) */
    uint32_t count; /* 累计崩溃次数(跨复位累加, 不清零) */
    uint32_t type;  /* CrashType_e */
    uint32_t arg0;  /* 附加信息: 断言所在的文件名指针 / 溢出任务名指针 */
    uint32_t arg1;  /* 附加信息: 断言行号 */

    /* ---- 故障状态寄存器(直接读寄存器得到) ---- */
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t bfar;
    uint32_t mmfar;
    uint32_t dfsr;

    /* ---- 出错时的异常栈帧(ARM 硬件压栈的 8 个字) ---- */
    uint32_t r0, r1, r2, r3, r12, lr, pc, xpsr;

    /* ---- 栈指针 ---- */
    uint32_t sp;  /* 出错时真正使用的栈指针(MSP 或 PSP) */
    uint32_t psp;
    uint32_t msp;

    /* ---- 栈快照(从 sp 起连续若干字, 便于事后手工回溯) ---- */
    uint32_t stack_snapshot[CRASH_STACK_SNAPSHOT_WORDS];
} CrashLog_t;

/** @brief 启动早期调用: 校验并打印上次崩溃现场(没有记录则什么都不做) */
void CrashLogInit(void);

/** @brief 读取上次崩溃现场; 返回 1=有有效记录(供 LCD / 调试读取) */
uint8_t CrashLogGetLast(CrashLog_t *out);

/** @brief 清空记录(下次启动就不再报告) */
void CrashLogClear(void);

/** @brief 类型名(日志用, 较长) */
const char *CrashLogTypeName(uint32_t type);
/** @brief 类型短名(≤4 字符, LCD 用) */
const char *CrashLogTypeShort(uint32_t type);

/* ---------------- 以下函数在异常上下文被调用 ----------------
   frame:     指向异常压栈的 8 个字(R0,R1,R2,R3,R12,LR,PC,xPSR); 没有帧时传 NULL
   arg0/arg1: 附加信息(断言=文件名指针+行号, 栈溢出=任务名指针)
   caller_pc: 没有异常帧时用作"出错位置"(由调用者用 __builtin_return_address(0) 取得);
              有异常帧时忽略(帧里的 PC 更准)
   这些函数记录完现场就主动复位, 不会返回。 */
void CrashLogRecord(uint32_t type, uint32_t *frame, uint32_t arg0, uint32_t arg1,
                    uint32_t caller_pc);

/** @brief FreeRTOS configASSERT 失败时调用 */
void CrashLogAssertFail(const char *file, uint32_t line);
/** @brief FreeRTOS 栈溢出钩子调用 */
void CrashLogStackOverflow(const char *task_name);

/* 异常入口的统一落点: stm32h7xx_it.c 里那几个 naked handler 用
   "b CrashLogHandlerC" 跳进来, r0 = 异常栈帧指针, r1 = 异常类型。 */
void CrashLogHandlerC(uint32_t *frame, uint32_t type);

/* ---------------- 崩溃自测(默认关闭) ----------------
   置 1 后: 每次启动依次触发 BusFault / 断言失败 / 栈溢出记录, 用来验证
   "记录 → 主动复位 → 重启后打印"整条链路。用 .noinit 里一个游标轮流,
   所以一次烧录能连着验证三种; 量产固件保持 0, 这段代码不参与编译。 */
#ifndef CRASH_TEST
#define CRASH_TEST 0
#endif
#if CRASH_TEST
/* 自测前先等多少个 daemon 循环(10ms/轮): 300 轮 ≈ 3s, 让启动日志先输出完 */
#ifndef CRASH_TEST_DELAY_LOOPS
#define CRASH_TEST_DELAY_LOOPS 300u
#endif
void CrashLogSelfTest(void);
#endif

#endif // BSP_CRASH_H
