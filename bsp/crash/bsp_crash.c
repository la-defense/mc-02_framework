#include "bsp_crash.h"
#include "bsp_crash_frame.h"
#include "bsp_crash_guard.h"
#include "bsp_safety.h"
#include "main.h"
#include "bsp_log.h"

#include <stddef.h> /* offsetof */

/* FreeRTOS 的头在这里是为了实现 vApplicationStackOverflowHook(栈溢出钩子),
   它由 configCHECK_FOR_STACK_OVERFLOW=2 触发。 */
#include "FreeRTOS.h"
#include "task.h"

/* 崩溃现场: 放在 .noinit 段 —— 启动代码的 FillZerobss 只清 .bss, 所以这一块
   能跨系统复位保留, 下次启动时由 CrashLogInit() 读出来。
   (掉电后内容丢失, 这是有意为之: handler 里不写 Flash, 避免二次故障。) */
__attribute__((section(".noinit"))) static CrashLog_t g_crash_log;
static volatile uint32_t g_crash_handler_active;

/* ---------------- 构建指纹 (2026-09-26, LOG-05) ----------------
   由编译时间派生: 换一个固件(哪怕只改了一行)指纹就变, 于是"上一个固件留下的
   记录"不会被当成这次的现场打印出来(那种记录的 PC 在新 ELF 上也反查不出来)。
   注意 __DATE__/__TIME__ 是在**本文件**展开的, 所以指纹对本文件所在的构建单元
   敏感 —— 对我们来说足够了, 改动代码通常都会重编到它(而且还有 crc32 兜底)。 */
static const char kCrashBuildStamp[] = __DATE__ " " __TIME__;

/* 当前构建配置保留为可查询字符串，并纳入崩溃构建指纹。 */
#if defined(MC02_PROFILE_BENCH_SAFE)
const char mc02_build_profile[] = "bench_safe";
#else
const char mc02_build_profile[] = "robot";
#endif

/* FNV-1a(32bit)。写成**宏**而不是函数: 异常上下文里要保持
   "纯内存写、不调用任何函数"的约束(见 bsp_crash.h 顶部说明)。 */
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

/* CRC32-IEEE(多项式 0xEDB88320, 与 bsp/param 的参数区同一套), 逐 32bit 字计算。
   写成宏的理由同上。用法: CRASH_CRC32_WORDS(acc, words_ptr, nwords);
   注意: 小端机上空口扫描"字"与扫描"字节"的顺序一致, 所以和 bsp/param 的
   字节版结果相同。 */
#define CRASH_CRC32_WORDS(acc, p, n)                                      \
    do {                                                                  \
        (acc) = 0xFFFFFFFFu;                                              \
        for (uint32_t _ci = 0u; _ci < (n); ++_ci)                         \
        {                                                                 \
            uint32_t _cw = (p)[_ci];                                      \
            for (uint32_t _cb = 0u; _cb < 4u; ++_cb)                       \
            {                                                             \
                (acc) ^= (_cw & 0xFFu);                                   \
                _cw >>= 8;                                                \
                for (uint32_t _ck = 0u; _ck < 8u; ++_ck)                   \
                    (acc) = ((acc) & 1u) ? (((acc) >> 1) ^ 0xEDB88320u)    \
                                         : ((acc) >> 1);                   \
            }                                                             \
        }                                                                 \
        (acc) = ~(acc);                                                   \
    } while (0)

/* payload 的范围: 头部(到 task_name 之前)不参与校验和,
   与参数区 bsp/param 的"header + crc(payload)"是同一种结构。 */
#define CRASH_PAYLOAD_OFFSET ((uint32_t)offsetof(CrashLog_t, task_name))
#define CRASH_PAYLOAD_WORDS \
    (((uint32_t)sizeof(CrashLog_t) - CRASH_PAYLOAD_OFFSET) / 4u)

/* 复位前等待"最后一次写落地"的空转次数: 见 CrashLogRecord 末尾的说明。
   volatile 循环在 -Og/-O2 下都是真实的内存访问循环, 约 0.1~0.2ms,
   只在崩溃路径执行一次, 代价可以忽略。 */
#define CRASH_RESET_DRAIN_LOOPS 20000u

/* 编译期兜底: 逐字 CRC 要求 payload 4 字节对齐且长度是 4 的整数倍;
   任务名数组必须放得下 FreeRTOS 的 configMAX_TASK_NAME_LEN。 */
_Static_assert(sizeof(CrashLog_t) % 4u == 0u,
               "崩溃记录长度必须是 4 的整数倍(逐字校验和的前提)");
_Static_assert(CRASH_PAYLOAD_OFFSET % 4u == 0u, "payload 必须 4 字节对齐");
_Static_assert(CRASH_TASK_NAME_LEN >= configMAX_TASK_NAME_LEN,
               "CRASH_TASK_NAME_LEN 放不下 FreeRTOS 的任务名");

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

/**
 * @brief 判断一个指针是不是"像样的可打印字符串"
 * @note  地址范围先过 CrashPtrPlausible, 再要求前 64 字节都是可打印 ASCII 且
 *        有 NUL 结尾。用来挡住"指针本身合法、但内容已经是别的数据"的情况
 *        (比如换了固件后 __FILE__ 指到了别处) —— 宁可少打一行, 也不打出一串乱码。
 */
static uint8_t CrashLogStringPlausible(uint32_t p)
{
    const char *s;

    if (!CrashPtrPlausible(p))
        return 0u;

    s = (const char *)(uintptr_t)p;
    /* 上限取 192: 本工程 __FILE__ 是完整路径(实测 65 字符), 原来写 64 反而把它挡掉了 */
    for (uint32_t i = 0u; i < 192u; ++i)
    {
        char c = s[i];

        if (c == '\0')
            return (i > 0u) ? 1u : 0u; /* 空串也不算 */
        if (c < 0x20 || c > 0x7E)
            return 0u; /* 不可打印 → 内容已经不是字符串了 */
    }
    return 0u; /* 192 字节还没结束: 可疑 */
}

/**
 * @brief 这份记录到底是不是"当前这个固件"写的?
 * @note  四重判据: magic(=OK) + len + version + fw_id + crc32。
 *        任何一个不符就当"没有记录" —— 残留数据、旧固件、换过布局的记录
 *        一律静默忽略, 启动时不打印任何东西。
 */
static uint8_t CrashLogValid(void)
{
    uint32_t fw_id;
    uint32_t crc;

    if (g_crash_log.magic != CRASH_LOG_MAGIC_OK)
    {
        /* 兜底: "正在写"标记(BUSY) + progress=4 + payload 校验和正确 ——
           说明整份记录其实写全了, 只是"发布用的那条 magic 写"在系统复位时丢了
           (2026-09-26 实测到这种硬件行为, 见 CrashLogRecord 末尾的说明)。
           这种记录一样可信, 照常接受。 */
        if ((g_crash_log.magic != CRASH_LOG_MAGIC_BUSY) ||
            (g_crash_log.progress != 4u))
            return 0u;
    }
    if (g_crash_log.len != (uint32_t)sizeof(CrashLog_t))
        return 0u; /* 记录布局变过 */
    if (g_crash_log.version != CRASH_LOG_VERSION)
        return 0u; /* 记录格式版本不符 */

    CRASH_FW_ID_COMPUTE(fw_id);
    if (g_crash_log.fw_id != fw_id)
        return 0u; /* 别的固件写的 */

    CRASH_CRC32_WORDS(crc,
                      (const uint32_t *)((const uint8_t *)&g_crash_log +
                                         CRASH_PAYLOAD_OFFSET),
                      CRASH_PAYLOAD_WORDS);
    return (crc == g_crash_log.crc32) ? 1u : 0u;
}

static void CrashLogBegin(void)
{
    CrashLogGuardEnter(&g_crash_handler_active);
    BSP_SafetyLatchOutputsOff();
}

static void CrashLogWriteRecord(uint32_t type, uint32_t *frame, uint32_t exception_return,
                                uint32_t arg0, uint32_t arg1, uint32_t caller_pc)
{
    /* CrashLogBegin has already disabled interrupts and directly shut down
       controlled outputs. Avoid HAL, RTOS, logging, and unbounded reads here;
       stack access is limited to ranges validated by bsp_crash_frame.c. */

    /* 1) 先把头部写成"正在记录"状态(magic=BUSY): 万一记录过程中又崩了,
          下次启动会看到 BUSY 并报一句"写到第 N 步就中断了" —— 这是真线索。 */
    {
        uint32_t prev_magic = g_crash_log.magic;

        g_crash_log.magic = CRASH_LOG_MAGIC_BUSY;
        g_crash_log.len = (uint32_t)sizeof(CrashLog_t);
        g_crash_log.version = CRASH_LOG_VERSION;
        CRASH_FW_ID_COMPUTE(g_crash_log.fw_id);

        /* 累计次数: 只有"看起来是我们写的记录"(OK/BUSY)才继续累加, 否则从 1 开始。
           不然残留数据里的垃圾数字会被当成"累计第 2261714040 次"打印出来。 */
        if (prev_magic == CRASH_LOG_MAGIC_OK || prev_magic == CRASH_LOG_MAGIC_BUSY)
            g_crash_log.count++;
        else
            g_crash_log.count = 1u;
    }
    g_crash_log.type = type;
    g_crash_log.arg0 = arg0;
    g_crash_log.arg1 = arg1;
    g_crash_log.progress = 1u;

    /* 任务名是"payload 的第一个字段": 只有栈溢出那条路会先填好它
       (见 CrashLogStackOverflow), 别的入口一律清零, 保证记录内容是确定的。 */
    if (type != CRASH_TYPE_STACK_OVF)
    {
        for (uint32_t i = 0u; i < CRASH_TASK_NAME_LEN; ++i)
            g_crash_log.task_name[i] = '\0';
    }

    /* 故障状态寄存器(直接读, 不需要外设时钟) */
    g_crash_log.cfsr = SCB->CFSR;
    g_crash_log.hfsr = SCB->HFSR;
    g_crash_log.bfar = SCB->BFAR;
    g_crash_log.mmfar = SCB->MMFAR;
    g_crash_log.dfsr = SCB->DFSR;
    g_crash_log.progress = 2u;

    g_crash_log.psp = __get_PSP();
    g_crash_log.msp = __get_MSP();
    uint32_t snapshot_words = 0u;

    if (frame != NULL)
    {
        CrashLogFrameView_t frame_view;
        g_crash_log.sp = (uint32_t)(uintptr_t)frame;
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
            g_crash_log.r0 = g_crash_log.r1 = g_crash_log.r2 = g_crash_log.r3 = 0u;
            g_crash_log.r12 = g_crash_log.lr = g_crash_log.pc = g_crash_log.xpsr = 0u;
            g_crash_log.frame_status = 2u;
        }
    }
    else
    {
        /* 断言/栈溢出这类"主动调用"没有异常帧: 直接取当前栈指针 */
        g_crash_log.r0 = g_crash_log.r1 = g_crash_log.r2 = g_crash_log.r3 = 0u;
        g_crash_log.r12 = g_crash_log.lr = g_crash_log.xpsr = 0u;
        g_crash_log.frame_status = 0u;
        g_crash_log.sp = (g_crash_log.psp != 0u) ? g_crash_log.psp : g_crash_log.msp;
        snapshot_words = CrashLogReadableStackWords((uintptr_t)g_crash_log.sp);
        /* 没有异常帧就没有"出错 PC"; 用调用者给的返回地址代替 ——
           至少能看出断言/溢出是从哪一行代码报上来的, 否则 PC 只能是 0。 */
        g_crash_log.pc = caller_pc;
    }
    g_crash_log.progress = 3u;

    /* 栈快照只读链接脚本定义 SRAM 范围内可完整访问的字，栈溢出或坏压栈时
       将不可读的尾部清零，避免记录过程再次因为越界读进入二次异常。 */
    {
        const volatile uint32_t *p = (const volatile uint32_t *)(uintptr_t)g_crash_log.sp;
        for (uint32_t i = 0; i < CRASH_STACK_SNAPSHOT_WORDS; ++i)
            g_crash_log.stack_snapshot[i] = (i < snapshot_words) ? p[i] : 0u;
    }
    g_crash_log.progress = 4u;

    /* 最后两件事: 算 payload 的校验和 → 发布 magic=OK。
       于是"读到 OK 且校验和正确" == "整份记录完整、且确实是本固件写的"。 */
    {
        uint32_t crc;

        CRASH_CRC32_WORDS(crc,
                          (const uint32_t *)((const uint8_t *)&g_crash_log +
                                             CRASH_PAYLOAD_OFFSET),
                          CRASH_PAYLOAD_WORDS);
        g_crash_log.crc32 = crc;
    }
    g_crash_log.magic = CRASH_LOG_MAGIC_OK;

    /* !! 复位前必须等这条写"落到 SRAM" !!
       2026-09-26 实测: 少了这一步, 下次启动读到的 magic 还是 BUSY、整份记录等于白写
       (单步调试却能看到值已经写进内存) —— 原因是系统复位会丢掉"还在总线上"的写:
       __DSB() 只保证 CPU 自己的缓冲排空, 不保证已经到达 SRAM 控制器。
       这里用一个有界空转等它落地(约 0.1~0.2ms), 反正马上就要复位, 不影响任何功能。
       (读取侧还有一层兜底: 见 CrashLogValid —— "BUSY + progress=4 + 校验和正确"
        也认作完整记录, 万一这条 magic 还是丢了, 记录依然可用。) */
    for (volatile uint32_t _drain = 0u; _drain < CRASH_RESET_DRAIN_LOOPS; ++_drain)
        ;

    /* 主动复位, 不再死循环等看门狗 */
    NVIC_SystemReset();

    for (;;) /* 理论上到不了这里 */
        ;
}

void CrashLogRecord(uint32_t type, uint32_t *frame, uint32_t arg0, uint32_t arg1,
                    uint32_t caller_pc)
{
    CrashLogBegin();
    CrashLogWriteRecord(type, frame, 0xFFFFFFF9u, arg0, arg1, caller_pc);
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
    /* Stack-overflow input may be in damaged memory; shut down and establish the
       reentry guard before reading the task-name pointer. */
    CrashLogBegin();

    /* 任务名不再"存指针", 而是在记录里内联一份副本: 指针指向的是 TCB 里的
       char 数组(RAM), 下次启动时那块内存可能已经被复用/重排。
       这里拷贝时顺手把非可打印字符换成 '.', 于是无论后面打印还是 LCD 显示,
       都不可能吐出乱码。 */
    for (uint32_t i = 0u; i < CRASH_TASK_NAME_LEN; ++i)
    {
        char c = '\0';

        if ((task_name != NULL) && (i < (CRASH_TASK_NAME_LEN - 1u)))
            c = task_name[i];
        if ((c != '\0') && ((c < 0x20) || (c > 0x7E)))
            c = '.';

        g_crash_log.task_name[i] = c;
        if (c == '\0')
            break;
    }

    CrashLogWriteRecord(CRASH_TYPE_STACK_OVF, NULL, 0u, 0u, 0u,
                        (uint32_t)(uintptr_t)__builtin_return_address(0));
}

/**
 * @brief FreeRTOS 栈溢出钩子(由 configCHECK_FOR_STACK_OVERFLOW=2 触发)
 * @note  传进来的任务名不是 Flash 常量, 而是 TCB 里的 char 数组(RAM),
 *        所以记录时会把名字拷进现场(见 CrashLogStackOverflow)。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    CrashLogStackOverflow((const char *)pcTaskName);
}

/* Naked exception wrappers shut outputs down before entering C. This entry
   checks reentry before recording and does not repeat those register writes. */
void CrashLogHandlerC(uint32_t *stack_frame, uint32_t type, uint32_t exception_return)
{
    /* The naked exception wrapper has already latched outputs off before
       entering C. Do not repeat RCC/GPIO/TIM accesses here; a second fault
       should reach the reset path immediately. */
    CrashLogGuardEnter(&g_crash_handler_active);
    CrashLogWriteRecord(type, stack_frame, exception_return, 0u, 0u, 0u);
}

void CrashLogInit(void)
{
    /* 四重判据全过才算"我们写的完整记录"; 不通过时:
         - magic 停在 BUSY → 记录确实写到一半就中断了(比如写到栈快照那一步又崩了),
           这是很有价值的线索, 只报一行;
         - 其它情况(残留数据 / 旧固件 / 换过布局) → 一个字都不打印,
           宁可没有信息, 也不要打出一份误导人的假现场。 */
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

    if (g_crash_log.type == CRASH_TYPE_ASSERT &&
        CrashLogStringPlausible(g_crash_log.arg0))
        LOGERROR("[crash] assert: %s:%lu", (const char *)g_crash_log.arg0,
                 (unsigned long)g_crash_log.arg1);
    if (g_crash_log.type == CRASH_TYPE_STACK_OVF)
        LOGERROR("[crash] 任务 '%s' 栈溢出", g_crash_log.task_name);

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
    if (out == NULL || !CrashLogValid())
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
