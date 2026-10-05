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

   安全顺序:
     异常汇编入口先调用 BSP_SafetyLatchOutputsOff(), 以直接寄存器访问关闭已知输出，
     不依赖 HAL、tick、RTOS 或日志。随后记录过程关闭中断、验证压栈地址与故障状态，
     只读允许范围内的 SRAM；二次异常通过活动标志走关断并立即复位路径。
     这里的寄存器关断是软件尽力措施，不代表已测量外部引脚电平或主供电边界。

   记录的可信度(2026-09-26, LOG-05)
   ----------------------------------------------------------------------------
   原来只用 4 字节 magic 判断"这份记录是不是我们写的", 于是 .noinit 的地址随
   布局漂移之后, RAM 里的残留数据会被当成"上次崩溃现场"打印出来(实测出现过
   "累计第 2261714040 次、任务名空白"这种误导横幅)。现在改成四重判据:

     magic(=OK) + len(=sizeof) + version(=当前格式版本) + fw_id(=本固件构建指纹)
     + crc32(对 payload 的 CRC32-IEEE)

   任何一个不符 → CrashLogValid() 返回 0 → 启动时**一个字都不打印**(静默忽略)。
   另外 magic 有两个取值: 开始写记录时先写 BUSY, 整份写完才写 OK ——
   于是"看到 BUSY"就是一条真线索: 我们写到一半又崩了(见 CrashLogInit)。
   ========================================================================== */

/* 记录格式版本: 改动 CrashLog_t 的布局时必须 +1 */
#define CRASH_LOG_VERSION 3u

/* 头部 magic 的两个取值(小端拼写): "BUSY" = 正在写, "CROK" = 写完整 */
#define CRASH_LOG_MAGIC_BUSY 0x59535542u
#define CRASH_LOG_MAGIC_OK 0x4B4F5243u

/* 内联保存的任务名长度: 至少要放得下 FreeRTOS 的 configMAX_TASK_NAME_LEN
   (bsp_crash.c 里有 _Static_assert 兜底) */
#define CRASH_TASK_NAME_LEN 16u

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
    uint32_t magic; /* BUSY(开始写) → OK(写完才写它) */
    uint32_t len;     /* = sizeof(CrashLog_t): 换记录布局后自动失效 */
    uint32_t version; /* = CRASH_LOG_VERSION */
    uint32_t fw_id;   /* 本固件的构建指纹(由 __DATE__/__TIME__ 派生): 换固件后自动失效 */
    uint32_t crc32;   /* 对 payload(下面第 1 个字段起到结构体末尾)的 CRC32-IEEE */
    uint32_t progress; /* 写入进度: 1=已写头部 2=已写故障寄存器 3=已写栈帧 4=已写栈快照
                          用来诊断"记录写到一半就死了"(此时 magic 停在 BUSY) */
    uint32_t count; /* 累计崩溃次数(跨复位累加, 不清零) */
    uint32_t type;  /* CrashType_e */
    uint32_t arg0;  /* 附加信息: 断言所在的文件名指针(栈溢出不用它, 见 task_name) */
    uint32_t arg1;  /* 附加信息: 断言行号 */

    /* ---- 以下都是 payload: 全部参与 crc32 ---- */

    /* 栈溢出时的任务名(内联副本)。为什么不再存指针: 任务名在 TCB 里(RAM),
       下次启动时那块内存可能已被复用, 指针指向的内容早就不是任务名了。 */
    char task_name[CRASH_TASK_NAME_LEN];

    /* ---- 故障状态寄存器(直接读寄存器得到) ---- */
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t bfar;
    uint32_t mmfar;
    uint32_t dfsr;

    /* ---- 出错时的异常栈帧(ARM 硬件压栈的 8 个字) ---- */
    uint32_t r0, r1, r2, r3, r12, lr, pc, xpsr;
    uint32_t frame_status; /* 0=无异常帧, 1=有效, 2=地址/压栈状态无效 */

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
   frame:     指向异常栈起始位置; FP 扩展帧布局由 EXC_RETURN 标志解析
   arg0/arg1: 附加信息(断言=文件名指针+行号; 栈溢出=0, 任务名走 task_name 内联副本)
   caller_pc: 没有异常帧时用作"出错位置"(由调用者用 __builtin_return_address(0) 取得);
              有异常帧时忽略(帧里的 PC 更准)
   这些函数记录完现场就主动复位, 不会返回。 */
void CrashLogRecord(uint32_t type, uint32_t *frame, uint32_t arg0, uint32_t arg1,
                    uint32_t caller_pc);

/** @brief FreeRTOS configASSERT 失败时调用 */
void CrashLogAssertFail(const char *file, uint32_t line);
/** @brief FreeRTOS 栈溢出钩子调用 */
void CrashLogStackOverflow(const char *task_name);

/* 异常入口先低层关断，再跳到这里；r0=异常栈指针，r1=异常类型，r2=EXC_RETURN。 */
void CrashLogHandlerC(uint32_t *stack_frame, uint32_t type, uint32_t exception_return);

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
