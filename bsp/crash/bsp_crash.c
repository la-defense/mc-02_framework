/**
 * @file bsp_crash.c
 * @brief 在固定链接保留区保存崩溃现场，启动后校验并交给日志或LCD查询。
 * 异常处理只读固定寄存器和链接范围内的RAM/Flash，不调用外设、RTOS、日志、分配器或Flash。
 * @author MC-02 contributors
 * @license BSD-3-Clause; see LICENSE.
 */
#include "bsp_crash.h"

#include "bsp_crash_crc.h"
#include "bsp_crash_frame.h"
#include "bsp_crash_guard.h"
#include "bsp_observation.h"
#include "bsp_safety.h"
#include "main.h"
#include "bsp_log.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#define CRASH_RESET_DRAIN_LOOPS 20000u
#define GNU_NOTE_TYPE_BUILD_ID 3u
#define GNU_NOTE_TOTAL_BYTES 36u
#define GNU_NOTE_BUILD_ID_OFFSET 16u
#define CRASH_CRC_WORD_COUNT ((sizeof(CrashLog_t) - 2u * sizeof(uint32_t)) / sizeof(uint32_t))

extern const uint8_t __mc02_build_id_note_start__[];
extern const uint8_t __mc02_build_id_note_end__[];
extern const uint8_t __mc02_flash_start__[];
extern const uint8_t __mc02_flash_end__[];
extern const uint8_t __mc02_dtcm_start__[];
extern const uint8_t __mc02_dtcm_end__[];
extern const uint8_t __mc02_main_ram_start__[];
extern const uint8_t __mc02_main_ram_end__[];
extern const uint8_t __mc02_ram_d2_start__[];
extern const uint8_t __mc02_ram_d2_end__[];
extern const uint8_t __mc02_ram_d3_start__[];
extern const uint8_t __mc02_ram_d3_end__[];
extern const BSP_ObservationIdentityTemplate_t bsp_observation_identity_template;

static volatile CrashLog_t g_crash_log
    __attribute__((section(".crash_record"), used, aligned(32)));
static volatile uint32_t g_crash_handler_active;

static const CrashMemoryRegion_t g_crash_memory_regions[CRASH_MEMORY_REGION_CAPACITY] = {
    {(uintptr_t)__mc02_flash_start__, (uintptr_t)__mc02_flash_end__, 0u},
    {(uintptr_t)__mc02_dtcm_start__, (uintptr_t)__mc02_dtcm_end__, 1u},
    {(uintptr_t)__mc02_main_ram_start__, (uintptr_t)__mc02_main_ram_end__, 1u},
    {(uintptr_t)__mc02_ram_d2_start__, (uintptr_t)__mc02_ram_d2_end__, 1u},
    {(uintptr_t)__mc02_ram_d3_start__, (uintptr_t)__mc02_ram_d3_end__, 1u},
};
static const CrashMemoryMap_t g_crash_memory_map = {
    g_crash_memory_regions,
    CRASH_MEMORY_REGION_CAPACITY,
};

_Static_assert(sizeof(CrashTestPermit_t) == 40u, "工具与固件的自测许可必须都是十个32位字");
_Static_assert(offsetof(CrashTestPermit_t, crc32) == 36u,
               "自测许可CRC偏移变化时须同步固定工具");
_Static_assert(sizeof(CrashLog_t) + sizeof(CrashTestPermit_t) <= CRASH_RECORD_CAPACITY,
               "崩溃记录和自测许可须装入链接保留区");
_Static_assert((sizeof(CrashLog_t) % 4u) == 0u, "异常记录须按32位字写入");
_Static_assert(CRASH_CRC_WORD_COUNT == 82u, "v4 CRC范围变化时须同步格式检查");
_Static_assert(CRASH_TASK_NAME_LEN >= configMAX_TASK_NAME_LEN,
               "崩溃记录任务名字段须容纳FreeRTOS任务名");

#if MC02_BENCH_CRASH_DIAGNOSTICS
volatile CrashTestPermit_t bsp_crash_test_permit
    __attribute__((section(".crash_test_arm"), used, aligned(4)));
#endif

static uint32_t CrashLogReadLE32(const volatile uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8u) |
           ((uint32_t)bytes[2] << 16u) | ((uint32_t)bytes[3] << 24u);
}

static uint8_t CrashLogBuildIdRead(uint32_t build_id[BSP_OBSERVATION_BUILD_ID_WORDS])
{
    const volatile uint8_t *note = __mc02_build_id_note_start__;
    const uintptr_t note_start = (uintptr_t)__mc02_build_id_note_start__;
    const uintptr_t note_end = (uintptr_t)__mc02_build_id_note_end__;

    if (note_end < note_start || note_end - note_start != GNU_NOTE_TOTAL_BYTES ||
        !CrashLogRangeReadable(&g_crash_memory_map, note_start, GNU_NOTE_TOTAL_BYTES, 4u) ||
        CrashLogReadLE32(note) != 4u || CrashLogReadLE32(note + 4u) != 20u ||
        CrashLogReadLE32(note + 8u) != GNU_NOTE_TYPE_BUILD_ID ||
        note[12] != 'G' || note[13] != 'N' || note[14] != 'U' || note[15] != 0u)
        return 0u;

    // Build ID按原始20字节复制，小端字序与主机工具读取的ELF note一致。
    for (uint32_t word = 0u; word < BSP_OBSERVATION_BUILD_ID_WORDS; ++word)
    {
        uint32_t value = 0u;
        for (uint32_t byte = 0u; byte < 4u; ++byte)
            value |= (uint32_t)note[GNU_NOTE_BUILD_ID_OFFSET + word * 4u + byte] << (byte * 8u);
        build_id[word] = value;
    }
    return 1u;
}

static uint8_t CrashLogTemplateCopy(volatile CrashLog_t *record)
{
    const BSP_ObservationIdentityTemplate_t *identity = &bsp_observation_identity_template;
    if (identity->magic != BSP_OBSERVATION_IDENTITY_MAGIC ||
        identity->version != BSP_OBSERVATION_IDENTITY_VERSION ||
        identity->size_bytes != sizeof(BSP_ObservationIdentity_t) ||
        identity->layout != BSP_OBSERVATION_IDENTITY_LAYOUT)
        return 0u;

    record->profile = identity->profile;
    record->board_role = identity->role;
    record->module_mask = identity->module_mask;
    record->config_version = identity->layout;
    return 1u;
}

static uint32_t CrashLogCopyText(volatile char *destination, uint32_t capacity,
                                 uintptr_t source)
{
    uint32_t length = 0u;
    if (destination == NULL || capacity == 0u || source == 0u)
        return 0u;

    // 每读一个字节都重新核对整段范围，字符串不会跨过链接区或RAM区的末尾。
    while (length + 1u < capacity && source <= UINTPTR_MAX - length &&
           CrashLogRangeReadable(&g_crash_memory_map, source + length, 1u, 1u))
    {
        char value = (char)*(const volatile uint8_t *)(source + length);
        if (value == '\0')
            break;
        if (value < 0x20 || value > 0x7E)
            value = '.';
        destination[length++] = value;
    }
    destination[length] = '\0';
    return length;
}

static uint8_t CrashLogCopyRecord(CrashLog_t *output)
{
    if (output == NULL)
        return 0u;
    const volatile uint32_t *source = (const volatile uint32_t *)&g_crash_log;
    uint32_t *destination = (uint32_t *)output;
    for (uint32_t index = 0u; index < (sizeof(CrashLog_t) / sizeof(uint32_t)); ++index)
        destination[index] = source[index];
    return 1u;
}

static CrashRecordStatus_t CrashLogValidateCurrent(CrashRecordInfo_t *info)
{
    uint8_t bytes[CRASH_RECORD_CAPACITY] = {0u};
    const volatile uint8_t *source = (const volatile uint8_t *)&g_crash_log;
    for (uint32_t index = 0u; index < sizeof(CrashLog_t); ++index)
        bytes[index] = source[index];
    return CrashLogRecordInspect(bytes, sizeof(CrashLog_t), info);
}

static void CrashLogBegin(void)
{
    CrashLogGuardEnter(&g_crash_handler_active);
    BSP_SafetyLatchOutputsOff();
}

static uint32_t CrashLogPreviousCountRead(void)
{
    uint32_t previous_crc = 0u;
    const uint32_t magic = g_crash_log.magic;

    // 只按当前固定布局检查上一条记录，避免异常栈上临时复制整份记录。
    if ((magic != CRASH_LOG_MAGIC_OK && magic != CRASH_LOG_MAGIC_BUSY) ||
        g_crash_log.len != sizeof(CrashLog_t) ||
        g_crash_log.version != CRASH_LOG_VERSION || g_crash_log.progress != 4u ||
        !CrashCRC32Words((const volatile uint32_t *)&g_crash_log.len,
                         CRASH_CRC_WORD_COUNT, &previous_crc) ||
        previous_crc != g_crash_log.crc32)
        return 0u;

    return g_crash_log.count;
}

static void CrashLogWriteRecord(uint32_t type, uint32_t *frame, uint32_t exception_return,
                                uint32_t arg0, uint32_t arg1, uint32_t caller_pc,
                                uintptr_t task_name, uintptr_t source_name)
{
    uint32_t previous_count = CrashLogPreviousCountRead();
    uint32_t current_count = 1u;

    // 只有长度、版本、阶段和CRC都正确的旧记录参与累计。
    if (previous_count != 0u)
        current_count = previous_count + 1u;

    g_crash_log.magic = CRASH_LOG_MAGIC_BUSY;
    volatile uint32_t *record_words = (volatile uint32_t *)&g_crash_log;
    for (uint32_t index = 1u; index < (sizeof(CrashLog_t) / sizeof(uint32_t)); ++index)
        record_words[index] = 0u;

    g_crash_log.len = sizeof(CrashLog_t);
    g_crash_log.version = CRASH_LOG_VERSION;
    g_crash_log.progress = 1u;
    g_crash_log.count = current_count;
    g_crash_log.type = type;
    g_crash_log.arg0 = arg0;
    g_crash_log.arg1 = arg1;
    uint32_t build_id[BSP_OBSERVATION_BUILD_ID_WORDS] = {0u};
    if (CrashLogBuildIdRead(build_id))
    {
        for (uint32_t index = 0u; index < BSP_OBSERVATION_BUILD_ID_WORDS; ++index)
            g_crash_log.build_id[index] = build_id[index];
        g_crash_log.identity_flags |= 1u;
    }
    if (CrashLogTemplateCopy(&g_crash_log))
        g_crash_log.identity_flags |= 2u;
    if (task_name != 0u)
        g_crash_log.task_name_len = CrashLogCopyText(g_crash_log.task_name,
                                                      CRASH_TASK_NAME_LEN, task_name);
    if (source_name != 0u)
        g_crash_log.source_name_len = CrashLogCopyText(g_crash_log.source_name,
                                                        CRASH_SOURCE_NAME_LEN, source_name);
    g_crash_log.progress = 2u;

    g_crash_log.cfsr = SCB->CFSR;
    g_crash_log.hfsr = SCB->HFSR;
    g_crash_log.bfar = SCB->BFAR;
    g_crash_log.mmfar = SCB->MMFAR;
    g_crash_log.dfsr = SCB->DFSR;
    g_crash_log.psp = __get_PSP();
    g_crash_log.msp = __get_MSP();

    uint32_t snapshot_words = 0u;
    if (frame != NULL)
    {
        CrashLogFrameView_t frame_view;
        g_crash_log.sp = (uint32_t)(uintptr_t)frame;
        g_crash_log.exception_return = exception_return;
        if (CrashLogFrameResolve(&g_crash_memory_map, (uintptr_t)frame, exception_return,
                                 g_crash_log.cfsr, &frame_view))
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
            g_crash_log.frame_status = 2u;
    }
    else
    {
        // 断言和任务钩子没有硬件保存帧；保存调用点和当前栈数据供离线查看。
        g_crash_log.frame_status = 0u;
        g_crash_log.sp = (g_crash_log.psp != 0u) ? g_crash_log.psp : g_crash_log.msp;
        g_crash_log.pc = caller_pc;
        snapshot_words = CrashLogReadableStackWords(&g_crash_memory_map, g_crash_log.sp);
    }
    g_crash_log.stack_words = snapshot_words;
    g_crash_log.progress = 3u;

    const volatile uint32_t *stack = (const volatile uint32_t *)(uintptr_t)g_crash_log.sp;
    for (uint32_t index = 0u; index < CRASH_STACK_SNAPSHOT_WORDS; ++index)
        g_crash_log.stack_snapshot[index] = (index < snapshot_words) ? stack[index] : 0u;
    g_crash_log.progress = 4u;

    // CRC包含长度、版本、身份、寄存器状态和实际栈长度，排除magic及末尾CRC字段。
    uint32_t crc = 0u;
    if (CrashCRC32Words((const volatile uint32_t *)&g_crash_log.len,
                        CRASH_CRC_WORD_COUNT, &crc))
    {
        g_crash_log.crc32 = crc;
        __DSB();
        g_crash_log.magic = CRASH_LOG_MAGIC_OK;
    }

    // 等待最后几笔RAM写入完成后请求复位；异常路径不写Flash。
    __DSB();
    for (volatile uint32_t drain = 0u; drain < CRASH_RESET_DRAIN_LOOPS; ++drain)
        __NOP();
    NVIC_SystemReset();
    for (;;)
        __NOP();
}

void CrashLogRecord(uint32_t type, uint32_t *frame, uint32_t arg0, uint32_t arg1,
                    uint32_t caller_pc)
{
    CrashLogBegin();
    CrashLogWriteRecord(type, frame, 0xFFFFFFF9u, arg0, arg1, caller_pc, 0u, 0u);
}

__attribute__((noinline)) void CrashLogAssertFail(const char *file, uint32_t line)
{
    CrashLogBegin();
    CrashLogWriteRecord(CRASH_TYPE_ASSERT, NULL, 0xFFFFFFF9u,
                        (uint32_t)(uintptr_t)file, line,
                        (uint32_t)(uintptr_t)__builtin_return_address(0), 0u,
                        (uintptr_t)file);
}

__attribute__((noinline)) void CrashLogStackOverflow(const char *task_name)
{
    CrashLogBegin();
    CrashLogWriteRecord(CRASH_TYPE_STACK_OVF, NULL, 0xFFFFFFF9u, 0u, 0u,
                        (uint32_t)(uintptr_t)__builtin_return_address(0),
                        (uintptr_t)task_name, 0u);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    CrashLogStackOverflow(task_name);
}

void CrashLogHandlerC(uint32_t *stack_frame, uint32_t type, uint32_t exception_return)
{
    CrashLogGuardEnter(&g_crash_handler_active);
    CrashLogWriteRecord(type, stack_frame, exception_return, 0u, 0u, 0u, 0u, 0u);
}

CrashRecordStatus_t CrashLogGetStatus(CrashRecordInfo_t *info)
{
    if (info == NULL)
        return CRASH_RECORD_BAD_ARGUMENT;
    return CrashLogValidateCurrent(info);
}

uint8_t CrashLogGetLast(CrashLog_t *output)
{
    CrashLog_t candidate;
    CrashRecordInfo_t info;
    if (output == NULL || !CrashLogCopyRecord(&candidate))
        return 0u;
    CrashRecordStatus_t status = CrashLogRecordInspect((const uint8_t *)&candidate,
                                                       sizeof(candidate), &info);
    if (status != CRASH_RECORD_VALID && status != CRASH_RECORD_VALID_BUSY_FINAL)
        return 0u;
    *output = candidate;
    return 1u;
}

void CrashLogClear(void)
{
    g_crash_log.magic = 0u;
    __DSB();
}

void CrashLogInit(void)
{
    CrashRecordInfo_t info;
    CrashRecordStatus_t status = CrashLogGetStatus(&info);
    if (status != CRASH_RECORD_VALID && status != CRASH_RECORD_VALID_BUSY_FINAL)
    {
        if (status != CRASH_RECORD_EMPTY)
            BSP_LOG_EVENT_3(EV_BSP_CRASH_BSP_CRASH_C_01,
                            BSP_LOG_FIELD_U32(status),
                            BSP_LOG_FIELD_U32(info.actual_length),
                            BSP_LOG_FIELD_U32(info.version));
        return;
    }

    CrashLog_t record;
    if (!CrashLogGetLast(&record))
        return;
    BSP_LOG_EVENT_1(EV_BSP_CRASH_BSP_CRASH_C_02, BSP_LOG_FIELD_U32(record.count));
    BSP_LOG_EVENT_5(EV_BSP_CRASH_BSP_CRASH_C_03,
                    BSP_LOG_FIELD_CSTR_LIMIT(CrashLogTypeName(record.type), 37u),
                    BSP_LOG_FIELD_CSTR_LIMIT(record.frame_status == 1u ? "valid" :
                                              record.frame_status == 2u ? "invalid" : "none", 37u),
                    BSP_LOG_FIELD_HEX32(record.pc), BSP_LOG_FIELD_HEX32(record.lr),
                    BSP_LOG_FIELD_HEX32(record.sp));
    BSP_LOG_EVENT_5(EV_BSP_CRASH_BSP_CRASH_C_04,
                    BSP_LOG_FIELD_HEX32(record.cfsr), BSP_LOG_FIELD_HEX32(record.hfsr),
                    BSP_LOG_FIELD_HEX32(record.bfar), BSP_LOG_FIELD_HEX32(record.mmfar),
                    BSP_LOG_FIELD_HEX32(record.dfsr));
    BSP_LOG_EVENT_6(EV_BSP_CRASH_BSP_CRASH_C_05,
                    BSP_LOG_FIELD_HEX32(record.r0), BSP_LOG_FIELD_HEX32(record.r1),
                    BSP_LOG_FIELD_HEX32(record.r2), BSP_LOG_FIELD_HEX32(record.r3),
                    BSP_LOG_FIELD_HEX32(record.r12), BSP_LOG_FIELD_HEX32(record.xpsr));
    BSP_LOG_EVENT_2(EV_BSP_CRASH_BSP_CRASH_C_06,
                    BSP_LOG_FIELD_HEX32(record.msp), BSP_LOG_FIELD_HEX32(record.psp));
    if (record.type == CRASH_TYPE_ASSERT && record.source_name_len != 0u)
        BSP_LOG_EVENT_2(EV_BSP_CRASH_BSP_CRASH_C_07,
                        BSP_LOG_FIELD_CSTR_LIMIT(record.source_name, CRASH_SOURCE_NAME_LEN),
                        BSP_LOG_FIELD_U32(record.arg1));
    if (record.type == CRASH_TYPE_STACK_OVF && record.task_name_len != 0u)
        BSP_LOG_EVENT_1(EV_BSP_CRASH_BSP_CRASH_C_08,
                        BSP_LOG_FIELD_CSTR_LIMIT(record.task_name, CRASH_TASK_NAME_LEN));

    // 只输出确实读取到的栈字，未读取位置不会作为零值现场显示。
    for (uint32_t index = 0u; index < record.stack_words; index += 4u)
    {
        uint32_t remaining = record.stack_words - index;
        uint32_t count = (remaining > 4u) ? 4u : remaining;
        if (count == 4u)
            BSP_LOG_EVENT_5(EV_BSP_CRASH_BSP_CRASH_C_12,
                            BSP_LOG_FIELD_U32(index * 4u),
                            BSP_LOG_FIELD_HEX32(record.stack_snapshot[index]),
                            BSP_LOG_FIELD_HEX32(record.stack_snapshot[index + 1u]),
                            BSP_LOG_FIELD_HEX32(record.stack_snapshot[index + 2u]),
                            BSP_LOG_FIELD_HEX32(record.stack_snapshot[index + 3u]));
    }
    BSP_LOG_EVENT_0(EV_BSP_CRASH_BSP_CRASH_C_09);
}

const char *CrashLogTypeName(uint32_t type)
{
    switch (type)
    {
    case CRASH_TYPE_HARDFAULT: return "HARDFAULT";
    case CRASH_TYPE_MEMMANAGE: return "MEMMANAGE";
    case CRASH_TYPE_BUSFAULT: return "BUSFAULT";
    case CRASH_TYPE_USAGEFAULT: return "USAGEFAULT";
    case CRASH_TYPE_NMI: return "NMI";
    case CRASH_TYPE_ASSERT: return "ASSERT";
    case CRASH_TYPE_STACK_OVF: return "STACKOVF";
    default: return "NONE";
    }
}

const char *CrashLogTypeShort(uint32_t type)
{
    switch (type)
    {
    case CRASH_TYPE_HARDFAULT: return "HF";
    case CRASH_TYPE_MEMMANAGE: return "MM";
    case CRASH_TYPE_BUSFAULT: return "BUS";
    case CRASH_TYPE_USAGEFAULT: return "UDF";
    case CRASH_TYPE_NMI: return "NMI";
    case CRASH_TYPE_ASSERT: return "ASRT";
    case CRASH_TYPE_STACK_OVF: return "STK";
    default: return "---";
    }
}

#if MC02_BENCH_CRASH_DIAGNOSTICS
void CrashLogSelfTest(void)
{
    if (bsp_crash_test_permit.magic != CRASH_TEST_PERMIT_MAGIC)
        return;

    CrashTestPermit_t permit;
    volatile uint32_t *source = (volatile uint32_t *)&bsp_crash_test_permit;
    uint32_t *destination = (uint32_t *)&permit;
    for (uint32_t index = 0u; index < sizeof(permit) / sizeof(uint32_t); ++index)
        destination[index] = source[index];

    // 先消费许可；复位后不再执行同一项异常自测。
    bsp_crash_test_permit.magic = CRASH_TEST_PERMIT_CONSUMED;
    __DSB();
    uint32_t build_id[BSP_OBSERVATION_BUILD_ID_WORDS] = {0u};
    uint32_t permit_crc = 0u;
    uint8_t valid = (permit.version == 1u && permit.test_case >= CRASH_TEST_CASE_UNDEFINED_INSTRUCTION &&
                     permit.test_case <= CRASH_TEST_CASE_STACK_HOOK &&
                     CrashLogBuildIdRead(build_id) &&
                     CrashCRC32Words((const volatile uint32_t *)&permit.version,
                                     (offsetof(CrashTestPermit_t, crc32) - sizeof(uint32_t)) / sizeof(uint32_t),
                                     &permit_crc) && permit_crc == permit.crc32);
    for (uint32_t index = 0u; valid && index < BSP_OBSERVATION_BUILD_ID_WORDS; ++index)
        valid = (permit.build_id[index] == build_id[index]);
    if (!valid)
        return;

    switch (permit.test_case)
    {
    case CRASH_TEST_CASE_UNDEFINED_INSTRUCTION:
        __asm volatile("udf #0" ::: "memory");
        break;
    case CRASH_TEST_CASE_ASSERT:
        CrashLogAssertFail("MC02-0068", 68u);
        break;
    case CRASH_TEST_CASE_STACK_HOOK:
        CrashLogStackOverflow("mc02-test");
        break;
    default:
        break;
    }
}
#endif
