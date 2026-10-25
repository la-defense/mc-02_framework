/**
 * @file bsp_param.c
 * @brief 管理 RAM 参数，完成内部 Flash A/B 记录的读取、保存和清除。
 *
 * ParamInit 选取可用记录；ParamGet/Set 访问 RAM；Commit/Reset 经回调允许后擦写。
 * 擦写前后保存和恢复 IWDG 请求时限，具体任务检查和加热处理由 RobotInit 注册的回调完成。
 * 接口用法与失败后的数据说明见 bsp_param.h。
 */
#include "bsp_param.h"
#include "bsp_flash.h"
#include "bsp_flash_internal.h"
#include "crc.h"
#include "bsp_watchdog.h"
#include "bsp_log.h"
#include <string.h>

#define PARAM_FLASHWORD 32u             /* 每次写入32字节，即256位 */

/* 记录头占32字节；CRC 只检查后面的参数数据，不包含本头部。 */
typedef struct
{
    uint32_t magic;         /* 固定标记，识别本模块写入的记录 */
    uint16_t version;       /* 记录格式版本 */
    uint16_t payload_len;   /* 参数条目共占多少字节，包括各条目的补齐字节 */
    uint32_t seq;           /* 本次记录序号，用于启动时选择 A/B 区 */
    uint32_t crc32;         /* 参数数据的 CRC32 检查值 */
    uint32_t reserved[4];   /* 补齐头部长度，写入时置0 */
} ParamHeader_t;

_Static_assert(sizeof(ParamHeader_t) == PARAM_FLASHWORD, "记录头必须正好 32 字节");

/* RAM 保存各参数条目的原始字节，提交时原样写进 Flash。 */
static uint8_t s_payload[PARAM_MAX_PAYLOAD];
static uint16_t s_payload_len = 0;
static uint8_t s_dirty = 0;
static ParamLongOperationGuard_t s_long_operation_guard;

void ParamSetLongOperationGuard(const ParamLongOperationGuard_t *guard)
{
    if (guard == NULL)
    {
        // 清除回调设置后，后续 Commit/Reset 都会拒绝擦写。
        s_long_operation_guard.begin = NULL;
        s_long_operation_guard.end = NULL;
        return;
    }

    s_long_operation_guard = *guard;
}

/* 先让回调完成准备；允许后保存原时限，再将 IWDG 请求时限改为8000ms并喂一次。 */
static uint8_t ParamFlashOperationBegin(uint32_t *previous_timeout_ms,
                                        BSPFlashAuthorization_t *authorization)
{
    if (previous_timeout_ms == NULL || authorization == NULL ||
        s_long_operation_guard.begin == NULL || s_long_operation_guard.end == NULL)
        return 0u;

    // 加热与任务监控回调只能从普通任务调用，先拒绝中断上下文。
    if (!BSPFlashMaintenanceIsTaskContext())
        return 0u;

    const uint32_t operation_started_at_ms = BSPFlashMaintenanceGetTick();
    // 回调不完整或拒绝开始时，不修改看门狗，也不进入 Flash 擦写。
    if (!s_long_operation_guard.begin())
        return 0u;

    *previous_timeout_ms = BSP_WatchdogGetTimeout();
    BSP_WatchdogSetTimeout(BSP_FLASH_MAINTENANCE_TIMEOUT_MS);
    BSP_WatchdogFeed();
    // 只有 MC02-0036 的加热与任务监控准入完成后，参数层才取得写入令牌。
    if (BSPFlashAuthorizationOpen(authorization, operation_started_at_ms) != BSP_FLASH_OK)
    {
        BSP_WatchdogSetTimeout(*previous_timeout_ms);
        (void)s_long_operation_guard.end(0u);
        return 0u;
    }
    return 1u;
}

/* 只在 Begin 成功后调用：恢复原时限，再把 Flash 操作结果交给收尾回调。 */
static uint8_t ParamFlashOperationEnd(uint32_t previous_timeout_ms,
                                      uint8_t flash_success,
                                      BSPFlashAuthorization_t *authorization)
{
    const BSPFlashStatus_e authorization_status = BSPFlashAuthorizationClose(authorization);
    const uint8_t operation_ok = (uint8_t)(flash_success &&
                                            authorization_status == BSP_FLASH_OK);
    BSP_WatchdogSetTimeout(previous_timeout_ms);
    return s_long_operation_guard.end(operation_ok);
}

volatile uint32_t param_commit_count = 0;
volatile uint32_t param_commit_fail = 0;
volatile uint8_t param_active_region = 0;
volatile uint32_t param_seq = 0;
volatile uint8_t param_loaded = 0;

/* 每条参数先保存编号和数据字节数，后面紧跟数据与补齐字节。 */
typedef struct
{
    uint16_t key;
    uint16_t len;
} ParamEntry_t;

/* 将字节数向上补到4的整数倍，供条目排列和 Flash 写入使用。 */
static uint16_t ParamRoundUp4(uint16_t v)
{
    return (uint16_t)((v + 3u) & (uint16_t)~3u);
}

/* 在 RAM 中查找 key，返回数据起点距数组开头的字节数；找不到返回0，out_len 不改写。 */
static uint16_t ParamFindEntry(uint16_t key, uint16_t *out_len)
{
    uint16_t off = 0;
    // 每次读一个条目头；找到编号就返回，否则跳过数据和补齐字节继续找。
    while (off + sizeof(ParamEntry_t) <= s_payload_len)
    {
        ParamEntry_t entry;
        memcpy(&entry, &s_payload[off], sizeof(entry));
        uint16_t data_off = (uint16_t)(off + sizeof(entry));
        // 条目声称的数据超出了已用区域，停止读取，避免访问后面的无关内容。
        if (data_off + entry.len > s_payload_len)
            break;
        if (entry.key == key)
        {
            if (out_len != NULL)
                *out_len = entry.len;
            return data_off;
        }
        off = (uint16_t)(data_off + ParamRoundUp4(entry.len));
    }
    return 0;
}

uint8_t ParamGet(uint16_t key, void *buf, uint16_t len)
{
    if (buf == NULL)
        return 0;
    uint16_t entry_len = 0;
    uint16_t data_off = ParamFindEntry(key, &entry_len);
    // 只有编号和长度都匹配才复制，失败时调用方的接收空间保持原样。
    if (data_off == 0 || entry_len != len)
        return 0;
    memcpy(buf, &s_payload[data_off], len);
    return 1;
}

uint8_t ParamSet(uint16_t key, const void *buf, uint16_t len)
{
    // 单条数据还要留出4字节存编号和长度；所有条目合计也不能超过512字节。
    if (buf == NULL || len == 0u || len > (PARAM_MAX_PAYLOAD - sizeof(ParamEntry_t)))
        return 0;

    uint16_t entry_len = 0;
    uint16_t data_off = ParamFindEntry(key, &entry_len);

    if (data_off != 0 && entry_len == len)
    {
        // 长度相同，直接覆盖数据，不移动其他条目。
        memcpy(&s_payload[data_off], buf, len);
        s_dirty = 1;
        return 1;
    }

    // 改长度时先移除旧条目，把后面的条目向前移，再尝试追加新条目。
    if (data_off != 0)
    {
        uint16_t entry_total = (uint16_t)(sizeof(ParamEntry_t) + ParamRoundUp4(entry_len));
        uint16_t tail = (uint16_t)(data_off - sizeof(ParamEntry_t) + entry_total);
        memmove(&s_payload[data_off - sizeof(ParamEntry_t)], &s_payload[tail], (size_t)(s_payload_len - tail));
        s_payload_len = (uint16_t)(s_payload_len - entry_total);
    }

    uint16_t need = (uint16_t)(sizeof(ParamEntry_t) + ParamRoundUp4(len));
    // 此处才检查剩余空间；若刚才移除了旧条目，失败也不会将它放回 RAM。
    if ((uint32_t)s_payload_len + need > PARAM_MAX_PAYLOAD)
    {
        BSP_LOG_EVENT_1(EV_BSP_PARAM_BSP_PARAM_C_01, BSP_LOG_FIELD_HEX32((unsigned)key));
        return 0;
    }

    ParamEntry_t entry = {.key = key, .len = len};
    memcpy(&s_payload[s_payload_len], &entry, sizeof(entry));
    memcpy(&s_payload[s_payload_len + sizeof(entry)], buf, len);
    uint16_t pad = (uint16_t)(ParamRoundUp4(len) - len);
    // 末尾不足4字节的部分补0，让下一个条目从4字节的整数倍位置开始。
    if (pad > 0u)
        memset(&s_payload[s_payload_len + sizeof(entry) + len], 0, pad);
    s_payload_len = (uint16_t)(s_payload_len + need);
    s_dirty = 1;
    return 1;
}

/* 写入指定的 A/B 区并回读；调用前应已完成擦写准备，失败也要由调用方收尾。 */
static uint8_t ParamWriteSector(BSPFlashPartition_e partition,
                                const BSPFlashAuthorization_t *authorization)
{
    // 写入源地址和总字节数都需按32字节安排，多出的字节预先补0。
    static __attribute__((aligned(32))) uint8_t buf[PARAM_FLASHWORD + PARAM_MAX_PAYLOAD + PARAM_FLASHWORD];
    ParamHeader_t header;
    memset(buf, 0, sizeof(buf));

    header.magic = PARAM_MAGIC;
    header.version = PARAM_VERSION;
    header.payload_len = s_payload_len;
    header.seq = (uint32_t)param_seq + 1u;
    // 计算完成后才能擦写，失败保留当前活动参数区。
    if (CRCCompute(CRC32_IEEE, s_payload, s_payload_len, &header.crc32) != CRC_OK)
        return 0u;
    memset(header.reserved, 0, sizeof(header.reserved));

    memcpy(buf, &header, sizeof(header));
    memcpy(&buf[PARAM_FLASHWORD], s_payload, s_payload_len);

    uint32_t total = (uint32_t)PARAM_FLASHWORD + ParamRoundUp4(s_payload_len);
    total = (total + PARAM_FLASHWORD - 1u) & ~(PARAM_FLASHWORD - 1u);

    // 只擦写非活动分区；低层再次校验分区、粒度和同一外层维护期限。
    BSPFlashStatus_e flash_status = BSPFlashErase(partition, 0u, 0x20000u,
                                                   authorization);
    if (flash_status != BSP_FLASH_OK)
    {
        BSP_LOG_EVENT_2(EV_BSP_PARAM_BSP_PARAM_C_02,
                        BSP_LOG_FIELD_HEX32((unsigned long)partition),
                        BSP_LOG_FIELD_U32((unsigned long)flash_status));
        return 0;
    }

    flash_status = BSPFlashProgram(partition, 0u, buf, total, authorization);
    if (flash_status != BSP_FLASH_OK)
    {
        BSP_LOG_EVENT_2(EV_BSP_PARAM_BSP_PARAM_C_03,
                        BSP_LOG_FIELD_U32((unsigned long)partition),
                        BSP_LOG_FIELD_U32((unsigned long)flash_status));
        return 0u;
    }

    // 用公共读取入口逐个 Flash word 回读，发现差异时不切换活动分区。
    uint8_t verify[PARAM_FLASHWORD];
    for (uint32_t off = 0u; off < total; off += PARAM_FLASHWORD)
    {
        flash_status = BSPFlashRead(partition, off, verify, sizeof(verify));
        if (flash_status != BSP_FLASH_OK ||
            memcmp(verify, &buf[off], sizeof(verify)) != 0)
        {
            BSP_LOG_EVENT_2(EV_BSP_PARAM_BSP_PARAM_C_04,
                            BSP_LOG_FIELD_U32((unsigned long)partition),
                            BSP_LOG_FIELD_U32((unsigned long)off));
            return 0u;
        }
    }
    return 1;
}

/* 检查指定 A/B 区；成功后才填写非空的 out/out_hdr，失败时两者保持原样。 */
static uint8_t ParamReadSector(BSPFlashPartition_e partition, uint8_t *out,
                               ParamHeader_t *out_hdr)
{
    ParamHeader_t hdr;
    if (BSPFlashRead(partition, 0u, &hdr, sizeof(hdr)) != BSP_FLASH_OK)
        return 0u;

    // 先检查记录格式和长度，避免按损坏的长度读取参数数据。
    if (hdr.magic != PARAM_MAGIC || hdr.version != PARAM_VERSION ||
        hdr.payload_len > PARAM_MAX_PAYLOAD)
        return 0;

    uint8_t payload[PARAM_MAX_PAYLOAD];
    if (BSPFlashRead(partition, PARAM_FLASHWORD, payload, hdr.payload_len) != BSP_FLASH_OK)
        return 0u;
    // 参数数据的 CRC 不符时整条记录都不使用；具体数值范围由使用参数的模块检查。
    uint32_t crc;
    if (CRCCompute(CRC32_IEEE, payload, hdr.payload_len, &crc) != CRC_OK || crc != hdr.crc32)
        return 0;

    if (out != NULL && hdr.payload_len > 0u)
        memcpy(out, payload, hdr.payload_len);
    if (out_hdr != NULL)
        *out_hdr = hdr;
    return 1;
}

uint8_t ParamInit(void)
{
    static uint8_t s_inited = 0;
    ParamHeader_t hdr_a, hdr_b;

    // 只在首次调用时读 Flash，避免覆盖后续 ParamSet 暂存的修改。
    if (s_inited)
        return param_loaded;
    s_inited = 1;

    uint8_t valid_a = ParamReadSector(BSP_FLASH_PARTITION_PARAMETER_A, NULL, &hdr_a);
    uint8_t valid_b = ParamReadSector(BSP_FLASH_PARTITION_PARAMETER_B, NULL, &hdr_b);

    BSPFlashPartition_e partition;
    // A 可用且序号不小于 B 时选 A；否则尝试 B；两区都不可用就交给上层使用默认值。
    if (valid_a && (!valid_b || hdr_a.seq >= hdr_b.seq))
    {
        partition = BSP_FLASH_PARTITION_PARAMETER_A;
        param_active_region = 0;
    }
    else if (valid_b)
    {
        partition = BSP_FLASH_PARTITION_PARAMETER_B;
        param_active_region = 1;
    }
    else
    {
        s_payload_len = 0;
        param_loaded = 0;
        param_seq = 0;
        BSP_LOG_EVENT_0(EV_BSP_PARAM_BSP_PARAM_C_05);
        return 0;
    }

    ParamHeader_t hdr;
    if (!ParamReadSector(partition, s_payload, &hdr))
    {
        s_payload_len = 0u;
        param_loaded = 0u;
        return 0u;
    }
    s_payload_len = hdr.payload_len;
    param_seq = hdr.seq;
    param_loaded = 1;
    s_dirty = 0;
    BSP_LOG_EVENT_3(EV_BSP_PARAM_BSP_PARAM_C_06, BSP_LOG_FIELD_I32((param_active_region == 0) ? 'A' : 'B'), BSP_LOG_FIELD_U32((unsigned long)param_seq), BSP_LOG_FIELD_U32((unsigned)s_payload_len));
    return 1;
}

uint8_t ParamCommit(void)
{
    uint32_t operation_previous_timeout_ms = 0u;
    BSPFlashAuthorization_t authorization = {0};
    // 开始前必须得到回调允许；拒绝时不擦写，也不用调用结束回调。
    if (!ParamFlashOperationBegin(&operation_previous_timeout_ms, &authorization))
    {
        param_commit_fail++;
        BSP_LOG_EVENT_0(EV_BSP_PARAM_BSP_PARAM_C_07);
        return 0u;
    }

    // 总是写另一个区，写入或回读失败时保留原活动区和序号。
    BSPFlashPartition_e target = (param_active_region == 0) ?
                                 BSP_FLASH_PARTITION_PARAMETER_B :
                                 BSP_FLASH_PARTITION_PARAMETER_A;
    uint8_t target_region = (param_active_region == 0) ? 1u : 0u;

    uint8_t ok = ParamWriteSector(target, &authorization);
    // 写入失败也必须恢复 IWDG 原时限，并把失败结果交给回调处理。
    uint8_t operation_ok = ParamFlashOperationEnd(operation_previous_timeout_ms, ok,
                                                   &authorization);
    if (ok)
    {
        // 新记录已经写入并回读成功，即使收尾检查失败，这里仍切换记录并计入成功次数。
        param_active_region = target_region;
        param_seq++;
        s_dirty = 0;
        param_loaded = 1;
        param_commit_count++;
        BSP_LOG_EVENT_2(EV_BSP_PARAM_BSP_PARAM_C_08, BSP_LOG_FIELD_I32((target_region == 0) ? 'A' : 'B'), BSP_LOG_FIELD_U32((unsigned long)param_seq));
    }
    else
    {
        param_commit_fail++;
        BSP_LOG_EVENT_1(EV_BSP_PARAM_BSP_PARAM_C_09, BSP_LOG_FIELD_I32((param_active_region == 0) ? 'A' : 'B'));
    }
    return (uint8_t)(ok && operation_ok);
}

uint8_t ParamReset(void)
{
    uint32_t operation_previous_timeout_ms = 0u;
    BSPFlashAuthorization_t authorization = {0};
    // 与提交使用同一组回调；尚未完成准备时保留 Flash 和 RAM 内容。
    if (!ParamFlashOperationBegin(&operation_previous_timeout_ms, &authorization))
    {
        BSP_LOG_EVENT_0(EV_BSP_PARAM_BSP_PARAM_C_10);
        return 0u;
    }

    // 两个区共用一次授权和期限；第二次擦除失败时也不清 RAM 缓存。
    BSPFlashStatus_e flash_status = BSPFlashErase(BSP_FLASH_PARTITION_PARAMETER_A, 0u,
                                                  0x20000u, &authorization);
    if (flash_status == BSP_FLASH_OK)
        flash_status = BSPFlashErase(BSP_FLASH_PARTITION_PARAMETER_B, 0u,
                                     0x20000u, &authorization);

    uint8_t operation_ok = ParamFlashOperationEnd(operation_previous_timeout_ms,
                                                   (uint8_t)(flash_status == BSP_FLASH_OK),
                                                   &authorization);
    if (flash_status != BSP_FLASH_OK || !operation_ok)
    {
        // Flash 可能已被擦除，但本次仍报告失败并保留 RAM 参数和记录状态。
        if (flash_status != BSP_FLASH_OK)
            BSP_LOG_EVENT_1(EV_BSP_PARAM_BSP_PARAM_C_11,
                            BSP_LOG_FIELD_U32((unsigned long)flash_status));
        else
            BSP_LOG_EVENT_0(EV_BSP_PARAM_BSP_PARAM_C_12);
        return 0u;
    }

    // 擦除和收尾检查都成功后，才将 RAM 中的参数标为空。
    s_payload_len = 0u;
    s_dirty = 0u;
    param_seq = 0u;
    param_loaded = 0u;
    BSP_LOG_EVENT_0(EV_BSP_PARAM_BSP_PARAM_C_13);
    return operation_ok;
}

void ParamDumpToLog(void)
{
    BSP_LOG_EVENT_4(EV_BSP_PARAM_BSP_PARAM_C_14, BSP_LOG_FIELD_U32((unsigned)param_loaded), BSP_LOG_FIELD_I32((param_active_region == 0) ? 'A' : 'B'), BSP_LOG_FIELD_U32((unsigned long)param_seq), BSP_LOG_FIELD_U32((unsigned)s_payload_len));

    uint16_t off = 0;
    // 逐条打印 RAM 内容；遇到长度超出已用区域的条目就停止。
    while (off + sizeof(ParamEntry_t) <= s_payload_len)
    {
        ParamEntry_t entry;
        memcpy(&entry, &s_payload[off], sizeof(entry));
        uint16_t data_off = (uint16_t)(off + sizeof(entry));
        if (data_off + entry.len > s_payload_len)
            break;
        // 每条最多显示前4字节的十六进制内容，不把它当作浮点数解释。
        uint32_t word = 0;
        memcpy(&word, &s_payload[data_off], (entry.len >= 4u) ? 4u : entry.len);
        BSP_LOG_EVENT_3(EV_BSP_PARAM_BSP_PARAM_C_15, BSP_LOG_FIELD_HEX32((unsigned)entry.key), BSP_LOG_FIELD_U32((unsigned)entry.len), BSP_LOG_FIELD_HEX32((unsigned long)word));
        off = (uint16_t)(data_off + ParamRoundUp4(entry.len));
    }
}

/* 便捷接口只按类型换算字节数，读取和暂存规则与 ParamGet/Set 相同。 */
uint8_t ParamGetFloat(uint16_t key, float *out)
{
    return ParamGet(key, out, sizeof(float));
}

uint8_t ParamSetFloat(uint16_t key, float value)
{
    return ParamSet(key, &value, sizeof(float));
}

uint8_t ParamGetFloats(uint16_t key, float *out, uint8_t count)
{
    return ParamGet(key, out, (uint16_t)(count * sizeof(float)));
}

uint8_t ParamSetFloats(uint16_t key, const float *in, uint8_t count)
{
    return ParamSet(key, in, (uint16_t)(count * sizeof(float)));
}

uint8_t ParamGetU32(uint16_t key, uint32_t *out)
{
    return ParamGet(key, out, sizeof(uint32_t));
}

uint8_t ParamSetU32(uint16_t key, uint32_t value)
{
    return ParamSet(key, &value, sizeof(uint32_t));
}
