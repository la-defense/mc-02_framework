#include "bsp_param.h"
#include "bsp_watchdog.h"
#include "bsp_log.h"
#include "main.h"
#include <string.h>

/* ---- Flash 物理参数(H723, 1MB, 8×128KB 扇区) ---- */
#define PARAM_SECTOR_A_ADDR 0x080C0000u /* FLASH_SECTOR_6 */
#define PARAM_SECTOR_B_ADDR 0x080E0000u /* FLASH_SECTOR_7 */
#define PARAM_SECTOR_SIZE 0x20000u      /* 128KB */
#define PARAM_FLASHWORD 32u             /* H7 的编程粒度: 256bit = 32 字节 */

/* ---- 记录头(32 字节, 与 flash word 对齐) ---- */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t payload_len;
    uint32_t seq;
    uint32_t crc32;
    uint32_t reserved[4];
} ParamHeader_t;

_Static_assert(sizeof(ParamHeader_t) == PARAM_FLASHWORD, "记录头必须正好 32 字节");

/* RAM 缓存: 直接保存 payload(TLV), 提交时原样写进 Flash */
static uint8_t s_payload[PARAM_MAX_PAYLOAD];
static uint16_t s_payload_len = 0;
static uint8_t s_dirty = 0;

volatile uint32_t param_commit_count = 0;
volatile uint32_t param_commit_fail = 0;
volatile uint8_t param_active_region = 0;
volatile uint32_t param_seq = 0;
volatile uint8_t param_loaded = 0;

/* ------------------------------ CRC32(IEEE, 软件实现) ------------------------------ */
static uint32_t ParamCrc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8u; ++bit)
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
    }
    return ~crc;
}

/* ------------------------------ TLV 读写 ------------------------------ */
typedef struct
{
    uint16_t key;
    uint16_t len;
} ParamEntry_t;

static uint16_t ParamRoundUp4(uint16_t v)
{
    return (uint16_t)((v + 3u) & (uint16_t)~3u);
}

/* 在 RAM 缓存里查找 key, 返回数据偏移; 找不到返回 0 */
static uint16_t ParamFindEntry(uint16_t key, uint16_t *out_len)
{
    uint16_t off = 0;
    while (off + sizeof(ParamEntry_t) <= s_payload_len)
    {
        ParamEntry_t entry;
        memcpy(&entry, &s_payload[off], sizeof(entry));
        uint16_t data_off = (uint16_t)(off + sizeof(entry));
        if (data_off + entry.len > s_payload_len)
            break; /* 结构损坏, 停止扫描 */
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
    if (data_off == 0 || entry_len != len)
        return 0;
    memcpy(buf, &s_payload[data_off], len);
    return 1;
}

uint8_t ParamSet(uint16_t key, const void *buf, uint16_t len)
{
    if (buf == NULL || len == 0u || len > (PARAM_MAX_PAYLOAD - sizeof(ParamEntry_t)))
        return 0;

    uint16_t entry_len = 0;
    uint16_t data_off = ParamFindEntry(key, &entry_len);

    if (data_off != 0 && entry_len == len)
    {
        /* 已有同长度条目: 原地覆盖 */
        memcpy(&s_payload[data_off], buf, len);
        s_dirty = 1;
        return 1;
    }

    /* 需要新增或改长度: 先删除旧条目, 再追加到末尾 */
    if (data_off != 0)
    {
        uint16_t entry_total = (uint16_t)(sizeof(ParamEntry_t) + ParamRoundUp4(entry_len));
        uint16_t tail = (uint16_t)(data_off - sizeof(ParamEntry_t) + entry_total);
        memmove(&s_payload[data_off - sizeof(ParamEntry_t)], &s_payload[tail], (size_t)(s_payload_len - tail));
        s_payload_len = (uint16_t)(s_payload_len - entry_total);
    }

    uint16_t need = (uint16_t)(sizeof(ParamEntry_t) + ParamRoundUp4(len));
    if ((uint32_t)s_payload_len + need > PARAM_MAX_PAYLOAD)
    {
        LOGERROR("[param] 参数区已满, 写入 key=0x%04X 失败", (unsigned)key);
        return 0;
    }

    ParamEntry_t entry = {.key = key, .len = len};
    memcpy(&s_payload[s_payload_len], &entry, sizeof(entry));
    memcpy(&s_payload[s_payload_len + sizeof(entry)], buf, len);
    uint16_t pad = (uint16_t)(ParamRoundUp4(len) - len);
    if (pad > 0u)
        memset(&s_payload[s_payload_len + sizeof(entry) + len], 0, pad);
    s_payload_len = (uint16_t)(s_payload_len + need);
    s_dirty = 1;
    return 1;
}

/* ------------------------------ Flash 读写 ------------------------------ */
static uint8_t ParamWriteSector(uint32_t sector_addr)
{
    /* 组织成 32 字节对齐的 flash word 缓冲: 头 + payload + 补零 */
    static __attribute__((aligned(32))) uint8_t buf[PARAM_FLASHWORD + PARAM_MAX_PAYLOAD + PARAM_FLASHWORD];
    ParamHeader_t header;
    memset(buf, 0, sizeof(buf));

    header.magic = PARAM_MAGIC;
    header.version = PARAM_VERSION;
    header.payload_len = s_payload_len;
    header.seq = (uint32_t)param_seq + 1u;
    header.crc32 = ParamCrc32(s_payload, s_payload_len);
    memset(header.reserved, 0, sizeof(header.reserved));

    memcpy(buf, &header, sizeof(header));
    memcpy(&buf[PARAM_FLASHWORD], s_payload, s_payload_len);

    uint32_t total = (uint32_t)PARAM_FLASHWORD + ParamRoundUp4(s_payload_len);
    total = (total + PARAM_FLASHWORD - 1u) & ~(PARAM_FLASHWORD - 1u);

    /* 擦除该扇区(单 bank, 期间 CPU 停摆) */
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = (sector_addr == PARAM_SECTOR_A_ADDR) ? FLASH_SECTOR_6 : FLASH_SECTOR_7;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
    {
        HAL_FLASH_Lock();
        LOGERROR("[param] 扇区擦除失败(0x%08lX, err=%lu)", (unsigned long)sector_addr,
                 (unsigned long)sector_error);
        return 0;
    }

    /* 按 32 字节 flash word 写入 */
    for (uint32_t off = 0; off < total; off += PARAM_FLASHWORD)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, sector_addr + off,
                              (uint32_t)(uintptr_t)&buf[off]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            LOGERROR("[param] 写入失败(off=%lu)", (unsigned long)off);
            return 0;
        }
    }
    HAL_FLASH_Lock();

    /* 回读校验 */
    SCB_InvalidateDCache_by_Addr((uint32_t *)sector_addr, (int32_t)total);
    if (memcmp((const void *)sector_addr, buf, total) != 0)
    {
        LOGERROR("[param] 回读校验失败(0x%08lX)", (unsigned long)sector_addr);
        return 0;
    }
    return 1;
}

/* 读取某个扇区的记录头, 校验并返回是否有效; 有效时把 payload 拷进 out */
static uint8_t ParamReadSector(uint32_t sector_addr, uint8_t *out, ParamHeader_t *out_hdr)
{
    ParamHeader_t hdr;
    SCB_InvalidateDCache_by_Addr((uint32_t *)sector_addr, PARAM_FLASHWORD);
    memcpy(&hdr, (const void *)sector_addr, sizeof(hdr));

    if (hdr.magic != PARAM_MAGIC || hdr.version != PARAM_VERSION ||
        hdr.payload_len > PARAM_MAX_PAYLOAD)
        return 0;

    SCB_InvalidateDCache_by_Addr((uint32_t *)(sector_addr + PARAM_FLASHWORD),
                                 (int32_t)ParamRoundUp4(hdr.payload_len));
    const uint8_t *payload = (const uint8_t *)(sector_addr + PARAM_FLASHWORD);
    if (ParamCrc32(payload, hdr.payload_len) != hdr.crc32)
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

    /* 幂等: 参数区在启动期读一次就够, 重复调用直接返回上次结果,
       避免把后来 ParamSet 的 RAM 缓存又用 Flash 老内容覆盖掉 */
    if (s_inited)
        return param_loaded;
    s_inited = 1;

    uint8_t valid_a = ParamReadSector(PARAM_SECTOR_A_ADDR, NULL, &hdr_a);
    uint8_t valid_b = ParamReadSector(PARAM_SECTOR_B_ADDR, NULL, &hdr_b);

    uint32_t sector_addr;
    if (valid_a && (!valid_b || hdr_a.seq >= hdr_b.seq))
    {
        sector_addr = PARAM_SECTOR_A_ADDR;
        param_active_region = 0;
    }
    else if (valid_b)
    {
        sector_addr = PARAM_SECTOR_B_ADDR;
        param_active_region = 1;
    }
    else
    {
        /* 两个区都没有有效记录: 参数为空, 由调用方使用默认值 */
        s_payload_len = 0;
        param_loaded = 0;
        param_seq = 0;
        LOGWARNING("[param] 未找到有效参数记录(A/B 都无效)");
        return 0;
    }

    ParamHeader_t hdr;
    (void)ParamReadSector(sector_addr, s_payload, &hdr);
    s_payload_len = hdr.payload_len;
    param_seq = hdr.seq;
    param_loaded = 1;
    s_dirty = 0;
    LOGINFO("[param] 载入参数: 区=%c seq=%lu, %u 字节", (param_active_region == 0) ? 'A' : 'B',
            (unsigned long)param_seq, (unsigned)s_payload_len);
    return 1;
}

uint8_t ParamCommit(void)
{
    uint32_t target = (param_active_region == 0) ? PARAM_SECTOR_B_ADDR : PARAM_SECTOR_A_ADDR;
    uint8_t target_region = (param_active_region == 0) ? 1u : 0u;

    /* 擦/写期间 CPU 停摆, 先把看门狗窗口放长(默认 8s > 最坏扇区擦除 ~4s) */
    uint32_t prev_timeout = BSP_WatchdogGetTimeout();
    BSP_WatchdogSetTimeout(8000);
    BSP_WatchdogFeed();

    uint8_t ok = ParamWriteSector(target);
    if (ok)
    {
        param_active_region = target_region;
        param_seq++;
        s_dirty = 0;
        param_loaded = 1;
        param_commit_count++;
        LOGINFO("[param] 提交成功: 区=%c seq=%lu", (target_region == 0) ? 'A' : 'B',
                (unsigned long)param_seq);
    }
    else
    {
        param_commit_fail++;
        LOGERROR("[param] 提交失败, 继续使用旧记录(区=%c)", (param_active_region == 0) ? 'A' : 'B');
    }

    BSP_WatchdogSetTimeout(prev_timeout); /* 恢复原来的超时(启动期 4s / 运行期 200ms) */
    BSP_WatchdogFeed();
    return ok;
}

void ParamReset(void)
{
    uint32_t prev_timeout = BSP_WatchdogGetTimeout();
    BSP_WatchdogSetTimeout(8000);
    BSP_WatchdogFeed();

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = FLASH_SECTOR_6;
    erase.NbSectors = 2; /* 扇区 6+7 一起擦 */
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_StatusTypeDef ret = HAL_FLASHEx_Erase(&erase, &sector_error);
    HAL_FLASH_Lock();

    BSP_WatchdogSetTimeout(prev_timeout);
    BSP_WatchdogFeed();

    s_payload_len = 0;
    s_dirty = 0;
    param_seq = 0;
    param_loaded = 0;
    if (ret != HAL_OK)
        LOGERROR("[param] 擦除参数区失败(err=%lu)", (unsigned long)sector_error);
    else
        LOGWARNING("[param] 参数区已清空(恢复默认值)");
}

void ParamDumpToLog(void)
{
    LOGINFO("[param] dump: loaded=%u region=%c seq=%lu len=%u",
            (unsigned)param_loaded, (param_active_region == 0) ? 'A' : 'B',
            (unsigned long)param_seq, (unsigned)s_payload_len);

    uint16_t off = 0;
    while (off + sizeof(ParamEntry_t) <= s_payload_len)
    {
        ParamEntry_t entry;
        memcpy(&entry, &s_payload[off], sizeof(entry));
        uint16_t data_off = (uint16_t)(off + sizeof(entry));
        if (data_off + entry.len > s_payload_len)
            break;
        /* RTT 不支持浮点, 用十六进制打印原始字节 */
        uint32_t word = 0;
        memcpy(&word, &s_payload[data_off], (entry.len >= 4u) ? 4u : entry.len);
        LOGINFO("  key=0x%04X len=%u data[0..3]=0x%08lX", (unsigned)entry.key,
                (unsigned)entry.len, (unsigned long)word);
        off = (uint16_t)(data_off + ParamRoundUp4(entry.len));
    }
}

/* ------------------------------ 便捷接口 ------------------------------ */
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
