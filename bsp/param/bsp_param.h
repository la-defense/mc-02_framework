#ifndef BSP_PARAM_H
#define BSP_PARAM_H

#include <stdint.h>

/* ============================================================================
   通用内部 Flash 参数存储 (2026-09 新增)
   ----------------------------------------------------------------------------
   - 物理布局: STM32H723 内部 Flash 的**扇区 6/7**(各 128KB)作为 A/B 两个区:
       A: 0x080C0000 (FLASH_SECTOR_6)
       B: 0x080E0000 (FLASH_SECTOR_7)
     链接脚本已把 FLASH 收到 768K, 这两块不会再被程序占用。
   - 记录格式(32 字节对齐):
       magic("MC02") | version(u16) | payload_len(u16) | seq(u32) | crc32(u32) | reserved
       + payload: 一串 TLV 条目 { key(u16) | len(u16) | data[len] }(每条按 4 字节对齐)
   - 提交(A/B 交替): 擦除**非活动区** → 写入新记录 → 回读校验 → 切换活动区。
     旧记录仍留在另一个区, 因此"A 区坏了 B 区还在", 掉电/擦写中途失败都能回退。
   - 注意: H7 是单 bank, 擦/写期间 CPU 会停摆(最坏几秒), 所以提交前会把 IWDG
     超时临时放长(默认 8s), 完成后恢复原值; 提交只应在 SAFE/CALIB(电机失能)时做。
   ========================================================================= */

#define PARAM_MAGIC 0x3243304Du /* 'MC02' 小端写法 */
#define PARAM_VERSION 1u
#define PARAM_MAX_PAYLOAD 512u /* 一条记录里 payload 的最大字节数 */

/* 参数 key 规划(高字节分段, 方便未来扩展) */
typedef enum
{
    PARAM_KEY_IMU_GYRO_OFFSET = 0x0100, /* float[3]: 陀螺仪零偏 */
    PARAM_KEY_IMU_G_NORM = 0x0101,      /* float  : 重力加速度模长 */
    PARAM_KEY_IMU_CALIB_META = 0x0102,  /* u32 结果/状态 | float 标定温度 | u32 标定时刻(s) */
    /* 0x0200~ 预留给 CAN 参数; 0x0300~ 预留给底盘参数; 0x0400~ 预留给配置框架 */
} Param_Key_e;

/* 标定/加载结果(写入 PARAM_KEY_IMU_CALIB_META 的第一项) */
typedef enum
{
    PARAM_IMU_CALIB_NONE = 0,    /* 无记录 */
    PARAM_IMU_CALIB_OK = 1,      /* 在线标定成功 */
    PARAM_IMU_CALIB_FAILED = 2,  /* 标定失败(用默认值, 下次仍会尝试) */
} Param_ImuCalibResult_e;

/* PARAM_KEY_IMU_CALIB_META 的内容: 标定结果 + 标定时的温度 + 标定时刻 */
typedef struct
{
    uint32_t result;      /* Param_ImuCalibResult_e */
    float temperature;    /* 标定时的 IMU 温度(摄氏度) */
    uint32_t time_s;      /* 距上电的秒数(仅作记录, 没有 RTC) */
} ParamImuCalibMeta_t;

_Static_assert(sizeof(ParamImuCalibMeta_t) == 12, "标定元数据必须是 12 字节");

/**
 * @brief 初始化参数存储: 扫描 A/B 区, 校验并从 seq 更大的有效记录载入 RAM 缓存
 * @return 1=载入到有效记录; 0=两个区都没有有效记录(参数为空, 调用方应使用默认值)
 */
uint8_t ParamInit(void);

/** @brief 读取一个参数; 返回 1=存在且长度匹配 */
uint8_t ParamGet(uint16_t key, void *buf, uint16_t len);
/** @brief 写入(暂存到 RAM, 不落 Flash); 返回 1=成功 */
uint8_t ParamSet(uint16_t key, const void *buf, uint16_t len);
/** @brief 把 RAM 缓存提交到 Flash(A/B 交替); 返回 1=成功 */
uint8_t ParamCommit(void);
/** @brief 擦除 A/B 两个区并清空 RAM 缓存 */
void ParamReset(void);
/** @brief 通过 RTT 打印当前所有参数(调试用) */
void ParamDumpToLog(void);

/* 便捷接口 */
uint8_t ParamGetFloat(uint16_t key, float *out);
uint8_t ParamSetFloat(uint16_t key, float value);
uint8_t ParamGetFloats(uint16_t key, float *out, uint8_t count);
uint8_t ParamSetFloats(uint16_t key, const float *in, uint8_t count);
uint8_t ParamGetU32(uint16_t key, uint32_t *out);
uint8_t ParamSetU32(uint16_t key, uint32_t value);

/* 运行状态(供 LCD / OpenOCD 观察) */
extern volatile uint32_t param_commit_count;  /* 成功提交次数 */
extern volatile uint32_t param_commit_fail;   /* 失败次数 */
extern volatile uint8_t param_active_region;  /* 0=A 1=B */
extern volatile uint32_t param_seq;           /* 当前记录序号 */
extern volatile uint8_t param_loaded;         /* 1=启动时从 Flash 载入了有效记录 */

#endif // BSP_PARAM_H
