/**
 * @file bsp_param.h
 * @brief 在 RAM 中读写参数，并交替保存到内部 Flash 的 A/B 两个区。
 *
 * BSPInit 调用 ParamInit 载入记录；ParamSet 只暂存，ParamCommit 才保存。
 * RobotInit 注册擦写前后的回调，BMI088 标定代码通过这些接口保存标定值。
 * 调用方应先初始化，并安排各次读写依次执行；本模块没有处理同时访问。
 */
#ifndef BSP_PARAM_H
#define BSP_PARAM_H

#include <stdint.h>

/** @brief 擦写前的准备函数：非零表示允许开始，0 表示拒绝；拒绝时自行处理已做的准备。 */
typedef uint8_t (*ParamLongOperationBeginFn)(void);
/**
 * @brief 擦写后的收尾函数，仅在 begin 允许后调用。
 * @param[in] success 非零表示写入及回读成功，或两个区擦除成功；0 表示 Flash 操作失败。
 * @return 非零表示收尾检查通过，0 表示失败。
 */
typedef uint8_t (*ParamLongOperationEndFn)(uint8_t success);

typedef struct
{
    ParamLongOperationBeginFn begin; /* 开始前关闭加热、检查任务状态并登记允许耗时的时段 */
    ParamLongOperationEndFn end;     /* 结束后检查用时，并通知加热器本次结果 */
} ParamLongOperationGuard_t;

/* STM32H723 扇区6/7各128KB：A 从0x080C0000开始，B 从0x080E0000开始。
 * 链接脚本只给程序使用前768KB。记录由32字节头部和参数条目组成；
 * 每条保存 key、字节长度和数据，末尾补到4字节的整数倍。
 * 提交时写另一个区，保留当前记录；下次启动检查两区，选择可用且序号较大的记录。
 * 擦写会暂停从同一 Flash 取指，必须在电机失能、允许等待的任务中调用。
 * 本模块将 IWDG 请求时限暂改为8000ms，结束后恢复；任务与加热检查由回调完成。 */

#define PARAM_MAGIC 0x3243304Du /* 按内存中的字节顺序读作 'MC02' */
#define PARAM_VERSION 1u
#define PARAM_MAX_PAYLOAD 512u /* 一条记录中，所有参数条目最多共占512字节，不含记录头 */

/* 参数 key 规划(高字节分段, 方便未来扩展) */
typedef enum
{
    PARAM_KEY_IMU_GYRO_OFFSET = 0x0100, /* float[3]：x/y/z 轴陀螺仪零偏，单位 rad/s */
    PARAM_KEY_IMU_G_NORM = 0x0101,      /* float：静止时的加速度大小，单位 m/s² */
    PARAM_KEY_IMU_CALIB_META = 0x0102,  /* u32 结果/状态 | float 标定温度 | u32 标定时刻(s) */
    /* 0x0200~ 预留给 CAN 参数; 0x0300~ 预留给底盘参数; 0x0400~ 预留给配置框架 */
} Param_Key_e;

/* 标定/加载结果(写入 PARAM_KEY_IMU_CALIB_META 的第一项) */
typedef enum
{
    PARAM_IMU_CALIB_NONE = 0,    /* 无记录 */
    PARAM_IMU_CALIB_OK = 1,      /* 在线标定成功 */
    PARAM_IMU_CALIB_FAILED = 2,  /* 标定未成功 */
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
 * @brief 首次调用时检查 A/B 区，将可用记录载入 RAM；两区都可用时取序号较大的，相同取 A。
 * @return 首次调用：1 表示载入成功，0 表示两区都不可用，RAM 参数为空，应使用默认值。
 *         后续调用直接返回当前 param_loaded，不重新读取 Flash。
 * @note 应在首次读写参数前调用；检查记录标记、版本、长度和数据 CRC，不检查参数数值范围。
 */
uint8_t ParamInit(void);

/**
 * @brief 设置擦写前后的回调，必须在 ParamCommit/ParamReset 前完成。
 * @param[in] guard 回调表，函数会复制表内指针，返回后可释放此表；NULL 表示清除设置。
 * @note 两个回调缺少任意一个，或 begin 返回0，都会拒绝擦写；操作过程中不要更换回调。
 */
void ParamSetLongOperationGuard(const ParamLongOperationGuard_t *guard);

/**
 * @brief 按 key 从 RAM 复制一个参数。
 * @param[in] key 参数编号。
 * @param[out] buf 接收数据的空间，至少有 len 字节；NULL 时返回0。
 * @param[in] len 要读取的字节数，必须等于该条目保存的长度。
 * @return 1 表示复制成功；0 表示空指针、未找到参数或长度不符，buf 内容保持原样。
 */
uint8_t ParamGet(uint16_t key, void *buf, uint16_t len);

/**
 * @brief 复制参数到 RAM；需要 ParamCommit 才能在掉电后保留。
 * @param[in] key 参数编号；已有同编号时替换内容。
 * @param[in] buf 待复制数据，至少有 len 字节；返回后可改写或释放，NULL 时返回0。
 * @param[in] len 数据字节数，1～508；所有条目连同编号、长度及补齐字节共用512字节。
 * @return 1 表示暂存成功；0 表示参数不符或空间不足。
 * @note 改变已有条目的长度时先移除旧条目；若随后空间不足，旧条目已从 RAM 移除。
 *       Flash 中的记录在本函数内保持原样。
 */
uint8_t ParamSet(uint16_t key, const void *buf, uint16_t len);

/**
 * @brief 将 RAM 参数写入另一个 Flash 区，回读一致后更新当前区和序号。
 * @return 1 表示写入、回读和 end 检查都成功；任一步拒绝或失败返回0，RAM 参数保留。
 * @note 写入及回读成功、但 end 检查失败时仍更新当前区、序号和成功计数，新记录已经保存。
 *       每次调用都会擦写，即使参数没有修改；调用条件见文件头和回调设置接口。
 */
uint8_t ParamCommit(void);

/**
 * @brief 擦除 A/B 两区，擦除及 end 检查都成功后清空 RAM 参数、序号和 param_loaded。
 * @return 1 表示完成；0 表示拒绝、擦除失败或 end 检查失败，RAM 参数和记录状态保留。
 * @note 返回0时 Flash 仍可能已被部分或全部擦除；不能据此认为旧记录还在。
 *       当前区编号与提交计数不清零；调用条件与 ParamCommit 相同。
 */
uint8_t ParamReset(void);

/** @brief 通过已初始化的 RTT 日志打印 RAM 条目的编号、字节数和最多前4字节。 */
void ParamDumpToLog(void);

/**
 * @brief 读取一个 float；按 ParamGet 的规则检查编号和长度。
 * @param[in] key 参数编号。
 * @param[out] out 接收一个 float，NULL 时失败。
 * @return 1 表示成功；0 表示失败，out 内容保持原样。
 */
uint8_t ParamGetFloat(uint16_t key, float *out);
/**
 * @brief 暂存一个 float，保存到 Flash 仍需 ParamCommit。
 * @param[in] key 参数编号。
 * @param[in] value 待暂存的数值，本函数不检查数值范围。
 * @return 1 表示暂存成功；0 表示失败，原因及处理见 ParamSet。
 */
uint8_t ParamSetFloat(uint16_t key, float value);
/**
 * @brief 读取一组 float，按 count * sizeof(float) 字节调用 ParamGet。
 * @param[in] key 参数编号。
 * @param[out] out 接收数组，至少可放 count 个 float，NULL 时失败。
 * @param[in] count 元素个数，必须与保存时的数组长度相符。
 * @return 1 表示成功；0 表示失败，out 内容保持原样。
 */
uint8_t ParamGetFloats(uint16_t key, float *out, uint8_t count);
/**
 * @brief 暂存一组 float，按 count * sizeof(float) 字节调用 ParamSet。
 * @param[in] key 参数编号。
 * @param[in] in 待复制数组，至少有 count 个 float，NULL 时失败；返回后可改写或释放。
 * @param[in] count 元素个数，不能为0；可用个数还受 ParamSet 的空间要求影响。
 * @return 1 表示暂存成功；0 表示失败，原因及处理见 ParamSet。
 */
uint8_t ParamSetFloats(uint16_t key, const float *in, uint8_t count);
/**
 * @brief 读取一个 uint32_t；按 ParamGet 的规则检查编号和长度。
 * @param[in] key 参数编号。
 * @param[out] out 接收一个 uint32_t，NULL 时失败。
 * @return 1 表示成功；0 表示失败，out 内容保持原样。
 */
uint8_t ParamGetU32(uint16_t key, uint32_t *out);
/**
 * @brief 暂存一个 uint32_t，保存到 Flash 仍需 ParamCommit。
 * @param[in] key 参数编号。
 * @param[in] value 待暂存的数值。
 * @return 1 表示暂存成功；0 表示失败，原因及处理见 ParamSet。
 */
uint8_t ParamSetU32(uint16_t key, uint32_t value);

/* 运行状态(供 LCD / OpenOCD 观察) */
extern volatile uint32_t param_commit_count;  /* 写入及回读成功次数，即使 end 检查失败也计入 */
extern volatile uint32_t param_commit_fail;   /* Commit 被拒绝或写入、回读失败的次数，不计 end 单独失败 */
extern volatile uint8_t param_active_region;  /* 0=A 1=B；没有记录时也有此值，不能单凭它判断记录可用 */
extern volatile uint32_t param_seq;           /* 当前记录序号 */
extern volatile uint8_t param_loaded;         /* 1=已载入或写入可用记录；成功 Reset 后变0 */

#endif // BSP_PARAM_H
