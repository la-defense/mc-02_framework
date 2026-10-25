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
#include <stddef.h>

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

/* STM32H723 链接脚本声明768KB应用区和两个128KB参数区(A/B)，
 * 参数驱动从链接符号校验分区位置，再以分区内偏移访问。记录由32字节头部和参数条目组成；
 * 每条保存 key、字节长度和数据，末尾补到4字节的整数倍。
 * 提交时写另一个区，保留当前记录；下次启动检查两区，选择可用且序号较大的记录。
 * 擦写会暂停从同一 Flash 取指，必须在电机失能、允许等待的任务中调用。
 * MC02-0036 回调先关闭并锁住加热、检查任务状态；成功后驱动发放最多8000ms的统一授权。
 * 本模块将 IWDG 请求时限暂改为相同期限，结束后恢复；应用区通过驱动只读。 */

#define PARAM_MAGIC 0x3243304Du /* 按内存中的字节顺序读作 'MC02' */
#define PARAM_VERSION 1u
#define PARAM_MAX_PAYLOAD 512u /* 一条记录中，所有参数条目最多共占512字节，不含记录头 */

/* 参数 key 规划(高字节分段, 方便未来扩展) */
typedef enum
{
    PARAM_KEY_IMU_GYRO_OFFSET = 0x0100, /* float[3]：x/y/z 轴陀螺仪零偏，单位 rad/s */
    PARAM_KEY_IMU_G_NORM = 0x0101,      /* float：静止时的加速度大小，单位 m/s² */
    PARAM_KEY_IMU_CALIB_META = 0x0102,  /* u32 结果/状态 | float 标定温度 | u32 标定时刻(s) */
    PARAM_KEY_IMU_CALIB_SCHEMA_VERSION = 0x0103, /* u32：BMI088 标定参数组逻辑版本 */
    /* 0x0200~ 仍预留给 CAN；0x0300~ 仍预留给底盘；0x0400~ 仍预留给配置描述。 */
} Param_Key_e;

/** @brief BMI088 标定组当前逻辑版本；与 Flash 记录容器版本分开。 */
#define PARAM_IMU_CALIB_SCHEMA_VERSION 2u
/** @brief 每轴陀螺仪零偏允许绝对值，单位 rad/s。 */
#define PARAM_IMU_GYRO_OFFSET_LIMIT_RAD_S 0.1f
/** @brief 未找到有效标定组时陀螺仪零偏默认值，单位 rad/s。 */
#define PARAM_IMU_GYRO_OFFSET_DEFAULT_RAD_S 0.0f
/** @brief BMI088 静止重力模长允许下限，单位 m/s²。 */
#define PARAM_IMU_G_NORM_MIN_M_S2 9.0f
/** @brief BMI088 静止重力模长允许上限，单位 m/s²。 */
#define PARAM_IMU_G_NORM_MAX_M_S2 10.5f
/** @brief 未找到有效标定组时的重力模长默认值，单位 m/s²。 */
#define PARAM_IMU_G_NORM_DEFAULT_M_S2 9.805f
/** @brief 标定元数据温度允许下边界，单位 °C；边界值本身不接受。 */
#define PARAM_IMU_CALIB_META_TEMP_MIN_C (-40.0f)
/** @brief 标定元数据温度允许上边界，单位 °C；边界值本身不接受。 */
#define PARAM_IMU_CALIB_META_TEMP_MAX_C 100.0f
/** @brief 参数描述名称副本的字节容量，包含结尾零字节。 */
#define PARAM_DESCRIPTOR_NAME_CAPACITY 32u
/** @brief 参数描述单位副本的字节容量，包含结尾零字节。 */
#define PARAM_DESCRIPTOR_UNIT_CAPACITY 24u

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

/** @brief 参数值在描述目录中的存储类型。 */
typedef enum
{
    PARAM_VALUE_FLOAT32 = 0,
    PARAM_VALUE_FLOAT32_ARRAY,
    PARAM_VALUE_IMU_CALIB_META,
    PARAM_VALUE_UINT32,
} ParamValueType_e;

/** @brief 参数被载入或更新后由哪个模块应用。 */
typedef enum
{
    PARAM_EFFECTIVE_BMI088_INIT = 0, /* BMI088 初始化时读取；在线重标定成功后立即应用 */
    PARAM_EFFECTIVE_DISPLAY_ONLY,    /* 只供标定状态显示，不参与控制计算 */
    PARAM_EFFECTIVE_STORAGE_ONLY,    /* 仅说明参数组版本，不参与运行计算 */
} ParamEffectiveMode_e;

/** @brief 描述目录查询和参数组操作返回的结果。 */
typedef enum
{
    PARAM_RESULT_OK = 0,
    PARAM_RESULT_INVALID_ARGUMENT,
    PARAM_RESULT_INDEX_OUT_OF_RANGE,
    PARAM_RESULT_NOT_INITIALIZED,
    PARAM_RESULT_NOT_FOUND,
    PARAM_RESULT_INVALID_VALUE,
    PARAM_RESULT_UNSUPPORTED_VERSION,
    PARAM_RESULT_NO_SPACE,
} ParamResult_e;

/**
 * @brief 可持久化参数的只读说明副本。
 * @note 名称和单位保存在本结构的定长数组中；没有默认值的字段由对应标记说明。
 *       浮点端点是否包含由 inclusive 字段说明；复合类型可同时使用浮点和整数范围。
 */
typedef struct
{
    uint16_t key;                         /**< 参数编号。 */
    ParamValueType_e value_type;          /**< 存储值类型。 */
    ParamEffectiveMode_e effective_mode;  /**< 参数在哪个阶段生效。 */
    uint16_t byte_length;                 /**< 固定字节长度。 */
    uint16_t logical_version;             /**< 当前描述所属逻辑版本。 */
    uint8_t element_count;                 /**< 元素数量；复合值按成员数填写。 */
    uint8_t optional;                      /**< 1表示可缺省，0表示必需。 */
    uint8_t has_default_float;             /**< 1表示 default_float 有效。 */
    uint8_t has_default_u32;               /**< 1表示 default_u32 有效。 */
    uint8_t minimum_float_inclusive;       /**< 浮点下边界是否包含，1包含。 */
    uint8_t maximum_float_inclusive;       /**< 浮点上边界是否包含，1包含。 */
    float minimum_float;                   /**< 浮点范围下界，单位由 unit 给出。 */
    float maximum_float;                   /**< 浮点范围上界，单位由 unit 给出。 */
    float default_float[3];                /**< 最多三个浮点默认值。 */
    uint32_t minimum_u32;                  /**< 整数范围下界。 */
    uint32_t maximum_u32;                  /**< 整数范围上界。 */
    uint32_t default_u32;                  /**< 整数默认值。 */
    char name[PARAM_DESCRIPTOR_NAME_CAPACITY]; /**< UTF-8名称副本，包含结尾零字节。 */
    char unit[PARAM_DESCRIPTOR_UNIT_CAPACITY]; /**< 单位副本，包含结尾零字节。 */
} ParamDescriptor_t;

/**
 * @brief BMI088 标定参数组。
 * @note 零偏每轴范围为[-0.1,0.1] rad/s，重力模长范围为[9.0,10.5] m/s²。
 *       元数据可缺省；存在时结果须为 PARAM_IMU_CALIB_OK，温度范围为(-40,100) °C。
 */
typedef struct
{
    float gyro_offset_rad_s[3]; /**< x/y/z轴陀螺仪零偏，单位 rad/s。 */
    float g_norm_m_s2;          /**< 静止重力模长，单位 m/s²。 */
    ParamImuCalibMeta_t meta;   /**< 可选的结果、摄氏温度和启动秒数。 */
    uint8_t has_meta;           /**< 取0表示无元数据，取1表示使用 meta；其他值非法。 */
} ParamImuCalibrationGroup_t;

/**
 * @brief 调用者保存的参数组候选。Stage 成功后才可交给 Accept。
 * @note 事务由调用者串行使用；Accept 成功后标记清零，失败时当前 RAM 参数不变。
 */
typedef struct
{
    ParamImuCalibrationGroup_t candidate;
    uint8_t staged;
} ParamImuCalibrationTransaction_t;

/**
 * @brief 首次调用时检查 A/B 区，将可用记录载入 RAM；两区都可用时取序号较大的，相同取 A。
 * @return 首次调用：1 表示载入成功，0 表示两区都不可用，RAM 参数为空，应使用默认值。
 *         后续调用直接返回当前 param_loaded，不重新读取 Flash。
 * @note 应在首次读写参数前调用；先检查记录格式和 CRC，再检查已知参数组的长度、版本及取值。
 *       较新记录的已知参数组不合法时，会尝试另一 A/B 记录；Flash 记录格式仍为 v1。
 */
uint8_t ParamInit(void);

/**
 * @brief 返回只读描述目录的条目数量。
 * @return 当前固定目录中的描述项数量；不访问 Flash，也不改写模块状态。
 */
size_t ParamGetDescriptorCount(void);
/**
 * @brief 按序号复制一条参数描述。
 * @param[in] index 目录下标，从0开始。
 * @param[out] output 接收描述；必须非空。
 * @return 成功返回 PARAM_RESULT_OK；参数为空或下标越界时失败且不改写 output。
 */
ParamResult_e ParamGetDescriptor(size_t index, ParamDescriptor_t *output);

/**
 * @brief 读取 RAM 中已验证的 BMI088 标定组。
 * @param[out] output 接收偏置、重力模长及可选元数据；必须非空。
 * @param[out] source_version 接收来源逻辑版本，1表示旧 A/B 记录，2表示新记录；可为空。
 * @return 找到完整有效参数组时返回 PARAM_RESULT_OK；缺少组、未初始化或参数无效时失败且不改写输出。
 */
ParamResult_e ParamImuCalibrationRead(ParamImuCalibrationGroup_t *output,
                                      uint32_t *source_version);
/**
 * @brief 校验 BMI088 候选组并复制到调用者事务。
 * @param[in] candidate 候选组；偏置每轴范围为[-0.1,0.1] rad/s，重力模长为[9.0,10.5] m/s²。
 * @param[out] transaction 接收已校验候选；必须非空。
 * @return 成功返回 PARAM_RESULT_OK；失败时 transaction 保持原值。
 */
ParamResult_e ParamImuCalibrationStage(const ParamImuCalibrationGroup_t *candidate,
                                       ParamImuCalibrationTransaction_t *transaction);
/**
 * @brief 将已暂存的整组参数原子接受到 RAM，不执行 Flash 擦写。
 * @param[in,out] transaction Stage 成功的事务；成功后 staged 清零，可安全复用。
 * @return 成功返回 PARAM_RESULT_OK；未初始化、参数无效或空间不足时保留整组旧 RAM 值。
 * @note 新接受值使用逻辑版本2；持久化仍须另行调用 ParamCommit。
 */
ParamResult_e ParamImuCalibrationAccept(ParamImuCalibrationTransaction_t *transaction);

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
 * @brief 复制未纳入已描述参数组的原始参数到 RAM；需要 ParamCommit 才能在掉电后保留。
 * @param[in] key 参数编号；已有同编号时替换内容。
 * @param[in] buf 待复制数据，至少有 len 字节；返回后可改写或释放，NULL 时返回0。
 * @param[in] len 数据字节数，1～508；所有条目连同编号、长度及补齐字节共用512字节。
 * @return 1 表示暂存成功；0 表示参数不符、属于受描述参数组或空间不足。
 * @note 本函数先在候选缓冲中计算完整新条目表；空间不足时旧 RAM 参数保持原样。
 *       已描述的 BMI088 标定字段和组版本只能通过 ParamImuCalibrationStage/Accept 修改。
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
