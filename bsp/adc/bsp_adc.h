/**
 * @file bsp_adc.h
 * @brief 通过 ADC1 和 DMA 采集电源、按键数据，提供最近一批的平均读数。
 * BSPInit() 调用 BSP_ADCInit() 启动采集，LCD 和加热控制通过 BSP_ADCGetSample() 读取。
 * 使用原始读数先检查返回状态；使用换算后的电压还要检查 voltage_config_valid。
 */
#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

/** @brief ADC 参考电压，单位 mV；换算要求大于 0 且不超过 5000。 */
#ifndef BSP_ADC_VREF_MV
#define BSP_ADC_VREF_MV 3300u
#endif

/** @brief 16 位 ADC 的最大读数，本驱动要求为 65535。 */
#ifndef BSP_ADC_FULL_SCALE
#define BSP_ADC_FULL_SCALE 65535u
#endif

/** @brief 电源电压相对 ADC 引脚电压的倍数乘 1000；要求为 1～100000，默认值表示 11 倍。 */
#ifndef BSP_ADC_VCC_IN_DIVIDER_MILLI
#define BSP_ADC_VCC_IN_DIVIDER_MILLI 11000u
#endif

// 循环缓冲区保存 32 对电源、按键读数；前后半段各完成 16 对后分别取平均。
#define BSP_ADC_DMA_BATCH_PAIR_COUNT 32u
#define BSP_ADC_SAMPLE_MAX_AGE_MS 20u ///< 采样最多允许过去的时间，单位 ms，等于 20ms 仍可用

/** @brief 原始采样的读取结果；电压换算是否可用另看 voltage_config_valid。 */
typedef enum
{
    BSP_ADC_STATUS_NOT_READY = 0,   ///< 尚未启动 DMA，或还没有完成一批采样
    BSP_ADC_STATUS_VALID,           ///< 已有完整采样，没有 ADC 错误且未超过 20ms
    BSP_ADC_STATUS_STALE,           ///< 最近一批采样已超过 20ms
    BSP_ADC_STATUS_ERROR,           ///< 校准、启动 DMA 或采集过程中出错
    BSP_ADC_STATUS_INVALID_ARGUMENT ///< 输出指针为 NULL
} BSP_ADC_Status_e;

/** @brief 各错误分别占一位，error_flags 会保留本次初始化后出现过的错误。 */
typedef enum
{
    BSP_ADC_ERROR_NONE = 0u,             ///< 尚无错误
    BSP_ADC_ERROR_CALIBRATION = 1u << 0, ///< ADC 校准失败
    BSP_ADC_ERROR_DMA_START = 1u << 1,   ///< DMA 启动失败
    BSP_ADC_ERROR_RUNTIME = 1u << 2      ///< 采集过程中 ADC 报错
} BSP_ADC_Error_e;

/** @brief 最近完成的半段采样结果；两个通道分别对 16 个读数取整数平均值。 */
typedef struct
{
    uint16_t raw_vcc_in;          ///< PC4 电源采样的平均读数，范围 0～65535
    uint16_t raw_key;             ///< PA5 按键采样的平均读数，范围 0～65535
    uint32_t sequence;            ///< 每完成半段加 1，超过 UINT32_MAX 后回到 0；读取不会增加
    uint32_t timestamp_ms;        ///< 完成该半段时的 HAL_GetTick()，单位 ms
    uint32_t error_flags;         ///< BSP_ADC_Error_e 中各错误位的组合
    uint32_t hal_error_code;      ///< 最近一次错误：初始化时为 HAL 返回状态，运行时为 ADC ErrorCode
    float vcc_in_volts;           ///< 按参考电压及分压倍数还原的电源电压，单位 V；配置不合要求时为 0
    uint8_t voltage_config_valid; ///< 1：换算配置符合要求；0：电压不可用，原始读数仍可按返回状态使用
} BSP_ADC_Sample_t;

/**
 * @brief 清除旧采样和错误记录，校准 ADC1 后启动循环 DMA。
 * @note 由 BSPInit() 在启动阶段调用，须先完成 MX_DMA_Init() 和 MX_ADC1_Init()。
 *       本函数不负责停止已运行的 DMA，采集中不要重复调用。
 *       校准或启动失败时记录错误并返回，通过 BSP_ADCGetSample() 查询结果；
 *       启动成功后也要等第一批采样完成，才能读取可用数据。
 */
void BSP_ADCInit(void);

/**
 * @brief 复制最近一批采样，并检查 ADC 错误和采样过去的时间。
 * @param[out] sample 接收采样及错误信息的结构体；为 NULL 时不写输出。
 * @return 按顺序检查并返回 INVALID_ARGUMENT、ERROR、NOT_READY、STALE；全部通过返回 VALID。
 *         各返回值均以 BSP_ADC_STATUS_ 为前缀，采样过去的时间不超过 20ms 才可用。
 * @note 指针非空时，即使未就绪、过期或报错，也会填入最近的记录，输出不会因失败而统一清零。
 *       使用原始读数须先确认返回 VALID；使用 vcc_in_volts 还须确认 voltage_config_valid 为 1。
 *       本函数不等待新采样，重复读取不会改变 sequence 或 timestamp_ms。
 */
BSP_ADC_Status_e BSP_ADCGetSample(BSP_ADC_Sample_t *sample);

#endif // !BSP_ADC_H
