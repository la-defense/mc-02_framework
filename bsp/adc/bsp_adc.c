/**
 * @file bsp_adc.c
 * @brief 处理 ADC1 的电源、按键采样，接口用法见 bsp_adc.h。
 * BSP_ADCInit() 启动循环 DMA，完成回调对已采好的半段取平均，BSP_ADCGetSample() 复制结果。
 *
 * VCC_IN -- R86 100K -- PC4(ADC1_INP4) -- R87 10K -- GND
 * 分压比 = 10K / (100K + 10K) = 1/11
 * ADC1 依次采集 ADC_CHANNEL_4（PC4，电源）和 ADC_CHANNEL_19（PA5，按键）。
 */
#include "bsp_adc.h"
#include "adc.h"
#include "main.h"
#include "bsp_log.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

// 每对包含两个 uint16_t 读数，前后半段分别处理。
#define BSP_ADC_DMA_WORD_COUNT (BSP_ADC_DMA_BATCH_PAIR_COUNT * 2u)
#define BSP_ADC_DMA_HALF_WORD_COUNT (BSP_ADC_DMA_WORD_COUNT / 2u)
#define BSP_ADC_PAIRS_PER_HALF (BSP_ADC_DMA_HALF_WORD_COUNT / 2u)
#define BSP_ADC_CACHE_LINE_BYTES 32u

// 缓存操作每次处理整组 32 字节，缓冲区从组的起点开始，避免影响旁边的数据。
static uint16_t adc_dma_buffer[BSP_ADC_DMA_WORD_COUNT] __attribute__((aligned(BSP_ADC_CACHE_LINE_BYTES)));
static volatile BSP_ADC_Sample_t latest_sample;
static volatile uint32_t adc_error_flags = BSP_ADC_ERROR_NONE;
static volatile uint32_t adc_hal_error_code = 0u;
static volatile uint8_t adc_started = 0u;
static volatile uint8_t sample_ready = 0u;
// 只读诊断保留在 RAM 中；更新任务和离线 ELF 工具共用头文件中的格式说明。
volatile BSP_ADC_Diagnostic_t bsp_adc_diagnostic;
static uint8_t diagnostic_published;

_Static_assert(sizeof(float) == sizeof(uint32_t), "ADC diagnostics require binary32 float");
_Static_assert(FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
               "ADC diagnostics require IEEE-754 binary32 float");
_Static_assert(sizeof(BSP_ADC_Diagnostic_t) == 15u * sizeof(uint32_t),
               "ADC diagnostic layout must contain 15 contiguous 32-bit fields");

// 检查参考电压、ADC 最大读数和分压倍数是否符合换算要求。
static uint8_t BSP_ADCVoltageConfigValid(void)
{
    return (BSP_ADC_VREF_MV > 0u && BSP_ADC_VREF_MV <= 5000u &&
            BSP_ADC_FULL_SCALE == 65535u && BSP_ADC_VCC_IN_DIVIDER_MILLI > 0u &&
            BSP_ADC_VCC_IN_DIVIDER_MILLI <= 100000u)
               ? 1u
               : 0u;
}

// 保留出现过的错误位，并保存最近一次 HAL 错误；修改期间暂时关闭普通中断。
static void BSP_ADCSetError(uint32_t error_flag, uint32_t hal_error_code)
{
    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    adc_error_flags |= error_flag;
    adc_hal_error_code = hal_error_code;
    __set_PRIMASK(previous_primask);
}

// 启动采集前清除旧结果和错误，防止把上次数据当成本次采样。
static void BSP_ADCResetPublishedSample(void)
{
    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    latest_sample.raw_vcc_in = 0u;
    latest_sample.raw_key = 0u;
    latest_sample.sequence = 0u;
    latest_sample.timestamp_ms = 0u;
    latest_sample.error_flags = BSP_ADC_ERROR_NONE;
    latest_sample.hal_error_code = 0u;
    latest_sample.vcc_in_volts = 0.0f;
    latest_sample.voltage_config_valid = 0u;
    adc_error_flags = BSP_ADC_ERROR_NONE;
    adc_hal_error_code = 0u;
    sample_ready = 0u;
    adc_started = 0u;
    diagnostic_published = 0u;
    // 重启测试或受控重新初始化时，先撤销旧记录，避免工具误读上次启动的数据。
    bsp_adc_diagnostic.version = 0u;
    bsp_adc_diagnostic.layout = 0u;
    bsp_adc_diagnostic.captured_ms = 0u;
    bsp_adc_diagnostic.status = 0u;
    bsp_adc_diagnostic.raw_vcc_in = 0u;
    bsp_adc_diagnostic.raw_key = 0u;
    bsp_adc_diagnostic.sample_sequence = 0u;
    bsp_adc_diagnostic.sample_timestamp_ms = 0u;
    bsp_adc_diagnostic.error_flags = 0u;
    bsp_adc_diagnostic.hal_error_code = 0u;
    bsp_adc_diagnostic.voltage_config_valid = 0u;
    bsp_adc_diagnostic.volts_bits = 0u;
    bsp_adc_diagnostic.vref_mv = 0u;
    bsp_adc_diagnostic.full_scale = 0u;
    bsp_adc_diagnostic.divider_milli = 0u;
    __set_PRIMASK(previous_primask);
}

// 由 DMA 完成回调调用；first_word 只能指向刚采好的前半段或后半段起点。
static void BSP_ADCProcessCompletedHalf(uint32_t first_word)
{
    // 未启动或已报错时不再更新结果，读取接口会返回对应状态。
    if (!adc_started || adc_error_flags != BSP_ADC_ERROR_NONE)
        return;

    uint16_t *half_buffer = &adc_dma_buffer[first_word];
    // DMA 已写入内存，先舍弃 CPU 中这一半的旧缓存，再读取新数据。
    SCB_InvalidateDCache_by_Addr((uint32_t *)half_buffer,
                                 (int32_t)(BSP_ADC_DMA_HALF_WORD_COUNT * sizeof(uint16_t)));

    uint32_t vcc_sum = 0u;
    uint32_t key_sum = 0u;
    // 遍历已完成的 16 对读数：偶数位置是电源，奇数位置是按键；另一半仍由 DMA 写入。
    for (uint32_t pair = 0u; pair < BSP_ADC_PAIRS_PER_HALF; ++pair)
    {
        vcc_sum += half_buffer[pair * 2u];
        key_sum += half_buffer[pair * 2u + 1u];
    }

    // 将两个平均值、序号和采样时间一起更新，防止其他普通中断读到一半新一半旧的数据。
    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    latest_sample.raw_vcc_in = (uint16_t)(vcc_sum / BSP_ADC_PAIRS_PER_HALF);
    latest_sample.raw_key = (uint16_t)(key_sum / BSP_ADC_PAIRS_PER_HALF);
    latest_sample.sequence++;
    latest_sample.timestamp_ms = HAL_GetTick();
    latest_sample.error_flags = adc_error_flags;
    latest_sample.hal_error_code = adc_hal_error_code;
    sample_ready = 1u;
    __set_PRIMASK(previous_primask);
}

void BSP_ADCInit(void)
{
    BSP_ADCResetPublishedSample();

    HAL_StatusTypeDef calibration_status =
        HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    // 校准失败时保留错误，不继续启动采集。
    if (calibration_status != HAL_OK)
    {
        BSP_ADCSetError(BSP_ADC_ERROR_CALIBRATION, (uint32_t)calibration_status);
        LOGERROR("[adc] calibration failed");
        return;
    }

    // DMA 写入前先处理整块旧缓存，避免缓存稍后写回内存时覆盖新采样。
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)adc_dma_buffer,
                                     (int32_t)(BSP_ADC_DMA_WORD_COUNT * sizeof(uint16_t)));

    HAL_StatusTypeDef dma_start_status =
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, BSP_ADC_DMA_WORD_COUNT);
    // 启动失败时保持 adc_started 为 0，读取接口通过错误记录说明原因。
    if (dma_start_status != HAL_OK)
    {
        BSP_ADCSetError(BSP_ADC_ERROR_DMA_START, (uint32_t)dma_start_status);
        LOGERROR("[adc] start DMA failed");
        return;
    }

    adc_started = 1u;
    LOGINFO("[adc] VCC_IN sense started (divider=1/11)");
}

BSP_ADC_Status_e BSP_ADCGetSample(BSP_ADC_Sample_t *sample)
{
    // 没有输出位置时直接报参数错误，不读取或改写采样记录。
    if (sample == NULL)
        return BSP_ADC_STATUS_INVALID_ARGUMENT;

    // 先复制同一批数据和当时的状态，防止 DMA 回调在读取途中更新字段。
    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    sample->raw_vcc_in = latest_sample.raw_vcc_in;
    sample->raw_key = latest_sample.raw_key;
    sample->sequence = latest_sample.sequence;
    sample->timestamp_ms = latest_sample.timestamp_ms;
    sample->error_flags = adc_error_flags;
    sample->hal_error_code = adc_hal_error_code;
    uint8_t started = adc_started;
    uint8_t ready = sample_ready;
    __set_PRIMASK(previous_primask);

    sample->voltage_config_valid = BSP_ADCVoltageConfigValid();
    sample->vcc_in_volts = 0.0f;
    // 配置合适才还原电源电压；两个因子各带 1000，除以 1000000 后得到 V。
    if (sample->voltage_config_valid)
    {
        sample->vcc_in_volts = ((float)sample->raw_vcc_in * (float)BSP_ADC_VREF_MV *
                                (float)BSP_ADC_VCC_IN_DIVIDER_MILLI) /
                               ((float)BSP_ADC_FULL_SCALE * 1000000.0f);
    }

    // 错误优先返回；保留输出中的最近数据供查看原因，不能继续当作有效采样使用。
    if (sample->error_flags != BSP_ADC_ERROR_NONE)
        return BSP_ADC_STATUS_ERROR;
    // 启动成功也要等前半段或后半段首次完成后，才有可用结果。
    if (!started || !ready)
        return BSP_ADC_STATUS_NOT_READY;
    // 用无符号时间差计算过去的毫秒数，tick 从最大值回到 0 时仍能检查 20ms 时限。
    if ((uint32_t)(HAL_GetTick() - sample->timestamp_ms) > BSP_ADC_SAMPLE_MAX_AGE_MS)
        return BSP_ADC_STATUS_STALE;
    return BSP_ADC_STATUS_VALID;
}

void BSP_ADCUpdateDiagnostics(void)
{
    uint32_t now = HAL_GetTick();
    // 用真实毫秒差限制频率，计时回绕后仍可继续记录。
    if (diagnostic_published &&
        (uint32_t)(now - bsp_adc_diagnostic.captured_ms) < BSP_ADC_DIAGNOSTIC_PERIOD_MS)
        return;

    BSP_ADC_Sample_t sample;
    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    uint32_t captured_ms = HAL_GetTick();
    BSP_ADC_Status_e status = BSP_ADCGetSample(&sample);
    uint32_t volts_bits;
    memcpy(&volts_bits, &sample.vcc_in_volts, sizeof(volts_bits));

    // 两个版本写入之间只放本次 getter 的数据；SWD 跨更新读取时必须整份丢弃。
    uint32_t writing_version = bsp_adc_diagnostic.version + 1u;
    bsp_adc_diagnostic.version = writing_version;
    __DMB();
    bsp_adc_diagnostic.layout = BSP_ADC_DIAGNOSTIC_LAYOUT;
    bsp_adc_diagnostic.captured_ms = captured_ms;
    bsp_adc_diagnostic.status = (uint32_t)status;
    bsp_adc_diagnostic.raw_vcc_in = sample.raw_vcc_in;
    bsp_adc_diagnostic.raw_key = sample.raw_key;
    bsp_adc_diagnostic.sample_sequence = sample.sequence;
    bsp_adc_diagnostic.sample_timestamp_ms = sample.timestamp_ms;
    bsp_adc_diagnostic.error_flags = sample.error_flags;
    bsp_adc_diagnostic.hal_error_code = sample.hal_error_code;
    bsp_adc_diagnostic.voltage_config_valid = sample.voltage_config_valid;
    bsp_adc_diagnostic.volts_bits = volts_bits;
    bsp_adc_diagnostic.vref_mv = BSP_ADC_VREF_MV;
    bsp_adc_diagnostic.full_scale = BSP_ADC_FULL_SCALE;
    bsp_adc_diagnostic.divider_milli = BSP_ADC_VCC_IN_DIVIDER_MILLI;
    __DMB();
    // 保留 0 表示未发布；版本回绕时跳过 0，不改变 ADC 的批次序号。
    bsp_adc_diagnostic.version = writing_version == UINT32_MAX ? 2u : writing_version + 1u;
    diagnostic_published = 1u;
    __set_PRIMASK(previous_primask);
}

/**
 * @brief ADC1 前半段完成时处理其 16 对读数，后半段留给 DMA 继续写入。
 * @param[in] hadc HAL 传入的有效 ADC 句柄，其他 ADC 的回调不处理。
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    // 只处理本文件负责的 ADC1。
    if (hadc == &hadc1)
        BSP_ADCProcessCompletedHalf(0u);
}

/**
 * @brief ADC1 后半段完成时处理其 16 对读数，DMA 随后从前半段继续采集。
 * @param[in] hadc HAL 传入的有效 ADC 句柄，其他 ADC 的回调不处理。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // 后半段从第 BSP_ADC_DMA_HALF_WORD_COUNT 项开始，前半段已交回 DMA。
    if (hadc == &hadc1)
        BSP_ADCProcessCompletedHalf(BSP_ADC_DMA_HALF_WORD_COUNT);
}

/**
 * @brief 保存 ADC1 采集错误，之后读取采样时报告 ERROR。
 * @param[in] hadc HAL 传入的有效 ADC 句柄，从其 ErrorCode 读取错误码。
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    // 其他 ADC 的错误不写入 ADC1 的记录。
    if (hadc != &hadc1)
        return;

    BSP_ADCSetError(BSP_ADC_ERROR_RUNTIME, hadc->ErrorCode);
}
