/** @file crc.h
 * @brief 纯 C 软件 CRC：整块计算或由调用者保存上下文后分段计算。
 * @note 不依赖硬件和初始化任务；不同上下文可并发，同一上下文须串行使用。
 */
#ifndef MC02_CRC_H
#define MC02_CRC_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
    CRC_OK = 0,             ///< 已完成计算；结果可以为0。
    CRC_ERROR_ARGUMENT,     ///< 空地址与长度不匹配，或没有提供输出地址。
    CRC_ERROR_ALGORITHM,    ///< 算法编号不在五种固定配置内。
    CRC_ERROR_STATE         ///< 上下文未初始化或余数超出该算法宽度。
} CRCStatus_t;

/** @brief 固定算法；完整参数见 CRCConfigurations，名称同时说明主要使用者。 */
typedef enum {
    CRC8_CAN_A5 = 0,
    CRC8_REFEREE,
    CRC16_MODBUS_A5,
    CRC16_SP_REFEREE,
    CRC32_IEEE,
    CRC_ALGORITHM_COUNT
} CRCAlgorithm_t;

/** @brief 多项式使用正常表示（省略最高位）；反射标志为0或1。 */
typedef struct {
    uint8_t width;
    uint8_t reflect_input;
    uint8_t reflect_output;
    uint32_t polynomial;
    uint32_t initial;
    uint32_t xor_output;
} CRCConfiguration_t;

extern const CRCConfiguration_t CRCConfigurations[CRC_ALGORITHM_COUNT];

/** @brief 调用者保存的历史数据；先置零，再调用 CRCInit，禁止直接改写成员。 */
typedef struct {
    CRCAlgorithm_t algorithm;
    uint32_t remainder;
    uint32_t initialized;
} CRCContext_t;

/** @brief 接受算法并重新开始计算。
 * @param[out] context 可写上下文，首次使用前置零；失败保持原内容。
 * @param[in] algorithm CRC8_CAN_A5～CRC32_IEEE 中的一项。
 * @return CRC_OK，或空参数／非法算法错误。
 */
CRCStatus_t CRCInit(CRCContext_t *context, CRCAlgorithm_t algorithm);

/** @brief 把一段字节加入已初始化的计算。
 * @param[in,out] context CRCInit 成功的上下文，不能与数据区重叠。
 * @param[in] data 至少可读 length 字节；仅 length 为0时允许 NULL。
 * @param[in] length 字节数；调用者负责实际容量，计算次数与长度成正比。
 * @return CRC_OK 或参数／算法／状态错误；失败保持上下文原值。
 */
CRCStatus_t CRCUpdate(CRCContext_t *context, const uint8_t *data, size_t length);

/** @brief 读取最终数值，不消耗上下文，之后仍可追加数据。
 * @param[in] context 已初始化的上下文。
 * @param[out] output 接收低 width 位结果，不得与上下文重叠；失败保持原值。
 * @return CRC_OK 或参数／算法／状态错误。
 */
CRCStatus_t CRCFinal(const CRCContext_t *context, uint32_t *output);

/** @brief 一次计算完整数据，无需先初始化。
 * @param[in] algorithm 固定算法编号。
 * @param[in] data 至少可读 length 字节；仅 length 为0时允许 NULL。
 * @param[in] length 字节数，调用者负责实际可读容量。
 * @param[out] output 接收低 width 位结果；失败保持原值。
 * @return CRC_OK 或参数／算法错误；有效结果为0也返回 CRC_OK。
 */
CRCStatus_t CRCCompute(CRCAlgorithm_t algorithm, const uint8_t *data,
                       size_t length, uint32_t *output);

#endif
