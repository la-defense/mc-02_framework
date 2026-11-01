/** @file dsp_matrix.h
 * @brief 声明不暴露厂商类型的浮点矩阵运算；当前后端使用 CMSIS-DSP。
 * @note 数据按行排列，由调用者保存；各运算不申请内存，允许的重叠见对应入口。
 */
#ifndef MC02_DSP_MATRIX_H
#define MC02_DSP_MATRIX_H
#include <stddef.h>
#include <stdint.h>

/** @brief 矩阵尺寸和数据地址；调用者保证地址指向至少 numRows*numCols 个 float。 */
typedef struct {
    uint16_t numRows; /**< 行数。 */
    uint16_t numCols; /**< 列数。 */
    float *pData; /**< 按行排列的数据，运算期间保持有效。 */
} DSPMatrixF32_t;

/** @brief 矩阵操作结果，不暴露所选厂商的错误类型。 */
typedef enum {
    DSP_MATRIX_OK = 0, /**< 运算成功。 */
    DSP_MATRIX_ARGUMENT = -1, /**< 空指针、对齐、重叠或工作区错误。 */
    DSP_MATRIX_LENGTH = -2, /**< 后端报告长度错误。 */
    DSP_MATRIX_SIZE = -3, /**< 输入或输出尺寸不匹配。 */
    DSP_MATRIX_NONFINITE = -4, /**< 输入或结果含非有限值。 */
    DSP_MATRIX_SINGULAR = -5, /**< 方阵不可逆。 */
    DSP_MATRIX_BACKEND = -6 /**< 后端返回其他错误。 */
} DSPMatrixStatus_t;

/** @brief 给描述符绑定尺寸和已有数据，不申请或清除数据。
 * @param[out] matrix 非空描述符。
 * @param[in] rows 行数。
 * @param[in] columns 列数。
 * @param[in,out] data 数据地址；零尺寸时允许为空。
 */
void DSPMatrixInitF32(DSPMatrixF32_t *matrix, uint16_t rows, uint16_t columns, float *data);

/** @brief 两个同形矩阵逐项相加，输出可与任一输入共用数据。
 * @param[in] a,b 输入矩阵，描述符和数据非空。
 * @param[out] output 与输入同形的输出。
 * @return 运算状态；尺寸检查由后端执行。
 */
DSPMatrixStatus_t DSPMatrixAddF32(const DSPMatrixF32_t *a, const DSPMatrixF32_t *b, DSPMatrixF32_t *output);

/** @brief 两个同形矩阵逐项相减，输出可与任一输入共用数据。
 * @param[in] a,b 输入矩阵，描述符和数据非空。
 * @param[out] output 与输入同形的输出。
 * @return 运算状态；尺寸检查由后端执行。
 */
DSPMatrixStatus_t DSPMatrixSubF32(const DSPMatrixF32_t *a, const DSPMatrixF32_t *b, DSPMatrixF32_t *output);

/** @brief 计算矩阵乘积。
 * @param[in] a,b 输入矩阵，描述符和数据非空。
 * @param[out] output 尺寸为 a 的行数乘以 b 的列数，不得与输入数据重叠。
 * @return 运算状态；调用方负责保证缓冲容量和不重叠。
 */
DSPMatrixStatus_t DSPMatrixMultiplyF32(const DSPMatrixF32_t *a, const DSPMatrixF32_t *b, DSPMatrixF32_t *output);

/** @brief 转置矩阵。
 * @param[in] input 输入矩阵，描述符和数据非空。
 * @param[out] output 行列与输入相反，不得与输入数据重叠。
 * @return 运算状态；调用方负责保证缓冲容量和不重叠。
 */
DSPMatrixStatus_t DSPMatrixTransposeF32(const DSPMatrixF32_t *input, DSPMatrixF32_t *output);

/** @brief 求逆成功且结果有限后才写输出。
 * @param[in] input 非空方阵，行列范围1～255，pData至少可读 n*n 个 float。
 * @param[out] output 同形状方阵，pData至少可写 n*n 个 float，失败保持描述符和数据。
 * @param[in,out] workspace 至少 workspace_floats 个 float，按 float 对齐，内容可被改写。
 * @param[in] workspace_floats float 元素容量，须不小于2*n*n。
 * @return DSP_MATRIX_OK 或参数、尺寸、奇异、数值及后端错误。
 * @note 输入、输出和工作区不得重叠；后端可能改写工作区，不改变原输入。
 */
DSPMatrixStatus_t DSPMatrixInverseF32(const DSPMatrixF32_t *input,
    DSPMatrixF32_t *output, float *workspace, size_t workspace_floats);
#endif
