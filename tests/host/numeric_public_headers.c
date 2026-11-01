/** @file numeric_public_headers.c
 * @brief 验证数值公共头只凭本工程头路径和标准 C 头就能一起编译。
 */
#include "dsp_matrix.h" // 矩阵接口及不依赖厂商的描述符。
#include "fast_trig.h" // 弧度输入的快速三角函数。
#include "kalman_filter.h" // 通用卡尔曼滤波器。
#include "QuaternionEKF.h" // 四元数姿态滤波器。
#include "attitude_filter.h" // 姿态后端选择及输出。
#include "RLS.h" // 递推最小二乘接口。

/** @brief 让编译器实际读取公开矩阵和滤波状态类型。
 * @param[in] matrix 可为空的矩阵描述符。
 * @return 非空时返回行数，否则返回零。
 */
int NumericPublicHeadersProbe(const DSPMatrixF32_t *matrix)
{
    return matrix != NULL ? (int)matrix->numRows : 0;
}
