/** @file kalman_filter.h
 * @brief 调用者保存实例和工作区的KF；先查询容量、初始化、填写模型，再更新及复制结果。
 * @author Wang Hongxi（原实现）；本轮保留计算公式，替换存储和结果处理。
 * @note 无硬件、RTOS或分配器依赖；同一实例由调用者串行访问。
 */
#ifndef MC02_KALMAN_FILTER_H
#define MC02_KALMAN_FILTER_H
#include "arm_math.h"
#include <stddef.h>
#include <stdint.h>
typedef enum {
    KF_OK=0, KF_PREDICTED=1, KF_ERROR_ARGUMENT=-1, KF_ERROR_CONFIG=-2,
    KF_ERROR_WORKSPACE=-3, KF_ERROR_STATE=-4, KF_ERROR_TIME=-5, KF_ERROR_NUMERIC=-6,
    KF_ERROR_MATRIX=-7, KF_CORRECTION_REJECTED=-8, KF_NO_MEASUREMENT=-9,
    KF_STALE_MEASUREMENT=-10, KF_DUPLICATE_MEASUREMENT=-11, KF_PREDICTION_LIMIT=-12
} KFStatus_t;
typedef enum { KF_INVALID=0, KF_CORRECTED=1, KF_PREDICTION=2 } KFResultMode_t;
/** @brief valid为0时，复制出的旧数值仅供诊断；预测不更新measurement_ms。 */
typedef struct {
    KFResultMode_t mode;
    KFStatus_t cause;
    uint8_t valid, has_measurement;
    uint32_t estimate_ms, measurement_ms, prediction_age_ms;
} KFResult_t;
struct kf_t;
/** @brief 扩展步骤只操作本轮候选数组，不发布结果、不访问硬件；非KF_OK停止该阶段。
 * after_state_prediction可修改xhatminus、F和候选P；prepare_measurement只写H/R；
 * correction只写候选xhat、K及临时数组，不改写预测xhatminus/Pminus。
 */
typedef KFStatus_t (*KFStageFunction_t)(struct kf_t *candidate, void *context);
/** @brief n/m为1～255，u为0～255；时限为毫秒，dt为秒，方差比无单位。 */
typedef struct {
    uint16_t state_count, control_count, measurement_capacity;
    uint32_t max_prediction_ms, max_measurement_age_ms;
    float max_prediction_variance_ratio, max_dt_s;
    KFStageFunction_t after_state_prediction, prepare_measurement, correction;
} KFConfig_t;
/** @brief 本轮量测含合法零值；存在数组为NULL时前measurement_count项全部存在。 */
typedef struct {
    float dt_s;
    uint32_t now_ms, measurement_ms, measurement_sequence;
    const float *measurements;
    const uint8_t *measurement_present;
    uint16_t measurement_count;
    void *callback_context; // 只在本次调用使用，不保存该地址。
} KFStep_t;
#define KF_WORKSPACE_ALIGNMENT _Alignof(max_align_t)
#define KF_MAX_DIM(n,m) ((size_t)(n)>(size_t)(m)?(size_t)(n):(size_t)(m))
// 与绑定顺序共用的容量公式：float数组在前，量测行索引在后，最后补齐地址要求。
#define KF_WORKSPACE_FLOATS(n,u,m) (7u*(size_t)(n)*(n)+4u*(size_t)(n)*(m)+5u*(size_t)(m)*(m)+ \
    (size_t)(n)*(u)+6u*(size_t)(n)+2u*(size_t)(u)+2u*(size_t)(m)+ \
    2u*KF_MAX_DIM(n,m)*KF_MAX_DIM(n,m)+2u*KF_MAX_DIM(n,m))
#define KF_WORKSPACE_BYTES(n,u,m) (((KF_WORKSPACE_FLOATS(n,u,m)*sizeof(float)+(size_t)(m)+ \
    KF_WORKSPACE_ALIGNMENT-1u)/KF_WORKSPACE_ALIGNMENT)*KF_WORKSPACE_ALIGNMENT)
/** @brief F/Q为n*n，B为n*u，H为m*n，R为m*m，由调用者填写模型；其他成员由接口维护。
 * 禁止改写指针和容量。回调中xhat_data/P_data指候选数据，xhatminus/Pminus指本轮预测。
 */
typedef struct kf_t {
    KFConfig_t config;
    KFResult_t result;
    uint32_t initialized,last_step_ms,last_seen_ms,last_seen_sequence;
    uint8_t has_time,has_seen,fatal;
    uint16_t xhatSize,uSize,zSize;
    void *workspace;
    size_t workspace_bytes;
    float *xhat_data,*P_data,*trial_x,*trial_P,*xhatminus_data,*Pminus_data;
    float *FilteredValue,*published_P,*corrected_variance;
    float *F_data,*FT_data,*Q_data,*B_data,*H_data,*HT_data,*K_data,*R_data,*S_data;
    float *selected_H,*selected_R,*temp_matrix_data,*temp_matrix_data1;
    float *temp_vector_data,*temp_vector_data1,*MeasuredVector,*z_data;
    float *ControlVector,*u_data,*StateMinVariance,*inverse_workspace;
    uint8_t *selected_rows;
    arm_matrix_instance_f32 xhat,P,xhatminus,Pminus,F,FT,Q,B,H,HT,K,R,S;
    arm_matrix_instance_f32 temp_matrix,temp_matrix1,temp_vector,temp_vector1,z,u;
} KalmanFilter_t;
/** @brief 查询存储要求，不分配内存。
 * @param[in] config 合法配置，方差比有限且>=1，max_dt_s有限且>0，时限不超过INT32_MAX。
 * @param[out] bytes 接收字节数，失败保持原值。
 * @param[out] alignment 接收地址对齐要求，失败保持原值。
 * @return KF_OK或参数／配置错误。
 */
KFStatus_t KFGetWorkspaceRequirements(const KFConfig_t *config,size_t *bytes,size_t *alignment);
/** @brief 绑定长期工作区并接受初始状态；成功后先填写模型再更新。
 * @param[out] filter 实例地址，失败保持原值。
 * @param[in] config 合法配置，成功复制。
 * @param[in,out] workspace 满足查询容量和对齐的存储，不能与实例、配置或初始数组重叠。
 * @param[in] bytes 可写字节数，成功后工作区由该实例独占。
 * @param[in] state n个有限float，成功复制，后续可释放。
 * @param[in] covariance n*n个有限float，对角非负，成功复制。
 * @return KF_OK或具体错误；首次校正前不发布预测。
 */
KFStatus_t KFInit(KalmanFilter_t *filter,const KFConfig_t *config,void *workspace,
    size_t bytes,const float *state,const float *covariance);
/** @brief 接受新起点，清除时间、校正及致命失败历史，保留模型与工作区。
 * @param[in,out] filter 已初始化实例。
 * @param[in] state n个有限float，不与工作区重叠。
 * @param[in] covariance n*n个有限float且对角非负，不与工作区重叠。
 * @return KF_OK或具体错误；失败保持旧数值，成功后等待首次校正。
 */
KFStatus_t KFReset(KalmanFilter_t *filter,const float *state,const float *covariance);
/** @brief 预测后尝试校正，完整数值通过检查后才发布。
 * @param[in,out] filter 已初始化且模型已填写的实例，禁止并发访问同一实例。
 * @param[in] step 实测dt及同一本机时钟的时间、源序号、量测和存在标记；数组至少count项。
 * @return KF_OK为校正，KF_PREDICTED为有效预测，负数为无效，原因同时写入result。
 * @note 无效时旧输出保持，valid清零；超限可用新校正恢复，预测失败须重置。
 * 时间及序号间隔小于2^31，允许回绕及同毫秒不同序号；存在标记取0或1。
 * step本身不能与实例或工作区重叠；输入数组只在调用期间使用。
 * F/Q等离散模型由调用方按dt填写；核心不替调用方选择运动模型。
 */
KFStatus_t KFUpdate(KalmanFilter_t *filter,const KFStep_t *step);
/** @brief 复制旧有效数值及当前有效标记，所有输出不得相互重叠或与工作区重叠。
 * @param[in] filter 已初始化实例，复制期间不能更新。
 * @param[out] state 至少state_capacity项的可写数组。
 * @param[in] state_capacity float容量，须>=n。
 * @param[out] covariance 至少covariance_capacity项的可写数组。
 * @param[in] covariance_capacity float容量，须>=n*n。
 * @param[out] result 接收模式、时间、有效性和失败原因。
 * @return KF_OK表示复制成功，不表示result.valid为1；失败所有输出保持原值。
 */
KFStatus_t KFGetState(const KalmanFilter_t *filter,float *state,size_t state_capacity,
    float *covariance,size_t covariance_capacity,KFResult_t *result);
#endif
