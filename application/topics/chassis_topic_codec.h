/** @file chassis_topic_codec.h
 * @brief 双板底盘话题的固定小端字段编码；不使用C结构体的内存布局。
 * @note 两端须使用相同协议版本；物理范围和控制许可仍由业务检查。
 */
#ifndef CHASSIS_TOPIC_CODEC_H
#define CHASSIS_TOPIC_CODEC_H
#include "robot_def.h"
#include <stddef.h>
#include <stdint.h>

#define CHASSIS_COMMAND_WIRE_BYTES 21u
#define CHASSIS_FEEDBACK_WIRE_BYTES 3u
typedef enum { CHASSIS_CODEC_OK=0, CHASSIS_CODEC_ARGUMENT, CHASSIS_CODEC_VALUE } ChassisCodecStatus_t;

/** @brief 编码底盘命令的四个IEEE754单精度数、模式和32位速度档。
 * @param[in] input 可读命令；速度单位mm/s，角度和角速度单位deg、deg/s，浮点必须有限。
 * @param[out] output 至少可写capacity字节；失败保持原值。
 * @param[in] capacity 字节容量，至少CHASSIS_COMMAND_WIRE_BYTES。
 * @return 成功、参数或数据错误；模式只允许0～3。
 */
ChassisCodecStatus_t ChassisCommandEncode(const Chassis_Ctrl_Cmd_s *input, uint8_t *output, size_t capacity);
/** @brief 解码固定长度底盘命令；参数和数值规则同ChassisCommandEncode。
 * @param[in] input 可读的线上字节；NULL失败。
 * @param[in] length 必须等于CHASSIS_COMMAND_WIRE_BYTES。
 * @param[out] output 接收完整命令；失败保持原值。
 * @return 成功、参数或数据错误。
 */
ChassisCodecStatus_t ChassisCommandDecode(const uint8_t *input, size_t length, Chassis_Ctrl_Cmd_s *output);
/** @brief 编码剩余热量、弹速枚举和敌方颜色；非允许枚举值拒绝。
 * @param[in] input 可读反馈。
 * @param[out] output 至少可写capacity字节；失败保持原值。
 * @param[in] capacity 字节容量，至少CHASSIS_FEEDBACK_WIRE_BYTES。
 * @return 成功、参数或数据错误。
 */
ChassisCodecStatus_t ChassisFeedbackEncode(const Chassis_Upload_Data_s *input, uint8_t *output, size_t capacity);
/** @brief 解码固定长度的底盘反馈。
 * @param[in] input 可读的线上字节；NULL失败。
 * @param[in] length 必须等于CHASSIS_FEEDBACK_WIRE_BYTES。
 * @param[out] output 接收完整反馈；失败保持原值。
 * @return 成功、参数或数据错误。
 */
ChassisCodecStatus_t ChassisFeedbackDecode(const uint8_t *input, size_t length, Chassis_Upload_Data_s *output);

/** @brief 供话题桥接使用的命令编码回调；输入可能未对齐，失败返回0。
 * @param[in] context 未使用，传NULL。
 * @param[in] input,input_size 命令字节及其精确大小。
 * @param[out] output 线上字节；失败保持原值。
 * @param[in] output_size 必须为CHASSIS_COMMAND_WIRE_BYTES。
 * @return 成功1，失败0。
 */
uint8_t ChassisCommandBridgeEncode(void *context,const void *input,size_t input_size,void *output,size_t output_size);
/** @brief 供桥接使用的命令解码回调；参数与编码回调相同但输入输出方向相反。
 * @param[in] context 未使用，传NULL。
 * @param[in] input,input_size 线上字节及其精确大小。
 * @param[out] output 命令字节；失败保持原值。
 * @param[in] output_size 必须为命令结构体大小。
 * @return 成功1，失败0。
 */
uint8_t ChassisCommandBridgeDecode(void *context,const void *input,size_t input_size,void *output,size_t output_size);
/** @brief 桥接反馈编码，参数与命令编码相同但使用反馈大小。
 * @param[in] context 未使用，传NULL。
 * @param[in] input,input_size 反馈字节及其精确大小。
 * @param[out] output 线上字节；失败保持原值。
 * @param[in] output_size 必须为CHASSIS_FEEDBACK_WIRE_BYTES。
 * @return 成功1，失败0。
 */
uint8_t ChassisFeedbackBridgeEncode(void *context,const void *input,size_t input_size,void *output,size_t output_size);
/** @brief 桥接反馈解码，参数与命令解码相同但使用反馈大小。
 * @param[in] context 未使用，传NULL。
 * @param[in] input,input_size 线上字节及其精确大小。
 * @param[out] output 反馈字节；失败保持原值。
 * @param[in] output_size 必须为反馈结构体大小。
 * @return 成功1，失败0。
 */
uint8_t ChassisFeedbackBridgeDecode(void *context,const void *input,size_t input_size,void *output,size_t output_size);
#endif
