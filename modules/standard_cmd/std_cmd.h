/** @file std_cmd.h
 * @brief 将已通过协议检查的人工输入换成统一运动意图；本模块不授予运行许可。
 */
#ifndef STD_CMD_H
#define STD_CMD_H

#include <stdint.h>

/** @brief 人工连续输入来源；各来源保留自己的接收时间和序号。 */
typedef enum {
    STD_CMD_REMOTE = 0, // 遥控摇杆。
    STD_CMD_KEY_MOUSE, // 有独立使用期限的键鼠。
    STD_CMD_MANUAL_SOURCE_COUNT
} StdCmdManualSource_t;

/** @brief 平移方向参照；速度均作用于底盘中心。 */
typedef enum {
    STD_CMD_FRAME_CHASSIS = 0, // 底盘前向。
    STD_CMD_FRAME_BIG_YAW, // 大 yaw 前向。
    STD_CMD_FRAME_MUZZLE, // 枪口前向。
    STD_CMD_FRAME_FIELD, // 已建立定位参考的场地前向。
    STD_CMD_FRAME_COUNT
} StdCmdFrame_t;

/** @brief 通道顺序；前左上坐标，角速度遵循右手方向。 */
typedef enum {
    STD_CMD_VX = 0, // 底盘前进速度，m/s。
    STD_CMD_VY, // 底盘左移速度，m/s。
    STD_CMD_WZ, // 底盘绕上方轴的角速度，rad/s。
    STD_CMD_YAW_RATE, // 枪口整体 yaw 角速度，rad/s。
    STD_CMD_PITCH_RATE, // 枪口 pitch 角速度，rad/s。
    STD_CMD_AXIS_COUNT
} StdCmdAxis_t;

typedef enum {
    STD_CMD_OK = 0,
    STD_CMD_BAD_ARGUMENT,
    STD_CMD_BAD_CONFIG,
    STD_CMD_INVALID_INPUT,
    STD_CMD_EXPIRED,
    STD_CMD_DUPLICATE
} StdCmdStatus_t;

/** @brief 一个通道的线性换算；死区／曲线由来源模块处理。 */
typedef struct {
    float input_limit; // 原始值绝对值上限，必须为正且有限；不能借限幅接受非法输入。
    float gain; // 原始单位到该通道 SI 单位的有符号比例，有限；0表示该通道不作用。
    float output_limit; // 换算后绝对值上限，有限且非负；0使该通道输出为0。
} StdCmdAxisMap_t;

typedef struct {
    StdCmdAxisMap_t axis[STD_CMD_AXIS_COUNT]; // 按上述五个通道逐项配置。
    uint32_t max_age_ms; // 本模块允许的最大来源年龄，1～INT32_MAX ms。
    uint32_t allowed_frames; // 对应1u<<StdCmdFrame_t的允许位，不允许未知位或全零。
} StdCmdManualMap_t;

typedef struct {
    float axis[STD_CMD_AXIS_COUNT]; // 已合法解码、去死区后的原始通道值。
    uint32_t source_time_ms; // 原始来源时刻，须已转换为本机毫秒；重复读取不改写。
    uint32_t sequence; // 来源序号，0保留为未收到；重复读取保留原序号。
    uint32_t max_age_ms; // 来源自身允许的年龄，1～INT32_MAX ms。
    StdCmdManualSource_t source; // 摇杆或键鼠，不能混用两者在线条件。
    StdCmdFrame_t frame; // 本次平移方向采用的参照。
    uint8_t valid; // 只有协议和来源条件均有效时置1。
} StdCmdManualInput_t;

typedef struct {
    float axis[STD_CMD_AXIS_COUNT]; // m/s、rad/s；无电机驱动原始单位。
    uint32_t source_time_ms; // 继承输入原时刻，换算不会延长有效时间。
    uint32_t sequence; // 继承输入序号，供后续控制接管检查。
    uint32_t max_age_ms; // 输入期限与配置期限中的较小值。
    StdCmdManualSource_t source; // 保存来源，供robot_cmd仲裁。
    StdCmdFrame_t frame; // 保存方向参照，实际转换由运动模块执行。
} StdCmdManualIntent_t;

/** @brief 按配置换算一份合法人工输入；不使能、不选择模式、不控制电机。
 * @param[in] map 本车型的人工映射，所有轴必须完整配置。
 * @param[in] input 当前来源数据，来源时刻须属于本机时钟。
 * @param[in] now_ms 当前本机毫秒；允许32位自然回绕，等于期限仍有效。
 * @param[out] output 完整换算成功才改写；须与输入及配置分开存储。
 * @return 具体失败原因；任何失败均保留原output，调用者须撤销该来源而非继续使用旧输出。
 */
StdCmdStatus_t StdCmdAdaptManual(const StdCmdManualMap_t *map,
                                const StdCmdManualInput_t *input,
                                uint32_t now_ms, StdCmdManualIntent_t *output);

/** @brief 独立编号的按次事件；不从连续快照的重复读取中推导事件。 */
typedef struct {
    uint32_t sequence; // 本来源事件序号；0无效，向前范围小于半个32位计数区间。
    uint32_t source_time_ms; // 事件发生的本机毫秒时刻，转发不重写。
    uint32_t max_age_ms; // 事件有效时间，1～INT32_MAX ms。
    StdCmdManualSource_t source; // 来源，各来源序号分别消费。
} StdCmdEvent_t;

typedef struct {
    uint32_t sequence[STD_CMD_MANUAL_SOURCE_COUNT]; // 各来源上次接受的事件号，调用者初始清零。
} StdCmdEventCursor_t;

/** @brief 在消费事件时检查期限和序号；调用者在返回OK后执行该事件。
 * @param[in,out] cursor 单一消费者持有的记录；失败不改写，不可并发访问。
 * @param[in] event 本次事件；实际动作类型和授权由调用模块管理。
 * @param[in] now_ms 本机毫秒，等于期限允许。
 * @return OK只表示本次可消费，不等于已取得执行器许可；旧序号和重复事件返回DUPLICATE。
 */
StdCmdStatus_t StdCmdConsumeEvent(StdCmdEventCursor_t *cursor,
                                 const StdCmdEvent_t *event, uint32_t now_ms);

#endif
