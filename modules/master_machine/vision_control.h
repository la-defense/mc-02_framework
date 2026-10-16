/**
 * @file vision_control.h
 * @brief 视觉目标校验、云台单位转换及视觉／手动发射命令选择。
 *
 * RobotCMDTask 提供本周期输入快照，通过下列接口生成云台与发射命令。
 * 本模块只做计算，不读取硬件或操作电机；最终状态许可由调用方检查。
 * 输入与输出对象须互不重叠，输入及其引用的配置在调用期间须保持有效且不变。
 */
#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include <stdint.h>

/* 偏航约束作用于转换后的云台命令，不对目标角做归一化或截断。 */
typedef enum
{
    VISION_YAW_UNCONFIGURED = 0, /* 未指定约束，拒绝视觉接管。 */
    VISION_YAW_CONTINUOUS,       /* 连续旋转，不检查偏航角是否落在上下限内。 */
    VISION_YAW_BOUNDED          /* 有限行程，偏航角须落在闭区间内。 */
} VisionYawConstraint_e;

/**
 * @brief 最终云台命令坐标系中的机械限位，角度单位均为 deg。
 * @note 所有浮点配置均须有限；pitch_min_deg < pitch_max_deg。
 *       有限偏航还要求 yaw_min_deg < yaw_max_deg；连续偏航的上下限仍须为有限值。
 */
typedef struct
{
    uint8_t configured;                  /* 非零表示调用方已提供机械配置。 */
    VisionYawConstraint_e yaw_constraint; /* 必须明确选择连续旋转或有限行程。 */
    float pitch_min_deg;
    float pitch_max_deg;
    float pitch_zero_offset_deg; /* 输入 pitch 转为 deg 后加此零偏，再检查限位。 */
    float yaw_min_deg;
    float yaw_max_deg;
} VisionControlLimits_s;

/* 视觉输入允许配置的最大年龄；调用方可设更短期限。 */
#define VISION_CONTROL_MAX_AGE_MS 50u

/** @brief 本周期视觉输入快照；角度、角速度、角加速度沿用 SP 的 rad、rad/s、rad/s^2。 */
typedef struct
{
    uint8_t online;       /* 非零表示通信在线，仍须通过模式、年龄和数值检查。 */
    uint8_t mode;         /* 1=瞄准并预热，2=瞄准并连续供弹；其余模式拒绝接管。 */
    uint32_t last_rx_ms;  /* 最近合法帧的接收时刻，与 now_ms 使用同一毫秒计数源。 */
    uint32_t now_ms;      /* 调用方固定的本周期时刻；按无符号时间差处理计数回绕。 */
    uint32_t max_age_ms;  /* 有效范围 0～50 ms，年龄等于期限时仍接受。 */
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    const VisionControlLimits_s *limits; /* 借用的只读配置，调用期间须有效；不在模块内保存。 */
} VisionControlInput_s;

/** @brief 转换后的云台目标及前馈，供调用方写入本周期控制命令。 */
typedef struct
{
    float yaw, pitch;                            /* 目标角，deg；pitch 已包含配置零偏。 */
    float yaw_vel, yaw_acc, pitch_vel, pitch_acc; /* vel 为 deg/s，acc 为 deg/s^2。 */
} VisionControlGimbalCommand_s;

/** @brief 一份完整的发射命令请求；实际执行还须经过调用方的安全门禁。 */
typedef struct
{
    uint8_t shoot_enabled;    /* 发射电机组总使能请求，1=开启，0=关闭。 */
    uint8_t friction_enabled; /* 摩擦轮使能请求，1=开启，0=关闭。 */
    /* 与 loader_mode_e 数值一致，用 uint8_t 避免依赖应用层类型。 */
    uint8_t load_mode;
    float shoot_rate; /* 连续供弹目标射频，单位：发/秒。 */
} VisionControlShootCommand_s;

/* 供弹模式数值由 robot_cmd.c 的静态断言与应用层 loader_mode_e 核对。 */
enum
{
    VISION_CONTROL_LOAD_STOP = 0u,
    VISION_CONTROL_LOAD_REVERSE = 1u,
    VISION_CONTROL_LOAD_SINGLE_BULLET = 2u,
    VISION_CONTROL_LOAD_THREE_BULLETS = 3u,
    VISION_CONTROL_LOAD_BURSTFIRE = 4u
};

/**
 * @brief 判断视觉输入能否生成合法云台命令，检查规则与 VisionControlBuildGimbalCommand() 相同。
 * @param[in] input 本周期输入；允许 NULL，此时返回 0。
 * @return 1=模式、在线状态、年龄、配置和转换后的数值均有效；0=拒绝接管。
 */
uint8_t VisionControlInputIsUsable(const VisionControlInput_s *input);

/**
 * @brief 校验视觉输入，将目标及前馈从弧度制转为角度制，并检查最终角度限位。
 * @param[in] input 本周期输入；NULL、缺配置、离线、过期或非法模式均被拒绝。
 * @param[out] command 云台命令；允许 NULL，此时返回 0 且不写入。
 * @return 1=生成有效命令；0=拒绝，非 NULL 的 command 所有字段均清零。
 * @note 六个输入数值及转换结果须有限。pitch 加零偏后检查闭区间限位，
 *       yaw 按已选约束检查；超限直接拒绝，不截断。输入内容保持不变。
 */
uint8_t VisionControlBuildGimbalCommand(const VisionControlInput_s *input,
                                      VisionControlGimbalCommand_s *command);

/**
 * @brief 按有效视觉模式生成完整发射请求，输入有效性与云台命令使用同一判据。
 * @param[in] input 本周期视觉输入；允许 NULL，此时拒绝。
 * @param[out] command 发射请求；允许 NULL，此时返回 0 且不写入。
 * @return 1=选用视觉策略；0=拒绝，非 NULL 的 command 关闭总使能与摩擦轮，
 *         停止供弹并清零射频。
 * @note mode 1 开启总使能和摩擦轮、停止供弹且射频为 0；mode 2 开启两者，
 *       连续供弹射频为 10 发/秒。请求不代表已获得实际击发许可。
 */
uint8_t VisionControlBuildShootCommand(const VisionControlInput_s *input,
                                       VisionControlShootCommand_s *command);

/**
 * @brief 优先使用有效视觉发射请求；视觉不可用时回退到调用方提供的手动命令。
 * @param[in] input 本周期视觉输入，允许 NULL。
 * @param[in] manual_command 已由调用方确认来源有效的本周期手动命令；NULL 表示无手动源。
 * @param[out] command 选中的完整命令，允许 NULL；不得与 manual_command 指向同一对象。
 * @return 1=选中视觉；0=回退手动、无来源或 command 为 NULL，不表示输出一定关闭。
 * @note 手动命令按原值复制，不在此校验其来源或数值；无视觉且无手动源时输出
 *       关闭并清零的命令。command 为 NULL 时不写入。
 */
uint8_t VisionControlSelectShootCommand(const VisionControlInput_s *input,
                                        const VisionControlShootCommand_s *manual_command,
                                        VisionControlShootCommand_s *command);

/**
 * @brief 将遥控拨轮值映射为完整手动发射请求，射频固定为 8 发/秒。
 * @param[in] dial 遥控通道解码后的有符号拨轮原始值；来源有效性由调用方确认。
 * @param[out] command 发射请求；NULL 时直接返回。
 * @note dial < -100 开启摩擦轮，dial < -500 连续供弹，否则停止供弹；
 *       摩擦轮或供弹任一有请求即开启总使能。此处不检查遥控在线状态。
 */
void VisionControlBuildRemoteShootCommand(int16_t dial,
                                          VisionControlShootCommand_s *command);

/**
 * @brief 将键鼠档位计数映射为完整手动发射请求，射频固定为 8 发/秒。
 * @param[in] load_cycle E 键按下边沿计数；模 4 对应停止、单发、三发、连续供弹。
 * @param[in] friction_cycle F 键按下边沿计数；奇数开启摩擦轮，偶数关闭。
 * @param[out] command 发射请求；NULL 时直接返回。
 * @note 调用方负责按键去重、计数及来源有效性；摩擦轮或供弹任一有请求即开启总使能。
 */
void VisionControlBuildMouseKeyShootCommand(uint8_t load_cycle,
                                            uint8_t friction_cycle,
                                            VisionControlShootCommand_s *command);

#endif
