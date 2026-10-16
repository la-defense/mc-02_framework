/**
 * @file vision_control.h
 * @brief 检查视觉数据，计算云台目标，并选择视觉或手动发射命令。
 *
 * RobotCMDTask 提供视觉数据，取回算好的命令；实际动作前还要检查当前是否允许。
 * 输入、角度设置和输出要分开存放，计算期间不能改写输入和设置。
 */
#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include <stdint.h>

/* 选择云台左右转动（yaw）的方式，角度范围在换算成度以后检查。 */
typedef enum
{
    VISION_YAW_UNCONFIGURED = 0, /* 未设置，不使用视觉目标。 */
    VISION_YAW_CONTINUOUS,       /* 连续转动，yaw 不受上下限限制。 */
    VISION_YAW_BOUNDED          /* 在上下限之间转动，包含上下限。 */
} VisionYawConstraint_e;

/**
 * @brief 云台目标角度的允许范围，单位：度（deg）；yaw 为左右转动，pitch 为上下转动。
 * @note 各角度不能是 NaN（无效数值）或 Inf（无穷大），连续转动时也要填好 yaw 上下限。
 *       pitch 下限必须小于上限；选择 VISION_YAW_BOUNDED 时，yaw 也有同样要求。
 */
typedef struct
{
    uint8_t configured;                  /* 非 0 表示已设置；为 0 时不使用视觉目标。 */
    VisionYawConstraint_e yaw_constraint; /* 左右转动的方式。 */
    float pitch_min_deg;
    float pitch_max_deg;
    float pitch_zero_offset_deg; /* pitch 换算成度后加上的补偿角度。 */
    float yaw_min_deg;
    float yaw_max_deg;
} VisionControlLimits_s;

/* 视觉数据最长可使用 50 毫秒，max_age_ms 可设得更短。 */
#define VISION_CONTROL_MAX_AGE_MS 50u

/**
 * @brief 这次计算使用的视觉数据、接收时间和角度设置。
 * @note SP 角度使用弧度（rad）；last_rx_ms 和 now_ms 要使用同一种毫秒计时方式。
 */
typedef struct
{
    uint8_t online;       /* 非 0 表示通信在线。 */
    uint8_t mode;         /* 1=瞄准并让摩擦轮转动，2=再加上连续送弹；其他值不使用视觉目标。 */
    uint32_t last_rx_ms;  /* 最近一次收到合法视觉数据的时刻，ms。 */
    uint32_t now_ms;      /* 这次计算的当前时刻，ms。 */
    uint32_t max_age_ms;  /* 数据可使用的时长，0～50 ms；等于时长仍允许，为 0 时仅接受同一毫秒的数据。 */
    float yaw;           /* 左右转动的目标角度，rad。 */
    float yaw_vel;       /* 左右转动的目标速度，rad/s（弧度/秒）。 */
    float yaw_acc;       /* yaw_vel 每秒变化多少，单位：rad/s^2。 */
    float pitch;         /* 上下转动的目标角度，rad。 */
    float pitch_vel;     /* 上下转动的目标速度，rad/s。 */
    float pitch_acc;     /* pitch_vel 每秒变化多少，rad/s^2。 */
    const VisionControlLimits_s *limits; /* 外部提供的角度设置，计算结束前不能释放。 */
} VisionControlInput_s;

/** @brief 换算成度后的云台目标及转动速度。 */
typedef struct
{
    float yaw, pitch;                            /* 目标角度，deg；pitch 已加上补偿角度。 */
    float yaw_vel, yaw_acc, pitch_vel, pitch_acc; /* vel 表示速度，deg/s；acc 表示速度每秒的变化量，deg/s^2。 */
} VisionControlGimbalCommand_s;

/** @brief 待执行的发射命令。 */
typedef struct
{
    uint8_t shoot_enabled;    /* 发射电机开关请求，1=开启，0=关闭。 */
    uint8_t friction_enabled; /* 摩擦轮开关请求，1=开启，0=关闭。 */
    /* 与 loader_mode_e 编号一致；用 uint8_t 保存，电脑测试时不必包含 robot_def.h。 */
    uint8_t load_mode;
    float shoot_rate; /* 每秒希望发射的数量，发/秒。 */
} VisionControlShootCommand_s;

/* robot_cmd.c 会在编译时检查这些供弹编号是否与 loader_mode_e 一致。 */
enum
{
    VISION_CONTROL_LOAD_STOP = 0u,
    VISION_CONTROL_LOAD_REVERSE = 1u,
    VISION_CONTROL_LOAD_SINGLE_BULLET = 2u,
    VISION_CONTROL_LOAD_THREE_BULLETS = 3u,
    VISION_CONTROL_LOAD_BURSTFIRE = 4u
};

/**
 * @brief 检查这份视觉数据能不能用于控制云台。
 * @param[in] input 视觉数据，NULL 时返回 0。
 * @return 1=通过检查；0=数据不能用。
 * @note 检查过程与 VisionControlBuildGimbalCommand() 相同。
 */
uint8_t VisionControlInputIsUsable(const VisionControlInput_s *input);

/**
 * @brief 换算视觉角度和速度的单位，并检查目标角度范围。
 * @param[in] input 视觉数据及角度设置；NULL、没填好设置、离线、超时或模式不对时返回 0。
 * @param[out] command 云台命令；NULL 时返回 0。
 * @return 1=计算成功；0=检查未通过，非 NULL 的 command 各项清零。
 * @note rad、rad/s、rad/s^2 分别换算成 deg、deg/s、deg/s^2，输入和结果都不能是 NaN 或无穷大。
 *       pitch 加补偿后检查范围，等于上下限也允许；超出就返回 0，不会把目标改成上下限。
 *       yaw 连续转动时保留换算后的角度，不折回一圈内。
 */
uint8_t VisionControlBuildGimbalCommand(const VisionControlInput_s *input,
                                      VisionControlGimbalCommand_s *command);

/**
 * @brief 通过云台数据检查后，按 mode 生成视觉发射命令。
 * @param[in] input 视觉数据，NULL 时返回 0。
 * @param[out] command 发射命令；NULL 时返回 0。
 * @return 1=已生成命令；0=数据不能用，非 NULL 的 command 各项清零。
 * @note mode 1 请求发射电机和摩擦轮工作，停止供弹，shoot_rate 为 0。
 *       mode 2 再请求以 10 发/秒连续供弹。
 */
uint8_t VisionControlBuildShootCommand(const VisionControlInput_s *input,
                                       VisionControlShootCommand_s *command);

/**
 * @brief 优先使用视觉发射命令，视觉不能用时改用手动命令。
 * @param[in] input 视觉数据，可传 NULL。
 * @param[in] manual_command 这次的手动命令；NULL 表示没有可用命令。
 * @param[out] command 选中的命令；NULL 时返回 0。
 * @return 1=选中视觉；0=使用手动或关闭命令，或 command 为 NULL。
 * @note 手动命令原样复制，需要外部代码先确认数据可用、数字正确；视觉不能用且没有手动命令时输出清零。
 *       manual_command 和 command 要分开存放，防止先清零输出时把手动输入也清掉。
 */
uint8_t VisionControlSelectShootCommand(const VisionControlInput_s *input,
                                        const VisionControlShootCommand_s *manual_command,
                                        VisionControlShootCommand_s *command);

/**
 * @brief 根据遥控拨轮的位置，生成摩擦轮和供弹命令。
 * @param[in] dial 解码后的拨轮数值，调用前先确认遥控数据可用。
 * @param[out] command 发射命令；NULL 时直接返回。
 * @note dial < -100 时请求摩擦轮转动，dial < -500 时请求连续供弹，否则停止供弹。
 *       摩擦轮或供弹有一项需要工作，就打开 shoot_enabled；shoot_rate 固定填 8 发/秒。
 */
void VisionControlBuildRemoteShootCommand(int16_t dial,
                                          VisionControlShootCommand_s *command);

/**
 * @brief 根据 E、F 键按下的次数，选择供弹方式和摩擦轮开关。
 * @param[in] load_cycle E 键按下的次数，每四次循环：停止、单发、三发、连续供弹。
 * @param[in] friction_cycle F 键按下的次数，奇数请求摩擦轮转动，偶数关闭。
 * @param[out] command 发射命令；NULL 时直接返回。
 * @note 外部代码先确认键鼠数据可用，并记录按下次数；持续按住不能反复计数。
 *       摩擦轮或供弹有一项需要工作，就打开 shoot_enabled；shoot_rate 固定填 8 发/秒。
 */
void VisionControlBuildMouseKeyShootCommand(uint8_t load_cycle,
                                            uint8_t friction_cycle,
                                            VisionControlShootCommand_s *command);

#endif
