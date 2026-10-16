/**
 * @file vision_control.h
 * @brief 检查视觉数据，计算云台目标，并选择视觉或手动发射命令。
 *
 * RobotCMDTask 把这次计算使用的视觉数据交给本模块，再取走算好的命令。
 * 这里只计算命令；真正控制电机前，调用这些函数的代码还要检查当前是否允许动作。
 * 输入数据、角度设置和输出结果要分开存放，计算期间不要让其他代码修改输入和设置。
 */
#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include <stdint.h>

/* 选择云台左右转动（yaw）的方式，角度范围在换算成度以后检查。 */
typedef enum
{
    VISION_YAW_UNCONFIGURED = 0, /* 还没选好转动方式，暂不使用视觉目标。 */
    VISION_YAW_CONTINUOUS,       /* 可以连续转动，yaw 不受上下限限制。 */
    VISION_YAW_BOUNDED          /* 只能在上下限之间转动，等于上下限也允许。 */
} VisionYawConstraint_e;

/**
 * @brief 设置云台允许转动的角度范围，单位都是度（deg）。
 * @note 这里检查的是换算后的云台目标角度。pitch 表示上下转动，yaw 表示左右转动。
 *       所有角度都要是正常数字，不能填 NaN（无效数值）或 Inf（无穷大）。
 *       pitch 的下限必须小于上限；选择 VISION_YAW_BOUNDED 时，yaw 也有同样要求。
 *       即使选择连续转动，yaw 的上下限仍要填正常数字。
 */
typedef struct
{
    uint8_t configured;                  /* 非 0 表示已填好这组设置；为 0 时不使用视觉目标。 */
    VisionYawConstraint_e yaw_constraint; /* 选择连续转动，或只在指定角度范围内转动。 */
    float pitch_min_deg;                 /* 上下转动的最小角度。 */
    float pitch_max_deg;                 /* 上下转动的最大角度。 */
    float pitch_zero_offset_deg; /* pitch 换算成度后加上的补偿角度。 */
    float yaw_min_deg;                   /* 左右转动的最小角度。 */
    float yaw_max_deg;                   /* 左右转动的最大角度。 */
} VisionControlLimits_s;

/* 收到视觉数据后，最多允许继续使用 50 毫秒；max_age_ms 可以设得更短。 */
#define VISION_CONTROL_MAX_AGE_MS 50u

/**
 * @brief 保存这次计算使用的视觉数据、接收时间和角度设置。
 * @note SP 通信中的角度用弧度（rad），例如半圈是 pi rad；输出命令会换算成度。
 *       last_rx_ms 和 now_ms 要使用同一种毫秒计时方式，计算期间保持这些数据不变。
 */
typedef struct
{
    uint8_t online;       /* 非 0 表示通信在线，还要继续检查模式、时间和各个数字。 */
    uint8_t mode;         /* 1=瞄准并让摩擦轮转动，2=再加上连续送弹；其他值不使用视觉目标。 */
    uint32_t last_rx_ms;  /* 最近一次收到合法视觉数据的时刻，单位：毫秒（ms）。 */
    uint32_t now_ms;      /* 这次计算使用的当前时刻，单位：毫秒（ms）。 */
    uint32_t max_age_ms;  /* 距收到数据最多能过多久，可设为 0～50 ms，等于这个值也允许。 */
    float yaw;           /* 左右转动的目标角度，rad。 */
    float yaw_vel;       /* 左右转动的目标速度，rad/s（弧度/秒）。 */
    float yaw_acc;       /* yaw_vel 每秒变化多少，单位：rad/s^2。 */
    float pitch;         /* 上下转动的目标角度，rad。 */
    float pitch_vel;     /* 上下转动的目标速度，rad/s。 */
    float pitch_acc;     /* pitch_vel 每秒变化多少，rad/s^2。 */
    const VisionControlLimits_s *limits; /* 指向外部提供的角度设置，计算结束前不能释放或改写。 */
} VisionControlInput_s;

/** @brief 换算成度以后的目标角度、转动速度及速度每秒的变化量，交给云台控制代码使用。 */
typedef struct
{
    float yaw, pitch;                            /* 目标角度，deg；pitch 已加上补偿角度。 */
    float yaw_vel, yaw_acc, pitch_vel, pitch_acc; /* vel 表示速度，deg/s；acc 表示速度每秒的变化量，deg/s^2。 */
} VisionControlGimbalCommand_s;

/** @brief 保存希望发射机构做什么；真正执行前，还要检查当前是否允许发射。 */
typedef struct
{
    uint8_t shoot_enabled;    /* 是否请求发射机构的电机工作：1=开启，0=关闭。 */
    uint8_t friction_enabled; /* 是否请求摩擦轮转动：1=开启，0=关闭。 */
    /* 下方供弹模式的编号，与 loader_mode_e 使用相同数字，方便直接转给 robot_cmd.c。
       用 uint8_t 存编号，避免为了这个类型额外包含 robot_def.h，也方便在电脑上测试。 */
    uint8_t load_mode;
    float shoot_rate; /* 连续发射时，每秒希望发多少发，单位：发/秒。 */
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
 * @param[in] input 这次计算使用的视觉数据；传 NULL（没有提供数据）时返回 0。
 * @return 1=通信、模式、时间、角度设置和换算结果都通过检查；0=这份数据不能用。
 * @note 内部调用 VisionControlBuildGimbalCommand() 完成检查，input 中的内容不会改变。
 */
uint8_t VisionControlInputIsUsable(const VisionControlInput_s *input);

/**
 * @brief 把视觉给出的角度和速度换算成云台使用的单位，并检查角度是否在允许范围内。
 * @param[in] input 视觉数据及角度设置；NULL、没填好设置、离线、超时或模式不对时返回 0。
 * @param[out] command 存放算好的云台命令；传 NULL 时返回 0，不写入结果。
 * @return 1=计算成功；0=检查未通过。只要 command 不为 NULL，失败时其中各项都会清零。
 * @note 角度由 rad 换成 deg，速度由 rad/s 换成 deg/s，速度每秒的变化量也按同样比例换算。
 *       六个输入数值和换算结果都不能是 NaN 或无穷大。input 中的内容不会改变。
 *       pitch 先加补偿角度再检查范围。例如测试中输入相当于 25 度，补偿 5 度，就检查 30 度。
 *       角度等于上下限也允许；超过范围就返回 0，不会把目标角改成上限或下限。
 *       yaw 选择连续转动时不检查角度范围，换算出 720 度就保留 720 度。
 *       max_age_ms 设为 50 时，距接收过去 50 ms 仍可使用，51 ms 就返回 0；设为 0 时只接受同一毫秒的数据。
 */
uint8_t VisionControlBuildGimbalCommand(const VisionControlInput_s *input,
                                      VisionControlGimbalCommand_s *command);

/**
 * @brief 检查视觉数据能不能用于控制云台，通过后再按 mode 生成发射命令。
 * @param[in] input 这次使用的视觉数据；传 NULL 时返回 0。
 * @param[out] command 存放发射命令；传 NULL 时返回 0，不写入结果。
 * @return 1=已生成视觉发射命令；0=数据不能用，command 不为 NULL 时生成关闭发射的命令。
 * @note 关闭命令会将 shoot_enabled、friction_enabled 和 shoot_rate 清零，load_mode 设为停止供弹。
 *       mode 1 把 shoot_enabled 和 friction_enabled 设为 1，停止供弹，shoot_rate 为 0。
 *       mode 2 保持这两项为 1，并请求以 10 发/秒连续供弹。
 *       这里先写出希望机构做什么，真正执行前，调用这个函数的代码还要检查当前是否允许发射。
 */
uint8_t VisionControlBuildShootCommand(const VisionControlInput_s *input,
                                       VisionControlShootCommand_s *command);

/**
 * @brief 优先使用视觉发射命令；视觉数据不能用时，改用这次提供的手动命令。
 * @param[in] input 这次使用的视觉数据，可以传 NULL。
 * @param[in] manual_command 这次算好的手动命令；传 NULL 表示没有可用的手动命令。
 * @param[out] command 存放选中的命令；传 NULL 时直接返回 0，不写入结果。
 * @return 1=选中了视觉命令；0=没有选中视觉，可能已用了手动命令，也可能生成了关闭命令。
 * @note 例如视觉超时，但手动命令仍要求连续发射，这里会复制手动命令并返回 0。
 *       手动命令会原样复制，使用它之前需要由外部代码确认遥控或键鼠数据可用、命令数字正确。
 *       manual_command 和 command 要分开存放，因为本函数会先清零 command，再读取手动命令。
 *       视觉数据不能用且 manual_command 为 NULL 时，生成关闭发射、摩擦轮和供弹的命令，shoot_rate 也清零。
 */
uint8_t VisionControlSelectShootCommand(const VisionControlInput_s *input,
                                        const VisionControlShootCommand_s *manual_command,
                                        VisionControlShootCommand_s *command);

/**
 * @brief 根据遥控拨轮的位置，生成摩擦轮和供弹命令。
 * @param[in] dial 从遥控数据中解出的拨轮数值；调用前先确认遥控数据可用。
 * @param[out] command 存放发射命令；传 NULL 时直接返回。
 * @note dial < -100 时请求摩擦轮转动，dial < -500 时请求连续供弹，否则停止供弹。
 *       两个条件都用小于号，例如 dial 等于 -500 时摩擦轮转动，但仍停止供弹。
 *       摩擦轮或供弹有一项需要工作，就打开 shoot_enabled；shoot_rate 固定填 8 发/秒。
 *       此函数只根据拨轮数字计算，遥控是否在线要在调用前检查。
 */
void VisionControlBuildRemoteShootCommand(int16_t dial,
                                          VisionControlShootCommand_s *command);

/**
 * @brief 根据 E、F 键按下的次数，选择供弹方式和摩擦轮开关。
 * @param[in] load_cycle E 键按下的次数，每四次循环：停止、单发、三发、连续供弹。
 * @param[in] friction_cycle F 键按下的次数，奇数请求摩擦轮转动，偶数关闭。
 * @param[out] command 存放发射命令；传 NULL 时直接返回。
 * @note E 键次数为 0、1、2、3 时依次选择上述四种方式，为 4 时又回到停止。
 *       外部代码负责记录次数：每次从松开变成按下才加 1，持续按住不能反复加。
 *       调用前先确认键鼠数据可用。摩擦轮或供弹有一项需要工作，就打开 shoot_enabled。
 *       shoot_rate 固定填 8 发/秒，不会沿用上一次视觉命令中的每秒发射数量。
 */
void VisionControlBuildMouseKeyShootCommand(uint8_t load_cycle,
                                            uint8_t friction_cycle,
                                            VisionControlShootCommand_s *command);

#endif
