#include "dji_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_can.h"
#include <math.h>

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用

/* ---------------- 性能探针(2026-09, CPU 占用排查) ----------------
   dji_prof_all : 单次 DJIMotorControl 总耗时
   dji_prof_pid : 串级 PID 计算 + 组帧(不含 CAN 发送)
   dji_prof_send: CAN 报文发送(含 bsp_can 里的自旋等待) */
volatile DWT_Probe_t dji_prof_all = {0};
volatile DWT_Probe_t dji_prof_pid = {0};
volatile DWT_Probe_t dji_prof_send = {0};
volatile uint32_t dji_current_sat_count = 0; /* 电流指令被饱和的次数(MOTOR-09 观测) */
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算


#ifdef FDCAN
static CANInstance sender_assignment[9] = {
    [0] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x1ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [1] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x200, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [2] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x2ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [3] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x1ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [4] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x200, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [5] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x2ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [6] = {.can_handle = &hfdcan3, .txconf.Identifier = 0x1ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [7] = {.can_handle = &hfdcan3, .txconf.Identifier = 0x200, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [8] = {.can_handle = &hfdcan3, .txconf.Identifier = 0x2ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
};

#else

/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
 *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
 *
 * @note  因为只用于发送,所以不需要在bsp_can中注册
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
static CANInstance sender_assignment[6] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [4] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [5] = {.can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
};

#endif

/**
 * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 */
static uint8_t sender_enable_flag[9] = {0};

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    uint8_t grouping_offset;
    //通过CAN计算分组偏移量
    if(config->can_handle == &hcan1)
    {
        grouping_offset=0;
    }
    else if(config->can_handle == &hcan2)
    {
        grouping_offset=3;
    }
    else
    {
        grouping_offset=6;
    }

    switch (motor->motor_type)
    {
    case M2006:
    case M3508:
        if (motor_id < 4) // 根据ID分组
        {
            motor_send_num = motor_id;
            motor_grouping = grouping_offset + 1;
            
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = grouping_offset + 0;
        }

        // 计算接收id并设置分组发送id
        config->rx_id = 0x200 + motor_id + 1;   // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        // 检查是否发生id冲突
        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
                    /* rx_id 是 uint32_t(在 arm-none-eabi 上是 unsigned long), can_bus 是
                       uint16_t: 用 %d 打它们属于格式符与实参类型不符(新日志机制启用 GCC 的
                       格式检查后暴露)。 */
                    LOGERROR("[dji_motor] id [%lu], can_bus [%u]",
                             (unsigned long)config->rx_id, (unsigned)can_bus);
            }
        }
        break;

    case GM6020:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = grouping_offset + 0;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = grouping_offset + 2;
        }

        config->rx_id = 0x204 + motor_id + 1;   // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
                    LOGERROR("[dji_motor] id [%lu], can_bus [%u]",
                             (unsigned long)config->rx_id, (unsigned)can_bus);
            }
        }
        break;

    default: // other motors should not be registered here
        while (1)
            LOGERROR("[dji_motor]You must not register other motors using the API of DJI motor."); // 其他电机不应该在这里注册
    }
}

/**
 * @todo  是否可以简化多圈角度的计算？
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void DecodeDJIMotor(CANInstance *_instance)
{
    // 这里对can instance的id进行了强制转换,从而获得电机的instance实例地址
    // _instance指针指向的id是对应电机instance的地址,通过强制转换为电机instance的指针,再通过->运算符访问电机的成员motor_measure,最后取地址获得指针
    uint8_t *rxbuff = _instance->rx_buff;
    DJIMotorInstance *motor = (DJIMotorInstance *)_instance->id;
    DJI_Motor_Measure_s *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销

    DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];

    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

static void DJIMotorLostCallback(void *motor_ptr)
{
    DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
    /* 电机离线检测是 20ms 周期触发的: 没接电机时每 20ms 一条, 会把 RTT 缓冲刷爆 → 限速 1 条/秒 */
    static LogRateLimit_t rl_motor_lost = {0};
    if (LogRateLimitAllow(&rl_motor_lost, 1000u))
        LOGWARNING("[dji_motor] 电机离线: bus %d id %d (累计 %lu 次, 期间限速 %lu 条)",
                   (int)can_bus, (int)motor->motor_can_instance->tx_id,
                   (unsigned long)rl_motor_lost.total, (unsigned long)rl_motor_lost.dropped);
}

// 电机初始化,返回一个电机实例
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    DJIMotorInstance *instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
    memset(instance, 0, sizeof(DJIMotorInstance));

    // motor basic setting 电机基本设置
    instance->motor_type = config->motor_type;                         // 6020 or 2006 or 3508
    instance->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等

    // motor controller init 电机控制器初始化
    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    // 后续增加电机前馈控制器(速度和电流)

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    MotorSenderGrouping(instance, &config->can_init_config);

    // 注册电机到CAN总线
    config->can_init_config.can_module_callback = DecodeDJIMotor; // set callback
    config->can_init_config.id = instance;                        // set id,eq to address(it is identity)
    instance->motor_can_instance = CANRegister(&config->can_init_config);

    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .callback = DJIMotorLostCallback,
        .owner_id = instance,
        .reload_count = 2, // 20ms未收到数据则丢失
    };
    instance->daemon = DaemonRegister(&daemon_config);

    // 安全默认态: 初始化后保持失能, 由上层状态机在确认IMU/标定/使能条件后
    // 通过 DJIMotorEnable() 显式恢复输出
    DJIMotorStop(instance);
    dji_motor_instance[idx++] = instance;
    return instance;
}

/* 电流只能通过电机自带传感器监测,后续考虑加入力矩传感器应变片等 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        LOGERROR("[dji_motor] loop type error, check memory access and func param"); // 检查是否传入了正确的LOOP类型,或发生了指针越界
}

void DJIMotorStop(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DJIMotorEnable(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* 修改电机的实际闭环对象 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

/**
 * @brief 判断电机是否离线
 *
 */
void DJIMotorIsOnline(DJIMotorInstance *motor)
{
    if (DaemonIsOnline(motor->daemon) > 0)
        motor->online_flag = MOTOR_ONLINE;
    else
        motor->online_flag = MOTOR_OFFLINE;
}

/**
 * @brief 修改电机的实际闭环目标(内层闭环)
 *
 * @param motor  要修改的电机实例指针
 * @param outer_loop 闭环类型(用于设置 close_loop_type)
 */
void DJIMotorCloseLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.close_loop_type = outer_loop;
}

void DJIMotorGetSummary(DJIMotorInstance *motor, DJIMotorSummary_t *summary)
{
    if (motor == NULL || summary == NULL)
        return;

    summary->online = (DaemonIsOnline(motor->daemon) > 0) ? 1 : 0;
    summary->enabled = (motor->stop_flag == MOTOR_ENALBED) ? 1 : 0;
    summary->speed_aps = motor->measure.speed_aps;
    summary->angle_single_round = motor->measure.angle_single_round;
    summary->total_angle = motor->measure.total_angle;
    summary->current = motor->measure.real_current;
    summary->temperature = motor->measure.temperature;
}

// 设置参考值
void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

// 为所有电机实例计算三环PID,发送控制报文
void DJIMotorControl()
{
    uint32_t probe_all = DWT_ProbeStart();
    // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
    uint8_t group, num; // 电机组号和组内编号
    int16_t set;        // 电机控制CAN发送设定值
    DJIMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    DJI_Motor_Measure_s *measure;           // 电机测量值
    float pid_measure, pid_ref;             // 电机PID测量值和设定值

    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < idx; ++i)
    { // 减小访存开销,先保存指针引用
        motor = dji_motor_instance[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
            pid_ref *= -1; // 设置反转

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->speed_aps;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->real_current, pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
            pid_ref *= -1;

        // 获取最终输出,功率限制时直接输出电流值
        /* 先饱和再截断(MOTOR-09): pid_ref 超出 int16 量程时直接强转会**回绕**成
           反方向的大电流(例如 30000 → -25536, 电机反向猛转)。
           DJI 电流指令的有效量程是 ±16384(对应 ±20A), 按它做饱和。 */
        if (pid_ref > 16384.0f)
        {
            pid_ref = 16384.0f;
            dji_current_sat_count++;
        }
        else if (pid_ref < -16384.0f)
        {
            pid_ref = -16384.0f;
            dji_current_sat_count++;
        }
        set = (int16_t)pid_ref;

        // 分组填入发送数据
        group = motor->sender_group;
        num = motor->message_num;
        sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8);         // 低八位
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // 高八位

        // 若该电机处于停止状态,直接将buff置零
        if (motor->stop_flag == MOTOR_STOP)
            memset(sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
    }

    // 遍历flag,检查是否要发送这一帧报文
    DWT_ProbeDone(&dji_prof_pid, probe_all);
    uint32_t probe_send = DWT_ProbeStart();
#ifdef FDCAN
    for (size_t i = 0; i < 9; ++i)
#else
    for (size_t i = 0; i < 6; ++i)
#endif
    {
        if (sender_enable_flag[i])
        {
            CANTransmit(&sender_assignment[i], 1);
        }
    }
    DWT_ProbeDone(&dji_prof_send, probe_send);
    DWT_ProbeDone(&dji_prof_all, probe_all);
}

/* ============================================================================
   DJI 假反馈注入自测(默认关闭, 开关见 dji_motor.h 的 CAN_DJI_INJECT_TEST)
   ========================================================================== */
#if CAN_DJI_INJECT_TEST >= 1

volatile uint32_t can_dji_inject_pass = 0;
volatile uint32_t can_dji_inject_fail = 0;
volatile uint32_t can_dji_inject_multi_turn_err = 0;
volatile uint32_t can_dji_inject_frame_cnt = 0;

static CANInstance *s_inject_tx = NULL;             /* 注入发送用的临时实例 */
static DJIMotorInstance *s_inject_target = NULL;    /* 被注入的电机 */
static DJI_Motor_Measure_s s_inject_backup;         /* 测试前的 measure, 结束后还原 */
static FDCAN_HandleTypeDef *s_inject_h = NULL;

/* 按 DJI 反馈帧格式填好 tx_buff 并发出(不等待接收) */
static void DJIInjectSend(uint16_t ecd, int16_t speed_rpm, int16_t current_raw, uint8_t temp)
{
    if (s_inject_tx == NULL)
        return;
    s_inject_tx->tx_buff[0] = (uint8_t)(ecd >> 8);
    s_inject_tx->tx_buff[1] = (uint8_t)(ecd & 0xFFu);
    s_inject_tx->tx_buff[2] = (uint8_t)(((uint16_t)speed_rpm) >> 8);
    s_inject_tx->tx_buff[3] = (uint8_t)(((uint16_t)speed_rpm) & 0xFFu);
    s_inject_tx->tx_buff[4] = (uint8_t)(((uint16_t)current_raw) >> 8);
    s_inject_tx->tx_buff[5] = (uint8_t)(((uint16_t)current_raw) & 0xFFu);
    s_inject_tx->tx_buff[6] = temp;
    s_inject_tx->tx_buff[7] = 0u;
    can_dji_inject_frame_cnt++;
    (void)CANTransmit(s_inject_tx, 0.0f);
}

/* 边消费接收队列边等, 直到目标电机的 ecd 变成期望值; 返回 1=等到 */
static uint8_t DJIInjectWaitEcd(uint16_t expect, uint32_t timeout_us)
{
    uint32_t t0 = DWT_ProbeStart();
    while (DWT_ProbeElapsedUs(t0) < timeout_us)
    {
        CANProcessRx();
        if (s_inject_target != NULL && s_inject_target->measure.ecd == expect)
            return 1u;
    }
    return 0u;
}

uint8_t CANRunDjiFeedbackInjectTest(void)
{
    can_dji_inject_pass = 0;
    can_dji_inject_fail = 0;
    can_dji_inject_multi_turn_err = 0;
    can_dji_inject_frame_cnt = 0;

    if (idx == 0u || dji_motor_instance[0] == NULL)
    {
        LOGERROR("[dji_inject] 没有已注册的 DJI 电机, 跳过注入自测");
        return 0u;
    }

    s_inject_target = dji_motor_instance[0];
    s_inject_h = s_inject_target->motor_can_instance->can_handle;
    uint32_t target_rx_id = s_inject_target->motor_can_instance->rx_id;
    s_inject_backup = s_inject_target->measure;

    /* 1. 切内部回环: 帧不会真的上线, 但自己发自己收, 不会干扰总线上的其它设备 */
    if (CANForceMode(s_inject_h, FDCAN_MODE_INTERNAL_LOOPBACK, 0u) == 0u)
    {
        LOGERROR("[dji_inject] 切内部回环失败, 跳过");
        /* CANForceMode 失败时可能已经把总线 Stop 掉了, 这里必须恢复一次,
           否则这条总线上线不了, 后面的电机控制帧全都发不出去。 */
        (void)CANForceMode(s_inject_h, FDCAN_MODE_NORMAL, 0u);
        CANReapplyFilters();
        return 0u;
    }

    /* 关键: CANForceMode 内部的 HAL_FDCAN_Init() 会重配消息 RAM, 把各电机实例
       原先注册的接收滤镜一起清掉。不重装的话, 我们发出去的 0x201 根本没有滤镜
       去接, 注入帧会被硬件直接丢弃(实测: 5 项全失败、只发出 4 帧就是这个原因)。
       回环自测不需要这一步, 因为它注册测试实例时自己会加滤镜。 */
    CANReapplyFilters();

    /* 2. 注册"只发不收"的注入实例: tx_id 取目标电机的 rx_id(如 0x201),
          回环回来的帧就会命中该电机的接收滤镜, 走进它正常的解码路径。
          rx_id 用一个用不到的 ID, 免得去抢别人要收的帧。 */
    CAN_Init_Config_s cfg = {
        .can_handle = s_inject_h,
        .tx_id = target_rx_id,
        .rx_id = 0x7FFu,
        .can_module_callback = NULL,
        .id = NULL,
    };
    s_inject_tx = CANRegister(&cfg);
    if (s_inject_tx == NULL)
    {
        LOGERROR("[dji_inject] 注册注入实例失败");
        (void)CANForceMode(s_inject_h, FDCAN_MODE_NORMAL, 0u);
        CANReapplyFilters();
        return 0u;
    }
    CANSetDLC(s_inject_tx, 8u);

    /* ---- 测试 1: 单帧解码(ecd / 温度) ---- */
    DJIInjectSend(1000u, 100, 50, 42u);
    if (DJIInjectWaitEcd(1000u, 3000u) && s_inject_target->measure.temperature == 42u)
        can_dji_inject_pass++;
    else
        can_dji_inject_fail++;

    /* ---- 测试 2: 正向跨圈。ecd 1000→8100, 差 +7100 > 4096(半圈) → total_round 应 -1 ---- */
    int32_t r0 = s_inject_target->measure.total_round;
    DJIInjectSend(8100u, 0, 0, 42u);
    if (DJIInjectWaitEcd(8100u, 3000u) && s_inject_target->measure.total_round == r0 - 1)
        can_dji_inject_pass++;
    else
    {
        can_dji_inject_fail++;
        can_dji_inject_multi_turn_err++;
    }

    /* ---- 测试 3: 反向跨圈。ecd 8100→1000, 差 -7100 < -4096 → total_round 应回到 r0 ---- */
    DJIInjectSend(1000u, 0, 0, 42u);
    if (DJIInjectWaitEcd(1000u, 3000u) && s_inject_target->measure.total_round == r0)
        can_dji_inject_pass++;
    else
    {
        can_dji_inject_fail++;
        can_dji_inject_multi_turn_err++;
    }

    /* ---- 测试 4: total_angle 与 (total_round, ecd) 一致 ---- */
    float expect_angle = (float)(r0 * 360) + ECD_ANGLE_COEF_DJI * 1000.0f;
    if (fabsf(s_inject_target->measure.total_angle - expect_angle) < 0.5f)
        can_dji_inject_pass++;
    else
        can_dji_inject_fail++;

    /* ---- 测试 5: 连续多帧的 ecd 序列(验证逐帧、按到达顺序被处理) ---- */
    uint8_t seq_ok = 1u;
    for (uint16_t k = 0; k < 20u; ++k)
    {
        uint16_t e = (uint16_t)((1000u + k * 13u) & 0x1FFFu);
        DJIInjectSend(e, 0, 0, 42u);
        if (!DJIInjectWaitEcd(e, 2000u))
        {
            seq_ok = 0u;
            break;
        }
    }
    if (seq_ok)
        can_dji_inject_pass++;
    else
        can_dji_inject_fail++;

#if CAN_DJI_INJECT_TEST >= 2
    /* 压测模式: 保留回环与注入实例, 由 daemon 循环持续注入, 用来观察队列深度与丢帧 */
    LOGINFO("[dji_inject] 压测模式: 保留回环, pass=%lu fail=%lu",
            (unsigned long)can_dji_inject_pass, (unsigned long)can_dji_inject_fail);
#else
    /* 收尾: 还原 measure、注销注入实例、切回正常模式并重装滤镜 */
    s_inject_target->measure = s_inject_backup;
    CANUnregisterInstance(s_inject_tx);
    s_inject_tx = NULL;
    (void)CANForceMode(s_inject_h, FDCAN_MODE_NORMAL, 0u);
    CANReapplyFilters();
#endif

    LOGINFO("[dji_inject] done: pass=%lu fail=%lu multierr=%lu frames=%lu",
            (unsigned long)can_dji_inject_pass, (unsigned long)can_dji_inject_fail,
            (unsigned long)can_dji_inject_multi_turn_err,
            (unsigned long)can_dji_inject_frame_cnt);
    return (can_dji_inject_fail == 0u) ? 1u : 0u;
}

#if CAN_DJI_INJECT_TEST >= 2
void DJIInjectStressTick(void)
{
    if (s_inject_tx == NULL)
        return;
    /* 每轮塞一小批帧(不等待消费), 制造"任务还没来得及消费"的场景,
       从而观察 rxq_peak(队列最大占用)与 rxq_drop(队列满丢帧)。 */
    static uint16_t ecd = 0u;
    for (uint8_t n = 0; n < 10u; ++n)
    {
        ecd = (uint16_t)((ecd + 7u) & 0x1FFFu);
        DJIInjectSend(ecd, 50, 20, 40u);
    }
}
#endif

#endif /* CAN_DJI_INJECT_TEST >= 1 */
