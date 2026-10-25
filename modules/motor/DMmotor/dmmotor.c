/**
 * @file dmmotor.c
 * @brief 注册 DM 直接力矩实例，并由每个电机任务独占处理命令和周期报文。
 * @note 注册与任务创建须在调度器启动前串行完成；不在初始化阶段驱动电机。
 */
#include "dmmotor.h"

#include "bsp_can.h"
#include "cmsis_os.h"
#include "daemon.h"
#include "memory.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct
{
    float velocity;
    float last_position;
    float position;
    float torque;
    float mos_temperature;
    float rotor_temperature;
} DM_Motor_Measure_t;

struct DMMotorInstance
{
    DM_Motor_Measure_t measure;
    Motor_Control_Setting_s setting;
    float pid_ref;
    uint32_t tx_id;
    uint32_t rx_id;
    FDCAN_HandleTypeDef *can_handle;
    CANInstance *can_instance;
    DaemonInstance *daemon;
    uint32_t command_sequence;
    uint32_t command_tx_request_id;
    DMMotorCommandType_t command_type;
    DMMotorCommandResult_t command_result;
    DMMotorCalculationStatus_t calculation_status;
    uint8_t task_ready;
    uint8_t running;
    uint8_t protocol_enabled;
    uint8_t reference_available;
    uint8_t command_processing;
    uint8_t enable_transition_pending;
};

static DMMotorInstance *dm_instances[DM_MOTOR_CNT];
static osThreadId dm_task_handles[DM_MOTOR_CNT];
static uint8_t dm_instance_count;
static uint8_t dm_control_started;
static uint8_t dm_registry_quarantined;

static void DMMotorTask(void const *argument);

/* CMSIS-RTOS v1 要求任务定义为编译期对象，名称固定且每个实例只建一次。 */
osThreadDef(dm_motor_0, DMMotorTask, osPriorityNormal, 0, 128);
osThreadDef(dm_motor_1, DMMotorTask, osPriorityNormal, 0, 128);
osThreadDef(dm_motor_2, DMMotorTask, osPriorityNormal, 0, 128);
osThreadDef(dm_motor_3, DMMotorTask, osPriorityNormal, 0, 128);

static const osThreadDef_t *const dm_task_definitions[DM_MOTOR_CNT] = {
    osThread(dm_motor_0), osThread(dm_motor_1), osThread(dm_motor_2), osThread(dm_motor_3)};

static uint8_t DMMotorIsRegistered(const DMMotorInstance *motor)
{
    if (motor == NULL)
        return 0u;
    // 只比较已发布的实例地址，避免查询接口解引用调用者传入的未知指针。
    for (uint8_t i = 0u; i < dm_instance_count; ++i)
    {
        if (dm_instances[i] == motor)
            return 1u;
    }
    return 0u;
}

static uint8_t DMMotorSupportedBus(FDCAN_HandleTypeDef *handle)
{
    return (uint8_t)(handle == &hfdcan1 || handle == &hfdcan2 || handle == &hfdcan3);
}

static void DMMotorDecode(CANInstance *can_instance)
{
    if (can_instance == NULL || can_instance->id == NULL || can_instance->rx_len < 8u)
        return;

    DMMotorInstance *motor = (DMMotorInstance *)can_instance->id;
    const uint8_t *data = can_instance->rx_buff;
    const uint16_t position = (uint16_t)(((uint16_t)data[1] << 8u) | data[2]);
    const uint16_t velocity = (uint16_t)(((uint16_t)data[3] << 4u) | (data[4] >> 4u));
    const uint16_t torque = (uint16_t)(((uint16_t)(data[4] & 0x0fu) << 8u) | data[5]);

    motor->measure.last_position = motor->measure.position;
    motor->measure.position = ((float)position * (DM_P_MAX - DM_P_MIN) / 65535.0f) + DM_P_MIN;
    motor->measure.velocity = ((float)velocity * (DM_V_MAX - DM_V_MIN) / 4095.0f) + DM_V_MIN;
    motor->measure.torque = ((float)torque * (DM_T_MAX - DM_T_MIN) / 4095.0f) + DM_T_MIN;
    motor->measure.mos_temperature = (float)data[6];
    motor->measure.rotor_temperature = (float)data[7];
    (void)DaemonReportAt(motor->daemon, can_instance->rx_timestamp_ms,
        DAEMON_REPORT_TRANSPORT_VALID | DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID);
}

static void DMMotorLostCallback(const DaemonEvent_t *event, void *owner)
{
    (void)event;
    (void)owner;
}

static uint16_t DMMotorFloatToUint(float value, float minimum, float maximum, uint8_t bits)
{
    const uint32_t maximum_integer = (1u << bits) - 1u;
    if (value <= minimum)
        return 0u;
    if (value >= maximum)
        return (uint16_t)maximum_integer;

    const float mapped = (value - minimum) * (float)maximum_integer / (maximum - minimum);
    if (!isfinite(mapped) || mapped < 0.0f || mapped > (float)maximum_integer)
        return 0u;
    return (uint16_t)mapped;
}

static void DMMotorWriteModeFrame(CANInstance *can_instance, DMMotor_Mode_e command)
{
    memset(can_instance->tx_buff, 0xff, 7u);
    can_instance->tx_buff[7] = (uint8_t)command;
}

static void DMMotorWriteControlFrame(CANInstance *can_instance, float torque)
{
    const uint16_t position = DMMotorFloatToUint(0.0f, DM_P_MIN, DM_P_MAX, 16u);
    const uint16_t velocity = DMMotorFloatToUint(0.0f, DM_V_MIN, DM_V_MAX, 12u);
    // 力矩范围的中点量化为0x7FF；停止时仍需发送完整的合法控制帧。
    const uint16_t torque_field = DMMotorFloatToUint(torque, DM_T_MIN, DM_T_MAX, 12u);

    can_instance->tx_buff[0] = (uint8_t)(position >> 8u);
    can_instance->tx_buff[1] = (uint8_t)position;
    can_instance->tx_buff[2] = (uint8_t)(velocity >> 4u);
    can_instance->tx_buff[3] = (uint8_t)(((velocity & 0x0fu) << 4u));
    can_instance->tx_buff[4] = 0u;
    can_instance->tx_buff[5] = 0u;
    can_instance->tx_buff[6] = (uint8_t)(torque_field >> 8u);
    can_instance->tx_buff[7] = (uint8_t)torque_field;
}

static uint8_t DMMotorProcessCommand(DMMotorInstance *motor)
{
    DMMotorCommandType_t type = DMMOTOR_COMMAND_NONE;
    uint32_t sequence = 0u;
    uint32_t tx_request_id = 0u;

    taskENTER_CRITICAL();
    if (motor->command_result == DMMOTOR_COMMAND_PENDING && motor->command_processing == 0u)
    {
        // 排队后仍复查本地运行状态，避免活动期间执行编码器置零。
        if (motor->command_type == DMMOTOR_COMMAND_ZERO_POSITION && motor->running != 0u)
        {
            motor->command_result = DMMOTOR_COMMAND_REJECTED;
            taskEXIT_CRITICAL();
            return 1u;
        }
        motor->command_processing = 1u;
        type = motor->command_type;
        sequence = motor->command_sequence;
        tx_request_id = motor->command_tx_request_id;
    }
    taskEXIT_CRITICAL();

    if (type == DMMOTOR_COMMAND_NONE)
        return 0u;

    uint8_t accepted = 0u;
    if (tx_request_id != 0u)
    {
        CAN_TxResult_t tx_result;
        memset(&tx_result, 0, sizeof(tx_result));
        if (CANGetTxRequestState(tx_request_id, &tx_result) != 0u)
        {
            accepted = (uint8_t)(tx_result.state == CAN_TX_RESULT_IN_FLIGHT ||
                                 tx_result.state == CAN_TX_RESULT_COMPLETE);
            if (tx_result.state == CAN_TX_RESULT_QUEUED ||
                tx_result.state == CAN_TX_RESULT_ACTIVE)
            {
                taskENTER_CRITICAL();
                if (motor->command_sequence == sequence &&
                    motor->command_result == DMMOTOR_COMMAND_PENDING)
                    motor->command_processing = 0u;
                taskEXIT_CRITICAL();
                return 1u;
            }
            if (tx_result.state == CAN_TX_RESULT_FAILED ||
                tx_result.state == CAN_TX_RESULT_EXPIRED ||
                tx_result.state == CAN_TX_RESULT_CANCELLED ||
                tx_result.state == CAN_TX_RESULT_SUPERSEDED)
                accepted = 0u;
        }
    }
    else
    {
        // 任务在本地候选帧中准备命令；队列接收后即可复用实例缓冲。
        if (type == DMMOTOR_COMMAND_ENABLE)
            DMMotorWriteModeFrame(motor->can_instance, DM_CMD_MOTOR_MODE);
        else
            DMMotorWriteModeFrame(motor->can_instance, DM_CMD_ZERO_POSITION);
        const CAN_TxFrame_t frame = {
            .identifier = motor->tx_id,
            .identifier_type = CAN_TX_ID_STANDARD,
            .data = motor->can_instance->tx_buff,
            .length = 8u,
        };
        uint32_t request_id = 0u;
        const CAN_TxSubmitStatus_t submit_status = CANSubmitOneShot(
            motor->can_handle, motor->can_instance, &frame, CAN_TX_PRIORITY_URGENT,
            100u, &request_id);
        taskENTER_CRITICAL();
        if (motor->command_sequence == sequence &&
            motor->command_result == DMMOTOR_COMMAND_PENDING)
        {
            if (submit_status == CAN_TX_SUBMIT_ACCEPTED)
                motor->command_tx_request_id = request_id;
            else
            {
                motor->command_result = DMMOTOR_COMMAND_REJECTED;
                motor->enable_transition_pending = 0u;
            }
        }
        motor->command_processing = 0u;
        taskEXIT_CRITICAL();
        return 1u;
    }

    taskENTER_CRITICAL();
    if (motor->command_sequence == sequence && motor->command_result == DMMOTOR_COMMAND_PENDING)
    {
        motor->command_result = accepted != 0u ? DMMOTOR_COMMAND_ACCEPTED : DMMOTOR_COMMAND_REJECTED;
        motor->command_tx_request_id = 0u;
        if (type == DMMOTOR_COMMAND_ENABLE && accepted != 0u)
        {
            motor->protocol_enabled = 1u;
            motor->pid_ref = 0.0f;
            motor->reference_available = 0u;
            if (motor->enable_transition_pending != 0u)
                motor->running = 1u;
            motor->enable_transition_pending = 0u;
        }
        else if (type == DMMOTOR_COMMAND_ENABLE)
        {
            motor->enable_transition_pending = 0u;
        }
    }
    motor->command_processing = 0u;
    taskEXIT_CRITICAL();
    return 1u;
}

static void DMMotorTask(void const *argument)
{
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    // 每轮先处理显式命令；未被协议使能前不发送周期力矩帧。
    for (;;)
    {
        if (DMMotorProcessCommand(motor) == 0u)
        {
            uint8_t protocol_enabled;
            uint8_t running;
            uint8_t reference_available;
            float reference;
            Motor_Reverse_Flag_e direction;

            taskENTER_CRITICAL();
            protocol_enabled = motor->protocol_enabled;
            running = motor->running;
            reference_available = motor->reference_available;
            reference = motor->pid_ref;
            direction = motor->setting.motor_reverse_flag;
            taskEXIT_CRITICAL();

            if (protocol_enabled != 0u)
            {
                float torque = 0.0f;
                DMMotorCalculationStatus_t result = DMMOTOR_CALCULATION_NO_REFERENCE;
                if (running > 1u || (direction != MOTOR_DIRECTION_NORMAL && direction != MOTOR_DIRECTION_REVERSE))
                {
                    result = DMMOTOR_CALCULATION_INVALID_INPUT;
                }
                else if (running != 0u && reference_available != 0u)
                {
                    if (!isfinite(reference))
                    {
                        result = DMMOTOR_CALCULATION_INVALID_INPUT;
                    }
                    else
                    {
                        // 先按方向反转，再限幅，保证写入编码字段的数值在协议范围内。
                        torque = direction == MOTOR_DIRECTION_REVERSE ? -reference : reference;
                        if (torque < DM_T_MIN)
                            torque = DM_T_MIN;
                        else if (torque > DM_T_MAX)
                            torque = DM_T_MAX;
                        result = isfinite(torque) ? DMMOTOR_CALCULATION_OK : DMMOTOR_CALCULATION_INVALID_INPUT;
                        if (result != DMMOTOR_CALCULATION_OK)
                            torque = 0.0f;
                    }
                }

                DMMotorWriteControlFrame(motor->can_instance, torque);
                const CAN_TxFrame_t frame = {
                    .identifier = motor->tx_id,
                    .identifier_type = CAN_TX_ID_STANDARD,
                    .data = motor->can_instance->tx_buff,
                    .length = 8u,
                };
                uint32_t request_id = 0u;
                (void)CANSubmitPeriodic(motor->can_handle, motor->can_instance, 0u,
                                        &frame, CAN_TX_PRIORITY_CONTROL, 10u, 2000u,
                                        &request_id);
                taskENTER_CRITICAL();
                motor->calculation_status = result;
                if (result == DMMOTOR_CALCULATION_INVALID_INPUT)
                {
                    // 非法输入只影响本轮；清除旧参考后，后续合法参考仍可恢复。
                    motor->pid_ref = 0.0f;
                    motor->reference_available = 0u;
                }
                taskEXIT_CRITICAL();
            }
        }
        osDelay(2u);
    }
}

DMMotorInstance *DMMotorInit(const Motor_Init_Config_s *config)
{
    if (config == NULL || dm_control_started != 0u || dm_registry_quarantined != 0u ||
        dm_instance_count >= DM_MOTOR_CNT)
        return NULL;

    const Motor_Control_Setting_s *setting = &config->controller_setting_init_config;
    if (!DMMotorSupportedBus(config->can_init_config.can_handle) ||
        config->can_init_config.tx_id > 0x7ffu || config->can_init_config.rx_id > 0x7ffu ||
        (setting->close_loop_type != OPEN_LOOP) || setting->outer_loop_type != OPEN_LOOP ||
        setting->feedforward_flag != FEEDFORWARD_NONE ||
        (setting->motor_reverse_flag != MOTOR_DIRECTION_NORMAL &&
         setting->motor_reverse_flag != MOTOR_DIRECTION_REVERSE))
        return NULL;

    // 仅阻止同一总线上的发送号或接收号重复。
    for (uint8_t i = 0u; i < dm_instance_count; ++i)
    {
        if (dm_instances[i]->can_handle == config->can_init_config.can_handle &&
            (dm_instances[i]->tx_id == config->can_init_config.tx_id ||
             dm_instances[i]->rx_id == config->can_init_config.rx_id))
            return NULL;
    }

    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(*motor));
    if (motor == NULL)
        return NULL;
    memset(motor, 0, sizeof(*motor));
    motor->setting = *setting;
    motor->can_handle = config->can_init_config.can_handle;
    motor->tx_id = config->can_init_config.tx_id;
    motor->rx_id = config->can_init_config.rx_id;
    motor->calculation_status = DMMOTOR_CALCULATION_NO_REFERENCE;

    Daemon_Init_Config_s daemon_config = {
        .transport_timeout_ms = 100u,
        .data_timeout_ms = 100u,
        .startup_grace_ms = 1000u,
        .event_max_age_ms = 1000u,
        .source_id = (uint16_t)(0x3000u | (config->can_init_config.rx_id & 0x01ffu)),
        .event_capacity = 1u,
        .consumer_id = DAEMON_CONSUMER_MOTOR_TASK,
        .event_policy = DAEMON_EVENT_COALESCED_STATE,
        .consumer_mode = DAEMON_CONSUMER_BOUND_TASK,
        .event_handler = DMMotorLostCallback,
        .owner_id = motor,
    };
    motor->daemon = DaemonRegister(&daemon_config);
    if (motor->daemon == NULL)
    {
        // Daemon 未登记时只有候选电机对象需要释放。
        free(motor);
        return NULL;
    }

    CAN_Init_Config_s can_config = config->can_init_config;
    can_config.can_module_callback = DMMotorDecode;
    can_config.id = motor;
    motor->can_instance = CANRegister(&can_config);
    if (motor->can_instance == NULL)
    {
        // CAN 失败时先撤销回调所需的 Daemon，再释放其 owner 对象。
        if (DaemonUnregister(motor->daemon) != 0u)
            free(motor);
        else
            dm_registry_quarantined = 1u;
        return NULL;
    }

    /* CAN 和 Daemon 均已成功；从此不再有需要回退的资源申请。 */
    dm_instances[dm_instance_count++] = motor;
    return motor;
}

DMMotorStatus_t DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    if (motor == NULL)
        return DMMOTOR_STATUS_INVALID_PARAMETER;
    if (!DMMotorIsRegistered(motor))
        return DMMOTOR_STATUS_NOT_REGISTERED;

    taskENTER_CRITICAL();
    if (!isfinite(ref))
    {
        motor->pid_ref = 0.0f;
        motor->reference_available = 0u;
        motor->calculation_status = DMMOTOR_CALCULATION_INVALID_INPUT;
        taskEXIT_CRITICAL();
        return DMMOTOR_STATUS_INVALID_VALUE;
    }
    motor->pid_ref = ref;
    motor->reference_available = 1u;
    taskEXIT_CRITICAL();
    return DMMOTOR_STATUS_OK;
}

DMMotorStatus_t DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    if (motor == NULL)
        return DMMOTOR_STATUS_INVALID_PARAMETER;
    if (!DMMotorIsRegistered(motor))
        return DMMOTOR_STATUS_NOT_REGISTERED;
    return type == OPEN_LOOP ? DMMOTOR_STATUS_OK : DMMOTOR_STATUS_INVALID_CONFIG;
}

DMMotorStatus_t DMMotorEnable(DMMotorInstance *motor)
{
    if (motor == NULL)
        return DMMOTOR_STATUS_INVALID_PARAMETER;
    if (!DMMotorIsRegistered(motor))
        return DMMOTOR_STATUS_NOT_REGISTERED;

    taskENTER_CRITICAL();
    if (motor->task_ready == 0u || dm_registry_quarantined != 0u)
    {
        taskEXIT_CRITICAL();
        return DMMOTOR_STATUS_NOT_READY;
    }
    if (motor->running != 0u)
    {
        // 已在采用参考时重复使能不再提交模式命令。
        taskEXIT_CRITICAL();
        return DMMOTOR_STATUS_OK;
    }
    if (motor->command_processing != 0u || motor->command_result == DMMOTOR_COMMAND_PENDING)
    {
        taskEXIT_CRITICAL();
        return DMMOTOR_STATUS_BUSY;
    }
    if (motor->protocol_enabled != 0u)
    {
        // Stop 保留协议使能；恢复本地控制前仍清除先前目标。
        motor->pid_ref = 0.0f;
        motor->reference_available = 0u;
        motor->running = 1u;
        motor->calculation_status = DMMOTOR_CALCULATION_NO_REFERENCE;
        taskEXIT_CRITICAL();
        return DMMOTOR_STATUS_OK;
    }

    motor->command_sequence++;
    if (motor->command_sequence == 0u)
        motor->command_sequence = 1u;
    motor->command_type = DMMOTOR_COMMAND_ENABLE;
    motor->command_result = DMMOTOR_COMMAND_PENDING;
    motor->enable_transition_pending = 1u;
    taskEXIT_CRITICAL();
    return DMMOTOR_STATUS_QUEUED;
}

DMMotorStatus_t DMMotorStop(DMMotorInstance *motor)
{
    if (motor == NULL)
        return DMMOTOR_STATUS_INVALID_PARAMETER;
    if (!DMMotorIsRegistered(motor))
        return DMMOTOR_STATUS_NOT_REGISTERED;

    uint32_t cancel_request_id = 0u;
    taskENTER_CRITICAL();
    motor->pid_ref = 0.0f;
    motor->reference_available = 0u;
    motor->running = 0u;
    motor->enable_transition_pending = 0u;
    DMMotorStatus_t status = DMMOTOR_STATUS_OK;
    if (motor->command_result == DMMOTOR_COMMAND_PENDING && motor->command_processing == 0u)
    {
        // 只有尚未被任务取走的请求可以取消；已经开始提交的 CAN 帧无法撤回。
        cancel_request_id = motor->command_tx_request_id;
        motor->command_tx_request_id = 0u;
        motor->command_result = DMMOTOR_COMMAND_CANCELLED;
        status = DMMOTOR_STATUS_CANCELLED;
    }
    taskEXIT_CRITICAL();
    if (cancel_request_id != 0u)
        (void)CANCancelTxRequest(cancel_request_id);
    return status;
}

DMMotorStatus_t DMMotorCaliEncoder(DMMotorInstance *motor)
{
    if (motor == NULL)
        return DMMOTOR_STATUS_INVALID_PARAMETER;
    if (!DMMotorIsRegistered(motor))
        return DMMOTOR_STATUS_NOT_REGISTERED;

    taskENTER_CRITICAL();
    if (motor->task_ready == 0u || dm_registry_quarantined != 0u)
    {
        taskEXIT_CRITICAL();
        return DMMOTOR_STATUS_NOT_READY;
    }
    if (motor->running != 0u)
    {
        taskEXIT_CRITICAL();
        return DMMOTOR_STATUS_NOT_READY;
    }
    if (motor->command_processing != 0u || motor->command_result == DMMOTOR_COMMAND_PENDING)
    {
        taskEXIT_CRITICAL();
        return DMMOTOR_STATUS_BUSY;
    }
    motor->command_sequence++;
    if (motor->command_sequence == 0u)
        motor->command_sequence = 1u;
    motor->command_type = DMMOTOR_COMMAND_ZERO_POSITION;
    motor->command_result = DMMOTOR_COMMAND_PENDING;
    taskEXIT_CRITICAL();
    return DMMOTOR_STATUS_QUEUED;
}

DMMotorStatus_t DMMotorControlInit(void)
{
    if (dm_control_started != 0u)
        return DMMOTOR_STATUS_OK;
    if (dm_registry_quarantined != 0u)
        return DMMOTOR_STATUS_RESOURCE_ERROR;
    if (dm_instance_count == 0u)
        return DMMOTOR_STATUS_NOT_REGISTERED;

    // 调度器尚未启动，因此全部实例任务可逐个创建后统一就绪。
    uint8_t created = 0u;
    for (; created < dm_instance_count; ++created)
    {
        dm_task_handles[created] = osThreadCreate(dm_task_definitions[created], dm_instances[created]);
        if (dm_task_handles[created] == NULL)
            break;
    }
    if (created != dm_instance_count)
    {
        // 部分创建失败时反向回收；有线程无法确认结束就保持整组失能。
        while (created > 0u)
        {
            --created;
            if (osThreadTerminate(dm_task_handles[created]) != osOK)
            {
                dm_registry_quarantined = 1u;
                return DMMOTOR_STATUS_RESOURCE_ERROR;
            }
            dm_task_handles[created] = NULL;
        }
        return DMMOTOR_STATUS_RESOURCE_ERROR;
    }

    for (uint8_t i = 0u; i < dm_instance_count; ++i)
        dm_instances[i]->task_ready = 1u;
    dm_control_started = 1u;
    return DMMOTOR_STATUS_OK;
}

DMMotorStatus_t DMMotorGetState(const DMMotorInstance *motor, DMMotorState_t *state)
{
    if (motor == NULL || state == NULL)
        return DMMOTOR_STATUS_INVALID_PARAMETER;
    if (!DMMotorIsRegistered(motor))
        return DMMOTOR_STATUS_NOT_REGISTERED;

    DMMotorState_t candidate;
    taskENTER_CRITICAL();
    candidate.task_ready = motor->task_ready;
    candidate.running = motor->running;
    candidate.protocol_enabled = motor->protocol_enabled;
    candidate.reference_available = motor->reference_available;
    candidate.command_sequence = motor->command_sequence;
    candidate.command_type = motor->command_type;
    candidate.command_result = motor->command_result;
    candidate.calculation_status = motor->calculation_status;
    taskEXIT_CRITICAL();
    *state = candidate;
    return DMMOTOR_STATUS_OK;
}
