/**
* @file ins_task.c
* @brief 读取 BMI088 样本，更新姿态估计，并向视觉链路提供有效数据。
* StartINSTASK 在专属任务中初始化本模块；INS_Task 按1kHz运行。
* @author Wang Hongxi
* @author annotation and modificaiton by neozng
 * @version V2.0.0
* @date 2022/2/23
 */
#include "ins_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "spi.h"
#include "tim.h"
#include "vector3.h"
#include "general_def.h"
#include "master_process.h"
#include "arm_math.h"
#include "bmi088.h"
#include "robot_safety.h"
#include "imu_heater.h"
#include "task_monitor.h"
#include <math.h>
#include <float.h>
#include <string.h>

#define DEG_2_RAD (1.0f / RAD_2_DEGREE) // 角度转弧度, sp_vision_25 协议为弧度制

static INS_t INS;
static IMU_Param_t IMU_Param;
static BMI088Instance *bmi088_ins;
static BMI088_Data_t bmi088_data;
static uint8_t ins_init_result;
static INS_Control_Snapshot_s ins_control_snapshot;
// KF工作区跟随任务存活；20ms和2倍方差是默认软件范围。
static _Alignas(KF_WORKSPACE_ALIGNMENT) uint8_t qekf_workspace[KF_WORKSPACE_BYTES(6,0,3)];
// 20ms只限制无校正预测的发布；较长的有限dt由计算结果检查决定能否继续。
static const QEKFConfig_t qekf_config={10.0f,0.001f,1000000.0f,1.0f,0.0f,20u,2.0f,FLT_MAX};
volatile uint32_t imu_filter_mode_diag=0u;
volatile uint32_t imu_filter_age_ms_diag=0u;
volatile int32_t imu_filter_cause_diag=0;

/* Fixed-width symbols used by the bounded, read-only SWD diagnostic tool. */
volatile uint32_t bmi088_sample_sequence_diag = 0u;
volatile uint32_t bmi088_sample_timestamp_ms_diag = 0u;
volatile uint32_t imu_sample_valid_diag = 0u;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

// 用于获取两次采样之间的时间间隔
static uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);
static void INSInvalidateControlSnapshot(void);
static void INSPublishControlSnapshot(void);

// 用初始加速度估计倾斜角；静止时加速度能表示重力方向，偏航角仍从0开始。
static MathStatus_t InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系重力方向；这里的向量长度为1
    float axis_rot[3] = {0};           // 旋转轴
    // 最多尝试200次，收集到100份有效样本后再求平均。
    uint8_t successful_samples = 0u;
    for (uint16_t attempt = 0; attempt < 200u && successful_samples < 100u; ++attempt)
    {
        if (BMI088Acquire(bmi088_ins, &bmi088_data) != BMI088_ACQUIRE_OK ||
            !BMI088SampleIsFresh(&bmi088_data, HAL_GetTick()))
        {
            TaskMonitorDelayMs(TASK_MONITOR_INS, 1u);
            continue;
        }
        acc_init[X] += bmi088_data.acc[X];
        acc_init[Y] += bmi088_data.acc[Y];
        acc_init[Z] += bmi088_data.acc[Z];
        successful_samples++;
        TaskMonitorDelayMs(TASK_MONITOR_INS, 1u);
    }
    // 只有样本收齐才使用平均值；不足100份时使用水平姿态作为起点。
    if (successful_samples == 100u)
        for (uint8_t i = 0; i < 3; ++i)
            acc_init[i] /= successful_samples;
    else
    {
        acc_init[X] = 0.0f;
        acc_init[Y] = 0.0f;
        acc_init[Z] = 1.0f;
    }
    // 加速度长度接近0时改用向上的方向，避免后续除以0。
    float acc_length;
    MathStatus_t math_status = Vec3NormF32(acc_init, &acc_length);
    if (math_status != MATH_OK) return math_status;
    if (acc_length < 1e-6f)
    {
        acc_init[0] = 0.0f;
        acc_init[1] = 0.0f;
        acc_init[2] = 1.0f;
    }
    // 除以向量长度，把加速度方向的长度调整为1。
    math_status = Vec3NormalizeF32(acc_init, acc_init);
    if (math_status != MATH_OK) return math_status;
    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float dot;
    math_status = Vec3DotF32(acc_init, gravity_norm, &dot);
    if (math_status != MATH_OK) return math_status;
    if (dot > 1.0f)
        dot = 1.0f;
    else if (dot < -1.0f)
        dot = -1.0f;
    float angle = acosf(dot);

    math_status = Vec3CrossF32(acc_init, gravity_norm, axis_rot);
    if (math_status != MATH_OK) return math_status;
    float axis_length;
    math_status = Vec3NormF32(axis_rot, &axis_length);
    if (math_status != MATH_OK) return math_status;
    if (axis_length < 1e-6f)
    {
        // 加速度与重力平行: 水平时为初始姿态(单位四元数), 倒置时绕 X 轴转 180°
        if (dot < 0.0f)
        {
            axis_rot[0] = 1.0f;
            axis_rot[1] = 0.0f;
            axis_rot[2] = 0.0f;
        }
        else
        {
            angle = 0.0f;
        }
    }
    else
    {
        // 旋转轴也要调整为长度1，才能用于四元数计算。
        math_status = Vec3NormalizeF32(axis_rot, axis_rot);
        if (math_status != MATH_OK) return math_status;
    }

    float candidate[4] = {cosf(angle / 2.0f), 0.0f, 0.0f, 0.0f};
    for (uint8_t i = 0; i < 2; ++i)
        candidate[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
    // 整份初值有限后才交给EKF，失败不发布部分姿态。
    for (uint8_t i = 0u; i < 4u; ++i)
        if (!isfinite(candidate[i])) return MATH_NONFINITE;
    memcpy(init_q4, candidate, sizeof(candidate));
    return MATH_OK;
}

attitude_t *INS_Init(void)
{
    INSInvalidateControlSnapshot();
    if (ins_init_result != 0u)
        return (ins_init_result == 3u) ? NULL : (attitude_t *)&INS.Gyro;
    INS.init = 1u;
    // 初始化传感器或读取参数前先关闭加热输出。
    IMUHeaterForceOff();
    IMUHeaterInit();
    bmi088_sample_sequence_diag = 0u;
    bmi088_sample_timestamp_ms_diag = 0u;
    imu_sample_valid_diag = 0u;

    BMI088_Init_Config_s imu_cfg = {
        .work_mode = BMI088_BLOCK_PERIODIC_MODE,
        .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
        .spi_gyro_config = {
            .spi_handle = &hspi2,
            .GPIOx = CS2_GYRO_GPIO_Port,
            .cs_pin = CS2_GYRO_Pin,
            .spi_work_mode = SPI_BLOCK_MODE,
        },
        .spi_acc_config = {
            .spi_handle = &hspi2,
            .GPIOx = CS2_ACCEL_GPIO_Port,
            .cs_pin = CS2_ACCEL_Pin,
            .spi_work_mode = SPI_BLOCK_MODE,
        },
        // heat_pwm_config 不配置: bmi088.c 仅在 htim 非空时注册温控 PWM, 温控由本任务自行控制
    };
    bmi088_ins = BMI088Register(&imu_cfg);
    if (bmi088_ins == NULL)
    {
        IMUHeaterForceOff();
        RobotSafetySetImuValid(0u);
        RobotSafetySetCalibValid(0u);
        INS.q[0] = 1.0f;
        INS.q[1] = 0.0f;
        INS.q[2] = 0.0f;
        INS.q[3] = 0.0f;
        INS.AccelLPF = 0.0085f;
        DWT_GetDeltaT(&INS_DWT_Count);
        BSP_LOG_EVENT_2(EV_MODULES_IMU_INS_TASK_C_01, BSP_LOG_FIELD_HEX32((unsigned)bmi088_init_error), BSP_LOG_FIELD_U32((unsigned)bmi088_init_retry_count));
        // 保留姿态存储供任务继续运行，但安全状态会记录传感器无效。
        ins_init_result = 1u;
        return (attitude_t *)&INS.Gyro;
    }
    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    float init_quaternion[4] = {0};
    if (InitQuaternion(init_quaternion) != MATH_OK ||
        IMU_QuaternionEKF_Init(init_quaternion, &qekf_config, qekf_workspace, sizeof(qekf_workspace)) != KF_OK)
    {
        IMUHeaterForceOff();
        RobotSafetySetImuValid(0u);
        RobotSafetySetCalibValid(0u);
        ins_init_result = 3u;
        return NULL;
    }
    // 加速度读数波动较大，后续用低通滤波减小高频变化。
    INS.AccelLPF = 0.0085;
    DWT_GetDeltaT(&INS_DWT_Count);
    ins_init_result = 2u;
    return (attitude_t *)&INS.Gyro; // @todo: 这里偷懒了,不要这样做! 修改INT_t结构体可能会导致异常,待修复.
}

attitude_t *INS_GetAttitude(void)
{
    if (!INS.init)
        return NULL;
    return (attitude_t *)&INS.Gyro;
}

attitude_t *INS_GetStorage(void)
{
    return (attitude_t *)&INS.Gyro;
}

uint8_t INSGetControlSnapshot(INS_Control_Snapshot_s *snapshot)
{
    if (snapshot == NULL)
        return 0u;
    taskENTER_CRITICAL();
    *snapshot = ins_control_snapshot;
    taskEXIT_CRITICAL();
    return snapshot->valid;
}

static void INSInvalidateControlSnapshot(void)
{
    taskENTER_CRITICAL();
    ins_control_snapshot.valid = 0u;
    taskEXIT_CRITICAL();
}

static void INSPublishControlSnapshot(void)
{
    INS_Control_Snapshot_s candidate = {
        .yaw_rad = INS.Yaw * DEG_2_RAD,
        .yaw_total_rad = INS.YawTotalAngle * DEG_2_RAD,
        .pitch_rad = INS.Pitch * DEG_2_RAD,
        .gyro_rad_s = {INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z]},
        .timestamp_ms = bmi088_data.timestamp_ms,
        .sequence = bmi088_data.sequence,
        .valid = 1u,
    };
    taskENTER_CRITICAL();
    ins_control_snapshot = candidate;
    taskEXIT_CRITICAL();
}

   /* INS_Task 每1ms运行一次；这里的计数和温控间隔都按任务调用次数计算。 */
/* ---------------- 性能探针(2026-09, CPU 占用排查) ----------------
   ins_prof_all   : 单次 INS_Task 总耗时
   ins_prof_read  : BMI088Acquire(传感器读取)
   ins_prof_ekf   : 四元数 EKF 更新
   ins_prof_temp  : 温控(每 2 个 tick 一次) */
volatile DWT_Probe_t ins_prof_all = {0};
volatile DWT_Probe_t ins_prof_read = {0};
volatile DWT_Probe_t ins_prof_ekf = {0};
volatile DWT_Probe_t ins_prof_temp = {0};

void INS_Task(void)
{
    static uint32_t count = 0;
    static float filter_elapsed_s=0.0f;
    const float gravity[3] = {0, 0, 9.81f};
    uint32_t probe_all = DWT_ProbeStart();
    uint32_t probe_seg;

    /* 左键或右键长按会提出标定请求；由本任务执行，避免和采样同时占用 SPI。 */
    if (BMI088CalibService())
    {
        INSInvalidateControlSnapshot();
        DWT_GetDeltaT(&INS_DWT_Count); /* 丢掉标定耗时, 否则 EKF 会拿到一个巨大的 dt */
        DWT_ProbeDone(&ins_prof_all, probe_all);
        return;
    }

    dt = DWT_GetDeltaT(&INS_DWT_Count);
    filter_elapsed_s+=dt;
    t += dt;

    // ins update
    if ((count % 1) == 0)
    {
        probe_seg = DWT_ProbeStart();
        uint8_t acq_ok = BMI088Acquire(bmi088_ins, &bmi088_data);
        DWT_ProbeDone(&ins_prof_read, probe_seg);

        uint8_t sample_valid = (acq_ok == BMI088_ACQUIRE_OK) &&
                               BMI088SampleIsFresh(&bmi088_data, HAL_GetTick());
        // 样本读取失败或已经过期时，清除有效标记、撤下旧姿态并关闭温控输入。
        if (!sample_valid)
        {
            INSInvalidateControlSnapshot();
            imu_sample_valid_diag = 0u;
            bmi088_data.valid = 0u;
            RobotSafetySetImuValid(0u);
            VisionUpdateTx(0u, 0.0f, 0.0f, 0.0f, 0.0f,
                           0.0f, 0.0f, 0.0f, 0.0f,
                           VISION_BULLET_SPEED_DEFAULT, 0u);
            Robot_Status_e state = RobotSafetyGetState();
            uint8_t force_off = (state == ROBOT_ESTOP || state == ROBOT_FAULT) ? 1u : 0u;
            IMUHeaterUpdate(NULL, force_off);
            goto ins_sample_processed;
        }
        bmi088_sample_sequence_diag = bmi088_data.sequence;
        bmi088_sample_timestamp_ms_diag = bmi088_data.timestamp_ms;
        INS.Accel[X] = bmi088_data.acc[X];
        INS.Accel[Y] = bmi088_data.acc[Y];
        INS.Accel[Z] = bmi088_data.acc[Z];
        INS.Gyro[X] = bmi088_data.gyro[X];
        INS.Gyro[Y] = bmi088_data.gyro[Y];
        INS.Gyro[Z] = bmi088_data.gyro[Z];

        // demo function,用于修正安装误差,可以不管,本demo暂时没用
        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
        // INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
        // INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

        // 核心函数,EKF更新四元数
        probe_seg = DWT_ProbeStart();
        KFStatus_t filter_status=IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z],
            INS.Accel[X], INS.Accel[Y], INS.Accel[Z], filter_elapsed_s, HAL_GetTick(),
            bmi088_data.timestamp_ms, bmi088_data.sequence);
        filter_elapsed_s=0.0f;
        imu_filter_mode_diag=QEKF_INS.IMU_QuaternionEKF.result.mode;
        imu_filter_age_ms_diag=QEKF_INS.IMU_QuaternionEKF.result.prediction_age_ms;
        imu_filter_cause_diag=QEKF_INS.IMU_QuaternionEKF.result.cause;
        DWT_ProbeDone(&ins_prof_ekf, probe_seg);

        // 滤波失效立即撤销许可；严重预测失败由显式重置恢复，不重建参考姿态。
        if ((filter_status!=KF_OK && filter_status!=KF_PREDICTED) || !QEKF_INS.IMU_QuaternionEKF.result.valid)
        {
            INSInvalidateControlSnapshot();
            imu_sample_valid_diag=0u;
            RobotSafetySetImuValid(0u);
            VisionUpdateTx(0u,0,0,0,0,0,0,0,0,VISION_BULLET_SPEED_DEFAULT,0u);
            IMUHeaterUpdate(NULL,1u);
            goto ins_sample_processed;
        }

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        // 先减去当前姿态下的重力，再滤掉高频变化，得到机体运动加速度。
        for (uint8_t i = 0; i < 3; ++i)
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

        // 任一姿态或角速度不是有效数值时，都不把这组结果送给视觉端。
        uint8_t imu_valid = sample_valid && isfinite(INS.q[0]) && isfinite(INS.q[1]) && isfinite(INS.q[2]) && isfinite(INS.q[3]) &&
                            isfinite(INS.Gyro[X]) && isfinite(INS.Gyro[Y]) && isfinite(INS.Gyro[Z]) &&
                            isfinite(INS.Yaw) && isfinite(INS.Pitch) && isfinite(INS.YawTotalAngle);
        RobotSafetySetImuValid(imu_valid);
        imu_sample_valid_diag = imu_valid;

        if (imu_valid)
        {
            /* 同一份姿态和陀螺快照供云台外环使用，并沿用 BMI088 来源时间。 */
            INSPublishControlSnapshot();
            // 姿态解算完成后填充视觉上行数据(与 sp_vision_25 GimbalToVision 对齐)
            // yaw/pitch 需转弧度, gyro 已是 rad/s; 只有READY状态才向NUC报告自瞄模式
            VisionUpdateTx(RobotSafetyIsReady() ? 1 : 0,
                           INS.q[0], INS.q[1], INS.q[2], INS.q[3],
                           INS.Yaw * DEG_2_RAD, INS.Gyro[Z],
                           INS.Pitch * DEG_2_RAD, INS.Gyro[Y],
                           VISION_BULLET_SPEED_DEFAULT, 0);
        }
        else
            INSInvalidateControlSnapshot();
        ins_sample_processed:;
    }

    // 每两次任务调用更新一次温控；急停或故障时强制关闭加热。
    if ((count % 2) == 0)
    {
        probe_seg = DWT_ProbeStart();
        Robot_Status_e state = RobotSafetyGetState();
        uint8_t force_off = (state == ROBOT_ESTOP || state == ROBOT_FAULT || !imu_sample_valid_diag) ? 1 : 0;
        IMUHeaterUpdate(&bmi088_data, force_off);
        DWT_ProbeDone(&ins_prof_temp, probe_seg);
    }

    if ((count++ % 1000) == 0)
    {
        // 1Hz 可以加入monitor函数,检查IMU是否正常运行/离线
    }
    DWT_ProbeDone(&ins_prof_all, probe_all);
}


void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}


void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------


void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}


void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}


void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
