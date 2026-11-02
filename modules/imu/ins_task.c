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
#include "platform_sync.h"
#include "platform_probe.h"
#include "bsp_log.h"
#include "controller.h"
#include "attitude_filter.h"
#include "vector3.h"
#include "../general_def.h"
#include "../../application/config/build_selection.h"
// 不含视觉功能的板卡仍运行姿态任务，但不依赖视觉链路发送模块。
#if MC02_FEATURE_VISION
#include "master_process.h"
#endif
#include "fast_trig.h"
#include "ins_sensor_port.h"
#include "robot_safety.h"
#if MC02_FEATURE_HEATER
#include "imu_heater.h"
#endif
#include "task_monitor.h"
#include <math.h>
#include <float.h>
#include <string.h>

#define DEG_2_RAD (1.0f / RAD_2_DEGREE) // 角度转弧度, sp_vision_25 协议为弧度制

static INS_t INS;
static IMU_Param_t IMU_Param;
static InsSensor *bmi088_ins;
static PlatformImuPort_t ins_sensor_port = PLATFORM_IMU_INVALID; // 启动前由车型选择。
static BMI088_Data_t bmi088_data;
static uint8_t ins_init_result;
static INS_Control_Snapshot_s ins_control_snapshot;
// 控制快照由任务读取和更新；任务临界区与原毫秒时基由平台提供。
static AttitudeFilter_t attitude_filter;
static _Alignas(ATTITUDE_FILTER_STORAGE_ALIGNMENT) uint8_t attitude_filter_workspace[ATTITUDE_FILTER_STORAGE_BYTES];
#ifndef MC02_IMU_BACKEND_FUSION
#define MC02_IMU_BACKEND_FUSION 0
#endif
static const AttitudeFilterConfig_t attitude_filter_config={
    .backend=MC02_IMU_BACKEND_FUSION ? ATTITUDE_BACKEND_FUSION : ATTITUDE_BACKEND_QUATERNION_EKF,
    .max_prediction_ms=20u,
    .max_measurement_age_ms=20u,
    .max_dt_s=FLT_MAX,
    .qekf={10.0f,0.001f,1000000.0f,1.0f,0.0f,2.0f},
    .fusion={1000.0f,0.5f,2000.0f,10.0f,5.0f,7.0f,12.5f},
};
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
        if (!InsSensorAcquire(bmi088_ins, &bmi088_data) ||
            !BMI088SampleIsFresh(&bmi088_data, PlatformTimeNowMs()))
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

uint8_t INS_ConfigureSensor(PlatformImuPort_t port)
{
    if (ins_init_result != 0u || !InsSensorIsBound(port)) return 0u; // 初始化后不切换，未绑定时保留旧选择。
    ins_sensor_port = port; // 只保存逻辑编号，不在此阶段访问硬件。
    return 1u;
}

attitude_t *INS_Init(void)
{
    INSInvalidateControlSnapshot();
    if (ins_init_result != 0u)
        return (ins_init_result == 3u) ? NULL : (attitude_t *)&INS.Gyro;
    INS.init = 1u;
#if MC02_FEATURE_HEATER
    // 初始化传感器或读取参数前先关闭加热输出。
    IMUHeaterForceOff();
    IMUHeaterInit();
#endif
    bmi088_sample_sequence_diag = 0u;
    bmi088_sample_timestamp_ms_diag = 0u;
    imu_sample_valid_diag = 0u;

    bmi088_ins = InsSensorRegister(ins_sensor_port); // 由板级后端绑定所选传感器和总线。
    if (bmi088_ins == NULL)
    {
#if MC02_FEATURE_HEATER
        IMUHeaterForceOff();
#endif
        RobotSafetySetImuValid(0u);
        RobotSafetySetCalibValid(0u);
        INS.q[0] = 1.0f;
        INS.q[1] = 0.0f;
        INS.q[2] = 0.0f;
        INS.q[3] = 0.0f;
        INS.AccelLPF = 0.0085f;
        PlatformProbeDeltaSeconds(&INS_DWT_Count);
        uint32_t error = 0u, retries = 0u;
        InsSensorGetInitDiagnostics(&error, &retries); // 失败日志保留原有诊断信息。
        BSP_LOG_EVENT_2(EV_MODULES_IMU_INS_TASK_C_01, BSP_LOG_FIELD_HEX32((unsigned)error), BSP_LOG_FIELD_U32((unsigned)retries));
        // 保留姿态存储供任务继续运行，但安全状态会记录传感器无效。
        ins_init_result = 3u;
        return NULL;
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
        AttitudeFilterInit(&attitude_filter,&attitude_filter_config,init_quaternion,
            attitude_filter_workspace,sizeof(attitude_filter_workspace)) != KF_OK)
    {
#if MC02_FEATURE_HEATER
        IMUHeaterForceOff();
#endif
        RobotSafetySetImuValid(0u);
        RobotSafetySetCalibValid(0u);
        ins_init_result = 3u;
        return NULL;
    }
    // 加速度读数波动较大，后续用低通滤波减小高频变化。
    INS.AccelLPF = 0.0085;
    PlatformProbeDeltaSeconds(&INS_DWT_Count);
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
    PlatformTaskCriticalEnter();
    *snapshot = ins_control_snapshot;
    PlatformTaskCriticalExit();
    return snapshot->valid;
}

static void INSInvalidateControlSnapshot(void)
{
    PlatformTaskCriticalEnter();
    ins_control_snapshot.valid = 0u;
    PlatformTaskCriticalExit();
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
    PlatformTaskCriticalEnter();
    ins_control_snapshot = candidate;
    PlatformTaskCriticalExit();
}

   /* INS_Task 每1ms运行一次；这里的计数和温控间隔都按任务调用次数计算。 */
/* ---------------- 性能探针(2026-09, CPU 占用排查) ----------------
   ins_prof_all   : 单次 INS_Task 总耗时
   ins_prof_read  : InsSensorAcquire(传感器读取)
   ins_prof_ekf   : 四元数 EKF 更新
   ins_prof_temp  : 温控(每 2 个 tick 一次) */
volatile PlatformProbe_t ins_prof_all = {0};
volatile PlatformProbe_t ins_prof_read = {0};
volatile PlatformProbe_t ins_prof_ekf = {0};
volatile PlatformProbe_t ins_prof_temp = {0};

void INS_Task(void)
{
    static uint32_t count = 0;
    static float filter_elapsed_s=0.0f;
    const float gravity[3] = {0, 0, 9.81f};
    uint32_t probe_all = PlatformProbeStart();
    uint32_t probe_seg;

    /* 左键或右键长按会提出标定请求；由本任务执行，避免和采样同时占用 SPI。 */
    if (InsSensorCalibrationService())
    {
        INSInvalidateControlSnapshot();
        PlatformProbeDeltaSeconds(&INS_DWT_Count); /* 丢掉标定耗时, 否则 EKF 会拿到一个巨大的 dt */
        PlatformProbeDone(&ins_prof_all, probe_all);
        return;
    }

    dt = PlatformProbeDeltaSeconds(&INS_DWT_Count);
    filter_elapsed_s+=dt;
    t += dt;

    // ins update
    if ((count % 1) == 0)
    {
        probe_seg = PlatformProbeStart();
        uint8_t acq_ok = InsSensorAcquire(bmi088_ins, &bmi088_data);
        PlatformProbeDone(&ins_prof_read, probe_seg);

        uint8_t sample_valid = (acq_ok != 0u) &&
                               BMI088SampleIsFresh(&bmi088_data, PlatformTimeNowMs());
        for (uint8_t axis = 0u; axis < 3u; ++axis)
            sample_valid = (uint8_t)(sample_valid && isfinite(bmi088_data.acc[axis]));

#if defined(MC02_VISION_PROTOCOL_V2) && MC02_FEATURE_VISION
        VisionIMUSample_t vision_imu_sample = {0};
        if (sample_valid != 0u)
        {
            vision_imu_sample.valid = 1u;
            vision_imu_sample.sample_sequence = bmi088_data.sequence;
            vision_imu_sample.sample_time_ms = bmi088_data.timestamp_ms;
            memcpy(vision_imu_sample.acceleration_m_s2, bmi088_data.acc,
                   sizeof(vision_imu_sample.acceleration_m_s2));
        }
        // 原始加速度状态独立于EKF结果；坏样本立即发布无效标记，避免远端沿用旧值。
        (void)VisionUpdateIMUSample(&vision_imu_sample);
#endif

        // 样本读取失败或已经过期时，清除有效标记、撤下旧姿态并关闭温控输入。
        if (!sample_valid)
        {
            INSInvalidateControlSnapshot();
            imu_sample_valid_diag = 0u;
            bmi088_data.valid = 0u;
            RobotSafetySetImuValid(0u);
#if MC02_FEATURE_VISION
            VisionUpdateTx(0u, 0.0f, 0.0f, 0.0f, 0.0f,
                           0.0f, 0.0f, 0.0f, 0.0f,
                           VISION_BULLET_SPEED_DEFAULT, 0u);
#endif
#if MC02_FEATURE_HEATER
            Robot_Status_e state = RobotSafetyGetState();
            uint8_t force_off = (state == ROBOT_ESTOP || state == ROBOT_FAULT) ? 1u : 0u;
            IMUHeaterUpdate(NULL, force_off);
#endif
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

        // 完整BMI088样本通过同一入口交给所选后端。
        probe_seg = PlatformProbeStart();
        AttitudeFilterInput_t filter_input={
            .dt_s=filter_elapsed_s,
            .now_ms=PlatformTimeNowMs(),
            .sample_time_ms=bmi088_data.timestamp_ms,
            .sequence=bmi088_data.sequence,
            .accelerometer_valid=sample_valid,
        };
        memcpy(filter_input.gyroscope_rad_s,INS.Gyro,sizeof(filter_input.gyroscope_rad_s));
        memcpy(filter_input.accelerometer_m_s2,INS.Accel,sizeof(filter_input.accelerometer_m_s2));
        AttitudeFilterState_t filter_state;
        KFStatus_t filter_status=AttitudeFilterUpdate(&attitude_filter,&filter_input,&filter_state);
        filter_elapsed_s=0.0f;
        imu_filter_mode_diag=filter_state.filter.mode;
        imu_filter_age_ms_diag=filter_state.filter.prediction_age_ms;
        imu_filter_cause_diag=filter_state.filter.cause;
        PlatformProbeDone(&ins_prof_ekf, probe_seg);

        // 滤波失效立即撤销许可；严重预测失败由显式重置恢复，不重建参考姿态。
        if ((filter_status!=KF_OK && filter_status!=KF_PREDICTED) || !filter_state.filter.valid)
        {
            INSInvalidateControlSnapshot();
            imu_sample_valid_diag=0u;
            RobotSafetySetImuValid(0u);
#if MC02_FEATURE_VISION
            VisionUpdateTx(0u,0,0,0,0,0,0,0,0,VISION_BULLET_SPEED_DEFAULT,0u);
#endif
#if MC02_FEATURE_HEATER
            IMUHeaterUpdate(NULL,1u);
#endif
            goto ins_sample_processed;
        }

        memcpy(INS.q,filter_state.quaternion,sizeof(INS.q));

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

        INS.Yaw = filter_state.yaw_deg;
        INS.Pitch = filter_state.pitch_deg;
        INS.Roll = filter_state.roll_deg;
        INS.YawTotalAngle = filter_state.yaw_total_deg;

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
#if MC02_FEATURE_VISION
            VisionUpdateTx(RobotSafetyIsReady() ? 1 : 0,
                           INS.q[0], INS.q[1], INS.q[2], INS.q[3],
                           INS.Yaw * DEG_2_RAD, INS.Gyro[Z],
                           INS.Pitch * DEG_2_RAD, INS.Gyro[Y],
                           VISION_BULLET_SPEED_DEFAULT, 0);
#endif
        }
        else
            INSInvalidateControlSnapshot();
        ins_sample_processed:;
    }

#if MC02_FEATURE_HEATER
    // 每两次任务调用更新一次温控；急停或故障时强制关闭加热。
    if ((count % 2) == 0)
    {
        probe_seg = PlatformProbeStart();
        Robot_Status_e state = RobotSafetyGetState();
        uint8_t force_off = (state == ROBOT_ESTOP || state == ROBOT_FAULT || !imu_sample_valid_diag) ? 1 : 0;
        IMUHeaterUpdate(&bmi088_data, force_off);
        PlatformProbeDone(&ins_prof_temp, probe_seg);
    }
#endif

    if ((count++ % 1000) == 0)
    {
        // 1Hz 可以加入monitor函数,检查IMU是否正常运行/离线
    }
    PlatformProbeDone(&ins_prof_all, probe_all);
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
        cosYaw = FastTrigCosF32(param->Yaw / 57.295779513f); // yaw 偏角转弧度并求余弦。
        cosPitch = FastTrigCosF32(param->Pitch / 57.295779513f); // pitch 偏角转弧度并求余弦。
        cosRoll = FastTrigCosF32(param->Roll / 57.295779513f); // roll 偏角转弧度并求余弦。
        sinYaw = FastTrigSinF32(param->Yaw / 57.295779513f); // yaw 偏角转弧度并求正弦。
        sinPitch = FastTrigSinF32(param->Pitch / 57.295779513f); // pitch 偏角转弧度并求正弦。
        sinRoll = FastTrigSinF32(param->Roll / 57.295779513f); // roll 偏角转弧度并求正弦。

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll; // 原 X 对目标 X 的系数。
        c_12 = cosPitch * sinYaw; // 原 Y 对目标 X 的系数。
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch; // 原 Z 对目标 X 的系数。
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw; // 原 X 对目标 Y 的系数。
        c_22 = cosYaw * cosPitch; // 原 Y 对目标 Y 的系数。
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch; // 原 Z 对目标 Y 的系数。
        c_31 = -cosPitch * sinRoll; // 原 X 对目标 Z 的系数。
        c_32 = sinPitch; // 原 Y 对目标 Z 的系数。
        c_33 = cosPitch * cosRoll; // 原 Z 对目标 Z 的系数。
        param->flag = 0; // 本次已应用强制更新请求。
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i) // 三轴先缩放并留存，避免原地写入覆盖后续输入。
        gyro_temp[i] = gyro[i] * param->scale[i]; // 应用该轴的陀螺仪比例系数。

    // 旋转矩阵第一行将原始三轴投影到目标 X 轴。
    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    // 第二行将原始三轴投影到目标 Y 轴。
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    // 第三行将原始三轴投影到目标 Z 轴。
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i) // 先复制三轴，后续原地写入不会破坏输入。
        accel_temp[i] = accel[i]; // 保存当前轴的加速度。

    // 第一行计算目标 X 轴加速度。
    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    // 第二行计算目标 Y 轴加速度。
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    // 第三行计算目标 Z 轴加速度。
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw; // 保存本次 yaw 偏角，供下次判断是否重算矩阵。
    lastPitchOffset = param->Pitch; // 保存本次 pitch 偏角。
    lastRollOffset = param->Roll; // 保存本次 roll 偏角。
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
    Yaw /= 57.295779513f; // 偏航输入从度转换为弧度。
    Pitch /= 57.295779513f; // 俯仰输入从度转换为弧度。
    Roll /= 57.295779513f; // 横滚输入从度转换为弧度。
    cosPitch = FastTrigCosF32(Pitch / 2); // 俯仰半角余弦用于组合四元数。
    cosYaw = FastTrigCosF32(Yaw / 2); // 偏航半角余弦用于组合四元数。
    cosRoll = FastTrigCosF32(Roll / 2); // 横滚半角余弦用于组合四元数。
    sinPitch = FastTrigSinF32(Pitch / 2); // 俯仰半角正弦用于组合四元数。
    sinYaw = FastTrigSinF32(Yaw / 2); // 偏航半角正弦用于组合四元数。
    sinRoll = FastTrigSinF32(Roll / 2); // 横滚半角正弦用于组合四元数。
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw; // 写入四元数 w 分量。
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw; // 写入四元数 x 分量。
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw; // 写入四元数 y 分量。
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw; // 写入四元数 z 分量。
}
