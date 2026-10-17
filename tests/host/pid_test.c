/** @file pid_test.c
 * @brief 通过生产 PID 接口检查数值、失败保留、重置及不同实例交错计算。
 * 所有检查在 Release 中继续执行；Linux 另以两个线程同时计算独立实例。
 */
#include "controller.h"
#include "motor_pid.h"
#include "test_check.h"
#include <float.h>
#include <math.h>
#include <string.h>
#ifdef PID_TEST_THREADS
#include <pthread.h>
#endif

// 固定增益对应手算样例，时间参数均以秒表示。
static PID_Init_Config_s config(void)
{
    return (PID_Init_Config_s){.Kp = 2.0f, .Ki = 3.0f, .Kd = 0.1f,
        .MaxOut = 1000.0f, .IntegralLimit = 0.5f,
        .DtMin_s = 0.00001f, .DtMax_s = 0.05f};
}

static void near(float actual, float expected)
{
    TEST_CHECK(isfinite(actual));
    TEST_CHECK(fabsf(actual - expected) <= 0.0001f * fmaxf(1.0f, fabsf(expected)));
}

static float calculate(PIDInstance *pid, float measure, float ref, float dt)
{
    float output = -123.0f;
    PID_Status_e status = PIDCalculate(pid, measure, ref, dt, &output);
    TEST_CHECK(status == PID_OK);
    return output;
}

static void numeric(void)
{
    PIDInstance pid = {0};
    PID_Init_Config_s c = config();
    PID_Status_e status = PIDInit(&pid, &c);
    TEST_CHECK(status == PID_OK);
    // 第一轮为 P=4、I=0.06、D=20；第二轮为 P=3、I=0.15、D=-2.5。
    near(calculate(&pid, 1.0f, 3.0f, 0.01f), 24.06f);
    near(calculate(&pid, 1.5f, 3.0f, 0.02f), 0.65f);
    PID_State_t state;
    status = PIDGetState(&pid, &state);
    TEST_CHECK(status == PID_OK);
    near(state.Iout, 0.15f);
    state.Output = 999.0f;
    status = PIDGetState(&pid, &state);
    TEST_CHECK(status == PID_OK);
    near(state.Output, 0.65f);
    status = PIDReset(&pid);
    TEST_CHECK(status == PID_OK);
    near(calculate(&pid, 1.0f, 3.0f, 0.01f), 24.06f);
    // 允许上下限本身，超界检查另在失败用例执行。
    (void)calculate(&pid, 1.0f, 3.0f, c.DtMin_s);
    (void)calculate(&pid, 1.0f, 3.0f, c.DtMax_s);
    status = PIDInit(&pid, &pid.config);
    TEST_CHECK(status == PID_OK);
    near(calculate(&pid, 1.0f, 3.0f, 0.01f), 24.06f);
}

static void improvements(void)
{
    PIDInstance pid = {0};
    PID_Init_Config_s c = config();
    c.Improve = PID_Trapezoid_Intergral;
    PID_Status_e status = PIDInit(&pid, &c);
    TEST_CHECK(status == PID_OK);
    near(calculate(&pid, 1.0f, 3.0f, 0.01f), 24.03f);
    near(calculate(&pid, 1.5f, 3.0f, 0.02f), 0.635f);
    c.Improve = PID_Derivative_On_Measurement;
    status = PIDInit(&pid, &c);
    TEST_CHECK(status == PID_OK);
    near(calculate(&pid, 1.0f, 3.0f, 0.01f), -5.94f);
    c.Improve = PID_DerivativeFilter;
    c.Derivative_LPF_RC = 0.01f;
    status = PIDInit(&pid, &c);
    TEST_CHECK(status == PID_OK);
    near(calculate(&pid, 1.0f, 3.0f, 0.01f), 14.06f);
    near(calculate(&pid, 1.5f, 3.0f, 0.02f), 4.8166667f);
    c.Ki = c.Kd = 0.0f;
    c.Improve = PID_OutputFilter;
    c.Output_LPF_RC = 0.01f;
    status = PIDInit(&pid, &c);
    TEST_CHECK(status == PID_OK);
    near(calculate(&pid, 1.0f, 3.0f, 0.01f), 2.0f);
    near(calculate(&pid, 1.0f, 3.0f, 0.02f), 3.3333333f);
    c.Kp = 0.0f;
    c.Ki = 10.0f;
    c.Improve = PID_Integral_Limit;
    status = PIDInit(&pid, &c);
    TEST_CHECK(status == PID_OK);
    near(calculate(&pid, 0.0f, 2.0f, 0.05f), 0.5f);
    status = PIDReset(&pid);
    TEST_CHECK(status == PID_OK);
    near(calculate(&pid, 0.0f, -2.0f, 0.05f), -0.5f);
    c.Ki = 1.0f;
    c.Improve = PID_ChangingIntegrationRate;
    c.CoefA = 2.0f;
    c.CoefB = 1.0f;
    status = PIDInit(&pid, &c);
    TEST_CHECK(status == PID_OK);
    near(calculate(&pid, 0.0f, 2.0f, 0.01f), 0.02f);
    near(calculate(&pid, 0.0f, 2.0f, 0.01f), 0.03f);
    near(calculate(&pid, 0.0f, 4.0f, 0.01f), 0.03f);
    c.DeadBand = 0.5f;
    status = PIDInit(&pid, &c);
    TEST_CHECK(status == PID_OK);
    (void)calculate(&pid, 0.0f, 2.0f, 0.01f);
    near(calculate(&pid, 0.0f, 0.5f, 0.01f), 0.0f);
    near(pid.state.Iout, 0.0f);
    c = config();
    c.MaxOut = 1.0f;
    status = PIDInit(&pid, &c);
    TEST_CHECK(status == PID_OK);
    near(calculate(&pid, 0.0f, 2.0f, 0.01f), 1.0f);
    near(calculate(&pid, 0.0f, -2.0f, 0.01f), -1.0f);
}

// 对失败调用逐字节比较原实例，输出参数也必须保留哨兵值。
static void rejected(PIDInstance *pid, float measure, float ref, float dt, PID_Status_e expected)
{
    unsigned char saved[sizeof(*pid)];
    memcpy(saved, pid, sizeof(*pid));
    float output = 123.0f;
    PID_Status_e status = PIDCalculate(pid, measure, ref, dt, &output);
    TEST_CHECK(status == expected);
    TEST_CHECK(memcmp(saved, pid, sizeof(*pid)) == 0);
    near(output, 123.0f);
}

static void failures(void)
{
    PIDInstance pid = {0};
    rejected(&pid, 0.0f, 1.0f, 0.01f, PID_NOT_INITIALIZED);
    PID_Init_Config_s c = config();
    PID_Status_e status = PIDInit(&pid, &c);
    TEST_CHECK(status == PID_OK);
    (void)calculate(&pid, 0.0f, 1.0f, 0.01f);
    const float bad_dt[] = {0.0f, -0.01f, NAN, INFINITY,
        nextafterf(c.DtMin_s, 0.0f), nextafterf(c.DtMax_s, INFINITY)};
    for (unsigned i = 0; i < sizeof(bad_dt) / sizeof(bad_dt[0]); ++i)
        rejected(&pid, 0.0f, 1.0f, bad_dt[i], PID_BAD_DT);
    rejected(&pid, NAN, 1.0f, 0.01f, PID_BAD_ARGUMENT);
    rejected(&pid, 0.0f, INFINITY, 0.01f, PID_BAD_ARGUMENT);
    rejected(&pid, -FLT_MAX, FLT_MAX, 0.01f, PID_NUMERIC_ERROR);
    status = PIDCalculate(&pid, 0.0f, 1.0f, 0.01f, NULL);
    TEST_CHECK(status == PID_BAD_ARGUMENT);
    status = PIDReset(NULL);
    TEST_CHECK(status == PID_BAD_ARGUMENT);
    c.Kp = FLT_MAX;
    status = PIDInit(&pid, &c);
    TEST_CHECK(status == PID_OK);
    rejected(&pid, 0.0f, 2.0f, 0.01f, PID_NUMERIC_ERROR);
    unsigned char saved[sizeof(pid)];
    memcpy(saved, &pid, sizeof(pid));
    for (unsigned i = 0; i < 8u; ++i)
    {
        // 每次只破坏一个条件，确保原有配置不会被半份新参数覆盖。
        c = config();
        switch (i)
        {
        case 0: c.Ki = NAN; break;
        case 1: c.MaxOut = -1.0f; break;
        case 2: c.DtMin_s = 0.0f; break;
        case 3: c.DtMax_s = c.DtMin_s / 2.0f; break;
        case 4: c.Output_LPF_RC = -1.0f; break;
        case 5: c.Improve = PID_ChangingIntegrationRate; break;
        case 6: c.Improve = PID_Proportional_On_Measurement; break;
        default: c.Improve = (PID_Improvement_e)0x100u; break;
        }
        status = PIDInit(&pid, &c);
        TEST_CHECK(status == (i >= 6u ? PID_UNSUPPORTED_IMPROVEMENT : PID_BAD_CONFIG));
        TEST_CHECK(memcmp(saved, &pid, sizeof(pid)) == 0);
    }
}

static void blocked(void)
{
    PIDInstance pid = {0};
    PID_Init_Config_s c = config();
    c.Kp = 1.0f;
    c.Ki = c.Kd = 0.0f;
    c.Improve = PID_ErrorHandle;
    PID_Status_e status = PIDInit(&pid, &c);
    TEST_CHECK(status == PID_OK);
    for (unsigned i = 0; i < 501u; ++i)
        (void)calculate(&pid, 0.0f, 2.0f, 0.001f);
    TEST_CHECK(pid.state.ERRORHandler.ERRORCount == 500u);
    TEST_CHECK(pid.state.ERRORHandler.ERRORType == PID_ERROR_NONE);
    (void)calculate(&pid, 0.0f, 2.0f, 0.001f);
    TEST_CHECK(pid.state.ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR);
    status = PIDReset(&pid);
    TEST_CHECK(status == PID_OK);
    TEST_CHECK(pid.state.ERRORHandler.ERRORCount == 0u);
}

typedef struct
{
    PIDInstance pid;
    float expected;
    float sign;
} Worker;

static void *run_worker(void *argument)
{
    Worker *worker = argument;
    float output = 0.0f;
    // 使用不同误差及时间间隔持续计算，两线程只访问各自实例。
    for (unsigned i = 0; i < 20000u; ++i)
        output = calculate(&worker->pid, (float)(i % 7u), worker->sign * 8.0f,
                           i % 2u ? 0.001f : 0.002f);
    worker->expected = output;
    return NULL;
}

static void instances(void)
{
    Worker a = {.sign = 1.0f}, b = {.sign = -1.0f};
    PID_Init_Config_s c = config();
    c.Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_DerivativeFilter;
    c.Derivative_LPF_RC = 0.01f;
    PID_Status_e status = PIDInit(&a.pid, &c);
    TEST_CHECK(status == PID_OK);
    status = PIDInit(&b.pid, &c);
    TEST_CHECK(status == PID_OK);
    Worker a_serial = a, b_serial = b;
    (void)run_worker(&a_serial);
    (void)run_worker(&b_serial);
#ifdef PID_TEST_THREADS
    pthread_t ta, tb;
    int result = pthread_create(&ta, NULL, run_worker, &a);
    TEST_CHECK(result == 0);
    result = pthread_create(&tb, NULL, run_worker, &b);
    TEST_CHECK(result == 0);
    result = pthread_join(ta, NULL);
    TEST_CHECK(result == 0);
    result = pthread_join(tb, NULL);
    TEST_CHECK(result == 0);
#else
    (void)run_worker(&a);
    (void)run_worker(&b);
#endif
    near(a.expected, a_serial.expected);
    near(b.expected, b_serial.expected);
    PIDInstance a_reference = a.pid, b_reference = b.pid;
    for (unsigned i = 0; i < 200u; ++i)
    {
        // 调换调用顺序时，各实例仍应等于单独运行的结果。
        float aa = calculate(&a.pid, 1.0f, 3.0f, 0.001f);
        float bb = calculate(&b.pid, -1.0f, -3.0f, 0.002f);
        near(bb, calculate(&b_reference, -1.0f, -3.0f, 0.002f));
        near(aa, calculate(&a_reference, 1.0f, 3.0f, 0.001f));
    }
}

static void timing(void)
{
    MotorPIDClock_t a = {0}, b = {0};
    float dt = -1.0f;
    PID_Status_e status = MotorPIDBegin(&a, 100u, &dt);
    TEST_CHECK(status == PID_BAD_DT);
    near(dt, -1.0f);
    status = MotorPIDBegin(&b, 500u, &dt);
    TEST_CHECK(status == PID_BAD_DT);
    status = MotorPIDBegin(&a, 110u, &dt);
    TEST_CHECK(status == PID_OK);
    near(dt, MOTOR_PID_DT_MIN_S);
    status = MotorPIDBegin(&b, 50500u, &dt);
    TEST_CHECK(status == PID_OK);
    near(dt, MOTOR_PID_DT_MAX_S);
    status = MotorPIDBegin(&a, 1000000u, &dt);
    TEST_CHECK(status == PID_BAD_DT);
    status = MotorPIDBegin(&a, 1001000u, &dt);
    TEST_CHECK(status == PID_OK);
    near(dt, 0.001f);
    status = MotorPIDBegin(&a, 0u, &dt);
    TEST_CHECK(status == PID_BAD_DT);
    PIDInstance boundary = {0};
    PID_Init_Config_s c = config();
    status = PIDInit(&boundary, &c);
    TEST_CHECK(status == PID_OK);
    status = MotorPIDBegin(&a, 10u, &dt);
    TEST_CHECK(status == PID_OK);
    (void)calculate(&boundary, 0.0f, 1.0f, dt);
    status = MotorPIDBegin(&a, 50010u, &dt);
    TEST_CHECK(status == PID_OK);
    (void)calculate(&boundary, 0.0f, 1.0f, dt);
    PIDInstance idle = {0};
    status = MotorPIDInit(&idle, NULL, 0u);
    TEST_CHECK(status == PID_OK);
    rejected(&idle, 0.0f, 0.0f, 0.001f, PID_NOT_INITIALIZED);
}

/** @brief 按 CTest 指定场景运行生产接口回归，失败立即返回非零。 */
int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "numeric") == 0) numeric();
    else if (strcmp(argv[1], "improvements") == 0) improvements();
    else if (strcmp(argv[1], "failures") == 0) failures();
    else if (strcmp(argv[1], "blocked") == 0) blocked();
    else if (strcmp(argv[1], "instances") == 0) instances();
    else if (strcmp(argv[1], "timing") == 0) timing();
    else TEST_CHECK(0);
    return EXIT_SUCCESS;
}
