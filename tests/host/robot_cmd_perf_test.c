/** @file robot_cmd_perf_test.c
 * @brief 用生产 RobotCMDTask 和消息中心测量每批平均时间及复制字节数。
 * @note 仅用于 WSL 主机对比；主机耗时不代表 STM32H723 的执行时间。
 * @note 入口为 main()；仅在 Linux 主机编译，使用 CPU 亲和设置和 GNU 链接器包装。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#define _GNU_SOURCE

#include "test_check.h"
#include "robot_def.h"
#include "robot_cmd.h"
#include "robot_safety.h"
#include "remote_control.h"
#include "message_center.h"
#include "topics/robot_topics.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#if defined(__linux__)
#include <sched.h>
#endif

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart9;
CAN_HandleTypeDef hcan1;

static RC_ctrl_t remote_input;
static uint32_t test_tick = 100u;
static Vision_Recv_s vision_frame;
static Vision_Status_t vision_status;
#if defined(ROBOT_CMD_COUNT_COPY_BYTES)
static uint64_t copied_bytes;
static uint8_t count_copies;
#endif

void RemoteControlGetSnapshot(RemoteControlSnapshot_s *snapshot)
{
    snapshot->input = remote_input;
    snapshot->remote_online = 1u;
    snapshot->pc_valid = 1u;
}

const RC_ctrl_t *RemoteControlInit(PlatformUartPort_t handle)
{
    (void)handle;
    return &remote_input;
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *handle)
{
    (void)handle;
    return &vision_frame;
}

void VisionSend(void) {}

void VisionGetSnapshot(Vision_Recv_s *frame, Vision_Status_t *status)
{
    *frame = vision_frame;
    *status = vision_status;
}

uint32_t HAL_GetTick(void)
{
    return test_tick;
}

#if defined(ROBOT_CMD_COUNT_COPY_BYTES)
void *__real_memcpy(void *destination, const void *source, size_t size);

void *__wrap_memcpy(void *destination, const void *source, size_t size)
{
    if (count_copies != 0u)
        copied_bytes += size;
    return __real_memcpy(destination, source, size);
}
#endif

static int ThreadCpuNs(uint64_t *output)
{
    if (output == NULL)
        return 0;
    struct timespec now;
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &now) != 0)
        return 0;
    *output = (uint64_t)now.tv_sec * 1000000000u + (uint64_t)now.tv_nsec;
    return 1;
}

static int CompareU64(const void *left, const void *right)
{
    const uint64_t a = *(const uint64_t *)left;
    const uint64_t b = *(const uint64_t *)right;
    return (a > b) - (a < b);
}

static int PinToAvailableCpu(void)
{
#if defined(__linux__)
    cpu_set_t available;
    if (sched_getaffinity(0, sizeof(available), &available) != 0)
        return -1;
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu)
    {
        if (!CPU_ISSET(cpu, &available))
            continue;
        cpu_set_t selected;
        CPU_ZERO(&selected);
        CPU_SET(cpu, &selected);
        return sched_setaffinity(0, sizeof(selected), &selected) == 0 ? cpu : -1;
    }
    return -1;
#else
    return -1;
#endif
}

int main(void)
{
    enum { WARMUP_CYCLES = 5000, BATCH_COUNT = 100, BATCH_CYCLES = 50000 };
    static uint64_t batch_mean_ns[BATCH_COUNT];
    const int pinned_cpu = PinToAvailableCpu();
#if defined(__linux__)
    TEST_CHECK(pinned_cpu >= 0);
#endif

    RobotSafetyInit();
    RobotSafetySetRcOnline(1u);
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetyRequestEnable();
    RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyIsReady());
    TEST_CHECK(RobotCMDInit());

    MessagePublisher_t *gimbal_feedback = MessagePublisherRegister(
        ROBOT_TOPIC_GIMBAL_FEEDBACK, ROBOT_TOPIC_ENDPOINT_GIMBAL);
    MessagePublisher_t *chassis_feedback = MessagePublisherRegister(
        ROBOT_TOPIC_CHASSIS_FEEDBACK, ROBOT_TOPIC_ENDPOINT_CHASSIS);
    TEST_CHECK(gimbal_feedback != NULL && chassis_feedback != NULL);

    Gimbal_Upload_Data_s gimbal_data = {0};
    Chassis_Upload_Data_s chassis_data = {0};
    gimbal_data.gimbal_imu_data.YawTotalAngle = 12.0f;
    gimbal_data.gimbal_imu_data.Pitch = 3.0f;
    remote_input.rc.switch_left = RC_SW_DOWN;
    remote_input.rc.switch_right = RC_SW_MID;
    vision_status.online = 0u;
    vision_status.last_rx_ms = test_tick;

    // 先运行同一条生产控制路径，让初始化、首次参考建立和缓存装入不计入样本。
    for (uint32_t i = 0u; i < WARMUP_CYCLES; ++i)
    {
        TEST_CHECK(MessagePublishNow(gimbal_feedback, &gimbal_data) == MESSAGE_OK);
        TEST_CHECK(MessagePublishNow(chassis_feedback, &chassis_data) == MESSAGE_OK);
        RobotCMDTask();
    }

#if defined(ROBOT_CMD_COUNT_COPY_BYTES)
    copied_bytes = 0u;
#endif
    uint64_t total_elapsed_ns = 0u;
    for (size_t batch = 0u; batch < BATCH_COUNT; ++batch)
    {
        uint64_t start_ns;
        uint64_t end_ns;
        TEST_CHECK(ThreadCpuNs(&start_ns));
#if defined(ROBOT_CMD_COUNT_COPY_BYTES)
        count_copies = 1u;
#endif
        // 每组使用相同的有效输入连续处理5万轮，减少计时函数对单轮结果的影响。
        for (size_t cycle = 0u; cycle < BATCH_CYCLES; ++cycle)
            RobotCMDTask();
#if defined(ROBOT_CMD_COUNT_COPY_BYTES)
        count_copies = 0u;
#endif
        TEST_CHECK(ThreadCpuNs(&end_ns));
        const uint64_t elapsed_ns = end_ns - start_ns;
        total_elapsed_ns += elapsed_ns;
        batch_mean_ns[batch] = elapsed_ns / BATCH_CYCLES;
    }

    qsort(batch_mean_ns, BATCH_COUNT, sizeof(batch_mean_ns[0]), CompareU64);

    const size_t total_cycles = BATCH_COUNT * BATCH_CYCLES;
    // 最近秩分位数取 ceil(百分位 × 样本数) 对应的下标；P50和P95分别取第50、第95个样本。
    const size_t p50_index = ((BATCH_COUNT * 50u + 99u) / 100u) - 1u;
    const size_t p95_index = ((BATCH_COUNT * 95u + 99u) / 100u) - 1u;
    printf("{\"scenario\":\"one_board_manual_ready\",\"cpu\":%d,"
           "\"chassis_feedback_size\":%zu,"
           "\"batches\":%u,"
           "\"cycles_per_batch\":%u,\"measurement\":\"thread_cpu_time\",",
           pinned_cpu, sizeof(Chassis_Upload_Data_s),
           (unsigned)BATCH_COUNT, (unsigned)BATCH_CYCLES);
#if defined(ROBOT_CMD_COUNT_COPY_BYTES)
    const uint64_t copy_bytes_per_cycle = copied_bytes / total_cycles;
    printf("\"copy_bytes_per_cycle\":%llu,",
           (unsigned long long)copy_bytes_per_cycle);
    TEST_CHECK(copy_bytes_per_cycle != 0u);
#else
    printf("\"copy_bytes_per_cycle\":null,");
#endif
    printf("\"mean_ns_per_cycle\":%llu,\"p50_batch_mean_ns\":%llu,"
           "\"p95_batch_mean_ns\":%llu,\"max_batch_mean_ns\":%llu}\n",
           (unsigned long long)(total_elapsed_ns / total_cycles),
           (unsigned long long)batch_mean_ns[p50_index],
           (unsigned long long)batch_mean_ns[p95_index],
           (unsigned long long)batch_mean_ns[BATCH_COUNT - 1u]);
    TEST_CHECK(batch_mean_ns[BATCH_COUNT - 1u] != 0u);
    return 0;
}
