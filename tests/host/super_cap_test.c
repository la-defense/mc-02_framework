/** @file super_cap_test.c
 * @brief 通过生产接口检查原始反馈、长度及过期处理；CAN和时钟由主机模型提供。
 */
#include "super_cap.h"
#include "test_check.h"
#include <pthread.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

pthread_mutex_t super_cap_test_mutex = PTHREAD_MUTEX_INITIALIZER;
static FDCAN_HandleTypeDef bus;
static CANInstance can;
static uint8_t sent[8];
static unsigned submissions;
static CAN_TxSubmitStatus_t submit_result = CAN_TX_SUBMIT_ACCEPTED;
static unsigned allocations, frees;
static uint8_t fail_allocation, fail_registration;
static atomic_int writer_done;
static atomic_uint reader_errors;

void *__real_malloc(size_t size);
void *__real_calloc(size_t count, size_t size);
void __real_free(void *pointer);
void *__wrap_malloc(size_t size)
{
    if (fail_allocation) return NULL;
    void *pointer = __real_malloc(size);
    if (pointer != NULL) ++allocations;
    return pointer;
}
void *__wrap_calloc(size_t count, size_t size)
{
    if (fail_allocation) return NULL;
    void *pointer = __real_calloc(count, size);
    if (pointer != NULL) ++allocations;
    return pointer;
}
void __wrap_free(void *pointer)
{
    if (pointer != NULL) ++frees;
    __real_free(pointer);
}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (fail_registration) return NULL;
    can.can_handle = config->can_handle;
    can.tx_id = config->tx_id;
    can.can_module_callback = config->can_module_callback;
    can.id = config->id;
    return &can;
}

CAN_TxSubmitStatus_t CANSubmitPeriodic(FDCAN_HandleTypeDef *handle, const void *source,
    uint32_t slot, const CAN_TxFrame_t *frame, CAN_TxPriority_t priority,
    uint32_t deadline_ms, uint32_t period_us, uint32_t *request_id)
{
    (void)handle; (void)source; (void)slot; (void)priority; (void)deadline_ms; (void)period_us;
    ++submissions;
    memcpy(sent, frame->data, frame->length);
    *request_id = 1u;
    return submit_result;
}

// 模拟 CAN 接收任务依次发布数字和对应的原始时间。
static void *writer(void *context)
{
    (void)context;
    for (uint32_t i = 1u; i <= 10000u; ++i) {
        memset(can.rx_buff, (uint8_t)i, 6u);
        can.rx_len = 6u;
        can.rx_timestamp_ms = i;
        can.can_module_callback(&can);
    }
    atomic_store_explicit(&writer_done, 1, memory_order_release);
    return NULL;
}

// 并行读取时核对三项数值与同一帧的时间是否对应。
static void *reader(void *context)
{
    SuperCapInstance *instance = context;
    while (!atomic_load_explicit(&writer_done, memory_order_acquire)) {
        SuperCapFeedback_t feedback;
        if (SuperCapGet(instance, 10000u, 10000u, &feedback) != SUPERCAP_OK) {
            atomic_fetch_add(&reader_errors, 1u);
            continue;
        }
        if (feedback.state != SUPERCAP_FEEDBACK_UNSEEN) {
            uint16_t expected = (uint16_t)((feedback.source_rx_ms & 0xffu) * 0x0101u);
            if (feedback.value.vol != expected || feedback.value.current != expected || feedback.value.power != expected)
                atomic_fetch_add(&reader_errors, 1u);
        }
    }
    return NULL;
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    SuperCap_Init_Config_s config = {.can_config = {.can_handle = &bus, .tx_id = 0x302u, .rx_id = 0x301u}};
    TEST_CHECK(SuperCapInit(NULL) == NULL && allocations == 0u);
    SuperCap_Init_Config_s invalid = config;
    invalid.can_config.can_handle = NULL;
    TEST_CHECK(SuperCapInit(&invalid) == NULL && allocations == 0u);
    if (strcmp(argv[1], "allocation") == 0) {
        fail_allocation = 1u;
        TEST_CHECK(SuperCapInit(&config) == NULL && allocations == 0u);
        fail_allocation = 0u;
    } else if (strcmp(argv[1], "registration") == 0) {
        fail_registration = 1u;
        TEST_CHECK(SuperCapInit(&config) == NULL && allocations == 1u && frees == 1u);
        fail_registration = 0u;
    }
    SuperCapInstance *instance = SuperCapInit(&config);
    TEST_CHECK(instance != NULL && SuperCapInit(&config) == NULL);
    if (strcmp(argv[1], "threads") == 0) {
        pthread_t producer, consumer;
        TEST_CHECK(pthread_create(&producer, NULL, writer, NULL) == 0);
        TEST_CHECK(pthread_create(&consumer, NULL, reader, instance) == 0);
        TEST_CHECK(pthread_join(producer, NULL) == 0 && pthread_join(consumer, NULL) == 0);
        TEST_CHECK(atomic_load(&reader_errors) == 0u);
        return 0;
    }
    SuperCapFeedback_t feedback;
    TEST_CHECK(SuperCapGet(instance, 0u, 50u, &feedback) == SUPERCAP_OK &&
               feedback.state == SUPERCAP_FEEDBACK_UNSEEN);
    const uint8_t data[8] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};
    memcpy(can.rx_buff, data, sizeof(data));
    for (uint8_t length = 0u; length < 6u; ++length) {
        can.rx_len = length;
        can.can_module_callback(&can);
    }
    TEST_CHECK(SuperCapGet(instance, 0u, 50u, &feedback) == SUPERCAP_OK &&
               feedback.state == SUPERCAP_FEEDBACK_UNSEEN);
    can.rx_len = 6u;
    can.can_module_callback(&can);
    TEST_CHECK(SuperCapGet(instance, 50u, 50u, &feedback) == SUPERCAP_OK &&
               feedback.state == SUPERCAP_FEEDBACK_FRESH && feedback.value.vol == 0x0102u &&
               feedback.value.current == 0x0304u && feedback.value.power == 0x0506u);
    TEST_CHECK(SuperCapGet(instance, 51u, 50u, &feedback) == SUPERCAP_OK &&
               feedback.state == SUPERCAP_FEEDBACK_EXPIRED);
    SuperCapFeedback_t previous = feedback;
    TEST_CHECK(SuperCapGet(instance, 0u, 0u, &feedback) == SUPERCAP_ERROR_ARGUMENT &&
               memcmp(&feedback, &previous, sizeof(feedback)) == 0);
    TEST_CHECK(SuperCapGet(instance, 0u, UINT32_MAX, &feedback) == SUPERCAP_ERROR_ARGUMENT &&
               memcmp(&feedback, &previous, sizeof(feedback)) == 0);
    TEST_CHECK(SuperCapGet(NULL, 0u, 50u, &feedback) == SUPERCAP_ERROR_ARGUMENT &&
               SuperCapGet(instance, 0u, 50u, NULL) == SUPERCAP_ERROR_ARGUMENT);
    can.rx_len = 7u; can.rx_timestamp_ms = 9u;
    can.can_module_callback(&can);
    TEST_CHECK(SuperCapGet(instance, 9u, 50u, &feedback) == SUPERCAP_OK &&
               feedback.state == SUPERCAP_FEEDBACK_FRESH && feedback.source_rx_ms == 9u);
    can.rx_len = 8u;
    can.rx_timestamp_ms = UINT32_MAX - 10u;
    can.can_module_callback(&can);
    TEST_CHECK(SuperCapGet(instance, 39u, 50u, &feedback) == SUPERCAP_OK &&
               feedback.state == SUPERCAP_FEEDBACK_FRESH && feedback.age_ms == 50u);
    can.rx_len = 3u; can.rx_timestamp_ms = 40u;
    can.can_module_callback(&can);
    TEST_CHECK(SuperCapGet(instance, 40u, 50u, &feedback) == SUPERCAP_OK &&
               feedback.state == SUPERCAP_FEEDBACK_EXPIRED && feedback.age_ms == 51u);
    TEST_CHECK(SuperCapSend(instance, data, 2u) == CAN_TX_SUBMIT_INVALID && submissions == 0u);
    TEST_CHECK(SuperCapSend(instance, NULL, 8u) == CAN_TX_SUBMIT_INVALID && submissions == 0u);
    TEST_CHECK(SuperCapSend(NULL, data, 8u) == CAN_TX_SUBMIT_INVALID && submissions == 0u);
    TEST_CHECK(SuperCapSend(instance, data, sizeof(data)) == CAN_TX_SUBMIT_ACCEPTED &&
               submissions == 1u && memcmp(sent, data, 8u) == 0);
    submit_result = CAN_TX_SUBMIT_FULL;
    TEST_CHECK(SuperCapSend(instance, data, sizeof(data)) == CAN_TX_SUBMIT_FULL && submissions == 2u);
    return 0;
}
