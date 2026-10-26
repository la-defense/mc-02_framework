/**
 * @file iic_registration_test.c
 * @brief 用生产 IICRegister 和可选驱动初始化入口检查分配、容量及失败回退。
 * 测试只替代 HAL、GPIO、DWT 和日志；所有场景在主机进程内运行，不访问板卡。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#include "bsp_iic.h"
#include "ist8310.h"
#include "test_check.h"
#include "tfminiplus.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *handle);

static int allocation_countdown = -1;
static size_t live_allocations;
static size_t allocation_attempts;
static uint32_t primask;
static uint32_t irq_disable_count;
static uint32_t irq_restore_count;
static uint32_t dmb_count;
static uint32_t hal_transfer_count;
static uint16_t last_iic_tx_size;
static uint16_t last_iic_rx_size;
static uint8_t last_iic_tx_data[5];
static uint8_t iic_response_data[11];
static uint16_t iic_response_size;
static uint32_t mem_read_count;
static uint32_t mem_write_count;
static uint32_t hal_delay_count;
static uint32_t dwt_delay_count;
static uint32_t gpio_register_count;
static uint32_t gpio_reset_count;
static uint32_t gpio_set_count;
static uint8_t ist8310_registers[256];
static GPIOInstance gpio_instances[4];
I2C_HandleTypeDef iic_test_tfmini_handle;

void *__real_malloc(size_t size);
void *__real_calloc(size_t count, size_t size);
void __real_free(void *pointer);

void *__wrap_malloc(size_t size)
{
    ++allocation_attempts;
    if (allocation_countdown == 0)
        return NULL;
    if (allocation_countdown > 0)
        --allocation_countdown;
    void *pointer = __real_malloc(size);
    if (pointer != NULL)
        ++live_allocations;
    return pointer;
}

void *__wrap_calloc(size_t count, size_t size)
{
    ++allocation_attempts;
    if (allocation_countdown == 0)
        return NULL;
    if (allocation_countdown > 0)
        --allocation_countdown;
    void *pointer = __real_calloc(count, size);
    if (pointer != NULL)
        ++live_allocations;
    return pointer;
}

void __wrap_free(void *pointer)
{
    if (pointer != NULL)
    {
        TEST_CHECK(live_allocations > 0u);
        --live_allocations;
    }
    __real_free(pointer);
}

uint32_t IICTestGetPrimask(void)
{
    return primask;
}

void IICTestDisableIrq(void)
{
    ++irq_disable_count;
    primask = 1u;
}

void IICTestDmb(void)
{
    ++dmb_count;
}

void IICTestSetPrimask(uint32_t value)
{
    ++irq_restore_count;
    primask = value;
}

void HAL_Delay(uint32_t delay_ms)
{
    (void)delay_ms;
    ++hal_delay_count;
}

static HAL_StatusTypeDef record_transfer(void)
{
    ++hal_transfer_count;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *handle, uint16_t address,
                                         uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)handle; (void)address; (void)timeout;
    last_iic_tx_size = size;
    if (size == sizeof(last_iic_tx_data))
        memcpy(last_iic_tx_data, data, sizeof(last_iic_tx_data));
    return record_transfer();
}

HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *handle, uint16_t address,
                                         uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)handle; (void)address; (void)timeout;
    last_iic_rx_size = size;
    // 模拟设备只写入实际提供的字节，额外数据留在源数组中供越界检查使用。
    uint16_t copy_size = iic_response_size < size ? iic_response_size : size;
    memcpy(data, iic_response_data, copy_size);
    return record_transfer();
}

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef *handle, uint16_t address,
                                                 uint8_t *data, uint16_t size, uint32_t options)
{
    (void)handle; (void)address; (void)data; (void)size; (void)options;
    return record_transfer();
}

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_DMA(I2C_HandleTypeDef *handle, uint16_t address,
                                                  uint8_t *data, uint16_t size, uint32_t options)
{
    (void)handle; (void)address; (void)data; (void)size; (void)options;
    return record_transfer();
}

HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef *handle, uint16_t address,
                                                uint8_t *data, uint16_t size, uint32_t options)
{
    (void)handle; (void)address; (void)data; (void)size; (void)options;
    return record_transfer();
}

HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_DMA(I2C_HandleTypeDef *handle, uint16_t address,
                                                 uint8_t *data, uint16_t size, uint32_t options)
{
    (void)handle; (void)address; (void)data; (void)size; (void)options;
    return record_transfer();
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *handle, uint16_t address,
                                    uint16_t register_address, uint16_t register_size,
                                    uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)handle; (void)address; (void)register_size; (void)timeout;
    ++mem_write_count;
    if (register_address < sizeof(ist8310_registers) && size > 0u)
        ist8310_registers[register_address] = data[0];
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *handle, uint16_t address,
                                   uint16_t register_address, uint16_t register_size,
                                   uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)handle; (void)address; (void)register_size; (void)timeout;
    ++mem_read_count;
    for (uint16_t i = 0u; i < size; ++i)
        data[i] = register_address + i < sizeof(ist8310_registers)
                      ? ist8310_registers[register_address + i]
                      : 0u;
    return HAL_OK;
}

void DWT_Delay(float seconds)
{
    (void)seconds;
    ++dwt_delay_count;
}

GPIOInstance *GPIORegister(GPIO_Init_Config_s *config)
{
    if (gpio_register_count >= (sizeof(gpio_instances) / sizeof(gpio_instances[0])))
        return NULL;
    GPIOInstance *instance = &gpio_instances[gpio_register_count++];
    instance->GPIOx = config->GPIOx;
    instance->GPIO_Pin = config->GPIO_Pin;
    instance->pin_state = config->pin_state;
    instance->exti_mode = config->exti_mode;
    instance->id = config->id;
    instance->gpio_model_callback = config->gpio_model_callback;
    return instance;
}

void GPIOReset(GPIOInstance *instance)
{
    (void)instance;
    ++gpio_reset_count;
}

void GPIOSet(GPIOInstance *instance)
{
    (void)instance;
    ++gpio_set_count;
}

static uint32_t receive_counts[MX_IIC_SLAVE_CNT];

static void count_receive(IICInstance *instance)
{
    uint32_t *count = (uint32_t *)instance->id;
    ++(*count);
}

static IIC_Init_Config_s make_config(I2C_HandleTypeDef *handle, uint8_t address,
                                     IIC_Work_Mode_e mode, void *id,
                                     void (*callback)(IICInstance *))
{
    IIC_Init_Config_s config = {
        .handle = handle,
        .dev_address = address,
        .work_mode = mode,
        .callback = callback,
        .id = id,
    };
    return config;
}

static void check_invalid_and_allocation_failure(void)
{
    static I2C_HandleTypeDef handle;
    IIC_Init_Config_s config = make_config(&handle, 0x10u, IIC_BLOCK_MODE, NULL, NULL);
    size_t live_before = live_allocations;
    size_t attempts_before = allocation_attempts;
    uint32_t critical_before = irq_disable_count;
    uint32_t restore_before = irq_restore_count;
    uint32_t barrier_before = dmb_count;

    TEST_CHECK(IICRegister(NULL) == NULL);
    config.handle = NULL;
    TEST_CHECK(IICRegister(&config) == NULL);
    config.handle = &handle;
    config.dev_address = 0x80u;
    TEST_CHECK(IICRegister(&config) == NULL);
    config.dev_address = 0x10u;
    config.work_mode = (IIC_Work_Mode_e)3;
    TEST_CHECK(IICRegister(&config) == NULL);
    config.work_mode = (IIC_Work_Mode_e)-1;
    TEST_CHECK(IICRegister(&config) == NULL);
    TEST_CHECK(allocation_attempts == attempts_before);
    TEST_CHECK(irq_disable_count == critical_before);

    config.work_mode = IIC_BLOCK_MODE;
    allocation_countdown = 0;
    TEST_CHECK(IICRegister(&config) == NULL);
    allocation_countdown = -1;
    TEST_CHECK(live_allocations == live_before);
    TEST_CHECK(irq_disable_count == critical_before && irq_restore_count == restore_before);

    // 分配失败不占用设备槽位，纠正分配器后同一设备可成功登记。
    size_t live_before_success = live_allocations;
    size_t attempts_before_success = allocation_attempts;
    primask = 1u;
    IICInstance *instance = IICRegister(&config);
    TEST_CHECK(instance != NULL);
    TEST_CHECK(allocation_attempts == attempts_before_success + 1u);
    TEST_CHECK(live_allocations == live_before_success + 1u);
    TEST_CHECK(instance->handle == &handle);
    TEST_CHECK(instance->dev_address == 0x20u);
    TEST_CHECK(instance->callback == NULL && instance->id == NULL);
    TEST_CHECK(instance->rx_buffer == NULL && instance->rx_len == 0u);
    TEST_CHECK(primask == 1u);
    TEST_CHECK(irq_disable_count == critical_before + 1u);
    TEST_CHECK(irq_restore_count == restore_before + 1u);
    TEST_CHECK(dmb_count == barrier_before + 1u);
    primask = 0u;
    TEST_CHECK(hal_transfer_count == 0u);

    IIC_Init_Config_s zero_address_config = config;
    zero_address_config.dev_address = 0u;
    IICInstance *zero_address = IICRegister(&zero_address_config);
    TEST_CHECK(zero_address != NULL && zero_address->dev_address == 0u);
}

static void check_duplicate_and_callback_routing(void)
{
    static I2C_HandleTypeDef first_handle;
    static I2C_HandleTypeDef second_handle;
    IIC_Init_Config_s first_config = make_config(&first_handle, 0x7Fu, IIC_BLOCK_MODE,
                                                  &receive_counts[0], count_receive);
    IICInstance *first = IICRegister(&first_config);
    TEST_CHECK(first != NULL && first->dev_address == 0xFEu);
    IICInstance saved_first = *first;

    IIC_Init_Config_s duplicate = make_config(&first_handle, 0x7Fu, IIC_DMA_MODE,
                                               &receive_counts[1], count_receive);
    size_t attempts_before = allocation_attempts;
    TEST_CHECK(IICRegister(&duplicate) == NULL);
    TEST_CHECK(allocation_attempts == attempts_before);
    TEST_CHECK(memcmp(first, &saved_first, sizeof(saved_first)) == 0);

    IIC_Init_Config_s same_bus_new_address = make_config(&first_handle, 0x10u, IIC_IT_MODE,
                                                          &receive_counts[1], count_receive);
    IICInstance *second = IICRegister(&same_bus_new_address);
    TEST_CHECK(second != NULL && second->dev_address == 0x20u);

    IIC_Init_Config_s same_address_new_bus = make_config(&second_handle, 0x7Fu, IIC_BLOCK_MODE,
                                                          &receive_counts[2], count_receive);
    IICInstance *third = IICRegister(&same_address_new_bus);
    TEST_CHECK(third != NULL && third->dev_address == first->dev_address);

    first_handle.Devaddress = second->dev_address;
    HAL_I2C_MasterRxCpltCallback(&first_handle);
    TEST_CHECK(receive_counts[0] == 0u && receive_counts[1] == 1u && receive_counts[2] == 0u);
    second_handle.Devaddress = third->dev_address;
    HAL_I2C_MasterRxCpltCallback(&second_handle);
    TEST_CHECK(receive_counts[0] == 0u && receive_counts[1] == 1u && receive_counts[2] == 1u);
    first_handle.Devaddress = first->dev_address;
    HAL_I2C_MasterRxCpltCallback(&first_handle);
    TEST_CHECK(receive_counts[0] == 1u && receive_counts[1] == 1u && receive_counts[2] == 1u);
}

static void check_exact_capacity(void)
{
    static I2C_HandleTypeDef handle;
    IICInstance *instances[MX_IIC_SLAVE_CNT] = {0};
    uint32_t counts[MX_IIC_SLAVE_CNT] = {0};
    for (uint8_t i = 0u; i < MX_IIC_SLAVE_CNT; ++i)
    {
        IIC_Init_Config_s config = make_config(&handle, (uint8_t)(0x20u + i), IIC_BLOCK_MODE,
                                               &counts[i], count_receive);
        instances[i] = IICRegister(&config);
        TEST_CHECK(instances[i] != NULL);
    }

    size_t attempts_before = allocation_attempts;
    IIC_Init_Config_s ninth = make_config(&handle, 0x30u, IIC_BLOCK_MODE, NULL, NULL);
    TEST_CHECK(IICRegister(&ninth) == NULL);
    TEST_CHECK(allocation_attempts == attempts_before);

    IIC_Init_Config_s duplicate = make_config(&handle, 0x20u, IIC_DMA_MODE, NULL, NULL);
    TEST_CHECK(IICRegister(&duplicate) == NULL);
    TEST_CHECK(allocation_attempts == attempts_before);

    // 八个有效实例均能收到对应句柄和地址的完成通知。
    for (uint8_t i = 0u; i < MX_IIC_SLAVE_CNT; ++i)
    {
        handle.Devaddress = instances[i]->dev_address;
        HAL_I2C_MasterRxCpltCallback(&handle);
        TEST_CHECK(counts[i] == 1u);
    }
}

static void check_tfmini_failure_and_recovery(void)
{
    I2C_HandleTypeDef *handle = &iic_test_tfmini_handle;
    size_t live_before = live_allocations;
    uint32_t delay_before = dwt_delay_count;

    allocation_countdown = 0;
    TEST_CHECK(TFMiniRegister(handle) == NULL);
    allocation_countdown = -1;
    TEST_CHECK(live_allocations == live_before);

    // 外层对象申请成功、I²C对象申请失败时释放外层对象。
    allocation_countdown = 1;
    TEST_CHECK(TFMiniRegister(handle) == NULL);
    allocation_countdown = -1;
    TEST_CHECK(live_allocations == live_before);
    TEST_CHECK(dwt_delay_count == delay_before);

    TEST_CHECK(TFMiniRegister(NULL) == NULL);
    TEST_CHECK(live_allocations == live_before);
    TEST_CHECK(dwt_delay_count == delay_before);

    TFMiniInstance *instance = TFMiniRegister(handle);
    TEST_CHECK(instance != NULL && instance->iic != NULL);
    TEST_CHECK(instance->iic->dev_address == 0x20u);
    TEST_CHECK(dwt_delay_count == delay_before + 1u);
    live_before = live_allocations;
    TEST_CHECK(TFMiniRegister(handle) == NULL);
    TEST_CHECK(live_allocations == live_before);
    TEST_CHECK(dwt_delay_count == delay_before + 1u);
}

static void check_tfmini_frame_bounds(void)
{
    static I2C_HandleTypeDef handle;
    IIC_Init_Config_s config = make_config(&handle, 0x10u, IIC_BLOCK_MODE, NULL, NULL);
    IICInstance *iic = IICRegister(&config);
    TEST_CHECK(iic != NULL);

    union
    {
        TFMiniInstance instance;
        uint8_t bytes[sizeof(TFMiniInstance)];
    } guarded;
    const size_t guard_offset = offsetof(TFMiniInstance, buf) + TFMINI_DATA_FRAME_SIZE;
    memset(&guarded, 0xA5, sizeof(guarded));
    guarded.instance.iic = iic;

    // 短响应只更新收到的字节；接口没有实际长度返回值，因此不据此判断读数有效。
    memset(iic_response_data, 0, sizeof(iic_response_data));
    iic_response_data[0] = 0x5Au;
    iic_response_data[1] = 0x11u;
    iic_response_data[2] = 0x34u;
    iic_response_size = 3u;
    last_iic_rx_size = 0u;
    (void)GetDistance(&guarded.instance);
    TEST_CHECK(last_iic_rx_size == TFMINI_DATA_FRAME_SIZE);
    TEST_CHECK(iic->rx_buffer == guarded.instance.buf &&
               iic->rx_len == TFMINI_DATA_FRAME_SIZE);
    TEST_CHECK(guarded.instance.buf[0] == 0x5Au && guarded.instance.buf[2] == 0x34u);
    TEST_CHECK(guarded.instance.buf[3] == 0xA5u);
    for (size_t i = guard_offset; i < sizeof(guarded.bytes); ++i)
        TEST_CHECK(guarded.bytes[i] == 0xA5u);

    // 完整 9 字节响应保持现有小端字段偏移和取数命令。
    memset(iic_response_data, 0, sizeof(iic_response_data));
    const uint8_t complete_response[TFMINI_DATA_FRAME_SIZE] =
        {0x5Au, 0x11u, 0x34u, 0x12u, 0x78u, 0x56u, 0x9Au, 0x00u, 0x00u};
    memcpy(iic_response_data, complete_response, sizeof(complete_response));
    iic_response_size = TFMINI_DATA_FRAME_SIZE;
    TEST_CHECK(GetDistance(&guarded.instance) == 4660.0f);
    TEST_CHECK(last_iic_tx_size == sizeof(last_iic_tx_data));
    const uint8_t expected_command[5] = {0x5Au, 0x05u, 0x00u, 0x06u, 0x65u};
    TEST_CHECK(memcmp(last_iic_tx_data, expected_command, sizeof(expected_command)) == 0);
    TEST_CHECK(guarded.instance.Mode == 0x9Au);
    TEST_CHECK(guarded.instance.Dist == 0x1234u);
    TEST_CHECK(guarded.instance.Strength == 0x5678u);
    for (size_t i = guard_offset; i < sizeof(guarded.bytes); ++i)
        TEST_CHECK(guarded.bytes[i] == 0xA5u);

    // 即使测试源提供 11 字节，生产接收调用仍只允许 HAL 写入 9 字节。
    iic_response_data[9] = 0xEEu;
    iic_response_data[10] = 0xFFu;
    iic_response_size = (uint16_t)sizeof(iic_response_data);
    TEST_CHECK(GetDistance(&guarded.instance) == 4660.0f);
    TEST_CHECK(last_iic_rx_size == TFMINI_DATA_FRAME_SIZE);
    TEST_CHECK(guarded.instance.buf[8] == 0x00u);
    for (size_t i = guard_offset; i < sizeof(guarded.bytes); ++i)
        TEST_CHECK(guarded.bytes[i] == 0xA5u);
}

static void check_ist8310_failure_and_recovery(void)
{
    TEST_CHECK(IST8310Init(NULL) == NULL);
    size_t live_before = live_allocations;
    uint32_t gpio_before = gpio_register_count;
    uint32_t delay_before = hal_delay_count;
    uint32_t reads_before = mem_read_count;
    uint32_t writes_before = mem_write_count;

    IST8310_Init_Config_s config;
    memset(&config, 0, sizeof(config));
    config.iic_config.handle = NULL;
    config.iic_config.dev_address = 0x0Eu;
    config.iic_config.work_mode = IIC_BLOCK_MODE;
    config.iic_config.id = &config;
    config.iic_config.callback = count_receive;
    config.gpio_conf_exti.id = &config;
    config.gpio_conf_rst.id = &config;
    IST8310_Init_Config_s saved_config = config;

    allocation_countdown = 0;
    TEST_CHECK(IST8310Init(&config) == NULL);
    allocation_countdown = -1;
    TEST_CHECK(live_allocations == live_before);
    TEST_CHECK(memcmp(&config, &saved_config, sizeof(config)) == 0);

    // I²C句柄为空时外层申请成功，但不得继续登记GPIO或访问设备。
    TEST_CHECK(IST8310Init(&config) == NULL);
    TEST_CHECK(live_allocations == live_before);
    TEST_CHECK(memcmp(&config, &saved_config, sizeof(config)) == 0);
    TEST_CHECK(gpio_register_count == gpio_before && hal_delay_count == delay_before);
    TEST_CHECK(mem_read_count == reads_before && mem_write_count == writes_before);

    // 父对象成功、I²C对象分配失败，必须释放父对象并在硬件动作前返回。
    config.iic_config.handle = &(I2C_HandleTypeDef){0};
    saved_config = config;
    allocation_countdown = 1;
    TEST_CHECK(IST8310Init(&config) == NULL);
    allocation_countdown = -1;
    TEST_CHECK(live_allocations == live_before);
    TEST_CHECK(memcmp(&config, &saved_config, sizeof(config)) == 0);
    TEST_CHECK(gpio_register_count == gpio_before && hal_delay_count == delay_before);
    TEST_CHECK(mem_read_count == reads_before && mem_write_count == writes_before);

    // 与已注册 TFmini 同句柄、同地址时拒绝，不改写 GPIO/I²C 配置或访问设备。
    config.iic_config.handle = &iic_test_tfmini_handle;
    config.iic_config.dev_address = 0x10u;
    saved_config = config;
    TEST_CHECK(IST8310Init(&config) == NULL);
    TEST_CHECK(live_allocations == live_before);
    TEST_CHECK(memcmp(&config, &saved_config, sizeof(config)) == 0);
    TEST_CHECK(gpio_register_count == gpio_before && hal_delay_count == delay_before);
    TEST_CHECK(mem_read_count == reads_before && mem_write_count == writes_before);

    static I2C_HandleTypeDef valid_handle;
    config.iic_config.handle = &valid_handle;
    config.iic_config.dev_address = 0x0Eu;
    saved_config = config;
    ist8310_registers[0] = 0x10u;
    IST8310Instance *instance = IST8310Init(&config);
    TEST_CHECK(instance != NULL && instance->iic != NULL);
    TEST_CHECK(instance->iic->dev_address == 0x1Cu);
    TEST_CHECK(mem_read_count == reads_before + 5u && mem_write_count == writes_before + 4u);
    TEST_CHECK(gpio_register_count == gpio_before + 2u);
    TEST_CHECK(gpio_reset_count == 1u && gpio_set_count == 1u);
    TEST_CHECK(hal_delay_count == delay_before + 2u);
    TEST_CHECK(config.iic_config.id == instance);
    TEST_CHECK(config.gpio_conf_exti.id == instance && config.gpio_conf_rst.id == instance);
    TEST_CHECK(config.iic_config.callback != saved_config.iic_config.callback);

    // 成功实例仍占用设备地址，重复初始化会释放新候选对象且不再复位设备。
    live_before = live_allocations;
    gpio_before = gpio_register_count;
    delay_before = hal_delay_count;
    reads_before = mem_read_count;
    writes_before = mem_write_count;
    IST8310_Init_Config_s duplicate_config = saved_config;
    duplicate_config.iic_config.handle = &valid_handle;
    TEST_CHECK(IST8310Init(&duplicate_config) == NULL);
    TEST_CHECK(live_allocations == live_before);
    TEST_CHECK(gpio_register_count == gpio_before && hal_delay_count == delay_before);
    TEST_CHECK(mem_read_count == reads_before && mem_write_count == writes_before);
    TEST_CHECK(duplicate_config.iic_config.handle == &valid_handle);
    TEST_CHECK(duplicate_config.gpio_conf_exti.id == saved_config.gpio_conf_exti.id);

    // 表满时两个调用方都释放外层对象，且不继续等待、注册 GPIO 或访问传感器。
    static I2C_HandleTypeDef capacity_handle;
    for (uint8_t i = 0u; i < 6u; ++i)
    {
        IIC_Init_Config_s capacity_item = make_config(&capacity_handle, (uint8_t)(0x20u + i),
                                                       IIC_BLOCK_MODE, NULL, NULL);
        TEST_CHECK(IICRegister(&capacity_item) != NULL);
    }
    static I2C_HandleTypeDef overflow_handle;
    live_before = live_allocations;
    delay_before = dwt_delay_count;
    TEST_CHECK(TFMiniRegister(&overflow_handle) == NULL);
    TEST_CHECK(live_allocations == live_before && dwt_delay_count == delay_before);

    IST8310_Init_Config_s capacity_config;
    memset(&capacity_config, 0, sizeof(capacity_config));
    capacity_config.iic_config.handle = &overflow_handle;
    capacity_config.iic_config.dev_address = 0x30u;
    capacity_config.iic_config.work_mode = IIC_BLOCK_MODE;
    capacity_config.iic_config.id = &capacity_config;
    capacity_config.iic_config.callback = count_receive;
    IST8310_Init_Config_s capacity_saved = capacity_config;
    live_before = live_allocations;
    gpio_before = gpio_register_count;
    delay_before = hal_delay_count;
    reads_before = mem_read_count;
    writes_before = mem_write_count;
    TEST_CHECK(IST8310Init(&capacity_config) == NULL);
    TEST_CHECK(live_allocations == live_before);
    TEST_CHECK(memcmp(&capacity_config, &capacity_saved, sizeof(capacity_config)) == 0);
    TEST_CHECK(gpio_register_count == gpio_before && hal_delay_count == delay_before);
    TEST_CHECK(mem_read_count == reads_before && mem_write_count == writes_before);
}

int main(int argc, char **argv)
{
    if (argc != 2)
        return EXIT_FAILURE;

    if (strcmp(argv[1], "register") == 0)
    {
        check_invalid_and_allocation_failure();
        check_duplicate_and_callback_routing();
        return 0;
    }
    if (strcmp(argv[1], "capacity") == 0)
    {
        check_exact_capacity();
        return 0;
    }
    if (strcmp(argv[1], "callers") == 0)
    {
        iic_test_tfmini_handle = (I2C_HandleTypeDef){0};
        check_tfmini_failure_and_recovery();
        check_ist8310_failure_and_recovery();
        return 0;
    }
    if (strcmp(argv[1], "tfmini-frame") == 0)
    {
        check_tfmini_frame_bounds();
        return 0;
    }
    return EXIT_FAILURE;
}
