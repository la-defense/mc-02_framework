/** @file platform_spi_registration_test.c
 * @brief 验证生产逻辑 SPI 注册适配器使用正确板级资源和失败结果。
 */
#include "test_check.h"
#include "bsp_spi.h"
#include "../../bsp/spi/platform_spi_blocking.h"
#include <stddef.h>

SPI_HandleTypeDef hspi1, hspi2;
GPIO_TypeDef test_gyro_port, test_accel_port;
static SPIInstance instance;
static unsigned calls;
static SPI_Init_Config_s saved;
static uint8_t fail_registration;

SPIInstance *SPIRegister(const SPI_Init_Config_s *config)
{
    ++calls; // 记录是否实际到达设备注册层。
    saved = *config; // 保存实际板级配置以检查资源映射。
    return fail_registration ? NULL : &instance;
}

uint8_t SPIUnregister(SPIInstance *device) {return (uint8_t)(device == &instance);}

int main(void)
{
    int owner = 1; // 模拟长期有效的设备上下文。
    PlatformSpiEndpoint_t endpoint = {PLATFORM_SPI_BUS_2, PLATFORM_GPIO_IMU_ACCEL_CS};
    TEST_CHECK(PlatformSpiRegisterBlocking(NULL, &owner) == NULL && calls == 0u);
    TEST_CHECK(PlatformSpiRegisterBlocking(&endpoint, &owner) == &instance);
    TEST_CHECK(saved.spi_handle == &hspi2 && saved.GPIOx == CS2_ACCEL_GPIO_Port);
    TEST_CHECK(saved.cs_pin == CS2_ACCEL_Pin && saved.spi_work_mode == SPI_BLOCK_MODE && saved.id == &owner);
    endpoint.chip_select = PLATFORM_GPIO_IMU_GYRO_CS; // 另一设备共享总线但使用独立片选。
    TEST_CHECK(PlatformSpiRegisterBlocking(&endpoint, NULL) == &instance);
    TEST_CHECK(saved.GPIOx == CS2_GYRO_GPIO_Port && saved.cs_pin == CS2_GYRO_Pin && saved.id == NULL);
    endpoint.bus = PLATFORM_SPI_INVALID;
    TEST_CHECK(PlatformSpiRegisterBlocking(&endpoint, &owner) == NULL && calls == 2u);
    endpoint.bus = PLATFORM_SPI_BUS_2;
    endpoint.chip_select = PLATFORM_GPIO_INVALID;
    TEST_CHECK(PlatformSpiRegisterBlocking(&endpoint, &owner) == NULL && calls == 2u);
    endpoint.chip_select = PLATFORM_GPIO_IMU_ACCEL_CS;
    fail_registration = 1u;
    TEST_CHECK(PlatformSpiRegisterBlocking(&endpoint, &owner) == NULL && calls == 3u);
    TEST_CHECK(PlatformSpiUnregister(&instance) == 1u && PlatformSpiUnregister(NULL) == 0u);
    return 0;
}
