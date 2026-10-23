/**
 * @file lcd_bounds_test.c
 * @brief 通过生产 LCD 绘图入口检查边界、字模、数值转换、发送失败和 SPI 隔离。
 * @note HAL、毫秒时钟和 DWT 由本测试替代；不证明实际屏幕显示或 SPI 电气波形。
 */
#include "lcd_buf.h"
#include "lcd_internal.h"
#include "lcd_ui.h"
#include "lcd_ui_test_types.h"
#include "rm_referee.h"

#include <math.h>
#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

GPIO_TypeDef fake_gpiob, fake_gpioc, fake_gpiod, fake_gpioe;
SPI_HandleTypeDef hspi1, hspi2;
FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
BMI088_CalibSource_e bmi088_calib_source = BMI088_CALIB_SRC_DEFAULT;
float bmi088_calib_temp = 25.0f;
volatile uint16_t lcd_key_raw_min = 0u;
extern volatile uint32_t lcd_rows_drawn;
extern volatile uint32_t lcd_rows_skipped;

static uint32_t fake_tick;
static uint32_t fake_probe_us;
static uint32_t transfers;
static uint32_t fail_on_transfer;
static uint32_t transfer_delay_ms;
static uint32_t lease_count;
static uint32_t abort_count;
static uint32_t fake_ipsr;
static uint8_t lease_available = 1u;
static HAL_StatusTypeDef transfer_result = HAL_ERROR;
static HAL_StatusTypeDef abort_result = HAL_OK;
static HAL_SPI_StateTypeDef spi_state = HAL_SPI_STATE_READY;
static uint8_t first_pixel[2];
static uint8_t last_pixel[2];
static uint8_t pixel_seen;
static uint8_t current_command;
static uint8_t address_bytes[4];
static uint8_t address_count;
static uint16_t window_x1, window_x2, window_y1, window_y2;
static uint32_t pixel_bytes;
static uint8_t pixel_prefix[1024];
static size_t pixel_prefix_length;
typedef struct { uint16_t x1, x2, y1, y2; } PixelWindow_t;
static PixelWindow_t pixel_windows[256];
static size_t pixel_window_count;
static uint32_t test_faults;
static BSP_ADC_Sample_t test_adc_sample = {0u, 12.4f, 1u};
static BSP_ADC_Status_e test_adc_status = BSP_ADC_STATUS_VALID;
static IMUHeaterStatus_t test_heater_status = {25.0f, 40.0f, 0u, 0u, 0u, 0u, 1u, 0u};
static attitude_t test_attitude = {1.0f, 2.0f, 3.0f};

#define TEST_CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL: %s (%s:%d)\n", #condition, __FILE__, __LINE__); \
        return 1; \
    } \
} while (0)

uint32_t HAL_GetTick(void) { return fake_tick; }
uint32_t __get_IPSR(void) { return fake_ipsr; }
uint32_t DWT_ProbeStart(void) { return fake_probe_us; }
uint32_t DWT_ProbeElapsedUs(uint32_t start) { return fake_probe_us - start; }
void HAL_Delay(uint32_t ms) { fake_tick += ms; fake_probe_us += ms * 1000u; }
void Error_Handler(void) { abort(); }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, uint32_t value)
{
    if (value == GPIO_PIN_SET) port->ODR |= pin; else port->ODR &= ~pin;
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config) { (void)port; (void)config; }
void HAL_GPIO_DeInit(GPIO_TypeDef *port, uint32_t pins) { (void)port; (void)pins; }
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config) { (void)config; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *handle) { (void)handle; return HAL_OK; }
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *handle) { (void)handle; return spi_state; }
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *handle)
{
    (void)handle;
    abort_count++;
    if (abort_result == HAL_OK) spi_state = HAL_SPI_STATE_READY;
    return abort_result;
}
void MX_SPI1_Init(void)
{
    memset(&hspi1, 0, sizeof(hspi1));
    hspi1.Instance = SPI1;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
}
void MX_SPI2_Init(void) {}
uint8_t BSP_HALTickTryAcquireBlockingWait(void)
{
    if (!lease_available)
        return 0u;
    lease_count++;
    return 1u;
}
void BSP_HALTickReleaseBlockingWait(void) { if (lease_count > 0u) lease_count--; }

Robot_Status_e RobotSafetyGetState(void) { return ROBOT_SAFE; }
uint32_t RobotSafetyGetFaults(void) { return test_faults; }
uint8_t EstopIsLatched(void) { return 0u; }
EstopReason_e EstopGetReason(void) { return ESTOP_REASON_MANUAL; }
BSP_ADC_Status_e BSP_ADCGetSample(BSP_ADC_Sample_t *output)
{
    if (output != NULL)
        *output = test_adc_sample;
    return test_adc_status;
}
uint8_t BSP_WatchdogIsRunning(void) { return 0u; }
uint8_t BSP_WatchdogWasIwdgReset(void) { return 0u; }
void TaskMonitorGetStatus(TaskMonitor_Id_e id, TaskMonitorStatus_t *output)
{
    (void)id;
    if (output != NULL)
        output->alive = 1u;
}
uint8_t RemoteControlIsOnline(void) { return 1u; }
void VisionGetStatus(Vision_Status_t *output)
{
    if (output != NULL)
        memset(output, 0, sizeof(*output));
}
void IMUHeaterGetStatus(IMUHeaterStatus_t *output)
{
    if (output != NULL)
        *output = test_heater_status;
}
attitude_t *INS_GetAttitude(void) { return &test_attitude; }
void CANGetStatus(FDCAN_HandleTypeDef *handle, CAN_Status_t *output)
{
    (void)handle;
    if (output != NULL)
        output->status = CAN_STATUS_OK;
}
RefereeStatus_t RefereeGetSnapshot(RefereeSnapshot_t *output)
{
    if (output == NULL)
        return REFEREE_ERROR_ARGUMENT;
    memset(output, 0, sizeof(*output));
    return REFEREE_OK;
}
uint8_t CrashLogGetLast(CrashLog_t *output) { (void)output; return 0u; }
const char *CrashLogTypeShort(CrashType_e type) { (void)type; return "--"; }

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *handle, uint8_t *data,
                                  uint16_t length, uint32_t timeout_ms)
{
    (void)timeout_ms;
    transfers++;
    if (transfer_delay_ms != 0u)
        fake_tick += transfer_delay_ms;
    if (handle != &hspi1 || (fake_gpioe.ODR & LCD_CS_Pin) != 0u)
        return HAL_ERROR;
    if (fail_on_transfer != 0u && transfers == fail_on_transfer)
        return transfer_result;
    if ((fake_gpiod.ODR & LCD_DC_Pin) == 0u)
    {
        current_command = data[0];
        if (current_command == 0x2Au || current_command == 0x2Bu)
            address_count = 0u;
        if (current_command == 0x2Cu)
        {
            pixel_bytes = 0u;
            pixel_seen = 0u;
            pixel_prefix_length = 0u;
            if (pixel_window_count < sizeof(pixel_windows) / sizeof(pixel_windows[0]))
            {
                pixel_windows[pixel_window_count++] =
                    (PixelWindow_t){window_x1, window_x2, window_y1, window_y2};
            }
        }
    }
    else if (current_command == 0x2Au || current_command == 0x2Bu)
    {
        for (uint16_t i = 0u; i < length && address_count < sizeof(address_bytes); ++i)
            address_bytes[address_count++] = data[i];
        if (address_count == sizeof(address_bytes))
        {
            uint16_t start = (uint16_t)(((uint16_t)address_bytes[0] << 8) | address_bytes[1]);
            uint16_t end = (uint16_t)(((uint16_t)address_bytes[2] << 8) | address_bytes[3]);
            if (current_command == 0x2Au)
            {
                window_x1 = start;
                window_x2 = end;
            }
            else
            {
                window_y1 = start;
                window_y2 = end;
            }
        }
    }
    else if (current_command == 0x2Cu)
    {
        if (length >= 2u && !pixel_seen)
        {
            first_pixel[0] = data[0];
            first_pixel[1] = data[1];
            pixel_seen = 1u;
        }
        if (length >= 2u)
        {
            last_pixel[0] = data[length - 2u];
            last_pixel[1] = data[length - 1u];
        }
        size_t available = sizeof(pixel_prefix) - pixel_prefix_length;
        size_t copy_length = length < available ? length : available;
        memcpy(pixel_prefix + pixel_prefix_length, data, copy_length);
        pixel_prefix_length += copy_length;
        pixel_bytes += length;
    }
    return HAL_OK;
}

static void reset_capture(void)
{
    transfers = 0u;
    fail_on_transfer = 0u;
    transfer_delay_ms = 0u;
    pixel_seen = 0u;
    first_pixel[0] = 0u;
    first_pixel[1] = 0u;
    last_pixel[0] = 0u;
    last_pixel[1] = 0u;
    current_command = 0u;
    address_count = 0u;
    window_x1 = window_x2 = window_y1 = window_y2 = 0u;
    pixel_bytes = 0u;
    pixel_prefix_length = 0u;
    pixel_window_count = 0u;
    transfer_result = HAL_ERROR;
}

static int initialize_panel(void)
{
    MX_SPI1_Init();
    TEST_CHECK(LCD_Init() == LCD_OK);
    TEST_CHECK(lease_count == 0u);
    return 0;
}

static int mark_test_ui_ready(void)
{
    TEST_CHECK(LCD_InternalBegin(20u, 0u) == LCD_OK);
    TEST_CHECK(LCD_InternalCommitUI() == LCD_OK);
    TEST_CHECK(LCD_InternalEnd(LCD_OK) == LCD_OK);
    return 0;
}

static int test_boundaries(void)
{
    TEST_CHECK(initialize_panel() == 0);
    reset_capture();
    TEST_CHECK(LCD_BufFill(5u, 6u, 7u, 8u, 0xF81Fu) == LCD_OK);
    TEST_CHECK(pixel_seen != 0u);
    TEST_CHECK(window_x1 == 5u + LCD_MARGIN_X + 20u && window_x2 == 7u + LCD_MARGIN_X + 20u &&
               window_y1 == 6u + LCD_MARGIN_Y && window_y2 == 8u + LCD_MARGIN_Y);
    TEST_CHECK(pixel_bytes == 3u * 3u * 2u);
    /* 行缓冲沿用 STM32 小端内存中的 RGB565 字节次序。 */
    TEST_CHECK(first_pixel[0] == 0x1Fu && first_pixel[1] == 0xF8u);
    TEST_CHECK(last_pixel[0] == 0x1Fu && last_pixel[1] == 0xF8u);
    uint32_t before = transfers;
    TEST_CHECK(LCD_BufFill(0u, 0u, LCD_W, 0u, RED) == LCD_OUT_OF_RANGE);
    TEST_CHECK(LCD_BufFill(UINT16_MAX, 0u, UINT16_MAX, 0u, RED) == LCD_OUT_OF_RANGE);
    TEST_CHECK(LCD_BufFill(3u, 2u, 2u, 2u, RED) == LCD_OUT_OF_RANGE);
    TEST_CHECK(transfers == before);

    static const char printable[] = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";
    TEST_CHECK(sizeof(printable) - 1u == 95u);
    for (size_t i = 0u; i < sizeof(printable) - 1u; ++i)
    {
        reset_capture();
        TEST_CHECK(LCD_BufShowAscii((uint16_t)(LCD_W - 6u), (uint16_t)(LCD_H - 12u),
                                   &printable[i], 1u, WHITE, BLACK, 12u) == LCD_OK);
        TEST_CHECK(window_x1 == LCD_W - 6u + LCD_MARGIN_X + 20u &&
                   window_x2 == LCD_W - 1u + LCD_MARGIN_X + 20u);
        TEST_CHECK(window_y1 == LCD_H - 12u + LCD_MARGIN_Y && window_y2 == LCD_H - 1u + LCD_MARGIN_Y);
        TEST_CHECK(pixel_bytes == 6u * 12u * 2u);
        reset_capture();
        TEST_CHECK(LCD_BufShowAscii((uint16_t)(LCD_W - 8u), (uint16_t)(LCD_H - 16u),
                                   &printable[i], 1u, WHITE, BLACK, 16u) == LCD_OK);
        TEST_CHECK(pixel_bytes == 8u * 16u * 2u);
    }

    static const char letter_a[] = "A";
    reset_capture();
    TEST_CHECK(LCD_BufShowAscii(0u, 0u, letter_a, 1u, WHITE, BLACK, 16u) == LCD_OK);
    TEST_CHECK(pixel_prefix_length >= 16u * 8u * 2u);
    /* 字库中 A 的第4行第4列为前景点，邻近首点为背景色。 */
    TEST_CHECK(pixel_prefix[0] == 0u && pixel_prefix[1] == 0u);
    TEST_CHECK(pixel_prefix[54] == 0xFFu && pixel_prefix[55] == 0xFFu);

    for (unsigned glyph_id = 0u; glyph_id < (unsigned)LCD_CN_COUNT; ++glyph_id)
    {
        Lcd_CnChar_e glyph = (Lcd_CnChar_e)glyph_id;
        reset_capture();
        TEST_CHECK(LCD_BufShowCnString((uint16_t)(LCD_W - 16u), (uint16_t)(LCD_H - 16u),
                                      &glyph, 1u, WHITE, BLACK) == LCD_OK);
        TEST_CHECK(window_x1 == LCD_W - 16u + LCD_MARGIN_X + 20u &&
                   window_x2 == LCD_W - 1u + LCD_MARGIN_X + 20u);
        TEST_CHECK(window_y1 == LCD_H - 16u + LCD_MARGIN_Y && window_y2 == LCD_H - 1u + LCD_MARGIN_Y);
        TEST_CHECK(pixel_bytes == 16u * 16u * 2u);
    }
    Lcd_CnChar_e bad_glyph = LCD_CN_COUNT;
    before = transfers;
    TEST_CHECK(LCD_BufShowCnString(0u, 0u, &bad_glyph, 1u, WHITE, BLACK) == LCD_BAD_GLYPH);
    TEST_CHECK(transfers == before);
    TEST_CHECK(LCD_BufShowAscii(0u, 0u, NULL, 0u, WHITE, BLACK, 16u) == LCD_OK);
    TEST_CHECK(LCD_BufShowCnString(0u, 0u, NULL, 0u, WHITE, BLACK) == LCD_OK);
    TEST_CHECK(lease_count == 0u);
    return 0;
}

static int test_invalid_and_huge_lengths(void)
{
    TEST_CHECK(initialize_panel() == 0);
    reset_capture();
    TEST_CHECK(LCD_BufShowAscii(0u, 0u, "A\nB", 3u, WHITE, BLACK, 16u) == LCD_BAD_TEXT);
    TEST_CHECK(LCD_BufShowAscii(0u, 0u, "A", SIZE_MAX, WHITE, BLACK, 16u) == LCD_OUT_OF_RANGE);
    TEST_CHECK(LCD_BufShowAscii(0u, 0u, "A", 1u, WHITE, BLACK, 24u) == LCD_OUT_OF_RANGE);
    Lcd_CnChar_e glyph = LCD_CN_XI;
    TEST_CHECK(LCD_BufShowCnString(0u, 0u, &glyph, SIZE_MAX, WHITE, BLACK) == LCD_OUT_OF_RANGE);
    TEST_CHECK(transfers == 0u);
    TEST_CHECK(lease_count == 0u);
    return 0;
}

static int test_numbers(void)
{
    TEST_CHECK(initialize_panel() == 0);
    reset_capture();
    char text[16] = "unchanged";
    size_t length = 77u;
    TEST_CHECK(LCD_BufFormatFloat(1.25f, 1u, text, sizeof(text), &length) == LCD_OK);
    TEST_CHECK(strcmp(text, "1.3") == 0 && length == 3u);
    TEST_CHECK(LCD_BufFormatFloat(-1.25f, 1u, text, sizeof(text), &length) == LCD_OK);
    TEST_CHECK(strcmp(text, "-1.3") == 0 && length == 4u);
    TEST_CHECK(LCD_BufFormatFloat(-0.04f, 1u, text, sizeof(text), &length) == LCD_OK);
    TEST_CHECK(strcmp(text, "0.0") == 0);
    TEST_CHECK(LCD_BufFormatFloat((float)INT32_MIN, 0u, text, sizeof(text), &length) == LCD_OK);
    TEST_CHECK(strcmp(text, "-2147483648") == 0);

    memcpy(text, "keep", 5u);
    length = 91u;
    TEST_CHECK(LCD_BufFormatFloat(NAN, 1u, text, sizeof(text), &length) == LCD_NONFINITE);
    TEST_CHECK(strcmp(text, "keep") == 0 && length == 91u);
    TEST_CHECK(LCD_BufFormatFloat(INFINITY, 1u, text, sizeof(text), &length) == LCD_NONFINITE);
    TEST_CHECK(LCD_BufFormatFloat(1.0f, 4u, text, sizeof(text), &length) == LCD_OUT_OF_RANGE);
    TEST_CHECK(LCD_BufFormatFloat(214748364.8f, 1u, text, sizeof(text), &length) == LCD_OUT_OF_RANGE);
    TEST_CHECK(LCD_BufFormatFloat(1.0f, 2u, text, 3u, &length) == LCD_OUT_OF_RANGE);
    TEST_CHECK(strcmp(text, "keep") == 0 && length == 91u);
    TEST_CHECK(LCD_BufShowInt(0u, 0u, INT32_MIN, WHITE, BLACK, 12u) == LCD_OK);
    TEST_CHECK(LCD_BufShowInt(0u, 0u, INT32_MAX, WHITE, BLACK, 12u) == LCD_OK);
    TEST_CHECK(LCD_BufShowFloat(0u, 0u, NAN, 1u, WHITE, BLACK, 16u) == LCD_NONFINITE);
    TEST_CHECK(LCD_BufShowFloat(0u, 0u, FLT_MAX, 3u, WHITE, BLACK, 16u) == LCD_OUT_OF_RANGE);
    TEST_CHECK(lease_count == 0u);
    return 0;
}

static int test_send_failure(void)
{
    TEST_CHECK(initialize_panel() == 0);
    TEST_CHECK(mark_test_ui_ready() == 0);
    reset_capture();
    fail_on_transfer = 1u;
    transfer_result = HAL_ERROR;
    TEST_CHECK(LCD_BufFill(0u, 0u, 2u, 2u, RED) == LCD_SEND_ERROR);
    LCDState_t state;
    TEST_CHECK(LCDGetState(&state) == LCD_OK);
    TEST_CHECK(state.failure_latched && !state.ui_usable && !state.backlight_on);
    TEST_CHECK((fake_gpiob.ODR & LCD_BLK_Pin) == 0u);
    uint32_t after_failure = transfers;
    TEST_CHECK(LCD_BufFill(0u, 0u, 1u, 1u, BLUE) == LCD_UNAVAILABLE);
    TEST_CHECK(transfers == after_failure);
    TEST_CHECK(lease_count == 0u);
    return 0;
}

static int test_ui_result_propagation(void)
{
    TEST_CHECK(initialize_panel() == 0);
    TEST_CHECK(mark_test_ui_ready() == 0);
    reset_capture();
    TEST_CHECK(LCD_UI_DrawStatic(0u) == LCD_OK);
    TEST_CHECK(transfers > 0u && lease_count == 0u);

    reset_capture();
    fail_on_transfer = 5u;
    transfer_result = HAL_BUSY;
    TEST_CHECK(LCD_UI_DrawStatic(1u) == LCD_SEND_BUSY);
    TEST_CHECK(transfers == fail_on_transfer);
    LCDState_t state;
    TEST_CHECK(LCDGetState(&state) == LCD_OK);
    TEST_CHECK(state.failure_latched && !state.ui_usable && !state.backlight_on);
    TEST_CHECK((fake_gpiob.ODR & LCD_BLK_Pin) == 0u);
    uint32_t stopped_after = transfers;
    TEST_CHECK(LCD_UI_SetFrozen(1u) == LCD_UNAVAILABLE);
    TEST_CHECK(transfers == stopped_after && lease_count == 0u);
    return 0;
}

static int test_ui_last_transfer_failure(void)
{
    TEST_CHECK(initialize_panel() == 0);
    TEST_CHECK(mark_test_ui_ready() == 0);
    TEST_CHECK(LCD_UI_DrawStatic(0u) == LCD_OK);

    LCD_UI_InvalidateValueCache();
    reset_capture();
    TEST_CHECK(LCD_UI_UpdateValues(0u) == LCD_OK);
    uint32_t complete_transfer_count = transfers;
    TEST_CHECK(complete_transfer_count > 1u);

    LCD_UI_InvalidateValueCache();
    reset_capture();
    fail_on_transfer = complete_transfer_count;
    transfer_result = HAL_ERROR;
    TEST_CHECK(LCD_UI_UpdateValues(0u) == LCD_SEND_ERROR);
    TEST_CHECK(transfers == complete_transfer_count);

    LCDState_t state;
    TEST_CHECK(LCDGetState(&state) == LCD_OK);
    TEST_CHECK(state.failure_latched && !state.ui_usable && !state.backlight_on);
    TEST_CHECK((fake_gpiob.ODR & LCD_BLK_Pin) == 0u);
    TEST_CHECK(lease_count == 0u);
    uint32_t after_failure = transfers;
    TEST_CHECK(LCD_UI_UpdateValues(0u) == LCD_UNAVAILABLE);
    TEST_CHECK(transfers == after_failure);
    return 0;
}

static int test_ui_cache_and_recovery(void)
{
    MX_SPI1_Init();
    reset_capture();
    test_faults = 0u;
    lcd_rows_drawn = 0u;
    lcd_rows_skipped = 0u;
    TEST_CHECK(LCD_UI_Init() == LCD_OK);
    LCDState_t state;
    TEST_CHECK(LCDGetState(&state) == LCD_OK);
    TEST_CHECK(state.panel_initialized && state.ui_usable && state.backlight_on);
    uint32_t drawn_after_init = lcd_rows_drawn;
    TEST_CHECK(drawn_after_init >= 9u);

    reset_capture();
    uint32_t skipped_before = lcd_rows_skipped;
    TEST_CHECK(LCD_UI_UpdateValues(0u) == LCD_OK);
    TEST_CHECK(transfers == 0u && lcd_rows_skipped == skipped_before + 9u);

    test_faults = ROBOT_FAULT_RC_OFFLINE;
    reset_capture();
    fail_on_transfer = 14u;
    transfer_result = HAL_ERROR;
    uint32_t drawn_before_failure = lcd_rows_drawn;
    TEST_CHECK(LCD_UI_UpdateValues(0u) == LCD_SEND_ERROR);
    TEST_CHECK(transfers == fail_on_transfer && lcd_rows_drawn == drawn_before_failure);
    TEST_CHECK(LCDGetState(&state) == LCD_OK);
    TEST_CHECK(state.failure_latched && !state.ui_usable && !state.backlight_on);
    TEST_CHECK((fake_gpiob.ODR & LCD_BLK_Pin) == 0u);
    uint32_t failed_transfer_count = transfers;
    TEST_CHECK(LCD_UI_UpdateValues(0u) == LCD_UNAVAILABLE);
    TEST_CHECK(transfers == failed_transfer_count);

    spi_state = HAL_SPI_STATE_BUSY_TX;
    abort_result = HAL_OK;
    reset_capture();
    TEST_CHECK(LCD_UI_Recover() == LCD_OK);
    TEST_CHECK(abort_count == 1u && lease_count == 0u);
    TEST_CHECK(LCDGetState(&state) == LCD_OK);
    TEST_CHECK(!state.failure_latched && state.ui_usable && state.backlight_on);
    TEST_CHECK(lcd_rows_drawn >= drawn_after_init + 9u);

    test_heater_status.temperature = NAN;
    test_heater_status.target_temp = INFINITY;
    test_attitude.Yaw = NAN;
    TEST_CHECK(LCD_UI_DrawStatic(1u) == LCD_OK);
    reset_capture();
    TEST_CHECK(LCD_UI_UpdateValues(1u) == LCD_OK);
    uint16_t x_offset = (uint16_t)(LCD_MARGIN_X + LCD_PANEL_X0);
    uint16_t y_offset = (uint16_t)(LCD_MARGIN_Y + LCD_PANEL_Y0);
    uint8_t row0_clear_seen = 0u, row0_text_seen = 0u, row0_unit_seen = 0u;
    for (size_t i = 0u; i < pixel_window_count; ++i)
    {
        PixelWindow_t window = pixel_windows[i];
        if (window.y1 != 28u + y_offset || window.y2 != 43u + y_offset)
            continue;
        if (window.x1 == 80u + x_offset && window.x2 == LCD_W - 1u + x_offset)
            row0_clear_seen = 1u;
        if (window.x1 == 80u + x_offset && window.x2 == 95u + x_offset)
            row0_text_seen = 1u;
        if (window.x1 == 144u + x_offset && window.x2 == 151u + x_offset)
            row0_unit_seen = 1u;
    }
    TEST_CHECK(row0_clear_seen && row0_text_seen && row0_unit_seen);
    return 0;
}

static int test_deadline_and_stop_isolation(void)
{
    TEST_CHECK(initialize_panel() == 0);
    TEST_CHECK(mark_test_ui_ready() == 0);
    reset_capture();
    transfer_delay_ms = 2u;
    const char text[] = "deadline";
    TEST_CHECK(LCD_BufShowAscii(0u, 0u, text, sizeof(text) - 1u, WHITE, BLACK, 16u) == LCD_DEADLINE_EXPIRED);
    LCDState_t state;
    TEST_CHECK(LCDGetState(&state) == LCD_OK && state.failure_latched && !state.backlight_on);
    TEST_CHECK(lease_count == 0u);

    spi_state = HAL_SPI_STATE_BUSY_TX;
    abort_result = HAL_ERROR;
    uint32_t previous_abort_count = abort_count;
    TEST_CHECK(LCD_InternalBegin(1000u, 1u) == LCD_OK);
    TEST_CHECK(LCD_InternalStopForRecovery() == LCD_STOP_FAILED);
    TEST_CHECK(LCD_InternalEnd(LCD_STOP_FAILED) == LCD_STOP_FAILED);
    TEST_CHECK(abort_count == previous_abort_count + 1u);
    TEST_CHECK(LCD_InternalBegin(1000u, 1u) == LCD_STOP_FAILED);
    TEST_CHECK(lease_count == 0u);
    return 0;
}

static int test_hal_timeout_and_context(void)
{
    TEST_CHECK(initialize_panel() == 0);
    LCDState_t state;
    memset(&state, 0x5Au, sizeof(state));
    LCDState_t previous = state;
    TEST_CHECK(LCDGetState(NULL) == LCD_BAD_ARGUMENT);
    fake_ipsr = 1u;
    TEST_CHECK(LCDGetState(&state) == LCD_BAD_CONTEXT);
    TEST_CHECK(memcmp(&state, &previous, sizeof(state)) == 0);
    TEST_CHECK(LCD_BufFill(0u, 0u, 0u, 0u, BLACK) == LCD_BAD_CONTEXT);
    fake_ipsr = 0u;

    lease_available = 0u;
    uint32_t sends_before = transfers;
    TEST_CHECK(LCD_BufFill(0u, 0u, 0u, 0u, BLACK) == LCD_TIMEBASE_UNAVAILABLE);
    TEST_CHECK(transfers == sends_before && lease_count == 0u);
    lease_available = 1u;

    reset_capture();
    fail_on_transfer = 1u;
    transfer_result = HAL_TIMEOUT;
    TEST_CHECK(LCD_BufFill(0u, 0u, 1u, 1u, RED) == LCD_SEND_TIMEOUT);
    TEST_CHECK(lease_count == 0u);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 2)
        return 2;
    fake_tick = 0u;
    fake_probe_us = 0u;
    lease_count = 0u;
    abort_count = 0u;
    fake_ipsr = 0u;
    lease_available = 1u;
    abort_result = HAL_OK;
    spi_state = HAL_SPI_STATE_READY;

    if (strcmp(argv[1], "boundaries") == 0) return test_boundaries();
    if (strcmp(argv[1], "invalid") == 0) return test_invalid_and_huge_lengths();
    if (strcmp(argv[1], "numbers") == 0) return test_numbers();
    if (strcmp(argv[1], "send-failure") == 0) return test_send_failure();
    if (strcmp(argv[1], "deadline-stop") == 0) return test_deadline_and_stop_isolation();
    if (strcmp(argv[1], "timeout-context") == 0) return test_hal_timeout_and_context();
    if (strcmp(argv[1], "ui-status") == 0) return test_ui_result_propagation();
    if (strcmp(argv[1], "ui-last-send-failure") == 0) return test_ui_last_transfer_failure();
    if (strcmp(argv[1], "ui-cache-recovery") == 0) return test_ui_cache_and_recovery();
    fprintf(stderr, "unknown scenario: %s\n", argv[1]);
    return 2;
}
