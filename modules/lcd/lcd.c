/**
 * @file lcd.c
 * @brief 负责 LCD 面板初始化、SPI1 发送期限、故障隔离和状态查询。
 * @note 绘图由 lcd_buf.c 和 lcd_ui.c 调用内部入口；运行时只允许单任务串行访问。
 */
#include "lcd_internal.h"

#include "platform_lcd.h" // 面板传输与引脚由板级后端提供。

#include <limits.h>
#include <string.h>

volatile uint32_t lcd_spi_errors = 0u;

#define LCD_HAL_SEND_TIMEOUT_MS 2u
#define LCD_INIT_BUDGET_MS 1000u

#define LCD_RES_Clr() PlatformLcdReset(0u) // 拉低复位。
#define LCD_RES_Set() PlatformLcdReset(1u) // 释放复位。
#define LCD_DC_Clr() PlatformLcdDataMode(0u) // 选择命令。
#define LCD_DC_Set() PlatformLcdDataMode(1u) // 选择数据。
#define LCD_CS_Clr() PlatformLcdChipSelect(0u) // 选中面板。
#define LCD_CS_Set() PlatformLcdChipSelect(1u) // 释放面板。
#define LCD_BLK_Clr(unused) do { (void)(unused); PlatformLcdBacklight(0u); } while (0) // 关闭背光。
#define LCD_BLK_Set(unused) do { (void)(unused); PlatformLcdBacklight(1u); } while (0) // 打开背光。

static LCDState_t lcd_state;
static uint8_t operation_depth;
static uint8_t operation_active;
static uint8_t recovery_operation;
static uint8_t stop_isolated;
static uint32_t operation_deadline_ms;
static LCDStatus_t operation_status;

static uint32_t saturating_add(uint32_t value, uint32_t amount)
{
    return (UINT32_MAX - value < amount) ? UINT32_MAX : value + amount;
}

static uint8_t is_io_failure(LCDStatus_t status)
{
    return (uint8_t)(status == LCD_SEND_BUSY || status == LCD_SEND_ERROR ||
                     status == LCD_SEND_TIMEOUT || status == LCD_DEADLINE_EXPIRED ||
                     status == LCD_STOP_FAILED);
}

static void lcd_latch_failure(LCDStatus_t status)
{
    if (!is_io_failure(status))
        return;

    LCD_CS_Set();
    LCD_RES_Set();
    LCD_BLK_Clr(0);
    lcd_state.backlight_on = 0u;
    lcd_state.ui_usable = 0u;
    lcd_state.failure_latched = 1u;
    if (status == LCD_STOP_FAILED)
    {
        stop_isolated = 1u;
        lcd_state.stop_isolated = 1u;
    }
    lcd_state.last_result = status;
    lcd_state.last_failure_ms = PlatformLcdNowMs();
    if (lcd_state.failure_count != UINT32_MAX)
        lcd_state.failure_count++;
    if (lcd_spi_errors != UINT32_MAX)
        lcd_spi_errors++;
}

static LCDStatus_t lcd_set_operation_error(LCDStatus_t status)
{
    if (operation_status == LCD_OK && status != LCD_OK)
    {
        operation_status = status;
        lcd_state.last_result = status;
        lcd_latch_failure(status);
    }
    return operation_status;
}

static LCDStatus_t lcd_check_context(void)
{
    if (PlatformLcdInHandler())
        return LCD_BAD_CONTEXT;
    if (!operation_active)
        return LCD_BAD_CONTEXT;
    return operation_status;
}

LCDStatus_t LCD_InternalBegin(uint32_t budget_ms, uint8_t recovery)
{
    if (budget_ms == 0u || budget_ms > INT32_MAX)
        return LCD_OUT_OF_RANGE;
    if (PlatformLcdInHandler())
        return LCD_BAD_CONTEXT;

    if (operation_active)
    {
        if (operation_status != LCD_OK)
            return operation_status;
        if (operation_depth == UINT8_MAX)
            return LCD_BUSY;
        operation_depth++;
        return LCD_OK;
    }

    if (stop_isolated != 0u)
        return LCD_STOP_FAILED;
    if (lcd_state.failure_latched && recovery == 0u)
        return LCD_UNAVAILABLE;
    if (!PlatformLcdBusReady())
        return LCD_NOT_INITIALIZED;
    if (!PlatformLcdAcquireTimebase())
        return LCD_TIMEBASE_UNAVAILABLE;

    operation_active = 1u;
    operation_depth = 1u;
    recovery_operation = recovery ? 1u : 0u;
    operation_status = LCD_OK;
    operation_deadline_ms = PlatformLcdNowMs() + budget_ms;
    return LCD_OK;
}

LCDStatus_t LCD_InternalEnd(LCDStatus_t result)
{
    if (!operation_active || operation_depth == 0u)
        return LCD_BAD_CONTEXT;
    if (result != LCD_OK && operation_status == LCD_OK)
    {
        operation_status = result;
        lcd_state.last_result = result;
        lcd_latch_failure(result);
    }

    operation_depth--;
    if (operation_depth != 0u)
        return operation_status;

    LCDStatus_t final_status = operation_status;
    operation_active = 0u;
    recovery_operation = 0u;
    operation_status = LCD_OK;
    lcd_state.last_result = final_status;
    PlatformLcdReleaseTimebase();
    return final_status;
}

LCDStatus_t LCD_InternalCurrentStatus(void)
{
    if (!operation_active)
        return LCD_BAD_CONTEXT;
    return operation_status;
}

LCDStatus_t LCD_InternalCheckDeadline(void)
{
    LCDStatus_t status = lcd_check_context();
    if (status != LCD_OK)
        return status;

    if ((int32_t)(PlatformLcdNowMs() - operation_deadline_ms) >= 0)
        return lcd_set_operation_error(LCD_DEADLINE_EXPIRED);
    return LCD_OK;
}

static LCDStatus_t lcd_transfer(uint8_t *data, size_t length)
{
    LCDStatus_t status = LCD_InternalCheckDeadline();
    if (status != LCD_OK)
        return status;
    if (data == NULL || length == 0u || length > UINT16_MAX)
        return lcd_set_operation_error(LCD_BAD_ARGUMENT);

    uint32_t remaining_ms = operation_deadline_ms - PlatformLcdNowMs();
    uint32_t timeout_ms = (remaining_ms < LCD_HAL_SEND_TIMEOUT_MS)
                              ? remaining_ms
                              : LCD_HAL_SEND_TIMEOUT_MS;
    if (timeout_ms == 0u)
        return lcd_set_operation_error(LCD_DEADLINE_EXPIRED);

    PlatformLcdTransferStatus_t transfer_status = PlatformLcdTransmit(data, (uint16_t)length, timeout_ms);
    if (transfer_status == PLATFORM_LCD_TRANSFER_BUSY)
        status = LCD_SEND_BUSY;
    else if (transfer_status == PLATFORM_LCD_TRANSFER_TIMEOUT)
        status = LCD_SEND_TIMEOUT;
    else if (transfer_status != PLATFORM_LCD_TRANSFER_OK)
        status = LCD_SEND_ERROR;
    else
    {
        if (lcd_state.send_count != UINT32_MAX)
            lcd_state.send_count++;
        lcd_state.sent_bytes = saturating_add(lcd_state.sent_bytes, (uint32_t)length);
        status = LCD_InternalCheckDeadline();
    }

    if (status != LCD_OK)
        return lcd_set_operation_error(status);
    return LCD_OK;
}

static LCDStatus_t lcd_write_byte(uint8_t value, uint8_t command)
{
    LCDStatus_t status = LCD_InternalCheckDeadline();
    if (status != LCD_OK)
        return status;

    if (command)
        LCD_DC_Clr();
    else
        LCD_DC_Set();
    LCD_CS_Clr();
    status = lcd_transfer(&value, sizeof(value));
    LCD_CS_Set();
    LCD_DC_Set();
    return status;
}

LCDStatus_t LCD_InternalWriteCommand(uint8_t command)
{
    return lcd_write_byte(command, 1u);
}

LCDStatus_t LCD_InternalWriteData8(uint8_t value)
{
    return lcd_write_byte(value, 0u);
}

static LCDStatus_t lcd_write_data16(uint16_t value)
{
    LCDStatus_t status = LCD_InternalWriteData8((uint8_t)(value >> 8));
    if (status != LCD_OK)
        return status;
    return LCD_InternalWriteData8((uint8_t)value);
}

LCDStatus_t LCD_InternalSetAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    if (!lcd_state.panel_initialized)
        return lcd_set_operation_error(LCD_NOT_INITIALIZED);
    if (x1 >= LCD_W || x2 >= LCD_W || y1 >= LCD_H || y2 >= LCD_H || x1 > x2 || y1 > y2)
        return lcd_set_operation_error(LCD_OUT_OF_RANGE);

    uint16_t x_offset = (uint16_t)(LCD_MARGIN_X + ((USE_HORIZONTAL >= 2) ? 20u : 0u));
    uint16_t y_offset = (uint16_t)(LCD_MARGIN_Y + ((USE_HORIZONTAL < 2) ? 20u : 0u));
    LCDStatus_t status = LCD_InternalWriteCommand(0x2Au);
    if (status == LCD_OK) status = lcd_write_data16((uint16_t)(x1 + x_offset));
    if (status == LCD_OK) status = lcd_write_data16((uint16_t)(x2 + x_offset));
    if (status == LCD_OK) status = LCD_InternalWriteCommand(0x2Bu);
    if (status == LCD_OK) status = lcd_write_data16((uint16_t)(y1 + y_offset));
    if (status == LCD_OK) status = lcd_write_data16((uint16_t)(y2 + y_offset));
    if (status == LCD_OK) status = LCD_InternalWriteCommand(0x2Cu);
    return status;
}

LCDStatus_t LCD_InternalBeginPixelData(void)
{
    LCDStatus_t status = LCD_InternalCheckDeadline();
    if (status != LCD_OK)
        return status;
    LCD_DC_Set();
    LCD_CS_Clr();
    return LCD_OK;
}

LCDStatus_t LCD_InternalWritePixelData(const uint8_t *data, size_t length)
{
    if (data == NULL || length == 0u)
        return lcd_set_operation_error(LCD_BAD_ARGUMENT);
    return lcd_transfer((uint8_t *)data, length);
}

LCDStatus_t LCD_InternalEndPixelData(void)
{
    LCD_CS_Set();
    return LCD_InternalCurrentStatus();
}

LCDStatus_t LCD_InternalBeginPanelInit(void)
{
    LCDStatus_t status = lcd_check_context();
    if (status != LCD_OK)
        return status;
    if (lcd_state.failure_latched && !recovery_operation)
        return LCD_UNAVAILABLE;

    LCD_BLK_Clr(0);
    LCD_CS_Set();
    lcd_state.backlight_on = 0u;
    lcd_state.panel_initialized = 0u;
    lcd_state.ui_usable = 0u;
    return LCD_OK;
}

LCDStatus_t LCD_InternalMarkPanelInitialized(void)
{
    LCDStatus_t status = LCD_InternalCheckDeadline();
    if (status != LCD_OK)
        return status;
    lcd_state.panel_initialized = 1u;
    lcd_state.failure_latched = 0u;
    return LCD_OK;
}

LCDStatus_t LCD_InternalCommitUI(void)
{
    LCDStatus_t status = LCD_InternalCheckDeadline();
    if (status != LCD_OK)
        return status;
    if (!lcd_state.panel_initialized)
        return lcd_set_operation_error(LCD_NOT_INITIALIZED);

    LCD_BLK_Set(0);
    lcd_state.backlight_on = 1u;
    lcd_state.ui_usable = 1u;
    lcd_state.failure_latched = 0u;
    lcd_state.last_result = LCD_OK;
    return LCD_OK;
}

LCDStatus_t LCD_InternalStopForRecovery(void)
{
    LCDStatus_t status = lcd_check_context();
    if (status != LCD_OK)
        return status;
    if (!recovery_operation)
        return lcd_set_operation_error(LCD_BAD_CONTEXT);

    if (!PlatformLcdStop()) // 未确认空闲时保持隔离，不复用面板总线。
        return lcd_set_operation_error(LCD_STOP_FAILED);

    LCD_CS_Set();
    LCD_BLK_Clr(0);
    lcd_state.backlight_on = 0u;
    lcd_state.panel_initialized = 0u;
    lcd_state.ui_usable = 0u;
    lcd_state.failure_latched = 0u;
    return LCD_InternalCheckDeadline();
}

LCDStatus_t LCDGetState(LCDState_t *output)
{
    if (output == NULL)
        return LCD_BAD_ARGUMENT;
    if (PlatformLcdInHandler())
        return LCD_BAD_CONTEXT;

    LCDState_t candidate = lcd_state;
    *output = candidate;
    return LCD_OK;
}

static LCDStatus_t lcd_send_register(uint8_t command, const uint8_t *values, size_t count)
{
    LCDStatus_t status = LCD_InternalWriteCommand(command);
    // 前一个命令或数据发送失败后停止，避免继续写入错位的寄存器值。
    for (size_t i = 0u; status == LCD_OK && i < count; ++i)
        status = LCD_InternalWriteData8(values[i]);
    return status;
}

LCDStatus_t LCD_Init(void)
{
    LCDStatus_t status = LCD_InternalBegin(LCD_INIT_BUDGET_MS, 0u);
    if (status != LCD_OK)
        return status;

    status = LCD_InternalBeginPanelInit();
    if (status != LCD_OK)
        goto done;

    LCD_RES_Clr();
    PlatformLcdDelayMs(100u);
    status = LCD_InternalCheckDeadline();
    if (status != LCD_OK) goto done;
    LCD_RES_Set();
    PlatformLcdDelayMs(100u);
    status = LCD_InternalCheckDeadline();
    if (status != LCD_OK) goto done;
    PlatformLcdDelayMs(100u);
    status = LCD_InternalCheckDeadline();
    if (status != LCD_OK) goto done;

    status = LCD_InternalWriteCommand(0x11u);
    if (status != LCD_OK) goto done;
    PlatformLcdDelayMs(120u);
    status = LCD_InternalCheckDeadline();
    if (status != LCD_OK) goto done;

    uint8_t orientation = (USE_HORIZONTAL == 0) ? 0x00u :
                          (USE_HORIZONTAL == 1) ? 0xC0u :
                          (USE_HORIZONTAL == 2) ? 0x70u : 0xA0u;
    status = LCD_InternalWriteCommand(0x36u);
    if (status == LCD_OK) status = LCD_InternalWriteData8(orientation);
    if (status == LCD_OK) status = LCD_InternalWriteCommand(0x3Au);
    if (status == LCD_OK) status = LCD_InternalWriteData8(0x05u);

    static const uint8_t reg_b2[] = {0x0Cu, 0x0Cu, 0x00u, 0x33u, 0x33u};
    static const uint8_t reg_b7[] = {0x35u};
    static const uint8_t reg_bb[] = {0x32u};
    static const uint8_t reg_c2[] = {0x01u};
    static const uint8_t reg_c3[] = {0x15u};
    static const uint8_t reg_c4[] = {0x20u};
    static const uint8_t reg_c6[] = {0x0Fu};
    static const uint8_t reg_d0[] = {0xA4u, 0xA1u};
    static const uint8_t reg_e0[] = {0xD0u, 0x08u, 0x0Eu, 0x09u, 0x09u, 0x05u, 0x31u,
                                     0x33u, 0x48u, 0x17u, 0x14u, 0x15u, 0x31u, 0x34u};
    static const uint8_t reg_e1[] = {0xD0u, 0x08u, 0x0Eu, 0x09u, 0x09u, 0x15u, 0x31u,
                                     0x33u, 0x48u, 0x17u, 0x14u, 0x15u, 0x31u, 0x34u};
    if (status == LCD_OK) status = lcd_send_register(0xB2u, reg_b2, sizeof(reg_b2));
    if (status == LCD_OK) status = lcd_send_register(0xB7u, reg_b7, sizeof(reg_b7));
    if (status == LCD_OK) status = lcd_send_register(0xBBu, reg_bb, sizeof(reg_bb));
    if (status == LCD_OK) status = lcd_send_register(0xC2u, reg_c2, sizeof(reg_c2));
    if (status == LCD_OK) status = lcd_send_register(0xC3u, reg_c3, sizeof(reg_c3));
    if (status == LCD_OK) status = lcd_send_register(0xC4u, reg_c4, sizeof(reg_c4));
    if (status == LCD_OK) status = lcd_send_register(0xC6u, reg_c6, sizeof(reg_c6));
    if (status == LCD_OK) status = lcd_send_register(0xD0u, reg_d0, sizeof(reg_d0));
    if (status == LCD_OK) status = lcd_send_register(0xE0u, reg_e0, sizeof(reg_e0));
    if (status == LCD_OK) status = lcd_send_register(0xE1u, reg_e1, sizeof(reg_e1));
    if (status == LCD_OK) status = LCD_InternalWriteCommand(0x21u);
    if (status == LCD_OK) status = LCD_InternalWriteCommand(0x29u);
    if (status == LCD_OK) status = LCD_InternalMarkPanelInitialized();

done:
    return LCD_InternalEnd(status);
}
