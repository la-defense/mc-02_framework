/**
 * @file lcd.h
 * @brief LCD 面板初始化、状态查询和绘图结果类型。
 *
 * 先配置 GPIO、SPI1 和 HAL 毫秒时基，再由单个普通任务调用 LCD 接口。
 * 坐标使用绘图区像素，原点在左上角；颜色使用 RGB565。发送成功表示
 * SPI1 阻塞传输完成，不表示面板已确认或实际显示。
 */
#ifndef MC02_LCD_H
#define MC02_LCD_H

#include <stdint.h>
#include "main.h"
#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USE_ANALOG_SPI 0
#define USE_HORIZONTAL 2

#define LCD_MARGIN_X 14
#define LCD_MARGIN_Y 14

#if USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1
#define LCD_W (240 - 2 * LCD_MARGIN_X)
#define LCD_H (280 - 2 * LCD_MARGIN_Y)
#define LCD_PANEL_X0 0
#define LCD_PANEL_Y0 20
#define LCD_PANEL_W 240
#define LCD_PANEL_H 280
#else
#define LCD_W (280 - 2 * LCD_MARGIN_X)
#define LCD_H (240 - 2 * LCD_MARGIN_Y)
#define LCD_PANEL_X0 20
#define LCD_PANEL_Y0 0
#define LCD_PANEL_W 280
#define LCD_PANEL_H 240
#endif

/** LCD 调用结果；LCD_OK 只代表本机 SPI 操作成功。 */
typedef enum
{
    LCD_OK = 0,
    LCD_BAD_ARGUMENT,
    LCD_OUT_OF_RANGE,
    LCD_BAD_TEXT,
    LCD_BAD_GLYPH,
    LCD_NONFINITE,
    LCD_NOT_INITIALIZED,
    LCD_BAD_CONTEXT,
    LCD_TIMEBASE_UNAVAILABLE,
    LCD_BUSY,
    LCD_SEND_BUSY,
    LCD_SEND_ERROR,
    LCD_SEND_TIMEOUT,
    LCD_DEADLINE_EXPIRED,
    LCD_STOP_FAILED,
    LCD_UNAVAILABLE
} LCDStatus_t;

/** 通过 LCDGetState() 复制得到的只读运行状态。 */
typedef struct
{
    uint8_t panel_initialized;
    uint8_t ui_usable;
    uint8_t backlight_on;
    uint8_t failure_latched;
    uint8_t stop_isolated;
    LCDStatus_t last_result;
    uint32_t last_failure_ms;
    uint32_t failure_count;
    uint32_t send_count;
    uint32_t sent_bytes;
} LCDState_t;

/**
 * @brief 初始化面板控制器并设置方向和 RGB565 格式。
 * @return LCD_OK 或具体失败状态；失败后面板保持关闭，需经 UI 恢复流程重试。
 * @note 外设与 HAL 毫秒时基须已配置；阻塞等待计入 1000ms 顶层期限。
 *       单独调用不会打开背光，完整界面绘制成功后才会点亮。
 */
LCDStatus_t LCD_Init(void);

/**
 * @brief 复制 LCD 当前状态，不发送 SPI、不执行恢复。
 * @param[out] output 非空状态副本；输出地址须可写。
 * @return LCD_OK；空指针返回 LCD_BAD_ARGUMENT，其他情况下不修改输出。
 */
LCDStatus_t LCDGetState(LCDState_t *output);

#ifdef __cplusplus
}
#endif

/* LCD 引脚电平宏供本模块和板级初始化使用。 */
#define LCD_RES_Clr() HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET)
#define LCD_RES_Set() HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET)
#define LCD_DC_Clr() HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET)
#define LCD_DC_Set() HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET)
#define LCD_CS_Clr() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define LCD_CS_Set() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)
#define LCD_BLK_Clr(unused) do { (void)(unused); HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_RESET); } while (0)
#define LCD_BLK_Set(unused) do { (void)(unused); HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_SET); } while (0)

#define WHITE     0xFFFFu
#define BLACK     0x0000u
#define BLUE      0x001Fu
#define BRED      0xF81Fu
#define GRED      0xFFE0u
#define GBLUE     0x07FFu
#define RED       0xF800u
#define MAGENTA   0xF81Fu
#define GREEN     0x07E0u
#define CYAN      0x7FFFu
#define YELLOW    0xFFE0u
#define BROWN     0xBC40u
#define BRRED     0xFC07u
#define GRAY      0x8430u
#define DARKBLUE  0x01CFu
#define LIGHTBLUE 0x7D7Cu
#define GRAYBLUE  0x5458u
#define LIGHTGREEN 0x841Fu
#define LGRAY     0xC618u
#define LGRAYBLUE 0xA651u
#define LBBLUE    0x2B12u

#endif /* MC02_LCD_H */
