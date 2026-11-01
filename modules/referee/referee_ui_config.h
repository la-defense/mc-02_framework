/** @file referee_ui_config.h
 * @brief 声明有界的裁判 UI 布局和只读数据来源。
 * 布局由机器人配置静态保存；本模块读取裁判数据并按布局绘制。
 */
#ifndef REFEREE_UI_CONFIG_H
#define REFEREE_UI_CONFIG_H

#include <stdint.h>

#define REFEREE_UI_MAX_FIELDS 8u
#define REFEREE_UI_LABEL_MAX_BYTES 12u
#define REFEREE_UI_TEXT_MAX_BYTES 30u
#define REFEREE_UI_MIN_FRAME_INTERVAL_MS 115u
#define REFEREE_UI_MAX_FRAME_INTERVAL_MS 1000u

/** @brief 当前支持的只读裁判数据来源。 */
typedef enum {
    REFEREE_UI_SOURCE_GAME_STAGE_REMAIN = 0,
    REFEREE_UI_SOURCE_ROBOT_HP,
    REFEREE_UI_SOURCE_CHASSIS_POWER_LIMIT,
    REFEREE_UI_SOURCE_BUFFER_ENERGY,
    REFEREE_UI_SOURCE_HEAT_17MM
} RefereeUIValueSource_t;

/** @brief 单个文字字段的位置、外观和裁判数据来源。 */
typedef struct {
    uint8_t graphic_name[3]; /**< 协议图形名；同一布局中必须唯一且为可读字节。 */
    const char *label; /**< ASCII短标签，最多REFEREE_UI_LABEL_MAX_BYTES字节。 */
    uint16_t x, y; /**< 屏幕坐标，分别为0～1920和0～1080像素。 */
    uint16_t font_size; /**< 协议字号字段，范围0～511。 */
    uint16_t line_width; /**< 文字线宽，范围0～1023。 */
    uint8_t color; /**< 裁判协议颜色编号，范围0～8。 */
    RefereeUIValueSource_t source; /**< 必须使用本模块已支持的只读来源。 */
} RefereeUIFieldConfig_t;

/** @brief 一套固定容量的车型 UI 布局。 */
typedef struct {
    const RefereeUIFieldConfig_t *fields; /**< 长期有效的字段数组。 */
    uint8_t field_count; /**< 1～REFEREE_UI_MAX_FIELDS。 */
    uint16_t frame_interval_ms; /**< 相邻实际启动间隔，范围115～1000ms。 */
} RefereeUIConfig_t;

/**
 * @brief 检查 UI 布局是否能在固定容量和裁判协议范围内绘制。
 * @param[in] config 长期有效的布局；字段数组必须至少包含field_count项。
 * @return 1表示配置可用，0表示空指针、越界字段、重复名称或无效来源。
 */
uint8_t RefereeUIConfigIsValid(const RefereeUIConfig_t *config);

#endif
