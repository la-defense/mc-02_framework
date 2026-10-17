/** @file led.h
 * @brief 通过 WS2812 显示报警色；关闭报警时显示粉色变化，闪烁计时按 LEDTask 调用次数累计。
 */
#ifndef LED_H
#define LED_H
#include "stdint.h"
#include "math.h"
#include "alarm_def.h"

// 闪烁时间间隔，Daemon任务频率为100Hz，故单位为10ms
#define  HightTime  20
#define  LowTime  30

typedef enum
{
    FLASH_HIGH = 0,
    FLASH_LOW,
    ALWAYS_ON
}flash_e;

typedef enum
{
    RED = 0,
    YELLOW,
    BLUE,
    GREEN,
    PINK
}color_e;

typedef struct
{
    flash_e flash;
    color_e color;
    uint8_t flash_count;
}LED_config_s;

typedef struct
{
    color_e color;
    flash_e flash;
    uint8_t flash_count;
    AlarmState_e alarm_state;
}LEDInstance;


/** @brief 初始化 WS2812 输出，随后还须 LEDRegister() 才能调用 LEDTask()。 */
void LEDInit();
/** @brief 根据最后注册对象的开关和颜色更新灯，须先完成初始化和注册。
 * @note 内部计数每次调用加一；HightTime、LowTime 按约 100 Hz 调用编写，实际节奏随调用频率变化。
 */
void LEDTask();
/** @brief 分配并保存唯一 LED 配置，初始关闭报警。
 * @param[in] config 有效配置地址，不能为 NULL。
 * @return 分配对象，未检查内存失败；重复调用会覆盖内部地址且不释放旧对象，只应注册一次。
 */
LEDInstance *LEDRegister(LED_config_s *config);
/** @brief 改写报警开关；ALARM_OFF 时任务仍显示粉色变化，不是完全熄灯。
 * @param[in,out] led 有效对象，不能为 NULL。
 * @param[in] state 报警开关。
 */
void LEDSetStatus(LEDInstance *led, AlarmState_e state);
/** @brief 改写颜色和闪烁方式，不清内部计数。
 * @param[in,out] led 有效对象，不能为 NULL。
 * @param[in] color 已定义颜色编号。
 * @param[in] flash 快闪、慢闪或常亮，具体节奏依任务调用频率。
 */
void LEDSetFlash(LEDInstance *led, color_e color, flash_e flash);
/** @brief 设置参与快速闪烁计数判断的值，不清内部计数。
 * @param[in,out] led 有效对象，不能为 NULL。
 * @param[in] flash_count 用于决定快速模式何时重置 n，不保证恰好闪此次数后停止。
 */
void LEDSetFlashTime(LEDInstance *led, uint8_t flash_count);
#endif // !LED_H
