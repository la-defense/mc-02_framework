/** @file bmi088_calibration.h
 * @brief 声明 BMI088 标定状态和运行中标定控制，不包含板级传感器类型。
 */
#ifndef MC02_BMI088_CALIBRATION_H
#define MC02_BMI088_CALIBRATION_H

#include <stdint.h>

/** @brief 当前标定参数的来源。 */
typedef enum
{
    BMI088_CALIB_SRC_NONE = 0, /**< 尚未确定来源。 */
    BMI088_CALIB_SRC_FLASH, /**< 读取已保存的参数。 */
    BMI088_CALIB_SRC_FIRST_AUTO, /**< 首次自动标定并保存。 */
    BMI088_CALIB_SRC_MANUAL, /**< 运行中手动标定并保存。 */
    BMI088_CALIB_SRC_DEFAULT /**< 标定失败后使用默认参数。 */
} BMI088_CalibSource_e;

extern volatile uint8_t bmi088_calib_source; /**< BMI088_CalibSource_e 中的来源值。 */
extern volatile float bmi088_calib_temp; /**< 标定温度，单位 °C。 */

/** @brief 按需标定的运行状态。 */
typedef enum
{
    BMI088_RECALIB_IDLE = 0, /**< 当前没有标定请求。 */
    BMI088_RECALIB_BUSY,     /**< 请求已受理，标定或参数写入正在进行。 */
    BMI088_RECALIB_OK,       /**< 标定成功且结果已经保存。 */
    BMI088_RECALIB_FAIL      /**< 标定失败，旧参数仍保留。 */
} BMI088_RecalibState_e;

extern volatile uint8_t bmi088_calib_attempts; /**< 最近一次标定使用的轮数。 */

/** @brief 请求 INS 任务执行一次运行中标定。
 * @return 1 表示请求已受理，0 表示没有可用传感器或已有请求。
 */
uint8_t BMI088CalibRequest(void);

/** @brief 查询当前按需标定状态。
 * @return BMI088_RecalibState_e 中的状态值。
 */
uint8_t BMI088CalibGetState(void);

/** @brief 显示完成功或失败结果后，将结果状态清回空闲。
 */
void BMI088CalibAckResult(void);

#endif
