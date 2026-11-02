/**
 * @file imu_heater.h
 * @brief 根据 IMU 温度和输入电压控制 TIM3_CH4 加热输出，记录故障并检查恢复条件。
 *
 * INS_Init 先初始化，INS 随后定期调用 Update；标定和 Flash 擦写前后调用 Begin/End。
 * GetStatus 供 LCD 显示。默认禁止加热；启用时仍须通过温度、电压和故障检查。
 */
#ifndef IMU_HEATER_H
#define IMU_HEATER_H

#include <stdint.h>
#include "bmi088_data.h"
#include "../../bsp/board/platform_resources.h"

#ifndef MC02_HEATER_ENABLED
// 构建未设置时保持禁热；非0才允许启动 PWM 并输出加热比较值。
#define MC02_HEATER_ENABLED 0
#endif

extern volatile uint32_t imu_heater_duty_diag; /**< 最近设置的 PWM 比较值，供调试查看。 */

typedef struct
{
    float target_temp;      /**< 目标温度，单位 °C，默认40，设置范围30～45。 */
    float temperature;      /**< 最近接受的温度，单位 °C；当前数据无效时保留旧值。 */
    uint16_t duty;          /**< PWM 比较值，0～500，禁热时为0；数值越大，导通时间越长。 */
    uint8_t sensor_valid;   /**< 最近一次 Update 的温度检查结果，未调用时不刷新。 */
    uint8_t fault;          /**< 已记录故障；温度恢复后也会保留，需通过清除检查。 */
    uint8_t overtemp;       /**< 已记录过温原因。 */
    uint8_t timeout_fault;  /**< 已记录 PWM 连续非0达到300000ms的原因。 */
    uint8_t heating;        /**< duty 非0时为1。 */
    uint8_t initialized;    /**< Init 已完成时为1。 */
    uint8_t enabled;         /**< 当前固件编译允许温控时为1；不代表正在加热。 */
} IMUHeaterStatus_t;

typedef enum
{
    IMU_HEATER_LONG_OPERATION_CALIBRATION = 0, /* 传感器标定 */
    IMU_HEATER_LONG_OPERATION_FLASH,           /* Flash 擦写 */
    IMU_HEATER_LONG_OPERATION_COUNT            /* 类型个数，不能作为操作类型传入 */
} IMUHeater_LongOperation_e;

/** @brief 启动阶段选择温控使用的 PWM 和供电电压输入。
 * @param[in] pwm 车型指定的逻辑 PWM。
 * @param[in] supply 车型指定的逻辑 ADC 电压输入。
 * @return 1 表示两项均已绑定，0 表示未绑定或温控已初始化。
 * @note 只保存编号，不操作硬件；失败保留此前选择。
 */
uint8_t IMUHeaterConfigure(PlatformPwmChannel_t pwm, PlatformAdcChannel_t supply);

/**
 * @brief 首次初始化温控，目标设为40°C，输出置0；需先配置 TIM3。
 * @note 须先调用 IMUHeaterConfigure；未配置或 PID 参数失败时记录故障且不启动 PWM。
 *       禁热也不启动 PWM。
 *       启热且 PID 初始化成功时调用 HAL 启动 CH4，本接口不报告 HAL 启动结果。
 *       重复调用只执行 ForceOff，不重设目标、不清除故障或耗时操作记录。
 */
void IMUHeaterInit(void);

/**
 * @brief 根据本次数据更新温控；先调用 Init，目前由 INS 约每2ms调用一次。
 * @param[in] sample 完整 IMU 数据，接收温度范围为-20～80°C，两端均允许。
 *                  空指针、标记无效或超过5ms时关闭本次输出；本函数不改写数据，不保留传入指针。
 * @param[in] force_off 非0要求本次输出为0；0仍需通过其余检查。
 * @note 同序号只使用已保存的原数据和时间；新序号的采样时间不得早于上次接受的数据。
 *       可用温度达到50°C，或本次温度无效且距上次接受的采样时刻达到500ms时记录故障。
 *       输入电压需在10～30V，两端均允许；ADC 数据需有效且未超过20ms。
 *       连续非零输出达到300000ms时记录故障，温度达到目标加2°C时关闭本次输出。
 *       禁热仍检查数据和故障；未初始化时直接返回，不写硬件。
 *       首轮或关闭后首次可用更新只记录时刻；后续间隔须大于0且不超过20ms。
 *       间隔或 PID 计算失败时输出为0，清除计算历史，后续重新测量间隔。
 */
void IMUHeaterUpdate(const BMI088_Data_t *sample, uint8_t force_off);

/**
 * @brief 将 PWM 比较值和 PID 全部计算历史清0，结束本段连续加热计时。
 * @note 初始化前不写 TIM3；保留故障、采样和耗时操作记录。
 *       本函数不持续禁止加热，后续 Update 仍可按条件恢复输出。
 */
void IMUHeaterForceOff(void);

/**
 * @brief 登记一类耗时操作并立即调用 ForceOff，操作期间 Update 保持零输出。
 * @param[in] operation 标定或 Flash 类型；两类可同时登记，同一类不能重复开始。
 * @return 1 表示登记成功；0 表示编号无效、该类正在进行或曾失败，保持原状态。
 * @note 在 Init 后使用；返回1后必须用相同类型调用 End，耗时是否超限由调用方检查。
 */
uint8_t IMUHeaterBeginLongOperation(IMUHeater_LongOperation_e operation);

/**
 * @brief 结束已登记的耗时操作；类型无效或未开始时不处理。
 * @param[in] operation 与 Begin 相同的操作类型。
 * @param[in] success 非0表示操作及调用方的耗时检查均成功，0表示失败。
 * @note 成功后仍等待序号更新、采样时间不早于结束时刻的数据，也保留已有故障。
 *       失败会记录故障并继续禁止加热，此失败记录不能用 ClearFault 或重复 Init 清除。
 */
void IMUHeaterEndLongOperation(IMUHeater_LongOperation_e operation, uint8_t success);

/**
 * @brief 先调用 ForceOff，再检查当前条件是否允许清除故障。
 * @return 1 表示清除检查通过，0表示拒绝；两种结果都保持本次零输出。
 * @note 需已初始化、最近 Update 未要求强制关闭、没有进行中或失败的耗时操作，且不在等待新数据。
 *       最近一次 Update 的温度检查需通过，所用温度仍在5ms内、至少-20°C且严格低于42°C，不能为 NaN/Inf；
 *       当前 ADC 数据需有效且未超过20ms，输入电压为10～30V，两端均允许。
 *       已记录故障时，还要求序号晚于首次故障所记的序号，采样时间不早于故障时刻。
 *       成功后记录本次使用的数据，仍需再收到序号更新且时间不早于它的数据，才继续温控检查。
 */
uint8_t IMUHeaterClearFault(void);

/**
 * @brief 读取已保存的状态，不重新检查传感器或经过时间。
 * @param[out] status 接收状态；空指针时不处理。温度需结合 sensor_valid 使用。
 * @note 普通任务调用；在短临界区复制同一时刻的温控字段。heating只表示PWM比较值非0，
 *       不代表已测得加热电阻实际电流或温升；本接口不判断热稳定状态。
 */
void IMUHeaterGetStatus(IMUHeaterStatus_t *status);

/**
 * @brief 设置目标温度；不直接改变输出，也不清除故障。
 * @param[in] target_temp 温度，单位 °C；小于30按30，大于45按45，NaN/Inf 时保持原目标。
 * @note 建议在 Init 后设置；首次 Init 会将目标重新设为40°C。
 */
void IMUHeaterSetTarget(float target_temp);

#endif // !IMU_HEATER_H
