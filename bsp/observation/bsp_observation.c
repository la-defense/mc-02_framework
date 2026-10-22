/**
 * @file bsp_observation.c
 * @brief 发布固件身份和可由 OpenOCD 读取的运行统计。
 * @note 身份在首次关闭输出后登记；统计由 LCD 任务周期调用，供主机核对和展示。
 * @author MC-02 contributors
 * @license BSD-3-Clause; see LICENSE.
 */
#include "bsp_observation.h"

#include "bsp_can.h"
#include "bsp_dwt.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#if MC02_OBS_HAS_INS
extern volatile DWT_Probe_t r25_task_ins;
extern volatile DWT_Probe_t ins_prof_all;
extern volatile DWT_Probe_t ins_prof_read;
extern volatile DWT_Probe_t ins_prof_ekf;
extern volatile DWT_Probe_t ins_prof_temp;
#endif
#if MC02_OBS_HAS_MOTOR
extern volatile DWT_Probe_t r25_task_motor;
extern volatile DWT_Probe_t motor_prof_all;
#endif
extern volatile DWT_Probe_t r25_task_daemon;
extern volatile DWT_Probe_t r25_task_robot;
#if MC02_OBS_HAS_DJI
extern volatile DWT_Probe_t dji_prof_pid;
extern volatile DWT_Probe_t dji_prof_send;
#endif
#if MC02_OBS_HAS_CAN
extern volatile DWT_Probe_t can_prof_tx;
extern volatile DWT_Probe_t can_prof_rx_dispatch;
#endif
#if MC02_OBS_HAS_UART
extern volatile DWT_Probe_t usart_tx_task_probe;
#endif

#if MC02_OBS_HAS_LCD
extern volatile uint32_t lcd_prof_sample_last_us;
extern volatile uint32_t lcd_prof_sample_max_us;
extern volatile uint32_t lcd_prof_update_last_us;
extern volatile uint32_t lcd_prof_update_max_us;
extern volatile uint32_t lcd_prof_draw_last_us;
extern volatile uint32_t lcd_prof_draw_max_us;
extern volatile uint32_t lcd_prof_init_us;
extern volatile uint32_t lcd_prof_recover_us;
extern volatile uint32_t lcd_prof_busy_us;
extern volatile uint32_t lcd_prof_cycles;
extern volatile uint32_t lcd_prof_clear_us;
extern volatile uint32_t lcd_prof_row_us;
extern volatile uint16_t lcd_prof_row_bytes;
extern volatile uint32_t lcd_prof_spi_us;
extern volatile uint32_t lcd_prof_rows;
extern volatile uint32_t lcd_spi_bytes;
extern volatile uint32_t lcd_spi_errors;
extern volatile uint8_t lcd_module_absent;
extern volatile uint32_t lcd_recover_count;
extern volatile uint8_t lcd_init_done;
#endif

extern const uint8_t __mc02_build_id_note_start__[];
extern const uint8_t __mc02_build_id_note_end__[];

#define GNU_NOTE_TYPE_BUILD_ID 3u
#define GNU_NOTE_HEADER_BYTES 12u
#define GNU_NOTE_NAME_BYTES 4u
#define GNU_NOTE_BUILD_ID_BYTES 20u

const BSP_ObservationIdentityTemplate_t bsp_observation_identity_template
    __attribute__((used, section(".rodata.mc02_observation_identity"))) = {
        .magic = BSP_OBSERVATION_IDENTITY_MAGIC,
        .version = BSP_OBSERVATION_IDENTITY_VERSION,
        .size_bytes = sizeof(BSP_ObservationIdentity_t),
        .layout = BSP_OBSERVATION_IDENTITY_LAYOUT,
#if defined(MC02_PROFILE_ROBOT)
        .profile = BSP_OBSERVATION_PROFILE_ROBOT,
#else
        .profile = BSP_OBSERVATION_PROFILE_BENCH_SAFE,
#endif
#if defined(CHASSIS_BOARD)
        .role = BSP_OBSERVATION_ROLE_CHASSIS,
#elif defined(GIMBAL_BOARD)
        .role = BSP_OBSERVATION_ROLE_GIMBAL,
#else
        .role = BSP_OBSERVATION_ROLE_SINGLE,
#endif
        .module_mask =
            (MC02_OBS_HAS_INS ? BSP_OBSERVATION_MODULE_INS : 0u) |
            (MC02_OBS_HAS_MOTOR ? BSP_OBSERVATION_MODULE_MOTOR : 0u) |
            (MC02_OBS_HAS_DJI ? BSP_OBSERVATION_MODULE_DJI : 0u) |
            (MC02_OBS_HAS_CAN ? BSP_OBSERVATION_MODULE_CAN : 0u) |
            (MC02_OBS_HAS_UART ? BSP_OBSERVATION_MODULE_UART : 0u) |
            (MC02_OBS_HAS_USB ? BSP_OBSERVATION_MODULE_USB : 0u) |
            (MC02_OBS_HAS_LCD ? BSP_OBSERVATION_MODULE_LCD : 0u),
};

volatile BSP_ObservationIdentity_t bsp_observation_identity;
volatile BSP_ObservationTimingRecord_t bsp_observation_timing;
volatile BSP_ObservationCANRecord_t bsp_observation_can;
volatile BSP_ObservationLCDRecord_t bsp_observation_lcd;

_Static_assert(sizeof(BSP_ObservationIdentity_t) == 12u * sizeof(uint32_t),
               "身份记录字段必须保持连续的 32 位字");
_Static_assert(sizeof(BSP_ObservationTimingRecord_t) == 61u * sizeof(uint32_t),
               "耗时记录字段必须保持连续的 32 位字");
_Static_assert(sizeof(BSP_ObservationCANRecord_t) == 50u * sizeof(uint32_t),
               "CAN 记录字段必须保持连续的 32 位字");
_Static_assert(sizeof(BSP_ObservationLCDRecord_t) == 25u * sizeof(uint32_t),
               "LCD 记录字段必须保持连续的 32 位字");

static uint32_t observation_next_nonzero(uint32_t value)
{
    ++value;
    return (value == 0u) ? 1u : value;
}

static uint8_t observation_build_id_valid(void)
{
    const uint8_t *note = __mc02_build_id_note_start__;
    const uint8_t *note_end = __mc02_build_id_note_end__;
    uint32_t namesz;
    uint32_t descsz;
    uint32_t type;

    if (note == NULL || (uintptr_t)note_end < (uintptr_t)note ||
        (uintptr_t)(note_end - note) != 36u)
        return 0u;

    memcpy(&namesz, note, sizeof(namesz));
    memcpy(&descsz, note + 4u, sizeof(descsz));
    memcpy(&type, note + 8u, sizeof(type));
    return (uint8_t)(namesz == GNU_NOTE_NAME_BYTES && descsz == GNU_NOTE_BUILD_ID_BYTES &&
                     type == GNU_NOTE_TYPE_BUILD_ID && note[12] == 'G' && note[13] == 'N' &&
                     note[14] == 'U' && note[15] == '\0');
}

uint8_t BSPObservationInitEarly(void)
{
    const uint8_t *note = __mc02_build_id_note_start__;
    const BSP_ObservationIdentityTemplate_t *template = &bsp_observation_identity_template;
    volatile BSP_ObservationIdentity_t *identity = &bsp_observation_identity;

    identity->magic = 0u;
    if (!observation_build_id_valid() || template->magic != BSP_OBSERVATION_IDENTITY_MAGIC ||
        template->version != BSP_OBSERVATION_IDENTITY_VERSION ||
        template->size_bytes != sizeof(BSP_ObservationIdentity_t) ||
        template->layout != BSP_OBSERVATION_IDENTITY_LAYOUT)
        return 0u;

    identity->version = template->version;
    identity->size_bytes = template->size_bytes;
    identity->layout = template->layout;
    for (uint32_t index = 0u; index < BSP_OBSERVATION_BUILD_ID_WORDS; ++index)
    {
        uint32_t word;
        memcpy(&word, note + GNU_NOTE_HEADER_BYTES + GNU_NOTE_NAME_BYTES + index * sizeof(word),
               sizeof(word));
        identity->build_id[index] = word;
    }
    identity->profile = template->profile;
    identity->role = template->role;
    identity->module_mask = template->module_mask;
    __DMB();
    identity->magic = template->magic;
    return 1u;
}

static void observation_copy_probe(BSP_ObservationProbe_t *out,
                                   const volatile DWT_Probe_t *in)
{
    out->last_us = in->last_us;
    out->max_us = in->max_us;
    out->total_us = in->total_us;
    out->calls = in->calls;
}

static void observation_publish_timing(const BSP_ObservationTimingRecord_t *value)
{
    volatile BSP_ObservationTimingRecord_t *out = &bsp_observation_timing;
    uint32_t generation = out->generation & ~1u;
    generation += 2u;
    if (generation == 0u)
        generation = 2u;
    out->generation = generation - 1u;
    __DMB();
    out->layout = value->layout;
    out->sample_id = value->sample_id;
    out->captured_ms = value->captured_ms;
    out->module_mask = value->module_mask;
    for (uint32_t index = 0u; index < BSP_OBSERVATION_PROBE_COUNT; ++index)
    {
        out->probes[index].last_us = value->probes[index].last_us;
        out->probes[index].max_us = value->probes[index].max_us;
        out->probes[index].total_us = value->probes[index].total_us;
        out->probes[index].calls = value->probes[index].calls;
    }
    __DMB();
    out->generation = generation;
}

static void observation_publish_can(const BSP_ObservationCANRecord_t *value)
{
    volatile BSP_ObservationCANRecord_t *out = &bsp_observation_can;
    uint32_t generation = out->generation & ~1u;
    generation += 2u;
    if (generation == 0u)
        generation = 2u;
    out->generation = generation - 1u;
    __DMB();
    out->layout = value->layout;
    out->sample_id = value->sample_id;
    out->captured_ms = value->captured_ms;
    out->active_bus_mask = value->active_bus_mask;
    for (uint32_t index = 0u; index < BSP_OBSERVATION_CAN_BUS_COUNT; ++index)
    {
        const BSP_ObservationCANBus_t *src = &value->bus[index];
        volatile BSP_ObservationCANBus_t *dst = &out->bus[index];
        dst->active = src->active;
        dst->tx_ok = src->tx_ok;
        dst->tx_drop = src->tx_drop;
        dst->tx_error = src->tx_error;
        dst->tx_done = src->tx_done;
        dst->tx_fifo_empty_events = src->tx_fifo_empty_events;
        dst->busoff = src->busoff;
        dst->busoff_recover = src->busoff_recover;
        dst->rx_lost = src->rx_lost;
        dst->rx_frames = src->rx_frames;
        dst->rx_queue_drop = src->rx_queue_drop;
        dst->rx_queue_peak = src->rx_queue_peak;
        dst->tx_error_count = src->tx_error_count;
        dst->rx_error_count = src->rx_error_count;
        dst->status = src->status;
    }
    __DMB();
    out->generation = generation;
}

static void observation_publish_lcd(const BSP_ObservationLCDRecord_t *value)
{
    volatile BSP_ObservationLCDRecord_t *out = &bsp_observation_lcd;
    uint32_t generation = out->generation & ~1u;
    generation += 2u;
    if (generation == 0u)
        generation = 2u;
    out->generation = generation - 1u;
    __DMB();
    out->layout = value->layout;
    out->sample_id = value->sample_id;
    out->captured_ms = value->captured_ms;
    out->module_mask = value->module_mask;
    out->sample_last_us = value->sample_last_us;
    out->sample_max_us = value->sample_max_us;
    out->update_last_us = value->update_last_us;
    out->update_max_us = value->update_max_us;
    out->draw_last_us = value->draw_last_us;
    out->draw_max_us = value->draw_max_us;
    out->init_us = value->init_us;
    out->recover_us = value->recover_us;
    out->busy_us = value->busy_us;
    out->cycles = value->cycles;
    out->clear_us = value->clear_us;
    out->row_us = value->row_us;
    out->row_bytes = value->row_bytes;
    out->spi_us = value->spi_us;
    out->rows = value->rows;
    out->spi_bytes = value->spi_bytes;
    out->spi_errors = value->spi_errors;
    out->module_absent = value->module_absent;
    out->recover_count = value->recover_count;
    out->init_done = value->init_done;
    __DMB();
    out->generation = generation;
}

void BSPObservationCapture(void)
{
    BSP_ObservationTimingRecord_t timing = {0};
    BSP_ObservationCANRecord_t can = {0};
    BSP_ObservationLCDRecord_t lcd = {0};
    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();

    timing.layout = BSP_OBSERVATION_TIMING_LAYOUT;
    timing.sample_id = observation_next_nonzero(bsp_observation_timing.sample_id);
    timing.captured_ms = HAL_GetTick();
    timing.module_mask = bsp_observation_identity.module_mask;
#if MC02_OBS_HAS_MOTOR
    observation_copy_probe(&timing.probes[BSP_OBSERVATION_PROBE_TASK_MOTOR], &r25_task_motor);
    observation_copy_probe(&timing.probes[BSP_OBSERVATION_PROBE_MOTOR_ALL], &motor_prof_all);
#endif
#if MC02_OBS_HAS_INS
    observation_copy_probe(&timing.probes[BSP_OBSERVATION_PROBE_TASK_INS], &r25_task_ins);
    observation_copy_probe(&timing.probes[BSP_OBSERVATION_PROBE_INS_ALL], &ins_prof_all);
    observation_copy_probe(&timing.probes[BSP_OBSERVATION_PROBE_INS_READ], &ins_prof_read);
    observation_copy_probe(&timing.probes[BSP_OBSERVATION_PROBE_INS_EKF], &ins_prof_ekf);
    observation_copy_probe(&timing.probes[BSP_OBSERVATION_PROBE_INS_TEMP], &ins_prof_temp);
#endif
    observation_copy_probe(&timing.probes[BSP_OBSERVATION_PROBE_TASK_DAEMON], &r25_task_daemon);
    observation_copy_probe(&timing.probes[BSP_OBSERVATION_PROBE_TASK_ROBOT], &r25_task_robot);
#if MC02_OBS_HAS_DJI
    observation_copy_probe(&timing.probes[BSP_OBSERVATION_PROBE_DJI_PID], &dji_prof_pid);
    observation_copy_probe(&timing.probes[BSP_OBSERVATION_PROBE_DJI_SEND], &dji_prof_send);
#endif
#if MC02_OBS_HAS_CAN
    observation_copy_probe(&timing.probes[BSP_OBSERVATION_PROBE_CAN_TX], &can_prof_tx);
    observation_copy_probe(&timing.probes[BSP_OBSERVATION_PROBE_CAN_DISPATCH], &can_prof_rx_dispatch);
    can.layout = BSP_OBSERVATION_CAN_LAYOUT;
    can.sample_id = observation_next_nonzero(bsp_observation_can.sample_id);
    can.captured_ms = timing.captured_ms;
    for (uint8_t index = 0u; index < BSP_OBSERVATION_CAN_BUS_COUNT; ++index)
    {
        CAN_ObservationCounters_t counts;
        if (CANGetObservationCounters(index, &counts))
        {
            BSP_ObservationCANBus_t *bus = &can.bus[index];
            bus->active = counts.active;
            bus->tx_ok = counts.tx_ok;
            bus->tx_drop = counts.tx_drop;
            bus->tx_error = counts.tx_error;
            bus->tx_done = counts.tx_done;
            bus->tx_fifo_empty_events = counts.tx_fifo_empty_events;
            bus->busoff = counts.busoff;
            bus->busoff_recover = counts.busoff_recover;
            bus->rx_lost = counts.rx_lost;
            bus->rx_frames = counts.rx_frames;
            bus->rx_queue_drop = counts.rx_queue_drop;
            bus->rx_queue_peak = counts.rx_queue_peak;
            bus->tx_error_count = counts.tx_error_count;
            bus->rx_error_count = counts.rx_error_count;
            bus->status = counts.status;
            if (counts.active)
                can.active_bus_mask |= 1u << index;
        }
    }
#endif
#if MC02_OBS_HAS_UART
    observation_copy_probe(&timing.probes[BSP_OBSERVATION_PROBE_UART_TX], &usart_tx_task_probe);
#endif

    lcd.layout = BSP_OBSERVATION_LCD_LAYOUT;
    lcd.sample_id = observation_next_nonzero(bsp_observation_lcd.sample_id);
    lcd.captured_ms = timing.captured_ms;
    lcd.module_mask = bsp_observation_identity.module_mask;
#if MC02_OBS_HAS_LCD
    lcd.sample_last_us = lcd_prof_sample_last_us;
    lcd.sample_max_us = lcd_prof_sample_max_us;
    lcd.update_last_us = lcd_prof_update_last_us;
    lcd.update_max_us = lcd_prof_update_max_us;
    lcd.draw_last_us = lcd_prof_draw_last_us;
    lcd.draw_max_us = lcd_prof_draw_max_us;
    lcd.init_us = lcd_prof_init_us;
    lcd.recover_us = lcd_prof_recover_us;
    lcd.busy_us = lcd_prof_busy_us;
    lcd.cycles = lcd_prof_cycles;
    lcd.clear_us = lcd_prof_clear_us;
    lcd.row_us = lcd_prof_row_us;
    lcd.row_bytes = lcd_prof_row_bytes;
    lcd.spi_us = lcd_prof_spi_us;
    lcd.rows = lcd_prof_rows;
    lcd.spi_bytes = lcd_spi_bytes;
    lcd.spi_errors = lcd_spi_errors;
    lcd.module_absent = lcd_module_absent;
    lcd.recover_count = lcd_recover_count;
    lcd.init_done = lcd_init_done;
#endif

    if (saved_primask == 0u)
        __enable_irq();

    observation_publish_timing(&timing);
#if MC02_OBS_HAS_CAN
    observation_publish_can(&can);
#endif
    observation_publish_lcd(&lcd);
}
