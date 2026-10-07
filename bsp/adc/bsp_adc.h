#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

#ifndef BSP_ADC_VREF_MV
#define BSP_ADC_VREF_MV 3300u
#endif

#ifndef BSP_ADC_FULL_SCALE
#define BSP_ADC_FULL_SCALE 65535u
#endif

#ifndef BSP_ADC_VCC_IN_DIVIDER_MILLI
#define BSP_ADC_VCC_IN_DIVIDER_MILLI 11000u
#endif

#define BSP_ADC_DMA_BATCH_PAIR_COUNT 32u
#define BSP_ADC_SAMPLE_MAX_AGE_MS 20u

typedef enum
{
    BSP_ADC_STATUS_NOT_READY = 0,
    BSP_ADC_STATUS_VALID,
    BSP_ADC_STATUS_STALE,
    BSP_ADC_STATUS_ERROR,
    BSP_ADC_STATUS_INVALID_ARGUMENT
} BSP_ADC_Status_e;

typedef enum
{
    BSP_ADC_ERROR_NONE = 0u,
    BSP_ADC_ERROR_CALIBRATION = 1u << 0,
    BSP_ADC_ERROR_DMA_START = 1u << 1,
    BSP_ADC_ERROR_RUNTIME = 1u << 2
} BSP_ADC_Error_e;

typedef struct
{
    uint16_t raw_vcc_in;
    uint16_t raw_key;
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t error_flags;
    uint32_t hal_error_code;
    float vcc_in_volts;
    /* Raw VCC/key transport validity is separate from VIN conversion validity. */
    uint8_t voltage_config_valid;
} BSP_ADC_Sample_t;

void BSP_ADCInit(void);
/* Reads the latest completed VIN/key batch and validates its age and ADC error state. */
BSP_ADC_Status_e BSP_ADCGetSample(BSP_ADC_Sample_t *sample);

#endif // !BSP_ADC_H
