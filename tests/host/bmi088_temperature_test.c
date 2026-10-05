#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include "bmi088_temperature.h"

typedef struct
{
    uint16_t raw;
    float expected_celsius;
} TemperatureCase;

static void EncodeRaw(uint16_t raw, uint8_t *temp_msb, uint8_t *temp_lsb)
{
    *temp_msb = (uint8_t)(raw >> 3u);
    *temp_lsb = (uint8_t)((raw & 0x7u) << 5u);
}

int main(void)
{
    static const TemperatureCase cases[] = {
        {0u, 23.0f},
        {496u, 85.0f},
        {1023u, 150.875f},
        {1024u, -105.0f},
        {1544u, -40.0f},
        {2047u, 22.875f},
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        uint8_t temp_msb;
        uint8_t temp_lsb;
        EncodeRaw(cases[i].raw, &temp_msb, &temp_lsb);
        float actual_celsius = BMI088DecodeTemperature(temp_msb, temp_lsb);
        if (!(actual_celsius >= cases[i].expected_celsius - 0.001f &&
              actual_celsius <= cases[i].expected_celsius + 0.001f))
        {
            fprintf(stderr,
                    "FAIL raw=%u expected=%.3f actual=%.3f\n",
                    cases[i].raw,
                    (double)cases[i].expected_celsius,
                    (double)actual_celsius);
            return 1;
        }
    }

    puts("PASS: BMI088 11-bit signed temperature decode boundaries");
    return 0;
}
