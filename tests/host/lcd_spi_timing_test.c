/* Production initialization + LCD byte sender; external ST7789 rising-edge model.
 * Datasheet v1.0 sections 8.4.2 and 7.4.3 require data stable around rising SCL.
 * This detects a zero setup window, not measured pin timing or rendered pixels.
 */
#include "lcd.h"
#include <stdio.h>
#include <stdlib.h>

GPIO_TypeDef fake_gpiob, fake_gpioc, fake_gpiod, fake_gpioe;
static unsigned setup_violations, bytes_sent, sleep_out, display_on, bus_errors;
static unsigned check_select_boundary, unexpected_rising_edges, wire_sck, released_sck_level;
static uint8_t last_command;

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL: %s (line %d)\n", #c, __LINE__); return 1; } } while (0)

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *handle)
{
    wire_sck = handle->Init.MasterKeepIOState == SPI_MASTER_KEEP_IO_STATE_ENABLE
                   ? handle->Init.CLKPolarity == SPI_POLARITY_HIGH : released_sck_level;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *cfg) { (void)cfg; return HAL_OK; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *cfg) { (void)port; (void)cfg; }
void HAL_GPIO_DeInit(GPIO_TypeDef *port, uint32_t pins) { (void)port; (void)pins; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pins, uint32_t value)
{
    if (value) port->ODR |= pins; else port->ODR &= ~pins;
}
void HAL_Delay(uint32_t ms) { (void)ms; }
void Error_Handler(void) { abort(); }

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *handle, uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)timeout;
    if (handle->Instance != SPI1 || (fake_gpioe.ODR & LCD_CS_Pin) ||
        handle->Init.DataSize != SPI_DATASIZE_8BIT || handle->Init.FirstBit != SPI_FIRSTBIT_MSB)
        ++bus_errors;
    /* CPHA=0 preloads bit7, launches subsequent bits on trailing edges.
     * CPHA=1 launches bits on leading edges. The panel always samples rising.
     * Model the worst permitted race when launch and sample share an edge.
     */
    unsigned sck = handle->Init.CLKPolarity == SPI_POLARITY_HIGH;
    /* H7 HAL enables SPE here, after the production sender has asserted CS.
     * Inject either possible wire level while the peripheral releases GPIOs.
     * A low->high transition now is an extra panel sample outside the byte.
     */
    if (check_select_boundary && sck && !wire_sck) ++unexpected_rising_edges;
    unsigned launch_on_leading = handle->Init.CLKPhase == SPI_PHASE_2EDGE;
    for (unsigned byte = 0; byte < size; ++byte)
    {
        for (unsigned edge = 0; edge < 16; ++edge)
        {
            unsigned leading = (edge & 1u) == 0;
            sck ^= 1u;
            unsigned launch = leading == launch_on_leading;
            if (sck && launch) ++setup_violations;
        }
        ++bytes_sent;
        if (!(fake_gpiod.ODR & LCD_DC_Pin))
        {
            last_command = data[byte];
            if (last_command == 0x11) ++sleep_out;
            if (last_command == 0x29) ++display_on;
        }
    }
    /* SPI_CloseTransfer disables SPE before LCD_Writ_Bus raises CS. */
    wire_sck = handle->Init.MasterKeepIOState == SPI_MASTER_KEEP_IO_STATE_ENABLE
                   ? sck : released_sck_level;
    return HAL_OK; /* HAL success does not prove external setup/hold compliance. */
}

int main(int argc, char **argv)
{
    (void)argv;
    check_select_boundary = argc > 1;
    for (unsigned repeat = 0; repeat < 2; ++repeat)
    {
        setup_violations = bytes_sent = sleep_out = display_on = bus_errors = 0;
        unexpected_rising_edges = 0;
        released_sck_level = repeat; /* Exercise both low and high released-line states. */
        MX_SPI1_Init();
        LCD_Init();
        CHECK(bytes_sent > 0 && sleep_out == 1 && display_on == 1 && last_command == 0x29);
        CHECK(bus_errors == 0);
        if (setup_violations)
            fprintf(stderr, "ST7789 rising-edge sample overlaps %u MOSI launch edges\n", setup_violations);
        CHECK(setup_violations == 0);
        if (check_select_boundary)
        {
            if (unexpected_rising_edges)
                fprintf(stderr, "ST7789 sees %u extra rising edges after CS assertion\n", unexpected_rising_edges);
            CHECK(unexpected_rising_edges == 0);
        }
    }
    puts("PASS: production LCD init and repeat init keep MOSI stable at panel sampling edges");
    return 0;
}
