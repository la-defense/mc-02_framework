#ifndef MC02_HOST_BMI088_FAKE_BSP_SPI_H
#define MC02_HOST_BMI088_FAKE_BSP_SPI_H

#include <stdint.h>
#include <string.h>

typedef enum
{
    SPI_BLOCK_MODE = 0,
    SPI_IT_MODE,
    SPI_DMA_MODE
} SPI_TXRX_MODE_e;

typedef struct spi_ins_temp SPIInstance;
struct spi_ins_temp
{
    SPI_TXRX_MODE_e spi_work_mode;
    void (*callback)(SPIInstance *);
    void *id;
    uint8_t is_accelerometer;
};

typedef struct
{
    SPI_TXRX_MODE_e spi_work_mode;
    void (*callback)(SPIInstance *);
    void *id;
} SPI_Init_Config_s;

SPIInstance *SPIRegister(SPI_Init_Config_s *config);
void SPITransmit(SPIInstance *spi, uint8_t *tx, uint8_t len);
void SPITransRecv(SPIInstance *spi, uint8_t *rx, uint8_t *tx, uint8_t len);

#endif
