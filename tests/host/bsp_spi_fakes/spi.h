#ifndef MC02_HOST_SPI_FAKE_SPI_H
#define MC02_HOST_SPI_FAKE_SPI_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { void *Instance; } SPI_HandleTypeDef;
#define SPI1 ((void *)1)
#define SPI2 ((void *)2)

extern uint32_t mc02_test_ipsr;
extern _Thread_local uint32_t mc02_test_primask;
extern uint32_t mc02_test_basepri;
extern uint32_t mc02_test_faultmask;
void mc02_test_disable_irq(void);
void mc02_test_set_primask(uint32_t value);
#define __get_IPSR() (mc02_test_ipsr)
#define __get_PRIMASK() (mc02_test_primask)
#define __get_BASEPRI() (mc02_test_basepri)
#define __get_FAULTMASK() (mc02_test_faultmask)
#define __disable_irq() mc02_test_disable_irq()
#define __set_PRIMASK(value) mc02_test_set_primask(value)

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);

#endif
