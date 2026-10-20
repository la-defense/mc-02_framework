/** @file spi.h
 * @brief SPI／DMA／NVIC 的主机寄存器模型；仅供生产 SPI 测试，不连接硬件。
 */
#ifndef MC02_HOST_SPI_FAKE_SPI_H
#define MC02_HOST_SPI_FAKE_SPI_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef int IRQn_Type;
typedef struct { uint32_t CR1, CFG1, IER; } SPI_TypeDef;
typedef struct { uint32_t CR; } DMA_Stream_TypeDef;
extern SPI_TypeDef spi_registers[2];
extern DMA_Stream_TypeDef dma_registers[16];
#define SPI1 (&spi_registers[0])
#define SPI2 (&spi_registers[1])
#define SPI1_IRQn 16
#define SPI2_IRQn 17
#define DMA1_Stream0 (&dma_registers[0])
#define DMA1_Stream0_IRQn 0
#define DMA1_Stream1 (&dma_registers[1])
#define DMA1_Stream1_IRQn 1
#define DMA1_Stream2 (&dma_registers[2])
#define DMA1_Stream2_IRQn 2
#define DMA1_Stream3 (&dma_registers[3])
#define DMA1_Stream3_IRQn 3
#define DMA1_Stream4 (&dma_registers[4])
#define DMA1_Stream4_IRQn 4
#define DMA1_Stream5 (&dma_registers[5])
#define DMA1_Stream5_IRQn 5
#define DMA1_Stream6 (&dma_registers[6])
#define DMA1_Stream6_IRQn 6
#define DMA1_Stream7 (&dma_registers[7])
#define DMA1_Stream7_IRQn 7
#define DMA2_Stream0 (&dma_registers[8])
#define DMA2_Stream0_IRQn 8
#define DMA2_Stream1 (&dma_registers[9])
#define DMA2_Stream1_IRQn 9
#define DMA2_Stream2 (&dma_registers[10])
#define DMA2_Stream2_IRQn 10
#define DMA2_Stream3 (&dma_registers[11])
#define DMA2_Stream3_IRQn 11
#define DMA2_Stream4 (&dma_registers[12])
#define DMA2_Stream4_IRQn 12
#define DMA2_Stream5 (&dma_registers[13])
#define DMA2_Stream5_IRQn 13
#define DMA2_Stream6 (&dma_registers[14])
#define DMA2_Stream6_IRQn 14
#define DMA2_Stream7 (&dma_registers[15])
#define DMA2_Stream7_IRQn 15
#define SPI_CR1_SPE 1u
#define SPI_CR1_CSTART 2u
#define SPI_CFG1_TXDMAEN 1u
#define SPI_CFG1_RXDMAEN 2u
#define DMA_SxCR_EN 1u
#define SPI_IT_EOT 1u
#define SPI_IT_TXP 2u
#define SPI_IT_RXP 4u
#define SPI_IT_DXP 8u
#define SPI_IT_UDR 16u
#define SPI_IT_OVR 32u
#define SPI_IT_FRE 64u
#define SPI_IT_MODF 128u
#define SPI_MODE_MASTER 1u
#define SPI_DATASIZE_8BIT 8u
#define SPI_DIRECTION_2LINES 0u
#define SPI_DIRECTION_2LINES_TXONLY 1u
#define DMA_MEMORY_TO_PERIPH 1u
#define DMA_PERIPH_TO_MEMORY 2u
#define DMA_NORMAL 0u
#define DMA_MINC_ENABLE 1u
#define DMA_PINC_DISABLE 0u
#define DMA_MDATAALIGN_BYTE 0u
#define DMA_PDATAALIGN_BYTE 0u
typedef enum { HAL_DMA_STATE_READY, HAL_DMA_STATE_BUSY } HAL_DMA_StateTypeDef;
typedef enum { HAL_SPI_STATE_READY, HAL_SPI_STATE_BUSY } HAL_SPI_StateTypeDef;
typedef struct {
 DMA_Stream_TypeDef *Instance;
 void *Parent;
 struct { uint32_t Direction, Mode, MemInc, PeriphInc, MemDataAlignment, PeriphDataAlignment; } Init;
 HAL_DMA_StateTypeDef State;
} DMA_HandleTypeDef;
typedef struct {
 SPI_TypeDef *Instance;
 struct { uint32_t Mode, DataSize, Direction; } Init;
 HAL_SPI_StateTypeDef State;
 DMA_HandleTypeDef *hdmatx, *hdmarx;
} SPI_HandleTypeDef;
extern _Thread_local uint32_t mc02_test_ipsr, mc02_test_primask, mc02_test_basepri, mc02_test_faultmask;
void mc02_test_disable_irq(void);
void mc02_test_set_primask(uint32_t);
#define __get_IPSR() (mc02_test_ipsr)
#define __get_PRIMASK() (mc02_test_primask)
#define __get_BASEPRI() (mc02_test_basepri)
#define __get_FAULTMASK() (mc02_test_faultmask)
#define __disable_irq() mc02_test_disable_irq()
#define __set_PRIMASK(value) mc02_test_set_primask(value)
uint32_t NVIC_GetEnableIRQ(IRQn_Type);
void NVIC_ClearPendingIRQ(IRQn_Type);
void mc02_dma_clear(DMA_HandleTypeDef *, uint32_t);
#define __HAL_DMA_CLEAR_FLAG(dma, flags) mc02_dma_clear(dma, flags)
#define __HAL_DMA_GET_TC_FLAG_INDEX(dma) 1u
#define __HAL_DMA_GET_HT_FLAG_INDEX(dma) 2u
#define __HAL_DMA_GET_TE_FLAG_INDEX(dma) 4u
#define __HAL_DMA_GET_DME_FLAG_INDEX(dma) 8u
#define __HAL_DMA_GET_FE_FLAG_INDEX(dma) 16u
uint32_t HAL_GetTick(void);
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *);
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t);

#endif
