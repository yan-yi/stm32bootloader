#ifndef  __SPI1_H
#define  __SPI1_H
#include "stm32f0xx_hal.h"

extern uint32_t Spi1Timeout; /*<! Value of Timeout when SPI communication fails */
extern SPI_HandleTypeDef spi1_handle;

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void SPI1_Init(void);
void SPI1_Uninit(void);
void SPI1_Error (void);
HAL_StatusTypeDef SPI1_WriteData(uint8_t *DataIn, uint16_t DataLength);
HAL_StatusTypeDef SPI1_ReadData(uint8_t *DataIn, uint16_t DataLength);
HAL_StatusTypeDef SPI1_ReadData_DMA(uint8_t *DataIn, uint16_t DataLength);
void SPI1_FlushFifo(void);

#endif/*__SPI1_H*/



