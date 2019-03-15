

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#if defined(HAL_SPI_MODULE_ENABLED)

uint32_t Spi1Timeout = SPIFLASH_SPI1_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */
SPI_HandleTypeDef hnucleo_Spi1;



/**
  * @brief  Initialize SPI MSP.
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};

  if (hspi->Instance == SPI1)
  {

    /*** Configure the GPIOs ***/
    /* Enable GPIO clock */
    SPIFLASH_SPI1_SCK_GPIO_CLK_ENABLE();
    SPIFLASH_SPI1_MISO_MOSI_GPIO_CLK_ENABLE();

    /*** Configure the SPI peripheral ***/
    /* Enable SPI clock */
    SPIFLASH_SPI1_CLK_ENABLE();

    /* Configure SPI SCK */
    gpioinitstruct.Pin = SPIFLASH_SPI1_SCK_PIN;
    gpioinitstruct.Mode = GPIO_MODE_AF_PP;
    gpioinitstruct.Pull  = GPIO_PULLUP;
    gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpioinitstruct.Alternate = SPIFLASH_SPI1_SCK_AF;
    HAL_GPIO_Init(SPIFLASH_SPI1_SCK_GPIO_PORT, &gpioinitstruct);

    /* Configure SPI MISO and MOSI */
    gpioinitstruct.Pin = SPIFLASH_SPI1_MOSI_PIN;
    gpioinitstruct.Alternate = SPIFLASH_SPI1_MISO_MOSI_AF;
    gpioinitstruct.Pull  = GPIO_PULLDOWN;
    HAL_GPIO_Init(SPIFLASH_SPI1_MISO_MOSI_GPIO_PORT, &gpioinitstruct);

    gpioinitstruct.Pin = SPIFLASH_SPI1_MISO_PIN;
    HAL_GPIO_Init(SPIFLASH_SPI1_MISO_MOSI_GPIO_PORT, &gpioinitstruct);


  }
  else if (hspi->Instance == SPI2)
  {
//    /*** Configure the GPIOs ***/
//    /* Enable GPIO clock */
//    LCD_SPI2_SCK_GPIO_CLK_ENABLE();
//    LCD_SPI2_MISO_MOSI_GPIO_CLK_ENABLE();

//    /*** Configure the SPI peripheral ***/
//    /* Enable SPI clock */
//    LCD_SPI2_CLK_ENABLE();

//    /* Configure SPI SCK */
//    gpioinitstruct.Pin = LCD_SPI2_SCK_PIN;
//    gpioinitstruct.Mode = GPIO_MODE_AF_PP;
//    gpioinitstruct.Pull  = GPIO_PULLUP;
//    gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    gpioinitstruct.Alternate = LCD_SPI2_SCK_AF;
//    HAL_GPIO_Init(LCD_SPI2_SCK_GPIO_PORT, &gpioinitstruct);

//    /* Configure SPI MISO and MOSI */
//    gpioinitstruct.Pin = LCD_SPI2_MOSI_PIN;
//    gpioinitstruct.Alternate = LCD_SPI2_MISO_MOSI_AF;
//    gpioinitstruct.Pull  = GPIO_PULLDOWN;
//    HAL_GPIO_Init(LCD_SPI2_MISO_MOSI_GPIO_PORT, &gpioinitstruct);

//    gpioinitstruct.Pin = LCD_SPI2_MISO_PIN;
//    HAL_GPIO_Init(LCD_SPI2_MISO_MOSI_GPIO_PORT, &gpioinitstruct);

  }
}

/**
  * @brief  Initialize SPI HAL.
  * @retval None
  */
static void SPI1_Init(void)
{
  if (HAL_SPI_GetState(&hnucleo_Spi1) == HAL_SPI_STATE_RESET)
  {
    /* SPI Config */
    hnucleo_Spi1.Instance = SPIFLASH_SPI1;
    /* SPI baudrate is set to 12 MHz maximum (PCLK1/SPI_BaudRatePrescaler = 48/4 = 12 MHz)
     to verify these constraints:
        - ST7735 LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
          Since the provided driver doesn't use read capability from LCD, only constraint
          on write baudrate is considered.
        - SD card SPI interface max baudrate is 25MHz for write/read
        - PCLK1 max frequency is 48 MHz
     */
    hnucleo_Spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;//SPI_BAUDRATEPRESCALER_4;
    hnucleo_Spi1.Init.Direction = SPI_DIRECTION_2LINES;
    hnucleo_Spi1.Init.CLKPhase = SPI_PHASE_1EDGE;//SPI_PHASE_2EDGE;
    hnucleo_Spi1.Init.CLKPolarity = SPI_POLARITY_LOW;//SPI_POLARITY_HIGH;//
    hnucleo_Spi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hnucleo_Spi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hnucleo_Spi1.Init.CRCPolynomial = 7;
    hnucleo_Spi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hnucleo_Spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hnucleo_Spi1.Init.NSS = SPI_NSS_SOFT;
    hnucleo_Spi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hnucleo_Spi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hnucleo_Spi1.Init.Mode = SPI_MODE_MASTER;

    HAL_SPI_Init(&hnucleo_Spi1);
  }
}

void SPI1_Uninit(void)
{
  HAL_SPI_DeInit(&hnucleo_Spi1);

  SPIFLASH_SPI1_CLK_DISABLE();

  SPIFLASH_CS_HIGH();

}

/**
  * @brief  SPI error treatment function
  * @retval None
  */
static void SPI1_Error (void)
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&hnucleo_Spi1);

  /* Re-Initiaize the SPI communication BUS */
  SPI1_Init();
}

/**
  * @brief  SPI Write an amount of data to device
  * @param  Value: value to be written
  * @param  DataLength: number of bytes to write
  * @retval None
  */
static HAL_StatusTypeDef SPI1_WriteData(uint8_t *DataIn, uint16_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Transmit(&hnucleo_Spi1, DataIn, DataLength, Spi1Timeout);
  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPI1_Error();
  }

  return status;
}

static HAL_StatusTypeDef SPI1_ReadData(uint8_t *DataIn, uint16_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Receive(&hnucleo_Spi1, DataIn, DataLength, Spi1Timeout);
  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPI1_Error();
  }

  return status;
}


static HAL_StatusTypeDef SPI1_ReadData_DMA(uint8_t *DataIn, uint16_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Receive(&hnucleo_Spi1, DataIn, DataLength, Spi1Timeout);
  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPI1_Error();
  }

  return status;
}

/**
  * @brief  SPI1_FlushFifo
  * @retval None
  */
static void SPI1_FlushFifo(void)
{
  HAL_SPIEx_FlushRxFifo(&hnucleo_Spi1);
}


///**
//  * @brief  Initialize SPI HAL.
//  * @retval None
//  */
//static void SPI2_Init(void)
//{
//  if (HAL_SPI_GetState(&hnucleo_Spi2) == HAL_SPI_STATE_RESET)
//  {
//    /* SPI Config */
//    hnucleo_Spi2.Instance = LCD_SPI2;
//    /* SPI baudrate is set to 12 MHz maximum (PCLK1/SPI_BaudRatePrescaler = 48/4 = 12 MHz)
//     to verify these constraints:
//        - ST7735 LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
//          Since the provided driver doesn't use read capability from LCD, only constraint
//          on write baudrate is considered.
//        - SD card SPI interface max baudrate is 25MHz for write/read
//        - PCLK1 max frequency is 48 MHz
//     */
//    hnucleo_Spi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;//SPI_BAUDRATEPRESCALER_2;//
//    hnucleo_Spi2.Init.Direction = SPI_DIRECTION_2LINES;
//    hnucleo_Spi2.Init.CLKPhase = SPI_PHASE_2EDGE;
//    hnucleo_Spi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
//    hnucleo_Spi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//    hnucleo_Spi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
//    hnucleo_Spi2.Init.CRCPolynomial = 7;
//    hnucleo_Spi2.Init.DataSize = SPI_DATASIZE_8BIT;
//    hnucleo_Spi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
//    hnucleo_Spi2.Init.NSS = SPI_NSS_SOFT;
//    hnucleo_Spi2.Init.TIMode = SPI_TIMODE_DISABLE;
//    hnucleo_Spi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
//    hnucleo_Spi2.Init.Mode = SPI_MODE_MASTER;

//    HAL_SPI_Init(&hnucleo_Spi2);
//  }
//}

//void SPI2_Uninit(void)
//{
//  HAL_SPI_DeInit(&hnucleo_Spi2);
//}
///**
//  * @brief  SPI error treatment function
//  * @retval None
//  */
//static void SPI2_Error (void)
//{
//  /* De-initialize the SPI communication BUS */
//  //HAL_SPI_DeInit(&hnucleo_Spi2);

//  /* Re-Initiaize the SPI communication BUS */
//  //SPI2_Init();
//}

//#if 0
///**
//  * @brief  SPI Write a byte to device
//  * @param  Value: value to be written
//  * @retval None
//  */
//static void SPI2_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
//{
//  HAL_StatusTypeDef status = HAL_OK;

//  status = HAL_SPI_TransmitReceive(&hnucleo_Spi2, (uint8_t*) DataIn, DataOut, DataLength, Spi2Timeout);

//  /* Check the communication status */
//  if (status != HAL_OK)
//  {
//    /* Execute user timeout callback */
//    SPI2_Error();
//  }
//}
//#endif

///**
//  * @brief  SPI Write an amount of data to device
//  * @param  Value: value to be written
//  * @param  DataLength: number of bytes to write
//  * @retval None
//  */
//static void SPI2_WriteData(uint8_t *DataIn, uint16_t DataLength)
//{
//  HAL_StatusTypeDef status = HAL_OK;

//  status = HAL_SPI_Transmit(&hnucleo_Spi2, DataIn, DataLength, Spi2Timeout);

//  /* Check the communication status */
//  if (status != HAL_OK)
//  {
//    /* Execute user timeout callback */
//    SPI2_Error();
//  }
//}

///**
//  * @brief  SPI Write a byte to device
//  * @param  Value: value to be written
//  * @retval None
//  */
//static void SPI2_Write(uint8_t Value)
//{
//  HAL_StatusTypeDef status = HAL_OK;
//  uint8_t data;

//  status = HAL_SPI_TransmitReceive(&hnucleo_Spi2, (uint8_t*) &Value, &data, 1, Spi2Timeout);

//  /* Check the communication status */
//  if (status != HAL_OK)
//  {
//    /* Execute user timeout callback */
//    SPI2_Error();
//  }
//}

///**
//  * @brief  SPI2_FlushFifo
//  * @retval None
//  */
//static void SPI2_FlushFifo(void)
//{
//  HAL_SPIEx_FlushRxFifo(&hnucleo_Spi2);
//}



/******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/********************************* LINK SPIFLASH ************************************/
/**
  * @brief  Initialize the SD Card and put it into StandBy State (Ready for
  *         data transfer).
  * @retval None
  */

void SPIFLASH_IO_Init(void)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};

  /* SD_CS_GPIO Periph clock enable */
  SPIFLASH_CS_GPIO_CLK_ENABLE();

  /* Configure SD_CS_PIN pin: SD Card CS pin */
  gpioinitstruct.Pin = SPIFLASH_CS_PIN;
  gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull = GPIO_NOPULL;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPIFLASH_CS_GPIO_PORT, &gpioinitstruct);

  /* SD chip select high */
  SPIFLASH_CS_HIGH();
  /*------------Put SD in SPI mode--------------*/
  /* SD SPI Config */
  SPI1_Init();


}



/**
  * @brief  Write an amount of data on the SD.
  * @param  Data: byte to send.
  * @param  DataLength: number of bytes to write
  * @retval none
  */
HAL_StatusTypeDef SPIFLASH_IO_ReadData(uint8_t *DataOut, uint16_t DataLength)
{
  return SPI1_ReadData(DataOut, DataLength);
}

HAL_StatusTypeDef SPIFLASH_IO_ReadData_DMA(uint8_t *DataOut, uint16_t DataLength)
{
  return SPI1_ReadData_DMA(DataOut, DataLength);
}

/**
  * @brief  Write an amount of data on the SD.
  * @param  Data: byte to send.
  * @param  DataLength: number of bytes to write
  * @retval none
  */
HAL_StatusTypeDef SPIFLASH_IO_WriteData(const uint8_t *Data, uint16_t DataLength)
{
  HAL_StatusTypeDef sta;
  /* Send the byte */
  sta = SPI1_WriteData((uint8_t *)Data, DataLength);

  SPI1_FlushFifo();

  return sta;
}

#endif /*HAL_SPI_MODULE_ENABLED*/

