#include "stdio.h"
#include "string.h"
#include "stm32f0xx_hal.h"
#include "spi1.h"

#ifdef HAL_SPI_MODULE_ENABLED

#define SPI1_CLK_ENABLE()                  __HAL_RCC_SPI1_CLK_ENABLE()
#define SPI1_CLK_DISABLE()                 __HAL_RCC_SPI1_CLK_DISABLE()
#define SPI1_FORCE_RESET()                 __HAL_RCC_SPI1_FORCE_RESET()
#define SPI1_RELEASE_RESET()               __HAL_RCC_SPI1_RELEASE_RESET()


#define SPI1_SCK_AF                          GPIO_AF0_SPI1
#define SPI1_SCK_GPIO_PORT                   GPIOB
#define SPI1_SCK_PIN                         GPIO_PIN_3
#define SPI1_SCK_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI1_SCK_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE()

#define SPI1_MISO_MOSI_AF                    GPIO_AF0_SPI1
#define SPI1_MISO_MOSI_GPIO_PORT             GPIOB
#define SPI1_MISO_MOSI_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI1_MISO_MOSI_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()
#define SPI1_MISO_PIN                        GPIO_PIN_4
#define SPI1_MOSI_PIN                        GPIO_PIN_5
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define SPI1_TIMEOUT_MAX                   1000UL


uint32_t Spi1Timeout = SPI1_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */
SPI_HandleTypeDef spi1_handle;

/**
  * @brief SPI MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    /*##-1- Reset peripherals ##################################################*/
    SPI1_FORCE_RESET();
    SPI1_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    /* Configure SPI SCK as alternate function  */
    HAL_GPIO_DeInit(SPI1_SCK_GPIO_PORT, SPI1_SCK_PIN);
    /* Configure SPI MISO as alternate function  */
    HAL_GPIO_DeInit(SPI1_MISO_MOSI_GPIO_PORT, SPI1_MISO_PIN);
    /* Configure SPI MOSI as alternate function  */
    HAL_GPIO_DeInit(SPI1_MISO_MOSI_GPIO_PORT, SPI1_MOSI_PIN);
  }
}


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
    SPI1_SCK_GPIO_CLK_ENABLE();
    SPI1_MISO_MOSI_GPIO_CLK_ENABLE();

    /*** Configure the SPI peripheral ***/
    /* Enable SPI clock */
    SPI1_CLK_ENABLE();

    //__HAL_RCC_DMA1_CLK_ENABLE();

    /* Configure SPI SCK */
    gpioinitstruct.Pin = SPI1_SCK_PIN;
    gpioinitstruct.Mode = GPIO_MODE_AF_PP;
    gpioinitstruct.Pull  = GPIO_PULLUP;
    gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpioinitstruct.Alternate = SPI1_SCK_AF;
    HAL_GPIO_Init(SPI1_SCK_GPIO_PORT, &gpioinitstruct);

    /* Configure SPI MISO and MOSI */
    gpioinitstruct.Pin = SPI1_MOSI_PIN;
    gpioinitstruct.Alternate = SPI1_MISO_MOSI_AF;
    gpioinitstruct.Pull  = GPIO_PULLDOWN;
    HAL_GPIO_Init(SPI1_MISO_MOSI_GPIO_PORT, &gpioinitstruct);

    gpioinitstruct.Pin = SPI1_MISO_PIN;
    HAL_GPIO_Init(SPI1_MISO_MOSI_GPIO_PORT, &gpioinitstruct);

#if 0
    /*##-3- Configure the DMA ##################################################*/
    /* Configure the DMA handler for Transmission process */
    hdma_tx.Instance                 = DMA1_Channel5;
    hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_tx.Init.Mode                = DMA_NORMAL;
    hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;

    HAL_DMA_Init(&hdma_tx);

    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(&spi1_handle, hdmatx, hdma_tx);

    /* Configure the DMA handler for Transmission process */
    hdma_rx.Instance                 = DMA1_Channel2;

    hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_rx.Init.Mode                = DMA_NORMAL;
    hdma_rx.Init.Priority            = DMA_PRIORITY_LOW;//DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&hdma_rx);

    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(&spi1_handle, hdmarx, hdma_rx);

    /*##-4- Configure the NVIC for DMA #########################################*/
    /* NVIC configuration for DMA transfer complete interrupt (SPI3_TX) */
    HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt (SPI3_RX) */
    HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
#endif

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
void SPI1_Init(void)
{
  if (HAL_SPI_GetState(&spi1_handle) == HAL_SPI_STATE_RESET)
  {
    /* SPI Config */
    spi1_handle.Instance = SPI1;
    /* SPI baudrate is set to 12 MHz maximum (PCLK1/SPI_BaudRatePrescaler = 48/4 = 12 MHz)
     to verify these constraints:
        - ST7735 LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
          Since the provided driver doesn't use read capability from LCD, only constraint
          on write baudrate is considered.
        - SD card SPI interface max baudrate is 25MHz for write/read
        - PCLK1 max frequency is 48 MHz
     */
    spi1_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;//SPI_BAUDRATEPRESCALER_4;
    spi1_handle.Init.Direction = SPI_DIRECTION_2LINES;
    spi1_handle.Init.CLKPhase = SPI_PHASE_1EDGE;//SPI_PHASE_2EDGE;
    spi1_handle.Init.CLKPolarity = SPI_POLARITY_LOW;//SPI_POLARITY_HIGH;//
    spi1_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi1_handle.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    spi1_handle.Init.CRCPolynomial = 7;
    spi1_handle.Init.DataSize = SPI_DATASIZE_8BIT;
    spi1_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi1_handle.Init.NSS = SPI_NSS_SOFT;
    spi1_handle.Init.TIMode = SPI_TIMODE_DISABLE;
    spi1_handle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    spi1_handle.Init.Mode = SPI_MODE_MASTER;

    HAL_SPI_Init(&spi1_handle);
  }
}

void SPI1_Uninit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  HAL_SPI_DeInit(&spi1_handle);

  SPI1_CLK_DISABLE();

  GPIO_InitStruct.Pin = SPI1_SCK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;//GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;//GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    GPIO_InitStruct.Alternate = SPI1_SCK_AF;
  HAL_GPIO_Init(SPI1_SCK_GPIO_PORT, &GPIO_InitStruct);

  /* Configure SPI MISO and MOSI */
  GPIO_InitStruct.Pin = SPI1_MOSI_PIN;
//    GPIO_InitStruct.Alternate = SPI1_MISO_MOSI_AF;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;//GPIO_PULLDOWN;
  HAL_GPIO_Init(SPI1_MISO_MOSI_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SPI1_MISO_PIN;
  HAL_GPIO_Init(SPI1_MISO_MOSI_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  SPI error treatment function
  * @retval None
  */
void SPI1_Error (void)
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&spi1_handle);

  /* Re-Initiaize the SPI communication BUS */
  SPI1_Init();
}

/**
  * @brief  SPI Write an amount of data to device
  * @param  Value: value to be written
  * @param  DataLength: number of bytes to write
  * @retval None
  */
HAL_StatusTypeDef SPI1_WriteData(uint8_t *DataIn, uint16_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Transmit(&spi1_handle, DataIn, DataLength, Spi1Timeout);
  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPI1_Error();
  }

  return status;
}

HAL_StatusTypeDef SPI1_ReadData(uint8_t *DataIn, uint16_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Receive(&spi1_handle, DataIn, DataLength, Spi1Timeout);
  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPI1_Error();
  }

  return status;
}


HAL_StatusTypeDef SPI1_ReadData_DMA(uint8_t *DataIn, uint16_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Receive(&spi1_handle, DataIn, DataLength, Spi1Timeout);
  //status = HAL_SPI_Receive_DMA(&spi1_handle, DataIn, DataLength);
  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPI1_Error();
  }

#if 0
  Timeout = 100;

  while (HAL_SPI_GetState(&spi1_handle) != HAL_SPI_STATE_READY)
  {
    //if((Timeout == 0) || ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick()-tickstart) >=  Timeout)))
    //{
    //  goto L_Error;
    //}
  }
#endif

  return status;
}

/**
  * @brief  SPI1_FlushFifo
  * @retval None
  */
void SPI1_FlushFifo(void)
{
  HAL_SPIEx_FlushRxFifo(&spi1_handle);
}
#endif /* HAL_SPI_MODULE_ENABLED */

