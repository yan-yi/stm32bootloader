/**
  ******************************************************************************
  * @file    IAP_Main/Src/main.c
  * @author  MCD Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "menu.h"

/** @addtogroup STM32F0xx_IAP_Main
  * @{
  */
#define C_BUF_SIZE  0x100

#define C_SPIFLASH_SN_DATA_ADDR       0ul
#define C_SPIFLASH_SN_DATA_SIZE       0x1000ul  // 4KB
#define C_SPIFLASH_FIRMWARE_ADDR      (C_SPIFLASH_SN_DATA_ADDR+C_SPIFLASH_SN_DATA_SIZE) // 0x1000
#define C_SPIFLASH_FIRMWARE_SIZE      0x100000ul // 1024KB

#define C_SPIFLASH_SETTING_DATA_ADDR  (0x200000ul - 0x8000ul) // 0x1F8000
#define C_SPIFLASH_SETTING_DATA_SIZE  0x8000ul  // 32KB

/* Base address of the Flash sectors */
#define FLASH_USER_START_ADDR   ((uint32_t)0x08004000)   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ((uint32_t)0x08010000)   /* End @ of user Flash area */

/* Exported variables --------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* CRC handler declaration */
CRC_HandleTypeDef CrcHandle;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define kUpKeyPort              GPIOA     //Volume (+) Key   0:OFF  1:ON
#define kUpKeyPin               GPIO_PIN_11
#define kDownKeyPort            GPIOA     //Volume (-) Key   0:OFF  1:ON
#define kDownKeyPin             GPIO_PIN_12

#define kPowerKeyPort           GPIOC
#define kPowerKeyPin            GPIO_PIN_13
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern pFunction JumpToApplication;
extern uint32_t JumpAddress;

/* Private function prototypes -----------------------------------------------*/
static void IAP_Init(void);
void SystemClock_Config(void);
void BSP_KEY_Init(void);
uint8_t Is_Update_From_SPIFLASH(void);
void Update_From_SPIFLASH(void);   


extern void SPI_Flash_init(void);
extern uint8_t SPI_Flash_read(uint32_t addr, uint8_t *buf, uint32_t nByte);
extern uint8_t SPI_Flash_read_page(uint32_t addr, uint8_t *buf);
extern uint8_t SPI_Flash_erase_sector(uint32_t addr);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    /* Enable the SYSCFG peripheral clock*/
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    __HAL_SYSCFG_REMAPMEMORY_FLASH();

  BSP_KEY_Init();

  if (HAL_GPIO_ReadPin(kPowerKeyPort, kPowerKeyPin) == GPIO_PIN_SET)
  {
    goto L_APPLICTION;
  }
  else
  {
    HAL_Init();

    /* Configure the system clock to 48 MHz */
    SystemClock_Config();

    SPI_Flash_init();

    if(Is_Update_From_SPIFLASH()==0)       
    {
    Update_From_SPIFLASH();   
    }    

    /* Test if Tamper push-button on STM32091C-EVAL Board is pressed */
    if ((HAL_GPIO_ReadPin(kPowerKeyPort, kPowerKeyPin) == GPIO_PIN_RESET) \
        && (HAL_GPIO_ReadPin(kUpKeyPort, kUpKeyPin) == GPIO_PIN_RESET)  \
        && (HAL_GPIO_ReadPin(kDownKeyPort, kDownKeyPin) == GPIO_PIN_RESET))
    {
      /* Initialise Flash */
      FLASH_If_Init();
      /* Execute the IAP driver in order to reprogram the Flash */
      IAP_Init();
      /* Display main menu */
      Main_Menu ();
    }
    /* Keep the user application running */
    else
    {

L_APPLICTION:
      /* Test if user code is programmed starting from address "APPLICATION_ADDRESS" */
      if (((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
      {
        /* Jump to user application */
        JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
        JumpToApplication = (pFunction) JumpAddress;
        /* Initialize user application's Stack Pointer */
        __set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
        JumpToApplication();
      }
    }

  }

  HAL_NVIC_SystemReset();

}

void BSP_KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable the Tamper Clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = kPowerKeyPin;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  /* Configure Button pin as input */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

  HAL_GPIO_Init(kPowerKeyPort, &GPIO_InitStruct);


  /* Enable the Tamper Clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = kUpKeyPin;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  /* Configure Button pin as input */
  // GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

  HAL_GPIO_Init(kUpKeyPort, &GPIO_InitStruct);


  /* Enable the Tamper Clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = kDownKeyPin;
  // GPIO_InitStruct.Pull = GPIO_PULLUP;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // /* Configure Button pin as input */
  // GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(kDownKeyPort, &GPIO_InitStruct);

}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PREDIV                         = 1
  *            PLLMUL                         = 6
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* No HSE Oscillator on Nucleo, Activate PLL with HSI/2 as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    HAL_NVIC_SystemReset();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    /* Initialization Error */
    HAL_NVIC_SystemReset();
  }

  /*Configure the SysTick to have interrupt in 10ms time basis*/
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 100U);

  /*Configure the Systick */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}

int fputc(int ch, FILE *f )
{
  HAL_UART_Transmit(&UartHandle, (uint8_t*)&ch, 1, 1000);
  return ch;
}

/**
  * @brief  Initialize the IAP: Configure USART.
  * @param  None
  * @retval None
  */
void IAP_Init(void)
{
  /* USART resources configuration (Clock, GPIO pins and USART registers) ----*/
  /* USART configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  UartHandle.Instance      = USART1;
  UartHandle.Init.BaudRate = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode = UART_MODE_RX | UART_MODE_TX;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
//      Error_Handler();
  }

  HAL_UART_Init(&UartHandle);

//  __HAL_RCC_CRC_CLK_ENABLE();

//  /*##-2- Configure the CRC peripheral #####################################*/
//  CrcHandle.Instance = CRC;

//  /* The CRC-16-CCIT polynomial is used */
//  CrcHandle.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;
////  CrcHandle.Init.GeneratingPolynomial    = 0x1021;
////  CrcHandle.Init.CRCLength               = CRC_POLYLENGTH_16B;

//  /* The zero init value is used */
//  CrcHandle.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;
////  CrcHandle.Init.InitValue               = 0;

//  /* The input data are not inverted */
//  CrcHandle.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_NONE;

//  /* The output data are not inverted */
//  CrcHandle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;

//  /* The input data are 32-bit long words */
//  CrcHandle.InputDataFormat              = CRC_INPUTDATA_FORMAT_BYTES;

//  if (HAL_CRC_Init(&CrcHandle) != HAL_OK)
//  {
//    /* Initialization Error */
//    while (1)
//    {}
//  }
}


uint32_t FLASH_Write(__IO uint32_t* FlashAddress, __IO uint16_t* Data, uint16_t DataLength)
{
  uint32_t i = 0;

  for (i = 0; (i < DataLength) && (*FlashAddress <= (FLASH_USER_END_ADDR - 2)); i += 2)
  {
    /* the operation will be done by word */
    //if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, *FlashAddress, *(uint32_t*)(Data)) == HAL_OK)
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, *FlashAddress, *(uint16_t*)(Data)) == HAL_OK)
    {
      /* Check the written value */
      if (*(uint16_t*)*FlashAddress != *(uint16_t*)(Data))
      {
        /* Flash content doesn't match SRAM content */
        return (2);
      }
      /* Increment FLASH destination address */
      *FlashAddress += 2;
      Data += 1;
    }
    else
    {
      /* Error occurred while writing data in Flash memory */
      return (1);
    }
  }

  return (0);
}

void Update_Flash(uint32_t addr, uint32_t size)
{
  uint8_t Buf[C_BUF_SIZE];
  uint8_t *pRdBuf = Buf;
  uint8_t error_cnt = 0;
  uint32_t cnt = 0;
  uint32_t spi_addr = addr;

  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t Address = 0, PageError = 0;
  __IO uint32_t data32 = 0, MemoryProgramStatus = 0;

//  printf("update from spi flash 0x%x  0x%x \r\n", addr, size);

L_Loop:

  if (error_cnt > 3)
  {

//    printf("update from spi flash fail\r\n");

    SPI_Flash_erase_sector(C_SPIFLASH_FIRMWARE_ADDR);

    HAL_NVIC_SystemReset();
  }
  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Erase the user Flash area
     (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
  EraseInitStruct.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    /*
       Error occurred while page erase.
       User can add here some code to deal with this error.
       PageError will contain the faulty page and then to know the code error on this page,
       user can call function 'HAL_FLASH_GetError()'
     */

    error_cnt++;
    goto L_Loop;

  }

  /* Program the user Flash area word by word
     (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  Address = FLASH_USER_START_ADDR;
  while ((Address < FLASH_USER_END_ADDR) && (cnt < size))
  {
//    printf("spi flash addr: 0x%x\r\n", spi_addr);

    SPI_Flash_read_page(spi_addr, pRdBuf);

    if (FLASH_Write(&Address, (uint16_t*)pRdBuf, C_BUF_SIZE) == 0)
    {
      spi_addr = spi_addr + C_BUF_SIZE;
      cnt = cnt + C_BUF_SIZE;
    }
    else
    {
      error_cnt++;
      goto L_Loop;
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

//  printf("update from spi flash success\r\n");

  SPI_Flash_erase_sector(C_SPIFLASH_FIRMWARE_ADDR);

  HAL_NVIC_SystemReset();
}

uint8_t Is_Update_From_SPIFLASH(void)
{
    uint32_t addr;
  SPI_Flash_read(C_SPIFLASH_FIRMWARE_ADDR + 0x0000, (uint8_t *)&addr, 4);
  if (addr == 0x41544F)// OTA
  {
      return 0;
  }      
  
  return 1;
}    

void Update_From_SPIFLASH(void)           
{
  uint32_t addr, len;

  SPI_Flash_read(C_SPIFLASH_FIRMWARE_ADDR + 0x0000, (uint8_t *)&addr, 4);
  if (addr == 0x41544F)// OTA
  {
    SPI_Flash_read(C_SPIFLASH_FIRMWARE_ADDR + 0x0010, (uint8_t *)&addr, 4);
    SPI_Flash_read(C_SPIFLASH_FIRMWARE_ADDR + 0x0014, (uint8_t *)&len, 4);
    addr = addr + C_SPIFLASH_FIRMWARE_ADDR;
    if (len != 0)
    {
      IAP_Init();

      Update_Flash(addr, len);
    }
  }
}


#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  printf("Wrong: file %s on line %d\r\n", file, line);

  /* Infinite loop */
//  HAL_NVIC_SystemReset();
}
#endif

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
