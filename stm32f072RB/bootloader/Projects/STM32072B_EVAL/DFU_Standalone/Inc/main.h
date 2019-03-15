/**
  ******************************************************************************
  * @file    USB_Device/DFU_Standalone/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_dfu.h"
#include "usbd_dfu_flash.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Uncomment the line below to select your USB clock source */
#define USE_USB_CLKSOURCE_CRSHSI48   1
//#define USE_USB_CLKSOURCE_PLL        1

#if !defined (USE_USB_CLKSOURCE_PLL) && !defined (USE_USB_CLKSOURCE_CRSHSI48)
#error "Missing USB clock definition"
#endif

#define DEBUG               0
#define SPIFLASH_SUPPORT    1


#define FLASH_ERASE_TIME    (uint16_t)50
#define FLASH_PROGRAM_TIME  (uint16_t)50

/*128 pages of 2 Kbytes*/
#define FLASH_DESC_STR      "@Internal Flash   /0x08000000/10*02Ka,2*02Kg,52*02Kg,1024*04Kg"
// BootLoader   10*02Ka 10*2*1024   = 0x5000    0x08000000
// Data         2*02Kg  2*2*1024    = 0x1000    0x08005000
// APP          52*02Kg 52*2*1024   = 0x1A000   0x08006000
// SPIFLASH     1024*04Kg 1024*4*1024 = 0x200000

#define BOOTLOADER_SIZE   (10*2*1024) //0x5000  
#define CONFIG_PARAM_SIZE (2*2*1024)  //0x1000
#define FIRMWARE_SIZE     (52*2*1024) //0x1A000 

#define BOOTLOADER_ADDR     (0x08000000)
#define CONFIG_PARAM_ADDR   (BOOTLOADER_ADDR + BOOTLOADER_SIZE) //0x08005000
#define FIRMWARE_START_ADDR (BOOTLOADER_ADDR + BOOTLOADER_SIZE + CONFIG_PARAM_SIZE) //0x08006000
#define SPIFLASH_ADDR       (BOOTLOADER_ADDR + BOOTLOADER_SIZE + CONFIG_PARAM_SIZE + FIRMWARE_SIZE) //0x08020000


#if SPIFLASH_SUPPORT == 1
#define C_BUF_SIZE  0x100

#define C_SPIFLASH_SN_DATA_ADDR       0ul
#define C_SPIFLASH_SN_DATA_SIZE       0x1000ul  // 4KB
#define C_SPIFLASH_FIRMWARE_ADDR      (C_SPIFLASH_SN_DATA_ADDR+C_SPIFLASH_SN_DATA_SIZE) // 0x1000
#define C_SPIFLASH_FIRMWARE_SIZE      0x100000ul // 1024KB
#define D_SPEECH_ADDR                 (C_SPIFLASH_SN_DATA_ADDR+C_SPIFLASH_SN_DATA_SIZE+C_SPIFLASH_FIRMWARE_SIZE) // 0x101000

#define C_SPIFLASH_SETTING_DATA_ADDR  (0x200000ul - 0x8000ul) // 0x1F8000
#define C_SPIFLASH_SETTING_DATA_SIZE  0x8000ul  // 32KB

#define ERR_RETRY_NUMBER 3

#define FLASH_USER_START_ADDR   ((uint32_t)FIRMWARE_START_ADDR)   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ((uint32_t)SPIFLASH_ADDR)   /* End @ of user Flash area */
#endif


#if defined(HAL_SPI_MODULE_ENABLED)
/*###################### SPI1 ###################################*/

/**
  * @brief  SPIFLASH Control Lines management
  */

/**
  * @brief  SPIFLASH Control Interface pins (shield D4)
  */

#define SPIFLASH_SPI1                                 SPI1
#define SPIFLASH_SPI1_CLK_ENABLE()                  __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIFLASH_SPI1_CLK_DISABLE()                 __HAL_RCC_SPI1_CLK_DISABLE()
#define SPIFLASH_SPI1_FORCE_RESET()                 __HAL_RCC_SPI1_FORCE_RESET()
#define SPIFLASH_SPI1_RELEASE_RESET()               __HAL_RCC_SPI1_RELEASE_RESET()


#define SPIFLASH_SPI1_SCK_AF                          GPIO_AF0_SPI1
#define SPIFLASH_SPI1_SCK_GPIO_PORT                   GPIOB
#define SPIFLASH_SPI1_SCK_PIN                         GPIO_PIN_3
#define SPIFLASH_SPI1_SCK_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIFLASH_SPI1_SCK_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE()

#define SPIFLASH_SPI1_MISO_MOSI_AF                    GPIO_AF0_SPI1
#define SPIFLASH_SPI1_MISO_MOSI_GPIO_PORT             GPIOB
#define SPIFLASH_SPI1_MISO_MOSI_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIFLASH_SPI1_MISO_MOSI_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()
#define SPIFLASH_SPI1_MISO_PIN                        GPIO_PIN_4
#define SPIFLASH_SPI1_MOSI_PIN                        GPIO_PIN_5
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define SPIFLASH_SPI1_TIMEOUT_MAX                   1000UL

#define SPIFLASH_CS_PIN                                 GPIO_PIN_15
#define SPIFLASH_CS_GPIO_PORT                           GPIOA
#define SPIFLASH_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIFLASH_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOA_CLK_DISABLE()

#define SPIFLASH_CS_LOW()      HAL_GPIO_WritePin(SPIFLASH_CS_GPIO_PORT, SPIFLASH_CS_PIN, GPIO_PIN_RESET)
#define SPIFLASH_CS_HIGH()     HAL_GPIO_WritePin(SPIFLASH_CS_GPIO_PORT, SPIFLASH_CS_PIN, GPIO_PIN_SET)

#endif /* HAL_SPI_MODULE_ENABLED */


void API_Timer_Reset(void);
uint32_t API_GetTimerCnt(void);
void SetTimer(uint32_t delay_time_msec);
uint32_t CheckTimer(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
