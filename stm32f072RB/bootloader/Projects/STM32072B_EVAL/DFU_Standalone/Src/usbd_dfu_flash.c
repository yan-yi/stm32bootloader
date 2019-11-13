/**
  ******************************************************************************
  * @file    USB_Device/DFU_Standalone/Src/usbd_dfu_flash.c
  * @author  MCD Application Team
  * @brief   Memory management layer
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
#include "usbd_dfu_flash.h"
#include "stm32f0xx_hal.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/

#if SPIFLASH_SUPPORT == 1
extern uint16_t SPI_If_Init(void);
extern uint16_t SPI_If_Erase(uint32_t SectorAddress);
extern uint16_t SPI_If_Write(uint8_t *src, uint8_t *dest, uint32_t Len);
extern uint8_t *SPI_If_Read(uint8_t *src, uint8_t *dest, uint32_t Len);
#endif

extern void Reset_Usbd_End_Time(void);

uint16_t Flash_If_Init(void);
uint16_t Flash_If_Erase(uint32_t Add);
uint16_t Flash_If_Write(uint8_t *src, uint8_t *dest, uint32_t Len);
uint8_t *Flash_If_Read(uint8_t *src, uint8_t *dest, uint32_t Len);
uint16_t Flash_If_DeInit(void);
uint16_t Flash_If_GetStatus(uint32_t Add, uint8_t Cmd, uint8_t *buffer);

USBD_DFU_MediaTypeDef USBD_DFU_Flash_fops = {
  (uint8_t *)FLASH_DESC_STR,
  Flash_If_Init,
  Flash_If_DeInit,
  Flash_If_Erase,
  Flash_If_Write,
  Flash_If_Read,
  Flash_If_GetStatus,
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes Memory.
  * @param  None
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
uint16_t Flash_If_Init(void)
{
  /* Unlock the internal flash */
  HAL_FLASH_Unlock();
  return 0;
}

/**
  * @brief  De-Initializes Memory.
  * @param  None
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
uint16_t Flash_If_DeInit(void)
{
  /* Lock the internal flash */
  HAL_FLASH_Lock();
  return 0;
}

/**
  * @brief  Erases sector.
  * @param  Add: Address of sector to be erased.
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
uint16_t Flash_If_Erase(uint32_t Add)
{
  uint32_t NbOfPages = 0;
  uint32_t PageError = 0;
  /* Variable contains Flash operation status */
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef eraseinitstruct;

  // printf("erase addr 0x%x \r\n", (uint32_t)Add);
#if SPIFLASH_SUPPORT == 1
  if (Add >= SPIFLASH_ADDR)
  {
    if (SPI_If_Erase(Add) != 0)
    {
      Reset_Usbd_End_Time();
      // printf("erase spi flash error \r\n");
      return 1;
    }
    else
    {
      Reset_Usbd_End_Time();
      return 0;
    }
  }
  else
#endif

  {

    /* Get the number of sector to erase from 1st sector*/
    NbOfPages = 1;
    eraseinitstruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseinitstruct.PageAddress = Add;
    eraseinitstruct.NbPages = NbOfPages;
    status = HAL_FLASHEx_Erase(&eraseinitstruct, &PageError);

    if (status != HAL_OK)
    {
      Reset_Usbd_End_Time();
      // printf("erase flash error \r\n");
      return 1;
    }
    else
    {
      Reset_Usbd_End_Time();
      return 0;
    }
  }
}

/**
  * @brief  Writes Data into Memory.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be written (in bytes).
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
uint16_t Flash_If_Write(uint8_t *src, uint8_t *dest, uint32_t Len)
{
  uint32_t i = 0;

  // printf("write addr 0x%x len 0x%x \r\n", (uint32_t)dest, Len);
#if SPIFLASH_SUPPORT == 1
  if ((uint32_t)dest >= SPIFLASH_ADDR)
  {
    if (SPI_If_Write(src, dest, Len) != 0)
    {
      Reset_Usbd_End_Time();

      // printf("write spi flash error \r\n");
      return 1;
    }
    else
    {
      Reset_Usbd_End_Time();
      return 0;
    }
  }
  else
#endif
  {
    for (i = 0; i < Len; i += 4)
    {
      /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
         be done by byte */
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(dest + i), *(uint32_t*)(src + i)) == HAL_OK)
      {
        /* Check the written value */
        if (*(uint32_t *)(src + i) != *(uint32_t*)(dest + i))
        {
          /* Flash content doesn't match SRAM content */
          Reset_Usbd_End_Time();

          // printf("write flash vierty error \r\n");
          return 2;
        }
      }
      else
      {
        /* Error occurred while writing data in Flash memory */
        Reset_Usbd_End_Time();

        // printf("write spi flash error \r\n");
        return 1;
      }
    }
    Reset_Usbd_End_Time();
    return 0;
  }
}

/**
  * @brief  Reads Data into Memory.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be read (in bytes).
  * @retval Pointer to the physical address where data should be read.
  */
uint8_t *Flash_If_Read(uint8_t *src, uint8_t *dest, uint32_t Len)
{
  uint32_t i = 0;
  uint8_t *psrc = src;

  // printf("read addr 0x%x len 0x%x \r\n", (uint32_t)src, Len);
#if SPIFLASH_SUPPORT == 1
  if ((uint32_t)src >= SPIFLASH_ADDR)
  {
    SPI_If_Read(src, dest, Len);
  }
  else
#endif
  {
    for (i = 0; i < Len; i++)
    {
      dest[i] = *psrc++;
    }
  }
  Reset_Usbd_End_Time();
  /* Return a valid address to avoid HardFault */
  return (uint8_t*)(dest);
}

/**
  * @brief  Gets Memory Status.
  * @param  Add: Address to be read from.
  * @param  Cmd: Number of data to be read (in bytes).
  * @retval 0 if operation is successeful
  */
uint16_t Flash_If_GetStatus(uint32_t Add, uint8_t Cmd, uint8_t *buffer)
{
  switch (Cmd)
  {
  case DFU_MEDIA_PROGRAM:
    buffer[1] = (uint8_t)FLASH_PROGRAM_TIME;
    buffer[2] = (uint8_t)(FLASH_PROGRAM_TIME << 8);
    buffer[3] = 0;

    Reset_Usbd_End_Time();
    break;

  case DFU_MEDIA_ERASE:
  default:
    buffer[1] = (uint8_t)FLASH_ERASE_TIME;
    buffer[2] = (uint8_t)(FLASH_ERASE_TIME << 8);
    buffer[3] = 0;

    Reset_Usbd_End_Time();
    break;
  }
  return 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
