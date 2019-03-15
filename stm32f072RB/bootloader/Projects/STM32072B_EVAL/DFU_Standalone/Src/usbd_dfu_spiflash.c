/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : spi_if.c
* Author             : MCD Application Team
* Version            : V3.0.1
* Date               : 04/27/2009
* Description        : specific media access Layer for SPI flash
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usbd_dfu_flash.h"
#include "stm32f0xx_hal.h"
#include "main.h"

#if SPIFLASH_SUPPORT == 1

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
extern uint8_t SPI_Flash_init(void);
extern uint8_t SPI_Flash_erase_sector(uint32_t addr);
extern uint8_t SPI_Flash_write_Sector(uint32_t addr, uint8_t *buf);
extern uint8_t SPI_Flash_read_Sector(uint32_t addr, uint8_t *buf);
extern uint8_t SPI_Flash_Write_And_Check_Sector(uint32_t addr, uint8_t *buf, uint32_t size);
extern uint8_t SPI_Flash_read(uint32_t addr, uint8_t *buf, uint32_t nByte);

extern void Debug_Msg_Data(uint8_t *str, uint32_t dat);

/*******************************************************************************
* Function Name  : SPI_If_Init
* Description    : Initializes the Media on the STM32
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t SPI_If_Init(void)
{
	SPI_Flash_init();
	return 0;
}

/*******************************************************************************
* Function Name  : SPI_If_Erase
* Description    : Erase sector
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t SPI_If_Erase(uint32_t SectorAddress)
{
	uint8_t ret;
	uint32_t addr;
	addr = SectorAddress - SPIFLASH_ADDR;

	if ((addr & 0x00000fff) == 0)
	{
		ret = SPI_Flash_erase_sector(addr);
		return ret;
	}
	else
	{
		return 0;
	}

}

/*******************************************************************************
* Function Name  : SPI_If_Write
* Description    : Write sectors
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t SPI_If_Write(uint8_t *src, uint8_t *dest, uint32_t Len)
{
	uint32_t addr;
	uint8_t ret;
	addr = (uint32_t)dest - SPIFLASH_ADDR;
	ret = SPI_Flash_Write_And_Check_Sector((uint32_t)addr, src, Len);
	return ret;

}

/*******************************************************************************
* Function Name  : SPI_If_Read
* Description    : Read sectors
* Input          : None
* Output         : None
* Return         : buffer address pointer
*******************************************************************************/
uint8_t *SPI_If_Read(uint8_t *src, uint8_t *dest, uint32_t Len)
{
	uint32_t addr;
	addr = (uint32_t)src - SPIFLASH_ADDR;

	SPI_Flash_read((uint32_t)addr, dest, Len);

	return (uint8_t*)(dest);
}
#endif
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
