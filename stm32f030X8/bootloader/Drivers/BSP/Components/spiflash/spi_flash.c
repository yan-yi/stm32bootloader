/*
* @Author: Green
* @Date:   2016-06-01 09:23:13
* @Last Modified by:   Green
* @Last Modified time: 2018-01-04 17:44:56
*/
////////////////////////////////////////
//Include files
#include "stm32f0xx_hal.h"
#include "main.h"
#include "spi1.h"

#define STATUS_OK             0

#define C_SPI_TIME_OUT        3000      // time out count

#define SPI_FLASH_BUSY        1
#define SPI_FLASH_TIMEOUT     2

#define SPI_FLASH_SR_WIP      0x01

//Flash
#define SPI_FLASH_PAGE_SIZE       256
#define SPI_FLASH_SECTOR_SIZE     4096
#define SPI_FLASH_SECTOR_COUNT    1024
#define SPI_FLASH_BLOCK_SIZE      65536
#define SPI_FLASH_PAGES_PER_SECTOR  (SPI_FLASH_SECTOR_SIZE/SPI_FLASH_PAGE_SIZE)

#define SPIFLASH_CS_PIN                                 GPIO_PIN_15
#define SPIFLASH_CS_GPIO_PORT                           GPIOA
#define SPIFLASH_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIFLASH_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOA_CLK_DISABLE()

#define SPIFLASH_CS_LOW()      HAL_GPIO_WritePin(SPIFLASH_CS_GPIO_PORT, SPIFLASH_CS_PIN, GPIO_PIN_RESET)
#define SPIFLASH_CS_HIGH()     HAL_GPIO_WritePin(SPIFLASH_CS_GPIO_PORT, SPIFLASH_CS_PIN, GPIO_PIN_SET)


// Functions defined in this file
static void SPI_CS_HIGH(void);
static void SPI_CS_LOW(void);
static uint8_t SPI_Flash_readID(uint8_t* Id);
static uint8_t SPI_Flash_WriteEn(void);
//static uint8_t SPI_Flash_WriteDisable(void);
//static uint8_t SPI_Flash_WriteStatus(uint8_t status);
static uint8_t CheckStatus(void);


void SPIFLASH_IO_Init(void)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};

  /* SPIFLASH_CS_GPIO Periph clock enable */
  SPIFLASH_CS_GPIO_CLK_ENABLE();

  /* Configure SPIFLASH_CS_PIN pin: SPIFLASH CS pin */
  gpioinitstruct.Pin = SPIFLASH_CS_PIN;
  gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull = GPIO_NOPULL;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPIFLASH_CS_GPIO_PORT, &gpioinitstruct);

  SPIFLASH_CS_HIGH();

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


static void SPI_CS_HIGH(void)
{
  SPIFLASH_CS_HIGH();
}

static void SPI_CS_LOW(void)
{
  SPIFLASH_CS_LOW();
}

uint8_t SPI_Flash_init(void)
{
  uint8_t buf[3];

  SPIFLASH_IO_Init();

  SPI_Flash_readID(buf);

//  printf("SPI FLASH 0x%x 0x%x 0x%x\r\n",buf[0],buf[1],buf[2]);

  if (((buf[0] == 0) && (buf[1] == 0) && (buf[2] == 0)) || ((buf[0] == 0xff) && (buf[1] == 0xff) && (buf[2] == 0xff)))
  {
    return 1;
  }
  else
  {
    return STATUS_OK;
  }
}

void SPI_LOCK(void)
{

}

void SPI_UNLOCK(void)
{

}

// byte 1 is manufacturer ID(0xc2), byte 2 is memory type ID ,byte 3 is device ID
uint8_t SPI_Flash_readID(uint8_t* Id)
{
  uint8_t ret = STATUS_OK;
  uint8_t cmd[1];

  cmd[0] = 0x9f;

  //send command
  SPI_LOCK();

  SPI_CS_LOW();
  ret = SPIFLASH_IO_WriteData(cmd, 1);
  if (ret != STATUS_OK) {
    SPI_CS_HIGH();
    SPI_UNLOCK();
    return ret;
  }

  // read spi flash id
  ret = SPIFLASH_IO_ReadData(Id, 3);
  SPI_CS_HIGH();

  SPI_UNLOCK();

  return ret;
}

// input the start address of sector
// a sector size is 4k Byte(0x1000)
uint8_t SPI_Flash_erase_sector(uint32_t addr)
{
  uint8_t ret = STATUS_OK;
  uint8_t cmd[4];

  // addr *= SPI_FLASH_SECTOR_SIZE;

  SPI_LOCK();

  if (CheckStatus() != STATUS_OK) {
    SPI_UNLOCK();
    return SPI_FLASH_TIMEOUT;
  }

  SPI_Flash_WriteEn();

  cmd[0] = 0x20;
  cmd[1] = ((addr >> 16) & 0xFF);
  cmd[2] = ((addr >> 8) & 0xFF);
  cmd[3] = (addr & 0xFF);

  //send command
  SPI_CS_LOW();
  ret = SPIFLASH_IO_WriteData(cmd, 4);
  SPI_CS_HIGH();

  if (CheckStatus() != STATUS_OK) {
    SPI_UNLOCK();
    return SPI_FLASH_TIMEOUT;
  }
  SPI_UNLOCK();

  return ret;
}

#if 0
// input the start address of block
// a block size is 64k Byte(0x10000)
uint8_t SPI_Flash_erase_block(uint32_t addr)
{
  uint8_t ret = STATUS_OK;
  uint8_t cmd[4];

  addr *= SPI_FLASH_BLOCK_SIZE;

  SPI_LOCK();

  if (CheckStatus() != STATUS_OK) {
    SPI_UNLOCK();
    return SPI_FLASH_TIMEOUT;
  }

  SPI_Flash_WriteEn();

  cmd[0] = 0xd8;
  cmd[1] = ((addr >> 16) & 0xFF);
  cmd[2] = ((addr >> 8) & 0xFF);
  cmd[3] = (addr & 0xFF);

  //send command
  SPI_CS_LOW();
  ret = SPIFLASH_IO_WriteData(cmd, 4);
  SPI_CS_HIGH();

  if (CheckStatus() != STATUS_OK) {
    SPI_UNLOCK();
    return SPI_FLASH_TIMEOUT;
  }

  SPI_UNLOCK();

  return ret;
}

uint8_t SPI_Flash_erase_chip(void)
{
  uint8_t ret = STATUS_OK;
  uint8_t cmd[1];

  SPI_LOCK();

  if (CheckStatus() != STATUS_OK) {
    SPI_UNLOCK();
    return SPI_FLASH_TIMEOUT;
  }

  SPI_Flash_WriteEn();

  cmd[0] = 0x60;

  //send command
  SPI_CS_LOW();
  ret = SPIFLASH_IO_WriteData(cmd, 1);
  SPI_CS_HIGH();

  if (CheckStatus() != STATUS_OK) {
    SPI_UNLOCK();
    return SPI_FLASH_TIMEOUT;
  }

  SPI_UNLOCK();

  return ret;
}
#endif

// read a page data(256 Byte)
// the addr must be 256 byte allign(low 8 bit addr must be 0)
uint8_t SPI_Flash_read_page(uint32_t addr, uint8_t *buf)
{
  uint8_t ret = STATUS_OK;
  uint8_t cmd[4];

  SPI_LOCK();

  if (CheckStatus() != STATUS_OK) {
    SPI_UNLOCK();
    return SPI_FLASH_TIMEOUT;
  }

  cmd[0] = 0x03;
  cmd[1] = ((addr >> 16) & 0xFF);
  cmd[2] = ((addr >> 8) & 0xFF);
  cmd[3] = (addr & 0xFF);

  //send command
  SPI_CS_LOW();
  ret = SPIFLASH_IO_WriteData(cmd, 4);
  if (ret != STATUS_OK) {
    SPI_CS_HIGH();
    SPI_UNLOCK();
    return ret;
  }

  // read page
  ret = SPIFLASH_IO_ReadData(buf, 256);
  SPI_CS_HIGH();

  SPI_UNLOCK();

  return ret;
}


// write a page data(256 Byte)
// the addr must be 256 byte allign(low 8 bit addr must be 0)
uint8_t SPI_Flash_write_page(uint32_t addr, uint8_t *buf)
{
  uint8_t ret = STATUS_OK;
  uint8_t cmd[6];

  SPI_LOCK();

  if (CheckStatus() != STATUS_OK) {
    SPI_UNLOCK();
    return SPI_FLASH_TIMEOUT;
  }

  SPI_Flash_WriteEn();

  cmd[0] = 0x02;
  cmd[1] = ((addr >> 16) & 0xFF);
  cmd[2] = ((addr >> 8) & 0xFF);
  cmd[3] = (addr & 0xFF);

  //send command
  SPI_CS_LOW();
  ret = SPIFLASH_IO_WriteData(cmd, 4);
  if (ret != STATUS_OK) {
    SPI_CS_HIGH();
    SPI_UNLOCK();
    return ret;
  }

  // write data
  ret = SPIFLASH_IO_WriteData(buf, 256);
  SPI_CS_HIGH();
  if (CheckStatus() != STATUS_OK) {
    SPI_UNLOCK();
    return SPI_FLASH_TIMEOUT;
  }


  SPI_UNLOCK();

  return ret;
}

// read start from "addr", read "nByte" byte to "buf"
// the address can be any byte address
uint8_t SPI_Flash_read(uint32_t addr, uint8_t *buf, uint32_t nByte)
{
  uint8_t ret = STATUS_OK;
  uint8_t cmd[4];

  SPI_LOCK();

  if (CheckStatus() != STATUS_OK) {
    SPI_UNLOCK();
    return SPI_FLASH_TIMEOUT;
  }

  cmd[0] = 0x03;
  cmd[1] = ((addr >> 16) & 0xFF);
  cmd[2] = ((addr >> 8) & 0xFF);
  cmd[3] = (addr & 0xFF);

  //send command
  SPI_CS_LOW();
  ret = SPIFLASH_IO_WriteData(cmd, 4);
  if (ret != STATUS_OK) {
    SPI_CS_HIGH();
    SPI_UNLOCK();
    return ret;
  }

  // read page
  ret = SPIFLASH_IO_ReadData(buf, nByte);
  SPI_CS_HIGH();

  SPI_UNLOCK();

  return ret;
}

uint8_t SPI_Flash_write_Sector(uint32_t addr, uint8_t *buf)
{
  uint16_t j;

  for (j = 0; j < SPI_FLASH_PAGES_PER_SECTOR; j++)
  {
    SPI_Flash_write_page(addr, buf);
    buf += SPI_FLASH_PAGE_SIZE;
    addr += SPI_FLASH_PAGE_SIZE;
  }
  return 0;
}

uint8_t SPI_Flash_read_Sector(uint32_t addr, uint8_t *buf)
{
  uint16_t j;

  for (j = 0; j < SPI_FLASH_PAGES_PER_SECTOR; j++)
  {
    SPI_Flash_read_page(addr, buf);
    buf += SPI_FLASH_PAGE_SIZE;
    addr += SPI_FLASH_PAGE_SIZE;
  }
  return 0;
}

static uint8_t SPI_Flash_WriteEn(void)
{
  uint8_t ret = STATUS_OK;
  uint8_t cmd[1];

  cmd[0] = 0x06;

  //send command
  SPI_CS_LOW();
  ret = SPIFLASH_IO_WriteData(cmd, 1);
  SPI_CS_HIGH();

  return ret;
}

#if 0
static uint8_t SPI_Flash_WriteDisable(void)
{

  uint8_t ret = STATUS_OK;
  uint8_t cmd[1];

  cmd[0] = 0x06;

  //send command
  cmd[0] = 0x04;
  SPI_CS_LOW();
  ret = SPIFLASH_IO_WriteData(cmd, 1);
  SPI_CS_HIGH();
  return ret;

}
#endif

static uint8_t SPI_Flash_ReadStatus(void)
{
  uint8_t ret = STATUS_OK;
  uint8_t cmd[1];

  cmd[0] = 0x05;

  //send command
  SPI_CS_LOW();
  ret = SPIFLASH_IO_WriteData(cmd, 1);
  if (ret != STATUS_OK) {
    SPI_CS_HIGH();
    return ret;
  }

  // read status
  ret = SPIFLASH_IO_ReadData(cmd, 1);
  if (ret != STATUS_OK) {
    SPI_CS_HIGH();
    return ret;
  }
  SPI_CS_HIGH();

  return (uint8_t)cmd[0];
}

#if 0
static uint8_t SPI_Flash_WriteStatus(uint8_t status)
{

  uint8_t ret = STATUS_OK;
  uint8_t cmd[2];

  SPI_Flash_WriteEn();

  cmd[0] = 0x01;
  cmd[1] = status;

  SPI_CS_LOW();
  ret = SPIFLASH_IO_WriteData(cmd, 2);
  if (ret != STATUS_OK) {
    SPI_CS_HIGH();
    return ret;
  }
  SPI_CS_HIGH();
  return ret;

}
#endif

static uint8_t CheckStatus(void)
{
  uint16_t  i = 0;

  while ((SPI_Flash_ReadStatus() & SPI_FLASH_SR_WIP) != 0) {

    HAL_Delay(1);

    if (++i >= C_SPI_TIME_OUT) {
      return SPI_FLASH_TIMEOUT;
    }
  }
  return STATUS_OK;
}



int buf_cmp(uint8_t * buf1, uint8_t * buf2, uint32_t cnt)
{
  uint32_t i;
//  uint8_t* buf1;
//  uint8_t* buf2;

  uint8_t data1;
  uint8_t data2;

//  buf1 = pWr;
//  buf2 = pRd;

  for (i = 0; i < cnt; i++ )
  {
    data1 = *buf1;
    data2 = *buf2;
    if (data1 != data2)
    {
      //if (*buf1 != 0)
      {
        return 1;
      }
    }

    buf1++;
    buf2++;
  }

  return 0;
}

uint8_t SPI_Flash_Write_And_Check_Sector(uint32_t addr, uint8_t *buf, uint32_t size)
{
  uint8_t *pWrBuf;
  uint8_t *pRdBuf;
  uint32_t ret;
  uint32_t spi_addr;
//  uint8_t retry;
  uint8_t error;
  uint8_t RdBuf[SPI_FLASH_PAGE_SIZE + 4] = {0};

  error = 0;

  pWrBuf = buf;
  pRdBuf = RdBuf;

  for (spi_addr = addr; spi_addr < (addr + size); spi_addr += 0x100)
  {

//        ret = SPI_Flash_read_page(spi_addr, pRdBuf);

    ret = SPI_Flash_write_page(spi_addr, pWrBuf);
    if (ret != 0)
    {
      error = 1;
    }

    HAL_Delay(2);

    ret = SPI_Flash_read_page(spi_addr, pRdBuf);

    ret = buf_cmp((uint8_t *)pWrBuf, (uint8_t *)pRdBuf, SPI_FLASH_PAGE_SIZE);
    if (ret == 0)
    {

    }
    else
    {
      error = 1;
    }

    if (error == 1)
    {
      break;
    }

    pWrBuf += 0x100;

  }


  if (error == 1)
  {
    goto L_END;
  }

//L_OK:

L_END:

  return error;
}

