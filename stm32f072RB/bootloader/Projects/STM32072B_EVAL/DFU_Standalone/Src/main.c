/**
  ******************************************************************************
  * @file    USB_Device/DFU_Standalone/Src/main.c
  * @author  MCD Application Team
  * @brief   USB device DFU demo main file
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


#define POWER_CTR_GPIO_PIN                  GPIO_PIN_1
#define POWER_CTR_GPIO_PORT                 GPIOA
#define POWER_CTR_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()

#define kPowerKeyPort                       GPIOC
#define kPowerKeyPin                        GPIO_PIN_13
#define POWER_KEY_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()

#define kUpKeyPort                          GPIOC
#define kUpKeyPin                           GPIO_PIN_2
#define UP_KEY_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()

#define kDownKeyPort                        GPIOC    
#define kDownKeyPin                         GPIO_PIN_3
#define DOWN_KEY_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOC_CLK_ENABLE()

#define kExtPowerDetectPort                 GPIOA
#define kExtPowerDetectPin                  GPIO_PIN_0
#define EXT_POWER_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef USBD_Device;
uint8_t usbd_flag = 0;

typedef enum
{
    UpdateNull = 0,
    UpdateFormUSB = 1,
    UpdateFormSPIFLASH = 2 
} UpdateModeEnum;

typedef struct {
    UpdateModeEnum update_mode;
    uint8_t hardware_version;
    uint8_t software_version[4];
    uint8_t sum;
} configParam_t;

configParam_t configParam;

typedef enum
{
    kPowerSourceOff = 0,                            // Power Source OFF
    kPowerSourceBattery,                            // Power Source Battery
    kPowerSourceAdapter                             // Power Source DC Adapter
} PowerSourceEnum;

PowerSourceEnum g_power_source = kPowerSourceAdapter;
uint16_t g_power_source_cnt = 0;


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

void ConfigParamReadToFlash(void);
void ConfigParamReset(void);


#if SPIFLASH_SUPPORT == 1
void Update_From_SPIFLASH(void);

extern void SPI_Flash_init(void);
extern uint8_t SPI_Flash_erase_sector(uint32_t addr);
extern uint8_t SPI_Flash_write_page(uint32_t addr, uint8_t *buf);
extern uint8_t SPI_Flash_read_page(uint32_t addr, uint8_t *buf);
extern uint8_t SPI_Flash_read(uint32_t addr, uint8_t *buf, uint32_t nByte);
#endif


void IO_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    POWER_CTR_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = POWER_CTR_GPIO_PIN;
    HAL_GPIO_Init(POWER_CTR_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(POWER_CTR_GPIO_PORT, POWER_CTR_GPIO_PIN, GPIO_PIN_SET);

    POWER_KEY_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = kPowerKeyPin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(kPowerKeyPort, &GPIO_InitStruct);

    UP_KEY_GPIO_CLK_ENABLE();
   
    GPIO_InitStruct.Pin = kUpKeyPin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(kUpKeyPort, &GPIO_InitStruct);
    
    DOWN_KEY_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = kDownKeyPin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(kDownKeyPort, &GPIO_InitStruct);
    
    EXT_POWER_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = kExtPowerDetectPin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(kExtPowerDetectPort, &GPIO_InitStruct);
}


void Power_OFF(void)
{
    HAL_GPIO_WritePin(POWER_CTR_GPIO_PORT, POWER_CTR_GPIO_PIN, GPIO_PIN_RESET);

    while (1)
    {
        HAL_Delay(10);

        HAL_NVIC_SystemReset();
    }
}


void Reset_Usbd_End_Time(void)
{
    SetTimer(4000);
    usbd_flag = 2;
}


/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
    pFunction JumpToApplication;
    uint32_t JumpAddress;

    /* Enable the SYSCFG peripheral clock*/
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    __HAL_SYSCFG_REMAPMEMORY_FLASH();

    IO_Init();

    API_Timer_Reset();

    HAL_Init();

    /* Configure the system clock to get correspondent USB clock source */
    SystemClock_Config();

#if DEBUG == 1
    extern void DEBUG_Inital(void);
    DEBUG_Inital();
    printf("Rest !!!\r\n");
#endif

    ConfigParamReadToFlash();

    if (configParam.update_mode == UpdateFormUSB) // Update form USB, write it in APP.
    {
        ConfigParamReset();
        goto L_USB_UPDATE;
    }
    else if (configParam.update_mode == UpdateFormSPIFLASH) // Update form SPIFLASH, write it in APP.
    {
        ConfigParamReset();
#if SPIFLASH_SUPPORT == 1
        Update_From_SPIFLASH();
#endif
    }
        
    /* Check if the Tamper Button is pressed */
    if (HAL_GPIO_ReadPin(kExtPowerDetectPort, kExtPowerDetectPin) == SET)
    {
        if (HAL_GPIO_ReadPin(kUpKeyPort, kUpKeyPin) == RESET)
        {
            HAL_Delay(1);
            if (HAL_GPIO_ReadPin(kExtPowerDetectPort, kExtPowerDetectPin) == SET)
            {
                if (HAL_GPIO_ReadPin(kUpKeyPort, kUpKeyPin) == RESET)
                {

                    if (HAL_GPIO_ReadPin(kDownKeyPort, kDownKeyPin) == RESET)
                    {
                        extern void startDFUMain(void);
                        startDFUMain();
                    }

L_USB_UPDATE:

#if SPIFLASH_SUPPORT == 1
                    SPI_Flash_init();
#endif

                    /* Init Device Library */
                    USBD_Init(&USBD_Device, &DFU_Desc, 0);

                    /* Register the DFU class */
                    USBD_RegisterClass(&USBD_Device, &USBD_DFU);

                    /* Add DFU Media interface */
                    USBD_DFU_RegisterMedia(&USBD_Device, &USBD_DFU_Flash_fops);

                    /* Start Device Process */
                    USBD_Start(&USBD_Device);
                    /* In an infinite loop */

                    usbd_flag = 1;

                    while (1)
                    {
                        if (usbd_flag == 2)
                        {
                            if (CheckTimer() == 0)
                            {
                                usbd_flag = 0;

                                Power_OFF();
                            }
                        }
                        else
                        {
                            if (API_GetTimerCnt() != 0)
                            {
                                if (HAL_GPIO_ReadPin(kExtPowerDetectPort, kExtPowerDetectPin) == SET)
                                {
                                    if (g_power_source != kPowerSourceAdapter)
                                    {
                                        g_power_source_cnt++;
                                        if (g_power_source_cnt > 200)
                                        {
                                            g_power_source_cnt = 0;
                                            g_power_source = kPowerSourceAdapter;
                                        }
                                    }
                                    else
                                    {
                                        g_power_source_cnt = 0;
                                    }
                                }
                                else
                                {
                                    if (g_power_source != kPowerSourceBattery)
                                    {
                                        g_power_source_cnt++;
                                        if (g_power_source_cnt > 200)
                                        {
                                            g_power_source_cnt = 0;
                                            g_power_source = kPowerSourceBattery;

                                            Power_OFF();
                                        }
                                    }
                                    else
                                    {
                                        g_power_source_cnt = 0;
                                    }
                                }
                            }
                        }
                    }

                }
            }
        }
    }


    /* Test if user code is programmed starting from address 0x08007000 */
    if (((*(__IO uint32_t*)FIRMWARE_START_ADDR) & 0x2FFE0000 ) == 0x20000000)
    {
        /* Jump to user application */
        JumpAddress = *(__IO uint32_t*) (FIRMWARE_START_ADDR + 4);
        JumpToApplication = (pFunction) JumpAddress;

        /* Initialize user application's Stack Pointer */
        __set_MSP(*(__IO uint32_t*) FIRMWARE_START_ADDR);
        JumpToApplication();
    }
    else
    {
        goto L_USB_UPDATE;

    }
}

uint32_t FLASH_Write(__IO uint32_t* FlashAddress, __IO uint16_t* Data, uint16_t DataLength)
{
    uint32_t i = 0;

    for (i = 0; (i < DataLength) && (*FlashAddress <= (FIRMWARE_START_ADDR - 2)); i += 2)
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

L_Loop:

    if (error_cnt > 3)
    {
        Power_OFF();
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

    Power_OFF();
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
            Update_Flash(addr, len);

            Power_OFF();
        }

    }
}



void STM32_Flash_Write(uint32_t addr, uint8_t* buf, uint32_t size)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;
    uint8_t error_cnt = 0;

L_STM32_Flash_Write_Loop:

    if (error_cnt > 3)
    {
        return;
    }

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    /* Erase the user Flash area
       (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = addr;
    EraseInitStruct.NbPages = 1;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        error_cnt++;
        goto L_STM32_Flash_Write_Loop;

    }

    if (FLASH_Write(&addr, (uint16_t*)buf, size) != 0)
    {
        error_cnt++;
        goto L_STM32_Flash_Write_Loop;
    }

    /* Lock the Flash to disable the flash control register access (recommended
       to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();
}

void ConfigParamReset(void)
{
    configParam.update_mode = UpdateNull;
    STM32_Flash_Write(CONFIG_PARAM_ADDR, (uint8_t*)&configParam, sizeof(configParam));
}

void ConfigParamReadToFlash(void)
{
    memcpy((uint8_t*)&configParam, (uint8_t*)CONFIG_PARAM_ADDR, sizeof(configParam));
    // printf("update mode %d \r\nharaware version %d\r\nsoftware version %s\r\n", configParam.update_mode, configParam.hardware_version, configParam.software_version);
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow:
 *
 *            HSI48 used as USB clock source (USE_USB_CLKSOURCE_CRSHSI48 defined in main.h)
 *              - System Clock source            = HSI48
 *              - SYSCLK(Hz)                     = 48000000
 *              - HCLK(Hz)                       = 48000000
 *              - AHB Prescaler                  = 1
 *              - APB1 Prescaler                 = 1
 *              - Flash Latency(WS)              = 1
 *
 *              - PLL(HSE) used as USB clock source (USE_USB_CLKSOURCE_PLL defined in main.h)
 *              - System Clock source            = PLL (HSE)
 *              - SYSCLK(Hz)                     = 48000000
 *              - HCLK(Hz)                       = 48000000
 *              - AHB Prescaler                  = 1
 *              - APB1 Prescaler                 = 1
 *              - HSE Frequency(Hz)              = 8000000
 *              - PREDIV                         = 1
 *              - PLLMUL                         = 6
 *              - Flash Latency(WS)              = 1
 *
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

#if defined (USE_USB_CLKSOURCE_CRSHSI48)
    static RCC_CRSInitTypeDef RCC_CRSInitStruct;
#endif

#if defined (USE_USB_CLKSOURCE_CRSHSI48)

    /* No HSE Oscillator on Nucleo, Activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        /* Initialization Error */
        NVIC_SystemReset () ;
    }

    /* Select HSI48 as USB clock source */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        /* Initialization Error */
        NVIC_SystemReset () ;
    }

    /*Configure the clock recovery system (CRS)**********************************/

    /*Enable CRS Clock*/
    __HAL_RCC_CRS_CLK_ENABLE();

    /* Default Synchro Signal division factor (not divided) */
    RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;

    /* Set the SYNCSRC[1:0] bits according to CRS_Source value */
    RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;

    /* HSI48 is synchronized with USB SOF at 1KHz rate */
    RCC_CRSInitStruct.ReloadValue =  __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
    RCC_CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;

    /* Set the TRIM[5:0] to the default value*/
    RCC_CRSInitStruct.HSI48CalibrationValue = 0x20;

    /* Start automatic synchronization */
    HAL_RCCEx_CRSConfig (&RCC_CRSInitStruct);

#elif defined (USE_USB_CLKSOURCE_PLL)

    /* Enable HSE Oscillator and activate PLL with HSE as source
       PLLCLK = (8 * 6) / 1) = 48 MHz */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /*Select PLL 48 MHz output as USB clock source */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /* Select PLL as system clock source and configure the HCLK and PCLK1
       clock dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

#endif /*USE_USB_CLKSOURCE_CRSHSI48*/

}


#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
  */
void assert_failed(char* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
