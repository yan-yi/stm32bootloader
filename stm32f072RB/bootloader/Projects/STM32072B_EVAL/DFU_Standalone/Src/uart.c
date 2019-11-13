
/* Includes ------------------------------------------------------------------*/
#include "usbd_dfu_flash.h"
#include "stm32f0xx_hal.h"
#include "main.h"

#if DEBUG == 1

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#ifdef HAL_UART_MODULE_ENABLED

#define UART1_USE       0
#define UART2_USE       1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief UART MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration
 *           - NVIC configuration for UART interrupt request enable
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_InitStruct;

#if UART1_USE  ==  1
	if (huart->Instance == USART1)
	{

		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Enable GPIO TX/RX clock */
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/* Enable USARTx clock */
		__HAL_RCC_USART1_CLK_ENABLE();

		/*##-2- Configure peripheral GPIO ##########################################*/
		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = GPIO_PIN_9;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;                //jGPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = GPIO_PIN_10;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/*##-3- Configure the NVIC for UART ########################################*/
		/* NVIC for USART */
		HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
	}
#endif

#if UART2_USE == 1
	if (huart->Instance == USART2)
	{
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Enable GPIO TX/RX clock */
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/* Enable USARTx clock */
		__HAL_RCC_USART2_CLK_ENABLE();

		/*##-2- Configure peripheral GPIO ##########################################*/
		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = GPIO_PIN_2;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;                //GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;                //GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/*##-3- Configure the NVIC for UART ########################################*/
		/* NVIC for USART */
		HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(USART2_IRQn);
	}
#endif
}

/**
 * @brief UART MSP De-Initialization
 *        This function frees the hardware resources used in this example:
 *          - Disable the Peripheral's clock
 *          - Revert GPIO and NVIC configuration to their default state
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
#if UART1_USE == 1
	if (huart->Instance == USART1)
	{
		/*##-1- Reset peripherals ##################################################*/
		__HAL_RCC_USART1_FORCE_RESET();
		__HAL_RCC_USART1_RELEASE_RESET();

		/*##-2- Disable peripherals and GPIO Clocks #################################*/
		/* Configure UART Tx as alternate function  */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);
		/* Configure UART Rx as alternate function  */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10);

		/*##-3- Disable the NVIC for UART ##########################################*/
		HAL_NVIC_DisableIRQ(USART1_IRQn);
	}
#endif

#if UART2_USE == 1
	if (huart->Instance == USART2)
	{
		/*##-1- Reset peripherals ##################################################*/
		__HAL_RCC_USART2_FORCE_RESET();
		__HAL_RCC_USART2_RELEASE_RESET();

		/*##-2- Disable peripherals and GPIO Clocks #################################*/
		/* Configure UART Tx as alternate function  */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
		/* Configure UART Rx as alternate function  */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);

		/*##-3- Disable the NVIC for UART ##########################################*/
		HAL_NVIC_DisableIRQ(USART2_IRQn);
	}
#endif
}

#if UART1_USE == 1
void UART_Inital(void)
{
	/*##-1- Configure the UART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART configured as follows:
	    - Word Length = 8 Bits
	    - Stop Bit = One Stop bit
	    - Parity = None
	    - BaudRate = 9600 baud
	    - Hardware flow control disabled (RTS and CTS signals) */
	UartHandle.Instance        = USART1;

	UartHandle.Init.BaudRate   = 115200;

	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;
	//UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_DeInit(&UartHandle) != HAL_OK)
	{
//    Error_Handler();
	}

	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
//    Error_Handler();
	}

//	/*##-2- Put UART peripheral in IT reception process ########################*/
//	/* Any data received will be stored in "BtRxBuffer" buffer  */
//	if (HAL_UART_Receive_IT(&UartHandle, (uint8_t *)UserRxBuffer, 1) != HAL_OK)
//	{
//		/* Transfer error in reception process */
////    Error_Handler();
//	}
}

void UART1_Uninital(void)
{
	HAL_UART_MspDeInit(&UartHandle);

	__HAL_RCC_USART1_CLK_DISABLE();

}

#endif


#if UART2_USE == 1

void UART2_Inital(void)
{
	/*##-1- Configure the UART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART configured as follows:
	    - Word Length = 8 Bits
	    - Stop Bit = One Stop bit
	    - Parity = None
	    - BaudRate = 9600 baud
	    - Hardware flow control disabled (RTS and CTS signals) */
	UartHandle.Instance        = USART2;

	UartHandle.Init.BaudRate   = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;
	UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_DeInit(&UartHandle) != HAL_OK)
	{
//    Error_Handler();
	}

	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
//    Error_Handler();
	}

//	/*##-2- Put UART peripheral in IT reception process ########################*/
//	/* Any data received will be stored in "BtRxBuffer" buffer  */
//	if (HAL_UART_Receive_IT(&UartHandle, (uint8_t *)UserRxBuffer, 1) != HAL_OK)
//	{
//		/* Transfer error in reception process */
////    Error_Handler();
//	}

}

void UART2_Uninital(void)
{
	HAL_UART_MspDeInit(&UartHandle);

	__HAL_RCC_USART2_CLK_DISABLE();
}
#endif

void DEBUG_Inital(void)
{
#if UART1_USE == 1
	UART1_Inital();
#endif

#if UART2_USE == 1
	UART2_Inital();
#endif

}


/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 300);
	return ch;
}

#endif

#endif
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
