#include "stdio.h"
#include "string.h"
#include "stm32f0xx_hal.h"
//#include "usart1.h"
//#include "usart2.h"

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
  GPIO_InitTypeDef  GPIO_InitStruct;

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
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*##-3- Configure the NVIC for UART ########################################*/
    /* NVIC for USART */
//    HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
//    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
//  else
//  {
//    /*##-1- Enable peripherals and GPIO Clocks #################################*/
//    /* Enable GPIO TX/RX clock */
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    __HAL_RCC_GPIOA_CLK_ENABLE();

//    /* Enable USARTx clock */
//    __HAL_RCC_USART2_CLK_ENABLE();

//    /*##-2- Configure peripheral GPIO ##########################################*/
//    /* UART TX GPIO pin configuration  */
//    GPIO_InitStruct.Pin       = GPIO_PIN_2;
//    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull      = GPIO_PULLUP;
//    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    /* UART RX GPIO pin configuration  */
//    GPIO_InitStruct.Pin = GPIO_PIN_3;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    /*##-3- Configure the NVIC for UART ########################################*/
//    /* NVIC for USART */
////    HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
////    HAL_NVIC_EnableIRQ(USART2_IRQn);
//  }
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
//  else
//  {
//    /*##-1- Reset peripherals ##################################################*/
//    __HAL_RCC_USART2_FORCE_RESET();
//    __HAL_RCC_USART2_RELEASE_RESET();

//    /*##-2- Disable peripherals and GPIO Clocks #################################*/
//    /* Configure UART Tx as alternate function  */
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
//    /* Configure UART Rx as alternate function  */
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);

//    /*##-3- Disable the NVIC for UART ##########################################*/
//    HAL_NVIC_DisableIRQ(USART2_IRQn);
//  }
}


/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}


/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//  if (huart->Instance == USART1)
//  {
//    UART1_receive_process_event();
//  }
//  else
//  {
//    UART2_receive_process_event();
//  }
}


/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  //Error_Handler();

//  if (huart->Instance == USART1)
//  {
//    UART1_Init();
//  }
//  else
//  {
//    UART2_Init();
//  }
}




/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_UART_Rx_Timeout_Check(void)
{
//  UART1_receive_timeout_process_event();

//  UART2_receive_timeout_process_event();
}

void UART_RX_TI_Check(void)
{
//  if ((HAL_UART_GetState(&Uart1Handle) & (0x02)) ==  0)
//  {
//    UART1_Init();
//  }

//  if ((HAL_UART_GetState(&Uart2Handle) & (0x02)) ==  0)
//  {
//    UART2_Init();
//  }
}
