#ifndef  __USART_H
#define  __USART_H
#include "stm32f0xx_hal.h"

void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);
void HAL_UART_Rx_Timeout_Check(void);
void UART_RX_TI_Check(void);


#endif/*__USART_H*/



