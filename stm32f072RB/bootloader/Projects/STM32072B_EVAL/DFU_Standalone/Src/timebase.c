/*
 * @Author: Green
 * @Date:   2016-06-28 14:54:08
 * @Last Modified by:   Green
 * @Last Modified time: 2016-06-28 14:56:59
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/** @defgroup Private Variables
 * @{
 */
__IO uint32_t g_TimerCurrentValue;
__IO uint32_t g_TimerLastValue;
__IO uint32_t down_timer;

void USER_IncTick(void)
{
	g_TimerCurrentValue++;
}


//
//========================================================
// Function Name: API_GetTimerCnt
// Syntax:    unsigned API_GetTimerCnt(void);
// Purpose:   get system timer value[x] - value[x-1].
// Note:    No
// Parameter :  N/A
// Return:    system timer tick number.
//
// Destory:   N/A
// Stack Depth:
// CPU Cycle:
// Transfer method: stack
//========================================================
uint32_t API_GetTimerCnt(void)
{
	uint32_t temp;

	temp = g_TimerLastValue;
	g_TimerLastValue = g_TimerCurrentValue;
	if (g_TimerLastValue >= temp)
	{
		return (g_TimerLastValue - temp);
	}
	else
	{
		return 1;
	}
}

//-------------------------------------------------------
// void API_Timer_Reset(void);
//-------------------------------------------------------
void API_Timer_Reset(void)
{
	g_TimerLastValue = 0;
	g_TimerCurrentValue = 0;
	down_timer = 0;
}


//============================================================================
//Function Name: void SetTimer(INT32U delay_time_msec)
//Purpose      : This function Set Down Timer.
//Note         :
//Parameters   : INT32U delay_time_msec
//Return       : void
//Date         : 2013/06/03
//Revision     : Ver 1.00
//============================================================================

void SetTimer(uint32_t delay_time_msec)
{
	down_timer = g_TimerCurrentValue + delay_time_msec;
}

//============================================================================
//Function Name: INT32U CheckTimer(void)
//Purpose      : This function Set Down Counter.
//Note         :
//Parameters   : INT32U delay_time_msec
//Return       : >0=Remaining time , 0=Time Up
//Date         : 2013/06/03
//Revision     : Ver 1.00
//============================================================================
uint32_t CheckTimer(void)
{
	if (down_timer > g_TimerCurrentValue) // set end timer
	{
		return  (down_timer - g_TimerCurrentValue);
	}
	else
	{
		return 0;
	}
}

