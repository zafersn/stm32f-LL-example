/**
  ******************************************************************************
  * @file    BSL.c
  * @fileLoc C:\Users\TR\STM32CubeIDE\workspace_1.6.1\Test072B-Disco\Core\Src\BSL.c
  * @author  TR
  * @version V1.0.0
  * @date    Jun 16, 2021
  * @brief   
  * @verbatim
  *
  *		V1.0.0 : 	First version of library.
  *		<describe this file>
  * @endverbatim
  * ===========================================================================
  *                       ##### How to use this file #####
  * ===========================================================================
  *	@attention
  *
  *	@todo
  *
  *
  *
  * ===========================================================================
  *                       ##### Change Activity #####
  * ===========================================================================
  * mm/dd/yy   author   description
  * --------  -------   -----------
  * 11/27/21   TR   Initial coding
  *
**/


/* Includes ------------------------------------------------------------------*/
#include "BSL.h"
#include "stdbool.h"
#include <string.h>
#include "main.h"
/* END Of Includes ------------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* END Of Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* END Of Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* END Of Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__IO bool ms50ms = false;
__IO bool ms500ms = false;
__IO bool ms3000ms = false;

/**
  * @brief Variables related to MasterReceive process
  */
const uint8_t len_TX_BUF = 6;
uint8_t      u_RXBuffer1[10];
uint8_t      u_TXBuffer1[] = {'T','X','H','I','2',' '};
__IO uint8_t ubReceiveIndex1      = 0;

uint8_t      u_RXBuffer2[10];
uint8_t      u_TXBuffer2[] = {'r','X','M','U','3',' '};
__IO uint8_t ubReceiveIndex2      = 0;

/* END Of Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/*
void BSL_Startup();

void BSL_Config();

void BSL_Control();*/

void LED_Init();
void UART1_Init();
void UART2_Init();
void UART1_TXHandle();
void UART2_TXHandle();

/* END Of Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  TODO Auto-generated constructor stub
 * @param  (parm_1,parm_2,...)
 * @retval (parm_1,parm_2,-help,-debug)
 */
void BSL_Startup()
{
	LED_Init();
	UART2_Init();

	UART1_Init();
}

void UART1_Init()
{
	ubReceiveIndex1 = 0;
#ifdef STM32F0xx_HAL_UART_H
	HAL_UART_Receive_IT(&huart1, u_RXBuffer1, len_TX_BUF);
#else
	LL_USART_EnableHalfDuplex(USART1);
	LL_USART_EnableIT_RXNE(USART1);
#endif
}
void UART2_Init()
{
	ubReceiveIndex2 = 0;
#ifdef STM32F0xx_HAL_UART_H
	HAL_UART_Receive_IT(&huart3, u_RXBuffer2, len_TX_BUF);
#else
	LL_USART_EnableHalfDuplex(USART3);
		LL_USART_EnableIT_RXNE(USART3);
#endif
}


/**
 * @brief  TODO Auto-generated constructor stub
 * @param  (parm_1,parm_2,...)
 * @retval (parm_1,parm_2,-help,-debug)
 */
void BSL_Config()
{
	LL_SYSTICK_EnableIT();
}
int i = 0;
/**
 * @brief  TODO Auto-generated constructor stub
 * @param  (parm_1,parm_2,...)
 * @retval (parm_1,parm_2,-help,-debug)
 */
void BSL_Control()
{

	while(1)
	{
		if(ms500ms)
		{
			ms500ms = false;
		}

		u_TXBuffer1[5] = u_TXBuffer2[5]= i++;
		UART1_TXHandle();
		if(u_RXBuffer2[4] == '2' && u_RXBuffer2[3] == 'I')
		{
			UART2_TXHandle();
		}
		LL_mDelay(500);
	}
}

void LED_Init()
{
	LED1_OFF();

}

void UART1_TXHandle()
{
	ubReceiveIndex1 = 0;
	ubReceiveIndex2 = 0;
#ifdef STM32F0xx_HAL_UART_H
	HAL_HalfDuplex_EnableTransmitter(&huart1);
	HAL_HalfDuplex_EnableReceiver(&huart3);

	memset(&u_RXBuffer2,0,sizeof(u_RXBuffer2));

	HAL_UART_Transmit(&huart1, u_TXBuffer1, len_TX_BUF, 10);
	while((READ_BIT(huart1.Instance->ISR, USART_ISR_TC) != (USART_ISR_TC)));
#else
	LL_USART_EnableDirectionTx(USART1);
	LL_USART_EnableDirectionRx(USART3);
	for(int i = 0; i<len_TX_BUF;i++)
	{
		LL_USART_TransmitData8(USART1, u_TXBuffer1[i]);

		while(!LL_USART_IsActiveFlag_TC(USART1));
	}
	if (LL_USART_IsActiveFlag_TC(USART1))
	{
		LL_USART_ClearFlag_TC(USART1); /* Clear transfer complete flag */
	}
#endif
}
void UART1_Intrrpt_Handle()
{
	if(LL_USART_IsActiveFlag_RXNE(USART1) )
	{
			u_RXBuffer1[ubReceiveIndex1++] = LL_USART_ReceiveData8(USART1);
	}
}
void UART2_TXHandle()
{
	ubReceiveIndex2 = 0;
	ubReceiveIndex1 = 0;
#ifdef STM32F0xx_HAL_UART_H
	HAL_HalfDuplex_EnableTransmitter(&huart3);
	HAL_HalfDuplex_EnableReceiver(&huart1);

	memset(&u_RXBuffer1,0,sizeof(u_RXBuffer1));

	HAL_UART_Transmit(&huart3, u_TXBuffer2, len_TX_BUF, 10);
	while((READ_BIT(huart3.Instance->ISR, USART_ISR_TC) != (USART_ISR_TC)));
#else
	LL_USART_EnableDirectionTx(USART3);
	LL_USART_EnableDirectionRx(USART1);
	for(int i = 0; i<len_TX_BUF;i++)
	{
		LL_USART_TransmitData8(USART3, u_TXBuffer2[i]);

		 while(!LL_USART_IsActiveFlag_TC(USART3));
	}
	if (LL_USART_IsActiveFlag_TC(USART3))
	{
			LL_USART_ClearFlag_TC(USART3); /* Clear transfer complete flag */
	}
#endif

}
void UART2_Intrrpt_Handle()
{
	if(LL_USART_IsActiveFlag_RXNE(USART3) )
	{
			u_RXBuffer2[ubReceiveIndex2++] = LL_USART_ReceiveData8(USART3);
	}
}
/**
  * @brief  Function called in case of error detected in I2C IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void)
{
  /* Unexpected event : Set LED4 to Blinking mode to indicate error occurs */
  LED_Blinking(LED_BLINK_ERROR);
}
/**
  * @brief  Turn-on LED4.
  * @param  None
  * @retval None
  */
void LED_On(void)
{
  /* Turn LED4 on */
  LL_GPIO_SetOutputPin(LED_C7_GPIO_Port, LED_C7_Pin);
}

/**
  * @brief  Turn-off LED4.
  * @param  None
  * @retval None
  */
void LED_Off(void)
{
  /* Turn LED4 off */
  LL_GPIO_ResetOutputPin(LED_C7_GPIO_Port, LED_C7_Pin);
}

/**
  * @brief  Set LED4 to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
  * @param  Period : Period of time (in ms) between each toggling of LED
  *   This parameter can be user defined values. Pre-defined values used in that example are :
  *     @arg LED_BLINK_FAST : Fast Blinking
  *     @arg LED_BLINK_SLOW : Slow Blinking
  *     @arg LED_BLINK_ERROR : Error specific Blinking
  * @retval None
  */
void LED_Blinking(uint32_t Period)
{
  /* Turn LED4 on */
  LL_GPIO_SetOutputPin(LED_C7_GPIO_Port, LED_C7_Pin);

  /* Toggle IO in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED_C7_GPIO_Port, LED_C7_Pin);
    LL_mDelay(Period);
  }
}

void BSL_Tick()
{
	static __IO uint32_t cntr_ms_50 = 0;
	static __IO uint32_t cntr_ms_500 = 0;
	static __IO uint32_t cntr_ms_3000 = 0;

	if(++cntr_ms_50 >49)
	{
		ms50ms = true;
		cntr_ms_50 = 0;
	}
	if(++cntr_ms_500 >499)
		{
			ms500ms = true;
			cntr_ms_500 = 0;
		}
	if(++cntr_ms_3000 > 2999)
	{
		ms3000ms = true;
		cntr_ms_3000 = 0;
	}
}
/* END Of Private functions ---------------------------------------------------------*/

/*** (C) COPYRIGHT -------------------  *****END OF FILE****/
