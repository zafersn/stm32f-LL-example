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
#include "fonts.h"
#include "stm32f0xx_ll_rcc.h"
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
uint8_t      aReceiveBuffer[0xF] = {0};
__IO uint8_t ubReceiveIndex      = 0;

/* END Of Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/*
void BSL_Startup();

void BSL_Config();

void BSL_Control();*/

void LED_Init();

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
	}
}

void LED_Init()
{
	LED1_OFF();
	LED2_OFF();
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
  LL_GPIO_SetOutputPin(LE_DC6_GPIO_Port, LE_DC6_Pin);
}

/**
  * @brief  Turn-off LED4.
  * @param  None
  * @retval None
  */
void LED_Off(void)
{
  /* Turn LED4 off */
  LL_GPIO_ResetOutputPin(LE_DC6_GPIO_Port, LE_DC6_Pin);
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
  LL_GPIO_SetOutputPin(LE_DC6_GPIO_Port, LE_DC6_Pin);

  /* Toggle IO in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LE_DC6_GPIO_Port, LE_DC6_Pin);
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
