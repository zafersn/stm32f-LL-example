/**
  ******************************************************************************
  * @file    BSL.h
  * @fileLoc C:\Users\TR\STM32CubeIDE\workspace_1.6.1\Test072B-Disco\Core\Src\BSL.h
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SRC_BSL_H_
#define SRC_BSL_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdbool.h"
/* END Of Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/


#define GPIO_OFF()						LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET) // Green off
#define GPIO_ON() 						LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET) // Green ON
#define GPIO_TOGGLE() 					LL_GPIO_TogglePin(GPIOC, GPIO_PIN_5) // Green TOGGLE

/*LEDS*/
#define LED1_OFF() 						LL_GPIO_ResetOutputPin(LE_DC6_GPIO_Port, LE_DC6_Pin) // Green off
#define LED1_ON() 						LL_GPIO_SetOutputPin(LE_DC6_GPIO_Port, LE_DC6_Pin) // Green ON
#define LED1_TOGGLE() 					LL_GPIO_TogglePin(LE_DC6_GPIO_Port, LE_DC6_Pin) // Green TOGGLE

#define LED2_OFF() 						LL_GPIO_ResetOutputPin(LE_DC7_GPIO_Port, LE_DC7_Pin) // Green off
#define LED2_ON() 						LL_GPIO_SetOutputPin(LE_DC7_GPIO_Port, LE_DC7_Pin) // Green ON
#define LED2_TOGGLE() 					LL_GPIO_TogglePin(LE_DC7_GPIO_Port, LE_DC7_Pin) // Green TOGGLE

#define LED3_OFF() 						LL_GPIO_ResetOutputPin(LE_DC8_GPIO_Port, LE_DC8_Pin) // Green off
#define LED3_ON() 						LL_GPIO_SetOutputPin(LE_DC8_GPIO_Port, LE_DC8_Pin) // Green ON
#define LED3_TOGGLE() 					LL_GPIO_TogglePin(LE_DC8_GPIO_Port, LE_DC8_Pin) // Green TOGGLE

#define BTN_TOTAL_COUNT 2

#define LED_TOTAL_COUNT 1

#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))


/* END Of Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* END Of Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* END Of Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
__IO bool ms50ms;
__IO bool ms3000ms;

/* END Of Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/**
 * @brief  TODO Auto-generated constructor stub
 * @param  (parm_1,parm_2,...)
 * @retval (parm_1,parm_2,-help,-debug)
 */
void BSL_Startup();

/**
 * @brief  TODO Auto-generated constructor stub
 * @param  (parm_1,parm_2,...)
 * @retval (parm_1,parm_2,-help,-debug)
 */
void BSL_Config();

/**
 * @brief  TODO Auto-generated constructor stub
 * @param  (parm_1,parm_2,...)
 * @retval (parm_1,parm_2,-help,-debug)
 */
void BSL_Control();
void BSL_Tick();
void Error_Callback(void);
void LED_On(void);
void LED_Off(void);
void LED_Blinking(uint32_t Period);
/* END Of Exported functions ------------------------------------------------------- */

#endif /* SRC_BSL_H_ */
/*** (C) COPYRIGHT -------------- *****END OF FILE****/
