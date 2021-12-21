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
#define LED1_OFF() 						LL_GPIO_ResetOutputPin(LED_C7_GPIO_Port, LED_C7_Pin) // Green off
#define LED1_ON() 						LL_GPIO_SetOutputPin(LED_C7_GPIO_Port, LED_C7_Pin) // Green ON
#define LED1_TOGGLE() 					LL_GPIO_TogglePin(LED_C7_GPIO_Port, LED_C7_Pin) // Green TOGGLE

#define BTN_TOTAL_COUNT 2

#define LED_TOTAL_COUNT 1

#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#ifndef F_CPU
#warning "F_CPU not defined"
#define F_CPU 48000000L
#endif

#define BAUD_115200 (uint16_t)((64L * F_CPU) / (16L * 115200L))
#define BAUD_9600 (uint16_t)((64L * F_CPU) / (16L * 9600L))
/*
 *
 *
 * // Calculate baud register value for 9600 baud
	// Baud rate compensated with factory stored frequency error
	// Synchronous communication without Auto-baud (Sync Field)
	// 20MHz Clock, and 3V. For different voltage and clock this should be changed
	int8_t  sigrow_value = SIGROW.OSC20ERR3V; // read signed error
	int32_t baud         = BAUD_9600;         // ideal baud rate
	baud *= (1024 + sigrow_value);            // sum resolution + error
	baud /= 1024;                             // divide by resolution
	baud_9600 = (int16_t)baud;

	// Set baud rate to 115200
	// Baud rate compensated with factory stored frequency error
	// Synchronous communication without Auto-baud (Sync Field)
	// 20MHz Clock, and 3V. For different voltage and clock this should be changed
	baud = BAUD_115200;            // ideal baud rate
	baud *= (1024 + sigrow_value); // sum resolution + error
	baud /= 1024;                  // divide by resolution
	baud_115200 = (int16_t)baud;
	USART0.BAUD = baud_115200; // set adjusted baud rate
 */
/* END Of Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* END Of Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* END Of Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
__IO bool ms50ms;
__IO bool ms3000ms;
extern uint8_t      u_RXBuffer1[10] ;
extern uint8_t      u_RXBuffer2[10] ;
extern const uint8_t len_TX_BUF;
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
void UART2_Intrrpt_Handle();
void UART1_Intrrpt_Handle();


/* END Of Exported functions ------------------------------------------------------- */

#endif /* SRC_BSL_H_ */
/*** (C) COPYRIGHT -------------- *****END OF FILE****/
