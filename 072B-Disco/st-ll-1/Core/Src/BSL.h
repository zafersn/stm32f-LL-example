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

#define LED2_OFF() 						LL_GPIO_ResetOutputPin(LD4_GPIO_Port, LD4_Pin) // Green off
#define LED2_ON() 						LL_GPIO_SetOutputPin(LD4_GPIO_Port, LD4_Pin) // Green ON
#define LED2_TOGGLE() 					LL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin) // Green TOGGLE

#define LED3_OFF() 						LL_GPIO_ResetOutputPin(LD6_GPIO_Port, LD6_Pin) // Green off
#define LED3_ON() 						LL_GPIO_SetOutputPin(LD6_GPIO_Port, LD6_Pin) // Green ON
#define LED3_TOGGLE() 					LL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin) // Green TOGGLE

#define BTN_TOTAL_COUNT 2

#define LED_TOTAL_COUNT 1

#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR        0x78
#endif // SSD1306_I2C_ADDR

// SSD1306 width in pixels
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH           128
#endif // SSD1306_WIDTH

// SSD1306 LCD height in pixels
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT          32
#endif // SSD1306_HEIGHT

#ifndef LCD_SLAVE_ADD
#define LCD_SLAVE_ADD	SSD1306_I2C_ADDR
#endif // LCD_SLAVE_ADD


#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))


/* END Of Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
//
//  Enumeration for screen colors
//
typedef enum {
    Black = 0x00,   // Black color, no pixel
    White = 0x01,   // Pixel is set. Color depends on LCD
} SSD1306_COLOR;

//
//  Struct to store transformations
//
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} SSD1306_t;
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
void SPI_ReadData_MEMS();
void Calgulate_FreqTim14();
void USART_CharReception_Callback(void);

static inline uint8_t I2C_Handle(uint8_t memAddr, uint8_t* pData, uint16_t size)
{

		int32_t to = 0, timeout = 10;
		to = timeout;

		while(LL_I2C_IsActiveFlag_BUSY(I2C2)) {
			if (LL_SYSTICK_IsActiveCounterFlag()) {
				if(to-- == 0) {
					return 1;
				}
			}
		}

		to = timeout;
//	  /* Master Generate Start condition */
	  to = timeout;
	 LL_I2C_HandleTransfer(I2C2, LCD_SLAVE_ADD, LL_I2C_ADDRSLAVE_7BIT, size+1, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

	  while (!LL_I2C_IsActiveFlag_TXE(I2C2)) {
	  		if (LL_SYSTICK_IsActiveCounterFlag()) {
	  			if(to-- == 0) {
	  				return 1;
	  			}
	  		}
	  	}
	  		to = timeout;
	  		LL_I2C_TransmitData8(I2C2, memAddr);

	  		do {
	  			while (!(LL_I2C_IsActiveFlag_TXE(I2C2)) && !(LL_I2C_IsActiveFlag_TC(I2C2))) {
	  				if (LL_SYSTICK_IsActiveCounterFlag()) {
	  					if(to-- == 0) {
	  						return 1;
	  					}
	  				}
	  			}
	  			to = timeout;
	  			LL_I2C_TransmitData8(I2C2, (*pData++));
	  		} while (--size > 0);

	  		while (!LL_I2C_IsActiveFlag_TXE(I2C2)) {
	  			if (LL_SYSTICK_IsActiveCounterFlag()) {
	  				if(to-- == 0) {
	  					return 1;
	  				}
	  			}
	  		}
	  		LL_I2C_GenerateStopCondition(I2C2);
	  		return 0;
}
/* END Of Exported functions ------------------------------------------------------- */

#endif /* SRC_BSL_H_ */
/*** (C) COPYRIGHT -------------- *****END OF FILE****/
