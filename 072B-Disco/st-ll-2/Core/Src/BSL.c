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


/**
  * @brief Variables related to MasterTransmit process
  */

uint8_t 		SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 4];
SSD1306_t SSD1306;
double temperature = 0, humidity = 0;
volatile uint32_t ic_value1 = 0, ic_value2 = 0, is_first_capture = 0, difference = 0, freqTim14 = 0, expect_freq = 0, currentFreq =100;

/* END Of Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/*
void BSL_Startup();

void BSL_Config();

void BSL_Control();*/
void IIC_Start(void);
static uint8_t ssd1306_WriteCommand(uint8_t command);
static uint8_t I2C_Handle(uint8_t memAddr, uint8_t* pData,uint16_t size);

static void LED_TEST();
void ssd1306_UpdateScreen();
void ssd1306_Fill(SSD1306_COLOR color);
void Display_Init();
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color);
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);
void Display_Test();
void LED_Init();
void DelayUs(uint32_t us);
void Init_TIM6(void);
void PWM_Test();
void TIM_PWM_Init();
void Init_TIM15_CH1N(void);
void PWM15_Test();
void Init_TIM14_INCAP(void);
void Init_USERBTN();

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
	Display_Init();
	Init_TIM6();
	TIM_PWM_Init();
	Init_TIM14_INCAP();
	Init_TIM15_CH1N();
	Init_USERBTN();
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

			Display_Test();

			PWM15_Test();
		}
	}
}

/**
  * @brief   init func. for display.
  * @param  ------
  * @retval none
  */
void Display_Init()
{
		int status = 0;

			LL_I2C_SetTransferSize(I2C2, 50);
		// Init LCD
			status += ssd1306_WriteCommand(0xAE);   // Display off
		if (status != 0) {
			return ;
		}
			status += ssd1306_WriteCommand( 0x20);   // Set Memory Addressing Mode
		    status += ssd1306_WriteCommand( 0x00);   // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
		    status += ssd1306_WriteCommand( 0xB0);   // Set Page Start Address for Page Addressing Mode,0-7
		    status += ssd1306_WriteCommand( 0xC8);   // Set COM Output Scan Direction
		    status += ssd1306_WriteCommand( 0x00);   // Set low column address
		    status += ssd1306_WriteCommand( 0x10);   // Set high column address
		    status += ssd1306_WriteCommand( 0x40);   // Set start line address
		    status += ssd1306_WriteCommand( 0x81);   // set contrast control register
		    status += ssd1306_WriteCommand( 0xFF);
		    status += ssd1306_WriteCommand( 0xA1);   // Set segment re-map 0 to 127
		    status += ssd1306_WriteCommand( 0xA6);   // Set normal display
		    status += ssd1306_WriteCommand( 0xA8);   // Set multiplex ratio(1 to 64)
		    status += ssd1306_WriteCommand( 0x1F);	// ( SSD1306_HEIGHT == 32)
		    status += ssd1306_WriteCommand( 0xA4);   // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
		    status += ssd1306_WriteCommand( 0xD3);   // Set display offset
		    status += ssd1306_WriteCommand( 0x00);   // No offset
		    status += ssd1306_WriteCommand( 0xD5);   // Set display clock divide ratio/oscillator frequency
		    status += ssd1306_WriteCommand( 0xF0);   // Set divide ratio
		    status += ssd1306_WriteCommand( 0xD9);   // Set pre-charge period
		    status += ssd1306_WriteCommand( 0x22);
		    status += ssd1306_WriteCommand( 0xDA);   // Set com pins hardware configuration
		#ifdef SSD1306_COM_LR_REMAP
		    status += ssd1306_WriteCommand( 0x32);   // Enable COM left/right remap
		#else
		    status += ssd1306_WriteCommand( 0x02);   // Do not use COM left/right remap
		#endif // SSD1306_COM_LR_REMAP
		    status += ssd1306_WriteCommand( 0xDB);   // Set vcomh
		    status += ssd1306_WriteCommand( 0x20);   // 0x20,0.77xVcc
		    status += ssd1306_WriteCommand( 0x8D);   // Set DC-DC enable
		    status += ssd1306_WriteCommand( 0x14);   //
		    status += ssd1306_WriteCommand( 0xAF);   // Turn on SSD1306 panel

		    if (status != 0) {
		        return ;
		    }
		    LL_mDelay(100);
			ssd1306_Fill(White);
			ssd1306_UpdateScreen();
			  // Set default values for screen object
			SSD1306.CurrentX = 0;
			SSD1306.CurrentY = 0;

			SSD1306.Initialized = 1;
			LL_mDelay(100);

}


void Display_Test()
{
	ssd1306_Fill(Black);
	/*for (int i = 0; i < 128; i++) {
		ssd1306_DrawPixel(i, 10, White);
	}*/
	char tBuf[255];
	sprintf(tBuf,"expctF:%d",(int)expect_freq);
	ssd1306_SetCursor(0, 10);
	ssd1306_WriteString(tBuf, Font_7x10, White);
	sprintf(tBuf,"msredF:%d",(int)freqTim14);
	ssd1306_SetCursor(0, 20);
	ssd1306_WriteString(tBuf, Font_7x10, White);
	ssd1306_UpdateScreen();
}
static uint8_t ssd1306_WriteCommand(uint8_t command)
{
	uint8_t memAdd= 0x00;
	uint16_t size = 1;
	return I2C_Handle(memAdd,&command,size);
}
void LED_Init()
{
	LED1_OFF();
	LED2_OFF();
}
void Init_TIM6(void)
{
	LL_TIM_ClearFlag_UPDATE(TIM6);
	/* Enable counter */
	LL_TIM_EnableCounter(TIM6);

	/* Force update generation */
	LL_TIM_GenerateEvent_UPDATE(TIM6);
}
void Init_TIM15_CH1N(void)
{
	LL_TIM_ClearFlag_UPDATE(TIM15);
	LL_TIM_CC_EnableChannel(TIM15, 		LL_TIM_CHANNEL_CH1N);

	/* Enable counter */
	LL_TIM_EnableCounter(TIM15);
	LL_TIM_EnableAllOutputs(TIM15);

	/* Force update generation */
	LL_TIM_GenerateEvent_UPDATE(TIM15);

}
void PWM15_Test()
{
		LL_TIM_OC_SetCompareCH1(TIM15, LL_TIM_GetAutoReload(TIM15)/2);
		LL_TIM_SetAutoReload(TIM15,	currentFreq);

}

void Init_USERBTN()
{
	expect_freq = LL_TIM_GetAutoReload(TIM15)/2;
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);
	LL_EXTI_EnableEvent_0_31(LL_EXTI_LINE_0);
}
void UserBTN_Callback()
{
	static char swtch = 1;

	(swtch == 1) ? (currentFreq += 100 ) : (currentFreq -= 100);
	if(currentFreq >= 65535)
	{
		swtch = 0;
		currentFreq = 65535;
	}
	else if( currentFreq <= 100)
	{
		swtch = 1;
		currentFreq = 100;
	}
	expect_freq = 48000000 /48 / (currentFreq );

}
void Init_TIM14_INCAP(void)
{
	LL_TIM_EnableIT_CC1(TIM14);
	LL_TIM_CC_EnableChannel(TIM14, 		LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM14);

}

void Calgulate_FreqTim14()
{
	if(LL_TIM_IsActiveFlag_CC1(TIM14))
	{
		LL_TIM_ClearFlag_CC1(TIM14);
		LL_TIM_ClearFlag_UPDATE(TIM14);

		if(TIM14->CCER == LL_TIM_CHANNEL_CH1)
		{
			if(!IS_TIM_CC1_INSTANCE(TIM14))
			return;
			if(is_first_capture == 0)
			{
				ic_value1 = LL_TIM_OC_GetCompareCH1(TIM14);
				is_first_capture = 1;
			}
			else if(is_first_capture == 1)
			{
				ic_value2 = LL_TIM_OC_GetCompareCH1(TIM14);

				if(ic_value2 > ic_value1)
				{
					difference = ic_value2 - ic_value1;
				}
				else if(ic_value2 < ic_value1)
				{
					difference = (0xffff - ic_value1) + ic_value2 + 1;
				}
				else
				{
				//	Error_Handler();
				}
	//			LL_RCC_ClocksTypeDef RCC_Clocks;
	//			LL_RCC_GetSystemClocksFreq(&RCC_Clocks);
				uint32_t PCLK1_Frequency = 48000000;
				freqTim14 = PCLK1_Frequency / difference;
				is_first_capture = 0;
			}

		}
	}
}

void TIM_PWM_Init()
{
	LL_TIM_OC_SetCompareCH4(TIM3, 0);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	LL_TIM_ClearFlag_UPDATE(TIM3);
	/* Enable counter */
	LL_TIM_EnableCounter(TIM3);

	/* Force update generation */
	LL_TIM_GenerateEvent_UPDATE(TIM3);


}
void PWM_Test()
{

	for(int i = 0; i< LL_TIM_GetAutoReload(TIM3); i+=20)
	{
		LL_TIM_OC_SetCompareCH4(TIM3, i);
		LL_mDelay(10);

	}
	for(int i = LL_TIM_GetAutoReload(TIM3); i > 0; i-=20)
	{
		LL_TIM_OC_SetCompareCH4(TIM3, i);
		LL_mDelay(10);
	}


	LL_TIM_OC_SetCompareCH4(TIM3, 0);
}


void DelayUs(uint32_t us)
{
	LL_TIM_SetCounter(TIM6, 0);
	while(LL_TIM_GetCounter(TIM6) < us);
}
/**
  * @brief   Write the screenbuffer with changed to the screen
  * @param  ------
  * @retval none
  */
void ssd1306_UpdateScreen()
{
    uint8_t i;
    int sum = 0;

    for (i = 0; i < SSD1306_HEIGHT/4; i++) {
    	sum+= ssd1306_WriteCommand( 0xB0 + i);
    	sum+= ssd1306_WriteCommand( 0x00);
    	sum+= ssd1306_WriteCommand( 0x10);
    	sum+= I2C_Handle(0x40, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
    }
    if(sum != 0)
    {
    	LED1_ON();
    }
    else
    {
    	LED2_ON();
    }
}

/**
  * @brief   Fill the whole screen with the given color
  * @param  ------
  * @retval none
  */
void ssd1306_Fill(SSD1306_COLOR color)
{
    // Fill screenbuffer with a constant value (color)
    uint32_t i;

    for(i = 0; i < sizeof(SSD1306_Buffer); i++)
    {
        SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}
/**
  * @brief   Write full string to screenbuffer
  * @param  ------
  * @retval none
  */
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color)
{
    // Write until null-byte
    while (*str)
    {
        if (ssd1306_WriteChar(*str, Font, color) != *str)
        {
            // Char could not be written
            return *str;
        }

        // Next char
        str++;
    }

    // Everything ok
    return *str;
}

/**
  * @brief   Draw one pixel in the screenbuffer
  * @param  X => X Coordinate
  * 		Y => Y Coordinate
  * 		color => Pixel color
  * @retval none
  */
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    {
        // Don't write outside the buffer
        return;
    }

    // Check if pixel should be inverted
    if (SSD1306.Inverted)
    {
        color = (SSD1306_COLOR)!color;
    }

    // Draw in the correct color
    if (color == White)
    {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    }
    else
    {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

/**
  * @brief   Set cursor position
  * @param  x      => axis
  * 		y      => axis
  * @retval none
  */
void ssd1306_SetCursor(uint8_t x, uint8_t y)
{
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}

/**
  * @brief  Draw 1 char to the screen buffer
  * @param  ch      => Character to write
  * 		Font    => Font to use
  * 		color   => Black or White
  * @retval char
  */
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
    uint32_t i, b, j;

    // Check remaining space on current line
    if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
        SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
    {
        // Not enough space on current line
        return 0;
    }

    // Translate font to screenbuffer
    for (i = 0; i < Font.FontHeight; i++)
    {
        b = Font.data[(ch - 32) * Font.FontHeight + i];
        for (j = 0; j < Font.FontWidth; j++)
        {
            if ((b << j) & 0x8000)
            {
                ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
            }
            else
            {
                ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
            }
        }
    }

    // The current space is now taken
    SSD1306.CurrentX += Font.FontWidth;

    // Return written char for validation
    return ch;
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
