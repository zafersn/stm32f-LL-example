/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t us_10_counter = 0;
volatile bool is_10us_done = false,is_10us_done2 = false;;
volatile uint32_t interval_high ;
volatile uint32_t us_20_counter = 0;
volatile bool is_20us_done = false;

volatile uint32_t us_80_counter = 0;
volatile bool is_80us_done = false;

__IO uint8_t ubSend = 0;
double temperature, humidity,rawV;
char txData[255];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void     LED_Init(void);
void     LED_On(void);
void     LED_Off(void);
void     LED_Blinking(uint32_t Period);

void Init_TIM2(void);
void Init_TIM3(void);

void DHT11_Init(void);
void delay_us (uint16_t us);
int ReadDHT11(void);
int ReadDHT(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t dht_byts[5];  // buffer to receive data
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  LED_Init();
  DHT11_Init();
  Init_TIM2();
  Init_TIM3();
  LED_Off();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  rawV = ReadDHT11();
	  LL_mDelay(200);

		  	if (ubSend == 0)
			{
			/* Start USART transmission : Will initiate TXE interrupt after DR register is empty */
			LL_USART_TransmitData8(USART1, dht_byts[ubSend++]);

			/* Enable TXE interrupt */
			LL_USART_EnableIT_TXE(USART1);
			}

	  LL_mDelay(200);
	 /* LL_TIM_SetCounter(TIM3, 0);
	  LL_mDelay(1);
	  uint32_t v= LL_TIM_GetCounter(TIM3);*/
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(64000000);
  LL_SetSystemCoreClock(64000000);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PA8   ------> TIM1_CH1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 9;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 63;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 63;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0xffff-LL_TIM_IC_FILTER_FDIV1_N2;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_ERROR(USART1);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LED_Pin|LL_GPIO_PIN_14);

  /**/
  LL_GPIO_ResetOutputPin(DHT11_IN_GPIO_Port, DHT11_IN_Pin);

  /**/
  GPIO_InitStruct.Pin = LED_Pin|LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DHT11_IN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(DHT11_IN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Initialize LED2.
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
  /* Enable the LED Clock */
 // LED_GPIO_CLK_ENABLE();

  /* Configure IO in output push-pull mode to drive external LED */
 // LL_GPIO_SetPinMode(LED_GPIO_Port, LED_Pin, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  //LL_GPIO_SetPinOutputType(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
  //LL_GPIO_SetPinSpeed(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_SPEED_FREQ_LOW);
  /* Reset value is LL_GPIO_PULL_NO */
  //LL_GPIO_SetPinPull(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_PULL_NO);
}

/**
  * @brief  Initialize dht11.
  * @param  None
  * @retval None
  */
void DHT11_Init(void)
{

/* Enable the LED Clock */
	DHT11_GPIO_CLK_ENABLE();

	LL_GPIO_SetPinMode(DHT11_IN_GPIO_Port, DHT11_IN_Pin, LL_GPIO_MODE_OUTPUT);
	/* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
	LL_GPIO_SetPinOutputType(DHT11_IN_GPIO_Port, DHT11_IN_Pin, LL_GPIO_OUTPUT_PUSHPULL);
	/* Reset value is LL_GPIO_SPEED_FREQ_LOW */
	LL_GPIO_SetPinSpeed(DHT11_IN_GPIO_Port, DHT11_IN_Pin, LL_GPIO_SPEED_FREQ_LOW);
	/* Reset value is LL_GPIO_PULL_NO */
	//LL_GPIO_SetPinPull(DHT11_IN_GPIO_Port, DHT11_IN_Pin, LL_GPIO_OUTPUT_PUSHPULL);
}

int ReadDHT11(void)
{
	// READ VALUES
	int rv = ReadDHT();
	if (rv != 0) return rv;

	// CONVERT AND STORE
	humidity    = dht_byts[0];  // bit[1] == 0;
	temperature = dht_byts[2];  // bits[3] == 0;

	// TEST CHECKSUM
	uint8_t sum = dht_byts[0] + dht_byts[2]; // bits[1] && bits[3] both 0
	if (dht_byts[4] != sum) return -1;

	return 0;
}

/**
  * @brief  read data from dht11 sensor.
  * @param  None
  * @retval :
  *  0 : OK
  *  -2 : timeout
  */
int ReadDHT(void)
{
	// INIT BUFFERVAR TO RECEIVE DATA
		uint8_t cnt = 7;
		uint8_t idx = 0;

		// EMPTY BUFFER
	for (int i=0; i< 5; i++) dht_byts[i] = 0;

	LL_GPIO_SetPinMode(DHT11_IN_GPIO_Port, DHT11_IN_Pin, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinPull(DHT11_IN_GPIO_Port, DHT11_IN_Pin, LL_GPIO_OUTPUT_PUSHPULL);

	LL_GPIO_ResetOutputPin(DHT11_IN_GPIO_Port, DHT11_IN_Pin);

	LL_mDelay(20);

	LL_GPIO_SetOutputPin(DHT11_IN_GPIO_Port, DHT11_IN_Pin);

	delay_us(4);//40uS delay

	LL_GPIO_SetPinMode(DHT11_IN_GPIO_Port, DHT11_IN_Pin, LL_GPIO_MODE_FLOATING);
	//LL_GPIO_SetPinPull((DHT11_IN_GPIO_Port, DHT11_IN_Pin,)
	int tims = 0;
	//uint32_t vl = LL_GPIO_IsInputPinSet(DHT11_IN_GPIO_Port, DHT11_IN_Pin);

	while(!LL_GPIO_IsInputPinSet(DHT11_IN_GPIO_Port, DHT11_IN_Pin))
	{
		if(is_10us_done)
		{
			is_10us_done = false;
			if(++tims > 9)
			{
				return -2;
			}
		}
	}
	tims = 0;
	while(LL_GPIO_IsInputPinSet(DHT11_IN_GPIO_Port, DHT11_IN_Pin))
	{
		if(is_10us_done)
		{
			is_10us_done = false;
			if(++tims > 9)
			{
				return -2;
			}
		}
	}

	for (int i = 0; i < 40; i++)
	{
		tims = 0;
		//vl = LL_GPIO_IsInputPinSet(DHT11_IN_GPIO_Port, DHT11_IN_Pin);
		while(!LL_GPIO_IsInputPinSet(DHT11_IN_GPIO_Port, DHT11_IN_Pin))
		{
			if(is_10us_done)
			{
				is_10us_done = false;
				if(++tims > 6)
				{
					return -2;
				}
			}
		}
		tims = 0;
//
		LL_TIM_SetCounter(TIM3, 0);
	//	volatile uint32_t cnt_timer = TIM3->CNT;

		while(LL_GPIO_IsInputPinSet(DHT11_IN_GPIO_Port, DHT11_IN_Pin))
		{
			if(is_10us_done)
			{
				is_10us_done = false;
				if(++tims > 8)
				{
					return -2;
				}
			}
		}
		interval_high = LL_TIM_GetCounter(TIM3);// - cnt_timer;
		if(interval_high > 40)
		{
			dht_byts[idx] |= (1 << cnt);
		}
		if (cnt == 0)   // next byte?
		{
			cnt = 7;
			idx++;
		}
		else cnt--;
	}
	return 0;
}
/**
  * @brief  to start tim1
  * 		Updatefrequency = TIM clk/((PSC+1)*(ARR+1))
  * 		SO, we need 10uS(100K hz) delay. therefore, 8 000 000 / (9+1) / (7 + 1) = 100K
  *			PSC=9, ARR = 15
  * @param  None
  * @retval None
  */
void Init_TIM2(void)
{
	/* Enable the update interrupt */
	  LL_TIM_EnableIT_UPDATE(TIM2);
	/* Enable counter */
	  LL_TIM_EnableCounter(TIM2);

	  /* Force update generation */
	  LL_TIM_GenerateEvent_UPDATE(TIM2);
}

void Init_TIM3(void)
{

	/* Enable counter */
	  LL_TIM_EnableCounter(TIM3);

	  /* Force update generation */
	  LL_TIM_GenerateEvent_UPDATE(TIM3);
}
/**
  * @brief  10uS delay
  * @param  None
  * @retval None
  */
void delay_us (uint16_t us)
{
	int i = 0;
	while(i<us)
	{
		if(is_10us_done2){
			is_10us_done2 = false;
			i++;
		}
	}
}


/**
  * @brief  Turn-on LED2.
  * @param  None
  * @retval None
  */
void LED_On(void)
{
  /* Turn LED2 on */
  LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
}

/**
  * @brief  Turn-off LED2.
  * @param  None
  * @retval None
  */
void LED_Off(void)
{
  /* Turn LED2 off */
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
}

/**
  * @brief  Set LED2 to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
  * @param  Period : Period of time (in ms) between each toggling of LED
  *   This parameter can be user defined values. Pre-defined values used in that example are :
  *     @arg LED_BLINK_FAST : Fast Blinking
  *     @arg LED_BLINK_SLOW : Slow Blinking
  *     @arg LED_BLINK_ERROR : Error specific Blinking
  * @retval None
  */
void LED_Blinking(uint32_t Period)
{
  /* Toggle LED2 in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    LL_mDelay(Period);
  }
}

void My_UsTick(void)
{
	if(++us_10_counter > 0)
	{
		is_10us_done = true;
		is_10us_done2 = true;
		us_10_counter = 0;
	}
	if(++us_20_counter > 1)
	{
		is_20us_done = true;
		us_20_counter = 0;
	}

	if(++us_80_counter > 7)
	{
		is_80us_done = true;
		us_80_counter = 0;
	}

}

/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART_CharReception_Callback(void)
{
__IO uint32_t received_char;

  /* Read Received character. RXNE flag is cleared by reading of DR register */
  received_char = LL_USART_ReceiveData8(USART1);

  /* Check if received value is corresponding to specific one : S or s */
  if ((received_char == 'S') || (received_char == 's'))
  {
    /* Turn LED2 On : Expected character has been received */
	  LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }

  /* Echo received character on TX */
  LL_USART_TransmitData8(USART1, received_char);
  LL_USART_TransmitData8(USART1, '\n');
}

/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void)
{
  __IO uint32_t sr_reg;

  /* Disable USARTx_IRQn */
 // NVIC_DisableIRQ(USART1);

  /* Error handling example :
    - Read USART SR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  sr_reg = LL_USART_ReadReg(USART1, SR);
  if (sr_reg & LL_USART_SR_NE)
  {
    /* case Noise Error flag is raised : ... */
    LED_Blinking(LED_BLINK_FAST);
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
    LED_Blinking(LED_BLINK_ERROR);
  }
}

/**
  * @brief  Function called for achieving next TX Byte sending
  * @param  None
  * @retval None
  */
void USART_TXEmpty_Callback(void)
{
  if(ubSend == (4 - 1))
  {
    /* Disable TXE interrupt */
    LL_USART_DisableIT_TXE(USART1);

    /* Enable TC interrupt */
    LL_USART_EnableIT_TC(USART1);
  }

  /* Fill DR with a new char */
  LL_USART_TransmitData8(USART1, dht_byts[ubSend++]);
}
/**
  * @brief  Function called at completion of last byte transmission
  * @param  None
  * @retval None
  */
void USART_CharTransmitComplete_Callback(void)
{
  if(ubSend == sizeof(dht_byts))
  {
    ubSend = 0;

    /* Disable TC interrupt */
    LL_USART_DisableIT_TC(USART1);

    /* Turn LED2 On at end of transfer : Tx sequence completed successfully */
    LED_On();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
