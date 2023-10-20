/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables#define TIM2CLK 8000000   // STM Clock frequency
#define F_SIGNAL 50 // Frequency of output analog signal
#define NS 250
#define TIM2CLK 8000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs
uint32_t i = 0; // used to switch between lut_tables

uint32_t curr_millis = 0; // used in debounding

uint32_t Sin_LUT[NS] = {512,524,537,550,563,576,589,601,614,627,639,652,664,676,688,701,712,724,736,747,759,770,781,792,803,813,824,834,844,853,863,872,881,890,898,907,915,923,930,937,944,951,958,964,970,975,981,986,990,995,999,1003,1006,1009,1012,1014,1017,1019,1020,1021,1022,1023,1023,1023,1023,1022,1021,1019,1018,1016,1013,1011,1008,1004,1001,997,993,988,983,978,973,967,961,954,948,941,934,926,919,911,903,894,886,877,867,858,848,839,829,818,808,797,787,776,764,753,742,730,718,706,695,682,670,658,645,633,620,608,595,582,569,557,544,531,518,505,492,479,466,454,441,428,415,403,390,378,365,353,341,328,317,305,293,281,270,259,247,236,226,215,205,194,184,175,165,156,146,137,129,120,112,104,97,89,82,75,69,62,56,50,45,40,35,30,26,22,19,15,12,10,7,5,4,2,1,0,0,0,0,1,2,3,4,6,9,11,14,17,20,24,28,33,37,42,48,53,59,65,72,79,86,93,100,108,116,125,133,142,151,160,170,179,189,199,210,220,231,242,253,264,276,287,299,311,322,335,347,359,371,384,396,409,422,434,447,460,473,486,499,512};

uint32_t saw_LUT[NS] = {0,4,8,12,16,21,25,29,33,37,41,45,49,53,58,62,66,70,74,78,82,86,90,94,99,103,107,111,115,119,123,127,131,136,140,144,148,152,156,160,164,168,173,177,181,185,189,193,197,201,205,210,214,218,222,226,230,234,238,242,247,251,255,259,263,267,271,275,279,283,288,292,296,300,304,308,312,316,320,325,329,333,337,341,345,349,353,357,362,366,370,374,378,382,386,390,394,399,403,407,411,415,419,423,427,431,435,440,444,448,452,456,460,464,468,472,477,481,485,489,493,497,501,505,509,514,518,522,526,530,534,538,542,546,551,555,559,563,567,571,575,579,583,588,592,596,600,604,608,612,616,620,624,629,633,637,641,645,649,653,657,661,666,670,674,678,682,686,690,694,698,703,707,711,715,719,723,727,731,735,740,744,748,752,756,760,764,768,772,776,781,785,789,793,797,801,805,809,813,818,822,826,830,834,838,842,846,850,855,859,863,867,871,875,879,883,887,892,896,900,904,908,912,916,920,924,929,933,937,941,945,949,953,957,961,965,970,974,978,982,986,990,994,998,1002,1007,1011,1015,1019,1023};

uint32_t triangle_LUT[NS] = {0,8,17,25,33,41,50,58,66,74,83,91,99,107,116,124,132,140,149,157,165,173,182,190,198,206,215,223,231,239,248,256,264,272,281,289,297,305,314,322,330,338,347,355,363,371,380,388,396,404,413,421,429,437,446,454,462,470,479,487,495,503,512,520,528,536,545,553,561,569,578,586,594,602,611,619,627,635,644,652,660,668,677,685,693,701,710,718,726,734,743,751,759,767,776,784,792,800,809,817,825,833,842,850,858,866,875,883,891,899,908,916,924,932,941,949,957,965,974,982,990,998,1007,1015,1023,1023,1015,1007,998,990,982,974,965,957,949,941,932,924,916,908,899,891,883,875,866,858,850,842,833,825,817,809,800,792,784,776,767,759,751,743,734,726,718,710,701,693,685,677,668,660,652,644,635,627,619,611,602,594,586,578,569,561,553,545,536,528,520,512,503,495,487,479,470,462,454,446,437,429,421,413,404,396,388,380,371,363,355,347,338,330,322,314,305,297,289,281,272,264,256,248,239,231,223,215,206,198,190,182,173,165,157,149,140,132,124,116,107,99,91,83,74,66,58,50,41,33,25,17,8,0};

// : Equation to calculate TIM2_Ticks
uint32_t TIM2_Ticks = TIM2CLK/(F_SIGNAL*NS); // How often to write new LUT value
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  HAL_Init();

  /* USER CODE BEGIN Init */
  init_LCD();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // TODO: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1.
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

  // TODO: Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is TIM3->CCR3; start with Sine LUT&htim2

  HAL_DMA_Start_IT(&hdma_tim2_ch1, Sin_LUT, DestAddress, NS);

  // TODO: Write current waveform to LCD ("Sine")
  delay(3000);
  lcd_command(CLEAR);
  lcd_putstring("Sine");
  // TODO: Enable DMA (start transfer from LUT to CCR)
  __HAL_TIM_ENABLE_DMA(&htim2,TIM_DMA_CC1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_Ticks - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{

	// TODO: Debounce using HAL_GetTick()
static uint32_t last_millis = 0;
	static uint32_t delay_t = 50; // for controlling debouncing


curr_millis = HAL_GetTick();// getting the System Counter Value

	if((curr_millis - last_millis) >= 50){// preventing mechanical bouncing


	// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
	__HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
	HAL_DMA_Abort_IT(&hdma_tim2_ch1);

			if(i==0) // Sawtooth wave
			{
				lcd_command(CLEAR);
				lcd_putstring("Sawtooth");
				HAL_DMA_Start_IT(&hdma_tim2_ch1, saw_LUT, DestAddress, NS);
				__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
				i = 1; // So that next is triangle
			}
			else if(i==1) // Triangle wave
				{
					lcd_command(CLEAR);
					lcd_putstring("Triangle");
					HAL_DMA_Start_IT(&hdma_tim2_ch1, triangle_LUT, DestAddress, NS);
					__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
					i = 2; // So that next is sine
				}
			else if(i==2) // Sine Wave
				{
					lcd_command(CLEAR);
					lcd_putstring("Sine");
					HAL_DMA_Start_IT(&hdma_tim2_ch1, Sin_LUT , DestAddress, NS);
					__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
					i = 0; // So that next is sawtooth
				}
	}
			last_millis = curr_millis;
	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
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
