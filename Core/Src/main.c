/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "LCD1602.h"
#include "motor.h"

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t En_Counter = 0;
volatile uint8_t Position_Cur = 0;
volatile uint8_t STOP_MOTOR = 0;
volatile uint8_t flag_irq = 0;
volatile uint32_t time_irq = 0;

volatile uint32_t STPS = 1600;
volatile uint32_t DIR = 1;
volatile uint32_t TIME = 1;
volatile uint8_t FLAG = 0;
volatile uint8_t FLAG1 = 1;
volatile uint16_t enpos = 0;
//
volatile uint8_t flag1 = 1;
volatile uint8_t flag2 = 1;
volatile uint8_t flag3 = 0;
volatile uint32_t time1 = 1;
volatile uint32_t time11 = 1;

//volatile uint8_t KN_LEFT = 0;
//volatile uint8_t KN_RIGHT = 0;
volatile uint8_t KN = 0;
volatile uint8_t flag_usk = 1;

volatile uint8_t flag_test = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { //Прерывание. Р

	if (GPIO_Pin == KN2_Pin) { // остановка или запуск двигателя
		NVIC->ICER[1] = (1 << (EXTI15_10_IRQn & 0x1F));
		time1 = HAL_GetTick();
		flag2 = 1;
	}
	if (GPIO_Pin == KN1_Pin) {
		NVIC->ICER[1] = (1 << (EXTI15_10_IRQn & 0x1F)); //абота в меню
		time11 = HAL_GetTick();
		flag1 = 1;

	}
}


void ext() { // Работа с прерываниями
	if (flag1 && HAL_GetTick() - time11 > 400
			&& HAL_GPIO_ReadPin(GPIOB, KN1_Pin) == 0) { //прерывание. установка курсора в меню
		Position_Cur++;
		Position_Cur = Position_Cur % 2;
		flag1 = 0;

		if (Position_Cur == 1) {
			lcd_put_cur(0, 7);
			lcd_send_string("  ");
			lcd_put_cur(1, 7);
			lcd_send_string(" _");
		} else {
			lcd_put_cur(1, 7);
			lcd_send_string("  ");
			lcd_put_cur(0, 7);
			lcd_send_string(" _");
		}
		NVIC->ISER[1] = (1 << (EXTI15_10_IRQn & 0x1F));

	}
	if (flag2 && HAL_GetTick() - time1 > 400 // прерывание. остановка или запуск двигателя
	&& HAL_GPIO_ReadPin(GPIOB, KN2_Pin) == 0) {
		STOP_MOTOR++;
		STOP_MOTOR = STOP_MOTOR % 2;
		flag2 = 0;
		NVIC->ISER[1] = (1 << (EXTI15_10_IRQn & 0x1F));
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_TIM4_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_GPIO_WritePin(GPIOA, LCD_LED_Pin, GPIO_PIN_SET);
	HAL_TIM_Base_Start(&htim4);
	lcd_init();
	HAL_Delay(100);
	lcd_put_cur(0, 0);
	HAL_Delay(100);
	HAL_Delay(1000);
	lcd_send_string("HELLO");
	HAL_Delay(1000);
	lcd_clear();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	TIM1->ARR = 5000;
	char buff[33];
	int32_t SPEED_RPM = 10; //Скорость вращения двигателя в оборотах в минуту
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	Init_Pins(GPIOB, ENA_Pin, GPIOA, STP_Pin, GPIOA, DIR_Pin); // Инициализация пинов
// ============================================================================
	lcd_send_cmd(0b00001100);
	HAL_Delay(10);
	lcd_put_cur(0, 0);
	lcd_send_string("DIR:"); //направление
	if (DIR == 0) {
		lcd_send_string("CCW");
	} else {
		lcd_send_string("CW ");
	}
// ============================================================================
	HAL_Delay(5);
	lcd_put_cur(1, 0);
	lcd_send_string("RPM:");
	HAL_Delay(10);
	lcd_send_string(itoa(SPEED_RPM, buff, 10));
	uint32_t enc_previous = 0;
	uint32_t time_of_delay = 0;
	lcd_put_cur(0, 7);
	lcd_send_string(" _");
	HAL_Delay(100);
	HAL_GPIO_EXTI_Callback(KN1_Pin);
// ==============================================================================
	void MENU_SET(uint8_t enc_pos, uint32_t Delay) {

		if (enc_pos != enc_previous && HAL_GetTick() - time_of_delay >= Delay) {
			switch (Position_Cur) {
			case 0:
				if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) {
					DIR = 1;
				} else {
					DIR = 0;
				}
				break;

			case 1:
				if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) {
					if (HAL_GPIO_ReadPin(GPIOB, ENC_KN_Pin) == GPIO_PIN_RESET) {
						SPEED_RPM = SPEED_RPM - 10;
						if (SPEED_RPM <= 0) {
							SPEED_RPM = 1;
						}

					}
					if (HAL_GPIO_ReadPin(GPIOB, ENC_KN_Pin) == GPIO_PIN_SET) {
						SPEED_RPM = SPEED_RPM - 1;
						if (SPEED_RPM <= 0) {
							SPEED_RPM = 1;
						}
					}

				} else {
					if (HAL_GPIO_ReadPin(GPIOB, ENC_KN_Pin) == GPIO_PIN_RESET) {
						SPEED_RPM = SPEED_RPM + 10;
					}
					if (HAL_GPIO_ReadPin(GPIOB, ENC_KN_Pin) == GPIO_PIN_SET) {
						SPEED_RPM = SPEED_RPM + 1;
					}
				}
				break;
			}
			enc_previous = enc_pos;
			HAL_Delay(1);
			if (Position_Cur == 0) {

				lcd_put_cur(0, 4);
				if (DIR == 0) {
					lcd_send_string("CW ");
				} else {
					lcd_send_string("CCW");
				}

			}

			if (Position_Cur == 1) {

				lcd_put_cur(1, 4);
				lcd_send_string("   ");
				lcd_put_cur(1, 4);
				lcd_send_string(itoa(SPEED_RPM, buff, 10));
			}
			time_of_delay = HAL_GetTick();
		}
	}
// =============================================================================
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if (STOP_MOTOR == 0) { //РАБОТА В МЕНЮ

// ==================================================================
//			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

			ext();
			lcd_put_cur(1, 9);
			lcd_send_string("    ");
			lcd_put_cur(1, 9);
			lcd_send_string("OFF");
			MENU_SET(TIM1->CNT / 2, 0);
//			time1 = HAL_GetTick()

		}
		if (STOP_MOTOR == 1) {
			lcd_put_cur(1, 9);
			lcd_send_string("    ");
			HAL_Delay(100);
			lcd_put_cur(1, 9);
			lcd_send_string("ON");
			flag_usk = 1;
			while (1) {

				Motor_On();
				Speed(SPEED_RPM);
				MOTOR_Direction(DIR);
				ext();
				Steps(10);
				MENU_SET(TIM1->CNT / 2, 300);

				if (flag_test == 1) {
					flag_test = 0;
					break;
				}
			}

		}
	}
}

/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}

	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */

static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 72 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0xffff - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			LCD_LED_Pin | LCD_RS_Pin | LCD_RW_Pin | LCD_E_Pin | LCD_D4_Pin
					| LCD_D5_Pin | LCD_D6_Pin | LCD_D7_Pin | DIR_Pin | STP_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_LED_Pin LCD_RS_Pin LCD_RW_Pin LCD_E_Pin
	 LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin
	 DIR_Pin STP_Pin */
	GPIO_InitStruct.Pin = LCD_LED_Pin | LCD_RS_Pin | LCD_RW_Pin | LCD_E_Pin
			| LCD_D4_Pin | LCD_D5_Pin | LCD_D6_Pin | LCD_D7_Pin | DIR_Pin
			| STP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ENC_KN_Pin */
	GPIO_InitStruct.Pin = ENC_KN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(ENC_KN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : KN2_Pin KN1_Pin */
	GPIO_InitStruct.Pin = KN2_Pin | KN1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : LEFT_Pin RIGHT_Pin */
	GPIO_InitStruct.Pin = LEFT_Pin | RIGHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ENA_Pin */
	GPIO_InitStruct.Pin = ENA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ENA_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
