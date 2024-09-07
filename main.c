/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFFER_SIZE 20
#define BUFFER_SIZE 15
#define CURRENT_MAX 15
#define VOLTAGE_IDLE 2.444
#define VOLTAGE_DELTA_LOWERBOUND -0.05// lowerbound is at 0.5Nm CCW
#define VOLTAGE_DELTA_UPPERBOUND 0.057// upperbound is at 0.5Nm CW
#define TORQUE_AMIN 1.0
#define TORQUE_AMAX 9.0
#define K_V 1.5
#define ZERO_SPEED_DUTYCYCLE 0.375
#define DEVIATION 0.125

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void vTorqueSensorReadTask(void* pvParameters);
void vCurrentSensorReadTask(void* pvParameters);
//void vPIDControlTask(void* pvParameters);
void vPWMControlTask(void* pvParameters);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;

/* Definitions for defaultTask */
/* USER CODE BEGIN PV */

float currentBuffer[BUFFER_SIZE] = {0};
uint8_t bufferIndex = 0;
float currentSum = 0;

float torque = 0;
float current_feedback = 0;


float adc_value;
float current_nofilter;
float voltage_value_current;
float adc_buffer[ADC_BUFFER_SIZE] = {0};
float sum = 0;
uint8_t adc_index = 0;
float average = 0;
float voltage = 0;
uint8_t valid_entries = 0;
uint8_t valid_entries_current = 0;
float voltage_delta = 0;
float current_assist_percent = 0.0;
float current_setpoint = 0.0;
float current_error = 0.0;
float current_error_ratio = 0.0;
float current_ratio = 0.0;

float pwm_duty_cycle = 0.0;
//PID
float prev_error = 0.0;
float integral = 0.0;
float derivative = 0.0;
float dt = 0.004; //timestep
//PID gains:
float kp = 0.95;
float ki = 0;
float kd = 0.0007;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // Zero Speed (1500*3000)/4000
//  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 750);  // Full Reverse Speed (1000*3000)/4000
//  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1500); // Full Forward Speed (2000*3000)/4000


  xTaskCreate(vTorqueSensorReadTask,
		  	  "Torque Sensor Read",
			  1000,
			  NULL,
			  3,
			  NULL);

  xTaskCreate(vCurrentSensorReadTask,
		  	  "Current Sensor Read",
			  2000,
			  NULL,
			  2,
			  NULL);

//  xTaskCreate(vPIDControlTask,
//		  	  "PID Control",
//			  1000,
//			  NULL,
//			  2,
//			  NULL);
//
  xTaskCreate(vPWMControlTask,
              "PWM Control",
              1000,  // Increased stack size
              NULL,
              1,     // Higher priority
              NULL);

  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Init scheduler */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */


  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void vTorqueSensorReadTask(void* pvParameters){
	while(1){
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, 1000);
		  adc_value = HAL_ADC_GetValue(&hadc1);
		  sum -= adc_buffer[adc_index];
		  adc_buffer[adc_index] = adc_value;
		  sum += adc_value;
		  adc_index = (adc_index + 1) % ADC_BUFFER_SIZE;
		  if (valid_entries < ADC_BUFFER_SIZE) {
			  valid_entries++;
		  }
		  average = sum / valid_entries;
		  voltage = average*(3.35/4095.0);

		  voltage_delta = voltage - VOLTAGE_IDLE;
		  //voltage_delta > 0: Clockwise
		  //voltage_delta < 0: CounterClockwise

		  if(adc_value <= 400 && adc_value >= 0){
			torque = 0;
		  } else {
			  if(voltage_delta < VOLTAGE_DELTA_LOWERBOUND){
				torque = -8.4096*voltage + 20.915;
			  } else if ( voltage_delta > VOLTAGE_DELTA_UPPERBOUND){
				torque = 15.886*voltage - 39.401;
			  } else {
				torque = 0;
			  }
		  }

		  HAL_ADC_Stop(&hadc1);
		// Delay to simulate sensor read interval
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}


void vCurrentSensorReadTask(void* pvParameters){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	__HAL_TIM_SET_AUTORELOAD(&htim1, 2999);
	while(1){
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, 1000);
		voltage_value_current = (float)(HAL_ADC_GetValue(&hadc2)*25)/(3*4095);
		current_nofilter = (float)fabs((voltage_value_current - 2.5)/(0.04));
		// Subtract the oldest value from the sum
		currentSum -= currentBuffer[bufferIndex];
		// Add the new value to the buffer and sum
		currentBuffer[bufferIndex] = current_nofilter;
		currentSum += current_nofilter;
		// Update the buffer index
	    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
		if (valid_entries_current < BUFFER_SIZE) {
			valid_entries_current++;
		}
	    current_feedback = currentSum / valid_entries_current;


//		current = current_nofilter;


		HAL_ADC_Stop(&hadc2);
		//Assistant Torque Control
		if(torque >= TORQUE_AMAX){
			current_assist_percent = 1.0;
		} else if (torque <= TORQUE_AMIN){
			current_assist_percent = 0.0;
		} else {
			current_assist_percent = (K_V*(torque - TORQUE_AMIN))/TORQUE_AMAX;
		}
		// Saturation current_assist_percent between 0 and 1
		if(current_assist_percent < 0.0){
			current_assist_percent = 0.0;
		} else if (current_assist_percent > 1.0){
			current_assist_percent = 1.0;
		}
		// Calculate Current setpoint
		current_setpoint = current_assist_percent*(CURRENT_MAX); //unit : A
//		current_setpoint = 0;
		// Calculate current error
		current_error = current_setpoint - current_feedback;
//		if(current_error>=0 && current_error<2){
//			current_error = 0;
//		}

		/***************************PID************************************/
		//Discrete PID
		integral += current_error*dt;
		derivative = (current_error-prev_error)/dt;

		current_ratio = (kp*current_error + ki*integral + kd*derivative)/CURRENT_MAX;
		prev_error = current_error;
		/*******************************************************************/

		// Saturation current_ratio between 0 and 1
		if(current_ratio < 0.0){
			current_ratio = 0.0;
		} else if (current_ratio > 1.0){
			current_ratio = 1.0;
		}




        // Delay to simulate sensor read interval
        vTaskDelay(pdMS_TO_TICKS(10));
	}
}

//void vPIDControlTask(void* pvParameters){
//	while(1){
////		/***************************PID************************************/
////		//Discrete PID
////		integral += current_error*dt;
////		derivative = (current_error-prev_error)/dt;
////
////		current_ratio = (kp*current_error + ki*integral + kd*derivative)/CURRENT_MAX;
////		prev_error = current_error;
////		/*******************************************************************/
////
////		// Saturation current_ratio between 0 and 1
////		if(current_ratio < 0.0){
////			current_ratio = 0.0;
////		} else if (current_ratio > 1.0){
////			current_ratio = 1.0;
////		}
//
//        // Delay to allow other tasks to run
//        vTaskDelay(pdMS_TO_TICKS(10));
//	}
//}

void vPWMControlTask(void* pvParameters){
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//	__HAL_TIM_SET_AUTORELOAD(&htim1, 2999);
	while(1){
		// PRE PWM & PWM
		if(voltage_delta >= 0.0){
			pwm_duty_cycle = ZERO_SPEED_DUTYCYCLE+(DEVIATION*current_ratio);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(pwm_duty_cycle*__HAL_TIM_GET_AUTORELOAD(&htim1)));
		} else {
			pwm_duty_cycle = ZERO_SPEED_DUTYCYCLE-(DEVIATION*current_ratio);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(pwm_duty_cycle*__HAL_TIM_GET_AUTORELOAD(&htim1)));
		}

        // Delay to allow other tasks to run
        vTaskDelay(pdMS_TO_TICKS(10));
	}
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
