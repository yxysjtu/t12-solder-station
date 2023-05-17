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
#include "adc.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);
uint8_t OLED_Init(void);
uint8_t OLED_CLS(void);
uint8_t OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N);

uint32_t cnt = 0;
int32_t pwm_max = 1000, temp_t = 50, pwm = 500;
int16_t temp_set = 300, temp_real, state = 1; //0:sleep; 1:awake
uint16_t measure_temp, measure_vibration, measure_temp_set;
float kp = 60, ki = 2, kd = 0;
float err, prev_err = 0, sum_err = 0;

int width = 16, offset = 14;
int teststate = 0;
#define oled_shownum(x,y,n) if(OLED_ShowCN(x,y,n)){HAL_I2C_MspDeInit(&hi2c1);HAL_I2C_MspInit(&hi2c1);MX_I2C1_Init();if(OLED_Init()){teststate++;}else {/*OLED_CLS();*/teststate = 0;}return;}
void oled_show(){
	//temp_real / temp_set
	int num = temp_real;
	if(state){
		oled_shownum(width * 6 + offset, 2, num / 100);
		oled_shownum(width * 5 + offset, 2, (num / 10) % 10);
		oled_shownum(width * 4 + offset, 2, num % 10);
	}else{
		oled_shownum(width * 7, 2, 11);
		oled_shownum(width * 6, 2, 11);
		oled_shownum(width * 5, 2, 11);
	}
	
	oled_shownum(54, 2, 10);
	
	num = temp_set;
	oled_shownum(32, 2, num / 100);
	oled_shownum(16, 2, (num / 10) % 10);
	//oled_shownum(0, 2, 0);
	oled_shownum(0, 2, num % 10);
	
}

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	OLED_CLS();

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val, 3);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    HAL_IWDG_Refresh(&hiwdg);
		oled_show();
	  
	  	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);//对GPIO口的电平进行反转（低-高，高—低）
		HAL_Delay(500);
	  
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void ADC_Select_CH (uint32_t channel)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = channel;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}


void adc_scan(){
	ADC_Select_CH(ADC_CHANNEL_1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	int temp_val = HAL_ADC_GetValue(&hadc1);
	if(temp_val < 3800){
		measure_temp = measure_temp * 0.95 + HAL_ADC_GetValue(&hadc1) * 0.05;
	}
	HAL_ADC_Stop(&hadc1);
	ADC_Select_CH(ADC_CHANNEL_2);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	measure_vibration = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	ADC_Select_CH(ADC_CHANNEL_3);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	measure_temp_set = measure_temp_set * 0.95 + HAL_ADC_GetValue(&hadc1) * 0.05;
	HAL_ADC_Stop(&hadc1);
}
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
	HAL_IWDG_Refresh(&hiwdg);
	if(htim == &htim2){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			cnt++;
			adc_scan();
			//calculate temperature
			temp_set = (float)measure_temp_set / 4096 * 200 + 200;
			temp_real = measure_temp * 0.1238 + 17.2814;
			//pid=>pwm
			err = temp_set - temp_real;
			if(err < 10) sum_err += err;
			if(sum_err > 100) sum_err = 100;
			else if(sum_err < -100) sum_err = -100;
			if(err <= 0) pwm = 0;
			else pwm = kp * err + kd * (err - prev_err) + ki * sum_err;
			if(pwm > 1000) pwm = 1000;
			if(pwm < 0) pwm = 0;
			if(state == 0) pwm = 0;
			prev_err = err;
			//pwm = (float)measure_temp_set / 4096 * 1800;
			TIM2->CCR1 = pwm;
			
			//sense vibration
			if(measure_vibration > 1000){ //threshold
				cnt = 0;
				state = 1;
			}else if(cnt > 20000){ //1 min no movement
				state = 0;
			}

		}
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
