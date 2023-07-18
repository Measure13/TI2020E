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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <arm_math.h>
#include "USART_HMI.h"
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
static float32_t FFT_Res[MAX_DATA_NUM_FFT * 2];
static float32_t FFT_Mag[MAX_DATA_NUM_FFT];
static float32_t ADC_values_f[MAX_DATA_NUM_FFT];
static float32_t ADC_values_backup[MAX_DATA_NUM_FFT];
static float32_t FFT_Max_val;
static uint32_t FFT_Max_index;
static uint64_t ADC_values_sum = 0;
static float32_t THD = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Get_Data(void);
static void Data_Analysis(void);
static void Analyze_Distortion(void);
static inline uint32_t max_u32(uint32_t a, uint32_t b);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, max_u32(MAX_DATA_NUM_FFT + 4, MAX_DATA_NUM_SPC + 4) );
  Get_Data();
  Data_Analysis();
  printf("page0.t2.pco=2023\xff\xff\xff");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (new_setting)
    {
      new_setting = false;
      Get_Data();
      Data_Analysis();
      printf("page0.t2.pco=2023\xff\xff\xff");
    }
    
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void Get_Data(void)
{
  ADC_Get_Values(SAMPLE_RATE_FFT);
	ADC_values_sum = 0;
  
  arm_rfft_fast_instance_f32 S;
  arm_rfft_fast_init_f32(&S, MAX_DATA_NUM_FFT);
  for (uint16_t i = 0; i < MAX_DATA_NUM_FFT; ++i)
  {
    ADC_values_sum += adc_values[i + 4];
    ADC_values_f[i] = (float32_t)adc_values[i + 4];
  }
  float ADC_values_mean = (float)ADC_values_sum / MAX_DATA_NUM_FFT;
  for (uint16_t i = 0; i < MAX_DATA_NUM_FFT; ++i)
  {
    ADC_values_f[i] -= (float32_t)ADC_values_mean;
    ADC_values_f[i] *= 0.0008056640625f;
	ADC_values_backup[i] = ADC_values_f[i];
  }
  arm_rfft_fast_f32(&S, ADC_values_f, FFT_Res, 0);
  arm_cmplx_mag_squared_f32(FFT_Res, FFT_Mag, MAX_DATA_NUM_FFT);
  arm_max_f32(FFT_Mag, MAX_DATA_NUM_FFT / 2, &FFT_Max_val, &FFT_Max_index);

  ADC_Get_Values(SAMPLE_RATE_SPC);
  UARTHMI_Draw_ADC_Wave(0, adc_values + 4, MAX_DATA_NUM_SPC, 10);
}

static void Analyze_Distortion(void)
{
	float32_t max_val, min_val;
	uint32_t max_index, min_index;
	for (uint16_t i = 0; i < MAX_DATA_NUM_SPC; ++i)
  {
    ADC_values_backup[i] = (float32_t)adc_values[i];
  }
  arm_max_f32(ADC_values_backup, MAX_DATA_NUM_SPC, &max_val, &max_index);
  arm_min_f32(ADC_values_backup, MAX_DATA_NUM_SPC, &min_val, &min_index);
  
}

static void Data_Analysis(void)
{
  uint16_t ref_index;
  float32_t U1 = FFT_Max_val;
  float32_t U25[4];
  for (uint16_t i = 1; i < 4; ++i)
  {
    if (FFT_Mag[FFT_Max_index + i] > 0.25f * FFT_Max_val)
    {
      U1 += FFT_Mag[FFT_Max_index + i];
    }
	if (FFT_Mag[FFT_Max_index - i] > 0.25f * FFT_Max_val)
    {
      U1 += FFT_Mag[FFT_Max_index + i];
    }
  }
  
  for (uint16_t i = 2; i <= 5; ++i)
  {
    ref_index = i * FFT_Max_index;
	  U25[i - 2] = FFT_Mag[ref_index];
	for (uint16_t j = 1; j < 4; ++j)
	{
		if (FFT_Mag[ref_index + j] > 0.25f * FFT_Mag[ref_index])
		{
			U25[i - 2] += FFT_Mag[ref_index + j];
		}
		if (FFT_Mag[ref_index - j] > 0.25f * FFT_Mag[ref_index])
		{
			U25[i - 2] += FFT_Mag[ref_index + j];
		}
	}
  }
  arm_sqrt_f32((U25[0] + U25[1] + U25[2] + U25[3]) / U1, &THD);
  UARTHMI_Send_Float(0, THD);
}

static inline uint32_t max_u32(uint32_t a, uint32_t b)
{
	return (a > b) ? a : b;
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
    printf("ERROR!\n");
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
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
