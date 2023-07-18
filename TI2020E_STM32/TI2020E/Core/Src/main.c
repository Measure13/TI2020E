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
static float32_t FFT_Res[MAX_DATA_NUM_FFT];
static float32_t FFT_Mag[MAX_DATA_NUM_FFT];
static float32_t ADC_values_f[MAX_DATA_NUM_FFT];
static float32_t ADC_values_backup[MAX_DATA_NUM_FFT];
static float32_t FFT_Max_val;
static uint32_t FFT_Max_index;
static uint64_t ADC_values_sum = 0;
static float32_t THD = 0.0f;
static const float32_t Hamming[] = {0.076720f, 0.076859f, 0.077276f, 0.077971f, 0.078943f, 0.080192f, 0.081717f, 0.083516f, 0.085590f, 0.087937f, 0.090555f, 0.093442f, 0.096598f, 0.100020f, 0.103706f, 0.107653f, 0.111860f, 0.116324f, 0.121042f, 0.126012f, 0.131230f, 0.136693f, 0.142398f, 0.148342f, 0.154520f, 0.160930f, 0.167567f, 0.174428f, 0.181507f, 0.188802f, 0.196307f, 0.204019f, 0.211931f, 0.220041f, 0.228342f, 0.236829f, 0.245499f, 0.254344f, 0.263361f, 0.272544f, 0.281887f, 0.291384f, 0.301030f, 0.310818f, 0.320744f, 0.330801f, 0.340983f, 0.351284f, 0.361698f, 0.372218f, 0.382838f, 0.393552f, 0.404353f, 0.415235f, 0.426191f, 0.437214f, 0.448299f, 0.459437f, 0.470623f, 0.481850f, 0.493111f, 0.504400f, 0.515708f, 0.527031f, 0.538360f, 0.549689f, 0.561012f, 0.572320f, 0.583609f, 0.594870f, 0.606097f, 0.617283f, 0.628421f, 0.639506f, 0.650529f, 0.661485f, 0.672367f, 0.683168f, 0.693882f, 0.704502f, 0.715022f, 0.725436f, 0.735737f, 0.745919f, 0.755976f, 0.765902f, 0.775690f, 0.785336f, 0.794833f, 0.804176f, 0.813359f, 0.822376f, 0.831221f, 0.839891f, 0.848378f, 0.856679f, 0.864789f, 0.872701f, 0.880413f, 0.887918f, 0.895213f, 0.902292f, 0.909153f, 0.915790f, 0.922200f, 0.928378f, 0.934322f, 0.940027f, 0.945490f, 0.950708f, 0.955678f, 0.960396f, 0.964860f, 0.969067f, 0.973014f, 0.976700f, 0.980122f, 0.983278f, 0.986165f, 0.988783f, 0.991130f, 0.993204f, 0.995003f, 0.996528f, 0.997777f, 0.998749f, 0.999444f, 0.999861f, 1.000000f, 0.999861f, 0.999444f, 0.998749f, 0.997777f, 0.996528f, 0.995003f, 0.993204f, 0.991130f, 0.988783f, 0.986165f, 0.983278f, 0.980122f, 0.976700f, 0.973014f, 0.969067f, 0.964860f, 0.960396f, 0.955678f, 0.950708f, 0.945490f, 0.940027f, 0.934322f, 0.928378f, 0.922200f, 0.915790f, 0.909153f, 0.902292f, 0.895213f, 0.887918f, 0.880413f, 0.872701f, 0.864789f, 0.856679f, 0.848378f, 0.839891f, 0.831221f, 0.822376f, 0.813359f, 0.804176f, 0.794833f, 0.785336f, 0.775690f, 0.765902f, 0.755976f, 0.745919f, 0.735737f, 0.725436f, 0.715022f, 0.704502f, 0.693882f, 0.683168f, 0.672367f, 0.661485f, 0.650529f, 0.639506f, 0.628421f, 0.617283f, 0.606097f, 0.594870f, 0.583609f, 0.572320f, 0.561012f, 0.549689f, 0.538360f, 0.527031f, 0.515708f, 0.504400f, 0.493111f, 0.481850f, 0.470623f, 0.459437f, 0.448299f, 0.437214f, 0.426191f, 0.415235f, 0.404353f, 0.393552f, 0.382838f, 0.372218f, 0.361698f, 0.351284f, 0.340983f, 0.330801f, 0.320744f, 0.310818f, 0.301030f, 0.291384f, 0.281887f, 0.272544f, 0.263361f, 0.254344f, 0.245499f, 0.236829f, 0.228342f, 0.220041f, 0.211931f, 0.204019f, 0.196307f, 0.188802f, 0.181507f, 0.174428f, 0.167567f, 0.160930f, 0.154520f, 0.148342f, 0.142398f, 0.136693f, 0.131230f, 0.126012f, 0.121042f, 0.116324f, 0.111860f, 0.107653f, 0.103706f, 0.100020f, 0.096598f, 0.093442f, 0.090555f, 0.087937f, 0.085590f, 0.083516f, 0.081717f, 0.080192f, 0.078943f, 0.077971f, 0.077276f, 0.076859f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Get_FFT_Data(void);
static void Get_Wave_Data(void);
static void Data_Analysis(void);
static void Analyze_Distortion(void);
static inline uint32_t max_u32(uint32_t a, uint32_t b);
static inline uint32_t min_u32(uint32_t a, uint32_t b);
static void merge_sort(float32_t* p, uint32_t len, bool ascending);
static float32_t Get_Median(float32_t* p, uint32_t len);
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
  UARTHMI_Forget_It();
  UARTHMI_Reset();
  HAL_Delay(150);
  Get_Wave_Data();
  Data_Analysis();
  Analyze_Distortion();
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
      Get_Wave_Data();
      Data_Analysis();
      Analyze_Distortion();
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
static void Get_FFT_Data(void)
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
  for (uint16_t i = 0; i < MAX_DATA_NUM_FFT; ++i)
  {
    ADC_values_f[i] *= Hamming[i];
  }
  arm_rfft_fast_f32(&S, ADC_values_f, FFT_Res, 0);
  arm_cmplx_mag_squared_f32(FFT_Res, FFT_Mag, MAX_DATA_NUM_FFT);
  arm_max_f32(FFT_Mag, MAX_DATA_NUM_FFT / 2, &FFT_Max_val, &FFT_Max_index);

}

static void Get_Wave_Data(void)
{
  ADC_Get_Values(SAMPLE_RATE_SPC);
  UARTHMI_Draw_ADC_Wave(0, adc_values + 4, MAX_DATA_NUM_SPC, 10);
}

static void Analyze_Distortion(void)
{
	float32_t max_val, min_val;
	uint32_t max_index, min_index, ADC_values_sum;
  ADC_Get_Values(SAMPLE_RATE_SPC);
	ADC_values_sum = 0;
  for (uint16_t i = 0; i < MAX_DATA_NUM_SPC; ++i)
  {
    ADC_values_sum += adc_values[i + 4];
    ADC_values_backup[i] = (float32_t)adc_values[i + 4];
  }
  float32_t ADC_values_mean = (float32_t)ADC_values_sum / MAX_DATA_NUM_SPC;
  for (uint16_t i = 0; i < MAX_DATA_NUM_SPC; ++i)
  {
    ADC_values_backup[i] -= ADC_values_mean;
    ADC_values_backup[i] *= 0.0008056640625f;
  }
  
  arm_max_f32(ADC_values_backup, MAX_DATA_NUM_SPC, &max_val, &max_index);
  arm_min_f32(ADC_values_backup, MAX_DATA_NUM_SPC, &min_val, &min_index);
  if (max_val + min_val > 0.2f)
  {
    UARTHMI_Send_Text(2, BOTTOM_DISTORTION);
  }
  else if (max_val + min_val < -0.2f)
  {
    UARTHMI_Send_Text(2, TOP_DISTORTION);
  }
  else
  {
    float32_t room[2 * TOP_SMOOTH_NUM];
    for (uint8_t i = 0; i < 2 * TOP_SMOOTH_NUM; ++i)
    {
      room[i] = fabs(ADC_values_backup[max_index - TOP_SMOOTH_NUM + i + 1] - ADC_values_backup[max_index - TOP_SMOOTH_NUM + i]);
    }
	float32_t med_temp = Get_Median(room, 2 * TOP_SMOOTH_NUM);
    if ((med_temp < ACCEPT_DIFF) && (med_temp > -ACCEPT_DIFF))
    {
      UARTHMI_Send_Text(2, BOTH_DISTORTION);
    }
    else
    {
      uint16_t i = 0;
      uint16_t cnt = 0;
	  uint16_t cycle = 0;
      while (cycle <= 3)
      {
        while ((i < 255) && (!((ADC_values_backup[i] - ADC_values_backup[i + 1] > 0.0f) && (ADC_values_backup[i + 1] - ADC_values_backup[i + 2] > 0.0f) && (ADC_values_backup[i + 2] - ADC_values_backup[i + 3] > 0.0f) && (ADC_values_backup[i] >= 0.4f))))
        {
          ++i;
        }
        while ((i < 255) && (ADC_values_backup[i] >= 0.15f))
        {
          ++i;
        }
        while ((i < 255) && (ADC_values_backup[i] >= -0.15f))
        {
          ++i;
          ++cnt;
        }
		++cycle;
      }
      if (cnt > CROSS_OVER_ZEROS)
      {
        UARTHMI_Send_Text(2, CO_DISTORTION);
      }
      else
      {
        UARTHMI_Send_Text(2, NO_DISTORTION);
      }
      
    }
    
  }
  
}

static void Data_Analysis(void)
{
  uint16_t ref_index;
  float32_t U1;
  float32_t U25[4];
  float32_t THD_Array[REMEASURE_NUM];
  for (uint8_t k = 0; k < REMEASURE_NUM; ++k)
  {
    Get_FFT_Data();
    U1 = FFT_Max_val;
    for (uint16_t i = 1; i < ACCEPT_CNT_RANGE; ++i)
    {
      if (FFT_Mag[FFT_Max_index + i] > ACCEPT_PROPORTION * FFT_Max_val)
      {
        U1 += FFT_Mag[FFT_Max_index + i];
      }
    if (FFT_Mag[FFT_Max_index - i] > ACCEPT_PROPORTION * FFT_Max_val)
      {
        U1 += FFT_Mag[FFT_Max_index - i];
      }
    }
    
    for (uint16_t i = 2; i <= 5; ++i)
    {
      ref_index = i * FFT_Max_index;
      U25[i - 2] = FFT_Mag[ref_index];
      for (uint16_t j = 1; j < ACCEPT_CNT_RANGE; ++j)
      {
        if (FFT_Mag[ref_index + j] > ACCEPT_PROPORTION * FFT_Mag[ref_index])
        {
          U25[i - 2] += FFT_Mag[ref_index + j];
        }
        if (FFT_Mag[ref_index - j] > ACCEPT_PROPORTION * FFT_Mag[ref_index])
        {
          U25[i - 2] += FFT_Mag[ref_index - j];
        }
      }
    }
    arm_sqrt_f32((U25[0] + U25[1] + U25[2] + U25[3]) / U1, &THD_Array[k]);
  }
  merge_sort(THD_Array, REMEASURE_NUM, true);
  THD = (THD_Array[REMEASURE_NUM / 2] + THD_Array[REMEASURE_NUM / 2 + 1]) / 2;
  UARTHMI_Send_Float(0, THD);
}

static inline uint32_t max_u32(uint32_t a, uint32_t b)
{
	return (a > b) ? a : b;
}

static inline uint32_t min_u32(uint32_t a, uint32_t b)
{
	return (a > b) ? b : a;
}

static void merge_sort(float32_t* p, uint32_t len, bool ascending)
{
    uint32_t left = 0, right = 0, left_limit, right_limit, i, room_len;
    for (uint32_t window = 1; window < len; window <<= 1)
    {
        for (uint32_t offset = 0; offset < len; offset += (window << 1))
        {
            left_limit = offset + window - 1;
            if (left_limit >= len)
            {
                break;
            }
            right_limit = min_u32(left_limit + window, len - 1);
            left = offset;
            right = left_limit + 1;
            room_len = right_limit - offset + 1;
            float32_t* room = (float32_t*)malloc(sizeof(float32_t) * room_len);
            i = 0;
            while (left <= left_limit && right <= right_limit)
            {
                room[i++] = ((p[left] <= p[right]) == ascending) ? p[left++] : p[right++];
            }
            while (left <= left_limit)
            {
                room[i++] = p[left++];
            }
            while (right <= right_limit)
            {
                room[i++] = p[right++];
            }
            for (uint32_t j = 0; j < room_len; ++j)
            {
                p[j + offset] = room[j];
            }
            free(room);
        }
    }
}
static float32_t Get_Median(float32_t* p, uint32_t len)
{
	merge_sort(p, len, true);
	if (len % 2)
	{
		return p[len / 2];
	}
	else
	{
		return (p[len / 2 - 1] + p[len / 2]) / 2;
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
