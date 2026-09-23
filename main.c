/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define ARM_MATH_CM4
#include "kalman_filter.h"
#include "kalman_filter_cmsis.h"
#include "kalman_filter_c.h"
#include "arm_math.h"
#include <stdlib.h>

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
#define TEST_LEN 100
float input_array[TEST_LEN];
float output_asm[TEST_LEN];
float output_c[TEST_LEN];
float output_cmsis[TEST_LEN];

float diff[TEST_LEN];
float diff_cmsis[TEST_LEN];
float corr[2*TEST_LEN - 1];
float corr_cmsis[2*TEST_LEN - 1];
float conv[2*TEST_LEN - 1];
float conv_cmsis[2*TEST_LEN - 1];
float mean_val, std_val;
float mean_cmsis, std_cmsis;

int status_asm, status_c, status_cmsis;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Enables the CPU cycle counter (DWT) for profiling
void EnableCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

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
  /* USER CODE BEGIN 2 */

  EnableCycleCounter();


  // 1. Declare and initialize a state struct, with variables set to 0.

  kalman_state test_state;
  test_state.q = 0.1f;
  test_state.r = 0.1f;
  test_state.x = 5.0f;
  test_state.p = 0.1f;
  test_state.k = 0.0f;

  kalman(&test_state, 0.0f);   // after this: p=0.067, x=1.67, k=0.67
  kalman(&test_state, 1.0f);   // after this: p=0.0625, x=1.25, k=0.625
  kalman(&test_state, 2.0f);   // after this: p=0.06, x=1.71, k=0.62
  kalman(&test_state, 3.0f);   // after this: p=0.06, x=2.5, k=0.62




  // to test the CMSIS stats functions, can directly call them from here using:
  // arm_sub_f32()
  // arm_mean_f32() + arm_std_f32()
  // arm_conv_f32()
  // arm_correlate_f32()

  //PROFILING: ACTUAL KALMAN FILTER
  for (int i = 0; i < TEST_LEN; i++)
  {
      float true_signal = 10.0f * sinf(2.0f * PI * i / 20.0f); // slow sine wave
      float noise = ((float)(rand() % 200) - 100.0f) / 100.0f;  // noise in [-1, 1]
      input_array[i] = true_signal + noise;
  }
  kalman_state state_asm = {0.1f, 0.1f, 5.0f, 0.1f, 0.0f};
  kalman_state state_c    = {0.1f, 0.1f, 5.0f, 0.1f, 0.0f};
  kalman_state state_cmsis = {0.1f, 0.1f, 5.0f, 0.1f, 0.0f};

  uint32_t t0, t_asm, t_c, t_cmsis;

  t0 = DWT->CYCCNT;
  status_asm = Kalmanfilter(input_array, output_asm, &state_asm, TEST_LEN);         // your Part 1 assembly-backed version
  t_asm = DWT->CYCCNT - t0;

  t0 = DWT->CYCCNT;
  status_c = Kalmanfilter_c(input_array, output_c, &state_c, TEST_LEN);           // your plain-C version
  t_c = DWT->CYCCNT - t0;

  t0 = DWT->CYCCNT;
  status_cmsis = Kalmanfilter_cmsis(input_array, output_cmsis, &state_cmsis, TEST_LEN); // your CMSIS version
  t_cmsis = DWT->CYCCNT - t0;


  //PROFILING: 4 STATS RUNS

  t0 = DWT->CYCCNT;
  DataSubtraction(input_array, output_asm, diff, TEST_LEN);
  uint32_t t_sub_c = DWT->CYCCNT - t0;

  t0 = DWT->CYCCNT;
  CalculateMeanStd(diff, TEST_LEN, &mean_val, &std_val);
  uint32_t t_meanstd_c = DWT->CYCCNT - t0;

  t0 = DWT->CYCCNT;
  Correlation(input_array, TEST_LEN, output_asm, TEST_LEN, corr);
  uint32_t t_corr_c = DWT->CYCCNT - t0;

  t0 = DWT->CYCCNT;
  Convolution(input_array, TEST_LEN, output_asm, TEST_LEN, conv);
  uint32_t t_conv_c = DWT->CYCCNT - t0;

//CMSIS

  t0 = DWT->CYCCNT;
  arm_sub_f32(input_array, output_asm, diff_cmsis, TEST_LEN);
  uint32_t t_sub_cmsis = DWT->CYCCNT - t0;

  t0 = DWT->CYCCNT;
  arm_mean_f32(diff_cmsis, TEST_LEN, &mean_cmsis);
  arm_std_f32(diff_cmsis, TEST_LEN, &std_cmsis);
  uint32_t t_meanstd_cmsis = DWT->CYCCNT - t0;

  t0 = DWT->CYCCNT;
  arm_correlate_f32(input_array, TEST_LEN, output_asm, TEST_LEN, corr_cmsis);
  uint32_t t_corr_cmsis = DWT->CYCCNT - t0;

  t0 = DWT->CYCCNT;
  arm_conv_f32(input_array, TEST_LEN, output_asm, TEST_LEN, conv_cmsis);
  uint32_t t_conv_cmsis = DWT->CYCCNT - t0;


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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
