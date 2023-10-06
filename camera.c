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
#include "SEEKFREE_MT9V03X_CONFIG.h"
#include "SEEKFREE_IIC.h"
#include "SEEKFREE_MT9V03X.h"
#include <stdio.h>
#include <math.h>

extern UART_HandleTypeDef huart1;
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

#define col 188
#define row 120

void adapt_threshold(void);
void detect_seed(void);
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
/*视觉参数*/
uint8_t image_threshold[row][col] = {0};
uint8_t mode = 0;  // 0代表视觉运行，1代表电磁运行
uint8_t flag1 = 0; //入断路
uint8_t flag2 = 0; //出断路
uint8_t seed_col = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == VSY_Pin)
  {
    VSY_EXTI_Callback();
  }
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  mt9v03x_init();
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //	  if (mt9v03x_finish_flag) {
    //		  image_transmit();
    //		  mt9v03x_finish_flag = 0;
    //	  }
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
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim5)
  {
    /*图像处理*/

    if (mt9v03x_finish_flag)
    {

      /*第一步 优化大津法*/
      //算方差 阈值限制 二值化
      adapt_threshold();

      /*第二步 扫线 并判断是否出入断路*/
      detect_seed();
      image_transmit();
      mt9v03x_finish_flag = 0;
    }
  }
}

/*优化大津法*/
void adapt_threshold(void)
{

  uint16_t pixelCount[256] = {0};
  float pixelPro[256] = {0};
  uint16_t i, j = 0;
  uint16_t pixelSum = col * row / 4; // 间接取点
  uint8_t threshold = 0;
  uint32_t gray_sum = 0;
  float w0 = 0, w1 = 0, u0tmp = 0, u1tmp = 0, u0 = 0, u1 = 0, u = 0, deltaTmp = 0, deltaMax = 0;

  //灰度直方图数组
  //整体灰度 方便减少遍历
  for (i = 0; i < row; i += 2)
  {
    for (j = 0; j < col; j += 2)
    {
      pixelCount[mt9v03x_image[i][j]]++;
      gray_sum += mt9v03x_image[i][j];
    }
  }

  //灰度占比
  for (i = 0; i <= 255; i++)
  {
    pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }

  for (j = 0; j <= 255; j++)
  {
    //背景
    w0 += pixelPro[j];
    u0tmp += j * pixelPro[j];
    w1 = 1 - w0;
    u = (float)gray_sum / pixelSum;
    u1tmp = u - u0tmp;

    u0 = u0tmp / w0; //背景平均灰度
    u1 = u1tmp / w1;

    //    u = u0tmp + u1tmp; //全局平均灰度
    deltaTmp = w0 * (u0 - u) * (u0 - u) + w1 * (u1 - u) * (u1 - u);
    if (deltaTmp > deltaMax)
    {
      deltaMax = deltaTmp;
      threshold = j;
    }
    if (deltaTmp < deltaMax)
    {
      break;
    }
  }

  //阈值限制
  if (threshold < 40)
  {
    threshold = 50;
  }

  //二值化
  for (i = 0; i < row; i++)
  {
    for (j = 0; j < col; j++)
    {
      if (mt9v03x_image[i][j] > threshold)
      {
        image_threshold[i][j] = 255;
      }
      else
      {
        image_threshold[i][j] = 0;
      }
    }
  }
}

/*扫线 判断出入断路*/
void detect_seed()
{
  uint8_t seed_row;
  uint8_t seed_col;

  uint16_t seed_left[120] = {0};
  uint16_t seed_right[120] = {0};

  uint8_t last_col = 1;
  uint8_t left_cnt = 0;
  uint8_t right_cnt = 0;

  /*扫线 sao xian*/
  for (seed_row = row - 1; seed_row > 0; seed_row--)
  {
    if (seed_row == 119)
    {
      seed_col = 94;
    }
    else
    {
      seed_col = last_col;
    }

    // left
    for (left_cnt = seed_col; left_cnt > 0; left_cnt--)
    {
      uint8_t left_delta = image_threshold[seed_row][left_cnt] - image_threshold[seed_row][left_cnt - 1];

      if (left_delta > 200)
      {
        seed_left[seed_row] = left_cnt;
        break;
      }
      else
      {
        seed_left[seed_row] = 1;
      }
    }

    // right
    for (right_cnt = seed_col; right_cnt < 187; right_cnt++)
    {
      int right_delta = image_threshold[seed_row][right_cnt] - image_threshold[seed_row][right_cnt + 1];
      if (right_delta > 200)
      {
        seed_right[seed_row] = right_cnt;
        break;
      }
      else
      {
        seed_right[seed_row] = 186;
      }
    }

    if (seed_row == 126)
    {
      seed_col = (uint8_t)((seed_left[seed_row] + seed_right[seed_row]) / 2);
    }
    last_col = (uint8_t)((seed_left[seed_row] + seed_right[seed_row]) / 2);
    if (last_col < 1)
    {
      last_col = 1;
    }
    else if (last_col > 186)
    {
      last_col = 186;
    }

    if (row > 63 && row < 126 && (seed_right[seed_row] - seed_left[seed_row]) < 5 && flag1 == 0 && seed_left[seed_row] > 10 && seed_right[seed_row] < 170)
    {
      flag1 = 5; //已入断路
      break;
    }
  }

  if (flag1 == 1)
  {
    // var txt='Hello world';
    for (int i = 118; i > 110; i--)
    {
      if (image_threshold[i][94] != 0)
      {
        // road_break = 0;
        flag2 = 1; //必须连续八次中点为白，则判断已经出断路
      }
      else
      {
        // road_break = 1;
        flag2 = 0; //但凡有一行中点不是白，就认为没有出断路
        break;
      }
    }
  }
  else if (flag1 > 1)
  {
    flag1--;
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
