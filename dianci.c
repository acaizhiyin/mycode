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
#include "stdio.h"
#include "math.h"

extern UART_HandleTypeDef huart1;
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

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
int16_t measureval_l = 0, measureval_r = 0, delta_speed = 0, ref_speed_l = 600, ref_speed_r = 600;
int pwm_l = 0, pwm_r = 0;
float err_l = 0, last_err_l = 0, err_sum_l = 0, err_r = 0, err_sum_r = 0, last_err_r = 0;
float kp_l = 15.5, ki_l = 0.455, kd_l = 0.125, rp_l = 0, ri_l = 0, rd_l = 0; //左轮
float kp_r = 15.5, ki_r = 0.49, kd_r = 0.15, rp_r = 0, ri_r = 0, rd_r = 0;   //右轮

uint16_t inductance_val[3] = {0}, adc_val_l[10] = {0}, adc_val_m[10] = {0}, adc_val_r[10] = {0}, sum_l = 0, sum_m = 0, sum_r = 0;
uint16_t val_max_l = 1190, val_min_l = 0, val_max_m = 900, val_min_m = 0, val_max_r = 1190, val_min_r = 0;
float bias = 0, last_bias = 0;
float position_kp = 8, position_kd = 400, position_rp = 0, position_rd = 0, last_rd = 0, val_l = 0, val_r = 0, val_m = 0, km = 5;
float position_alpha = 0.2; //不完全微分系数
//	float position_gama = 0.2；//微分先行滤波系数
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
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
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
    //		if (mt9v03x_finish_flag) {
    //
    //			mt9v03x_finish_flag = 0;
    //		}
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

    // position
    uint8_t i, j;
    for (i = 0; i < 10; i++)
    {
      for (uint8_t n = 0; n < 3; n++)
      {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 50);
        inductance_val[n] = HAL_ADC_GetValue(&hadc1);
      }
      adc_val_l[i] = inductance_val[0];
      adc_val_m[i] = inductance_val[1];
      adc_val_r[i] = inductance_val[2];
    }

    //中值平均滤波
    uint16_t temp;
    //左电感
    for (j = 0; j < 9; j++)
    {
      for (i = 0; i < 9 - j; i++)
      {
        if (adc_val_l[i] > adc_val_l[i + 1])
        {
          temp = adc_val_l[i];
          adc_val_l[i] = adc_val_l[i + 1];
          adc_val_l[i + 1] = temp;
        }
      }
    }

    for (i = 1; i < 9; i++)
    {
      sum_l += adc_val_l[i];
    }
    val_l = (float)sum_l / 8;
    sum_l = 0;
    temp = 0;

    //中间电感
    for (j = 0; j < 9; j++)
    {
      for (i = 0; i < 9 - j; i++)
      {
        if (adc_val_m[i] > adc_val_m[i + 1])
        {
          temp = adc_val_m[i];
          adc_val_m[i] = adc_val_m[i + 1];
          adc_val_m[i + 1] = temp;
        }
      }
    }

    for (i = 1; i < 9; i++)
    {
      sum_m += adc_val_m[i];
    }
    val_m = (float)sum_m / 8;
    sum_m = 0;
    temp = 0;

    //右电感
    for (j = 0; j < 9; j++)
    {
      for (i = 0; i < 9 - j; i++)
      {
        if (adc_val_r[i] > adc_val_r[i + 1])
        {
          temp = adc_val_r[i];
          adc_val_r[i] = adc_val_r[i + 1];
          adc_val_r[i + 1] = temp;
        }
      }
    }

    for (i = 1; i < 9; i++)
    {
      sum_r += adc_val_r[i];
    }
    val_r = (float)sum_r / 8;
    sum_r = 0;
    temp = 0;

    bias = 10000 * (val_l - val_r) / ((val_l + val_r) * val_m * km); //差比和

    //		printf("%f,%d,%d,%d,%d,%d,%d\n",bias,delta_speed,flag,ref_speed_l,ref_speed_r,measureval_l,measureval_r);

    //位置环pid
    if (bias < 1 && bias > -1)
    {
      bias *= 0.1f;
    }
    else if ((bias < 2 && bias > 1) || (bias < -1 && bias > -2))
    {
      bias *= 0.425f;
    }
    position_rp = position_kp * bias;
    position_rd = position_kd * (1 - position_alpha) * (bias - last_bias) + position_alpha * last_rd; //不完全微分
    delta_speed = (int16_t)(position_rp + position_rd);
    if (delta_speed > 200)
      delta_speed = 200;
    if (delta_speed < -200)
      delta_speed = -200;
    last_bias = bias;
    last_rd = position_rd;

    //左轮
    measureval_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    ref_speed_l = ref_speed_l - delta_speed / 2;

    if (ref_speed_l > 650)
      ref_speed_l = 650;
    else if (ref_speed_l < 0)
      ref_speed_l = 0;
    err_l = ref_speed_l - measureval_l;
    err_sum_l += err_l;
    if (err_sum_l > 80000)
      err_sum_l = 80000;
    if (err_sum_l < -80000)
      err_sum_l = -80000;
    //抗积分饱和
    //		if (err_sum_l > 80000)
    //		{
    //			if(err_l <= 0)err_sum_l += err_l;
    //
    //		}
    //		else if(err_sum_l < -80000)
    //		{
    //			if(err_l >= 0)err_sum_l += err_l;
    //		}
    //		else
    //		{
    //			err_sum_l += err_l;
    //		}
    //梯形积分，将err_l换为(err_l + last_err_l) / 2即可
    rp_l = kp_l * err_l;
    ri_l = ki_l * err_sum_l;
    rd_l = kd_l * (err_l - last_err_l);
    pwm_l = rp_l + ri_l + rd_l;
    if (pwm_l > 0)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
      if (pwm_l > 8000)
        TIM4->CCR3 = 8000;
      else
      {
        TIM4->CCR3 = pwm_l;
      }
    }
    if (pwm_l < 0)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
      if (pwm_l < -8000)
        TIM4->CCR3 = 8000;
      else
      {
        TIM4->CCR3 = -pwm_l;
      }
    }
    //		printf("%d %d\r\n",measureval_l,targetval_l);
    last_err_l = err_l;

    //右轮
    measureval_r = (int16_t)-__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    ref_speed_r = ref_speed_r + delta_speed / 2;

    if (ref_speed_r > 650)
      ref_speed_r = 650;
    else if (ref_speed_r < 0)
      ref_speed_r = 0;
    err_r = ref_speed_r - measureval_r;
    err_sum_r += err_r;
    if (err_sum_r > 80000)
      err_sum_r = 80000;
    if (err_sum_r < -80000)
      err_sum_r = -80000;
    //抗积分饱和
    //		if (err_sum_r > 80000)
    //		{
    //			if(err_r <= 0)err_sum_r += err_r;
    //
    //		}
    //		else if(err_sum_r < -80000)
    //		{
    //			if(err_r >= 0)err_sum_r += err_r;
    //		}
    //		else
    //		{
    //			err_sum_r += err_r;
    //		}
    //梯形积分，将err_r换为(err_r + last_err_r) / 2即可
    rp_r = kp_r * err_r;
    ri_r = ki_r * err_sum_r;
    rd_r = kd_r * (err_r - last_err_r);
    pwm_r = rp_r + ri_r + rd_r;
    if (pwm_r > 0)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
      if (pwm_r > 8000)
        TIM4->CCR4 = 8000;
      else
      {
        TIM4->CCR4 = pwm_r;
      }
    }
    if (pwm_r < 0)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
      if (pwm_r < -8000)
        TIM4->CCR4 = 8000;
      else
      {
        TIM4->CCR4 = -pwm_r;
      }
    }
    //		printf("%d %d\r\n",measureval_r,targetval_r);
    last_err_r = err_r;
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
