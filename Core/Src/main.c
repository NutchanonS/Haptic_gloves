/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "DFPLAYER_MINI.h"
#include <stdio.h>
#include <stdlib.h>
#include "mpu6050.h"
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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;

//-------------------------------//

double raw_value;
char Resist[20] = "wowo";
double Readvoltage[3];
double ReadR[3];
const int R = 10; // Resistance
const int V = 3.3; // voltage
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char buf[1];
char urtrx[10]={0};
uint8_t urtrxStatus;

char val[200];
char gesture[200];
uint8_t result = 0x05;
int vol;
char volu[20];


void Adc_select_0(){
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void Adc_select_1(){
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void Adc_select_4(){
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void Adc_select_6_poten(){
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_6;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
float resist_to_percent(float R, float min_old, float max_old, float min_new, float max_new) {
	return (R-min_old)/(max_old-min_old)*(max_new-min_new);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MPU6050_Init(&hi2c1);
  /* USER CODE BEGIN 2 */
  DF_Init(10);
//  DF_PlayFromStart();
  uint8_t a = 0x01;
  Send_cmd(0x03,0x00, 0x05); //play
//  HAL_Delay(500);

	float Ax, Ay, Az, Gx, Gy, Gz;
	float Axmin[4] = {-0.34,0.46,0.61,0.09};  // คน, สวัสดี, อันตราย, เกรงใจ
	float Axmax[4] = {0.42 ,1.1 ,1.17, 0.86};
	float Aymin[4] = {0.75,-0.14,0.06,0.52};
	float Aymax[4] = {1.2 ,0.61 ,0.83,1.13};
	float Azmin[4] = {-0.54,-1.15,-0.41,-0.41};
	float Azmax[4] = {0.18 ,-0.41 ,0.43 ,0.51};

	float V0min[4] = {0.52,0.54,0.42, 0.53};
	float V0max[4] = {0.9,0.9,0.79,0.88};
	float V1min[4] = {0.38,0.35,0.04,0.1};
	float V1max[4] = {0.83,0.81,0.41,0.52};
	float V2min[4] = {0.35,0.38,0.13,0.15};
	float V2max[4] = {0.75,0.78,0.52,0.53};
  int oldvoice = 5;
  int newvoice = 0;
  int cou = 0;
  char couu[15];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_UART_Transmit(&huart2,"first" ,sizeof("first"),HAL_MAX_DELAY);
  while (1)
  {
	  MPU6050_Read_All(&hi2c1, &MPU6050);

	  	Ax = MPU6050.Ax;
	  	Ay = MPU6050.Ay;
	  	Az = MPU6050.Az;
	  	Gx = MPU6050.Gx;
	  	Gy = MPU6050.Gy;
	  	Gz = MPU6050.Gz;

	  	Adc_select_0();
	  	HAL_ADC_Start(&hadc1);
	  	HAL_ADC_PollForConversion(&hadc1, 1000);
	  	raw_value = (double)HAL_ADC_GetValue(&hadc1);
	  	Readvoltage[0] = raw_value*V/4095;
	  	ReadR[0] = ((V*R)/Readvoltage[0])-R;
//	  	ReadR[0] = resist_to_percent(ReadR[0],30,105,0,100);
	  	sprintf(Resist,"read0=%f\t" , Readvoltage[0]);
	  	HAL_UART_Transmit(&huart2,Resist ,sizeof(Resist),HAL_MAX_DELAY);
	  	HAL_ADC_Stop(&hadc1);

	  	Adc_select_1();
	  	HAL_ADC_Start(&hadc1);
	  	HAL_ADC_PollForConversion(&hadc1, 1000);
	  	raw_value = (double)HAL_ADC_GetValue(&hadc1);
	  	Readvoltage[1] = raw_value*V/4095;
	  	ReadR[1] = ((V*R)/Readvoltage[1])-R;
//	  	ReadR[1] = resist_to_percent(ReadR[1],27,48,0,100);
	  	sprintf(Resist,"read1=%f\t" , Readvoltage[1]);
	  	HAL_UART_Transmit(&huart2,Resist ,sizeof(Resist),HAL_MAX_DELAY);
	  	HAL_ADC_Stop(&hadc1);

	  	Adc_select_4();
	  	HAL_ADC_Start(&hadc1);
	  	HAL_ADC_PollForConversion(&hadc1, 1000);
	  	raw_value = (double)HAL_ADC_GetValue(&hadc1);
	  	Readvoltage[2] = raw_value*V/4095;
	  	ReadR[2] = ((V*R)/Readvoltage[2])-R;
//	  	ReadR[2] = resist_to_percent(ReadR[2],30,60,0,100);
	  	sprintf(Resist,"read2=%f\t" ,Readvoltage[2]);
	  	HAL_UART_Transmit(&huart2,Resist ,sizeof(Resist),HAL_MAX_DELAY);
	  	HAL_ADC_Stop(&hadc1);

	  	Adc_select_6_poten();
	  	HAL_ADC_Start(&hadc1);
	  	HAL_ADC_PollForConversion(&hadc1, 1000);
	  	raw_value = (double)HAL_ADC_GetValue(&hadc1);
	  	vol = raw_value*30/4095;
	  	//sprintf(volu,"%d\r\n" , vol);
//	  	HAL_UART_Transmit(&huart2,volu ,sizeof(volu),HAL_MAX_DELAY);
	  	Send_cmd(0x06, 0x00, vol);
	  	HAL_ADC_Stop(&hadc1);

	     if (Ax >= Axmin[0] && Ax <= Axmax[0] && Ay>=Aymin[0] && Ay <= Aymax[0] && Az >= Azmin[0] && Az <= Azmax[0]  && Readvoltage[0] >= V0min[0] && Readvoltage[0] <= V0max[0] && Readvoltage[1] >= V1min[0] && Readvoltage[1] <= V1max[0] && Readvoltage[2] >= V2min[0] && Readvoltage[2] <= V2max[0]) {
	  	   sprintf(gesture,"คน \r\n");
	  	   newvoice =1;
	  	   result = 0x01;
	     }
	     else if (Ax >= Axmin[1] && Ax <= Axmax[1] && Ay>=Aymin[1] && Ay <= Aymax[1] && Az >= Azmin[1] && Az <= Azmax[1]  && Readvoltage[0] >= V0min[1] && Readvoltage[0] <= V0max[1] && Readvoltage[1] >= V1min[1] && Readvoltage[1] <= V1max[1] && Readvoltage[2] >= V2min[1] && Readvoltage[2] <= V2max[1] ){
	  	   sprintf(gesture,"สวัสดี \r\n");
	  	   newvoice =2;
	  	   result = 0x04;
	     }
	     else if (Ax >= Axmin[2] && Ax <= Axmax[2] && Ay>=Aymin[2] && Ay <= Aymax[2] && Az >= Azmin[2] && Az <= Azmax[2]  && Readvoltage[0] >= V0min[2] && Readvoltage[0] <= V0max[2] && Readvoltage[1] >= V1min[2] && Readvoltage[1] <= V1max[2] && Readvoltage[2] >= V2min[2] && Readvoltage[2] <= V2max[2] ){
	  	   sprintf(gesture,"อันตราย \r\n");
	  	   newvoice =3;
	  	   result = 0x02;
	     }

	     else if (Ax >= Axmin[3] && Ax <= Axmax[3] && Ay>=Aymin[3] && Ay <= Aymax[3] && Az >= Azmin[3] && Az <= Azmax[3]  && Readvoltage[0] >= V0min[3] && Readvoltage[0] <= V0max[3] && Readvoltage[1] >= V1min[3] && Readvoltage[1] <= V1max[3] && Readvoltage[2] >= V2min[3] && Readvoltage[2] <= V2max[3] ){
	  	   sprintf(gesture,"เกรงใจ \r\n");
	  	   newvoice =4;
	  	   result = 0x03;
	     }
	     else {
	  	   sprintf(gesture,"อ่อน \r\n");
	  	   newvoice =5;
	  	   result = 0x05;
	     }
	 sprintf(val,"Ax=%.2f\t  Ay=%.2f\t Az=%.2f\t %d\t",Ax,Ay,Az,result);
	 HAL_UART_Transmit(&huart2,val,strlen((char*)val),HAL_MAX_DELAY);
	  //----------------------------------------------------------//
	  Send_cmd(0x42, 0, 0);
	  HAL_UART_Receive(&huart1,urtrx,sizeof(urtrx),HAL_MAX_DELAY);
	  sprintf(buf,"%d\r\n",urtrx[7]);
	  if(buf[0] == '0' && newvoice != oldvoice ){ //not play yet && newvoice != oldvoice
		  sprintf(buf,"%d\r\n",result);
//		  HAL_UART_Transmit(&huart2,buf,sizeof(buf),HAL_MAX_DELAY);
		  Send_cmd(0x03,0x00, result);
		  cou++;
		  sprintf(couu,"0 ,%d\r\n" ,cou);
		  HAL_UART_Transmit(&huart2,couu ,sizeof(couu),HAL_MAX_DELAY);
		  oldvoice = newvoice;
	  }
	  else if (buf[0] == '1'){ //playing
		  HAL_UART_Transmit(&huart2,"1\r\n" ,sizeof("1\r\n"),HAL_MAX_DELAY);
	  }
	  else {
		  HAL_UART_Transmit(&huart2,"0\r\n" ,sizeof("0\r\n"),HAL_MAX_DELAY);
	}
	  HAL_Delay(1500);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
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
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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
