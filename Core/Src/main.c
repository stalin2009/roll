  4У1144У 4У4 4У1Ы4 4  44		4444444  44444444У																																																																																																																																																																																		44У41 1111111111111111111                                                                                                                                                                                                                                                                                                                                                                        41 1	1 14 1 4УУ88 уё ёё                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        ёёёёёёёёё ёёёёёёёёёё                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 	ёёёёёёёёёыууууууууууууууууууууууууууууууууууууууяёёёёёёёёёёёёёёёёёёёёёёёёёёёяёёёёёёёёёёёёёёёёёёёяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяяууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууууяяяяя                                                                                                                                                                                                                                                                                                                                                                                            4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        ы874у ЁЯЫ 4УЯ           4УЫ84УУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУУ                                                                                                                                                                                                                                                                                                      48вё ё                                                                                                                                                                       4ё ёёё                                                                                                                                                                                    ё ёёёёёёёёёёёёё                                                                                                                                                                                                                           ё ёёёё ё  ё              ёёёёёёёёёё ёёё                     ё ё          ё      4ё ё ёёёёёёёёёёёёёёёёёёёё          ё     ё ё4уёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёё ё ё/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENABLE_ON 0
#define ENABLE_OFF 1
#define DIR_UP 0
#define DIR_DOWN 1
#define PROJ_ON 1
#define PROJ_OFF 0
#define DAT_ON 0
#define DAT_OFF 1
#define BUTTON_ON 0
#define BUTTON_OFF 1

//#define ENABLE_ON 1
//#define ENABLE_ON 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t   TMC2208_SYNC =  0x05;
uint8_t  TMC_WRITE  = 0x80;
uint8_t  topFlag = 0; 
uint32_t stepCount=0;
uint8_t GENERAL_ADDR = 0x00;
uint32_t botPos=14000000;
uint8_t bufer[8]={0,0,0,0,0,0,0};
uint8_t bufer1[7]={0x45,0x67,0x78,0x90,0xab,0xcd,0xef};
uint8_t VACTUAL_ADDR = 0x22;
uint32_t VACTUAL_DATA = 0b11111111;
uint8_t runCount=0;

int resetCount   =    0;
int resetValue =    0;
#define delitel 8
#define schetchik 29
int adcValue     = 2048;
int speed=10;


int upSpeed = 200;
int downSpeed = 200;
int botFlag = 0; // ?�?�?�?? ?????�?�?�?�?�???�?????? ?????�???�???? ?????�?�?�?????�, 1- ?�?�?�?? ?????�?�?�?�?�???�?????�
int dir = 0; //???�?????�???�?�?????� ?????�?�?�??????, 1 = ?????�???�;
int en = 0; //???�?�???�?�?�?????� ?????�?�?�??????
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//if(htim -> Instance == TIM2)
	//{
		if (resetCount >= (resetValue))
		{
			resetCount = 0;
			HAL_GPIO_TogglePin(step_GPIO_Port, step_Pin);
			if(runCount)
			{
				stepCount++;
			}
		}
		else
		{
			resetCount++;
		}
	//}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  {/* USER CODE BEGIN 1 */

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(4000);

//HAL_UART_Receive_IT(&huart3, bufer, 8); 
//write(GENERAL_ADDR,GENERAL_DATA);
//HAL_Delay(1000);
//write(VACTUAL_ADDR, VACTUAL_DATA);
//HAL_Delay(1000);
//write(0x6C, 0b001110000000000000001
//HAL_Delay(2000);
//HAL_UART_Transmit_IT(&huart2, bufer1, 7);
  /* USER CODE END 2 */
if(HAL_GPIO_ReadPin(datTop_GPIO_Port, datTop_Pin)== 0)
{
	dir= DIR_DOWN;
	en=0;
					HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, DIR_DOWN);

	HAL_GPIO_WritePin(enable_GPIO_Port,enable_Pin, ENABLE_OFF);
	topFlag=1;
}
HAL_TIM_Base_Start_IT(&htim2);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */}
  while (1)
  {
	//	HAL_UART_Receive_IT(&huart3, bufer, 8); 
		//HAL_UART_Transmit_IT(&huart3, bufer1, 1);
		
		if((HAL_GPIO_ReadPin(proj_GPIO_Port, proj_Pin) == PROJ_ON)&&(topFlag==1)) //проектор включен и срабатывал верхний датчик
			{ 
			if (dir == DIR_UP)//направление было вверх
			{
				dir = DIR_DOWN;
				HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, DIR_DOWN);
			
			}
			runCount =1;
			if(botFlag == 0) // нижний датчик не срабатывал
			{
				if(/*HAL_GPIO_ReadPin(datBot_GPIO_Port,datBot_Pin) == DAT_ON*/stepCount>=botPos) //приехали вниз
				{
					botFlag = 1;
					HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, ENABLE_OFF); //тормозим 
					en = 0;
					runCount=0;//стоп счетчик
				}
				else
				{
					if(en == 0 )
					{
					HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, ENABLE_ON); //
						en = 1;
					}
				}
			}
			else //
			{
				if (HAL_GPIO_ReadPin(knDown_GPIO_Port, knDown_Pin) == BUTTON_ON)
				{
					HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, ENABLE_ON); //
					HAL_Delay(100);
					HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, ENABLE_OFF); //
				}
				
				if ((HAL_GPIO_ReadPin(knUp_GPIO_Port, knUp_Pin) == BUTTON_ON)&&(HAL_GPIO_ReadPin(datTop_GPIO_Port, datTop_Pin)== DAT_OFF))//
				{
					HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, DIR_UP);
					HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, ENABLE_ON); //
					HAL_Delay(100);
					HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, ENABLE_OFF); //
					HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, DIR_DOWN);
				}
			}
			
		
	}
		else //
		{
			HAL_Delay(1000);
			if(HAL_GPIO_ReadPin(proj_GPIO_Port, proj_Pin) == PROJ_ON)//ждем секунду, если проектор включен пропускаем цикл
			{
				continue;
			}
			if (dir == DIR_DOWN)//если  было направление вниз
			{
				dir = DIR_UP;
				HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, DIR_UP); //переключаем крутить вверх
				botFlag  = 0 ; //
								
			}
			
			if(HAL_GPIO_ReadPin(datTop_GPIO_Port, datTop_Pin)== DAT_OFF)// 
			{topFlag=0;
				if (en == 0)
				{
					HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, ENABLE_ON); //
					en = 1;
				}
			}
			else //
			{
				if (en == 1)
				{
					topFlag=1;
					stepCount=0;
					HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, ENABLE_OFF);// 
					en = 0;
				}
			}
		}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = delitel;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = schetchik;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|enable_Pin|dir_Pin|step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : knDown_Pin knUp_Pin datTop_Pin datBot_Pin */
  GPIO_InitStruct.Pin = knDown_Pin|knUp_Pin|datTop_Pin|datBot_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : proj_Pin */
  GPIO_InitStruct.Pin = proj_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(proj_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 enable_Pin dir_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|enable_Pin|dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : step_Pin */
  GPIO_InitStruct.Pin = step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(step_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//uint8_t calcCRC(uint8_t datagram[], uint8_t len) {
//     uint8_t crc = 0;
//     for (uint8_t i = 0; i < len; i++) {
//         uint8_t currentByte = datagram[i];
//         for (uint8_t j = 0; j < 8; j++) {
//             if ((crc >> 7) ^ (currentByte & 0x01)) {
//                 crc = (crc << 1) ^ 0x07;
//             } else {
//                 crc = (crc << 1);
//             }
//             crc &= 0xff;
//             currentByte = currentByte >> 1;
//         }
//     }
//     return crc;
// }
  

// void write(uint8_t addr, uint32_t regVal) {
//     uint8_t len = 7;
//     addr |= TMC_WRITE;
//     uint8_t datagram[] = {TMC2208_SYNC, slave_address, addr, (uint8_t)(regVal>>24), (uint8_t)(regVal>>16), (uint8_t)(regVal>>8), (uint8_t)(regVal>>0), 0x00};
//  
//     datagram[len] = calcCRC(datagram, len);

//              
//							
//							while(HAL_UART_Transmit_IT(&huart3, datagram, len) == HAL_BUSY);// ждем готовность и отправяем в uart
//        
//     HAL_Delay(100);
// }
  
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
