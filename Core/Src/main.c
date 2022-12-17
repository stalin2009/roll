
#include "main.h"


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


TIM_HandleTypeDef htim2;

uint32_t stepCount=0;

uint32_t botPos=14000000;

uint8_t runCount=0;

int resetCount   =    0;
int resetValue =    0;
#define delitel 8
#define schetchik 29
int adcValue     = 2048;
int speed=10;


int upSpeed = 200;
int downSpeed = 200;
int botFlag = 0; 
int dir = 0; 
int en = 0; 

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

enum states {
	init,
	goDown,
	goUp,
	stayDown,
	stayUp
};
enum states curState = init;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

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

}


int main(void)
{
  
  HAL_Init();


  SystemClock_Config();



  MX_GPIO_Init();
  MX_TIM2_Init();

	HAL_Delay(4000);

HAL_TIM_Base_Start_IT(&htim2);
  
		/* Infinite loop _-----------------------------------------------------------------------*/

  while (1)
  { 
	
		switch (curState) {
			case init:
				{
					curState = goUp;
					break;
				}
			case goUp:
				{
					if(HAL_GPIO_ReadPin(datTop_GPIO_Port, datTop_Pin)== DAT_OFF)
					{
						HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, DIR_UP);
						HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, ENABLE_ON);
					}
					else
					{
						HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, ENABLE_OFF);
						curState = stayUp;
						
					}
					break;
				}
			case stayUp:
			{
				if(HAL_GPIO_ReadPin(proj_GPIO_Port, proj_Pin) == PROJ_ON)
				{
					HAL_Delay(500);
					if(HAL_GPIO_ReadPin(proj_GPIO_Port, proj_Pin) == PROJ_ON)
					{
						curState = goDown;
						runCount = 1; //zapuskaem schetchik wniz
					}
					else
						break;
				}
				break;			
			}
			case goDown:
			{
				if(HAL_GPIO_ReadPin(proj_GPIO_Port, proj_Pin) == PROJ_OFF)
				{
					HAL_Delay(1000);
					if(HAL_GPIO_ReadPin(proj_GPIO_Port, proj_Pin) == PROJ_OFF)
					{
						curState = goUp;
						runCount =0;
						stepCount=0;
					}
				}
				else{
					if (stepCount<botPos)
					{
						HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, DIR_DOWN);
						HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, ENABLE_ON);
					}
					else {
							HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, ENABLE_OFF);
						curState = stayDown;
						runCount=0;
						
					}				
				}
				break;
			}
			case stayDown:
			{
				if(HAL_GPIO_ReadPin(proj_GPIO_Port, proj_Pin) == PROJ_OFF)
				{
					HAL_Delay(1000);
					if(HAL_GPIO_ReadPin(proj_GPIO_Port, proj_Pin) == PROJ_OFF)
					{
						curState = goUp;
						runCount =0;
						stepCount=0;
					}
				}
			
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
				break;
			}
			}
			
		}

}



void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

 
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);


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


static void MX_TIM2_Init(void)
{


  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};


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

}


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
