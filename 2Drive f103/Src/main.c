/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;

uint32_t              TxMailbox;
uint8_t        		  	CANRxData[8];
volatile uint16_t 							RxFlags = 0;
uint32_t							MaxGoal =0;

// variables for MotorControl
volatile 	uint32_t Goal = 0; 								// Goal of cart position 
	uint32_t Position = 0;   					// position whaere cart located
volatile 	uint32_t MinDelay = 30000;  			// Max Speed (delay between toggles step pin than speed is MAX)
	uint32_t StepDelay = 0;						// delay between steps
volatile 	uint16_t Boost = 10000; 						// 1 = MAX boost 65535 = MIN boost 
	uint32_t MinSpeed = 600;	  			// move graphic boost function to left side. Whan value == 0 boost function do not move 
	GPIO_PinState  MoveDown = GPIO_PIN_RESET, MoveUp = GPIO_PIN_SET; // Direction for moving Up or Down
	

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
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;// ????????? ???????????? DWT
  DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; // ???????? ???????
  DWT_CYCCNT = 0;// ???????? ???????
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
  MX_CAN_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET); // turn Off drivers
  GPIOC->ODR = 0 << 13;
	HAL_Delay(1000);
	GPIOC->ODR = 1 << 13;
	
  //section for LED	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
 
	TIM2->CCR1 = 255;
	TIM2->CCR2 = 255;
	TIM2->CCR3 = 255;
	
	
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		RxAnswerFunction();
		MotorControl();
		SpeedColibration();
		CalobrationDistance ();
		//LimSwChack ();
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
			// FUNCTION EMERGENCY STOP /////////////////////////////////
	
			if ( (RxFlags & (uint16_t)1<<7) == (uint16_t)1<<7)
			{
			//--------------------------------
			// HERE MUST BE writing in FLAH
			
			while(1) 
				{
					HAL_Delay(100);
					HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
				}
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 80;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
CAN_FilterTypeDef  sFilterConfig;
	
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh = 0x0200  << 5;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0F00 << 5;
  sFilterConfig.FilterMaskIdLow = 0xFFFF;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /* Start the CAN peripheral */
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /* Activate CAN RX notification */
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }
  
  /* Configure Transmission process */
	TxHeader.StdId = 0x0700;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
  /* USER CODE END CAN_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 256;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_Pin|STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PowerMon_Pin */
  GPIO_InitStruct.Pin = PowerMon_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PowerMon_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LimSwLOW_Pin LimSwHIGH_Pin */
  GPIO_InitStruct.Pin = LimSwLOW_Pin|LimSwHIGH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin STEP_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
     /* Get RX message */
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CANRxData);
	
	switch (RxHeader.StdId & 0x00FF)
	{
		case 1:
			// calibrate distanse range	
			
			RxFlags |= 1<<1;
		break;
		case 2:
			// calibrate max speed	
				
			
			RxFlags |= 1<<2;
		break;
		case 3:
			//new Goal of position 'N'
		
			RxFlags |= 1<<3;
			Goal =( ((uint32_t) CANRxData[0]<<24) | ((uint32_t) CANRxData[1]<<16) | ((uint32_t) CANRxData[2]<<8) | CANRxData[3]);
//			if (Goal==Position)
//				{
//				CanTxMsg
//				}
		break;
		case 4:
			//request cart position (answer will be "Position" variable)
			
			RxFlags |= 1<<4;
		break;
		case 5:
			// RGB light settings. If R,G,B == 0,0,0 light is off.
		
			TIM2->CCR1 = CANRxData[0];  // Red light
			TIM2->CCR2 = CANRxData[1];  // Green light
			TIM3->CCR3 = CANRxData[2];  // Blue light
		break;
		case 6:
			// RESET MCU!
		
			NVIC_SystemReset();
		break;
		case 7:
			// EMERGENCY STOP!
			
			RxFlags |= 1<<7;
		break;
		case 8:
			// Manual set MinDelay (max speed)
				
			MinDelay = ( ((uint32_t) CANRxData[0]<<24) | ((uint32_t) CANRxData[1]<<16) | ((uint32_t) CANRxData[2]<<8) | CANRxData[3]);			
		break;
		case 9:
			// Manual set Boost
			
			Boost = ( ((uint32_t) CANRxData[0]<<8) | ((uint32_t) CANRxData[1]) );
		break;
	}
	
	/*
   Display LEDx 
  if ((RxHeader.StdId == 0x0020) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 8))
  {
		 GPIOC->ODR = DATAS.TransmitParams.Code << 13;
  }
	*/
}
void HAL_CAN_TxMailbox0CompleteCallback (CAN_HandleTypeDef *hcan)
{
	 GPIOC->ODR = 0 << 13;
	GPIOC->ODR = 1 << 13;
		
}

char MotorControl (void)
{		
	/* This function for work use graphic (1/x*x) 
	
	Global variables for work this function:
	
	uint32_t Goal = 0; 								// Goal of cart position 
	uint32_t Position = 0;   					// position whaere cart located
	uint32_t MinDelay = 50000;  			// Max Speed (delay between toggles step pin than speed is MAX)
	uint32_t StepDelay = 0;						// delay between steps
	uint16_t Boost = 8000; 						// 1 = MAX boost 65535 = MIN boost 
	uint32_t MinSpeed = 600;	  			// move graphic boost function to left side. Whan value == 0 boost function do not move 
	uint8_t MoveDown = 0, MoveUp =0; 	// Direction for moving Up or Down
	*/
	
		static uint32_t Distance = 0;		// nomber of steps between Location and Goal 
		static uint32_t Step = 0;				// nomber of step
		static uint8_t FlagStep = 0;
		
	if (Position != Goal)
	{
		HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET); // turnon driver
		
		// Direction computing
		if(Goal < Position) 
			{//movimmg down
				Distance = Position - Goal;
				HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, MoveDown);
			}											
		else															
			{// moving up																
				Distance = Goal - Position;
				HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, MoveUp);
			}																
																					
		// pulce fore drive	control
		if(DWT_CYCCNT > 1000 && FlagStep == 1) // turn off Step Pin. Make pulse long ~ 20mks
				{														
					HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
					FlagStep = 0;										
				}											
																	
		// Boost
		if (Step < Distance/2 && StepDelay < DWT_CYCCNT) 
			{
				HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_SET);
				DWT_CYCCNT = 0;	//counter of tacts = 0
				FlagStep = 1;		
				Step++;			
				StepDelay = ((MinDelay*Boost)/(((Step)*(Step))+MinSpeed))+MinDelay; // computing delay
			}		
		
		// SLowDOwn (inverted boot)
		if (Step >= Distance/2 && StepDelay < DWT_CYCCNT) 
			{
				HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_SET);
				DWT_CYCCNT = 0;
				FlagStep = 1;
				Step++;
				StepDelay = ((MinDelay*Boost)/(((Distance-Step)*(Distance-Step))+MinSpeed))+MinDelay;
			}		
	
			// if we arrived to destenation
	if (Step == Distance)
			{		
					Position = Goal;
					Distance=0;
					Step = 0;
					FlagStep = 0;
					StepDelay = 0;
				//	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);// turnoff driver
				
					return 0; // Function Finish Work
			}
			return 1;	// Function in Work
		}
	return 0; // Function redy to Work
}


void RxAnswerFunction(void)
{
	if ( ((RxFlags & 1<<3) == (uint16_t)1<<3) && (Goal == Position) )
		{// send message if Goal is reached 
				uint8_t TxData[4] = {0};
				
				TxData[0] =	Position >> 24; 
				TxData[1] =	Position >> 16;
				TxData[2] =	Position >> 8;
				TxData[3] =	Position;
				
				TxHeader.StdId = 0x0703;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.DLC = 4;
			
				//send msg and reset flg
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) RxFlags &= ~((uint16_t)1<<3);							
						
		}
	if ( (RxFlags & 1<<4) == (uint16_t)1<<4)
		{// answer on position reqest
				uint8_t TxData[4] = {0};
				
				TxData[0] =	Position >> 24; 
				TxData[1] =	Position >> 16;
				TxData[2] =	Position >> 8;
				TxData[3] =	Position ;
				
				TxHeader.StdId = 0x0703;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.DLC = 4;
			
				//send msg and reset flg
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) RxFlags &= ~((uint16_t)1<<4);
		}
	
}
// This function colibrate max speed and 
// check state of Limit Switchs (if state is set :EMERGENCY STOP!)
void SpeedColibration (void)
{
		//colibrae max speed
	
}
/*			//	HAL_Delay(100);
			MinDelay = 30000;
			//static uint8_t iterations =0;
			Goal = 0;
			if (Position == 0) 
				{
					MinDelay -= 4000;
					//iterations ++;
					Goal = MaxGoal;
				}	
			if ( (Position == MaxGoal) && (Goal == Position) )	
				{
					uint8_t counter =0;
					HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);//turn ON Driver				
					HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, MoveUp); // derection for Move UP
					
					while ( (HAL_GPIO_ReadPin(LimSwHIGH_GPIO_Port, LimSwHIGH_Pin) != 1 )&& (counter <= 200) )
						{
							HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
							HAL_Delay(1);
							HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
							HAL_Delay(1);
							counter ++;
						}
						if( (HAL_GPIO_ReadPin(LimSwHIGH_GPIO_Port, LimSwHIGH_Pin) != 1 )&&(counter>200) )
						{// max speed determinate
							HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, MoveDown); // derection for Move DOWN	
							while  (HAL_GPIO_ReadPin(LimSwLOW_GPIO_Port, LimSwLOW_Pin) != 1 )
								{
									HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
									HAL_Delay(1);
									HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
									HAL_Delay(1);
								}
							HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, MoveUp); // derection for Move UP
							while  (HAL_GPIO_ReadPin(LimSwLOW_GPIO_Port, LimSwLOW_Pin) == 1 )
								{
									HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
									HAL_Delay(1);
									HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
									HAL_Delay(1);
								}
								for (char i=0;i<=20;i++)
								{
									HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
									HAL_Delay(1);
									HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
									HAL_Delay(1);
								}
								
							MinDelay += 5000;
							RxFlags &= ~(1<<2); // reset flag
							CanTxMsg(MinDelay,0x0702,4);
							HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);//turn OFF Driver				
							Goal = 0;
							Position = 0;
							return 0;//ok
						}
						if(HAL_GPIO_ReadPin(LimSwHIGH_GPIO_Port, LimSwHIGH_Pin) == 1 )
						{
							HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);//turn ON Driver				
							HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, MoveDown); // derection for Move UP
							for(char i = counter; i==0; i--)
								{
									HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
									HAL_Delay(1);
									HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
									HAL_Delay(1);	
								}
								HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);//turn OFF Driver				
							Goal = 0;	
						}
					
				}
	
		}
return 0;
}
	
	
	while ((RxFlags & 1<<2) == (uint16_t)1<<2) 
	{
		
		//set new min delay
		MinDelay = 30000;	
		// go to max positiom
		Goal = MaxGoal;
		if (Position == Goal)
		{	// go to max Position + 3 Steps for push on High Limit Switch
			HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);//turn ON Driver				
			HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, MoveUp); // derection for Move UP
			for (char i=0; i<10; i++)
			{
						HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
						HAL_Delay(1);
						HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
						HAL_Delay(1);
			}
			HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);//turn OFF Driver
		}
	
		if(HAL_GPIO_ReadPin(LimSwHIGH_GPIO_Port,LimSwHIGH_Pin) == 1 ) 
		{
			Goal = 0;
			//this is value of decrement MinDelay if max speed not detepminate
			//! \/ \/ \/ \/ \/ !!!!need calibrate vaue "2000"!!!! 
			MinDelay -= 2000;
			if (MinDelay < 5000)
				{ //max speed determinated
					MinDelay = 8000;
					RxFlags &= ~(1<<2); // reset flag
							
							// set message: MaxSpeed determinated
							uint8_t TxData[4] = {0};
							TxData[0] =	MinDelay >> 24; 
							TxData[1] =	MinDelay >> 16;
							TxData[2] =	MinDelay >> 8;
							TxData[3] =	MinDelay ;
							TxHeader.StdId = 0x0702;
							TxHeader.RTR = CAN_RTR_DATA;
							TxHeader.DLC = 4;
							//send msg 
							HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);								
				}
				
				
		}
		else 
		{	// slowly move down and send value of max peed (MinDelay)
			HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);//turn ON Driver
			HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, MoveDown);
			while(HAL_GPIO_ReadPin(LimSwLOW_GPIO_Port,LimSwLOW_Pin) != 1)
			{
						HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
						HAL_Delay(1);
						HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
						HAL_Delay(1);
			}
			HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);//turn OFF Driver
			Goal = 0;
			Position = 0;
			// this is increment value MinDelay if was lost steps
			//! \/ \/ \/ \/ \/ !!!!need calibrate vaue "4000", "25000", "6000"!!!! 
			MinDelay += 4000;// determinate max speed (MinDelay)
				if (MinDelay > 25000) MinDelay += 6000;
			RxFlags &= ~(1<<2);//reset flag

					// set message: MaxSpeed determinated
					uint8_t TxData[4] = {0};
					TxData[0] =	MinDelay >> 24; 
					TxData[1] =	MinDelay >> 16;
					TxData[2] =	MinDelay >> 8;
					TxData[3] =	MinDelay ;
					TxHeader.StdId = 0x0702;
					TxHeader.RTR = CAN_RTR_DATA;
					TxHeader.DLC = 4;
					//send msg 
					HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);								
		}
		
		// if High Limit Switch is pushed we need Move Down 6 seps 
		if (HAL_GPIO_ReadPin(LimSwHIGH_GPIO_Port,LimSwHIGH_Pin) == 1)
		{
			HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);//turn ON Driver
			HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, MoveDown);
			for (char i=0; i<6; i++)
			{
						HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
						HAL_Delay(1);
						HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
						HAL_Delay(1);
			}
			HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);//turn OFF Driver
			Goal = MaxGoal - 3;
			Position = MaxGoal - 3;
		}
	}	
}
*/

void LimSwChack (void)
{
	// check state of Limit Swiches
	if (HAL_GPIO_ReadPin(LimSwHIGH_GPIO_Port, LimSwHIGH_Pin) == 1)
		{
			  // send to 3rd block "ERROR lost Zero position!" 
				TxHeader.StdId = 0x0704;
				TxHeader.RTR = CAN_RTR_REMOTE;
				TxHeader.DLC = 0;
				
				//send msg and reset flg
				HAL_CAN_AddTxMessage(&hcan, &TxHeader, 0, &TxMailbox);
				
				RxFlags |= 1<<7; // EMERGENCY STOP!!!
		}
	if ( (HAL_GPIO_ReadPin (LimSwLOW_GPIO_Port, LimSwLOW_Pin) == 1)  )
		{
				// send to 3rd block "ERROR lost Zero position!" 
				TxHeader.StdId = 0x0704;
				TxHeader.RTR = CAN_RTR_REMOTE;
				TxHeader.DLC = 0;
				
				//send msg and reset flg
				HAL_CAN_AddTxMessage(&hcan, &TxHeader, 0, &TxMailbox);
				
				RxFlags |= 1<<7; // EMERGENCY STOP!!!
		}
}

uint8_t CalobrationDistance (void)
{
	if ( (RxFlags & 1<<1) == (uint16_t)1<<1)
			{	// Calibration distance range
			
				uint32_t Counter = 0;
				uint32_t errorCounter =0;
				HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);	// turn on Driver
				HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_RESET);					// select DIR
				
					while ( ((HAL_GPIO_ReadPin(LimSwHIGH_GPIO_Port, LimSwHIGH_Pin) != GPIO_PIN_SET) && \
									 (HAL_GPIO_ReadPin(LimSwLOW_GPIO_Port,  LimSwLOW_Pin)  != GPIO_PIN_SET)) && \
									 (errorCounter < 0x0002FFFF) )
					 // stepping while Limit Switch will not pushed
					 //	For determinate low position of cart
					{
						HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
						HAL_Delay(1);
						HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
						HAL_Delay(1);
						errorCounter++;
					}
					if (HAL_GPIO_ReadPin(LimSwLOW_GPIO_Port, LimSwLOW_Pin) == GPIO_PIN_SET)
						{// if pushed LOW switch
							MoveDown = GPIO_PIN_RESET;
							MoveUp = GPIO_PIN_SET;
							HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, MoveUp);
							errorCounter = 0;
							Counter = 0;
							while ( (HAL_GPIO_ReadPin(LimSwHIGH_GPIO_Port, LimSwHIGH_Pin) != GPIO_PIN_SET) && (errorCounter < 0x0002FFFF) )
								{
									errorCounter ++;
									Counter++;
									HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
									HAL_Delay(1);
									HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
									HAL_Delay(1);										
								}
							if (HAL_GPIO_ReadPin(LimSwHIGH_GPIO_Port, LimSwHIGH_Pin) == GPIO_PIN_SET)
								{
										HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, MoveDown);
										uint16_t uncount = 0;
										while (HAL_GPIO_ReadPin(LimSwHIGH_GPIO_Port, LimSwHIGH_Pin) == 1)
										{
											uncount++;
											HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
											HAL_Delay(1);
											HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
											HAL_Delay(1);
										}
										for (char i =0; i<=40; i++)
										{
											HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
											HAL_Delay(1);
											HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
											HAL_Delay(1);
										}
									MaxGoal = Counter - (uncount + 40);
									Position = MaxGoal+20;
									Goal= MaxGoal+20;
									if (CanTxMsg(MaxGoal,0x0701,4)==0) RxFlags &= ~((uint16_t)1<<1);
									//HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);	//turn off Driver
									return 0; // OK	
								}
						}
/////////////////////////////////////////////////////////////////////////////////
					if (HAL_GPIO_ReadPin(LimSwHIGH_GPIO_Port, LimSwHIGH_Pin) == GPIO_PIN_SET)
						{// if pushed HIGH switch
							MoveDown = GPIO_PIN_SET;
							MoveUp = GPIO_PIN_RESET;
							HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, MoveDown);
							errorCounter = 0;
							Counter = 0;
							while ( (HAL_GPIO_ReadPin(LimSwLOW_GPIO_Port, LimSwLOW_Pin) != GPIO_PIN_SET) && (errorCounter < 0x0002FFFF) )
								{
									errorCounter ++;
									Counter++;
									HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
									HAL_Delay(1);
									HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
									HAL_Delay(1);										
								}
							if (HAL_GPIO_ReadPin(LimSwLOW_GPIO_Port, LimSwLOW_Pin) == GPIO_PIN_SET)
								{
										HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, MoveUp);
										uint16_t uncount = 0;
										while (HAL_GPIO_ReadPin(LimSwLOW_GPIO_Port, LimSwLOW_Pin) == 1)
										{
											uncount++;
											HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
											HAL_Delay(1);
											HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
											HAL_Delay(1);
										}
										for (char i = 0; i<20; i++ )
										{
											HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
											HAL_Delay(1);
											HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
											HAL_Delay(1);
										}
									MaxGoal = Counter - (uncount + 40);
									Position = 0;
									Goal= 0;
									if (CanTxMsg(MaxGoal,0x0701,4) == 0) RxFlags &= ~((uint16_t)1<<1);
									// HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);	//turn off Driver
									return 0; // OK	
								}	
								
						}
			
				
			}	
		return 1;// ERRor
	}		
		
		
		
		

uint8_t CanTxMsg(uint64_t  Data, uint16_t Addres, uint8_t size)
{
			if (Data == 0)
				{
					TxHeader.StdId = Addres;
					TxHeader.RTR = CAN_RTR_REMOTE;
					TxHeader.DLC = 0;
					
					//send msg and reset flg
					if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, 0, &TxMailbox) == HAL_OK) return 0;
				}
				
			if (size == 1)
				{
					uint8_t TxData[1];
					TxData[0] = Data;
				
					TxHeader.StdId = Addres;
					TxHeader.RTR = CAN_RTR_DATA;
					TxHeader.DLC = 1;
					
					//send msg and reset flg
					if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) return 0;
				}
			if (size == 2)
				{
					uint8_t TxData[2];
					
					TxData[0] = Data>>8;
					TxData[1] = Data;
				
					TxHeader.StdId = Addres;
					TxHeader.RTR = CAN_RTR_DATA;
					TxHeader.DLC = 2;
					
					//send msg and reset flg
					if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) return 0;
				}	
			if (size == 4)
				{
					uint8_t TxData[4];
					
					TxData[0] = Data>>24;
					TxData[1] = Data>>16;
					TxData[2] = Data>>8;
					TxData[3] = Data;
				
					TxHeader.StdId = Addres;
					TxHeader.RTR = CAN_RTR_DATA;
					TxHeader.DLC = 4;
					
					//send msg and reset flg
					if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) return 0;
				}
	return 1; // same errors
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
