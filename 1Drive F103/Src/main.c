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

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */


	// variables for CAN communication
	CAN_TxHeaderTypeDef   TxHeader = {0};
	CAN_RxHeaderTypeDef   RxHeader	= {0};
	 uint8_t               RxData[8] = {0};
	uint32_t              TxMailbox = 0;
	volatile uint16_t		 					RxFlags = 0;				// flags for RX request handling

	//variables for control stepdrives
	volatile int16_t Goal = 0; 						// Goal of paws position 
	volatile int16_t Position = 0;   			// position whaere paws located
	volatile uint32_t MinDelay = 40000; 	  // Max Speed (delay between toggles step pin than speed is MAX) 
	volatile uint32_t StepDelay = 0;				// delay between steps
	volatile uint16_t Boost = 10000; 			// 1 = MAX boost 65535 = MIN boost 
	volatile uint32_t MinSpeed = 600;	  	// move graphic boost function to left side. Whan value == 0 boost function do not move 
	volatile int16_t PawMaxPosition = 2000;// this value must be determinate by colibration!!!
	volatile int16_t PawMinPosition = 2000;// this value must be determinate by colibration!!!
	volatile int16_t PositionForTransmit =0;
	
	//variable for Servo control
	volatile uint16_t PawRxSize = 0;
	volatile uint16_t PawSize = 0;			 	// range between 600 and 870
	volatile uint16_t PawMaxSize = 850; 	// MAX value of variable PawSize, need colibration after reset!
	// NEED REALISE PawMinSize FUNCTION!
	volatile uint16_t PawMinSize = 600; 	// MIN value of variable PawSize, need colibration after reset! 
	volatile uint32_t PawOldGetTick = 0;
//uint8_t PawZeroPosition = 1;
	
	//variables for Conveyor control 
	volatile int16_t ConvRot = 0;			// ritation conveyor on this steps + or -
	

	
	
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(ENABLE1_GPIO_Port, ENABLE1_Pin, GPIO_PIN_SET); // turn Off drivers
	HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin, GPIO_PIN_SET); // turn Off drivers
	
	GPIOC->ODR = 0 << 13;HAL_Delay(500);GPIOC->ODR = 1 << 13;HAL_Delay(300);
	
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	TIM4 -> CCR1 = 850;
	TIM4 -> CCR2 = 850;
	HAL_Delay(500);
	
  /* USER CODE END 2 */

 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		MotorControl();
		RxAnswerFunction();
		ScanBox();
		
		
	/////////////for debug////////	
//		if ( (HAL_GPIO_ReadPin(PAW2CHECK_GPIO_Port,PAW2CHECK_Pin) == 1) )GPIOC->ODR = 0 << 13;
//		else GPIOC->ODR = 1 << 13;
//	if ( (HAL_GPIO_ReadPin(DETECT12_GPIO_Port,DETECT12_Pin) == 1) )GPIOC->ODR = 0 << 13;
//		else GPIOC->ODR = 1 << 13;
		///////for debug/////////////
		
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
  hcan.Init.AutoRetransmission = ENABLE;
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
  sFilterConfig.FilterIdHigh = 0x0100 << 5;
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
	
	//TxHeader.StdId = 0x0300;
  //TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  //TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
	
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 127;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  HAL_GPIO_WritePin(GPIOB, ServoPwr_Pin|DIR_Pin|STEP_Pin|ENABLE2_Pin 
                          |ENABLE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CONVENABLE_Pin|CONVSTEP_Pin|CONVDIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF2_Pin RF1_Pin */
  GPIO_InitStruct.Pin = RF2_Pin|RF1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAW4CHECK_Pin PAW3CHECK_Pin PAW2CHECK_Pin PAW1CHECK_Pin 
                           DETECT34_Pin DETECT12_Pin SWHUCK4_Pin SWHUCK3_Pin 
                           PAW34MaxRange_Pin */
  GPIO_InitStruct.Pin = PAW4CHECK_Pin|PAW3CHECK_Pin|PAW2CHECK_Pin|PAW1CHECK_Pin 
                          |DETECT34_Pin|DETECT12_Pin|SWHUCK4_Pin|SWHUCK3_Pin 
                          |PAW34MaxRange_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SWHUCK2_Pin SWHUCK1_Pin PAW12MaxRange_Pin RF4_Pin 
                           RF3_Pin */
  GPIO_InitStruct.Pin = SWHUCK2_Pin|SWHUCK1_Pin|PAW12MaxRange_Pin|RF4_Pin 
                          |RF3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ServoPwr_Pin DIR_Pin STEP_Pin */
  GPIO_InitStruct.Pin = ServoPwr_Pin|DIR_Pin|STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ENABLE2_Pin ENABLE1_Pin */
  GPIO_InitStruct.Pin = ENABLE2_Pin|ENABLE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CONVENABLE_Pin CONVSTEP_Pin CONVDIR_Pin */
  GPIO_InitStruct.Pin = CONVENABLE_Pin|CONVSTEP_Pin|CONVDIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PowerMon_Pin */
  GPIO_InitStruct.Pin = PowerMon_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PowerMon_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	
	switch (RxHeader.StdId & 0x00FF)
	{
		case 1:
			// request valie of distanse 1,2,3,4	
			
		
			RxFlags |= 1<<1;
		break;
		case 2:
			// RESET mcu
			NVIC_SystemReset();
		break;
		case 3://data s16
			//new Goal of position paws 'N'
				
			//////for debug////////
		GPIOC->ODR = 0 << 13;
		//////end debug///////
		
			RxFlags |= 1<<3;
			Goal =(((uint16_t) RxData[0]<<8) | RxData[1]);
		break;
		case 4:
			//reduction paws until tuch
				
			RxFlags |= 1<<4;
		break;
		case 5:
			//remove the grip in '0' position
				
			RxFlags |= 1<<5;
			Goal = 0;
		break;
		case 6://data u16
			// reduction paws on the distanse 'PawSize'
			GPIOC->ODR = 0 << 13;
				
			RxFlags |= 1<<6;
			PawOldGetTick = HAL_GetTick();
			PawRxSize = (((uint16_t) RxData[0]<<8) | RxData[1]);
			PawSize = PawRxSize;
			TIM4 -> CCR1 = PawSize;
			TIM4 -> CCR2 = PawSize; 
			
		break;
		case 7://data u32
			// set max speed moving paws
		
			RxFlags |= 1<<7;
			MinDelay = ( ((uint32_t) RxData[0]<<24) | ((uint32_t) RxData[1]<<16) | ((uint32_t) RxData[2]<<8) | RxData[3]);
		break;
		case 8://data s16
			// rotate conveyor on "A" steps + or -
		
			RxFlags |= 1<<8;
			ConvRot = ( ((uint16_t) RxData[0]<<8) | RxData[1]);
		break;
		case 9:
			// request of conveer position 
			 	
			RxFlags |= 1<<9;
		break;
		case 10:
			// request of paws size, need send "PawSize"
			
			RxFlags |= 1<<10;
		break;
		case 11:
			// calibrate max speed and long paws.
			// need send value:  max posetive range '+'16b, Max negative range '-'16b, Max Speed(Min delay)32b. 
			
			RxFlags |= 1<<11;
		break;
		case 12:
			// calibrate max and min range between paws 
			// anser weel be (Colibration complete, MAX uint16_t value, MIN uint16_t value)
		
			RxFlags |= 1<<12;
		break;
		case 13:
			// EMERGENCY STOP! 
			
			RxFlags |= 1<<13;
		break;
		case 14:
			// scan DEPTH of BOX
		//	GPIOC->ODR = 0 << 13;
			RxFlags |= 1<<14;
		break;
	}
		
  /* Display LEDx */
	/*  
	if ((RxHeader.StdId == 0x0010) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 8))
  {
	 GPIOC->ODR = RxData[0] << 13;
  }
	*/
}
void HAL_CAN_TxMailbox0CompleteCallback (CAN_HandleTypeDef *hcan)
{
	 GPIOC->ODR = 0 << 13;
//	HAL_Delay(100);
	GPIOC->ODR = 1 << 13;
		
}

char MotorControl (void)
{		
	/* This function for work use graphic (1/x*x) 
	
	Global variables for work this function:
	
	uint32_t Goal = 0; 					// Goal of cart position 
	uint32_t Position = 0;   		// position whaere cart located
	uint32_t MinDelay = 20000;  // Max Speed (delay between toggles step pin than speed is MAX)
	uint32_t StepDelay = 0;			// delay between steps
	uint16_t Boost = 8000; 			// 1 = MAX boost 65535 = MIN boost 
	uint32_t MinSpeed = 600;	  // move graphic boost function to left side. Whan value == 0 boost function do not move 
	*/
	
		static uint32_t Distance = 0;		// nomber of steps between Location and Goal 
		static uint32_t Step = 0;				// nomber of step
		static uint8_t FlagStep = 0;
		
	if (Position != Goal)
	{
		HAL_GPIO_WritePin(ENABLE1_GPIO_Port, ENABLE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin, GPIO_PIN_RESET);
		
		// Direction computing
		if(Goal < Position)// && Goal >= 0) 
			{
				Distance = Position - Goal;
				HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
			}											
		else// if (Goal > Position)															
			{																
				Distance = Goal - Position;
				HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
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
				StepDelay = ((MinDelay* Boost)/(((Step)*(Step))+MinSpeed))+MinDelay; // computing delay	
				if(Goal > Position)PositionForTransmit++;
			  else PositionForTransmit--;
			}		
		
		// SLowDOwn (inverted boot)
		if (Step >= Distance/2 && StepDelay < DWT_CYCCNT) 
			{
				HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_SET);
				DWT_CYCCNT = 0;
				FlagStep = 1;
				Step++;
				StepDelay = ((MinDelay*Boost)/(((Distance-Step)*(Distance-Step))+MinSpeed))+MinDelay;
				if(Goal > Position)PositionForTransmit++;
				else PositionForTransmit--;
			}		
	
			// if we arrived to destenation
	if (Step == Distance)
			{		
					Position = Goal;
					Distance=0;
					Step = 0;
					FlagStep = 0;
					StepDelay = 0;
					HAL_GPIO_WritePin(ENABLE1_GPIO_Port, ENABLE1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin, GPIO_PIN_SET);
					return 0; // Function Finish Work
				
			}
			return 1;	// Function in Work
		}
	return 0; // Function redy to Work
}


void RxAnswerFunction(void )	
{		
	/* aftre complete writing
	1 check all Flag bits (set or reset)
	2 makes check paws zero position
	3 complete function #5
	4 modifide function #4 with use PawGripPotenciometr whan it is will possably
	5 invert serfo 
	*/
	
		// Anser on rangefinder request
		if ((RxFlags & 1<<1) == (uint16_t)1<<1)
		{
				uint8_t TxData[4] = {0};
				
				TxData[0] =	HAL_GPIO_ReadPin(RF1_GPIO_Port, RF1_Pin); 
				TxData[1] =	HAL_GPIO_ReadPin(RF2_GPIO_Port, RF2_Pin); 
				TxData[2] =	HAL_GPIO_ReadPin(RF3_GPIO_Port, RF3_Pin); 
				TxData[3] =	HAL_GPIO_ReadPin(RF4_GPIO_Port, RF4_Pin); 
				
				TxHeader.StdId = 0x0301;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.DLC = 4;
			
				//send msg and reset flg
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) RxFlags &= ~((uint16_t)1<<1);
		}
		
		// moving paws to Position Complete
		if (((RxFlags & 1<<3) == (uint16_t)1<<3)&& (Goal == Position))
		{
				///////////////////////////////////
				//here must be check  Max and Min limites of position
				///////////////////////////////////
									//////for debug////////
								GPIOC->ODR = 1 << 13;
								//////end debug///////
				//send msg and reset flag
			
				if (CanTxMsg(Position,0x0303,2)==0)
				{					RxFlags &= ~((uint16_t)1<<3);
					GPIOC->ODR = 1 << 13;}
		}
		
		//function to grip on box 
		if ((RxFlags & 1<<4) == (uint16_t)1<<4)
		{	// side 1+ (1,2)        if PawSize== 600 -> |<<-->>|	far  (razvedeny) IT IS NEED Fixed !!!!
			// side 2- (3,4)				if PawSize== 900 -> ->>||<<-	near (svedeny)	 IT IS NEED Fixed !!!! (inverted)	
			// optimal speed of convergence: every 4 MilliSeconds make PawSize++
		static uint8_t CheckBoxError = 0;	
			//convergence of paws
			
////////if position > 0////////////////////////////////////////////
			if (Position > 0 && HAL_GetTick() > PawOldGetTick) // side 1
			{
				//check existence BOX between paws
				if(HAL_GPIO_ReadPin(DETECT12_GPIO_Port, DETECT12_Pin) == 0  && CheckBoxError == 0)
					{//send ERROR!! BOX not existance!
						CanTxMsg(0,0x0308,0);
						CheckBoxError = 1;	
						////for debug////////////////
						GPIOC ->ODR = 0 <<13;
						/////end debug///////////////
					}
				// check if paws do not tuch the box (buttons do not triggered)
				if (PawSize + 10 >= PawMaxSize/*BoxMinSize*/)// minimum size of paws
					{//send FATAL ERROR!! BOX do not gripped
							CheckBoxError = 0;
							RxFlags &= ~((uint16_t)1<<4);
							CanTxMsg(0,0x0309,0);
							TIM4 -> CCR1 = PawRxSize;
							TIM4 -> CCR1 = PawRxSize;
					}	
				PawOldGetTick = HAL_GetTick() + 4;	// delay 4 milli seconds
				PawSize++;
				TIM4 -> CCR1 = PawSize;
				TIM4 -> CCR2 = PawSize - 3;// range of back side must be more on 3
			}
//////if position <= 0  ///////////////////////////////////////////////////////////
			if (Position <= 0 && HAL_GetTick() > PawOldGetTick) // side 2
			{
				//check existence BOX between paws
				if(HAL_GPIO_ReadPin(DETECT34_GPIO_Port, DETECT34_Pin) == 0 && CheckBoxError == 0)
					{//send ERROR
							CanTxMsg(0,0x0308,0);
							CheckBoxError = 1;
					}
				// check if paws do not tuched the box (buttons do not triggered)
					if (PawSize + 10 >= PawMaxSize/*BoxMinSize*/)// minimum size of paws
					{//send FATAL ERROR!
							CheckBoxError = 0;
							RxFlags &= ~((uint16_t)1<<4);
							CanTxMsg(0,0x0309,0);
							TIM4 -> CCR1 = PawRxSize;
							TIM4 -> CCR1 = PawRxSize;	
					}	
					
				PawOldGetTick = HAL_GetTick() + 4;	
				PawSize++;
				TIM4 -> CCR1 = PawSize - 3;// range of back side must be more on 3
				TIM4 -> CCR2 = PawSize; 
			}
///////////if gripping successful///////////////////////////////			
			if  (HAL_GPIO_ReadPin(SWHUCK1_GPIO_Port, SWHUCK1_Pin)&&  				\
					 HAL_GPIO_ReadPin(SWHUCK2_GPIO_Port, SWHUCK2_Pin) == 1)
					{
						////for deug//////////////
						GPIOC ->ODR = 1 <<13;
						///end debug///////////////
						
						// here must be PawSize1 whan it whill be declorated
						CanTxMsg(PawSize,0x0304,2);
						RxFlags &= ~((uint16_t)1<<4);
						CheckBoxError = 0;
					}
					
			if	(HAL_GPIO_ReadPin(SWHUCK3_GPIO_Port, SWHUCK3_Pin)&&  				\
					 HAL_GPIO_ReadPin(SWHUCK4_GPIO_Port, SWHUCK4_Pin) == 1)
					{				
						
						// here must be PawSize2 whan it whill be declorated
						CanTxMsg(PawSize,0x0304, 2);
						RxFlags &= ~((uint16_t)1<<4);
						CheckBoxError = 0;
					}
		}
				// Check ERROR "paws lost position"
				// IF "0" position of paws is lost! ERROR!
			if ((Position == 0) && ((HAL_GPIO_ReadPin(PAW1CHECK_GPIO_Port,PAW1CHECK_Pin))|| \
															(HAL_GPIO_ReadPin(PAW2CHECK_GPIO_Port,PAW2CHECK_Pin))|| \
															(HAL_GPIO_ReadPin(PAW3CHECK_GPIO_Port,PAW3CHECK_Pin))|| \
															(HAL_GPIO_ReadPin(PAW4CHECK_GPIO_Port,PAW4CHECK_Pin))== 1))//HERE POSSABLY BE MISTAKE 0 OR 1!!!!
			{// Send ERROR!! Paws lost "0" position
					TxHeader.StdId = 0x030A;
					TxHeader.RTR = CAN_RTR_REMOTE;
					TxHeader.DLC = 0;
					
				HAL_CAN_AddTxMessage(&hcan, &TxHeader, 0, &TxMailbox);
			}
			
			// function moves paws to "0" position
		if (((RxFlags & 1<<5) ==  (uint16_t)1<<5) && 																					\
															(0 == Position) && 																					\
															((HAL_GPIO_ReadPin(PAW1CHECK_GPIO_Port,PAW1CHECK_Pin))|| 		\
															(HAL_GPIO_ReadPin(PAW2CHECK_GPIO_Port,PAW2CHECK_Pin)) || 		\
															(HAL_GPIO_ReadPin(PAW3CHECK_GPIO_Port,PAW3CHECK_Pin)) || 		\
															(HAL_GPIO_ReadPin(PAW4CHECK_GPIO_Port,PAW4CHECK_Pin))== 0))	//HERE POSSABLY BE MISTAKE 0 OR 1!!!!// 
		{
					// paws in "0" position
					TxHeader.StdId = 0x0305;
					TxHeader.RTR = CAN_RTR_REMOTE;
					TxHeader.DLC = 0;
					
					//send msg and reset flg
					if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, 0, &TxMailbox) == HAL_OK) RxFlags &= ~((uint16_t)1<<5);
		}
		// reduction paws on the distanse 'PawSize'
		if ( ((RxFlags & 1<<6) == (uint16_t)1<<6) && ((PawOldGetTick + 700) < HAL_GetTick()) )
		{	
				
				GPIOC->ODR = 1 << 13;
			
				uint8_t TxData[2] = {0};
				TxData[0] = PawSize >> 8;
				TxData[1] = PawSize;
				/*///////////////////////  
				its will be cool if PawSize will be check by potenchiometr
				*////////////////////////
				
			  TxHeader.StdId = 0x0306;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.DLC = 2;
				
				//send msg and reset flg
				//if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) RxFlags &= ~((uint16_t)1<<6);
				HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
				RxFlags &= ~((uint16_t)1<<6);
		}
		//rotation conveyor complete
		if (((RxFlags & 1<<8) == (uint16_t)1<<8) && (ConvRot == 0))
		{
				TxHeader.StdId = 0x1307;
				TxHeader.RTR = CAN_RTR_REMOTE;
				TxHeader.DLC = 0;
				
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, 0, &TxMailbox) == HAL_OK) RxFlags &= ~((uint16_t)1<<8);
		}
		// section for 12 function: control of lost box
			if(1){
						//if 1,2 buttn pushed
						static uint8_t OldButtPress1 =0,OldButtPress2 =0,OldButtPress3 =0,OldButtPress4 =0;
						if ((HAL_GPIO_ReadPin(SWHUCK1_GPIO_Port, SWHUCK1_Pin)&&  				\
								 HAL_GPIO_ReadPin(SWHUCK2_GPIO_Port, SWHUCK2_Pin) == 1))
								{
									OldButtPress1 =1;
									OldButtPress2 =1;
								}
						//if 3,4 buttn pushed
						if ((HAL_GPIO_ReadPin(SWHUCK3_GPIO_Port, SWHUCK3_Pin)&&  				\
								 HAL_GPIO_ReadPin(SWHUCK4_GPIO_Port, SWHUCK4_Pin) == 1))
								{
									OldButtPress3 =1;
									OldButtPress4 =1;
								}
						//if 1, 2 buttn released
						if ((HAL_GPIO_ReadPin(SWHUCK1_GPIO_Port, SWHUCK1_Pin)&&  				\
								 HAL_GPIO_ReadPin(SWHUCK2_GPIO_Port, SWHUCK2_Pin) == 0)&&			\
								(OldButtPress1 && OldButtPress2 == 1))
								{
									OldButtPress1 =0;
									OldButtPress2 =0;
									// send: Box lost or released
										TxHeader.StdId = 0x030C;
										TxHeader.RTR = CAN_RTR_REMOTE;
										TxHeader.DLC = 0;
										
										HAL_CAN_AddTxMessage(&hcan, &TxHeader, 0, &TxMailbox);
								}
						//if 3, 4 buttn released
						if ((HAL_GPIO_ReadPin(SWHUCK3_GPIO_Port, SWHUCK3_Pin)&&  				\
								 HAL_GPIO_ReadPin(SWHUCK4_GPIO_Port, SWHUCK4_Pin) == 0)&&   \
									(OldButtPress3 && OldButtPress4 == 1))
								{
									OldButtPress3 =0;
									OldButtPress4 =0;
									// send: Box lost or released
										TxHeader.StdId = 0x030C;
										TxHeader.RTR = CAN_RTR_REMOTE;
										TxHeader.DLC = 0;
										
										HAL_CAN_AddTxMessage(&hcan, &TxHeader, 0, &TxMailbox);
								}
						}

}	

void CalobrationPawSize(void)
{
	if ((RxFlags & 1<<12) == (uint16_t)1<<12)
	{	
		uint32_t TimeOff = HAL_GetTick();
		uint8_t flaglag = 0;
		PawSize = 700;
		while ((TimeOff + 40000 > HAL_GetTick())&& flaglag == 0)
		{
			/////////!!!!///////
			// need makes correction for parallel paws
			PawSize++;
			TIM4 -> CCR1 = PawSize;
			TIM4 -> CCR2 = PawSize;
			HAL_Delay(6);
			
			if(HAL_GPIO_ReadPin(SWHUCK1_GPIO_Port, SWHUCK1_Pin)&&\
				 HAL_GPIO_ReadPin(SWHUCK2_GPIO_Port, SWHUCK2_Pin)&&\
				 HAL_GPIO_ReadPin(SWHUCK3_GPIO_Port, SWHUCK3_Pin)&&\
				 HAL_GPIO_ReadPin(SWHUCK4_GPIO_Port, SWHUCK4_Pin) == 1) 
			///////////////////////////////////////////////////////
			//  here must be function correction for parallels paws
			///////////////////////////////////////////////////////
			{
				flaglag = 1;
				PawMinSize = PawSize;
				TIM4 -> CCR1 = 700;
				TIM4 -> CCR2 = 700;
			}
		}
		//////////////////////////////////
		// Here must be colibration max range between paws
		// Use two limit switch on the 12 and 34 side
		
		
		///// send msg section
		uint8_t TxData[4] = {0};
				
				TxData[0] =	PawMinSize >> 8;
				TxData[1] =	PawMinSize;
				TxData[2] =	PawMaxSize >> 8;
				TxData[3] =	PawMaxSize;
				
				TxHeader.StdId = 0x030E;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.DLC = 4;
			
				//send msg and reset flg
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) RxFlags &= ~((uint16_t)1<<12);
	}	
}

void CalibrationMinDelayAndRange(void)
{
		// colibration of PawsMaxSpeed (MinDelay) and PawMaxPosition and PawMinPosition 
			if ((RxFlags & 1<<11) == (uint16_t)1<<11)
			{
				///////////////////////////
				///here mut be functions of colibrtion
				///////////////////////////
				///in time of calibration Voltage on the Motors must be slow 
				///////////////////////////
				
				//send msg section
					uint8_t TxData[8] = {0};
				
				TxData[0] =	PawMinPosition >> 8;
				TxData[1] =	PawMinPosition;
				TxData[2] =	PawMaxPosition >> 8;
				TxData[3] =	PawMaxPosition;
				TxData[4] =	MinDelay >> 24;
				TxData[5] =	MinDelay >> 16;
				TxData[6] =	MinDelay >> 8;
				TxData[7] =	MinDelay;
				
				TxHeader.StdId = 0x030D;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.DLC = 8;
			
				//send msg and reset flg
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) RxFlags &= ~((uint16_t)1<<11);
			}
			
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
	return 1;
}

void ScanBox (void)
{
if ((RxFlags & 1<<14) == (uint16_t)1<<14)
	{
		static uint32_t DelayTime    = 0;
		static uint8_t status = 0;
				if(Goal>0)
					{
						if ( (HAL_GPIO_ReadPin(DETECT12_GPIO_Port,DETECT12_Pin) == 1)&& (DelayTime <20000) ) DelayTime ++;
						if ( (HAL_GPIO_ReadPin(DETECT12_GPIO_Port,DETECT12_Pin) == 0)&& (DelayTime>0) ) DelayTime --;
						
						if ( (DelayTime == 20000) && (status == 0) )
							{
								status = 1;
								//DelayTime = 0;					
								CanTxMsg(PositionForTransmit, 0x030E, 2);
							//		GPIOC->ODR = 0 << 13;HAL_Delay(200);GPIOC->ODR = 1 << 13;HAL_Delay(200);
							}
					
						if ( (DelayTime == 0) && (status == 1) )
							{
								status = 0;
								//DelayTime = 0;					
								CanTxMsg(PositionForTransmit,0x030E,2);
								RxFlags &= ~((uint16_t)1<<14);
								//////for debug////////
								//GPIOC->ODR = 1 << 13;
								//////end debug///////
						//		GPIOC->ODR = 0 << 13;HAL_Delay(200);GPIOC->ODR = 1 << 13;HAL_Delay(200);
							}
					
					 }
				if(Goal<0)
					{ 
						////////////////////////////////
						///here must be like whan(goal>0)
						///|||||||||||||||||||||||||||||
						
						if ( (HAL_GPIO_ReadPin(DETECT34_GPIO_Port,DETECT12_Pin) == 1)&& (DelayTime <20000) ) DelayTime ++;
						if ( (HAL_GPIO_ReadPin(DETECT34_GPIO_Port,DETECT12_Pin) == 0)&& (DelayTime>0) ) DelayTime --;
						
						if ( (DelayTime == 20000) && (status == 0) )
							{
								status = 1;				
								CanTxMsg(PositionForTransmit, 0x030E, 2);
							}
					
						if ( (DelayTime == 0) && (status == 1) )
							{
								status = 0;			
								CanTxMsg(PositionForTransmit,0x030E,2);
								RxFlags &= ~((uint16_t)1<<14);
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
