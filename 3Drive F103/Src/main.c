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
	#define    DWT_CYCCNT    *(volatile uint32_t*)0xE0001004
  #define    DWT_CONTROL   *(volatile uint32_t*)0xE0001000
  #define    SCB_DEMCR     *(volatile uint32_t*)0xE000EDFC

/* USER CODE END PM */

	

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

//uint8_t UARTTxBuffer[25] = {0};
volatile uint8_t UARTRxBuffer[25] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
CAN_TxHeaderTypeDef   TxHeader;
CAN_TxHeaderTypeDef   TxHeaderTo2;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               CANRxData[8];
uint32_t              TxMailbox;

//variable for drive
	volatile uint32_t Goal = 0; 					// Goal of cart position 
	volatile uint32_t Position = 0;   		// position whaere cart located
	volatile uint32_t MinDelay = 27000;//normal 20000// Max Speed (delay between toggles step pin than speed is MAX)
	volatile uint32_t StepDelay = 0;			// delay between steps
	volatile uint16_t Boost = 12000; 			// 1 = MAX boost 65535 = MIN boost 
	volatile uint32_t MinSpeed = 600;	  // move graphic boost function to left side. Whan value == 0 boost function do not move 

//variales for calibration 
	volatile uint32_t MaxGoal =0;
	GPIO_PinState  MoveDown = GPIO_PIN_RESET, MoveUp = GPIO_PIN_SET; // Direction for moving Up or Down
	volatile uint8_t CalibrationFlag = 0;

//variables	for CAN connection

	// variables for 2nd block
	volatile uint32_t Y2MinDelaySpeed = 0;
	volatile uint32_t Y2DelaySpeed = 0;
	volatile uint32_t Y2MaxPosition = 0;
	volatile uint32_t Y2Position = 0;
	volatile uint16_t Y2RxFlags = 0;


	//variables for 1st block
	volatile int16_t  Z1Position = 0;
	volatile uint16_t Z1PawSize = 0;
	volatile uint16_t Z1PawSizeTuch = 0;
	volatile uint16_t Z1PawPosRange = 0;
	volatile uint16_t Z1PawNegRange = 0;
	volatile uint32_t Z1PawMinDelay = 0;
	volatile int16_t  Z1ConvRot = 0;
	volatile uint16_t Z1RxFlags = 0;
	volatile uint8_t  Z1GapPawSize = 50; // this value add to PawSize for mowing paws in Gap between boxes
	
// variables for hucking box	
	volatile int16_t Z1BoxPos = 0;
	volatile int16_t Z1StartBox = 0;
	volatile int16_t Z1EndBox = 0;
	volatile int16_t Z1ExpectedGoal = 0;
	volatile int16_t Z1code = 0;
	
// Distance detectors from rangefinder
	volatile uint8_t  Z1Distance1 =0;
	volatile uint8_t  Z1Distance2 =0;
	volatile uint8_t  Z1Distance3 =0;
	volatile uint8_t  Z1Distance4 =0;

	//structure for sheduling of route
	//if command code == 1
	struct DataRout
	{
		uint32_t CoordGoalX;
		uint32_t CoordGoalY;
		int16_t  CoordGoalZ;
		uint16_t SizeOfGoal;
		uint16_t DepthOfGoal;
		uint32_t CoordDropX;
		uint32_t CoordDropY;
		int16_t  CoordDropZ;
	}RxDataRout[2] = {0}; //massive of 240 bytes SRAM;
	
	uint16_t RoadSteps =0;

	//struct for variables for sheduling loder
	//if command code == 2
	struct SShedulingLoader
	{
		uint32_t GetXPos;
		uint32_t GetYPos;
		uint16_t PawSizeGet;
	}LoadShedul;
	
	uint8_t LoadFlags = 0;
	
	struct SMove
	{
		uint32_t CoordGoalX;
		uint32_t CoordGoalY;
	} MoveShedul;
	
	uint8_t MoveFlag;
	
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
	
	/////Variables

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET); // turn Off drivers
	
	GPIOC->ODR = 0 << 13;HAL_Delay(3000);GPIOC->ODR = 1 << 13;//HAL_Delay(1500);
		
	
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	HAL_UART_Receive_DMA(&huart1, UARTRxBuffer, sizeof(UARTRxBuffer));

//CanTxMsg(0,0x0201,0);
//while (((Y2RxFlags & 1<<1) != ((uint16_t)1<<1))){GPIOC->ODR = 0 << 13;HAL_Delay(100);}
		uint8_t Msg1Ok[] = {"Start progran, whait command!"};
		HAL_UART_Transmit_DMA(&huart1, Msg1Ok, sizeof(Msg1Ok));		
	
	
				
					
  while (1)
  {
		ShedulerOfLoad();
		ShedulerOfRoad();
		ShedulerOfMove();
		MotorControl();
		CalobrationDistance();
		
		//for debug///////
		//GPIOC->ODR = 1 << 13;
		///end debug///////////
		
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
  hcan.Init.Prescaler = 90;
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
/* Configure the CAN Filter */
	 CAN_FilterTypeDef  sFilterConfig;
	//filter bank for massege from 1 block
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh = 0x0300 << 5;
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

	// fiter bank for massege from 2 block
  sFilterConfig.FilterBank = 1;
  sFilterConfig.FilterIdHigh = 0x0700 << 5;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0F00 << 5;
  sFilterConfig.FilterMaskIdLow = 0xFFFF;

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
	
 // TxHeaderTo1.StdId = 0x3100;
 // TxHeaderTo1.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  //TxHeaderTo1.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
		
  /* USER CODE END CAN_Init 2 */

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
  huart1.Init.BaudRate = 2400;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, LED_Pin|ZOMMER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_Pin|STEP_Pin|ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LimSwHIGH_Pin|LimSwLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin ZOMMER_Pin */
  GPIO_InitStruct.Pin = LED_Pin|ZOMMER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin STEP_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LimSwHIGH_Pin LimSwLOW_Pin */
  GPIO_InitStruct.Pin = LimSwHIGH_Pin|LimSwLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t i = 0;
	switch (UARTRxBuffer[0])
		{
			case 0://select free cell and writing coordinates for sheduler
				//while(RxDataRout[i].CoordGoalX !=0)i++;
				RxDataRout[i].CoordGoalX = ((UARTRxBuffer[1]<<24)|(UARTRxBuffer[2]<<16)|(UARTRxBuffer[3]<<8)|(UARTRxBuffer[4]));
				RxDataRout[i].CoordGoalY = ((UARTRxBuffer[5]<<24)|(UARTRxBuffer[6]<<16)|(UARTRxBuffer[7]<<8)|(UARTRxBuffer[8]));
				RxDataRout[i].CoordGoalZ = ((UARTRxBuffer[9]<<8)|(UARTRxBuffer[10]));
				RxDataRout[i].SizeOfGoal = ((UARTRxBuffer[11]<<8)|(UARTRxBuffer[12]));
				RxDataRout[i].DepthOfGoal =((UARTRxBuffer[13]<<8)|(UARTRxBuffer[14]));
				RxDataRout[i].CoordDropX = ((UARTRxBuffer[15]<<24)|(UARTRxBuffer[16]<<16)|(UARTRxBuffer[17]<<8)|(UARTRxBuffer[18]));
				RxDataRout[i].CoordDropY = ((UARTRxBuffer[19]<<24)|(UARTRxBuffer[20]<<26)|(UARTRxBuffer[21]<<8)|(UARTRxBuffer[22]));
				RxDataRout[i].CoordDropZ = ((UARTRxBuffer[23]<<8)|(UARTRxBuffer[24]));
				RoadSteps=1;
			break;
			case 1:// move to X, Y coordinates
				MoveShedul.CoordGoalX = ((UARTRxBuffer[1]<<24)|(UARTRxBuffer[2]<<16)|(UARTRxBuffer[3]<<8)|(UARTRxBuffer[4]));
				MoveShedul.CoordGoalY = ((UARTRxBuffer[5]<<24)|(UARTRxBuffer[6]<<16)|(UARTRxBuffer[7]<<8)|(UARTRxBuffer[8]));
				MoveFlag = 1;
			break;
			case 2://scan and gripping the box
				LoadShedul.GetXPos = ((UARTRxBuffer[1]<<24)|(UARTRxBuffer[2]<<16)|(UARTRxBuffer[3]<<8)|(UARTRxBuffer[4]));
				LoadShedul.GetYPos = ((UARTRxBuffer[5]<<24)|(UARTRxBuffer[6]<<16)|(UARTRxBuffer[7]<<8)|(UARTRxBuffer[8]));
				LoadShedul.PawSizeGet = ((UARTRxBuffer[9]<<8)|(UARTRxBuffer[10]));
				Z1ExpectedGoal = ((UARTRxBuffer[11]<<8)|(UARTRxBuffer[12]));
				LoadFlags = 1;
			break;
			case 14: // calibrate X axis	
				CalibrationFlag = 1;
			break;
			case 18: // calibrate Y axis
				CanTxMsg(0,0x0201,0);
			break;
			
		}
		
		HAL_UART_Receive_DMA(&huart1, UARTRxBuffer, sizeof(UARTRxBuffer));
		////////this is for prototype version: 
					Z1RxFlags = 0;
		//for(char i = 0; i==25; i++)UARTRxBuffer[i] = 0;
		/////////////////////////////
		
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CANRxData);

	// if massege from 1st block
	if ((RxHeader.StdId & 0xFF00) == 0x0300)
	{
		switch (RxHeader.StdId & 0x00FF)
		{
			case 1: //data u32
				// answer valie of distanse 1,2,3,4	
				// resive data
				
				Z1RxFlags |= 1<<1;
				
				Z1Distance1 =	CANRxData[0];
				Z1Distance2 =	CANRxData[1];
				Z1Distance3 = CANRxData[3];
				Z1Distance4 = CANRxData[4];
			break;
			case 2:
				// Not Use
				
			break;
			case 3: //data u16
				// paws reached position "Z1Position"
				//GPIOC->ODR = 0 << 13;
			
				Z1RxFlags |= 1<<3;
				Z1Position = ( ((uint16_t) CANRxData[0]<<8) | CANRxData[1]);
			break;
			case 4: //data u16
				// capture successful, the box size is 
			
				Z1RxFlags |= 1<<4;
				Z1PawSizeTuch = ( ((uint16_t) CANRxData[0]<<8) | CANRxData[1]);	
			break;
			case 5:
				// paws in "zero" position

				Z1RxFlags |= 1<<5;
			break;
			case 6://data u16
				// paws have taken size "Z1PawSize"
				
				//for debug////////////////////////////////////////////////
			//GPIOC->ODR = 1<<13;
				// end debug//////////////////////////////////////////////
				
				Z1RxFlags |= 1<<6;
				Z1PawSize = ( ((uint16_t) CANRxData[0]<<8) | CANRxData[1] );
			break;
			case 7://data u16
				// conveyor have taken pozition "Z1ConPos"
				
				Z1RxFlags |= 1<<7;
				Z1ConvRot = ( ((uint16_t) CANRxData[0]<<8) | CANRxData[1] );
			break;
			case 8:
				//ERROR object btween paws not detected
				//Handler
				UARTTxMsg(201,1);
				Z1RxFlags |= 1<<8;
			break;
			case 9:
				//FATAL ERROR Box is not captured!
				// STOP ROAD PROGRAM!
				UARTTxMsg(200,1);// meens the FatalError! Box is not Captured
				//RoadSteps = 0;// STOP ROAD PROGRAM!
				
				Z1RxFlags |= 1<<9;
			break;
			case 0x0A:
				//ERROR paws lost zero pozition. need colibration
				// Hendler
				
				Z1RxFlags |= 1<<10;
			break;
			case 0x0B:
				// ERROR lost power on 1st- block!
				//Hendler
				
				Z1RxFlags |= 1<<11;
			break;
			case 0x0C:
				//ERROR! Box is lost until transportation
				Z1RxFlags |= 1<<12;
				// handler
			break;
			case 0x0D://data u64
				//calibration of speed and range complite, 
				//Max Negative range '+' 16b, Max Posetive range '-'16b, Max Speed(Min delay)32b
				Z1RxFlags |= 1<<13;
				
				Z1PawNegRange = ( ((uint16_t) CANRxData[0]<<8) | CANRxData[1] );
				Z1PawPosRange = ( ((uint16_t) CANRxData[2]<<8) | CANRxData[3] );
				Z1PawMinDelay = ( ((uint32_t) CANRxData[4]<<24) | ((uint32_t) CANRxData[5]<<16) | ((uint32_t) CANRxData[6]<<8) | CANRxData[7]);
			break;
			case 0x0E://data u32
				// Start or End of BoX while scaning
				Z1RxFlags |= 1<<14;
			
			////for debug///////
		//GPIOC->ODR = 0 << 13;//for(uint16_t i = 0;i<65535;i++){};
			////end debug////////	
			
				Z1BoxPos = ( ((uint16_t) CANRxData[0]<<8) | CANRxData[1] );
			break;
		}
		
	}
	// if maege from 2nd block
	if (((RxHeader.StdId & 0xFF00) == 0x0700))
	{
		switch (RxHeader.StdId & 0x00FF)
		{
			case 1:
				// calibration max position complete, max position is "Y2MaxPosition"
				
				Y2RxFlags |= 1<<1;
				Y2MaxPosition = ( ((uint32_t) CANRxData[0]<<24) | ((uint32_t) CANRxData[1]<<16) | ((uint32_t) CANRxData[2]<<8) | CANRxData[3]);
				HAL_UART_Transmit_DMA(&huart1,CANRxData,4);
				for(uint16_t i =0; i<2000; i++){}
			break;
			case 2:
				// Calibration Max Speed (MinDelay) complite, min delay is "Y2MinDelaySpeed"
				
				Y2RxFlags |= 1<<2;
				Y2MinDelaySpeed = ( ((uint32_t) CANRxData[0]<<24) | ((uint32_t) CANRxData[1]<<16) | ((uint32_t) CANRxData[2]<<8) | CANRxData[3]);
			break;
			case 3:
				// position "H" reached
				
				
			////////for debug////////
			GPIOC ->ODR = 1 <<13;
			//////////////////end debug
				
				Y2RxFlags |= 1<<3;
				Y2Position =( ((uint32_t) CANRxData[0]<<24) | ((uint32_t) CANRxData[1]<<16) | ((uint32_t) CANRxData[2]<<8) | CANRxData[3]);
				// Flag position reached
			break;
			case 4:
				//ERROR! lost zero point. Need colibration max position(command 1)
				
				Y2RxFlags |= 1<<4;
				// Flag error/ lot zero point. Hendler!
			break;
			case 5:
				//ERROR! lost power on 2nd block!
				
				Y2RxFlags |= 1<<5;
				// hendler
			break;
		}
	}
	
	/* Display LEDx */
  /*if ((RxHeader.StdId == 0x0030) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 8))
  {
		GPIOC->ODR = RxData[0] << 13;
	}
	*/
}

void HAL_CAN_TxMailbox0CompleteCallback (CAN_HandleTypeDef *hcan)
{
	 //GPIOC->ODR = 0 << 13;
//	HAL_Delay(100);
	//GPIOC->ODR = 1 << 13;
		
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
		HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);
		
		// Direction computing
		if(Goal < Position) 
			{
				Distance = Position - Goal;
				HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, MoveDown);
			}											
		else															
			{																
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
					HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
					return 0; // Function Finish Work
			}
			return 1;	// Function in Work
		}
	return 0; // Function redy to Work
}



void ShedulerOfRoad(void)
{	
	//if((RxDataRout[0].CoordGoalX != 0) && (Z1RxFlags == 0) && RoutSteps==0	) RF.Step1 = 1; 
		
	//1 step start 
	if	(RoadSteps == 1)
		{
			Goal = RxDataRout[0].CoordGoalX;//moving axis X
			CanTxMsg (RxDataRout[0].CoordGoalY, 0x0203, 4);// moving axis Y
			CanTxMsg (RxDataRout[0].SizeOfGoal - Z1GapPawSize, 0x0106, 2);//reduction paws size SizeOfGoal + Gap
			RoadSteps = 2;
			///for debug/////////////////
			GPIOC->ODR = 0 << 13;
			/////end debug///////////////
		}
	//1 step finish, 2 step start
	if ( ((Y2RxFlags & 1<<3) == ((uint16_t)1<<3)) && ((Z1RxFlags & 1<<6)== ((uint16_t)1<<6)) && (Goal == Position) && (RoadSteps==2) )
		{	
			CanTxMsg(RxDataRout[0].CoordGoalZ, 0x0103, 2);//moving paws to box 
			RoadSteps = 3;
			Y2RxFlags &= ~(1<<3);
			Z1RxFlags &= ~(1<<6);
			///for debug/////////////////
			//GPIOC->ODR = 1 << 13;
			/////end debug///////////////
		} 
	//2 step finish, 3 step start	
					if ( ((Z1RxFlags & 1<<9)==((uint16_t)1<<9)) )
							{
								RoadSteps = 0;// STOP ROAD PROGRAM!
								CanTxMsg(0, 0x0105, 0);// Go paws to "0" position
								Z1RxFlags &= ~(1<<9);
							}
	if ( ((Z1RxFlags & 1<<3)==((uint16_t)1<<3)) && (RoadSteps==3) )
		{
			CanTxMsg(0,0x0104, 0);//captire the Box
			Z1RxFlags &= ~(1<<3);
			RoadSteps=4;
		}
	//3 step finish, 4 step start
	if ( ((Z1RxFlags & 1<<4)==((uint16_t)1<<4)) && (RoadSteps== 4) )
		{
			//uint8_t msg512[] = {"Box Capture success! Size:"};
			//HAL_UART_Transmit_DMA (&huart1,msg512,sizeof(msg512));
			HAL_UART_Transmit_DMA (&huart1,CANRxData/*(uint8_t*)&Z1PawSizeTuch*/,2);
			CanTxMsg(0, 0x0105, 0);// Go paws to "0" position
			Z1RxFlags &= ~(1<<4);
			RoadSteps =5;
		}
		// while RF.Step5 == 1 chack ERRORs Paws
	//4 step finish, 5 step start	
	if ( ((Z1RxFlags & 1<<5)==((uint16_t)1<<5)) && (RoadSteps == 5) )
		{
			CanTxMsg(RxDataRout[0].CoordDropY,0x0203,4);// moving axis Y to drop position
			Goal = RxDataRout[0].CoordDropX;// moving axis X to drop position
			Z1RxFlags &= ~(1<<5);
			RoadSteps = 6;
		}
	//5 step finish, 6 step start		
	if ( ((Y2RxFlags & 1<<3) == ((uint16_t)1<<3)) && (Goal == Position) && (RoadSteps == 6) )
		{
			CanTxMsg(RxDataRout[0].CoordDropZ, 0x0103,2);//moving paws to drop position 
			Y2RxFlags &= ~(1<<3);
			RoadSteps = 7;
		}
	 if ( ((Z1RxFlags & 1<<3)==((uint16_t)1<<3)) && (RoadSteps == 7) )
		{
			CanTxMsg (Z1PawSizeTuch - Z1GapPawSize, 0x0106, 2);//reduction paws size Z1PawSizeTuch + Gap
			Z1RxFlags &= ~(1<<3);
			RoadSteps = 8;
		}
	 if ( ((Z1RxFlags & 1<<6)== ((uint16_t)1<<6)) && (RoadSteps == 8) )
	  {
			CanTxMsg (0, 0x0105, 0);// GO paws to "0" Position!
			RoadSteps = 9;
		}
		if ( ((Z1RxFlags & 1<<5)==((uint16_t)1<<5)) && (RoadSteps == 9) )
		{
			Z1RxFlags &= ~(1<<5);
			uint8_t MsgOk[18] = {"Transfer complete!"};
			HAL_UART_Transmit_DMA(&huart1, MsgOk, 18);
			HAL_Delay(100);
			RxDataRout[0].CoordGoalX = 0;
			RoadSteps = 0;
		}
}




void ShedulerOfLoad (void)
{
	//if( (LoadShedul.GetXPos !=0) && (Z1RxFlags == 0) && (LoadFlags==0)	) LoadFlags = 1;
	
	if(LoadFlags == 1)		
	{
		////for debug///////
		//GPIOC->ODR = 0 << 13;for(uint16_t i = 0;i<65535;i++){};
			////end debug////////	
			Goal = LoadShedul.GetXPos;//moving axis X
			CanTxMsg (LoadShedul.GetYPos, 0x0203, 4);// moving axis Y
			CanTxMsg (LoadShedul.PawSizeGet, 0x0106, 2);//reduction paws size SizeOfGoal + Gap
			LoadFlags = 2;
	}
	if( (LoadFlags == 2)&& ((Y2RxFlags & 1<<3) == ((uint16_t)1<<3)) && ((Z1RxFlags & 1<<6)== ((uint16_t)1<<6)) && (Goal == Position) )
	{
		////for debug///////
	//	GPIOC->ODR = 0 << 13;for(uint16_t i = 0;i<65535;i++){};
			////end debug////////	
			CanTxMsg (0, 0x010E, 0); // command for start Scan
			CanTxMsg(Z1ExpectedGoal, 0x0103,2);//moving paws over expected position 
			Y2RxFlags &= ~(1<<3);
			Z1RxFlags &= ~(1<<6);
			LoadFlags=3;
	}
	if(	((Z1RxFlags & 1<<14)== ((uint16_t)1<<14))	&&(LoadFlags==3) )
	{
		if(Z1StartBox == 0 )
			{
					Z1StartBox = Z1BoxPos;
					uint8_t dat[2];
					dat[0] = Z1BoxPos>>8;
					dat[1] = Z1BoxPos;
					HAL_UART_Transmit_DMA(&huart1,dat,2);
					Z1RxFlags &= ~(1<<14);
					HAL_Delay(100);
					Z1BoxPos=0;
				//Z1StartBox=0xFFFF;
			}	
		else if	(Z1EndBox == 0 && Z1StartBox !=0)
			{
				 
				////for debug///////
		//for debug///////
	//	GPIOC->ODR = 1 << 13;
		///end debug///////////
			////end debug////////	
				Z1EndBox = Z1BoxPos;
				uint8_t dat2[2];
					dat2[0] = Z1BoxPos>>8;
					dat2[1] = Z1BoxPos;
				HAL_UART_Transmit_DMA(&huart1,dat2,2);
				Z1RxFlags &= ~(1<<14);
				HAL_Delay(100);
				LoadFlags=4;
				Z1BoxPos=0;
			}
		
	}		
	 if( (Z1RxFlags & 1<<3)== ((uint16_t)1<<3) && (LoadFlags == 4) )
	  {
			////for debug///////
	//	GPIOC->ODR = 0 << 13;for(uint16_t i = 0;i<65535;i++){};
			////end debug////////	
			CanTxMsg (0, 0x0105, 0);// GO paws to "0" Position!
			LoadFlags=5;
			Z1RxFlags &= ~(1<<3);
		}
		if ( ((Z1RxFlags & 1<<5)==((uint16_t)1<<5)) && (LoadFlags == 5) )
		{
			Z1RxFlags &= ~(1<<5);
			LoadFlags=0;
			Z1StartBox =0;
			Z1EndBox = 0;
			
			////for debug///////
		//GPIOC->ODR = 0 << 13;for(uint16_t i = 0;i<65535;i++){};
			////end debug////////	
//			LoadShedul.GetXPos =0;
//			LoadShedul.GetYPos = 0;
//			LoadShedul.PawSizeGet = 0;
//			Z1ExpectedGoal = 0;
		}
}

void ShedulerOfMove (void)
{
if (MoveFlag ==1)	
	{
	Goal = MoveShedul.CoordGoalX;//moving axis X
	CanTxMsg (MoveShedul.CoordGoalY,0x0203,4);// moving axis Y
	MoveFlag = 2;
	}
if ( (MoveFlag == 2) && (Position == Goal) && ((Y2RxFlags & 1<<3) == ((uint16_t)1<<3)) )	
{
	Y2RxFlags &= ~(1<<3);
	MoveFlag = 0;
	UARTTxMsg (101,1);
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
		
		


uint8_t CalobrationDistance (void)
{
	if ( CalibrationFlag == 1)
			{	// Calibration distance range
			
				uint32_t Counter = 0;
				uint32_t errorCounter =0;
				HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);	// turn on Driver
				HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_RESET);					// select DIR
				
					while ( ((HAL_GPIO_ReadPin(LimSwHIGH_GPIO_Port, LimSwHIGH_Pin) != GPIO_PIN_SET) && \
									 (HAL_GPIO_ReadPin(LimSwLOW_GPIO_Port,  LimSwLOW_Pin)  != GPIO_PIN_SET)) && \
									 (errorCounter < 0x0004FFFF) )
					 // stepping while Limit Switch will not pushed
					 //	For determinate low position of cart
					{
						HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET);
						HAL_Delay(1);
						HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin, GPIO_PIN_RESET);
						HAL_Delay(1);
						errorCounter++;
					}
/////////////////////////////////////////////////////////////
					if (HAL_GPIO_ReadPin(LimSwLOW_GPIO_Port, LimSwLOW_Pin) == GPIO_PIN_SET)
						{// if pushed LOW switch
							MoveDown = GPIO_PIN_RESET;
							MoveUp = GPIO_PIN_SET;
							HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, MoveUp);
							errorCounter = 0;
							Counter = 0;
							while ( (HAL_GPIO_ReadPin(LimSwHIGH_GPIO_Port, LimSwHIGH_Pin) != GPIO_PIN_SET) && (errorCounter < 0x0009FFFF) )
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
									UARTTxMsg(Position, 4);
									//HAL_Delay(200);	
										CalibrationFlag =0; // reset flag
										HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);	//turn off Driver
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
									UARTTxMsg(MaxGoal, 4);
									//HAL_Delay(200);	
										CalibrationFlag =0;
									 HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);	//turn off Driver
									return 0; // OK	
								}	
								
						}
			
				
			}	
		return 1;// ERRor
	}	


uint8_t UARTTxMsg(volatile uint32_t Data, volatile uint8_t size)
{		
	
	// FOR USE You shoude add global variables
	//static volatile uint8_t TxData4[4];
	
			if (size == 1)
				{
					static uint8_t TxData1[1];
					TxData1[0] = Data;
	
					//send msg and reset flg
					if(HAL_UART_Transmit_DMA(&huart1, TxData1, 1) == HAL_OK) return 0;
				}
			if (size == 2)
				{
					
					static uint8_t TxData2[2];
					TxData2[0] = Data>>8;
					TxData2[1] = Data;
					
					//send msg and reset flg
					if(HAL_UART_Transmit_DMA(&huart1, TxData2, 2) == HAL_OK)return 0;
				}	
			if (size == 4)
				{
					static uint8_t TxData4[4];
					
					TxData4[0] = Data>>24;
					TxData4[1] = Data>>16;
					TxData4[2] = Data>>8;
					TxData4[3] = Data;
				
					//send msg and reset flg
					if(HAL_UART_Transmit_DMA(&huart1, TxData4, 4) == HAL_OK) return 0;
				}
	return 1;
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
