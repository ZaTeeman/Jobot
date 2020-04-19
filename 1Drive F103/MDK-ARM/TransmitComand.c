
//this file cosist functions for transmit information to 3rd block by CAN line

/*
description of pins:
RF# (#:1,2,3,4) - RangeFinder pins: detected shelf 
PAW#CHECK (#: 1,2,3,4) - detector "0 " position of paws. 
DETECT# (#:12,34) - Detector box between paws
SWHUCK# (#:1,2,3,4) - Detector tuch paws to box
*/

#include "TransmitComand.h"

extern CAN_TxHeaderTypeDef TxHeader;
extern unsigned int RxFlags;
extern CAN_HandleTypeDef hcan;
extern int16_t Position;
/*
void RxAnswerFunction(void )	
{		
		// Anser on rangefinder request
		if ((RxFlags & 1<<1) == (uint16_t)1<<1)
		{
				uint8_t TxData[4] = {0};
				uint32_t TxMailbox = 0;
				
				TxData[0] =	HAL_GPIO_ReadPin(RF1_GPIO_Port, RF1_Pin); 
				TxData[1] =	HAL_GPIO_ReadPin(RF2_GPIO_Port, RF2_Pin); 
				TxData[2] =	HAL_GPIO_ReadPin(RF3_GPIO_Port, RF3_Pin); 
				TxData[3] =	HAL_GPIO_ReadPin(RF4_GPIO_Port, RF4_Pin); 
				
				
				TxHeader.StdId = 0x1301;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.IDE = CAN_ID_STD;
				TxHeader.DLC = 4;
				TxHeader.TransmitGlobalTime = DISABLE;
			
				HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
				//reset flag
				RxFlags &= ~((uint16_t)1<<1);
		}
		
		if ((RxFlags & 1<<4) == (uint16_t)1<<4)
		{
			if (Position > 0)
			{
			TIM4 -> CCR1 = 500;
			TIM4 -> CCR2 = 500;
			}
			RxFlags &= ~((uint16_t)1<<4);
		}
}

*/

