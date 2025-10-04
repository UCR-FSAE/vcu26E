/*
 * inverter.c
 *
 *  Created on: Oct 3, 2025
 *      Author: Justin Im
 */

#include "inverter.h"

uint32_t appsRaw1;
uint32_t appsRaw1Max;
uint32_t appsRaw1Min;

uint32_t appsRaw2;
uint32_t appsRaw2Max;
uint32_t appsRaw2Min;

uint32_t appsCalculated;

float appsConverted1 = 0.0f;
float appsConverted2 = 0.0f;

extern ADC_HandleTypeDef hadc1;

extern CAN_HandleTypeDef hcan1;
static uint16_t torqueCommand = 0;
static uint16_t prevTorqueCommand = 0;

void Inverter_Init(void) {
	  Inverter_DisableInverter();
	  HAL_Delay(100);
	  Inverter_ClearInverterFaults();
	  HAL_Delay(100);
	  Inverter_EnableInverter();
	  HAL_Delay(100);

	  // read can messages for inverter awake and then toggle a status led
}

/**
  * @brief  Process Inverter main functionality (to be called periodically)
  * @retval None
  */
void Inverter_Process(void) {
	// ADC pedal readings
	if (HAL_ADC_Start(&hadc1) != HAL_OK) { Error_Handler(); }

	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		appsRaw1 = HAL_ADC_GetValue(&hadc1);
		appsRaw2 = HAL_ADC_GetValue(&hadc1);
	}
	else {
		appsRaw1 = 0;
		appsRaw2 = 0;
		HAL_ADC_Stop(&hadc1);
	}

	appsConverted1 = (float)(appsRaw1)/ (float)(4096);
	appsConverted2 = (float)(appsRaw2) / (float)(4096);

	appsCalculated = ((appsConverted1 + appsConverted2) / 2);


	if (appsCalculated >= 0.4 && appsCalculated <= 1.0) {
//		if (torqueCommand == 0) { Inverter_TransmitCANMessage(0, Inverter_DIRECTION_FORWARD, Inverter_INVERTER_DISABLE); }
//		else { Inverter_TransmitCANMessage(400, Inverter_DIRECTION_FORWARD, Inverter_INVERTER_ENABLE); }
		HAL_GPIO_WritePin(GPIOB, LD1_Pin, SET);
		HAL_GPIO_WritePin(GPIOB, LD2_Pin, RESET);
		Inverter_TransmitCANMessage(400, Inverter_DIRECTION_FORWARD, Inverter_INVERTER_ENABLE);
	}
	else if (appsCalculated >= 0.0 && appsCalculated <= 0.3) {
		HAL_GPIO_WritePin(GPIOB, LD1_Pin, SET);
		HAL_GPIO_WritePin(GPIOB, LD2_Pin, RESET);
		Inverter_TransmitCANMessage(0, Inverter_DIRECTION_FORWARD, Inverter_INVERTER_ENABLE);
	}
	else {
		HAL_GPIO_WritePin(GPIOB, LD1_Pin, RESET);
		HAL_GPIO_WritePin(GPIOB, LD2_Pin, SET);
	}


}

/**
  * @brief  Transmit CAN message to inverter
  * @param  torque: Torque command value (0-32767)
  * @param  direction: Direction command (0=forward, 1=reverse)
  * @param  inverterEnable: Inverter enable state (0=disable, 1=enable)
  * @retval None
  */
void Inverter_TransmitCANMessage(uint16_t torque, uint8_t direction, uint8_t inverterEnable)
{
  CAN_TxHeaderTypeDef txHeader;
  uint8_t txData[8];
  uint32_t txMailbox;
  HAL_StatusTypeDef status;

  /* Configure transmission */
  txHeader.StdId = Inverter_INVERTER_COMMAND_ID;
  txHeader.ExtId = 0;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 8;
  txHeader.TransmitGlobalTime = DISABLE;

  /* Pack torque command (little-endian) */
  txData[0] = (uint8_t)(torque & 0xFF);
  txData[1] = (uint8_t)((torque >> 8) & 0xFF);

  /* Speed command (0 for torque control mode) */
  txData[2] = 0;
  txData[3] = 0;

  /* Direction and inverter control */
  txData[4] = direction;
  txData[5] = inverterEnable;

  /* Torque limits (using default) */
  txData[6] = 0;
  txData[7] = 0;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {

  }
  else {
	status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

	if (status != HAL_OK) {
		HAL_GPIO_WritePin(GPIOB, LD3_Pin, RESET);
	    if (HAL_CAN_AbortTxRequest(&hcan1, txMailbox) != HAL_OK) { Error_Handler(); }
	}
	else { HAL_GPIO_WritePin(GPIOB, LD3_Pin, SET); }
  }
}

/**
  * @brief  Enable inverter
  * @retval None
  */
void Inverter_EnableInverter(void)
{
  Inverter_TransmitCANMessage(0, Inverter_DIRECTION_FORWARD, Inverter_INVERTER_ENABLE);
}

void Inverter_DisableInverter(void)
{
  Inverter_TransmitCANMessage(0, Inverter_DIRECTION_FORWARD, Inverter_INVERTER_DISABLE);
}


void Inverter_ClearInverterFaults(void)
{
	  CAN_TxHeaderTypeDef txHeader;
	  uint8_t txData[8];
	  uint32_t txMailbox;
	  HAL_StatusTypeDef status;

	  /* Configure transmission */
	  txHeader.StdId = Inverter_INVERTER_CLEAR_ID;
	  txHeader.ExtId = 0;
	  txHeader.IDE = CAN_ID_STD;
	  txHeader.RTR = CAN_RTR_DATA;
	  txHeader.DLC = 8;
	  txHeader.TransmitGlobalTime = DISABLE;

	  txData[0] = (uint8_t) 0x14; // command id for clearing fault
	  txData[1] = 0;

	  txData[2] = 1; // command to clear faults
	  txData[3] = 0;

	  txData[4] = 0;
	  txData[5] = 0;

	  txData[6] = 0;
	  txData[7] = 0;

	  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
	    HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0);
	    HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX1);
	    HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX2);
	  }

	  status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

	  if (status != HAL_OK) {
	    if (HAL_CAN_AbortTxRequest(&hcan1, txMailbox) != HAL_OK) {
	      Error_Handler();
	    }
	  }
}
