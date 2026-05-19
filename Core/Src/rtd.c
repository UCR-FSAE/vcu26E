/*
 * rtd.c
 *
 *  Created on: Dec 26, 2025
 *      Author: Justin Im
 */

#include "rtd.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[8];

char InverterCheck() {
	Inverter_Init();
	if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
			if (RxHeader.StdId == Inverter_INVERTER_STATUS_ID && (RxData[6] & 0x01) == 0x01) { return 1; }
		}
	 }

	return 0;
}


char RTDCheck(float bseThreshold) {
	float bseRaw = ADC_BSECollection();

	// return 1 if RTD has been fulfilled
	if ((bseRaw > bseThreshold) &&
			!HAL_GPIO_ReadPin(Driver_Action_GPIO_Port, Driver_Action_Pin) ){
			//HAL_GPIO_ReadPin(Tractive_Active_GPIO_Port, Tractive_Active_Pin)) {

	  uint32_t startTick = HAL_GetTick();
	  HAL_GPIO_WritePin(GPIOB, RTD_Output_Pin, SET);
	  while(HAL_GetTick() - startTick < 1500) {}
	  HAL_GPIO_WritePin(GPIOB, RTD_Output_Pin, RESET);
	  return 1;
	}
	// return 0 if RTD has not been fulfilled
	return 0;
}
