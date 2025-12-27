/*
 * initialization.c
 *
 *  Created on: Dec 26, 2025
 *      Author: Justin Im
 */

#include "initialization.h"
extern char RTDActive;
extern char InverterActive;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[8];
extern CAN_HandleTypeDef hcan1;

char Inverter_Initialization() {
	// attempt to initialize the inverter
	Inverter_Init();

	// check CAN
	if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
			if(RxHeader.StdId == 0x0AA) {
				if (RxData[6] & 0x01) {
					for (int i = 0; i < 2; i++) {
					  HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
					  HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
					  HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
					  HAL_Delay(1000);
					}
					InverterActive = 1;
					return 1;
				}
			}
		}
	}
	return 0;
}
