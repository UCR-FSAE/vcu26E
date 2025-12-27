/*
 * rtd.c
 *
 *  Created on: Dec 26, 2025
 *      Author: Justin Im
 */

#include "rtd.h"

char RTDCheck(float bseGradient, float bseThreshold) {
	// return 1 if RTD has been fulfilled
	if (bseGradient > bseThreshold && HAL_GPIO_ReadPin(GPIOB, Driver_Action_Pin)) {
	  uint32_t startTick = HAL_GetTick();
	  HAL_GPIO_WritePin(GPIOB, RTD_Output_Pin, SET);
	  while(HAL_GetTick() - startTick < 1500) {}
	  HAL_GPIO_WritePin(GPIOB, RTD_Output_Pin, RESET);
	  return 1;
	}
	// return 0 if RTD has not been fulfilled
	return 0;
}
