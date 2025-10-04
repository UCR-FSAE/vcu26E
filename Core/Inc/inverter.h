/*
 * inverter.h
 *
 *  Created on: Oct 3, 2025
 *      Author: Justin Im
 */

#ifndef INC_INVERTER_H_
#define INC_INVERTER_H_

#include "main.h"



void Inverter_Init(void);
void Inverter_Process(float torqueCommand);
void Inverter_EnableInverter(void);
void Inverter_DisableInverter(void);
void Inverter_ClearInverterFaults(void);
void Inverter_ProcessAnalogInputs(void);
void Inverter_TransmitCANMessage(uint16_t torque, uint8_t direction, uint8_t inverterEnable);

#define Inverter_INVERTER_COMMAND_ID 0x0C0
#define Inverter_INVERTER_CLEAR_ID 0x0C1

#define Inverter_DIRECTION_FORWARD 0x00
#define Inverter_DIRECTION_REVERSE 0x01
#define Inverter_INVERTER_ENABLE  0x01
#define Inverter_INVERTER_DISABLE 0x00

#endif /* INC_INVERTER_H_ */
