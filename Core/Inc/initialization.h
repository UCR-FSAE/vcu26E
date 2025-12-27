/*
 * initialization.h
 *
 *  Created on: Dec 26, 2025
 *      Author: Justin Im
 */

#ifndef INC_INITIALIZATION_H_
#define INC_INITIALIZATION_H_

#include "main.h"

/* Attempts to initialize the inverter and validate via CANBUS.
 * Returns 0 on failure and 1 on success.*/
char Inverter_Initialization(void);

#endif /* INC_INITIALIZATION_H_ */
