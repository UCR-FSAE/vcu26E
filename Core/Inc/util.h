/*
 * util.h
 *
 *  Created on: Jan 19, 2026
 *      Author: imjus
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include <main.h>

extern ADC_HandleTypeDef hadc3;


// ADC Collection
void ADC_APPSCollection(float *readings);
uint32_t ADC_BSECollection();

// Torque Command Calculation Functions
float Drive_CalculateTorqueCommand(float appsRaw);
float Drive_CalculateBrakesActivation(float bseRaw);

// Plausibility Check Functions

// FIlter Functions
float APPS_SlewFilter(float appsADC, char channel);




#endif /* INC_UTIL_H_ */
