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
extern ADC_HandleTypeDef hadc1;


void RecalibratePedals();

// ADC Collection
void ADC_APPSCollection(float *readings);
float ADC_BSECollection();

// ADC Percentage Calculation Functions
float APPS_CalculateActivationPercentage(float adc, char channel);

// Torque Command Calculation Functions
float Drive_CalculateTorqueCommand(float appsRaw);
float Drive_CalculateBrakesActivation(float bseRaw);

// Plausibility Check Functions
#ifndef TIMER_STRUCT
#define TIMER_STRUCT

typedef struct {
    uint32_t curTick;
    uint32_t duration;
    uint8_t hasStarted;
    char hasFailed;
    uint32_t failTick;
} Timer;

#endif

char APPS_ImplausibilityCheck(Timer *t, float appsFiltered1, float appsFiltered2);
char BSE_ImplausibilityCheck(Timer *bseTimer, float bseRaw);
void timerStart(Timer *t, uint32_t duration);
void timerReset(Timer *t);
char timerExpires(Timer *t);

// FIlter Functions
float APPS_SlewFilter(float appsADC, char channel);


#endif /* INC_UTIL_H_ */
