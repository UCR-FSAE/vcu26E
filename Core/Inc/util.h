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
void ADC_APPSCollection(uint32_t *readings);
uint32_t ADC_BSECollection();

// Torque Command Calculation Functions
float Drive_CalculateTorqueCommand(uint32_t appsRaw);
float Drive_CalculateBrakesActivation(uint32_t bseRaw);

// Plausibility Check Functions
#ifndef TIMER_STRUCT
#define TIMER_STRUCT

typedef struct {
    uint32_t curTick;
    uint32_t duration;
    uint8_t hasStarted;
} Timer;

#endif

char BSE_ImplausibilityCheck(bseRaw, &bseTimer);
void timerStart(Timer *bseTimer, uint32_t duration);
void timerReset(Timer *bseTimer);
char timerExpires(Timer *bseTimer);


// FIlter Functions
uint32_t APPS_SlewFilter(uint32_t appsADC);




#endif /* INC_UTIL_H_ */
