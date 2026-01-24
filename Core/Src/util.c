/*
 * util.c
 *
 *  Created on: Jan 19, 2026
 *      Author: imjus
 */

#include <util.h>

// APPS Slew Filter Values
static float appsPrev = 0.0f;          // previous filtered voltage
static float appsSlewVPerS = 1.5f;     // max allowed V/sec change (tune as needed)
static float appsRaw0 = 0.0f, appsRaw1 = 0.0f, appsRaw2 = 0.0f; // rolling raw samples

// configuration and calibration variables
float appsMin = 		2.0; // 2v
float appsMax = 		3.95; // 3.8
float bseMin = 			1.58933; // 1v
float bseMax = 			3.25; // 3.25
char RTDActive = 		0; // bool for ready to drive
char InverterReady = 	0;
uint16_t delay = 		30;	// delay length in between loop executions


// APPS ADC Collection
void ADC_APPSCollection(uint32_t *readings) {
	HAL_ADC_Start(&hadc3);

	// ADC Input for APPS 1
	if (HAL_ADC_PollForConversion(&hadc3, 1) == HAL_OK) {
		readings[0] = (float) (HAL_ADC_GetValue(&hadc3) / (float) (4095.0)) * 5.0;
	}
	// ADC Input for APPS 2
	if (HAL_ADC_PollForConversion(&hadc3, 1) == HAL_OK) {
		readings[1] = (float) (HAL_ADC_GetValue(&hadc3) / (float) (4095.0)) * 5.0;
	}

	HAL_ADC_Stop(&hadc3);
}

// BSE ADC Collection
uint32_t ADC_BSECollection() {
	HAL_ADC_Start(&hadc3);
	if (HAL_ADC_PollForConversion(&hadc3, 1) == HAL_OK) {
		return ((float) (HAL_ADC_GetValue(&hadc3) / (float) (4095.0)) * 5.0);
	}
	else {
		return 0b1111111111111111111111111111111;
	}
}

// torque calculations
float Drive_CalculateTorqueCommand(uint32_t appsRaw) {
	return 2*(513.924 * (appsRaw) - 1080); // 0-1000
}

// brakes percentage calculations
float Drive_CalculateBrakesActivation(uint32_t bseRaw) {
	return (44.44 * (bseRaw-1));
}


// Slew Filter for APPS
uint32_t APPS_SlewFilter(uint32_t appsADC) {
    // simple median-of-3 to reject single-sample spikes
    // push new sample into history
    appsRaw0 = appsRaw1;
    appsRaw1 = appsRaw2;
    appsRaw2 = appsADC;
    // median-of-3 on copies so history isn't mutated
    float a = appsRaw0, b = appsRaw1, c = appsRaw2;
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    float appsMed = b;

    const float appsDt = 0.001f * delay; // delay is in ms
    float appsDv = appsMed - appsPrev;
    float appsMaxDv = appsSlewVPerS * appsDt;
    if (appsDv >  appsMaxDv) appsMed = appsPrev + appsMaxDv;
    if (appsDv < -appsMaxDv) appsMed = appsPrev - appsMaxDv;
    appsPrev = appsMed;

    // use filtered value going forward
    return appsMed;
}


char BSE_ImpausabilityCheck(Timer*bseTimer, float bseRaw)
{
 if (bseRaw < bseMin || bseRaw > bseMax){
      TimerStart(bseTimer, 100);
      }
            else { timerResets(bseTimer); 
            }
            
 if (bseRaw < bseMin || bseRaw > bseMax){
			  timerStart(bseTimer, 100);
		  }

          else {
        	  timerReset(bseTimer);
           }

 if (timeExpires(bseTimer)){
			  Inverter_DisableInverter();
		  }
    }

