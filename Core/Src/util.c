/*
 * util.c
 *
 *  Created on: Jan 19, 2026
 *      Author: imjus
 */

#include <util.h>

// APPS Slew Filter Values for apps 1
static float appsPrev1 = 0.0f;          							// previous filtered voltage
static float appsSlewVPerS1 = 1.5f;     							// max allowed V/sec change (tune as needed)
static float appsRaw01 = 0.0f, appsRaw11 = 0.0f, appsRaw21 = 0.0f; 	// rolling raw samplesstatic float appsPrev1 = 0.0f;

// APPS Slew Filter Values for apps 2
static float appsPrev2 = 0.0f;          							// previous filtered voltage
static float appsSlewVPerS2 = 1.5f;     							// max allowed V/sec change (tune as needed)
static float appsRaw02 = 0.0f, appsRaw12 = 0.0f, appsRaw22 = 0.0f; 	// rolling raw samples


// configuration and calibration variables
float appsMin = 		2.0; 		// 2v
float appsMax = 		3.95; 		// 3.8
float bseMin = 			1.58933; 	// 1v
float bseMax = 			3.25; 		// 3.25
char RTDActive = 		0; 			// bool for ready to drive
char InverterReady = 	0;
uint16_t delay = 		30;			// delay length in between loop executions


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

char APPS_ImplausibilityCheck(Timer *t, float appsFiltered1, float appsFiltered2) {
	t->hasFailed = 0;
	if (appsFiltered1/appsFiltered2 > 0.1)
		t->hasFailed = 1;
	if (appsFiltered1 > appsMax || appsFiltered1 < appsMin)
		t->hasFailed = 1;
	if (appsFiltered2 > appsMax || appsFiltered2 < appsMin)
		t->hasFailed = 1;

	if (t->hasFailed) {
		timerStart(t, 100);
	} else {
		timerReset(t);
	}

	if (timerExpires(t)){
		return 0;
	}
	return 1;
}

void timerStart(Timer *t, uint32_t duration) {
	if (!t->hasStarted) {
		t->curTick = HAL_GetTick();
		t->duration = duration;
		t->hasStarted = 1;
	}
}

void timerReset(Timer *t) {
	t->duration = 0;
	t->hasStarted = 0;
	t->hasFailed = 0;
}

char timerExpires(Timer *t) {
	if (t->hasStarted) {
		if ((HAL_GetTick() - t->curTick) >= t->duration) { return 1; }
	}
	return 0;
}

// Slew Filter for APPS
uint32_t APPS_SlewFilter(uint32_t appsADC, char channel) {

	if (channel == 1) {
	    // simple median-of-3 to reject single-sample spikes
	    // push new sample into history
	    appsRaw01 = appsRaw11;
	    appsRaw11 = appsRaw21;
	    appsRaw21 = appsADC;
	    // median-of-3 on copies so history isn't mutated
	    float a = appsRaw01, b = appsRaw11, c = appsRaw21;
	    if (a > b) { float t = a; a = b; b = t; }
	    if (b > c) { float t = b; b = c; c = t; }
	    if (a > b) { float t = a; a = b; b = t; }
	    float appsMed = b;

	    const float appsDt = 0.001f * delay; // delay is in ms
	    float appsDv = appsMed - appsPrev1;
	    float appsMaxDv = appsSlewVPerS1 * appsDt;
	    if (appsDv >  appsMaxDv) appsMed = appsPrev1 + appsMaxDv;
	    if (appsDv < -appsMaxDv) appsMed = appsPrev1 - appsMaxDv;
	    appsPrev1 = appsMed;

	    // use filtered value going forward
	    return appsMed;
	}
	else {
	    // simple median-of-3 to reject single-sample spikes
	    // push new sample into history
	    appsRaw02 = appsRaw12;
	    appsRaw12 = appsRaw22;
	    appsRaw22 = appsADC;
	    // median-of-3 on copies so history isn't mutated
	    float a = appsRaw02, b = appsRaw12, c = appsRaw22;
	    if (a > b) { float t = a; a = b; b = t; }
	    if (b > c) { float t = b; b = c; c = t; }
	    if (a > b) { float t = a; a = b; b = t; }
	    float appsMed = b;

	    const float appsDt = 0.001f * delay; // delay is in ms
	    float appsDv = appsMed - appsPrev2;
	    float appsMaxDv = appsSlewVPerS2 * appsDt;
	    if (appsDv >  appsMaxDv) appsMed = appsPrev2 + appsMaxDv;
	    if (appsDv < -appsMaxDv) appsMed = appsPrev2 - appsMaxDv;
	    appsPrev2 = appsMed;

	    // use filtered value going forward
	    return appsMed;
	}
}






