/*
 * util.c
 *
 *  Created on: Jan 19, 2026
 *      Author: imjus
 */
#include <stdlib.h>
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
float apps1Min = 		1.34; 		// 1.36
float apps1Max = 		2.97; 		// 2.97
float apps2Min = 		1.53;
float apps2Max = 		2.97;
float bseMin = 			0.65;
float bseMax = 			2.28;
char RTDActive = 		0; 			// bool for ready to drive
char InverterReady = 	0;
uint16_t delay = 		30;			// delay length in between loop executions
float vScale =			3.3;


// APPS ADC Collection
void ADC_APPSCollection(float *readings) {

	// ADC Input for APPS 1
	HAL_ADC_Start(&hadc3);
	if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {
		uint32_t raw1 = HAL_ADC_GetValue(&hadc3);
		readings[0] = ( (float) raw1 / (float) (4095.0)) * vScale;
	}
	// ADC Input for APPS 2
	HAL_ADC_Start(&hadc3);
	if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {
		uint32_t raw2 = HAL_ADC_GetValue(&hadc3);
		readings[1] = ( (float) raw2 / (float) (4095.0)) * vScale;
	}

	HAL_ADC_Stop(&hadc3);
}

// BSE ADC Collection
float ADC_BSECollection() {
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		return ((float) (HAL_ADC_GetValue(&hadc1) / (float) (4095.0)) * vScale);
	}
	else {
		return 4095.0;
	}
}

// APPS percentage calculations
float APPS_CalculateActivationPercentage(float adc, char channel) {
	if (channel == 1) {
		return 70.2865539932 * (adc - 1.54725277);
	}
	else {
		return 62.0920235829 * (adc - 1.35948718);
	}
}


// torque calculations
float Drive_CalculateTorqueCommand(float appsRaw) {
	return 2*(513.924 * (appsRaw) - 1080); // 0-1000
}

// brakes percentage calculations
float Drive_CalculateBrakesActivation(float bseRaw) {
	return (60.9756 * (bseRaw - 0.64));
}

// starts timing if an implausibility occurs
char APPS_ImplausibilityCheck(Timer *t, float appsFiltered1, float appsFiltered2) {
	t->hasFailed = 0;

	if (appsFiltered1 > 100.0 || appsFiltered1 < 0.0) { t->hasFailed = 1; }
	if (appsFiltered2 > 100.0 || appsFiltered2 < 0.0) { t->hasFailed = 1; }
	if (abs(appsFiltered1-appsFiltered2) > 15.0) { t->hasFailed = 1; }

	if (t->hasFailed) {
		timerStart(t, 100);
	} else {
		timerReset(t);
	}

	if (timerExpires(t)) { return 1; }
	return 0;
}

// starts timer if bse value is out of bound.
// returns true if error persists for more than 100ms, else timer resets and returns false.

char BSE_ImplausibilityCheck(Timer* t, float bseRaw) {
	t->hasFailed = 0;
	if (bseRaw < bseMin || bseRaw > bseMax){ t->hasFailed = 1; }

	if (t->hasFailed) {
		timerStart(t, 100);
	} else {
		timerReset(t);
	}

	if (timerExpires(t)) { return 1; }
	return 0;
}

// starting timer for implausibility checks
void timerStart(Timer *t, uint32_t duration) {
	if (!t->hasStarted) {
		t->curTick = HAL_GetTick();
		t->duration = duration;
		t->hasStarted = 1;
	}
}

// reset timer for implausibility checks
void timerReset(Timer *t) {
	t->duration = 0;
	t->hasStarted = 0;
	t->hasFailed = 0;
}

// checks for timer duration. returns true if timer reaches duration. else return false
char timerExpires(Timer *t) {
	if (t->hasStarted) {
		if ((HAL_GetTick() - t->curTick) >= t->duration) { return 1; }
	}
	return 0;
}

// Slew Filter for APPS
float APPS_SlewFilter(float appsADC, char channel) {

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




