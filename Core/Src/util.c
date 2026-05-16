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
float apps1Min = 		1.4;
float apps1Max = 		3.2;
float apps2Min = 		0.4;
float apps2Max = 		1.4;
float bseMin = 			0.65;
float bseMax = 			2.28;
char RTDActive = 		0; 			// bool for ready to drive
char InverterReady = 	0;
uint16_t delay = 		30;			// delay length in between loop executions
float vScale =			3.3;
/* Allowed raw voltage tolerance */
float APPS_VOLT_TOL = 0.050; /*temp value*/
/* Max allowed difference between the two pedal percentages */
float APPS_PERCENT_DIFF_MAX = 10.0; /*temp value*/




// APPS ADC Collection
void ADC_APPSCollection(float *readings) {

	// ADC Input for APPS 1
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		uint32_t raw1 = HAL_ADC_GetValue(&hadc1);
		readings[0] = ( (float) raw1 / (float) (4095.0)) * vScale;
	}
	// ADC Input for APPS 2
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		uint32_t raw2 = HAL_ADC_GetValue(&hadc1);
		readings[1] = ( (float) raw2 / (float) (4095.0)) * vScale;
	}

	HAL_ADC_Stop(&hadc1);
}

// BSE ADC Collection
float ADC_BSECollection() {
	HAL_ADC_Start(&hadc3);
	if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {
		return ((float) (HAL_ADC_GetValue(&hadc3) / (float) (4095.0)) * vScale);
	}
	else {
		return 4095.0;
	}
}

// APPS percentage calculations
float APPS_CalculateActivationPercentage(float adc, char channel) {

	if (channel == 1) {

//		if (adc < apps1Min) { return 150.0f; }
//		if (adc > apps1Max) { return -150.0f; }

		if (1.5f <= adc && adc <= 1.82f) { return 62.5f * (adc - 1.4f); }
		else if (1.82f < adc && adc <= 2.14f) { return 20.0f + 62.5f * (adc - 1.82f); }
		else if (2.14f < adc && adc <= 2.46f) { return 40.0f + 62.5f * (adc - 2.14f); }
		else if (2.46f < adc && adc <= 2.72f) { return 60.0f + 76.92f * (adc - 2.46f); }
		else if (2.72f < adc && adc <= 3.1f) {return 80.0f + 52.63f * (adc - 2.72f); }

		return -1;
	}
	else {

//		if (adc < apps2Min) { return 155.0f; }
//		if (adc > apps2Max) { return -155.0f; }

		if (0.5f <= adc && adc <= 0.66f) { return 125.0f * (adc - 0.4f); }
		else if (0.66f < adc && adc <= 0.82f) { return 20.0f + 125.0f * (adc - 0.66f); }
		else if (0.82f < adc && adc <= 0.98f) { return 40.0f + 125.0f * (adc - 0.82f); }
		else if (0.98f < adc && adc <= 1.16f) { return 60.0f + 111.11f * (adc - 0.98f); }
		else if (1.16f < adc && adc <= 1.3f) {return 80.0f + 142.86f * (adc - 1.16f); }

		return -1;
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

// APPs plausibility code
char APPS_ImplausibilityCheck(Timer *t, float appsFiltered1, float appsFiltered2) {
	t->hasFailed = 0;

	// sets failbit of timer object to true if:
	// apps signals exceed min or max values
	// gradient of the 2 signals exceeds 15%
	if (appsFiltered1 > 100.0 || appsFiltered1 < -0.0) { t->hasFailed = 1; }
	if (appsFiltered2 > 100.0 || appsFiltered2 < -0.0) { t->hasFailed = 1; }
	if (abs(appsFiltered1-appsFiltered2) > 10.0) { t->hasFailed = 1; }

	// starts timer if failbit is true
	if (t->hasFailed) {
		timerStart(t, 100);
	} else {
		timerReset(t);
	}

	// function returns true if timer exceeds 100ms
	if (timerExpires(t)) {
		return 1;
	}
	return 0;
}

// BSE plausibility code
// starts timer if bse value is out of bound.
// returns true if error persists for more than 100ms, else timer resets and returns false.
char BSE_ImplausibilityCheck(Timer* t, float bseRaw) {
	t->hasFailed = 0;

	// sets failbit of timer object to true if BSE signal exceeds max or min value
	if (bseRaw < bseMin || bseRaw > bseMax){ t->hasFailed = 1; }

	// starts timer if failbit is true
	if (t->hasFailed) {
		timerStart(t, 100);
	} else {
		timerReset(t);
	}

	// function returns true if timer exceeds 100ms
	if (timerExpires(t)) { return 1; }
	return 0;
}

// Starts timing implausibility if timer hasnt already started
// Sets duration threshold
// starting timer for implausibility checks
void timerStart(Timer *t, uint32_t duration) {
	if (!t->hasStarted) {
		t->curTick = HAL_GetTick();
		t->duration = duration;
		t->hasStarted = 1;
	}
}

// Resets timer
// reset timer for implausibility checks
void timerReset(Timer *t) {
	t->duration = 0;
	t->hasStarted = 0;
	t->hasFailed = 0;
}

// Determines if timer has exceeded threshold given by timerStart()
// checks for timer duration. returns true if timer reaches duration. else return false
char timerExpires(Timer *t) {
	if (t->hasStarted) {
		uint32_t currentTick = HAL_GetTick();
		if ((currentTick - t->curTick) >= t->duration) {
			t->failTick = currentTick;
			return 1;
		}
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




