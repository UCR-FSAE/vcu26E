/*
 * util.c
 *
 *  Created on: Jan 19, 2026
 *      Author: imjus
 */
#include <stdlib.h>
#include <util.h>
#include <math.h>

// configuration and calibration variables
float apps1Min = 		1.4;
float apps1Max = 		3.2;
float apps2Min = 		0.4;
float apps2Max = 		1.4;
float bseMin = 			0.3;
float bseMax = 			1.3;
float vScale =			3.3;

// On the go recalibration function, triggered by user button
void RecalibratePedals() {

	float reading = 0.0;
	float newAppsMax1 = 0.0;
	float newAppsMax2 = 0.0;
	float newAppsMin1 = 6.0;
	float newAppsMin2 = 6.0;
	float newBseMax = 0.0;
	float newBseMin = 0.0;

	HAL_GPIO_WritePin(GPIOB, LD1_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, LD2_Pin, RESET);
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, RESET);

	// new apps mins
	int init = HAL_GetTick();
	while (HAL_GetTick() - init <= 3000) {
		// ADC Input for APPS 1
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
			uint32_t raw1 = HAL_ADC_GetValue(&hadc1);
			reading = ( (float) raw1 / (float) (4095.0)) * vScale;
			if (reading < newAppsMin1) { newAppsMin1 = reading; }
		}
		// ADC Input for APPS 2
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
			uint32_t raw2 = HAL_ADC_GetValue(&hadc1);
			reading = ( (float) raw2 / (float) (4095.0)) * vScale;
			if (reading < newAppsMin2) { newAppsMin2 = reading; }
		}

		HAL_ADC_Stop(&hadc1);
	}

	HAL_GPIO_WritePin(GPIOB, LD1_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, LD2_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, RESET);

	// new apps maxes
	init = HAL_GetTick();
	while (HAL_GetTick() - init <= 3000) {
		// ADC Input for APPS 1
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
			uint32_t raw1 = HAL_ADC_GetValue(&hadc1);
			reading = ( (float) raw1 / (float) (4095.0)) * vScale;
			if (reading > newAppsMax1) { newAppsMax1 = reading; }
		}
		// ADC Input for APPS 2
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
			uint32_t raw2 = HAL_ADC_GetValue(&hadc1);
			reading = ( (float) raw2 / (float) (4095.0)) * vScale;
			if (reading > newAppsMax2) { newAppsMax2 = reading; }
		}

		HAL_ADC_Stop(&hadc1);
	}

	HAL_GPIO_WritePin(GPIOB, LD1_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, LD2_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, SET);

	// new brakes mins
	init = HAL_GetTick();
	while (HAL_GetTick() - init <= 3000) {
		HAL_ADC_Start(&hadc3);
		if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {
			reading = (float) (HAL_ADC_GetValue(&hadc3) / (float) (4095.0)) * vScale;
			if (reading < newBseMin) { newBseMin = reading; }
		}
	}

	HAL_GPIO_WritePin(GPIOB, LD1_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, LD2_Pin, RESET);
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, SET);

	// new brakes maxes
	init = HAL_GetTick();
	while (HAL_GetTick() - init <= 3000) {
		HAL_ADC_Start(&hadc3);
		if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {
			reading = (float) (HAL_ADC_GetValue(&hadc3) / (float) (4095.0)) * vScale;
			if (reading > newBseMax) { newBseMax = reading; }
		}
	}

	HAL_GPIO_WritePin(GPIOB, LD1_Pin, RESET);
	HAL_GPIO_WritePin(GPIOB, LD2_Pin, RESET);
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, RESET);

	apps1Min = newAppsMin1 * 0.9;
	apps2Min = newAppsMin2 * 0.9;
	apps1Max = newAppsMax1 * 1.1;
	apps2Max = newAppsMax2 * 1.1;
	bseMin = newBseMin * 0.9;
	bseMax = newBseMax * 1.1;
}

// APPS ADC Collection
void ADC_APPSCollection(float* readings) {

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
void ADC_BSECollection(float* readings) {
	HAL_ADC_Start(&hadc3);
	if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {
		*readings = (HAL_ADC_GetValue(&hadc3) / (float) (4095.0)) * vScale;
	}
	HAL_ADC_Stop(&hadc3);
}

// APPS percentage calculations, segmented to better match the curvature of pedal travel
float APPS_CalculateActivationPercentage(float adc, char channel) {
	if (channel == 1) {
		if (adc < apps1Min) { return -1.0f; }
		if (adc > apps1Max) { return -1.0f; }
		if (1.5f <= adc && adc <= 1.82f) { return 62.5f * (adc - 1.4f); }
		else if (1.82f < adc && adc <= 2.14f) { return 20.0f + 62.5f * (adc - 1.82f); }
		else if (2.14f < adc && adc <= 2.46f) { return 40.0f + 62.5f * (adc - 2.14f); }
		else if (2.46f < adc && adc <= 2.72f) { return 60.0f + 76.92f * (adc - 2.46f); }
		else if (2.72f < adc && adc <= 3.1f) {return 80.0f + 52.63f * (adc - 2.72f); }
		else {return 80.0f + 52.63f * (adc - 2.72f); }

	}
	else {
		if (adc < apps2Min) { return -1.0f; }
		if (adc > apps2Max) { return -1.0f; }
		if (0.5f <= adc && adc <= 0.66f) { return 125.0f * (adc - 0.4f); }
		else if (0.66f < adc && adc <= 0.82f) { return 20.0f + 125.0f * (adc - 0.66f); }
		else if (0.82f < adc && adc <= 0.98f) { return 40.0f + 125.0f * (adc - 0.82f); }
		else if (0.98f < adc && adc <= 1.16f) { return 60.0f + 111.11f * (adc - 0.98f); }
		else {return 80.0f + 142.86f * (adc - 1.16f); }
	}
}

// torque calculations
float Drive_CalculateTorqueCommand(float pedalPercentage) {
	return MAX_TORQUE_COMMAND * ((pedalPercentage - DEADZONE) / 100);
}

// APPs plausibility code
char APPS_ImplausibilityCheck(Timer *t, float appsFiltered1, float appsFiltered2) {
	char currentFaultState = 0;

	// sets failbit of timer object to true if:
	// apps signals exceed min or max values
	// gradient of the 2 signals exceeds 15%
	if (appsFiltered1 > 110.0 || appsFiltered1 < 0.0) { currentFaultState = 1;}
	if (appsFiltered2 > 110.0 || appsFiltered2 < 0.0) { currentFaultState = 1; }
	if (fabsf(appsFiltered1-appsFiltered2) > 20.0) { currentFaultState = 1; }


	if (currentFaultState == 1) {
		if (t->hasFailed == 0) {
			timerStart(t, 100);
			t->hasFailed = 1;
		}
	}
	// starts timer if failbit is true
	 else {
			timerReset(t);
			t->hasFailed = 0;
			currentFaultState = 0;
	}

	// function returns true if timer exceeds 100ms
	if (t->hasFailed == 1 && timerExpires(t)) {
		return 1;
	}
	return 0;
}

// BSE plausibility code
// starts timer if bse value is out of bound.
// returns true if error persists for more than 100ms, else timer resets and returns false.
char BSE_ImplausibilityCheck(Timer* t, float bseRaw) {
	char currentFaultState = 0;

	// sets failbit of timer object to true if BSE signal exceeds max or min value
	if (bseRaw < bseMin || bseRaw > bseMax){ currentFaultState = 1; }

	// starts timer if failbit is true
	if (currentFaultState == 1) {
		if (t->hasFailed == 0) {
			timerStart(t, 100);
			t->hasFailed = 1;
		}
	}
	// starts timer if failbit is true
	 else {
			timerReset(t);
			t->hasFailed = 0;
			currentFaultState = 0;
	}

	// function returns true if timer exceeds 100ms
	if (t->hasFailed == 1 && timerExpires(t)) {
		return 1;
	}
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
