/*
 * active.c
 *
 *  Created on: Dec 26, 2025
 *      Author: Justin Im
 */

#include "active.h"
#include "rtd.h"

static float appsPrev = 0.0f;          // previous filtered voltage
static float appsSlewVPerS = 1.5f;     // max allowed V/sec change (tune as needed)
static float appsRaw0 = 0.0f, appsRaw1 = 0.0f, appsRaw2 = 0.0f; // rolling raw samples

float appsRaw;
float bseRaw;
float appsGradient;
float bseGradient;


// configuration and calibration variables
float appsMin = 		2.0; // 2v
float appsMax = 		3.95; // 3.8
float bseMin = 			1.58933; // 1v
float bseMax = 			3.25; // 3.25
float bseThreshold = 	40.0; // activation thresholds for the brakes
float torqueCommand = 	0.0; // torque command which will be sent to the inverter
char InverterReady = 	0;
uint16_t delay = 		30;	// delay length in between loop executions


extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern char RTDActive;
extern char InverterActive;

char Drive(void) {
	// accelerator pedal data collection
	HAL_ADC_Start(&hadc3);
	if (HAL_ADC_PollForConversion(&hadc3, 1) == HAL_OK) {
	  appsRaw = (((float) (HAL_ADC_GetValue(&hadc3)) / (float) (4095.0)) * 5.0);
	}
	HAL_ADC_Stop(&hadc3);
ww
	// brakes pedal data collection
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
	  bseRaw = (((float) (HAL_ADC_GetValue(&hadc1)) / (float) (4095.0)) * 5.0);
	}
	HAL_ADC_Stop(&hadc1);

	bseGradient = (44.44 * (bseRaw-1));
	torqueCommand = 2*(513.924 * (appsSlewFilter(appsRaw)) - 1080); // 0-1000

	// brake light logic
	if (bseGradient > bseThreshold) { HAL_GPIO_WritePin(GPIOA, Brake_Light_Active_Pin, SET); }
	else { HAL_GPIO_WritePin(GPIOA, Brake_Light_Active_Pin, RESET); }

	RTDActive = RTDCheck(bseGradient, bseThreshold);

	if (RTDActive && InverterActive) {
		if (bseGradient < 0.0) { bseGradient = 0.0; }
		if (bseGradient > 100.0) { bseGradient = 100.0; }
		if (bseGradient > bseThreshold) { torqueCommand = 0.0; }
		else {
		  if (torqueCommand <= 10.0) {torqueCommand = 0.0; }
		  if (torqueCommand >= 2000.0) {torqueCommand = 2000.0; }
		}
		Inverter_Process(torqueCommand);
	}

	// return 0 if there is an implausibility (to be implemented)

	return 1;
}

float appsSlewFilter(float appsRaw) {
	// simple median-of-3 to reject single-sample spikes
	// push new sample into history
	appsRaw0 = appsRaw1;
	appsRaw1 = appsRaw2;
	appsRaw2 = appsRaw;

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
