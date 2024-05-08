#include "main.h"
#include "imu_parser.h"
#include <stdbool.h>
#include "stm324xg_eval_io.h"
#include "stm324xg_eval_lcd.h"
#include "mpu6050.h"
#include "voxart_dev.h"
#include "audioEffects.h"
#include <stdio.h>
#include <stdlib.h>

/*********************** */
/*    Extern Variables   */ 
/*********************** */
extern MPU6050_t MPU6050;
//extern imuMovement imu;
extern uint32_t timestamp;
extern float Shift;
extern float pitchZeroAngle;
extern float wet;

enum effectState state = NONE;

char* dispValue;

void setStates() {
	if(MPU6050.KalmanAngleX > -10 && MPU6050.KalmanAngleX < 10 && MPU6050.KalmanAngleY > -10 && MPU6050.KalmanAngleY < 10 && state != PITCH && state != NONE) {
		state = NONE;
		BSP_LCD_DisplayStringAtLine(9, (uint8_t*)"Idle   ");
	} else if(MPU6050.KalmanAngleX > -10 && MPU6050.KalmanAngleX < 6 && MPU6050.KalmanAngleY > 100 && MPU6050.KalmanAngleY < 120 && state != PITCH && state != REVERB) {
		state = REVERB;
		BSP_LCD_DisplayStringAtLine(9, (uint8_t*)"Reverb");
	} else if(MPU6050.KalmanAngleX > -5 && MPU6050.KalmanAngleX < 5 && MPU6050.KalmanAngleY < -170 && MPU6050.KalmanAngleY > -181 && state != PITCH && state != CHORUS) {
		state = CHORUS;
		BSP_LCD_DisplayStringAtLine(9, (uint8_t*)"Chorus");
	}
}

void pitchAdjuster(float angle) {
	float cmp = 1.0f +(((angle - pitchZeroAngle)/90.0f)/3.0f);
	if(cmp < SHIFT_MIN) {
		Shift = SHIFT_MIN;
	} else if(cmp > SHIFT_MAX) {
		Shift = SHIFT_MAX;
	} else {
		Shift = cmp;
	}
}


void reverbAdjuster(float accel) {
	if(accel >= ACCEL_MIN) {
		wet = accel/3.0f;
		wet = (wet > 1.0f) ? 1.0f : (wet < 0.0f) ? 0.0f : wet;
	}
//	if(accel >= -ACCEL_MIN) {
//		wet = 0.0f;
//	}
}

