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
extern imuMovement imu;
extern TIM_HandleTypeDef htim13;
extern uint32_t timestamp;
extern float Shift;

enum effectState state = NONE;

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

void pitchAdjuster(float inc, float angle) {
	float cmp = Shift+inc;
	if(cmp < SHIFT_MIN) {
		Shift = SHIFT_MIN;
	} else if(cmp > SHIFT_MAX) {
		Shift = SHIFT_MAX;
	} else {
		Shift = Shift+inc;
	}
}



