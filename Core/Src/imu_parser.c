#include "main.h"
#include "imu_parser.h"
#include <stdbool.h>
#include "stm324xg_eval_io.h"
#include "stm324xg_eval_lcd.h"
#include "mpu6050.h"
#include "voxart_dev.h"
#include <stdio.h>
#include <stdlib.h>

/*********************** */
/*    Extern Variables   */ 
/*********************** */
extern MPU6050_t MPU6050;
extern imuMovement imu;
extern TIM_HandleTypeDef htim13;
extern uint32_t timestamp;


enum effectState state = NONE;

void setStates() {
	if(MPU6050.KalmanAngleX > -10 && MPU6050.KalmanAngleX < 10 && MPU6050.KalmanAngleY > -10 && MPU6050.KalmanAngleY < 10 && state != PITCH) {
		state = REVERB;
	} else if(MPU6050.KalmanAngleY < -175 && MPU6050.KalmanAngleY > -181 && state != PITCH) {
		state = CHORUS;
	}
}

void parseReverbData() {
	if (MPU6050.Ax > 1.2) {
		//serialPrintln("Positive X");
		imu.x = 1;
		}
	if(MPU6050.Ax < -1.5) {
		//serialPrintln("Negative X");
		imu.x = -1;
	} 
	if(MPU6050.Ax < 1.2 && MPU6050.Ax > -1) {
		imu.x = 0;
	}
}

void parseChorusData() {
	if(MPU6050.KalmanAngleX < -13) {
		imu.y = -1;
	}
	if(MPU6050.KalmanAngleX > 30) {
		imu.y = 1;
	}
	if(MPU6050.KalmanAngleX > -5 && MPU6050.KalmanAngleX < 30) {
		imu.y = 0;
	}
}



