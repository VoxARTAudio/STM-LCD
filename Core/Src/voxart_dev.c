//C COMMON
#include <stdbool.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//MISC
#include "voxart_dev.h"
#include "main.h"
#include "mpu6050.h"

//BSP
#include "stm324xg_eval_lcd.h"
#include "stm324xg_eval_io.h"

/*********************** */
/*    Extern Variables   */ 
/*********************** */
extern UART_HandleTypeDef huart3;
extern float accelBuff[64];
extern circle c;
extern MPU6050_t MPU6050;

/*********************** */
/*   Private Variables   */
/*********************** */
//Circle Min and Max
#define MAX_Y 175
#define MIN_Y 60
#define MIN_X 0
#define MAX_X 150
#define MAX_RADIUS 100
#define MIN_RADIUS 10
/* ********************* */
/*    Circle Control     */
/* ********************* */
void updateCircle(int x, int y, int r) {
	c.xPos += x;
	c.yPos += y;
	c.radius += r;
	
	if((c.radius > MAX_RADIUS) || (c.radius < MIN_RADIUS)) {
		c.radius = c.radius > MAX_RADIUS ? MAX_RADIUS : MIN_RADIUS;
	}

	if((c.yPos > MAX_Y) || (c.yPos < MIN_Y)) {
		c.yPos = c.yPos > MAX_Y ? MAX_Y : MIN_Y;
	}

	if(c.xPos > MAX_X || c.xPos < MIN_X) {
		c.xPos = c.xPos > MAX_X ? MAX_X : MIN_X;
	} 
	
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_FillCircle(c.xPos, c.yPos, c.radius);
}

/* ********************* */
/*    Serial Logging     */
/* ********************* */
void serialPrint(char* msg) {
	uint8_t MSG[35] = {'\0'};

	sprintf(MSG, "%s", msg);
	HAL_UART_Transmit(&huart3, MSG, sizeof(MSG), 100);
}

void serialPrintln(char* msg) {
	uint8_t MSG[35] = {'\0'};

	char* fstring = malloc(strlen(msg) + 4);
	strcpy(fstring, msg);
	strcat(fstring, "\r\n");
	sprintf(MSG, "%s", fstring);
	HAL_UART_Transmit(&huart3, MSG, sizeof(MSG), 100);
	free(fstring);
}

void serialPrintIMU() {
	char xPos[64];
	char yPos[64];
	char zPos[64];
	
	snprintf(xPos, sizeof(xPos), "%f", MPU6050.Ax);
	snprintf(yPos, sizeof(xPos), "%f", MPU6050.Ay);
	snprintf(zPos, sizeof(xPos), "%f", MPU6050.Az);
	
	uint8_t MSG[100] = {'\0'};
	//uint8_t X = 0;
	sprintf(MSG, "x: %0.1f  y: %0.1f\r\n", MPU6050.KalmanAngleX, MPU6050.KalmanAngleY);
	HAL_UART_Transmit(&huart3, MSG, sizeof(MSG), 100);
}
	
void lcdUpdate();