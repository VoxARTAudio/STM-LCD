#include "voxart_dev.h"
#include "main.h"
#include <string.h>
#include "stm324xg_eval_lcd.h"
#include "stm324xg_eval_io.h"
#include <stdio.h>
#include <stdlib.h>
#include "mpu6050.h"

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


/* ********************* */
/*    Circle Control     */
/* ********************* */
void updateCircle(int x, int y, int r) {
	c.xPos += x;
	c.yPos += y;
	c.radius += r;
	if(c.radius > 100 ||c.radius < 10) {
		c.radius = c.radius > 10 ? 100 : 10;
	} else {
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_FillCircle(c.xPos, c.yPos, c.radius);
	}
}

/* ********************* */
/*    Serial Logging     */
/* ********************* */
void serialPrint(char* msg) {
	uint8_t MSG[35] = {'\0'};
	//uint8_t X = 0;
	sprintf(MSG, "%s", msg);
	HAL_UART_Transmit(&huart3, MSG, sizeof(MSG), 100);
}

void serialPrintln(char* msg) {
	uint8_t MSG[35] = {'\0'};
	//uint8_t X = 0;
	msg = strcat(msg, "\n");
	sprintf(MSG, msg);
	HAL_UART_Transmit(&huart3, MSG, sizeof(MSG), 100);

	//HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen (msg), HAL_MAX_DELAY);
	
}

void serialPrintVarln(char* msg, void* var) {
	uint8_t MSG[35] = {'\0'};
	//uint8_t X = 0;
	msg = strcat(msg, "\r\n");
	sprintf(MSG, msg);
	HAL_UART_Transmit(&huart3, MSG, sizeof(MSG), 100);
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
	sprintf(MSG, "x: %0.1f  y: %0.1f  z: %0.1f\n", MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
	HAL_UART_Transmit(&huart3, MSG, sizeof(MSG), 100);
}
	
