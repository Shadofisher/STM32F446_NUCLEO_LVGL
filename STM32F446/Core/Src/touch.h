/*
 * touch.h
 *
 *  Created on: Dec 23, 2019
 *      Author: graeme
 */

#ifndef SRC_TOUCH_H_
#define SRC_TOUCH_H_
#include "lvgl.h"

#include "main.h"
#include "tft.h"
//Touch
#define TP_CS_0			HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_RESET)
#define TP_CS_1			HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET)

#define GET_TP_BUSY		HAL_GPIO_ReadPin(TP_BUSY_GPIO_Port, TP_BUSY_Pin)

#define GET_TP_IRQ		HAL_GPIO_ReadPin(TP_IRQ_GPIO_Port, TP_IRQ_Pin)

#define TP_PRESS_DOWN           0x80
#define TP_PRESSED              0x40

//Touch screen structure
typedef struct {
	POINT Xpoint0;
	POINT Ypoint0;
	POINT Xpoint;
	POINT Ypoint;
	uint8_t chStatus;
	uint8_t chType;
	int16_t iXoff;
	int16_t iYoff;
	float fXfac;
	float fYfac;
	LCD_SCAN_DIR TP_Scan_Dir;
}TP_DEV;


/********************************************************************************
function:
			dot pixel
********************************************************************************/
typedef enum {
    DOT_PIXEL_1X1  = 1,		// dot pixel 1 x 1
    DOT_PIXEL_2X2  , 		// dot pixel 2 X 2
    DOT_PIXEL_3X3  ,		// dot pixel 3 X 3
    DOT_PIXEL_4X4  ,		// dot pixel 4 X 4
    DOT_PIXEL_5X5  , 		// dot pixel 5 X 5
    DOT_PIXEL_6X6  , 		// dot pixel 6 X 6
    DOT_PIXEL_7X7  , 		// dot pixel 7 X 7
    DOT_PIXEL_8X8  , 		// dot pixel 8 X 8
} DOT_PIXEL;
//Brush structure
typedef struct{
	POINT Xpoint;
	POINT Ypoint;
	COLOR Color;
	DOT_PIXEL DotPixel;
}TP_DRAW;

#define DOT_PIXEL_DFT  DOT_PIXEL_1X1  //Default dot pilex

//uint8_t TP_GetTouch_CB(lv_indev_drv_t * drv, lv_indev_data_t *data);
#endif /* SRC_TOUCH_H_ */
