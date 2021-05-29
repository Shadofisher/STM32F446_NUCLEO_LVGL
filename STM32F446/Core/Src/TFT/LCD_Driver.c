/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include "main.h"
#include "tft.h"
#include <string.h>


extern SPI_HandleTypeDef hspi1;


LCD_DIS sLCD_DIS;
void LCD_WriteReg(uint8_t Reg);
void LCD_WriteData(uint8_t Data);
void LCD_SetWindow(POINT Xstart, POINT Ystart,	POINT Xend, POINT Yend);
uint8_t SPI4W_Write_Byte(uint8_t value);
/*******************************************************************************
function:
		Write register data
*******************************************************************************/
static void LCD_Write_AllData(uint16_t Data, uint32_t DataLen)
{
	uint16_t test_colour[10];
    uint32_t i;
	memset(test_colour,0x001f,10);
    LCD_DC_1;
    LCD_CS_0;
    //for(i = 0; i < DataLen; i++) {
    //    SPI4W_Write_Byte(Data >> 8);
    //    SPI4W_Write_Byte(Data & 0XFF);
   // }
    for(i = 0; i < 100; i++)
    	HAL_SPI_Transmit(&hspi1, &test_colour, 10, 100000);
    LCD_CS_1;
}

/********************************************************************************
function:	Set show color
parameter:
		Color  :   Set show color,16-bit depth
********************************************************************************/
//static void LCD_SetColor(LENGTH Dis_Width, LENGTH Dis_Height, COLOR Color ){
void LCD_SetColor(COLOR Color , POINT Xpoint, POINT Ypoint)
{
    LCD_Write_AllData(Color , (uint32_t)Xpoint * (uint32_t)Ypoint);
}

/********************************************************************************
function:	Fill the area with the color
parameter:
	Xstart :   Start point x coordinate
	Ystart :   Start point y coordinate
	Xend   :   End point coordinates
	Yend   :   End point coordinates
	Color  :   Set the color
********************************************************************************/
void LCD_SetArealColor(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,	COLOR Color)
{
    if((Xend > Xstart) && (Yend > Ystart)) {
        //LCD_SetWindow(Xstart , Ystart , Xend , Yend  );
        LCD_SetColor ( Color , Xend - Xstart, Yend - Ystart);
    }
}
/********************************************************************************
function:
			Clear screen
********************************************************************************/
void LCD_Clear(COLOR  Color)
{
    LCD_SetArealColor(0, 0, sLCD_DIS.LCD_Dis_Column , sLCD_DIS.LCD_Dis_Page , Color);
}
/********************************************************************************
function:	Sets the start position and size of the display area
parameter:
	Xstart 	:   X direction Start coordinates
	Ystart  :   Y direction Start coordinates
	Xend    :   X direction end coordinates
	Yend    :   Y direction end coordinates
********************************************************************************/
void LCD_SetWindow(POINT Xstart, POINT Ystart,	POINT Xend, POINT Yend)
{
    //set the X coordinates
    LCD_WriteReg(0x2A);
    LCD_WriteData(Xstart >> 8);	 				//Set the horizontal starting point to the high octet
    LCD_WriteData(Xstart & 0xff);	 				//Set the horizontal starting point to the low octet
    LCD_WriteData((Xend - 1) >> 8);	//Set the horizontal end to the high octet
    LCD_WriteData((Xend - 1) & 0xff);	//Set the horizontal end to the low octet

    //set the Y coordinates
    LCD_WriteReg(0x2B);
    LCD_WriteData(Ystart >> 8);
    LCD_WriteData(Ystart & 0xff );
    LCD_WriteData((Yend - 1) >> 8);
    LCD_WriteData((Yend - 1) & 0xff);
    LCD_WriteReg(0x2C);
}


