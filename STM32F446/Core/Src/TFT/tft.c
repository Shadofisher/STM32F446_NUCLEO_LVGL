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
#include "lv_conf.h"
#include "lvgl/lvgl.h"
#include <string.h>



extern SPI_HandleTypeDef hspi1;
extern LCD_DIS sLCD_DIS;

#define TFT_HOR_RES (480)

/*********************************************
function:	Hardware interface
note:
	SPI4W_Write_Byte(value) :
		Register hardware SPI
*********************************************/
uint8_t SPI4W_Write_Byte2(uint8_t value)
{
#if 1
    __HAL_SPI_ENABLE(&hspi1);
    SPI1->CR2 |= (1) << 12;

    while((SPI1->SR & (1 << 1)) == 0)
        ;

    *((__IO uint8_t *)(&SPI1->DR)) = value;

    while(SPI1->SR & (1 << 7)) ; //Wait for not busy

    while((SPI1->SR & (1 << 0)) == 0) ; // Wait for the receiving area to be empty

    return *((__IO uint8_t *)(&SPI1->DR));
#else
    __HAL_SPI_ENABLE(&hspi1);
    HAL_SPI_Transmit(&hspi1, &value, 1, 100000);
#endif
}

/*********************************************
function:	Hardware interface
note:
	SPI4W_Write_Byte(value) :
		Register hardware SPI
*********************************************/
uint8_t SPI4W_Write_Byte(uint8_t value)
{
#if 0
    __HAL_SPI_ENABLE(&hspi1);
    SPI1->CR2 |= (1) << 12;

    while((SPI1->SR & (1 << 1)) == 0)
        ;

    *((__IO uint8_t *)(&SPI1->DR)) = value;

    while(SPI1->SR & (1 << 7)) ; //Wait for not busy

    while((SPI1->SR & (1 << 0)) == 0) ; // Wait for the receiving area to be empty

    return *((__IO uint8_t *)(&SPI1->DR));
#else
    __HAL_SPI_ENABLE(&hspi1);
    HAL_SPI_Transmit(&hspi1, &value, 1, 100000);
#endif
}

uint8_t SPI4W_Read_Byte(uint8_t value)
{
	return SPI4W_Write_Byte2(value);
}


/********************************************************************************
function:	Delay function
note:
	Driver_Delay_ms(xms) : Delay x ms
	Driver_Delay_us(xus) : Delay x us
********************************************************************************/
void Driver_Delay_ms(uint32_t xms)
{
    HAL_Delay(xms);
}

/*******************************************************************************
function:
	Hardware reset
*******************************************************************************/
static void LCD_Reset(void)
{
    LCD_RST_1;
    Driver_Delay_ms(50);
    LCD_RST_0;
    Driver_Delay_ms(50);
    LCD_RST_1;
    Driver_Delay_ms(50);
}

/*******************************************************************************
function:
		Write register address and data
*******************************************************************************/
void LCD_WriteReg(uint8_t Reg)
{
    LCD_DC_0;
    LCD_CS_0;
    SPI4W_Write_Byte(Reg);
    LCD_CS_1;
}

void LCD_WriteData(uint8_t Data)
{
    LCD_DC_1;
    LCD_CS_0;
    SPI4W_Write_Byte(Data >> 8);
    SPI4W_Write_Byte(Data & 0XFF);
    LCD_CS_1;
}

/*******************************************************************************
function:
		Write register data
*******************************************************************************/
static void LCD_Write_AllData(uint16_t Data, uint32_t DataLen)
{
    uint32_t i;
    LCD_DC_1;
    LCD_CS_0;
    for(i = 0; i < DataLen; i++) {
        SPI4W_Write_Byte(Data >> 8);
        SPI4W_Write_Byte(Data & 0XFF);
    }
    LCD_CS_1;
}

/*******************************************************************************
function:
		Common register initialization
*******************************************************************************/
static void LCD_InitReg(void)
{
    LCD_WriteReg(0XF9);
    LCD_WriteData(0x00);
    LCD_WriteData(0x08);
//    for(;;)
//    {
       	LCD_WriteReg(0xC0);
        LCD_WriteData(0x19);//VREG1OUT POSITIVE
        LCD_WriteData(0x1a);//VREG2OUT NEGATIVE

//    }
   	LCD_WriteReg(0xC0);
    LCD_WriteData(0x19);//VREG1OUT POSITIVE
    LCD_WriteData(0x1a);//VREG2OUT NEGATIVE

    LCD_WriteReg(0xC1);
    LCD_WriteData(0x45);//VGH,VGL    VGH>=14V.
    LCD_WriteData(0x00);

    LCD_WriteReg(0xC2);	//Normal mode, increase can change the display quality, while increasing power consumption
    LCD_WriteData(0x33);

    LCD_WriteReg(0XC5);
    LCD_WriteData(0x00);
    LCD_WriteData(0x28);//VCM_REG[7:0]. <=0X80.

    LCD_WriteReg(0xB1);//Sets the frame frequency of full color normal mode
    LCD_WriteData(0xA0);//0XB0 =70HZ, <=0XB0.0xA0=62HZ
    LCD_WriteData(0x11);

    LCD_WriteReg(0xB4);
    LCD_WriteData(0x02); //2 DOT FRAME MODE,F<=70HZ.

    LCD_WriteReg(0xB6);//
    LCD_WriteData(0x00);
    LCD_WriteData(0x42);//0 GS SS SM ISC[3:0];
    LCD_WriteData(0x3B);

    LCD_WriteReg(0xB7);
    LCD_WriteData(0x07);

    LCD_WriteReg(0xE0);
    LCD_WriteData(0x1F);
    LCD_WriteData(0x25);
    LCD_WriteData(0x22);
    LCD_WriteData(0x0B);
    LCD_WriteData(0x06);
    LCD_WriteData(0x0A);
    LCD_WriteData(0x4E);
    LCD_WriteData(0xC6);
    LCD_WriteData(0x39);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);

    LCD_WriteReg(0XE1);
    LCD_WriteData(0x1F);
    LCD_WriteData(0x3F);
    LCD_WriteData(0x3F);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x1F);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x46);
    LCD_WriteData(0x49);
    LCD_WriteData(0x31);
    LCD_WriteData(0x05);
    LCD_WriteData(0x09);
    LCD_WriteData(0x03);
    LCD_WriteData(0x1C);
    LCD_WriteData(0x1A);
    LCD_WriteData(0x00);

    LCD_WriteReg(0XF1);
    LCD_WriteData(0x36);
    LCD_WriteData(0x04);
    LCD_WriteData(0x00);
    LCD_WriteData(0x3C);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x0F);
    LCD_WriteData(0xA4);
    LCD_WriteData(0x02);

    LCD_WriteReg(0XF2);
    LCD_WriteData(0x18);
    LCD_WriteData(0xA3);
    LCD_WriteData(0x12);
    LCD_WriteData(0x02);
    LCD_WriteData(0x32);
    LCD_WriteData(0x12);
    LCD_WriteData(0xFF);
    LCD_WriteData(0x32);
    LCD_WriteData(0x00);

    LCD_WriteReg(0XF4);
    LCD_WriteData(0x40);
    LCD_WriteData(0x00);
    LCD_WriteData(0x08);
    LCD_WriteData(0x91);
    LCD_WriteData(0x04);

    LCD_WriteReg(0XF8);
    LCD_WriteData(0x21);
    LCD_WriteData(0x04);

    LCD_WriteReg(0X3A);	//Set Interface Pixel Format
    LCD_WriteData(0x55);

}



/********************************************************************************
function:	Set the display scan and color transfer modes
parameter:
		Scan_dir   :   Scan direction
		Colorchose :   RGB or GBR color format
********************************************************************************/
void LCD_SetGramScanWay(LCD_SCAN_DIR Scan_dir)
{
    uint16_t MemoryAccessReg_Data = 0; //addr:0x36
    uint16_t DisFunReg_Data = 0; //addr:0xB6

    // Gets the scan direction of GRAM
    switch (Scan_dir) {
    case L2R_U2D:
        MemoryAccessReg_Data = 0x08;//0x08 | 0X8
        DisFunReg_Data = 0x22;
        break;
    case L2R_D2U:
        MemoryAccessReg_Data = 0x08;
        DisFunReg_Data = 0x62;
        break;
    case R2L_U2D: //0X4
        MemoryAccessReg_Data = 0x08;
        DisFunReg_Data = 0x02;
        break;
    case R2L_D2U: //0XC
        MemoryAccessReg_Data = 0x08;
        DisFunReg_Data = 0x42;
        break;
    case U2D_L2R: //0X2
        MemoryAccessReg_Data = 0x28;
        DisFunReg_Data = 0x22;
        break;
    case U2D_R2L: //0X6
        MemoryAccessReg_Data = 0x28;
        DisFunReg_Data = 0x02;
        break;
    case D2U_L2R: //0XA
        MemoryAccessReg_Data = 0x28;
        DisFunReg_Data = 0x62;
        break;
    case D2U_R2L: //0XE
        MemoryAccessReg_Data = 0x28;
        DisFunReg_Data = 0x42;
        break;
    }

    //Get the screen scan direction
    sLCD_DIS.LCD_Scan_Dir = Scan_dir;

    //Get GRAM and LCD width and height
    if(Scan_dir == L2R_U2D || Scan_dir == L2R_D2U || Scan_dir == R2L_U2D || Scan_dir == R2L_D2U) {
        sLCD_DIS.LCD_Dis_Column	= LCD_HEIGHT ;
        sLCD_DIS.LCD_Dis_Page = LCD_WIDTH ;
    } else {
        sLCD_DIS.LCD_Dis_Column	= LCD_WIDTH ;
        sLCD_DIS.LCD_Dis_Page = LCD_HEIGHT ;
    }

    // Set the read / write scan direction of the frame memory
    LCD_WriteReg(0xB6);
    LCD_WriteData(0X00);
    LCD_WriteData(DisFunReg_Data);

    LCD_WriteReg(0x36);
    LCD_WriteData(MemoryAccessReg_Data);
}


/*For LittlevGL*/
static void tft_flush_cb(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
	uint32_t size;
	LCD_SetWindow(area->x1, area->y1, area->x2+1, area->y2+1);

	//size = (area->x2-area->x1) * (area->y2-area->y1);
	size = (lv_area_get_width(area)) * (lv_area_get_height(area));

    LCD_DC_1;
    LCD_CS_0;
    //for(i = 0; i < DataLen; i++) {
    //    SPI4W_Write_Byte(Data >> 8);
    //    SPI4W_Write_Byte(Data & 0XFF);
   // }
//    for(i = 0; i < 100; i++)
    //memset(color_p,0x1f,size*2);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)color_p, size*2, 100000);
    LCD_CS_1;



	lv_disp_flush_ready(drv);


}

void tft_init(LCD_SCAN_DIR LCD_ScanDir)
{
	static lv_disp_buf_t disp_buf;
	static lv_color_t buf[TFT_HOR_RES * 50];
	lv_disp_buf_init(&disp_buf, buf, NULL, TFT_HOR_RES * 50);
	//lv_disp_buf_init(&disp_buf, 0xc0000000, 0xc0050000, 261120);
    LCD_BL_ON;


	lv_disp_drv_t disp_drv;
	lv_disp_drv_init(&disp_drv);
	disp_drv.flush_cb = tft_flush_cb;
	disp_drv.buffer = &disp_buf;

	lv_disp_drv_register(&disp_drv);
    //Hardware reset
    LCD_Reset();

    //Set the initialization register
    LCD_InitReg();

    //Set the display scan and color transfer modes
    LCD_SetGramScanWay( LCD_ScanDir );
    Driver_Delay_ms(200);

    //sleep out
    LCD_WriteReg(0x11);
    Driver_Delay_ms(120);

    //Turn on the LCD display
    LCD_WriteReg(0x29);

}


