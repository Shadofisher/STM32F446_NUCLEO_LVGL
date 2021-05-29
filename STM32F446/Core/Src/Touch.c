/*
 * Touch.c
 *
 *  Created on: Dec 23, 2019
 *      Author: graeme
 */
#include <stdbool.h>
#include "touch.h"
#include "tft.h"
extern LCD_SCAN_DIR Lcd_ScanDir;

static TP_DEV sTP_DEV;
static TP_DRAW sTP_Draw;


extern SPI_HandleTypeDef hspi1;
extern LCD_DIS sLCD_DIS;

static bool TP_Read_TwiceADC(uint16_t *pXCh_Adc, uint16_t  *pYCh_Adc );

void Driver_Delay_us(uint32_t xus)
{
	int j;
    for(j=xus; j > 0; j--);
}


/*******************************************************************************
function:
		Read the ADC of the channel
parameter:
	Channel_Cmd :	0x90: Read channel Y +, select the ADC resolution is 12 bits, set to differential mode
					0xd0: Read channel x +, select the ADC resolution is 12 bits, set to differential mode
*******************************************************************************/
static uint16_t TP_Read_ADC(uint8_t CMD)
{
    uint16_t Data = 0;

    //A cycle of at least 400ns.
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	HAL_SPI_Init(&hspi1);

	TP_CS_0;

	SPI4W_Write_Byte(CMD);
	Driver_Delay_us(200);

	Data = SPI4W_Read_Byte(0XFF);
	Data <<= 8;//7bit
	Data |= SPI4W_Read_Byte(0XFF);
	Data >>= 3;//5bit
	TP_CS_1;

	//LCD SPI speed = 18 MHz
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	HAL_SPI_Init(&hspi1);

    return Data;
}

/*******************************************************************************
function:
		Calculation
parameter:
		chCoordType:
					1 : calibration
					0 : relative position
*******************************************************************************/
static uint8_t TP_Scan(uint8_t chCoordType)
{
    //In X, Y coordinate measurement, IRQ is disabled and output is low
    if (!GET_TP_IRQ) {//Press the button to press
        //Read the physical coordinates
        if (chCoordType) {
            TP_Read_TwiceADC(&sTP_DEV.Xpoint, &sTP_DEV.Ypoint);
            //Read the screen coordinates
        } else if (TP_Read_TwiceADC(&sTP_DEV.Xpoint, &sTP_DEV.Ypoint)) {
//			DEBUG("(Xad,Yad) = %d,%d\r\n",sTP_DEV.Xpoint,sTP_DEV.Ypoint);
            if(sTP_DEV.TP_Scan_Dir == R2L_D2U) {		//Converts the result to screen coordinates
                sTP_Draw.Xpoint = sTP_DEV.fXfac * sTP_DEV.Xpoint +
                                  sTP_DEV.iXoff;
                sTP_Draw.Ypoint = sTP_DEV.fYfac * sTP_DEV.Ypoint +
                                  sTP_DEV.iYoff;
            } else if(sTP_DEV.TP_Scan_Dir == L2R_U2D) {
                sTP_Draw.Xpoint = sLCD_DIS.LCD_Dis_Column -
                                  sTP_DEV.fXfac * sTP_DEV.Xpoint -
                                  sTP_DEV.iXoff;
                sTP_Draw.Ypoint = sLCD_DIS.LCD_Dis_Page -
                                  sTP_DEV.fYfac * sTP_DEV.Ypoint -
                                  sTP_DEV.iYoff;
            } else if(sTP_DEV.TP_Scan_Dir == U2D_R2L) {
                sTP_Draw.Xpoint = sTP_DEV.fXfac * sTP_DEV.Ypoint +
                                  sTP_DEV.iXoff;
                sTP_Draw.Ypoint = sTP_DEV.fYfac * sTP_DEV.Xpoint +
                                  sTP_DEV.iYoff;
            } else {
                sTP_Draw.Xpoint = sLCD_DIS.LCD_Dis_Column -
                                  sTP_DEV.fXfac * sTP_DEV.Ypoint -
                                  sTP_DEV.iXoff;
                sTP_Draw.Ypoint = sLCD_DIS.LCD_Dis_Page -
                                  sTP_DEV.fYfac * sTP_DEV.Xpoint -
                                  sTP_DEV.iYoff;
            }
//			DEBUG("( x , y ) = %d,%d\r\n",sTP_Draw.Xpoint,sTP_Draw.Ypoint);
        }
        if (0 == (sTP_DEV.chStatus & TP_PRESS_DOWN)) {	//Not being pressed
            sTP_DEV.chStatus = TP_PRESS_DOWN | TP_PRESSED;
            sTP_DEV.Xpoint0 = sTP_DEV.Xpoint;
            sTP_DEV.Ypoint0 = sTP_DEV.Ypoint;
        }
    } else {
        if (sTP_DEV.chStatus & TP_PRESS_DOWN) {	//0x80
            sTP_DEV.chStatus &= ~(1 << 7);		//0x00
        } else {
            sTP_DEV.Xpoint0 = 0;
            sTP_DEV.Ypoint0 = 0;
            sTP_DEV.Xpoint = 0xffff;
            sTP_DEV.Ypoint = 0xffff;
        }
    }

    return (sTP_DEV.chStatus & TP_PRESS_DOWN);
}
/*******************************************************************************
function:
		Read the 5th channel value and exclude the maximum and minimum returns the average
parameter:
	Channel_Cmd :	0x90 :Read channel Y +
					0xd0 :Read channel x +
*******************************************************************************/
#define READ_TIMES  5	//Number of readings
#define LOST_NUM    1	//Discard value
static uint16_t TP_Read_ADC_Average(uint8_t Channel_Cmd)
{
    uint8_t i, j;
    uint16_t Read_Buff[READ_TIMES];
    uint16_t Read_Sum = 0, Read_Temp = 0;

    //Read and save multiple samples
    for(i = 0; i < READ_TIMES; i++){
		Read_Buff[i] = TP_Read_ADC(Channel_Cmd);
		Driver_Delay_us(200);
	}

    //Sort from small to large
    for (i = 0; i < READ_TIMES  -  1; i ++) {
        for (j = i + 1; j < READ_TIMES; j ++) {
            if (Read_Buff[i] > Read_Buff[j]) {
                Read_Temp = Read_Buff[i];
                Read_Buff[i] = Read_Buff[j];
                Read_Buff[j] = Read_Temp;
            }
        }
    }

    //Exclude the largest and the smallest
    for (i = LOST_NUM; i < READ_TIMES - LOST_NUM; i ++)
        Read_Sum += Read_Buff[i];

    //Averaging
    Read_Temp = Read_Sum / (READ_TIMES - 2 * LOST_NUM);

    return Read_Temp;
}

/*******************************************************************************
function:
		Read X channel and Y channel AD value
parameter:
	Channel_Cmd :	0x90 :Read channel Y +
					0xd0 :Read channel x +
*******************************************************************************/
static void TP_Read_ADC_XY(uint16_t *pXCh_Adc, uint16_t  *pYCh_Adc )
{
    *pXCh_Adc = TP_Read_ADC_Average(0xD0);
    *pYCh_Adc = TP_Read_ADC_Average(0x90);
}

/*******************************************************************************
function:
		2 times to read the touch screen IC, and the two can not exceed the deviation,
		ERR_RANGE, meet the conditions, then that the correct reading, otherwise the reading error.
parameter:
	Channel_Cmd :	pYCh_Adc = 0x90 :Read channel Y +
					pXCh_Adc = 0xd0 :Read channel x +
*******************************************************************************/
#define ERR_RANGE 50	//tolerance scope
static bool TP_Read_TwiceADC(uint16_t *pXCh_Adc, uint16_t  *pYCh_Adc )
{
    uint16_t XCh_Adc1, YCh_Adc1, XCh_Adc2, YCh_Adc2;

    //Read the ADC values Read the ADC values twice
    TP_Read_ADC_XY(&XCh_Adc1, &YCh_Adc1);
//	Driver_Delay_us(10);
    TP_Read_ADC_XY(&XCh_Adc2, &YCh_Adc2);
//	Driver_Delay_us(10);

    //The ADC error used twice is greater than ERR_RANGE to take the average
    if( ((XCh_Adc2 <= XCh_Adc1 && XCh_Adc1 < XCh_Adc2 + ERR_RANGE) ||
         (XCh_Adc1 <= XCh_Adc2 && XCh_Adc2 < XCh_Adc1 + ERR_RANGE))
        && ((YCh_Adc2 <= YCh_Adc1 && YCh_Adc1 < YCh_Adc2 + ERR_RANGE) ||
            (YCh_Adc1 <= YCh_Adc2 && YCh_Adc2 < YCh_Adc1 + ERR_RANGE))) {
        *pXCh_Adc = (XCh_Adc1 + XCh_Adc2) / 2;
        *pYCh_Adc = (YCh_Adc1 + YCh_Adc2) / 2;
        return true;
    }

    //The ADC error used twice is less than ERR_RANGE returns failed
    return false;
}

uint8_t TP_GetTouch_CB(lv_indev_drv_t * drv, lv_indev_data_t *data);
/*******************************************************************************
function:
		Touch pad initialization
*******************************************************************************/
void TP_Init( LCD_SCAN_DIR Lcd_ScanDir )
{
    TP_CS_1;

    sTP_DEV.TP_Scan_Dir = Lcd_ScanDir;

    TP_Read_ADC_XY(&sTP_DEV.Xpoint, &sTP_DEV.Ypoint);

    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = TP_GetTouch_CB;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);



}
/*******************************************************************************
function:
		Use the default calibration factor
*******************************************************************************/
void TP_GetAdFac(void)
{
    if(	sTP_DEV.TP_Scan_Dir == D2U_L2R ) { //SCAN_DIR_DFT = D2U_L2R
        sTP_DEV.fXfac = -0.132443F ;
        sTP_DEV.fYfac = 0.089997F ;
        sTP_DEV.iXoff = 516L ;
        sTP_DEV.iYoff = -22L ;
    } else if( sTP_DEV.TP_Scan_Dir == L2R_U2D ) {
        sTP_DEV.fXfac = 0.089697F ;
        sTP_DEV.fYfac = 0.134792F ;
        sTP_DEV.iXoff = -21L ;
        sTP_DEV.iYoff = -39L ;
    } else if( sTP_DEV.TP_Scan_Dir == R2L_D2U ) {
        sTP_DEV.fXfac = 0.089915F ;
        sTP_DEV.fYfac =  0.133178F ;
        sTP_DEV.iXoff = -22L ;
        sTP_DEV.iYoff = -38L ;
    } else if( sTP_DEV.TP_Scan_Dir == U2D_R2L ) {
        sTP_DEV.fXfac = -0.132906F ;
        sTP_DEV.fYfac = 0.087964F ;
        sTP_DEV.iXoff = 517L ;
        sTP_DEV.iYoff = -20L ;
    }
}

/*******************************************************************************
function:
		Use the default calibration factor
*******************************************************************************/
uint8_t TP_GetTouch_CB(lv_indev_drv_t * drv, lv_indev_data_t *data)
//uint8_t TP_GetTouch(void)
{
	static int16_t last_x = 0;
	static int16_t last_y = 0;

	TP_Scan(0);
	if (sTP_DEV.chStatus & TP_PRESS_DOWN)
	{
		//sTP_Draw.Xpoint = sTP_DEV.fXfac * sTP_DEV.Xpoint + sTP_DEV.iXoff;
		//sTP_Draw.Ypoint = sTP_DEV.fYfac * sTP_DEV.Ypoint + sTP_DEV.iYoff;
		data->point.x = sTP_Draw.Xpoint;
		data->point.y = sTP_Draw.Ypoint;
		last_x = data->point.x;
		last_y = data->point.y;
		data->state = LV_INDEV_STATE_PR;
	}
	else
	{
		data->point.x = last_x;
		data->point.y = last_y;
		data->state = LV_INDEV_STATE_REL;
	}
	return(false);
}

