
#ifndef _TFT_H_
#define _TFT_H_


#define	POINT				uint16_t		//The type of coordinate (unsigned short)
#define	COLOR				uint16_t		//The type of coordinate (unsigned short)

#define	LENGTH				uint16_t		//The type of coordinate (unsigned short)
#define SPI1_SCK_Pin 		GPIO_PIN_5
#define SPI1_SCK_GPIO_Port 	GPIOA
#define SPI1_MISO_Pin 		GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin 		GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define TP_BUSY_Pin 		GPIO_PIN_10
#define TP_BUSY_GPIO_Port 	GPIOB
#define LCD_BL_Pin 			GPIO_PIN_7
#define LCD_BL_GPIO_Port 	GPIOC
#define LCD_DC_Pin			GPIO_PIN_8
#define LCD_DC_GPIO_Port	GPIOA
#define LCD_RST_Pin 		GPIO_PIN_9
#define LCD_RST_GPIO_Port 	GPIOA
#define TP_IRQ_Pin 			GPIO_PIN_3
#define TP_IRQ_GPIO_Port 	GPIOB
#define SD_CS_Pin 			GPIO_PIN_4
#define SD_CS_GPIO_Port 	GPIOB
#define TP_CS_Pin 			GPIO_PIN_5
#define TP_CS_GPIO_Port 	GPIOB
#define LCD_CS_Pin 			GPIO_PIN_6
#define LCD_CS_GPIO_Port 	GPIOB

/* USER CODE END Header */

//LCD
#define LCD_CS_0		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define LCD_CS_1		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)

#define LCD_RST_0		HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET)
#define LCD_RST_1		HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET)

#define LCD_DC_0		HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET)
#define LCD_DC_1		HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET)

#define LCD_BL_Pin 			GPIO_PIN_7
#define LCD_BL_GPIO_Port 	GPIOC

#define LCD_BL_OFF		HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET)
#define LCD_BL_ON		HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET)


//Touch
#define TP_CS_0			HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_RESET)
#define TP_CS_1			HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET)

#define GET_TP_BUSY		HAL_GPIO_ReadPin(TP_BUSY_GPIO_Port, TP_BUSY_Pin)

#define GET_TP_IRQ		HAL_GPIO_ReadPin(TP_IRQ_GPIO_Port, TP_IRQ_Pin)


/********************************************************************************
function:
			scanning method
********************************************************************************/
typedef enum {
    L2R_U2D  = 0,	//The display interface is displayed , left to right, up to down
    L2R_D2U  ,
    R2L_U2D  ,
    R2L_D2U  ,
    U2D_L2R  ,
    U2D_R2L  ,
    D2U_L2R  ,
    D2U_R2L  ,
} LCD_SCAN_DIR;

/********************************************************************************
function:
	Defines the total number of rows in the display area
********************************************************************************/
typedef struct {
    LENGTH LCD_Dis_Column;	//COLUMN
    LENGTH LCD_Dis_Page;	//PAGE
    LCD_SCAN_DIR LCD_Scan_Dir;
    POINT LCD_X_Adjust;		//LCD x actual display position calibration
    POINT LCD_Y_Adjust;		//LCD y actual display position calibration
} LCD_DIS;


#define LCD_X_MAXPIXEL  480  //LCD width maximum memory
#define LCD_Y_MAXPIXEL  320 //LCD height maximum memory
#define LCD_X	 0
#define LCD_Y	 0

#define LCD_WIDTH  (LCD_X_MAXPIXEL - 2 * LCD_X)  //LCD width
#define LCD_HEIGHT  LCD_Y_MAXPIXEL //LCD height


#define SCAN_DIR_DFT  D2U_L2R  //Default scan direction = L2R_U2D

#endif


