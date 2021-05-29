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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lvgl.h"
#include "tft.h"
#include "touch.h"
#include "demo.h"
#include "PID.h"

void tft_init(LCD_SCAN_DIR LCD_ScanDir);
void LCD_Clear(COLOR  Color);
#define WHITE          0xFFFF
#define BLACK          0x0000
#define BLUE           0x001F
#define BRED           0XF81F
#define GRED 		   0XFFE0
#define GBLUE		   0X07FF
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0
#define BROWN 		   0XBC40
#define BRRED 		   0XFC07
#define GRAY  		   0X8430

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Structure to strore PID data and pointer to PID structure
struct pid_controller ctrldata;
struct pid_controller ctrldata1;
pid_t pid;
pid_t pid1;

// Control loop input,output and setpoint variables
float input = 0, output = 0;
float input1 = 0, output1 = 0;
float setpoint = 10;

// Control loop gains
float kp = 0.8, ki = 1, kd = 4;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

uint32_t IC_Value1 = 0;
uint32_t IC_Value2 = 0;
uint32_t Difference = 0;
uint32_t Frequency = 0;
uint8_t Is_First_Captured = 0;  // 0- not captured, 1- captured


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char label_text[64];
uint32_t my_timer  = 0;
uint32_t my_speed1 = 0;
uint32_t my_speed2 = 0;
extern  uint32_t int_my_timer;

lv_obj_t * gauge1;
lv_obj_t * gauge2;
lv_obj_t * speed1;
lv_obj_t * speed2;
//uint32_t prev_value = 0;


#if 0
lv_obj_t * gauge1;
lv_obj_t * gauge2;
lv_obj_t * speed1;
lv_obj_t * speed2;
uint32_t prev_value = 0;
static void slider_event_cb(lv_obj_t * slider, lv_event_t event)
{
	uint32_t period;
    if(event == LV_EVENT_VALUE_CHANGED)
    {
        //static char buf[4]; /* max 3 bytes for number plus 1 null terminating byte */
        //snprintf(buf, 4, "%u", lv_slider_get_value(slider));
        //lv_label_set_text(slider_label, buf);
    	lv_gauge_set_value(gauge1, 0,lv_slider_get_value(slider));
    	period  = 50*lv_slider_get_value(slider);
    	if (period >= 4800)
    		period = 4800;
    	if (period <=100)
    		period = 100;
    	//__HAL_TIM_SET_AUTORELOAD(&htim3, period);
    	//TIMx->CCR1 = OC_Config->Pulse;
    	//htim3.Instance->CCR1 = period;

    }
}
static void slider2_event_cb(lv_obj_t * slider, lv_event_t event)
{
	uint32_t period;
    if(event == LV_EVENT_VALUE_CHANGED)
    {
        //static char buf[4]; /* max 3 bytes for number plus 1 null terminating byte */
        //snprintf(buf, 4, "%u", lv_slider_get_value(slider));
        //lv_label_set_text(slider_label, buf);
    	lv_gauge_set_value(gauge2, 0,lv_slider_get_value(slider));
    	period  = 50*lv_slider_get_value(slider);
    	if (period >= 4800)
    		period = 4800;
    	if (period <=100)
    		period = 100;
    	//__HAL_TIM_SET_AUTORELOAD(&htim3, period);
    	//TIMx->CCR1 = OC_Config->Pulse;
    	//htim5.Instance->CCR4 = period;

    }
}

/**
 * Create a demo application
 */
void demo_create(void)
{
    lv_coord_t hres = lv_disp_get_hor_res(NULL);
    lv_coord_t vres = lv_disp_get_ver_res(NULL);

	static lv_style_t style;
	lv_style_copy(&style, &lv_style_pretty_color);
	style.body.main_color = lv_color_hex3(0x666);     /*Line color at the beginning*/
	style.body.grad_color =  lv_color_hex3(0x666);    /*Line color at the end*/
	style.body.padding.left = 10;                      /*Scale line length*/
	style.body.padding.inner = 8 ;                    /*Scale label padding*/
	style.body.border.color = lv_color_hex3(0x333);   /*Needle middle circle color*/
	style.line.width = 3;
	style.text.color = lv_color_hex3(0x333);
	style.line.color = LV_COLOR_RED;                  /*Line color after the critical value*/

#if LV_DEMO_WALLPAPER
    lv_obj_t * wp = lv_img_create(lv_disp_get_scr_act(NULL), NULL);
    lv_img_set_src(wp, &img_bubble_pattern);
    lv_obj_set_width(wp, hres * 6);
    lv_obj_set_protect(wp, LV_PROTECT_POS);
#endif

    lv_obj_t * obj1 = lv_page_create(NULL,NULL);
    lv_disp_load_scr(obj1);

#if LV_DEMO_WALLPAPER
    lv_obj_set_parent(wp, ((lv_tabview_ext_t *) tv->ext_attr)->content);
    lv_obj_set_pos(wp, 0, -5);
#endif


    gauge1 = lv_gauge_create(obj1,NULL);
    lv_obj_align(gauge1,NULL,LV_ALIGN_CENTER,0,20);
    lv_gauge_set_style(gauge1, LV_GAUGE_STYLE_MAIN, &style);

    lv_obj_t * slider1 = lv_slider_create(obj1,NULL);
	lv_obj_set_size(slider1, 30,140);
    lv_slider_set_range(slider1, 0, 100);
    lv_obj_align(slider1, gauge1,LV_ALIGN_OUT_RIGHT_MID,40,-30);
    lv_obj_set_event_cb(slider1, slider_event_cb);


#if LV_DEMO_SLIDE_SHOW
    lv_task_create(tab_switcher, 3000, LV_TASK_PRIO_MID, tv);
#endif
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t test = 0;
	uint32_t fault_counter = 0;
	uint32_t fault_counter1 = 0;

    float temp_speed;
    float temp_speed1;


  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  lv_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  LCD_SCAN_DIR Lcd_ScanDir = SCAN_DIR_DFT;//SCAN_DIR_DFT = D2U_L2R
  tft_init(Lcd_ScanDir);
  TP_Init(Lcd_ScanDir);
  TP_GetAdFac();//Get the default calibration factor

  pid = pid_create(&ctrldata, &input, &output, &setpoint, kp, ki, kd);
  pid1 = pid_create(&ctrldata1, &input1, &output1, &setpoint, kp, ki, kd);
  // Set controler output limits from 0 to 200
  pid_limits(pid, -20, 20);
  pid_limits(pid1, -20, 20);
  // Allow PID to compute and change output
  pid_auto(pid);
  pid_auto(pid1);

  demo_create();

  my_timer  = 0;
  my_speed1 = 0;
  my_speed2 = 0;
  temp_speed = 0;

  htim2.Instance->CCR2 = 0;
  htim2.Instance->CCR1= 0;

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC,MOTOR1_DIS_Pin, GPIO_PIN_SET);
  while(1)
  {
	  //test = htim4.Instance->CCR2;
	  lv_task_handler();
	  HAL_Delay(5);

	  //for (;;)
	 // {
		if (my_timer>=500)
		{
			temp_speed = (float)my_speed2;
			temp_speed1 = (float)my_speed1;

	    	lv_gauge_set_value(gauge1, 1,my_speed2);
	    	lv_gauge_set_value(gauge2, 1,my_speed1);


			sprintf(label_text,"%06d rps",(uint32_t)temp_speed1);
			lv_label_set_text(speed1,label_text);
			sprintf(label_text,"%06d rps",(uint32_t)temp_speed);
			lv_label_set_text(speed2,label_text);

			//my_speed2=0;
			my_timer=0;
			if (pid_need_compute(pid))
			{
				// Read process feedback
				input = /*process_input()*/temp_speed;
				pid->input = (&temp_speed);
				// Compute new PID output value
				pid_compute(pid);
				//Change actuator value
				//process_output(output);
				htim2.Instance->CCR2 += (int32_t)*pid->output;
#if 0
				if ((htim2.Instance->CCR2 >= 800) && (my_speed2<=2))
				{
					fault_counter++;
					if (fault_counter>=1)
					{
						htim2.Instance->CCR2 = 0;
						htim2.Instance->CCR1 = 0;
						for(;;);
					}
				}else
					fault_counter = 0;
#endif
			}
			if (pid_need_compute(pid1))
			{
				// Read process feedback
				input1 = /*process_input()*/temp_speed1;
				pid1->input = (&temp_speed1);
				// Compute new PID output value
				pid_compute(pid1);
				//Change actuator value
				//process_output(output);
				htim2.Instance->CCR1 += (int32_t)*pid1->output;
#if 0
				if ((htim2.Instance->CCR1 >= 800) && (my_speed1<=2))
				{
					fault_counter1++;
					if (fault_counter>=1)
					{
						htim2.Instance->CCR1 = 0;
						htim2.Instance->CCR2 = 0;
						for(;;);
					}
				}else
					fault_counter1 = 0;
#endif
			}
			my_speed2=0;
			my_speed1=0;

		//}
	  }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
     /* PWM Generation Error */
     Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
     /* PWM Generation Error */
     Error_Handler();
  }

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_BL_Pin_Pin|MOTOR1_DIS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_DC_Pin_Pin|LCD_RST_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TP_CS_Pin_Pin|LCD_CS_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TP_BUSY_Pin_Pin TP_IRQ_Pin_Pin */
  GPIO_InitStruct.Pin = TP_BUSY_Pin_Pin|TP_IRQ_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BL_Pin_Pin MOTOR1_DIS_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin_Pin|MOTOR1_DIS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DC_Pin_Pin LCD_RST_Pin_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin_Pin|LCD_RST_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TP_CS_Pin_Pin LCD_CS_Pin_Pin */
  GPIO_InitStruct.Pin = TP_CS_Pin_Pin|LCD_CS_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
    lv_tick_inc(1);
    my_timer++;
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
