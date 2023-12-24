/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ssd1306.h"
#include "fonts.h"
#include "ocxo.h"
#include "gps.h"
#include "display.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// STM32F410CBT6:Flash 128KB -> last sector 4 (64KB area)
#define FLASH_DATA_SECTOR		FLASH_SECTOR_4
#define FLASH_DATA_ADR			0x08010000
#define FLASH_DATA_SECTORSIZE	0x10000

#define LM73_I2C_ADDR       	0x98
#define OCXO_DEF_THRESHOLD		30		// degree C
#define SW_PRESSED				0
#define SW_SHORT_THRESHOLD  	3		// (Unit:10ms) 30msec
#define SW_LONG_THRESHOLD   	300		// (Unit:10ms) 3000msec

#define STATE_INIT_NOT_RESTORED	0
#define STATE_INIT_RESTORED		1
#define STATE_WAIT_WARMUP		2
#define STATE_WAIT_TIMEVALID	3
#define STATE_WAIT_GPSVALID		4
#define STATE_GPSVALID			5

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
int32_t	tim1_ovf_cnt = 0;
int32_t	cnt_1pps;
bool	previous_cnt_valid = false;
bool	update_cnt_1pps = false;
int32_t	tim11_ovf_cnt = 0;
int32_t	KeyStatus = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// LM73(I2C) Read out TCXO temperature (signed 10 bit)
int32_t LM73_temperature(void){
  uint8_t buf[2];
  HAL_I2C_Master_Receive(&hi2c1, LM73_I2C_ADDR, buf, 2, 10);
  return ((buf[0] << 1) | (buf[1] >> 7));
}

int32_t LoadData32Flash(uint32_t *data1, uint32_t *data2) {
  uint32_t* adr = (uint32_t *)(FLASH_DATA_ADR + FLASH_DATA_SECTORSIZE) - 2;
  for( int i=0; i < FLASH_DATA_SECTORSIZE/4/2; i++ ) {	// search from end of sector
    if ( *adr != 0xFFFFFFFF) {							// if not blank (data exist)
      *data1 = *adr++;
      *data2 = *adr;
      return 0;
    }
    adr -= 2;
  }
  return -1;
}

void SaveData32Flash(uint32_t data1, uint32_t data2) {
  HAL_FLASH_Unlock();

  uint32_t* adr = (uint32_t *)(FLASH_DATA_ADR + FLASH_DATA_SECTORSIZE) - 1;
  if (*adr != 0xFFFFFFFF ) {		// if not spire
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_DATA_SECTOR;	// STM32F410CBT6 FLASH_SECTOR_4
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t r; 					// SectorError
    HAL_FLASHEx_Erase(&erase, &r);
  }

  adr = (uint32_t *)FLASH_DATA_ADR;
  for (int i=0; i < FLASH_DATA_SECTORSIZE/4/2; i++) { // search from top of sector
    if (*adr == 0xFFFFFFFF) {                  // if blank
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)adr++, data1);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)adr, data2);
      break;
    }
    adr += 2;
  }
  HAL_FLASH_Lock();
}

bool IsPushSWPressed(void) {
  return (HAL_GPIO_ReadPin(PushSW_GPIO_Port, PushSW_Pin) == SW_PRESSED);
}

int32_t CheckPushSW(void) {
  static int32_t cnt = 0;
  static bool	 detect_push_long = false;

  if (IsPushSWPressed()) {
    if (cnt > SW_LONG_THRESHOLD -1 ) {
   	  if (detect_push_long) {
   		return 0;
   	  } else {
     	detect_push_long = true;
     	return 2;
   	  }
    } else {
      cnt++;
      return 0;
    }
  } else {
	if ((detect_push_long == false) && (cnt > SW_SHORT_THRESHOLD - 1)) {
      cnt = 0;
      return 1;
	} else {
	  detect_push_long = false;
	  cnt = 0;
	  return 0;
	}
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  bool ocxo_warm_up = false;
  bool gps_valid = false;
  bool gps_time_valid = false;
  bool req_update_state = false;
  bool req_update_display = false;
//bool req_display_location = false;
  bool req_display_msg_save = false;
  int32_t state;
  int32_t next_state;
  int32_t state_delay = 0;
  int32_t disp_mode = 0;
  int32_t disp_hold_cnt = 0;
  int32_t icon_sig_pat;
  int32_t temperature;
  int32_t saved_temperature;
  int32_t ocxo_threshold = OCXO_DEF_THRESHOLD;
  int32_t r;
  uint32_t k1, k2;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  MX_TIM11_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uart6_init();

  if (ssd1306_Init(&hi2c1) != 0) {
    Error_Handler();
  }
  HAL_Delay(1000);

  // load saved parameter
  if (!IsPushSWPressed()) {
	r = LoadData32Flash(&k1, &k2);
	if (r == -1) {
	  state = next_state = STATE_INIT_NOT_RESTORED;
	} else {
	  set_control_pwm_data((int32_t) k1);
	  ocxo_threshold = k2;
      state = next_state = STATE_INIT_RESTORED;
    }
  } else {
	state = next_state = STATE_INIT_NOT_RESTORED;
  }

  // Start Timer
  HAL_TIM_Base_Start_IT(&htim11);	// Timer11
  ocxo_init();						// Timer1

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);// for debug

    // OCXO frequency control
	//   the interval depends on 1PPS generated by GPS module
	if (state > STATE_WAIT_WARMUP) {
	  ocxo_control();
	}

    // PushSW operation
 	r = KeyStatus;			// read and clear KeyStatus
  	KeyStatus = 0;
	if (state == STATE_GPSVALID) {
  	  if (r == 1) {			// short pressed?
  		disp_mode++;
  		if (disp_mode > 2) disp_mode = 0;
  	  }
  	  else if (r == 2) {	// long pressed?
  	    r = get_control_pwm_data();
  	    saved_temperature = temperature - 5;
  	    SaveData32Flash((uint32_t) r, (uint32_t) saved_temperature);
  	    req_display_msg_save = true;
	  }
	}

	// Receive GPS message ( measured time max. 150us)
//	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	uart6_err_ckcl();
	while (get_gps_msg() == true) {
	  // Output NMEA sentence using UART2 (USB serial) for debugging
//    HAL_UART_Transmit(&huart2, (const uint8_t *)gps_msg, strlen((const char *)gps_msg), 10);
	  r = msg_parse(gps_msg);
	  if (r > 0) {  // if received GPRMC message
	    gps_valid = (r & 4)? true : false;
		gps_time_valid = (r & 2)? true : false;
		tim11_ovf_cnt = 0;
		req_update_state = true;
	  }
	}
//	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

	// if GPRMC sentence is not detected, update state/display every 1.2sec
	if (tim11_ovf_cnt > 120) {
		tim11_ovf_cnt = 0;
		req_update_state = true;
	}

    // update state basically every 1sec
	if (req_update_state) {
	  req_update_state = false;

	  // Measure OCXO temperature
	  temperature = LM73_temperature();
	  if (temperature > ocxo_threshold)
	    ocxo_warm_up = true;

	  // state machine
	  if (state_delay == 0) {
		state = next_state;
		req_update_display = true;
		switch(state) {
		  case STATE_INIT_NOT_RESTORED:
		  case STATE_INIT_RESTORED:
		    next_state = STATE_WAIT_WARMUP;
		    state_delay = 3; // 3sec
		    break;

		  case STATE_WAIT_WARMUP:
		    if (ocxo_warm_up) next_state = STATE_WAIT_TIMEVALID;
		    break;

		  case STATE_WAIT_TIMEVALID:
		    if (gps_time_valid) next_state = STATE_WAIT_GPSVALID;
		    break;

		  case STATE_WAIT_GPSVALID:
		    if (gps_valid) next_state = STATE_GPSVALID;
		    else if (!gps_time_valid) next_state = STATE_WAIT_TIMEVALID;
		    break;

		  case STATE_GPSVALID:
		    if (!gps_valid) next_state = STATE_WAIT_GPSVALID;
		    else if (!gps_time_valid) next_state = STATE_WAIT_TIMEVALID;
		    break;

		  default:
		    next_state = STATE_WAIT_WARMUP;
		    break;
		}

	  } else {
		state_delay--;
	  }
	} // if(req_update_state)

	// display control
    if (disp_hold_cnt) {
      disp_hold_cnt--;
    }
    else if (req_update_display) {
      req_update_display = false;
	  switch(state) {
		case STATE_INIT_NOT_RESTORED:
	      ssd1306_Fill(Black);
	      display_msg_not_load();
	      disp_hold_cnt = 3;
		  break;

		case STATE_INIT_RESTORED:
		  ssd1306_Fill(Black);
		  display_msg_restore(get_control_pwm_data(), ocxo_threshold);
		  disp_hold_cnt = 3;
		  break;

		case STATE_WAIT_WARMUP:
		  ssd1306_Fill(Black);
		  display_heater_icon(IPOS_Center_X, IPOS_Center_Y);
		  display_msg_warming_up();
		  display_temperature(temperature);
		  break;

		case STATE_WAIT_TIMEVALID:
		  ssd1306_Fill(Black);
		  display_satellite_icon(IPOS_Center_X, IPOS_Center_Y);
		  display_msg_waiting_gps();
		  display_temperature(temperature);
		  break;

		case STATE_WAIT_GPSVALID:
		  ssd1306_Fill(Black);
		  display_satellite_icon(IPOS_Corner_X, IPOS_Corner_Y);
		  display_temperature(temperature);
		  display_time();
		  display_SatellitesInView();
		  icon_sig_pat = 0;
		  break;

		case STATE_GPSVALID:
		  display_sat_icon_sig(IPOS_Corner_X, IPOS_Corner_Y, icon_sig_pat++);
		  if (icon_sig_pat > 2) icon_sig_pat = 0;
		  display_SatellitesUsed();
		  display_SatellitesInView();
		  display_freq_error();
		  display_blank_row(TPOS_Y3, Font_7x10);
		  display_blank_row(TPOS_Y4, Font_7x10);
		  if (req_display_msg_save) {
			  req_display_msg_save = false;
			  display_msg_save(get_control_pwm_data(), saved_temperature);
			  disp_hold_cnt = 5;
		  } else {
			switch(disp_mode) {
			  case 0:
		        display_control_period();
			    display_temperature(temperature);
		        display_time();
			    display_pwm_data();
			    break;
			  case 1:
		        display_control_period();
			    display_temperature(temperature);
			    display_summing_error();
			    display_pwm_data();
			    break;
			  case 2:
                display_Latitude();
                display_Longitude();
                break;
			  default:
				break;
			} // switch(disp_mode)
		  } // else
		  break;

		default:
		  break;
	  } // switch
	  ssd1306_UpdateScreen(&hi2c1);	// execution time: 100ms
	} // if (req_update_display)

//  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  } // while
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0x8000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 9999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PushSW_Pin */
  GPIO_InitStruct.Pin = PushSW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PushSW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
// static bool  previous_cnt_valid = false;
 static int32_t previous_cnt;

 if(htim == &htim1){
  int32_t tmp = tim1_ovf_cnt;
  tim1_ovf_cnt = 0;          		// clear overflow counter
  int32_t cnt = TIM1-> CCR1; 		// get cnt(16bit) and clear capture interrupt flag

  if (previous_cnt_valid) {			// previous cnt value exists?
   cnt_1pps = (tmp * 65536) + cnt - previous_cnt;     // Yes, calculate whole pulse in 1PSS
   previous_cnt = cnt; 				// store cnt value into previous_count for next calculation
   update_cnt_1pps = true;			// set 1pps cnt update flag
  }
  else {
   previous_cnt = cnt;				// store cnt value into previous_count
   previous_cnt_valid = true;		// set previous count flag
  }
 }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 if(htim == &htim1){				// Timer1
  TIM1-> SR &= 0x1EFE;				// clear update interrupt flag
  tim1_ovf_cnt++;					// increment overflow count
 }
 else if(htim == &htim11){			// Timer11
  TIM11-> SR &= 0x1EFE;				// clear update interrupt flag
  tim11_ovf_cnt++;					// increment overflow count
  if (KeyStatus == 0) {				// Check PushSW
    KeyStatus = CheckPushSW();
  }
 }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
