/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "cmsis_os.h"
#include "libjpeg.h"
#include "app_touchgfx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../mx25l512/mx25l512.h"
#include "../otm8009a/otm8009a.h"
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "defines.h"
#include "external_hmi_vars.h"
#include <inttypes.h>  // Make sure this header is included
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
HAL_StatusTypeDef ret;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define REFRESH_COUNT        1834
#define SDRAM_TIMEOUT                            ((uint32_t)0xFFFF)
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

/* QSPI Error codes */
#define QSPI_OK            ((uint8_t)0x00)
#define QSPI_ERROR         ((uint8_t)0x01)
#define QSPI_BUSY          ((uint8_t)0x02)
#define QSPI_NOT_SUPPORTED ((uint8_t)0x04)
#define QSPI_SUSPENDED     ((uint8_t)0x08)

/* DISPLAY */
#define LCD_ORIENTATION_LANDSCAPE 0x01
#define BOSS_NOT_PRESENT_ENABLED
#define INTERFERENCE_DETECT_ENABLED

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

DSI_HandleTypeDef hdsi;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

JPEG_HandleTypeDef hjpeg;
DMA_HandleTypeDef hdma_jpeg_in;
DMA_HandleTypeDef hdma_jpeg_out;

LTDC_HandleTypeDef hltdc;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 2000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TouchGFXTask */
osThreadId_t TouchGFXTaskHandle;
const osThreadAttr_t TouchGFXTask_attributes = {
  .name = "TouchGFXTask",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for videoTask */
osThreadId_t videoTaskHandle;
const osThreadAttr_t videoTask_attributes = {
  .name = "videoTask",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for i2cTask */
osThreadId_t i2cTaskHandle;
const osThreadAttr_t i2cTask_attributes = {
  .name = "i2cTask",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Serial */
osThreadId_t SerialHandle;
const osThreadAttr_t Serial_attributes = {
  .name = "Serial",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Led */
osThreadId_t LedHandle;
const osThreadAttr_t Led_attributes = {
  .name = "Led",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for autoCycle */
osThreadId_t autoCycleHandle;
const osThreadAttr_t autoCycle_attributes = {
  .name = "autoCycle",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GoHome */
osThreadId_t GoHomeHandle;
const osThreadAttr_t GoHome_attributes = {
  .name = "GoHome",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Timer10Overflow */
osThreadId_t Timer10OverflowHandle;
const osThreadAttr_t Timer10Overflow_attributes = {
  .name = "Timer10Overflow",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Autofindtask */
osThreadId_t AutofindtaskHandle;
const osThreadAttr_t Autofindtask_attributes = {
  .name = "Autofindtask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Buzzer_Task */
osThreadId_t Buzzer_TaskHandle;
const osThreadAttr_t Buzzer_Task_attributes = {
  .name = "Buzzer_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CoolingTask */
osThreadId_t CoolingTaskHandle;
const osThreadAttr_t CoolingTask_attributes = {
  .name = "CoolingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for repeatAutoCycle */
osThreadId_t repeatAutoCycleHandle;
const osThreadAttr_t repeatAutoCycle_attributes = {
  .name = "repeatAutoCycle",
  .stack_size = 500 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for controlDataMutex */
osMutexId_t controlDataMutexHandle;
const osMutexAttr_t controlDataMutex_attributes = {
  .name = "controlDataMutex"
};
/* USER CODE BEGIN PV */

TIM_HandleTypeDef htim2; //timer handle

// Counters
uint32_t overflow_count = 0; //variable to store overflow count
uint64_t microseconds = 0; //variable to store current microseconds

// Status Flags
uint8_t FLAG_GOING_HOME=0;
uint8_t FLAG_REACHED_HOME=0;
uint8_t FLAG_REQUIRE_HOME=0;
uint8_t FLAG_IN_CYCLE=0;
uint8_t FLAG_HALT=0;
uint8_t FLAG_WAITING_IN_CYCLE=0;
uint8_t FLAG_E_STOP_ACTIVE=0;
uint8_t STOP_COUNTERS=0;
uint8_t ANTI_TIE_DOWN_ACTIVE = 0;
uint8_t CYCLE_START_CLEARS=1;

// Timers

uint64_t step_delay=0;
uint32_t left_switch_press_time = 0;
uint32_t right_switch_press_time = 0;
uint8_t left_switch_prev_state = 0;
uint8_t right_switch_prev_state = 0;
uint8_t both_switches_released = 1;

// SSR Board VARS
uint8_t TIP_1;
uint8_t TIP_2;
uint8_t TIP_3;
uint8_t TIP_4;
uint8_t Cooling_Valve;

// SSR Board Received Packet
uint8_t right_switch_received;
uint8_t left_switch_received;
uint8_t home_switch_received;
uint16_t current1_received;
uint16_t current2_received;
uint16_t current3_received;
uint16_t current4_received;

float current1_peak =0;
float current2_peak =0;
float current3_peak =0;
float current4_peak =0;
float current1_amps =0;
float current2_amps =0;
float current3_amps =0;
float current4_amps =0;

// Buffer for received data
uint8_t dataReceived[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t control_data[9]={0,0,0,0,0,0,0,0,0};

// Stepper VARS
//const uint32_t STEP_PER_COUNT =  STEPPER_NUMBER_OF_STEPS / MM_PER_REV;
//const uint32_t STEPS_PER_REV = STEP_PER_COUNT * 100;
const uint32_t STEP_PER_COUNT = 1;
const uint32_t STEPS_PER_REV = 80;
uint8_t FLAG_MANUAL_STEPPER_MOVING=0;

// Linear Encoder Vars
double linear_encoder_mm_1=0;
double Tip_1_Distance_Previous=0;
uint8_t FLAG_DISTANCE_1_CHANGED=0;
uint8_t FLAG_HOLD_DISTANCE_READING=0;
uint8_t data_encoder;


// Fault Handler Vars
uint8_t FLAG_FAULT_ACTIVE=0;
uint8_t current_fault=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_LTDC_Init(void);
static void MX_FMC_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_DMA2D_Init(void);
static void MX_CRC_Init(void);
static void MX_JPEG_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM10_Init(void);
void StartDefaultTask(void *argument);
extern void TouchGFX_Task(void *argument);
extern void videoTaskFunc(void *argument);
void i2cTaskFunc(void *argument);
void print(void *argument);
void LedBlink(void *argument);
void autoCycleTask(void *argument);
void GoHomeTask(void *argument);
void Timer10OverflowTask(void *argument);
void T_Autofind(void *argument);
void T_Buzzer(void *argument);
void Cooling(void *argument);
void repeatAutoCycleTask(void *argument);

/* USER CODE BEGIN PFP */
static void BSP_SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command);

static uint8_t QSPI_ResetMemory(QSPI_HandleTypeDef *hqspi);
static uint8_t QSPI_EnterMemory_QPI(QSPI_HandleTypeDef *hqspi);
static uint8_t QSPI_EnterFourBytesAddress(QSPI_HandleTypeDef *hqspi);
static uint8_t QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi);
static uint8_t QSPI_OutDrvStrengthCfg(QSPI_HandleTypeDef *hqspi);
static uint8_t QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi);
static uint8_t QSPI_AutoPollingMemReady  (QSPI_HandleTypeDef *hqspi, uint32_t Timeout);
static uint8_t BSP_QSPI_EnableMemoryMappedMode(QSPI_HandleTypeDef *hqspi);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t last_stepper_run_time=0;
uint32_t stepper_count=0;
// Function to update the average and buffer
void updateAverage(double new_tip_1_distance, double new_tip_2_distance, double new_tip_3_distance,double new_tip_4_distance) {
// Subtract the oldest value from the sum
tip_1_sum -= tip_1_distance_buffer[current_index];
tip_2_sum -= tip_2_distance_buffer[current_index];
tip_3_sum -= tip_3_distance_buffer[current_index];
tip_4_sum -= tip_4_distance_buffer[current_index];


// Add the new values to the sum and buffer
tip_1_sum += new_tip_1_distance;
tip_2_sum += new_tip_2_distance;
tip_3_sum += new_tip_3_distance;
tip_4_sum += new_tip_4_distance;


tip_1_distance_buffer[current_index] = new_tip_1_distance;
tip_2_distance_buffer[current_index] = new_tip_2_distance;
tip_3_distance_buffer[current_index] = new_tip_3_distance;
tip_4_distance_buffer[current_index] = new_tip_4_distance;


// Move to the next index in a cyclic manner
current_index = (current_index + 1) % BUFFER_SIZE;
// Calculate the averages
average_tip_1 = tip_1_sum / BUFFER_SIZE;
average_tip_2 = tip_2_sum / BUFFER_SIZE;
average_tip_3 = tip_3_sum / BUFFER_SIZE;
average_tip_4 = tip_4_sum / BUFFER_SIZE;

}
//void TIM8_BRK_TIM12_IRQHandler(void)
//{
//    if (__HAL_TIM_GET_FLAG(&htim12, TIM_FLAG_UPDATE) != RESET)
//    {
//        if (__HAL_TIM_GET_IT_SOURCE(&htim12, TIM_IT_UPDATE) != RESET)
//        {
//            __HAL_TIM_CLEAR_IT(&htim12, TIM_IT_UPDATE);
//
//       	 if (FLAG_RUN_STEPPER==1)
//       	  {
//       		 //HAL_TIM_PWM_Start_IT(&htim10, TIM_CHANNEL_1); // Start PWM generation
//       	    last_stepper_run_time = HAL_GetTick(); // Update the time-stamp
//       	    FLAG_RUN_STEPPER=0;
//
////       		if (stepper_count > 4)
////       		{
////       			stepper_count=0;
//       			if (FLAG_STEPPER_DOWN)
//       			{
//       				steps_count++;
//       				down_pulses++;
//       			}
//       			else
//       			{
//       				steps_count--;
//       				up_pulses++;
//       			}
//   				HMI_MANUAL_PLATEN_DISTANCE =steps_count;
//   				HMI_MANUAL_PLATEN_DISTANCE *=1.04;
//       		}
////       		else
////       		{
////       			stepper_count++;
////       		}
////       	  }
//
//      	  // If FLAG_RUN_STEPPER wasn't 1 for more than 10 milliseconds, stop the stepper.
//      	  if (HAL_GetTick() - last_stepper_run_time >= 5)
//      	  {
//      	    //stepper_stop();
//      		HAL_TIM_PWM_Stop_IT(&htim12, TIM_CHANNEL_1); // Stop PWM generation
//      		FLAG_STEPPER_RUNNING=0;
//      	  }
//
//            // Your code to execute on each timer reset/update
//        }
//    }
//  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */
//
//  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim8);
//  HAL_TIM_IRQHandler(&htim12);
//  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */
//
//  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
//}
//volatile uint32_t pulses = 0;  // Global variable to track the number of pulses
//
//
//TIM_HandleTypeDef htim10;
//uint32_t pulse = 0;
//const uint32_t max_pulse = 3200;
//uint32_t pulses_to_move=0;
//double stepper_current_position=0;
//uint8_t FLAG_STEPPER_MOVING_UP;
//uint8_t FLAG_STEPPER_MOVE;
//
//void TIM1_UP_TIM10_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM10_IRQn 0 */
//
//	// taking stepper home at fixed speed
//	// taking stepper home at accelerating speed forming parabola
//	// taking stepper down at fixed speed while checking for linear encoder change, linear encoder fail or max distance reached
//	// taking stepper down at accelerating speed forming a parabola between 0 - work_distance
//	// after each pulse the platen_distance is incremented
//
//	if (pulses_to_move > 0)
//	{
//		pulses_to_move--;
//	}
//	else
//	{
//		HAL_TIM_PWM_Stop_IT(&htim10, TIM_CHANNEL_1); // Stop PWM generation
//	}
//
////	if (FLAG_STEPPER_MOVING_UP)
//
////  if(pulse >= max_pulse)
////  {
////      pulse = 0;
////      HAL_TIM_PWM_Stop_IT(&htim10, TIM_CHANNEL_1); // Stop PWM generation
////      if (htim10.Init.Prescaler > 20)
////      {
////          htim10.Init.Prescaler -=10; // Adjust prescalar for RPM. 99 is 60RPM
////          if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
////            {
////              Error_Handler();
////            }
////      }
////
////
////  }
//  /* USER CODE END TIM10_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim10);
//  /* USER CODE BEGIN TIM10_IRQn 1 */
//
//  /* USER CODE END TIM10_IRQn 1 */
//}
//
//
//void Stepper_Move_To(double position)
//{
//	if (stepper_current_position != position)
//	{
//		FLAG_STEPPER_MOVE=1;
//
//		if (stepper_current_position < position)
//		{
//			stepper_set_dir(STEPPER_DOWN_DIRECTION_SET);
//			FLAG_STEPPER_MOVING_UP=0;
//		}
//		else
//		{
//			stepper_set_dir(STEPPER_UP_DIRECTION_SET);
//			FLAG_STEPPER_MOVING_UP=1;
//		}
//
//		int position_difference = abs((int)stepper_current_position - (int)position);
//		pulses_to_move = STEP_PER_COUNT * position_difference;
//		stepper_current_position = position;
//		HAL_TIM_PWM_Start_IT(&htim10, TIM_CHANNEL_1); // Start PWM generation
//	}
//
//
//}
//
//
//void TIM10_Init(void)
//{
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  htim10.Instance = TIM10;
//  htim10.Init.Prescaler = 99; // Adjust these values for your specific MCU clock
//  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim10.Init.Period = 625; // Adjust these values to get your desired PWM frequency and duty cycle
//  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim10, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim10, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 313; // Set the pulse width
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  HAL_TIM_MspPostInit(&htim10);
//
//  /* Enable the Timer 10 global interrupt */
//   HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 5, 0);
//   HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
//}

void I2C_Error_Handler(void) {
    // Disable the I2C peripheral
    __HAL_I2C_DISABLE(&hi2c1);

    // Reset the I2C peripheral by setting the corresponding bit in the RCC_APB1RSTR register
    __HAL_RCC_I2C1_FORCE_RESET();

    // Add a small delay to allow the reset process to complete
    HAL_Delay(1);

    // Clear the reset bit
    __HAL_RCC_I2C1_RELEASE_RESET();

    // Re-initialize the I2C peripheral using HAL functions
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_Init(&hi2c1);

    // Clear any I2C error flags
    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR | I2C_FLAG_OVR | I2C_FLAG_TIMEOUT);

    // Re-enable the I2C peripheral
    __HAL_I2C_ENABLE(&hi2c1);
}

void success_cycle()
{

	  // Flag for clearing Cycle initials
	  CYCLE_START_CLEARS=1;

	  // Turn Off Pre-Cycle
	  FLAG_IN_PRE_CYCLE=0;

	  FLAG_CYCLE_SUCCESS=1;
}

void reset_linear_encoder(uint8_t slave)
{
	uint8_t data = 'R';
	 //send byte to slave
	    if(HAL_I2C_Master_Transmit(&hi2c1, slave << 1, &data, 1, 1000) != HAL_OK)
	    {
	        //transmission error
	    	I2C_Error_Handler();
	    }
}

void set_encoder_led_color(uint8_t color, uint8_t encoder_id)
{

	if (encoders[encoder_id].Enable == ENABLED)
	{
		encoders[encoder_id].Color = color;
	}

//	switch(color)
//	{
//	case RED:
//		data_encoder = RED;
//		break;
//	case GREEN:
//		data_encoder = GREEN;
//		break;
//	case BLUE:
//		data_encoder = BLUE;
//		break;
//	case OFF:
//		data_encoder = OFF;
//		break;
//	default:
//		data_encoder = OFF;
//		break;
//	}
//
//	if (_enabled)
//	{
//		 //send byte to slave
//		    if(HAL_I2C_Master_Transmit(&hi2c1, encoder_id << 1, &data_encoder, 1, 1000) != HAL_OK)
//		    {
//		        //transmission error
//		    	I2C_Error_Handler();
//		    }
//
//		    HAL_Delay(20);
//	}

}

void print_string(const char *str) {
  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 1000);
}

void print_uint16(uint16_t number) {
  char num_buffer[6];
  uint8_t i = 0;

  if (number == 0) {
    print_string("0");
    return;
  }

  while (number > 0) {
    num_buffer[i++] = '0' + (number % 10);
    number /= 10;
  }

  // Reverse the string
  for (uint8_t j = 0; j < i / 2; j++) {
    char temp = num_buffer[j];
    num_buffer[j] = num_buffer[i - j - 1];
    num_buffer[i - j - 1] = temp;
  }

  num_buffer[i] = '\0';
  print_string(num_buffer);
}

void print_uint32(uint32_t number) {
  char num_buffer[11]; // Buffer size increased to accommodate uint32_t
  uint8_t i = 0;

  if (number == 0) {
    print_string("0");
    return;
  }

  while (number > 0) {
    num_buffer[i++] = '0' + (number % 10);
    number /= 10;
  }

  // Reverse the string
  for (uint8_t j = 0; j < i / 2; j++) {
    char temp = num_buffer[j];
    num_buffer[j] = num_buffer[i - j - 1];
    num_buffer[i - j - 1] = temp;
  }

  num_buffer[i] = '\0';
  print_string(num_buffer);
}

void print_int16(int16_t number) {
  char num_buffer[7];
  uint8_t i = 0;

  if (number == 0) {
    print_string("0");
    return;
  }

  if (number < 0) {
    print_string("-");
    number = -number;
  }

  while (number > 0) {
    num_buffer[i++] = '0' + (number % 10);
    number /= 10;
  }

  // Reverse the string
  for (uint8_t j = 0; j < i / 2; j++) {
    char temp = num_buffer[j];
    num_buffer[j] = num_buffer[i - j - 1];
    num_buffer[i - j - 1] = temp;
  }

  num_buffer[i] = '\0';
  print_string(num_buffer);
}

void print_number(uint32_t number) {
  char num_buffer[12];
  uint8_t i = 0;

  if (number == 0) {
    print_string("0");
    return;
  }

  while (number > 0) {
    num_buffer[i++] = '0' + (number % 10);
    number /= 10;
  }

  // Reverse the string
  for (uint8_t j = 0; j < i / 2; j++) {
    char temp = num_buffer[j];
    num_buffer[j] = num_buffer[i - j - 1];
    num_buffer[i - j - 1] = temp;
  }

  num_buffer[i] = '\0';
  print_string(num_buffer);
}

void print_uint64(uint64_t number) {
  char num_buffer[21]; // Large enough buffer for a 64-bit unsigned integer (20 digits + null terminator)
  uint8_t i = 0;

  if (number == 0) {
    print_string("0");
    return;
  }

  while (number > 0) {
    num_buffer[i++] = '0' + (number % 10);
    number /= 10;
  }

  // Reverse the string
  for (uint8_t j = 0; j < i / 2; j++) {
    char temp = num_buffer[j];
    num_buffer[j] = num_buffer[i - j - 1];
    num_buffer[i - j - 1] = temp;
  }

  num_buffer[i] = '\0';
  print_string(num_buffer);
}

void print_float(float number) {
  int32_t int_part = (int32_t)number;
  uint32_t frac_part = (uint32_t)((number - int_part) * 100);

  if (number < 0) {
    print_string("-");
    int_part = -int_part;
    frac_part = -frac_part;
  }

  print_uint32(int_part);
  print_string(".");
  if (frac_part < 10) {
    print_string("0");
  }
  print_uint32(frac_part);
}

void TIM2_IRQHandler(void) //timer interrupt handler
{
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) //check for update flag
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET) //check for update interrupt
        {
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE); //clear update interrupt flag
            overflow_count++; //increment overflow count
        }
    }
}
void init_timer(void) //timer initialization function
{
    __HAL_RCC_TIM2_CLK_ENABLE(); //enable timer clock

    htim2.Instance = TIM2; //set timer instance
    htim2.Init.Period = 0xFFFFFFFF; //set timer period
    htim2.Init.Prescaler = 99; //set timer prescaler for microseconds
    htim2.Init.ClockDivision = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.RepetitionCounter = 0;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) //initialize timer
    {
        //Error_Handler(); //handle error
    }
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0); //set timer interrupt priority
    HAL_NVIC_EnableIRQ(TIM2_IRQn); //enable timer interrupt
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE); //enable update interrupt
    HAL_TIM_Base_Start(&htim2); //start timer
}
uint64_t micros(void) //function to get current microseconds
{
    microseconds = (uint64_t)overflow_count * 0xFFFFFFFF + __HAL_TIM_GET_COUNTER(&htim2); //update microseconds
    return microseconds;
}

uint64_t millis(void) //function to get current milliseconds
{
    return micros() / 1000;
}

void delayMicroseconds(uint64_t us)
{
	uint64_t _current_us = micros();
	while(micros() - _current_us < us );
}


//void stepper_setSpeed(uint32_t whatSpeed)
//{
//  step_delay = 60L * 1000L * 1000L / STEPPER_NUMBER_OF_STEPS / whatSpeed;
//}
uint32_t current_stepper_speed=40;
uint32_t last_stepper_speed_set=40;
uint32_t stepper_accel_start_time=0;
//uint16_t arr=0;

//double get_velocity()
//{
//	uint32_t current_velocity_time = HAL_GetTick();
//	static uint32_t last_velocity_time=0;
//	uint32_t v_time_passed = current_velocity_time -  last_velocity_time;
//	last_velocity_time = current_velocity_time;
//
//	double current_velocity_distance = HMI_MANUAL_PLATEN_DISTANCE;
//	static double last_velocity_distance = 0;
//	double v_distance_travelled= fabs(current_velocity_distance - last_velocity_distance);
//
//	return v_distance_travelled/v_time_passed;
//}
//
//double get_acceleration()
//{
//    static double last_velocity = 0;
//    static uint32_t last_velocity_time = 0;
//
//    uint32_t current_velocity_time = HAL_GetTick();
//    uint32_t v_time_passed = current_velocity_time - last_velocity_time;
//    last_velocity_time = current_velocity_time;
//
//    double current_velocity = get_velocity();  // Assuming you have a working get_velocity function
//    double v_acceleration = (current_velocity - last_velocity) / (v_time_passed / 1000.0);  // Convert time to seconds
//
//    last_velocity = current_velocity;
//
//    return v_acceleration;
//}


void stepper_setSpeed(uint32_t whatSpeed)
{
	last_stepper_speed_set = whatSpeed;
}

void stepper_setSpeed_accel(uint32_t whatSpeed)
{
    static uint32_t lastSpeed = 0;  // Static variable to keep the last speed set

    if (whatSpeed != lastSpeed)
    {
    	if (HAL_GetTick() - stepper_accel_start_time > 1)
    	{
    		stepper_accel_start_time = HAL_GetTick();
    		if (fabs(whatSpeed - current_stepper_speed) > 20)
    		    	{
    		        	if (whatSpeed > current_stepper_speed )
    		        	{
    		        		current_stepper_speed+=8;
    		        	}
    		        	else if (whatSpeed < current_stepper_speed)
    		        	{
    		        		current_stepper_speed-=8;
    		        	}
    		    	}

    		    	else
    		    	{
    		    		current_stepper_speed = whatSpeed;
    		    	}

    		        const uint32_t pulsesPerRevolution = 1000;
    		        const uint32_t systemClock = 200000000;
    		        uint16_t arr = 100;

    		        // Calculate the required prescaler based on desired RPM
    		        uint32_t prescaler = systemClock / ( ((pulsesPerRevolution * current_stepper_speed)/60) * arr) - 1;
    		        htim10.Instance->PSC = (uint16_t)prescaler - 1;

    				lastSpeed = current_stepper_speed; // Update the last speed value
    	}


    }
}


void stepper_set_dir(uint8_t dir) {
  HAL_GPIO_WritePin(STEP_DIR_PORT, STEP_DIR, dir);
  if (dir)
  {
	  FLAG_STEPPER_DOWN=0;
  }
  else
  {
	  FLAG_STEPPER_DOWN=1;
  }
}

void Buzzer(uint16_t beeps, uint32_t buzzer_delay)
{
	Buzzer_beeps = beeps;
	Buzzer_delay = buzzer_delay;
	FLAG_RUN_BUZZER=1;
}

void Buzz (uint16_t alarm_mode)
{
	switch(alarm_mode)
	{
	case FAST: Buzzer(4,200);
		break;
	case MEDIUM: Buzzer(3,500);
		break;
	case SLOW: Buzzer(3,1000);
		break;
	default:
		break;
	}
}

void cooling_burst(uint32_t _time_cooling)
{
  	  	Cooling_Valve = 1;
	  	osDelay(_time_cooling * 100);
	  	Cooling_Valve = 0;
}

uint32_t Heat_Start_Time=0;
uint32_t Reached_time[4] = {0,0,0,0};
uint32_t Last_reached_tip=0;
uint32_t Last_reached_time=0;
uint32_t Tip_Time_Offset[4]={0,0,0,0};
void heating_delays()
{
	// Find the tip that reached last
	Last_reached_time = Reached_time[0];
	for (int i=1; i < 4;i++)
	{
		if (Last_reached_time < Reached_time[i])
		{
			Last_reached_time = Reached_time[i];
			Last_reached_tip =i;
		}
	}

	// Set offset for each tip
	for (int i=0; i < 4; i++)
	{
		if (i != Last_reached_tip)
		{
			Tip_Time_Offset[i] += Last_reached_time - Reached_time[i];
		}
		else
		{
			Tip_Time_Offset[i] += 0;
		}
	}


}

uint8_t FLAG_HEATING_START_TIME_SET=0;



void UpdatePlatenDistance(void)
{
    uint32_t currentCounterValue = __HAL_TIM_GET_COUNTER(&htim12);

    uint32_t pulseDifference;

    // Check for overflow
    if (currentCounterValue < lastCounterValue)
    {
        pulseDifference = (MAX_COUNTER_VALUE - lastCounterValue) + currentCounterValue + 1;
    }
    else
    {
        pulseDifference = currentCounterValue - lastCounterValue;
    }

    // Update the last known counter value
    lastCounterValue = currentCounterValue;

    // Calculate the distance moved since the last check
    double distanceChange = pulseDifference * PULSE_DISTANCE;


    // Update the HMI_MANUAL_PLATEN_DISTANCE based on the direction
    if (FLAG_STEPPER_DOWN == 1)
    {
        HMI_MANUAL_PLATEN_DISTANCE += distanceChange;
    }
    else
    {
        HMI_MANUAL_PLATEN_DISTANCE -= distanceChange;
    }
}

void MonitorStepper(void)
{
    // If FLAG_RUN_STEPPER is 1, update the last_stepper_run_time.
    if (FLAG_RUN_STEPPER == 1)
    {
        last_stepper_run_time = HAL_GetTick(); // Update the time-stamp
        FLAG_RUN_STEPPER = 0;
    }
    // If FLAG_RUN_STEPPER wasn't set for more than 5 milliseconds, stop the stepper.
    else if (HAL_GetTick() - last_stepper_run_time >= 10)
    {
        HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1); // Stop PWM generation
        FLAG_STEPPER_RUNNING = 0;
    }
}
// Function to step the stepper motor
//void stepper_step(void) {
//  HAL_GPIO_TogglePin(STEP_PORT, STEP_PIN);
//}

//void stepper_Move_Up()
//{
//	if (!FLAG_E_STOP_ACTIVE)
//	{
//	  //Move the stepper clockwise
//	  stepper_set_dir(STEPPER_UP_DIRECTION_SET);
//  if (HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin))
//  {
//	  for(int i=0;i<STEP_PER_COUNT;i++)
//	  {
//		  if (HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin))
//		  {
//			  stepper_step();
//			  delayMicroseconds(step_delay);
//		  }
//	  }
//	  HMI_MANUAL_PLATEN_DISTANCE--;
//  }
//	}
//}
//
//
//void stepper_Move_Down()
//{
//	  //Move the stepper anti-clockwise
//	if ((!FLAG_E_STOP_ACTIVE) &&
//	   (HMI_MANUAL_PLATEN_DISTANCE < MAX_DISTANCE) &&
//	   (FAULT_ENCODER_1_FLAG ==0) &&
//	   (FAULT_ENCODER_2_FLAG ==0) &&
//	   (FAULT_ENCODER_3_FLAG ==0)
//	   )
//	{
//		  stepper_set_dir(STEPPER_DOWN_DIRECTION_SET);
////		  for(int i=0;i<STEP_PER_COUNT;i++)
////		  {
////			  stepper_step();
//		  HAL_GPIO_TogglePin(STEP_PORT, STEP_PIN);
//			  delayMicroseconds(step_delay);
////		  }
//		  HMI_MANUAL_PLATEN_DISTANCE++;
//	}
//
//}

void stepper_start()
{
    	FLAG_RUN_STEPPER =1;
    	if (FLAG_STEPPER_RUNNING==0)
    	{
    		HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1); // Start PWM generation
    		FLAG_STEPPER_RUNNING=1;
    	}


}

void stepper_stop()
{
	if (FLAG_RUN_STEPPER==1)
	{
		 FLAG_RUN_STEPPER =0;
		 HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
	}
	osDelay(300);

}
void stepper_Move_Up()
{
	if (!FLAG_E_STOP_ACTIVE)
	{
	  //Move the stepper clockwise
	  stepper_set_dir(STEPPER_UP_DIRECTION_SET);
  if (HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin))
  {
		  if (HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin))
		  {
			  stepper_start();
		  }
  }
	}
}


void stepper_Move_Down()
{
	  //Move the stepper anti-clockwise
	if ((!FLAG_E_STOP_ACTIVE) &&
	   (HMI_MANUAL_PLATEN_DISTANCE < MAX_DISTANCE) &&
	   (FAULT_ENCODER_1_FLAG ==0) &&
	   (FAULT_ENCODER_2_FLAG ==0) &&
	   (FAULT_ENCODER_3_FLAG ==0) &&
	   (FAULT_ENCODER_4_FLAG ==0)
	   )
	{
		  stepper_set_dir(STEPPER_DOWN_DIRECTION_SET);
		  stepper_start();
	}

}

void Go_Home()
{

	while(HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) && FLAG_HALT ==0)
	{
		if (HMI_MANUAL_PLATEN_DISTANCE > 0 && HMI_MANUAL_PLATEN_DISTANCE < 400 )
		{
			stepper_setSpeed(STEPPER_SPEED_MID_LOW);
		}
		stepper_Move_Up();
		FLAG_GOING_HOME=1;
		osDelay(2);
	}

	if (FLAG_CYCLE_STARTED)
	{
		FLAG_CYCLE_COMPLETED=1;
	}

	stepper_stop();

	stepper_setSpeed(STEPPER_SPEED_LOW);
	while(!HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) && FLAG_HALT ==0)
	{
		stepper_Move_Down();
		FLAG_GOING_HOME=1;
		osDelay(2);
	}

	stepper_stop();

	FLAG_GOING_HOME=0;
	FLAG_REACHED_HOME =1;
	FLAG_REQUIRE_HOME =0;
	HMI_MANUAL_PLATEN_DISTANCE=0;
	HMI_Distance_Counter=0;
	FLAG_FAULT_ACTIVE=0;
	FLAG_MANUAL_STEPPER_MOVING=0;
	ANTI_TIE_DOWN_ACTIVE=0;
}

void reset_linear_encoders()
{

	  set_encoder_led_color(OFF,encoders[0].ID);
	  set_encoder_led_color(OFF,encoders[1].ID);
	  set_encoder_led_color(OFF,encoders[2].ID);
	  set_encoder_led_color(OFF,encoders[3].ID);

	  // Reset All Enabled Encoders
	  for (int i=0; i < TOTAL_ENCODERS;i++)
	  {
			  encoders[i].Reset =1;
	  }
}

void Go_Home_AutoCycle()
{
	// Move the stepper fast initially
	FLAG_GOING_HOME=1;
	while(HMI_MANUAL_PLATEN_DISTANCE > 2000 && (HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin)))
	{
		stepper_Move_Up();
		osDelay(2);
	}

	// Slow stepper down at close point to start
	stepper_setSpeed(CD_LOW);
	while(HMI_MANUAL_PLATEN_DISTANCE > 0 && (HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin)))
	{
		stepper_Move_Up();
		osDelay(2);
	}
	stepper_stop();

	FLAG_GOING_HOME=0;
	FLAG_REACHED_HOME =1;
	FLAG_REQUIRE_HOME =0;
	HMI_Distance_Counter=0;
	FLAG_FAULT_ACTIVE=0;
	FLAG_MANUAL_STEPPER_MOVING=0;
	ANTI_TIE_DOWN_ACTIVE=0;
}


void Auto_Find()
{
	FLAG_HOLD_DISTANCE_READING=0;

	stepper_setSpeed(PU_HOME);

	// Go Home With Switch
	Go_Home();

	// Reset Encoders
	reset_linear_encoders();

	stepper_setSpeed(AF_HIGH);

	// Hit the Boss and Stop
	while(FLAG_E_STOP_ACTIVE ==0 && FLAG_GOING_HOME==0)
	{
		if (       (fabs(Tip_1_Distance) > BOSS_SENSE_HEIGHT)
				|| (fabs(Tip_2_Distance) > BOSS_SENSE_HEIGHT)
				|| (fabs(Tip_3_Distance) > BOSS_SENSE_HEIGHT)
				|| (fabs(Tip_4_Distance) > BOSS_SENSE_HEIGHT)
				)
			{
				break;
			}
		stepper_Move_Down();
		osDelay(1);
	}

	// Stop The Stepper
	stepper_stop();

	// Set stepper to low speed and start moving up
	stepper_setSpeed(AF_SLOW);
	slow_down_distance= HMI_MANUAL_PLATEN_DISTANCE - MOVE_UP_HITTING_BOSS_DISTANCE;
	while((HMI_MANUAL_PLATEN_DISTANCE > slow_down_distance) && FLAG_GOING_HOME ==0)
	{
		stepper_Move_Up();
		osDelay(1);
	}

	// Reset Encoders
	reset_linear_encoders();

	// Stop The Stepper
	stepper_stop();

	// Keep moving down until Tip Distance is less than 8 mm and emergency switch is off
	while(FLAG_E_STOP_ACTIVE ==0 && FLAG_GOING_HOME==0)
	{
		if (       (fabs(Tip_1_Distance) > AUTOFIND_STOP_HEIGHT)
				|| (fabs(Tip_2_Distance) > AUTOFIND_STOP_HEIGHT)
				|| (fabs(Tip_3_Distance) > AUTOFIND_STOP_HEIGHT)
				|| (fabs(Tip_4_Distance) > AUTOFIND_STOP_HEIGHT)
				)
			{
				break;
			}
		stepper_Move_Down();
		osDelay(1);
	}

	// Stop The Stepper
	stepper_stop();

	osDelay(200);

	// Save Current Position as work position
	HMI_TEST_PRESS_DOWN_POSITION = HMI_MANUAL_PLATEN_DISTANCE;

		Tip_1_Compression = fabs(Tip_1_Distance);
		Tip_2_Compression = fabs(Tip_2_Distance);
		Tip_3_Compression = fabs(Tip_3_Distance);
		Tip_4_Compression = fabs(Tip_4_Distance);

    // Save current distance as Tip Compressions
    Tip_1_Compression = fabs(Tip_1_Distance);
    Tip1_Boss_Start_Height = HMI_TEST_PRESS_DOWN_POSITION - (Tip_1_Compression*100);
    HMI_BOSS_HEIGHT = Tip1_Boss_Start_Height;

    // Add Tip 2 and Tip 3
    Tip_2_Compression = fabs(Tip_2_Distance);
    Tip2_Boss_Start_Height = HMI_TEST_PRESS_DOWN_POSITION - (Tip_2_Compression*100);

    Tip_3_Compression = fabs(Tip_3_Distance);
    Tip3_Boss_Start_Height = HMI_TEST_PRESS_DOWN_POSITION - (Tip_3_Compression*100);

    Tip_4_Compression = fabs(Tip_4_Distance);
    Tip4_Boss_Start_Height = HMI_TEST_PRESS_DOWN_POSITION - (Tip_4_Compression*100);

    FLAG_HOLD_DISTANCE_READING=1;
	// Go Home

    if (AUTOFIND_STOP_HEIGHT <= 3.1)
    {
    	HMI_TEST_PRESS_DOWN_POSITION = HMI_MANUAL_PLATEN_DISTANCE +  ( (8-AUTOFIND_STOP_HEIGHT) * 100);
    }

    osDelay(200);

    stepper_setSpeed(AF_HIGH);
    Go_Home_AutoCycle();
	FLAG_WORK_POSTION_SAVED=1;

}

double start_platen_position=0;
double start_1_platen_position=0;
double start_2_platen_position=0;
double start_3_platen_position=0;
double start_4_platen_position=0;
double current_platen_position=0;
double previous_platen_distance=0;
double platen_counter_micro=0;

void linear_encoder_calibrate()
{
	stepper_setSpeed(PU_HOME);

	// Go Home
	Go_Home();

	// Reset Encoders
	reset_linear_encoders();

	stepper_setSpeed(STEPPER_SPEED_MID_LOW);
	uint8_t flag_position_1_saved =0;
	uint8_t flag_position_2_saved =0;
	uint8_t flag_position_3_saved =0;
	uint8_t flag_position_4_saved =0;

	// Hit the Boss and Stop
	while(FLAG_E_STOP_ACTIVE ==0 && FLAG_GOING_HOME==0)
	{

		if (Tip_1_Distance > 0.1)
		{
			if (flag_position_1_saved==0)
			{
				start_1_platen_position = HMI_MANUAL_PLATEN_DISTANCE-Tip_1_Distance;
				flag_position_1_saved=1;
			}

		}
		if (Tip_2_Distance > 0.1)
		{
			if (flag_position_2_saved==0)
			{
			start_2_platen_position = HMI_MANUAL_PLATEN_DISTANCE-Tip_2_Distance;
			flag_position_2_saved=1;
			}
		}
		if (Tip_3_Distance > 0.1)
		{
			if (flag_position_3_saved==0)
			{
			start_3_platen_position = HMI_MANUAL_PLATEN_DISTANCE-Tip_3_Distance;
			flag_position_3_saved=1;
			}
		}
		if (Tip_4_Distance > 0.1)
		{
			if (flag_position_4_saved==0)
			{
			start_4_platen_position = HMI_MANUAL_PLATEN_DISTANCE-Tip_4_Distance;
			flag_position_4_saved=1;
			}
		}

		if (       (Tip_1_Distance > 0.1)
				&& (Tip_2_Distance > 0.1)
				&& (Tip_3_Distance > 0.1)
				&& (Tip_4_Distance > 0.1)
				)
			{
				break;
			}
		stepper_Move_Down();
		osDelay(1);
	}

	// Stop The Stepper
	stepper_stop();

	// Reset Encoders
	reset_linear_encoders();

	osDelay(500);

	// disable the timer on that pin
	// produce 500 pulses through GPIO

	stepper_setSpeed(20);

	start_platen_position = HMI_MANUAL_PLATEN_DISTANCE;
	current_platen_position = HMI_MANUAL_PLATEN_DISTANCE;
	previous_platen_distance=HMI_MANUAL_PLATEN_DISTANCE;
	platen_counter_micro=0;

	while ( current_platen_position < (start_platen_position + 800)
			&&
			(       (Tip_1_Distance < AUTOFIND_STOP_HEIGHT)
//					|| (fabs(Tip_2_Distance) > AUTOFIND_STOP_HEIGHT)
//					|| (fabs(Tip_3_Distance) > AUTOFIND_STOP_HEIGHT)
//					|| (fabs(Tip_4_Distance) > AUTOFIND_STOP_HEIGHT)
			)
	)
	{
		stepper_Move_Down();
		osDelay(1);
		if (current_platen_position > (previous_platen_distance+5))
		{
			// Stop The Stepper
			stepper_stop();
			double position_difference = current_platen_position - previous_platen_distance;
			platen_counter_micro+=position_difference;
			previous_platen_distance = current_platen_position;
			//osDelay(300);

		}
		current_platen_position = HMI_MANUAL_PLATEN_DISTANCE;
	}
	// Stop The Stepper
	stepper_stop();

	// Go Home
	Go_Home();

}


void reset_tips(uint8_t _tip1,uint8_t _tip2,uint8_t _tip3,uint8_t _tip4)
{
	if (_tip1)
	{
		TIP_1 = 0;
		Tip_1_Energy_Consumed=0;
		//control_data[0]=0;
		FLAG_TIP1_HEATING_START=0;
	}

	if (_tip2)
	{
		TIP_2 = 0;
		Tip_2_Energy_Consumed=0;
		//control_data[1]=0;
		FLAG_TIP2_HEATING_START=0;
	}

	if (_tip3)
	{
		TIP_3 = 0;
		Tip_3_Energy_Consumed=0;
		//control_data[2]=0;
		FLAG_TIP3_HEATING_START=0;
	}

	if (_tip4)
	{
		TIP_4 = 0;
		Tip_4_Energy_Consumed=0;
		//control_data[2]=0;
		FLAG_TIP4_HEATING_START=0;
	}

}

void estop (uint8_t e_stop_state)
{
	if (e_stop_state)
	{
		FLAG_REQUIRE_HOME=1;
		FLAG_E_STOP_ACTIVE =1;
		STOP_COUNTERS =1;
	}

	if (!e_stop_state)
	{
		FLAG_E_STOP_ACTIVE =0;
		STOP_COUNTERS =0;
	}
}


void set_home_banner()
{

	if (FAULT_SSR_1_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_SSR_1_FAIL, strlen(STRING_SSR_1_FAIL)+1);
		HMI_BANNER_COLOR = 2;
	}
	else if (FAULT_ESTOP_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_EMERGENCY_STOP, strlen(STRING_EMERGENCY_STOP)+1);
		HMI_BANNER_COLOR = 1;
	}

	else if (FAULT_HANDS_RELEASED_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT,STRING_FAULT_HANDS_RELEASED, strlen(STRING_FAULT_HANDS_RELEASED)+1);
		HMI_BANNER_COLOR = 1;
	}

	else if (FLAG_GOING_HOME && FLAG_CYCLE_COMPLETED ==0)
	{
		// Check if 500ms have passed
	    if (HAL_GetTick() - lastFlashTick >= 500) {
	        // Save the tick
	        lastFlashTick = HAL_GetTick();
	        // Toggle color
	        HMI_BANNER_COLOR = (HMI_BANNER_COLOR == 2) ? 3 : 2;
	    }
		memcpy(HMI_HOME_BANNER_TEXT,STRING_GOING_HOME,strlen(STRING_GOING_HOME)+1);
	}

	else if (FLAG_REQUIRE_HOME)
	{
		memcpy(HMI_HOME_BANNER_TEXT,STRING_HOME_MACHINE,strlen(STRING_HOME_MACHINE)+1);
		HMI_BANNER_COLOR = 2;
	}

	else if (FAULT_SE108_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_FAULT_SE108, strlen(STRING_FAULT_SE108)+1);
		HMI_BANNER_COLOR = 2;
	}
	else if (FAULT_SE208_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_FAULT_SE208, strlen(STRING_FAULT_SE208)+1);
		HMI_BANNER_COLOR = 2;
	}
	else if (FAULT_SE308_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_FAULT_SE308, strlen(STRING_FAULT_SE308)+1);
		HMI_BANNER_COLOR = 2;
	}
	else if (FAULT_SE408_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_FAULT_SE408, strlen(STRING_FAULT_SE408)+1);
		HMI_BANNER_COLOR = 2;
	}

	else if (FAULT_ENCODER_1_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_ENCODER_1_FAIL, strlen(STRING_ENCODER_1_FAIL)+1);
		HMI_BANNER_COLOR = 2;
	}
	else if (FAULT_ENCODER_2_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_ENCODER_2_FAIL, strlen(STRING_ENCODER_2_FAIL)+1);
		HMI_BANNER_COLOR = 2;
	}

	else if (FAULT_ENCODER_3_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_ENCODER_3_FAIL, strlen(STRING_ENCODER_3_FAIL)+1);
		HMI_BANNER_COLOR = 2;
	}
	else if (FAULT_ENCODER_4_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_ENCODER_4_FAIL, strlen(STRING_ENCODER_4_FAIL)+1);
		HMI_BANNER_COLOR = 2;
	}

	else if (FAULT_TE112_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_FAULT_TE112, strlen(STRING_FAULT_TE112)+1);
		HMI_BANNER_COLOR = 2;
	}

	else if (FAULT_TE212_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_FAULT_TE212, strlen(STRING_FAULT_TE212)+1);
		HMI_BANNER_COLOR = 2;
	}

	else if (FAULT_TE312_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_FAULT_TE312, strlen(STRING_FAULT_TE312)+1);
		HMI_BANNER_COLOR = 2;
	}
	else if (FAULT_TE412_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_FAULT_TE412, strlen(STRING_FAULT_TE412)+1);
		HMI_BANNER_COLOR = 2;
	}

	else if (FAULT_MAX_DISTANCE_FLAG)
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_FAULT_MAX_DISTANCE, strlen(STRING_FAULT_MAX_DISTANCE)+1);
		HMI_BANNER_COLOR = 2;
	}

	else if (FLAG_BOSS_NOT_PRESENT[0])
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_FAULT_BOSS_NOT_PRESENT_TIP1, strlen(STRING_FAULT_BOSS_NOT_PRESENT_TIP1)+1);
		HMI_BANNER_COLOR = 2;
	}

	else if (FLAG_BOSS_NOT_PRESENT[1])
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_FAULT_BOSS_NOT_PRESENT_TIP2, strlen(STRING_FAULT_BOSS_NOT_PRESENT_TIP2)+1);
		HMI_BANNER_COLOR = 2;
	}

	else if (FLAG_BOSS_NOT_PRESENT[2])
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_FAULT_BOSS_NOT_PRESENT_TIP3, strlen(STRING_FAULT_BOSS_NOT_PRESENT_TIP3)+1);
		HMI_BANNER_COLOR = 2;
	}

	else if (FLAG_BOSS_NOT_PRESENT[3])
	{
		memcpy(HMI_HOME_BANNER_TEXT, STRING_FAULT_BOSS_NOT_PRESENT_TIP4, strlen(STRING_FAULT_BOSS_NOT_PRESENT_TIP4)+1);
		HMI_BANNER_COLOR = 2;
	}
	else if (HMI_TEST_PRESS_DOWN_POSITION < MIN_WORK_POSITION || FLAG_WORK_POSTION_SAVED==0)
	{
		memcpy(HMI_HOME_BANNER_TEXT,STRING_WORK_HEIGHT,strlen(STRING_WORK_HEIGHT)+1);
		HMI_BANNER_COLOR = 2;
	}
	else if (FLAG_IN_CYCLE && FLAG_WORK_POSTION_SAVED ==1 && FLAG_FIRST_CYCLE==0)
	{
		elapsed_time = HAL_GetTick() - cycle_start_time;  // Get the elapsed time in milliseconds
		uint32_t seconds = elapsed_time / 1000;  // Convert to seconds
		uint32_t milliseconds = (elapsed_time % 1000) / 10;  // Get the remaining milliseconds (2 digits)

		// Add leading zero if seconds less than 10
		if (seconds < 10) {
		    seconds_str[0] = '0';
		    itoa(seconds, &seconds_str[1], 10);
		} else {
		    itoa(seconds, seconds_str, 10);
		}

		// Add leading zero if milliseconds less than 10
		if (milliseconds < 10) {
		    milliseconds_str[0] = '0';
		    itoa(milliseconds, &milliseconds_str[1], 10);
		} else {
		    itoa(milliseconds, milliseconds_str, 10);
		}

		memcpy(HMI_HOME_BANNER_TEXT, "IN CYCLE - ", 11);
		memcpy(HMI_HOME_BANNER_TEXT + 11, seconds_str, strlen(seconds_str));
		memcpy(HMI_HOME_BANNER_TEXT + 11 + strlen(seconds_str), ".", 2);
		memcpy(HMI_HOME_BANNER_TEXT + 11 + strlen(seconds_str) + 1, milliseconds_str, strlen(milliseconds_str) + 1);

		// Check if 500ms have passed
	    if (HAL_GetTick() - lastFlashTick >= 500) {
	        // Save the tick
	        lastFlashTick = HAL_GetTick();
	        // Toggle color
	        HMI_BANNER_COLOR = (HMI_BANNER_COLOR == 2) ? 3 : 2;
	    }

	}

	else if (ANTI_TIE_DOWN_ACTIVE ==0 && FLAG_CYCLE_COMPLETED ==1 && FLAG_WORK_POSTION_SAVED ==1 && FLAG_FIRST_CYCLE==0)
	{
		memcpy(HMI_HOME_BANNER_TEXT, "CYCLE COMPLETE - ", 17);
	    memcpy(HMI_HOME_BANNER_TEXT + 17, seconds_str, strlen(seconds_str));
	    memcpy(HMI_HOME_BANNER_TEXT + 17 + strlen(seconds_str), ".", 2);
	    memcpy(HMI_HOME_BANNER_TEXT + 17 + strlen(seconds_str) + 1, milliseconds_str, strlen(milliseconds_str) + 1);
	    HMI_BANNER_COLOR = 0;

	}

	else if (ANTI_TIE_DOWN_ACTIVE ==0 && FLAG_HOMING_WITH_COOLING==0)
	{
		memcpy(HMI_HOME_BANNER_TEXT,STRING_ANTI_TIE_DOWN_ACTIVE,strlen(STRING_ANTI_TIE_DOWN_ACTIVE)+1);
		HMI_BANNER_COLOR = 0;
	}



}

void assignControlData() {

//	if (FLAG_IN_CYCLE==0)
//	{
//		if (HMI_MANUAL_TIP_1)
//		{
//			FLAG_TIP1_HEATING_START=1;
//		}
//		else
//		{
//			FLAG_TIP1_HEATING_START=0;
//			HOME_Tip_1_Energy = 0;
//			Tip_1_Energy_Consumed = 0;
//		}
//		 if (HMI_MANUAL_TIP_2)
//		{
//			FLAG_TIP2_HEATING_START=1;
//		}
//			else
//			{
//				FLAG_TIP2_HEATING_START=0;
//				HOME_Tip_2_Energy = 0;
//				Tip_2_Energy_Consumed = 0;
//			}
//		 if (HMI_MANUAL_TIP_3)
//		{
//			FLAG_TIP3_HEATING_START=1;
//		}
//		 else
//		 {
//				FLAG_TIP3_HEATING_START=0;
//				HOME_Tip_3_Energy = 0;
//				Tip_3_Energy_Consumed = 0;
//		 }
//	}

    control_data[0] =  HMI_MANUAL_TIP_1 || TIP_1;
    control_data[1] =  HMI_MANUAL_TIP_2 || TIP_2;
    control_data[2] =  HMI_MANUAL_TIP_3 || TIP_3;
    control_data[3] =  HMI_MANUAL_TIP_4 || TIP_4;
    control_data[4] = Cooling_Valve || FLAG_MANUAL_COOLING_PRESSED;
    control_data[5] = 0;
    control_data[6] = 0;
    control_data[7] = 0;
}


char keypad_text[4];
char SETTINGS_PASSWORD[4] = "1379";

void keypad()
{
	// Password Check
	if (keypad_index >= 4)
	{
		FLAG_SCREEN_LOCKED = strcmp(keypad_text,SETTINGS_PASSWORD);

		// Clear Password
		keypad_index=0;
		memset(keypad_text, '\0', 4);

	}
	if (HMI_MODAL_LOCK_NUM_1)
	{
		keypad_text[keypad_index++] = '1';
		osDelay(100);
	}
	else if (HMI_MODAL_LOCK_NUM_2)
	{
		keypad_text[keypad_index++] = '2';
		osDelay(100);
	}
	else if (HMI_MODAL_LOCK_NUM_3)
	{
		keypad_text[keypad_index++] = '3';
		osDelay(100);
	}
	else if (HMI_MODAL_LOCK_NUM_4)
	{
		keypad_text[keypad_index++] = '4';
		osDelay(100);
	}
	else if (HMI_MODAL_LOCK_NUM_5)
	{
		keypad_text[keypad_index++] = '5';
		osDelay(100);
	}
	else if (HMI_MODAL_LOCK_NUM_6)
	{
		keypad_text[keypad_index++] = '6';
		osDelay(100);
	}
	else if (HMI_MODAL_LOCK_NUM_7)
	{
		keypad_text[keypad_index++] = '7';
		osDelay(100);
	}
	else if (HMI_MODAL_LOCK_NUM_8)
	{
		keypad_text[keypad_index++] = '8';
		osDelay(100);
	}
	else if (HMI_MODAL_LOCK_NUM_9)
	{
		keypad_text[keypad_index++] = '9';
		osDelay(100);
	}


}

uint16_t Brass_Energy_Val =0;
void update_counters()
{

    if (STOP_COUNTERS == 0)
    {

    	// Brass Mode. All tips change equally
    	if (!Brass_Mode_Enabled)
    	{
    		if (Tip_1_Energy_Up_Pressed ||
    			Tip_2_Energy_Up_Pressed ||
    			Tip_3_Energy_Up_Pressed ||
    			Tip_4_Energy_Up_Pressed )
    		{
    			Tip_1_Energy_Val+=ENERGY_INCREMENTS;
    		}

    		if (Tip_1_Energy_Down_Pressed ||
    			Tip_2_Energy_Down_Pressed ||
    			Tip_3_Energy_Down_Pressed ||
    			Tip_4_Energy_Down_Pressed)
    		{
    			Tip_1_Energy_Val-=ENERGY_INCREMENTS;
    		}

    		Tip_2_Energy_Val = Tip_1_Energy_Val;
    		Tip_3_Energy_Val = Tip_1_Energy_Val;
    		Tip_4_Energy_Val = Tip_1_Energy_Val;

    	}

        // Tip 1 Energy
        if (Tip_1_Energy_Up_Pressed)
        {
            if (Tip_1_Energy_Val < MAX_ENERGY)
            {
                Tip_1_Energy_Val+=ENERGY_INCREMENTS;
            }
        }
        if (Tip_1_Energy_Down_Pressed)
        {
            if (Tip_1_Energy_Val > 0)
            {
                Tip_1_Energy_Val-=ENERGY_INCREMENTS;
            }
        }

        // Tip 1 Distance
        if (Tip_1_Distance_Up_Pressed)
        {
            if (Tip_1_Distance_Val < 8.0)
            {
                Tip_1_Distance_Val += 0.1;
            }
        }
        if (Tip_1_Distance_Down_Pressed)
        {
            if (Tip_1_Distance_Val > 0.1)
            {
                Tip_1_Distance_Val -= 0.1;
            }
        }

        // Tip 1 Offset
        if (HMI_Tip_1_Offset_Up)
        {
            if (HMI_Tip_1_Offset < MAX_TIP_DISTANCE_OFFSET)
            {
            	HMI_Tip_1_Offset += 0.01;
            }
        }
        if (HMI_Tip_1_Offset_Down)
        {
            if (HMI_Tip_1_Offset > (MAX_TIP_DISTANCE_OFFSET*(-1)) )
            {
            	HMI_Tip_1_Offset -= 0.01;
            }
        }
    }

    // Tip2
    //-----------------------------------------------------

    // Tip 2 Energy
    if (Tip_2_Energy_Up_Pressed)
    {
        if (Tip_2_Energy_Val < MAX_ENERGY)
        {
            Tip_2_Energy_Val+=ENERGY_INCREMENTS;
        }
    }
    if (Tip_2_Energy_Down_Pressed)
    {
        if (Tip_2_Energy_Val > 0)
        {
            Tip_2_Energy_Val-=ENERGY_INCREMENTS;
        }
    }

    // Tip 2 Distance
    if (Tip_2_Distance_Up_Pressed)
    {
        if (Tip_2_Distance_Val < 8.0)
        {
            Tip_2_Distance_Val += 0.1;
        }
    }
    if (Tip_2_Distance_Down_Pressed)
    {
        if (Tip_2_Distance_Val > 0.1)
        {
            Tip_2_Distance_Val -= 0.1;
        }
    }

    // Tip 2 Offset
    if (HMI_Tip_2_Offset_Up)
    {
        if (HMI_Tip_2_Offset < MAX_TIP_DISTANCE_OFFSET)
        {
            HMI_Tip_2_Offset += 0.01;
        }
    }
    if (HMI_Tip_2_Offset_Down)
    {
        if (HMI_Tip_2_Offset > (MAX_TIP_DISTANCE_OFFSET*(-1)))
        {
            HMI_Tip_2_Offset -= 0.01;
        }
    }

    // Tip3
    //-----------------------------------------------------

    // Tip 3 Energy
    if (Tip_3_Energy_Up_Pressed)
    {
        if (Tip_3_Energy_Val < MAX_ENERGY)
        {
            Tip_3_Energy_Val+=ENERGY_INCREMENTS;
        }
    }
    if (Tip_3_Energy_Down_Pressed)
    {
        if (Tip_3_Energy_Val > 0)
        {
            Tip_3_Energy_Val-=ENERGY_INCREMENTS;
        }
    }

    // Tip 3 Distance
    if (Tip_3_Distance_Up_Pressed)
    {
        if (Tip_3_Distance_Val < 8.0)
        {
            Tip_3_Distance_Val += 0.1;
        }
    }
    if (Tip_3_Distance_Down_Pressed)
    {
        if (Tip_3_Distance_Val > 0.1)
        {
            Tip_3_Distance_Val -= 0.1;
        }
    }

    // Tip 3 Offset
    if (HMI_Tip_3_Offset_Up)
    {
        if (HMI_Tip_3_Offset < MAX_TIP_DISTANCE_OFFSET)
        {
            HMI_Tip_3_Offset += 0.01;
        }
    }
    if (HMI_Tip_3_Offset_Down)
    {
        if (HMI_Tip_3_Offset > (MAX_TIP_DISTANCE_OFFSET*(-1)))
        {
            HMI_Tip_3_Offset -= 0.01;
        }
    }

    // Tip4
       //-----------------------------------------------------

       // Tip 4 Energy
       if (Tip_4_Energy_Up_Pressed)
       {
           if (Tip_4_Energy_Val < MAX_ENERGY)
           {
               Tip_4_Energy_Val+=ENERGY_INCREMENTS;
           }
       }
       if (Tip_4_Energy_Down_Pressed)
       {
           if (Tip_4_Energy_Val > 0)
           {
               Tip_4_Energy_Val-=ENERGY_INCREMENTS;
           }
       }

       // Tip 4 Distance
       if (Tip_4_Distance_Up_Pressed)
       {
           if (Tip_4_Distance_Val < 8.0)
           {
               Tip_4_Distance_Val += 0.1;
           }
       }
       if (Tip_4_Distance_Down_Pressed)
       {
           if (Tip_4_Distance_Val > 0.1)
           {
               Tip_4_Distance_Val -= 0.1;
           }
       }

       // Tip 4 Offset
       if (HMI_Tip_4_Offset_Up)
       {
           if (HMI_Tip_4_Offset < MAX_TIP_DISTANCE_OFFSET)
           {
               HMI_Tip_4_Offset += 0.01;
           }
       }
       if (HMI_Tip_4_Offset_Down)
       {
           if (HMI_Tip_4_Offset > (MAX_TIP_DISTANCE_OFFSET*(-1)))
           {
               HMI_Tip_4_Offset -= 0.01;
           }
       }
}

void abort_cycle()
{
	  // Flag for clearing Cycle initials
	  CYCLE_START_CLEARS=1;

	  // Turn Off Pre-Cycle
	  FLAG_IN_PRE_CYCLE=0;

	  // Set Cycle Abortion Flag
	  FLAG_CYCLE_ABORTED =1;
}

void abort_heating()
{
	  // Turn off Heating
	  FLAG_TIP1_HEATING_START=0;
	  FLAG_TIP2_HEATING_START=0;
	  FLAG_TIP3_HEATING_START=0;
	  FLAG_TIP4_HEATING_START=0;
}

HAL_StatusTypeDef i2c_transmit(uint8_t _address, uint8_t _data[], uint8_t _data_length)
{
	HAL_StatusTypeDef _status=0;
    uint8_t retry_count_tx = 0;
    uint8_t max_retries = 3;

    // Keep trying to transmit data until successful or max retries reached
    while ( (_status = HAL_I2C_Master_Transmit(&hi2c1, _address << 1 , _data, _data_length, 1000)) != HAL_OK) {
        // Transmission error
        I2C_Error_Handler();
        retry_count_tx++;
        // Check if maximum retries reached
        if (retry_count_tx >= max_retries) {
            // Give up and break the loop
        	_status = HAL_ERROR;
            break;
        }
    }
    	return _status;

}


HAL_StatusTypeDef i2c_receive(uint8_t _address, uint8_t _data[], uint8_t _data_length)
{
	HAL_StatusTypeDef _status=0;
    uint8_t retry_count_rx = 0;
    uint8_t max_retries = 3;

    // Keep trying to receive data until successful or max retries reached
    while ( (_status = HAL_I2C_Master_Receive(&hi2c1, _address << 1 , _data, _data_length, 1000)) != HAL_OK) {
        // Reception error
        I2C_Error_Handler();
        retry_count_rx++;
        // Check if maximum retries reached
        if (retry_count_rx >= max_retries) {
            // Give up and break the loop
        	_status = HAL_ERROR;
            break;
        }
    }
    	return _status;
}

void ssr_transmit(SSR* ssrs, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        HAL_StatusTypeDef _status=0;
        if (ssrs[i].Enable == ENABLED)
        {
            _status = i2c_transmit(ssrs[i].Address,ssrs[i].Transmit,ssrs[i].T_Length);

            if (_status != HAL_OK )
            {
                FAULT_SSR_1_FLAG=1;
            }
            else
            {
                FAULT_SSR_1_FLAG=0;
            }
        }
    }
}

void ssr_receive(SSR* ssrs, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        HAL_StatusTypeDef _status=0;
        if (ssrs[i].Enable == ENABLED)
        {
            _status = i2c_receive(ssrs[i].Address,ssrs[i].Recieve,ssrs[i].R_Length);

            if (_status != HAL_OK )
            {
                FAULT_SSR_1_FLAG=1;
            }
            else
            {
                FAULT_SSR_1_FLAG=0;
            }
        }
    }
}

void ssr_sync(SSR* ssrs, size_t count)
{
	if (FLAG_IN_CYCLE==0)
	{
	uint32_t current_time = HAL_GetTick();
	if (control_data[0])
	{
		// Check if timeout Exceeded
		if (current_time - Tip_1_On_Start_Time > TIP_HEATING_TIMEOUT)
		{
			control_data[0]=0;
		}
	}
	else
	{
		Tip_1_On_Start_Time = current_time;
	}

	if (control_data[1])
	{
		// Check if timeout Exceeded
		if (current_time - Tip_2_On_Start_Time > TIP_HEATING_TIMEOUT)
		{
			control_data[1]=0;
		}
	}
	else
	{
		Tip_2_On_Start_Time = current_time;
	}


	if (control_data[2])
	{
		// Check if timeout Exceeded
		if (current_time - Tip_3_On_Start_Time > TIP_HEATING_TIMEOUT)
		{
			control_data[2]=0;
		}
	}
	else
	{
		Tip_3_On_Start_Time = current_time;
	}

	if (control_data[3])
	{
		// Check if timeout Exceeded
		if (current_time - Tip_4_On_Start_Time > TIP_HEATING_TIMEOUT)
		{
			control_data[3]=0;
		}
	}
	else
	{
		Tip_4_On_Start_Time = current_time;
	}
	}
	// Assign Data to Device IOs
    ssrs[0].Tips[0] = control_data[0]; // Tip 1 Heating
    ssrs[0].Tips[1] = control_data[1]; // Tip 2 Heating
    ssrs[0].Tips[2] = control_data[2]; // Tip 3 Heating
    ssrs[0].Tips[3] = control_data[3]; // Tip 4 Heating
    ssrs[0].Output[0]= 0;
    ssrs[0].Output[1]= 0;
    ssrs[0].Output[2]= control_data[4]; // Cooling
    ssrs[0].Output[3]= 0;
    ssrs[0].Output[4]= 0;
    ssrs[0].Output[5]= 0;
    ssrs[0].Button_Color = control_data[8]; // Button Color


    // Packet Formatting
    ssrs[0].Transmit[0] = ssrs[0].Tips[0]; // Tip 1 Heating
    ssrs[0].Transmit[1] = ssrs[0].Tips[1]; // Tip 2 Heating
    ssrs[0].Transmit[2] = ssrs[0].Tips[2]; // Tip 3 Heating
    ssrs[0].Transmit[3] = ssrs[0].Tips[3]; // Tip 4 Heating
    ssrs[0].Transmit[4] = ssrs[0].Output[2]; // Cooling
    ssrs[0].Transmit[5] = 0; // Unassigned
    ssrs[0].Transmit[6] = 0; // Unassigned
    ssrs[0].Transmit[7] = 0; // Unassigned
    ssrs[0].Transmit[8] = ssrs[0].Button_Color; // Unassigned

    // Transmit to SSRs
    ssr_transmit(ssrs, TOTAL_SSR);

    // Receive From SSRs
    ssr_receive(ssrs, TOTAL_SSR);

    // Parse Data Received

    // Inputs
    ssrs[0].Input[0] = ssrs[0].Recieve[0]; // Right Switch
    ssrs[0].Input[1] = ssrs[0].Recieve[1]; // Left Switch
    ssrs[0].Input[2] = ssrs[0].Recieve[2]; // Emergency Button
    ssrs[0].Input[3] = 0;


    // Convert current1 values back to uint16_t
    current1_received = (ssrs[0].Recieve[3] << 8) | ssrs[0].Recieve[4];
    if (current1_received < 30) {
        current1_received = 0;
    }

    // Convert current2 values back to uint16_t
    current2_received = (ssrs[0].Recieve[5] << 8) | ssrs[0].Recieve[6];
    if (current2_received < 30) {
        current2_received = 0;
    }

    // Convert current3 values back to uint16_t
    current3_received = (ssrs[0].Recieve[7] << 8) | ssrs[0].Recieve[8];
    if (current3_received < 30) {
        current3_received = 0;
    }

    // Convert current4 values back to uint16_t
    current4_received = (ssrs[0].Recieve[9] << 8) | ssrs[0].Recieve[10];
    if (current4_received < 30) {
        current4_received = 0;
    }

    //convert rms to peak
	current1_peak = current1_received * 1.414;
	current2_peak = current2_received * 1.414;
	current3_peak = current3_received * 1.414;
	current4_peak = current4_received * 1.414;

    // convert current values to amps. 180mV/A
	current1_amps = current1_peak /180;
	current2_amps = current2_peak /180;
	current3_amps = current3_peak /180;
	current4_amps = current4_peak /180;

    ssrs[0].Current[0] = current1_amps;
    ssrs[0].Current[1] = current2_amps;
    ssrs[0].Current[2] = current3_amps;
    ssrs[0].Current[3] = current4_amps;



}

void encoder_transmit(ENCODER* encoders, size_t count)
{

    for (size_t i = 0; i < count; i++) {
    	encoders[i].Fail=0;
        HAL_StatusTypeDef _status=0;
        // Handle Reset. Reset bit should be cleared after reset is sent
        if (encoders[i].Enable == ENABLED && encoders[i].Reset ==1)
        {
        	// Copy Reset Byte to Transmit
        	//encoders[i].Transmit = encoders[i].Reset;
        	encoders[i].Transmit = RESET_ENCODER;
            _status = i2c_transmit(encoders[i].Address,&encoders[i].Transmit,1);

            if (_status != HAL_OK )
            {
            	encoders[i].Fail =1;
            }
            else
            {
            	encoders[i].Fail =0;
            	// Clear Reset
            	encoders[i].Reset=0;
            }
        }
        // To handle color setting
        else if (encoders[i].Enable == ENABLED)
		{
        	// Copy Color Byte to Transmit
        	encoders[i].Transmit = encoders[i].Color;
            _status = i2c_transmit(encoders[i].Address,&encoders[i].Transmit,1);

            if (_status != HAL_OK )
            {
            	encoders[i].Fail =1;
            }
            else
            {
            	encoders[i].Fail =0;
            }
		}
        else
        {
        	encoders[i].Fail=0;
        }
    }

}

void encoder_receive(ENCODER* encoders, size_t count)
{
    for (size_t i = 0; i < count; i++) {

        HAL_StatusTypeDef _status=0;
        if (encoders[i].Enable == ENABLED)
        {
            _status = i2c_receive(encoders[i].Address,encoders[i].Recieve,encoders[i].R_Length);

            if (_status != HAL_OK )
            {
            	encoders[i].Fail=1;
            	encoders[i].Distance=0;
            }
            else
            {
            	encoders[i].Fail=0;

            	// Parse Received Data
            	int16_t encoder_distance = (encoders[i].Recieve[0] << 8) | encoders[i].Recieve[1];
            	encoders[i].Distance = (double)encoder_distance/100;
//    			if (fabs(encoders[i].Distance) > 0.1)
//    			{
//    			   // Apply the polynomial correction
//    				// Polynomial coefficients, pre-computed
//    				double a = -0.010788690801793961;
//    				double b = 1.181005931716059;
//    				double c = -0.09131103576323933;
//    				double corrected_distance = a*encoders[i].Distance*encoders[i].Distance + b*encoders[i].Distance + c;
//    	    	    // Update the distance with corrected value
//    				encoders[i].Distance = corrected_distance;
//    			}

            }
        }
        else
        {
        	encoders[i].Fail=0;
        }
    }
}


void encoder_sync(ENCODER* encoders, size_t count)
{
	encoder_transmit(encoders,TOTAL_ENCODERS);
	encoder_receive(encoders,TOTAL_ENCODERS);

}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  stepper_setSpeed(STEPPER_SPEED_MID);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
 // TIM10_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DSIHOST_DSI_Init();
  MX_LTDC_Init();
  MX_FMC_Init();
  MX_QUADSPI_Init();
  MX_DMA2D_Init();
  MX_I2C4_Init();
  MX_LIBJPEG_Init();
  MX_CRC_Init();
  MX_JPEG_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM10_Init();
  MX_TouchGFX_Init();
  /* Call PreOsInit function */
  MX_TouchGFX_PreOSInit();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
 // __HAL_TIM_ENABLE_IT(&htim12, TIM_IT_UPDATE);
  // Init Timer
  init_timer();
  //Go_Home();
  FLAG_REQUIRE_HOME=1;
  HAL_TIM_Base_Start(&htim12);

  reset_linear_encoders();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of controlDataMutex */
  controlDataMutexHandle = osMutexNew(&controlDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TouchGFXTask */
  TouchGFXTaskHandle = osThreadNew(TouchGFX_Task, NULL, &TouchGFXTask_attributes);

  /* creation of videoTask */
  videoTaskHandle = osThreadNew(videoTaskFunc, NULL, &videoTask_attributes);

  /* creation of i2cTask */
  i2cTaskHandle = osThreadNew(i2cTaskFunc, NULL, &i2cTask_attributes);

  /* creation of Serial */
  SerialHandle = osThreadNew(print, NULL, &Serial_attributes);

  /* creation of Led */
  LedHandle = osThreadNew(LedBlink, NULL, &Led_attributes);

  /* creation of autoCycle */
  autoCycleHandle = osThreadNew(autoCycleTask, NULL, &autoCycle_attributes);

  /* creation of GoHome */
  GoHomeHandle = osThreadNew(GoHomeTask, NULL, &GoHome_attributes);

  /* creation of Timer10Overflow */
  Timer10OverflowHandle = osThreadNew(Timer10OverflowTask, NULL, &Timer10Overflow_attributes);

  /* creation of Autofindtask */
  AutofindtaskHandle = osThreadNew(T_Autofind, NULL, &Autofindtask_attributes);

  /* creation of Buzzer_Task */
  Buzzer_TaskHandle = osThreadNew(T_Buzzer, NULL, &Buzzer_Task_attributes);

  /* creation of CoolingTask */
  CoolingTaskHandle = osThreadNew(Cooling, NULL, &CoolingTask_attributes);

  /* creation of repeatAutoCycle */
  repeatAutoCycleHandle = osThreadNew(repeatAutoCycleTask, NULL, &repeatAutoCycle_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief DSIHOST Initialization Function
  * @param None
  * @retval None
  */
static void MX_DSIHOST_DSI_Init(void)
{

  /* USER CODE BEGIN DSIHOST_Init 0 */
  /* Activate XRES active low */
  HAL_GPIO_WritePin(DSI_RESET_GPIO_Port, DSI_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(20); /* wait 20 ms */
  /* Desactivate XRES */
  HAL_GPIO_WritePin(DSI_RESET_GPIO_Port, DSI_RESET_Pin, GPIO_PIN_SET);
  /* Wait for 10ms after releasing XRES before sending commands */
  HAL_Delay(10);
  /* USER CODE END DSIHOST_Init 0 */

  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_LPCmdTypeDef LPCmd = {0};
  DSI_CmdCfgTypeDef CmdCfg = {0};

  /* USER CODE BEGIN DSIHOST_Init 1 */

  /* USER CODE END DSIHOST_Init 1 */
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 4;
  hdsi.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
  PLLInit.PLLNDIV = 100;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV5;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV1;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 28;
  PhyTimings.ClockLaneLP2HSTime = 33;
  PhyTimings.DataLaneHS2LPTime = 15;
  PhyTimings.DataLaneLP2HSTime = 25;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 0;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_OVF) != HAL_OK)
  {
    Error_Handler();
  }
  LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_ENABLE;
  LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_ENABLE;
  LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_ENABLE;
  LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_ENABLE;
  LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_ENABLE;
  LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_ENABLE;
  LPCmd.LPGenLongWrite = DSI_LP_GLW_ENABLE;
  LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_ENABLE;
  LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_ENABLE;
  LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_ENABLE;
  LPCmd.LPDcsLongWrite = DSI_LP_DLW_ENABLE;
  LPCmd.LPMaxReadPacket = DSI_LP_MRDP_ENABLE;
  LPCmd.AcknowledgeRequest = DSI_ACKNOWLEDGE_ENABLE;
  if (HAL_DSI_ConfigCommand(&hdsi, &LPCmd) != HAL_OK)
  {
    Error_Handler();
  }
  CmdCfg.VirtualChannelID = 0;
  CmdCfg.ColorCoding = DSI_RGB565;
  CmdCfg.CommandSize = 200;
  CmdCfg.TearingEffectSource = DSI_TE_EXTERNAL;
  CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
  CmdCfg.HSPolarity = DSI_HSYNC_ACTIVE_LOW;
  CmdCfg.VSPolarity = DSI_VSYNC_ACTIVE_LOW;
  CmdCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  CmdCfg.VSyncPol = DSI_VSYNC_FALLING;
  CmdCfg.AutomaticRefresh = DSI_AR_DISABLE;
  CmdCfg.TEAcknowledgeRequest = DSI_TE_ACKNOWLEDGE_ENABLE;
  if (HAL_DSI_ConfigAdaptedCommandMode(&hdsi, &CmdCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DSIHOST_Init 2 */

  /* USER CODE END DSIHOST_Init 2 */

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
  hi2c1.Init.Timing = 0x00401959;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00C0EAFF;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief JPEG Initialization Function
  * @param None
  * @retval None
  */
static void MX_JPEG_Init(void)
{

  /* USER CODE BEGIN JPEG_Init 0 */

  /* USER CODE END JPEG_Init 0 */

  /* USER CODE BEGIN JPEG_Init 1 */

  /* USER CODE END JPEG_Init 1 */
  hjpeg.Instance = JPEG;
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN JPEG_Init 2 */

  /* USER CODE END JPEG_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 1;
  hltdc.Init.VerticalSync = 1;
  hltdc.Init.AccumulatedHBP = 2;
  hltdc.Init.AccumulatedVBP = 2;
  hltdc.Init.AccumulatedActiveW = 202;
  hltdc.Init.AccumulatedActiveH = 482;
  hltdc.Init.TotalWidth = 203;
  hltdc.Init.TotalHeigh = 483;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 200;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 480;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0xC0000000;
  pLayerCfg.ImageWidth = 200;
  pLayerCfg.ImageHeight = 480;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */
  __HAL_LTDC_DISABLE(&hltdc);
  DSI_LPCmdTypeDef LPCmd;

  HAL_DSI_Start(&hdsi);
  OTM8009A_Init(OTM8009A_FORMAT_RBG565, LCD_ORIENTATION_LANDSCAPE);

  HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_DISPOFF, 0x00);

  LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_DISABLE;
  LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_DISABLE;
  LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_DISABLE;
  LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_DISABLE;
  LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_DISABLE;
  LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_DISABLE;
  LPCmd.LPGenLongWrite = DSI_LP_GLW_DISABLE;
  LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_DISABLE;
  LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_DISABLE;
  LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_DISABLE;
  LPCmd.LPDcsLongWrite = DSI_LP_DLW_DISABLE;
  HAL_DSI_ConfigCommand(&hdsi, &LPCmd);

  HAL_LTDC_SetPitch(&hltdc, 800, 0);
  __HAL_LTDC_ENABLE(&hltdc);

  // LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_DISABLE;
  // LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_DISABLE;
  // LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_DISABLE;
  // LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_DISABLE;
  // LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_DISABLE;
  // LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_DISABLE;
  // LPCmd.LPGenLongWrite = DSI_LP_GLW_DISABLE;
  // LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_DISABLE;
  // LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_DISABLE;
  // LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_DISABLE;
  // LPCmd.LPDcsLongWrite = DSI_LP_DLW_DISABLE;
  // HAL_DSI_ConfigCommand(&hdsi, &LPCmd);

  // HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_DISPOFF, 0x00);

  // HAL_LTDC_SetPitch(&hltdc, 800, 0);
  // __HAL_LTDC_ENABLE(&hltdc);

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 1;
  hqspi.Init.FifoThreshold = 16;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 25;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_4_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */
  /* QSPI memory reset */
  if (QSPI_ResetMemory(&hqspi) != QSPI_OK)
  {
    Error_Handler();
  }

  /* Put QSPI memory in QPI mode */
  if( QSPI_EnterMemory_QPI( &hqspi )!=QSPI_OK )
  {
    Error_Handler();
  }

  /* Set the QSPI memory in 4-bytes address mode */
  if (QSPI_EnterFourBytesAddress(&hqspi) != QSPI_OK)
  {
    Error_Handler();
  }

  /* Configuration of the dummy cycles on QSPI memory side */
  if (QSPI_DummyCyclesCfg(&hqspi) != QSPI_OK)
  {
    Error_Handler();
  }

  /* Configuration of the Output driver strength on memory side */
  if( QSPI_OutDrvStrengthCfg( &hqspi ) != QSPI_OK )
  {
    Error_Handler();
  }

  if( BSP_QSPI_EnableMemoryMappedMode(&hqspi) != QSPI_OK )
  {
    Error_Handler();
  }

  /* USER CODE END QUADSPI_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 399;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 100;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 30;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 1;
  if (HAL_TIM_SlaveConfigSynchro(&htim12, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_32;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 2;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 3;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  FMC_SDRAM_CommandTypeDef command;

  /* Program the SDRAM external device */
  BSP_SDRAM_Initialization_Sequence(&hsdram1, &command);

  //Deactivate speculative/cache access to first FMC Bank to save FMC bandwidth
  FMC_Bank1->BTCR[0] = 0x000030D2;
  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SW_UP_GPIO_Port, SW_UP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, DSI_RESET_Pin|BUZZER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Step_Dir_Pin|RENDER_TIME_Pin|VSYNC_FREQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ARD_OUT_GPIO_Port, ARD_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SW_UP_Pin */
  GPIO_InitStruct.Pin = SW_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SW_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DSI_RESET_Pin */
  GPIO_InitStruct.Pin = DSI_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(DSI_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Step_Dir_Pin */
  GPIO_InitStruct.Pin = Step_Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Step_Dir_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RENDER_TIME_Pin VSYNC_FREQ_Pin */
  GPIO_InitStruct.Pin = RENDER_TIME_Pin|VSYNC_FREQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_OUT_Pin */
  GPIO_InitStruct.Pin = ARD_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ARD_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Perform the SDRAM external memory initialization sequence
  * @param  hsdram: SDRAM handle
  * @param  Command: Pointer to SDRAM command structure
  * @retval None
  */
static void BSP_SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command)
{
 __IO uint32_t tmpmrd = 0;

    /* Step 1: Configure a clock configuration enable command */
    Command->CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
    Command->CommandTarget          =  FMC_SDRAM_CMD_TARGET_BANK1;
    Command->AutoRefreshNumber      = 1;
    Command->ModeRegisterDefinition = 0;

    /* Send the command */
    HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

    /* Step 2: Insert 100 us minimum delay */
    /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
    HAL_Delay(1);

    /* Step 3: Configure a PALL (precharge all) command */
    Command->CommandMode            = FMC_SDRAM_CMD_PALL;
    Command->CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
    Command->AutoRefreshNumber      = 1;
    Command->ModeRegisterDefinition = 0;

    /* Send the command */
    HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

    /* Step 4: Configure an Auto Refresh command */
    Command->CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
    Command->CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
    Command->AutoRefreshNumber      = 8;
    Command->ModeRegisterDefinition = 0;

    /* Send the command */
    HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

    /* Step 5: Program the external memory mode register */
    tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          | \
             SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   | \
             SDRAM_MODEREG_CAS_LATENCY_3           | \
             SDRAM_MODEREG_OPERATING_MODE_STANDARD | \
             SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

    Command->CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
    Command->CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
    Command->AutoRefreshNumber      = 1;
    Command->ModeRegisterDefinition = tmpmrd;

    /* Send the command */
    HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

    /* Step 6: Set the refresh rate counter */
    /* Set the device refresh rate */
    HAL_SDRAM_ProgramRefreshRate(hsdram, REFRESH_COUNT);

}

/**
  * @brief  This function reset the QSPI memory.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static uint8_t QSPI_ResetMemory(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef      s_command;
  QSPI_AutoPollingTypeDef  s_config;
  uint8_t                  reg;

  /* Send command RESET command in QPI mode (QUAD I/Os) */
  /* Initialize the reset enable command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = RESET_ENABLE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  /* Send the command */
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  /* Send the reset memory command */
  s_command.Instruction = RESET_MEMORY_CMD;
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Send command RESET command in SPI mode */
  /* Initialize the reset enable command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = RESET_ENABLE_CMD;
  /* Send the command */
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  /* Send the reset memory command */
  s_command.Instruction = RESET_MEMORY_CMD;
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* After reset CMD, 1000ms requested if QSPI memory SWReset occured during full chip erase operation */
  HAL_Delay( 1000 );

  /* Configure automatic polling mode to wait the WIP bit=0 */
  s_config.Match           = 0;
  s_config.Mask            = MX25L512_SR_WIP;
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction     = READ_STATUS_REG_CMD;
  s_command.DataMode        = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Initialize the reading of status register */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = READ_STATUS_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Enable write operations, command in 1 bit */
  /* Enable write operations */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = WRITE_ENABLE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for write enabling */
  s_config.Match           = MX25L512_SR_WREN;
  s_config.Mask            = MX25L512_SR_WREN;
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  s_command.Instruction    = READ_STATUS_REG_CMD;
  s_command.DataMode       = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Update the configuration register with new dummy cycles */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = WRITE_STATUS_CFG_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Enable the Quad IO on the QSPI memory (Non-volatile bit) */
  reg |= MX25L512_SR_QUADEN;

  /* Configure the command */
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* 40ms  Write Status/Configuration Register Cycle Time */
  HAL_Delay( 40 );

  return QSPI_OK;
}

/**
  * @brief  This function put QSPI memory in QPI mode (quad I/O).
  * @param  hqspi: QSPI handle
  * @retval None
  */
static uint8_t QSPI_EnterMemory_QPI( QSPI_HandleTypeDef *hqspi )
{
  QSPI_CommandTypeDef      s_command;
  QSPI_AutoPollingTypeDef  s_config;

  /* Initialize the QPI enable command */
  /* QSPI memory is supported to be in SPI mode, so CMD on 1 LINE */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = ENTER_QUAD_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait the QUADEN bit=1 and WIP bit=0 */
  s_config.Match           = MX25L512_SR_QUADEN;
  s_config.Mask            = MX25L512_SR_QUADEN|MX25L512_SR_WIP;
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = READ_STATUS_REG_CMD;
  s_command.DataMode          = QSPI_DATA_4_LINES;

  if (HAL_QSPI_AutoPolling(hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  This function set the QSPI memory in 4-byte address mode
  * @param  hqspi: QSPI handle
  * @retval None
  */
static uint8_t QSPI_EnterFourBytesAddress(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = ENTER_4_BYTE_ADDR_MODE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations */
  if (QSPI_WriteEnable(hqspi) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Send the command */
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait the memory is ready */
  if (QSPI_AutoPollingMemReady(hqspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  This function configure the dummy cycles on memory side.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static uint8_t QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef s_command;
  uint8_t reg[2];

  /* Initialize the reading of status register */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = READ_STATUS_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(hqspi, &(reg[0]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Initialize the reading of configuration register */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = READ_CFG_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(hqspi, &(reg[1]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Enable write operations */
  if (QSPI_WriteEnable(hqspi) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Update the configuration register with new dummy cycles */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = WRITE_STATUS_CFG_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 2;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* MX25L512_DUMMY_CYCLES_READ_QUAD = 3 for 10 cycles in QPI mode */
  MODIFY_REG( reg[1], MX25L512_CR_NB_DUMMY, (MX25L512_DUMMY_CYCLES_READ_QUAD << POSITION_VAL(MX25L512_CR_NB_DUMMY)));

  /* Configure the write volatile configuration register command */
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(hqspi, &(reg[0]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* 40ms  Write Status/Configuration Register Cycle Time */
  HAL_Delay( 40 );

  return QSPI_OK;
}

/**
  * @brief  This function configure the Output driver strength on memory side.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static uint8_t QSPI_OutDrvStrengthCfg( QSPI_HandleTypeDef *hqspi )
{
  QSPI_CommandTypeDef s_command;
  uint8_t reg[2];

  /* Initialize the reading of status register */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = READ_STATUS_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(hqspi, &(reg[0]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Initialize the reading of configuration register */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = READ_CFG_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(hqspi, &(reg[1]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Enable write operations */
  if (QSPI_WriteEnable(hqspi) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Update the configuration register with new output driver strength */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = WRITE_STATUS_CFG_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 2;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Set Output Strength of the QSPI memory 15 ohms */
  MODIFY_REG( reg[1], MX25L512_CR_ODS, (MX25L512_CR_ODS_15 << POSITION_VAL(MX25L512_CR_ODS)));

  /* Configure the write volatile configuration register command */
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(hqspi, &(reg[0]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static uint8_t QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef     s_command;
  QSPI_AutoPollingTypeDef s_config;

  /* Enable write operations */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = WRITE_ENABLE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for write enabling */
  s_config.Match           = MX25L512_SR_WREN;
  s_config.Mask            = MX25L512_SR_WREN;
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  s_command.Instruction    = READ_STATUS_REG_CMD;
  s_command.DataMode       = QSPI_DATA_4_LINES;

  if (HAL_QSPI_AutoPolling(hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  This function read the SR of the memory and wait the EOP.
  * @param  hqspi: QSPI handle
  * @param  Timeout
  * @retval None
  */
static uint8_t QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi, uint32_t Timeout)
{
  QSPI_CommandTypeDef     s_command;
  QSPI_AutoPollingTypeDef s_config;

  /* Configure automatic polling mode to wait for memory ready */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = READ_STATUS_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  s_config.Match           = 0;
  s_config.Mask            = MX25L512_SR_WIP;
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(hqspi, &s_command, &s_config, Timeout) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  Configure the QSPI in memory-mapped mode
  * @retval QSPI memory status
  */
static uint8_t BSP_QSPI_EnableMemoryMappedMode(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef      s_command;
  QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;

  /* Configure the command for the read instruction */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = QPI_READ_4_BYTE_ADDR_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = MX25L512_DUMMY_CYCLES_READ_QUAD_IO;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
  s_mem_mapped_cfg.TimeOutPeriod     = 0;

  if (HAL_QSPI_MemoryMapped(hqspi, &s_command, &s_mem_mapped_cfg) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

uint8_t left_switch_received = 0;
uint8_t right_switch_received = 0;
uint32_t left_switch_time = 0;
uint32_t right_switch_time = 0;
uint32_t time_difference = 0;
uint32_t last_i2c_request_time = 0;
uint8_t FLAGSTOPPWM=0;

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  for(;;)
  {

	  if (MODAL_OK_BUTTON)
	  {
		  FAULT_ACTIVE_FLAG=0;
		  Go_Home();

	  }

	  else if (Home_Machine)
	  {
		  stepper_setSpeed(PU_HOME);
		  Go_Home();
	  }

	  else if (HMI_SETTINGS_PLATEN_UP || HMI_MANUAL_PLATEN_UP || HMI_TEST_PRESS_PLATEN_UP)
	  {
		  if (HMI_MANUAL_PLATEN_DISTANCE > 0)
		  {
			  stepper_Move_Up();
		  }

	  }

	  /* This check needs cleanup. Tips are being checked irrespective of them being enabled*/
	  else if (HMI_SETTINGS_PLATEN_DOWN || HMI_MANUAL_PLATEN_DOWN || HMI_TEST_PRESS_PLATEN_DOWN )
	  {
			if (HMI_MANUAL_PLATEN_DISTANCE < MAX_DISTANCE)
			{
				FLAG_HOLD_DISTANCE_READING=0;
				if (FLAG_MANUAL_STEPPER_MOVING==0)
				{
					// Clear Encoders
					reset_linear_encoders();
					FLAG_MANUAL_STEPPER_MOVING=1;
				}
				  if (fabs(Tip_1_Distance) > 0.2
					  || fabs(Tip_2_Distance) > 0.2
					  || fabs(Tip_3_Distance) > 0.2
					  || fabs(Tip_4_Distance) > 0.2)
				{
					stepper_setSpeed(STEPPER_SPEED_LOW);
				}
				else
				{
					stepper_setSpeed(STEPPER_SPEED_MID);
				}
				  if (fabs(Tip_1_Distance) < AUTOFIND_STOP_HEIGHT
					  && fabs(Tip_2_Distance) < AUTOFIND_STOP_HEIGHT
					  && fabs(Tip_3_Distance) < AUTOFIND_STOP_HEIGHT
					  && fabs(Tip_4_Distance) < AUTOFIND_STOP_HEIGHT)
				{
					stepper_Move_Down();
				}
				FLAG_REQUIRE_HOME =1;
			}
	  }
	  else if (HMI_SETTINGS_WORK_POSITION)
	  {
	      HMI_TEST_PRESS_DOWN_POSITION = HMI_MANUAL_PLATEN_DISTANCE;

	      // Save current distance as Tip Compressions
	      Tip_1_Compression = fabs(Tip_1_Distance);
	      Tip1_Boss_Start_Height = HMI_TEST_PRESS_DOWN_POSITION - (Tip_1_Compression*100);

	      // Add Tip 2 and Tip 3
	      Tip_2_Compression = fabs(Tip_2_Distance);
	      Tip2_Boss_Start_Height = HMI_TEST_PRESS_DOWN_POSITION - (Tip_2_Compression*100);

	      Tip_3_Compression = fabs(Tip_3_Distance);
	      Tip3_Boss_Start_Height = HMI_TEST_PRESS_DOWN_POSITION - (Tip_3_Compression*100);

	      Tip_4_Compression = fabs(Tip_4_Distance);
	      Tip4_Boss_Start_Height = HMI_TEST_PRESS_DOWN_POSITION - (Tip_4_Compression*100);

	      // Go Home
	      stepper_setSpeed(AF_HIGH);
	      Go_Home();
	  }


	  else if (HMI_MANUAL_COOLING)
	  {
	      // Check if at least 500ms have passed since the last trigger
	      if (HAL_GetTick() - lastTriggerTime >= 500)
	      {
	          FLAG_MANUAL_COOLING_PRESSED = 1;

	          // Update the last trigger time
	          lastTriggerTime = HAL_GetTick();
	      }
	  }


		// If HMI_TEST_PRESS_SAVE_UP is equal to 1, execute code inside if statement
		else if (HMI_TEST_PRESS_SAVE_UP == 1) {
			HMI_TEST_PRESS_UP_POSITION = HMI_MANUAL_PLATEN_DISTANCE;
		}

		// If HMI_TEST_PRESS_SAVE_DOWN is equal to 1, execute code inside if statement
		else if (HMI_TEST_PRESS_SAVE_DOWN == 1) {
			HMI_TEST_PRESS_DOWN_POSITION = HMI_MANUAL_PLATEN_DISTANCE;
		}

		else if (HMI_COOLING_TIME_UP)
		{
			HMI_COOLING_TIME_SET++;
			osDelay(50);
		}
		else if (HMI_COOLING_TIME_DOWN && HMI_COOLING_TIME_SET > 0)
		{
			HMI_COOLING_TIME_SET--;
			osDelay(50);
		}
		else if (HMI_MANUAL_TIP_5)
		{
			//linear_encoder_calibrate();
			FLAG_REPEAT_CYCLE=1;
		}

		else
		{
			FLAG_MANUAL_COOLING_PRESSED=0;
			update_counters();
			keypad();
			osDelay(100);
		}

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_i2cTaskFunc */
/**
* @brief Function implementing the i2cTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_i2cTaskFunc */
void i2cTaskFunc(void *argument)
{
  /* USER CODE BEGIN i2cTaskFunc */
    /* Infinite loop */
    for(;;)
    {
    	HMI_CURRENT_HEIGHT = HMI_MANUAL_PLATEN_DISTANCE;
        assignControlData();

        ssr_sync(ssrs, TOTAL_SSR);

        if (FAULT_SSR_1_FLAG==0)
        {
        	// Extract switch readings
        	        right_switch_received = ssrs[0].Input[0];
        	        left_switch_received = ssrs[0].Input[1];
        	        home_switch_received = ssrs[0].Input[2];
        	        HMI_MONITOR_LEFT_START = left_switch_received;
        	        HMI_MONITOR_RIGHT_START= right_switch_received;
        	        HMI_MONITOR_ESTOP_CLEAR= !home_switch_received;
        	        if (!home_switch_received)
        	        {
        	        	FAULT_ESTOP_FLAG=1;
        	        	FLAG_HALT=1;
        	        	FLAG_REPEAT_CYCLE=0;
        	            estop(1);
        	        }
        	        else
        	        {
        	        	FAULT_ESTOP_FLAG=0;
        	        	FLAG_HALT=0;
        	        	estop(0);
        	        }

        	        if (both_switches_released) {
        	            if (left_switch_received && !left_switch_prev_state) {
        	                left_switch_press_time = HAL_GetTick();
        	                left_switch_prev_state = 1;
        	            } else if (!left_switch_received) {
        	                left_switch_prev_state = 0;
        	            }

        	            if (right_switch_received && !right_switch_prev_state) {
        	                right_switch_press_time = HAL_GetTick();
        	                right_switch_prev_state = 1;
        	            } else if (!right_switch_received) {
        	                right_switch_prev_state = 0;
        	            }

        	            if (left_switch_prev_state && right_switch_prev_state) {
        	                uint32_t time_diff = abs((int32_t)(right_switch_press_time - left_switch_press_time));
        	                if (time_diff <= 500) {
        	                    ANTI_TIE_DOWN_ACTIVE = 1;
        	                    //yourFlag=1;
        	                    both_switches_released = 0;
        	                }
        	            }
        	        } else {
        	            if (!left_switch_received || !right_switch_received) {
        	                left_switch_prev_state = 0;
        	                right_switch_prev_state = 0;
        	                left_switch_press_time = 0;
        	                right_switch_press_time = 0;
        	                ANTI_TIE_DOWN_ACTIVE = 0;
        	                both_switches_released = 1;
        	            }
        	        }
        }

        else
        {
			current1_amps = 0;
			current2_amps = 0;
			current3_amps = 0;
			current4_amps = 0;
			ANTI_TIE_DOWN_ACTIVE=0;
        }


        encoders[0].Enable = !Tip_1_Enabled;
        encoders[1].Enable = !Tip_2_Enabled;
        encoders[2].Enable = !Tip_3_Enabled;
        encoders[3].Enable = !Tip_4_Enabled;

        encoder_sync(encoders,TOTAL_ENCODERS);

        Tip_1_Distance = fabs(encoders[0].Distance);
        Tip_2_Distance = fabs(encoders[1].Distance);
        Tip_3_Distance = fabs(encoders[2].Distance);
        Tip_4_Distance = fabs(encoders[3].Distance);

        FAULT_ENCODER_1_FLAG = encoders[0].Fail;
        FAULT_ENCODER_2_FLAG = encoders[1].Fail;
        FAULT_ENCODER_3_FLAG = encoders[2].Fail;
        FAULT_ENCODER_4_FLAG = encoders[3].Fail;


        if (!Tip_1_Enabled)
        {
	    	if (FLAG_IN_CYCLE)
	    	{

				   //Calculate Tip 1 Collapse While running
				  if (Tip_1_Distance > 0.3)
				  {
					  if (HMI_MANUAL_PLATEN_DISTANCE > Tip1_Boss_Start_Height)
					  {
						  Tip1_Collapse = (HMI_MANUAL_PLATEN_DISTANCE - Tip1_Boss_Start_Height) - (Tip_1_Distance*100);
						  Tip1_Collapse /=100;
						  Tip1_Collapse += HMI_Tip_1_Offset;
					  }

				  }
	    	}
        }

        else
        {
        	Tip1_Collapse=0;
        	Tip_1_Distance=0;
        }

        if (!Tip_2_Enabled)
        {
	    	if (FLAG_IN_CYCLE)
	    	{
			   //Calculate Tip 2 Collapse While running
				if (Tip_2_Distance > 0.3)
				{
					if (HMI_MANUAL_PLATEN_DISTANCE > Tip2_Boss_Start_Height)
					{
						Tip2_Collapse = (HMI_MANUAL_PLATEN_DISTANCE - Tip2_Boss_Start_Height) - (Tip_2_Distance*100);
						Tip2_Collapse /= 100;
						Tip2_Collapse += HMI_Tip_2_Offset;
					}

				}
	    	}
        }

        else
        {
        	Tip2_Collapse=0;
        	Tip_2_Distance=0;
        }

        if (!Tip_3_Enabled)
        {
	    	if (FLAG_IN_CYCLE)
	    	{

		           //Calculate Tip 3 Collapse While running
		            if (Tip_3_Distance > 0.3)
		            {
		                if (HMI_MANUAL_PLATEN_DISTANCE > Tip3_Boss_Start_Height)
		                {
		                    Tip3_Collapse = (HMI_MANUAL_PLATEN_DISTANCE - Tip3_Boss_Start_Height) - (Tip_3_Distance*100);
		                    Tip3_Collapse /= 100;
		                    Tip3_Collapse += HMI_Tip_3_Offset;
		                }
		            }
	    	}
        }

        else
        {
        	Tip3_Collapse=0;
        	Tip_3_Distance=0;

        }

        if (!Tip_4_Enabled)
        {
	    	if (FLAG_IN_CYCLE)
	    	{

		           //Calculate Tip 3 Collapse While running
		            if (Tip_4_Distance > 0.3)
		            {
		                if (HMI_MANUAL_PLATEN_DISTANCE > Tip4_Boss_Start_Height)
		                {
		                    Tip4_Collapse = (HMI_MANUAL_PLATEN_DISTANCE - Tip4_Boss_Start_Height) - (Tip_4_Distance*100);
		                    Tip4_Collapse /= 100;
		                    Tip4_Collapse += HMI_Tip_4_Offset;
		                }
		            }
	    	}
        }

        else
        {
        	Tip4_Collapse=0;
        	Tip_4_Distance=0;

        }

//        if (FLAG_HOLD_DISTANCE_READING==0)
//        {
//        	if (FLAG_IN_CYCLE && FLAG_WORK_POSTION_SAVED ==1)
//        	{
//            	HMI_Tip_1_Distance = Tip1_Collapse > 0.01 ? Tip1_Collapse:0;
//                HMI_Tip_2_Distance = Tip2_Collapse > 0.01 ? Tip2_Collapse:0;
//                HMI_Tip_3_Distance = Tip3_Collapse > 0.01 ? Tip3_Collapse:0;
//        	}
//        	else
//        	{
//            	HMI_Tip_1_Distance = Tip_1_Distance> 0.2 ? Tip_1_Distance:0.00;
//                HMI_Tip_2_Distance = Tip_2_Distance> 0.2 ? Tip_2_Distance:0.00;
//                HMI_Tip_3_Distance = Tip_3_Distance> 0.2 ? Tip_3_Distance:0.00;
//        	}
//
//        }
//        updateAverage(Tip_1_Distance, Tip_2_Distance, Tip_3_Distance,Tip_4_Distance);

        if (FLAG_HOLD_DISTANCE_READING==0)
        {
        if (FLAG_IN_CYCLE && FLAG_WORK_POSTION_SAVED ==1)
        {
        HMI_Tip_1_Distance = Tip1_Collapse > 0.01 ? Tip1_Collapse:0;
        HMI_Tip_2_Distance = Tip2_Collapse > 0.01 ? Tip2_Collapse:0;
        HMI_Tip_3_Distance = Tip3_Collapse > 0.01 ? Tip3_Collapse:0;
        HMI_Tip_4_Distance = Tip4_Collapse > 0.01 ? Tip4_Collapse:0;
        }
        else
        {
         HMI_Tip_1_Distance = Tip_1_Distance> 0.2 ? Tip_1_Distance:0.00;
         HMI_Tip_2_Distance = Tip_2_Distance> 0.2 ? Tip_2_Distance:0.00;
         HMI_Tip_3_Distance = Tip_3_Distance> 0.2 ? Tip_3_Distance:0.00;
         HMI_Tip_4_Distance = Tip_4_Distance> 0.2 ? Tip_4_Distance:0.00;

        // Calculate the averages and assign to HMI variables
//        	HMI_Tip_1_Distance = fabs(HMI_Tip_1_Distance - average_tip_1) > 0.02 ?average_tip_1:HMI_Tip_1_Distance;
//        	HMI_Tip_2_Distance = fabs(HMI_Tip_2_Distance - average_tip_2) > 0.02 ?average_tip_2:HMI_Tip_2_Distance;
//        	HMI_Tip_3_Distance = fabs(HMI_Tip_3_Distance - average_tip_3) > 0.02 ?average_tip_3:HMI_Tip_3_Distance;
//        	HMI_Tip_4_Distance = fabs(HMI_Tip_4_Distance - average_tip_4) > 0.02 ?average_tip_4:HMI_Tip_4_Distance;
        }



        }
		set_home_banner();
        osDelay(SSR_PING_TIME);
    }
  /* USER CODE END i2cTaskFunc */
}

/* USER CODE BEGIN Header_print */
/**
* @brief Function implementing the Serial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_print */
void print(void *argument)
{
  /* USER CODE BEGIN print */
  /* Infinite loop */
  for(;;)
  {
		    	// Print switch readings
//		    	print_string("Switch readings: Right: ");
//		    	print_number(right_switch_received);
//		    	print_string(", Left: ");
//		    	print_number(left_switch_received);
//		    	print_string(", Home: ");
//		    	print_number(home_switch_received);
//		    	print_string("\r\n");
//
//		    	// Print current readings
//		    	print_string("Current readings: C1: ");
//		    	print_float(current1_amps);
//		    	print_string(", C2: ");
//		    	print_float(current2_amps);
//		    	print_string(", C3: ");
//		    	print_float(current3_amps);
//		    	print_string(", C4: ");
//		    	print_float(current4_amps);
//		    	print_string("\r\n");
//
//		    	// Print Linear Encoders
//		    	print_string("Linear Encoder 1: ");
//		    	print_float(fabs(Tip_1_Distance_Previous - Tip_1_Distance));
//	  	  	  	print_number(stepper_steps_count);
//	  	  	    print_string("\n");
//	  	  	  	print_number(stepper_run_count);
//	  	  	    print_string("\n");
//				print_string("Timer: ");
//				print_number(__HAL_TIM_GET_COUNTER(&htim12));
//				print_string("\r\n");

				// Print Linear Encoders
	  	  	  	print_string("Reached Time: ");
				print_string("T1: ");
				print_number(Reached_time[0]);
				print_string(",  T2: ");
				print_number(Reached_time[1]);
				print_string(",  T3: ");
				print_number(Reached_time[2]);
				print_string(",  T4: ");
				print_number(Reached_time[3]);
				print_string("\r\n");

	  	  	  	print_string("Tip Offsets: ");
				print_string("T1: ");
				print_number(Tip_Time_Offset[0]);
				print_string(",  T2: ");
				print_number(Tip_Time_Offset[1]);
				print_string(",  T3: ");
				print_number(Tip_Time_Offset[2]);
				print_string(",  T4: ");
				print_number(Tip_Time_Offset[3]);
				print_string("\r\n");


//
//				print_string("  ,Up Pulses: ");
//				print_number(up_pulses);
//				print_string("\r\n");
//
//				print_string("Current Height: ");
//				print_float(HMI_MANUAL_PLATEN_DISTANCE/1000);
//				print_string("\r\n");
//	  	  	  	print_string("Tip 1 Actual: ");
//	  	  	    print_float(Tip1_Boss_Start_Height/1000);
//	  	  	  	print_string(" ,Tip 1: ");
//	  	  	    print_float(TIP_1_BOSS_TRIGGERED);
//	  	  	  	print_string(" ,Tip 1 Current: ");
//	  	  	    print_float(TIP_1_CURRENT_BOSS_HEIGHT);
//	  	  	  	print_string(" ,Difference: ");
//	  	  	    print_float(Tip1_Boss_Start_Height - TIP_1_BOSS_TRIGGERED);
//	  	  	    print_string("\r\n");
//
//
//	  	  	  	print_string("Tip 2 Actual: ");
//	  	  	    print_float(Tip2_Boss_Start_Height);
//	  	  	  	print_string(" ,Tip 2: ");
//	  	  	    print_float(TIP_2_BOSS_TRIGGERED);
//	  	  	  	print_string(" ,Tip 2 Current: ");
//	  	  	    print_float(TIP_2_CURRENT_BOSS_HEIGHT);
//	  	  	  	print_string(" ,Difference: ");
//	  	  	    print_float(Tip2_Boss_Start_Height - TIP_2_BOSS_TRIGGERED);
//	  	  	    print_string("\r\n");
//
//	  	  	  	print_string("Tip 3 Actual: ");
//	  	  	    print_float(Tip3_Boss_Start_Height);
//	  	  	  	print_string(" ,Tip 3: ");
//	  	  	    print_float(TIP_3_BOSS_TRIGGERED);
//	  	  	  	print_string(" ,Tip 3 Current: ");
//	  	  	    print_float(TIP_3_CURRENT_BOSS_HEIGHT);
//	  	  	  	print_string(" ,Difference: ");
//	  	  	    print_float(Tip3_Boss_Start_Height - TIP_3_BOSS_TRIGGERED);
//	  	  	    print_string("\r\n");
//
//	  	  	  	print_string("Tip1: ");
//	  	  	    print_float(Tip_1_Distance);
//
//	  	  	  	print_string("  ,Tip2: ");
//	  	  	    print_float(Tip_2_Distance);
//
//	  	  	  	print_string("  ,Tip3: ");
//	  	  	    print_float(Tip_3_Distance);
//	  	  	    print_string("\r\n");
//		if (current_platen_position > (previous_platen_distance+5))
//		{
//		print_float(Tip_1_Distance);
//		print_string(",");
//		print_float( (HMI_MANUAL_PLATEN_DISTANCE-start_1_platen_position)/100);
//		print_string(",");
//		print_string(",");
//		print_float(Tip_2_Distance);
//		print_string(",");
//		print_float( (HMI_MANUAL_PLATEN_DISTANCE-start_2_platen_position)/100);
//		print_string(",");
//		print_string(",");
//		print_float(Tip_3_Distance);
//		print_string(",");
//		print_float( (HMI_MANUAL_PLATEN_DISTANCE-start_3_platen_position)/100);
//		print_string(",");
//		print_string(",");
//		print_float(Tip_4_Distance);
//		print_string(",");
//		print_float( (HMI_MANUAL_PLATEN_DISTANCE-start_4_platen_position)/100);
//		print_string(",");
//		print_string("\r\n");
//		}




    osDelay(500);
  }
  /* USER CODE END print */
}

/* USER CODE BEGIN Header_LedBlink */
/**
* @brief Function implementing the Led thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedBlink */
void LedBlink(void *argument)
{
  /* USER CODE BEGIN LedBlink */
  /* Infinite loop */
  for(;;)
  {
	  if (inactivityCounter >= 300)  // Change this value as per your requirement
	    {
	        // Put your screen to sleep or transition to the screen saver screen here
		  yourFlag = 1;  // Reset the counter after transition
	    }

	  if (control_data[8] ==0)
	  {
		  if (FLAG_REQUIRE_HOME)
		  {
			  control_data[8] =1;
			  osDelay(1000);
		  }
		  else if (FLAG_WAITING_IN_CYCLE)
		  {
			  control_data[8] =2;
			  osDelay(1000);
		  }
		  else if (FLAG_IN_CYCLE)
		  {
			  control_data[8] =3;
			  osDelay(1000);
		  }
	  }
	  control_data[8] =0;
	  osDelay(1000);
  }
  /* USER CODE END LedBlink */
}

/* USER CODE BEGIN Header_autoCycleTask */
/**
* @brief Function implementing the autoCycle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_autoCycleTask */
void autoCycleTask(void *argument)
{
  /* USER CODE BEGIN autoCycleTask */
  /* Infinite loop */
  for(;;)
  {
	  // 2 infinite loops which would break on certain conditions


	  // E-STOP State check
	  while(FLAG_HALT == 1)
	  {
		  FLAG_IN_CYCLE =0;
		  FLAG_WAITING_IN_CYCLE=0;
		  osDelay(10);
	  }

	  // Homing Required
	  while (FLAG_REQUIRE_HOME == 1)
	  {
		  FLAG_IN_CYCLE =0;
		  FLAG_WAITING_IN_CYCLE=0;
		  osDelay(10);
	  }

//	  // Work Position Setting Required
//	  while (FLAG_REQUIRE_HOME == 0 && (HMI_TEST_PRESS_DOWN_POSITION < MIN_WORK_POSITION) ) // Set Work Height State
//	  {
//		  FLAG_IN_CYCLE =0;
//		  FLAG_WAITING_IN_CYCLE=1;
//		  osDelay(10);
//	  }

	  ANTI_TIE_DOWN_ACTIVE=0;

	  // System Ready State
	  while( FLAG_REQUIRE_HOME == 0 && FLAG_HALT == 0 )

	  {
		  if (ANTI_TIE_DOWN_ACTIVE == 1 && FLAG_GOING_HOME == 0 && FLAG_HOMING_WITH_COOLING==0 )
		  {
			  FLAG_IN_PRE_CYCLE=1;

			  if (CYCLE_START_CLEARS==1)
			  {
				  CYCLE_START_CLEARS=0;

					// Reset all flags to 0
					FAULT_TE112_FLAG = 0;
					FAULT_TE212_FLAG=0;
					FAULT_TE312_FLAG=0;
					FAULT_TE412_FLAG=0;
					FAULT_ENCODER_1_FLAG = 0;
					FAULT_SSR_1_FLAG = 0;
					FAULT_SE108_FLAG = 0;
					FAULT_SE208_FLAG = 0;
					FAULT_SE308_FLAG = 0;
					FAULT_SE408_FLAG = 0;
					FAULT_MAX_DISTANCE_FLAG = 0;
					FAULT_CLEAR_FLAG = 0;
					FAULT_BOSS_NOT_PRESENT_FLAG_TIP1=0;
					FAULT_BOSS_NOT_PRESENT_FLAG_TIP2=0;
					FAULT_BOSS_NOT_PRESENT_FLAG_TIP3=0;
					FAULT_BOSS_NOT_PRESENT_FLAG_TIP4=0;
					FLAG_CYCLE_COMPLETED=0;

					//encoders[3].Enable = !Tip_1_Enabled;

//		  	  	  	print_string("  Tip1 BZ: ");
//		  	  	    print_float(Tip_1_Distance);
//
//		  	  	  	print_string("  Tip3 BZ: ");
//		  	  	    print_float(Tip_3_Distance);
		  	  	    //print_string("    ");

				  // Clear Encoders
					reset_linear_encoders();

					Tip_1_Distance = 0;
					HOME_Tip_1_Energy = 0;
					Tip_1_Energy_Consumed = 0;
					Tip1_Collapse = 0;

					Tip_2_Distance = 0;
					HOME_Tip_2_Energy = 0;
					Tip_2_Energy_Consumed = 0;
					Tip2_Collapse = 0;

					Tip_3_Distance = 0;
					HOME_Tip_3_Energy = 0;
					Tip_3_Energy_Consumed = 0;
					Tip3_Collapse = 0;

					Tip_4_Distance = 0;
					HOME_Tip_4_Energy = 0;
					Tip_4_Energy_Consumed = 0;
					Tip4_Collapse = 0;

//		  	  	  	print_string("  Tip1 AZ: ");
//		  	  	    print_float(Tip_1_Distance);
//
//		  	  	  	print_string("  Tip3 AZ: ");
//		  	  	    print_float(Tip_3_Distance);
//		  	  	    print_string("\r\n");

				  // Reset all tip energies
				  reset_tips(1,1,1,1);

				  if (FLAG_HOLD_DISTANCE_READING)
				  {
					  FLAG_HOLD_DISTANCE_READING=0;
				  }

				  // Cycle Timer
				  cycle_start_time = HAL_GetTick();  // Record the start time of the cycle

				  FLAG_CYCLE_ABORTED=0;

				  // Clear TIp Faults
				  FLAG_BOSS_NOT_PRESENT[0]=0;
				  FLAG_BOSS_NOT_PRESENT[1]=0;
				  FLAG_BOSS_NOT_PRESENT[2]=0;
				  FLAG_BOSS_NOT_PRESENT[3]=0;

				  // Clear Interference Errors
				  FLAG_INTERFERENCE[0]=0;
				  FLAG_INTERFERENCE[1]=0;
				  FLAG_INTERFERENCE[2]=0;
				  FLAG_INTERFERENCE[3]=0;

				  FLAG_TIP1_BOSS_TRIGGERED_SAVED=0;
				  FLAG_TIP2_BOSS_TRIGGERED_SAVED=0;
				  FLAG_TIP3_BOSS_TRIGGERED_SAVED=0;
				  FLAG_TIP4_BOSS_TRIGGERED_SAVED=0;

				  if (FLAG_WORK_POSTION_SAVED)
				  {
					  FLAG_FIRST_CYCLE=0;
				  }

				  FLAG_CYCLE_SUCCESS=0;

				  FAULT_HANDS_RELEASED_FLAG=0;
				  FLAG_CYCLE_STARTED=0;

				  FLAG_HEATING_START_TIME_SET=0;


				  // Set Tips distance according to mode selected

					AUTO_CYCLE_STOP_HEIGHT = !Brass_Mode_Enabled ? Tip_1_Distance_Val:AUTOCYCLE_STOP_HEIGHT;

					DISTANCE_STOP_TOLERANCE = !Brass_Mode_Enabled ?DISTANCE_STOP_TOLERANCE_BRASS:DISTANCE_STOP_TOLERANCE_PLASTIC;

			  }

			  if (HMI_MANUAL_PLATEN_DISTANCE > 200)
			  {
				  FLAG_CYCLE_STARTED=1;
			  }

			  // Check if heating distance reached
			  if ( (HMI_MANUAL_PLATEN_DISTANCE >= (PREHEAT_START_DISTANCE*HMI_TEST_PRESS_DOWN_POSITION)) && FLAG_WORK_POSTION_SAVED==1)
			  {

				  // Set Heating Started
				  FLAG_HEATING_STARTED=1;

					if (FLAG_HEATING_START_TIME_SET==0)
					{
						Heat_Start_Time = HAL_GetTick();
						FLAG_HEATING_START_TIME_SET=1;
					}


				  if (!Tip_1_Enabled )
				  {
				  // Set Energy Val to reach
				  tip1_energy_val_set = Tip_1_Energy_Val;

				  // Start Heating
				  FLAG_TIP1_HEATING_START=1;
				  }


				  if (!Tip_2_Enabled)
				  {
				  // Set Energy Val to reach for Tip 2
				  tip2_energy_val_set = Tip_2_Energy_Val;

				  // Start Heating for Tip 2
				  FLAG_TIP2_HEATING_START = 1;
				  }

				  if (!Tip_3_Enabled)
				  {
				  // Set Energy Val to reach for Tip 3
				  tip3_energy_val_set = Tip_3_Energy_Val;

				  // Start Heating for Tip 3
				  FLAG_TIP3_HEATING_START = 1;
				  }

				  if (!Tip_4_Enabled)
				  {
				  // Set Energy Val to reach for Tip 4
				  tip4_energy_val_set = Tip_4_Energy_Val;

				  // Start Heating for Tip 3
				  FLAG_TIP4_HEATING_START = 1;
				  }

			  }

			  // Check if Platen has reached the work position
			  if ((HMI_MANUAL_PLATEN_DISTANCE >= HMI_TEST_PRESS_DOWN_POSITION ) && FLAG_WORK_POSTION_SAVED==1)
			  {
				  // Cycle Reached Proper position to succeed to Distance Calc
				  success_cycle();
				  // Stop stepper and move to heating
				  break;

			  }


			  // Max Compression Reached By Tips. Stop Stepper
			  if ( (fabs(Tip_1_Distance) > AUTO_CYCLE_STOP_HEIGHT)
					|| (fabs(Tip_2_Distance) > AUTO_CYCLE_STOP_HEIGHT)
					|| (fabs(Tip_3_Distance) > AUTO_CYCLE_STOP_HEIGHT)
					|| (fabs(Tip_4_Distance) > AUTO_CYCLE_STOP_HEIGHT))
				{
					  // Cycle Reached Proper position to succeed to Distance Calc
					  success_cycle();
					break;
				}

#ifdef BOSS_NOT_PRESENT_ENABLED
				if (FLAG_WORK_POSTION_SAVED==1)
				{
				  // Boss Not present error routine

					if (HMI_MANUAL_PLATEN_DISTANCE >= (Tip1_Boss_Start_Height + BOSS_NOT_PRESENT_TOLERANCE) && !Tip_1_Enabled)
					{
						  FLAG_BOSS_NOT_PRESENT[0] = ((fabs(Tip_1_Distance) < 0.3) && !Tip_1_Enabled);
					}
					if (HMI_MANUAL_PLATEN_DISTANCE >= (Tip2_Boss_Start_Height + BOSS_NOT_PRESENT_TOLERANCE) && !Tip_2_Enabled)
					{
						  FLAG_BOSS_NOT_PRESENT[1] = ((fabs(Tip_2_Distance) < 0.3) && !Tip_2_Enabled);
					}
					if (HMI_MANUAL_PLATEN_DISTANCE >= (Tip3_Boss_Start_Height + BOSS_NOT_PRESENT_TOLERANCE) && !Tip_3_Enabled)
					{
						  FLAG_BOSS_NOT_PRESENT[2] = ((fabs(Tip_3_Distance) < 0.3) && !Tip_3_Enabled);
					}
					if (HMI_MANUAL_PLATEN_DISTANCE >= (Tip4_Boss_Start_Height + BOSS_NOT_PRESENT_TOLERANCE) && !Tip_3_Enabled)
					{
						  FLAG_BOSS_NOT_PRESENT[3] = ((fabs(Tip_4_Distance) < 0.3) && !Tip_4_Enabled);
					}


					  if ( FLAG_BOSS_NOT_PRESENT[0] || FLAG_BOSS_NOT_PRESENT[1] || FLAG_BOSS_NOT_PRESENT[2] || FLAG_BOSS_NOT_PRESENT[3] ) {

						  //Set Fault Active for Modal Windows
						  FAULT_ACTIVE_FLAG=1;

						  //Set home with cooling flag
						  FLAG_HOMING_WITH_COOLING=1;

						  // Buzz
						  Buzz(FAST);

						  // Abort Cycle
						  abort_cycle();

						  // Jump to Cooling
						  goto Cooling;
					  }
				}

#endif

			  if (fabs(Tip_1_Distance) > 0.3
				  || fabs(Tip_2_Distance) > 0.3
				  || fabs(Tip_3_Distance) > 0.3
				  || fabs(Tip_4_Distance) > 0.3
				  )
			  {



				  if (FLAG_WORK_POSTION_SAVED ==0)
				  {
						  stepper_setSpeed(AF_SLOW);
					  if (!Tip_1_Enabled)
					  {
						  if (Tip_1_Distance > 3.00)
						  {

							  if (FLAG_TIP1_BOSS_TRIGGERED_SAVED==0)
							  {
								  Tip1_Boss_Start_Height = HMI_MANUAL_PLATEN_DISTANCE-(Tip_1_Distance*100);;
								  TIP_1_CURRENT_BOSS_HEIGHT = Tip1_Boss_Start_Height;
								  FLAG_TIP1_BOSS_TRIGGERED_SAVED=1;
							  }

						  }
					  }
					  else
					  {
						  FLAG_TIP1_BOSS_TRIGGERED_SAVED=1;
					  }


					  if (!Tip_2_Enabled)
					  {
						  if (Tip_2_Distance > 3.00)
						  {
							  if (FLAG_TIP2_BOSS_TRIGGERED_SAVED==0)
							  {
								  Tip2_Boss_Start_Height = HMI_MANUAL_PLATEN_DISTANCE-(Tip_2_Distance*100);
								  TIP_2_CURRENT_BOSS_HEIGHT = Tip2_Boss_Start_Height;
								  FLAG_TIP2_BOSS_TRIGGERED_SAVED=1;
							  }
						  }
					  }
					  else
					  {
						  FLAG_TIP2_BOSS_TRIGGERED_SAVED=1;
					  }

					  if (!Tip_3_Enabled)
					  {
						  if (Tip_3_Distance > 3.00)
						  {
						  if (FLAG_TIP3_BOSS_TRIGGERED_SAVED==0)
							  {
							  	  Tip3_Boss_Start_Height = HMI_MANUAL_PLATEN_DISTANCE-(Tip_3_Distance*100);
							  	  TIP_3_CURRENT_BOSS_HEIGHT = Tip3_Boss_Start_Height;
								  FLAG_TIP3_BOSS_TRIGGERED_SAVED=1;
							  }
						  }
					  }
					  else
					  {
						  FLAG_TIP3_BOSS_TRIGGERED_SAVED=1;
					  }

					  if (!Tip_4_Enabled)
					  {
						  if (Tip_4_Distance > 3.00)
						  {
						  if (FLAG_TIP4_BOSS_TRIGGERED_SAVED==0)
							  {
							  	  Tip4_Boss_Start_Height = HMI_MANUAL_PLATEN_DISTANCE-(Tip_4_Distance*100);
							  	  TIP_4_CURRENT_BOSS_HEIGHT = Tip4_Boss_Start_Height;
								  FLAG_TIP4_BOSS_TRIGGERED_SAVED=1;
							  }
						  }
					  }
					  else
					  {
						  FLAG_TIP4_BOSS_TRIGGERED_SAVED=1;
					  }



					  if (      (FLAG_TIP1_BOSS_TRIGGERED_SAVED)
							  &&(FLAG_TIP2_BOSS_TRIGGERED_SAVED)
							  &&(FLAG_TIP3_BOSS_TRIGGERED_SAVED)
							  &&(FLAG_TIP4_BOSS_TRIGGERED_SAVED)
						 )
					  {
						  HMI_TEST_PRESS_DOWN_POSITION = HMI_MANUAL_PLATEN_DISTANCE + 500;
						  FLAG_WORK_POSTION_SAVED=1;
						  // Abort Cycle
						  abort_cycle();

						  // Jump to Cooling
						  goto Cooling;
					  }
				  }

				  else
				  {
#ifdef INTERFERENCE_DETECT_ENABLED
					    // Interference Detected
					    if (Tip_1_Distance > 3) {
//				  	  	  	print_string("Tip1 FAULT**************: ");
//				  	  	    print_float(Tip_1_Distance);
					        if (FLAG_INTERFERENCE[0]) {
					        	 FLAG_INTERFERENCE[0] = (HMI_MANUAL_PLATEN_DISTANCE < (Tip1_Boss_Start_Height - INTERFERENCE_TOLERANCE));
					            // Check if the condition is still met after the delay threshold
					            if (HAL_GetTick() - tip_1_condition_start_time >= INTERFERENCE_DELAY_THRESHOLD) {
					                // Set FLAG_INTERFERENCE to 1
					                FLAG_INTERFERENCE[0] = 1;
					            }
					        } else {
					            // Start the timer if the condition is first met
					            tip_1_condition_start_time = HAL_GetTick();
					            FLAG_INTERFERENCE[0] = (HMI_MANUAL_PLATEN_DISTANCE < (Tip1_Boss_Start_Height - INTERFERENCE_TOLERANCE));
					        }
					    } else {
					        // Reset the timer and FLAG_INTERFERENCE if condition is not met
					        tip_1_condition_start_time = HAL_GetTick();
					        FLAG_INTERFERENCE[0] = 0;
					    }

					    // Interference Detected for Tip 2
					    if (Tip_2_Distance > 3) {
//				  	  	  	print_string("  ,Tip2: ");
//				  	  	    print_float(Tip_2_Distance);
					        if (FLAG_INTERFERENCE[1]) {
					        	FLAG_INTERFERENCE[1] = (HMI_MANUAL_PLATEN_DISTANCE < (Tip2_Boss_Start_Height - INTERFERENCE_TOLERANCE));
					            // Check if the condition is still met after the delay threshold
					            if (HAL_GetTick() - tip_2_condition_start_time >= INTERFERENCE_DELAY_THRESHOLD) {
					                // Set FLAG_INTERFERENCE to 1
					                FLAG_INTERFERENCE[1] = 1;
					            }
					        } else {
					            // Start the timer if the condition is first met
					            tip_2_condition_start_time = HAL_GetTick();
					            FLAG_INTERFERENCE[1] = (HMI_MANUAL_PLATEN_DISTANCE < (Tip2_Boss_Start_Height - INTERFERENCE_TOLERANCE));
					        }
					    } else {
					        // Reset the timer and FLAG_INTERFERENCE if condition is not met
					        tip_2_condition_start_time = HAL_GetTick();
					        FLAG_INTERFERENCE[1] = 0;
					    }

					    // Interference Detected for Tip 3
					    if (Tip_3_Distance > 3) {
//				  	  	  	print_string("  Tip3 FAULT**************: ");
//				  	  	    print_float(Tip_3_Distance);
//				  	  	    print_string("\r\n");

					        if (FLAG_INTERFERENCE[2]) {
					        	FLAG_INTERFERENCE[2] = (HMI_MANUAL_PLATEN_DISTANCE < (Tip3_Boss_Start_Height - INTERFERENCE_TOLERANCE));
					            // Check if the condition is still met after the delay threshold
					            if (HAL_GetTick() - tip_3_condition_start_time >= INTERFERENCE_DELAY_THRESHOLD) {
					                // Set FLAG_INTERFERENCE to 1
					                FLAG_INTERFERENCE[2] = 1;
					            }
					        } else {
					            // Start the timer if the condition is first met
					            tip_3_condition_start_time = HAL_GetTick();
					            FLAG_INTERFERENCE[2] = (HMI_MANUAL_PLATEN_DISTANCE < (Tip3_Boss_Start_Height - INTERFERENCE_TOLERANCE));
					        }
					    } else {
					        // Reset the timer and FLAG_INTERFERENCE if condition is not met
					        tip_3_condition_start_time = HAL_GetTick();;
					        FLAG_INTERFERENCE[2] = 0;
					    }

					    // Interference Detected for Tip 4
					    if (Tip_4_Distance > 3) {
//				  	  	  	print_string("  Tip4 FAULT**************: ");
//				  	  	    print_float(Tip_4_Distance);
//				  	  	    print_string("\r\n");

					        if (FLAG_INTERFERENCE[3]) {
					        	FLAG_INTERFERENCE[3] = (HMI_MANUAL_PLATEN_DISTANCE < (Tip4_Boss_Start_Height - INTERFERENCE_TOLERANCE));
					            // Check if the condition is still met after the delay threshold
					            if (HAL_GetTick() - tip_4_condition_start_time >= INTERFERENCE_DELAY_THRESHOLD) {
					                // Set FLAG_INTERFERENCE to 1
					                FLAG_INTERFERENCE[3] = 1;
					            }
					        } else {
					            // Start the timer if the condition is first met
					            tip_4_condition_start_time = HAL_GetTick();
					            FLAG_INTERFERENCE[3] = (HMI_MANUAL_PLATEN_DISTANCE < (Tip3_Boss_Start_Height - INTERFERENCE_TOLERANCE));
					        }
					    } else {
					        // Reset the timer and FLAG_INTERFERENCE if condition is not met
					        tip_4_condition_start_time = HAL_GetTick();;
					        FLAG_INTERFERENCE[3] = 0;
					    }

					  if ( FLAG_INTERFERENCE[0] || FLAG_INTERFERENCE[1] || FLAG_INTERFERENCE[2] || FLAG_INTERFERENCE[3])
						  {


						  	  //Set home with cooling flag
							  FLAG_HOMING_WITH_COOLING=1;

							  // Buzz
							  Buzz(FAST);

							  // Abort Cycle
							  abort_cycle();

							  // Jump to Cooling
							  goto Cooling;
						  }


					  //Slow Down Stepper
					  else
						  {
						  	  stepper_setSpeed(CD_LOW);
						  }
#endif

#ifndef INTERFERENCE_DETECT_ENABLED
					  	  stepper_setSpeed(CD_LOW);
#endif
				  }


			  }


				  else
				  {
					  if (FLAG_FIRST_CYCLE==1 && FLAG_WORK_POSTION_SAVED==0)
					  {
						  stepper_setSpeed(AF_HIGH);
					  }
					  else if ((HMI_MANUAL_PLATEN_DISTANCE > (HMI_TEST_PRESS_DOWN_POSITION-5000)) && FLAG_WORK_POSTION_SAVED==1 )
					  {
						  stepper_setSpeed(CD_LOW);
					  }
					  else
					  {
						  stepper_setSpeed(CD_HIGH);
					  }

				  }



			  stepper_Move_Down();
			  FLAG_IN_CYCLE =1;
			  FLAG_WAITING_IN_CYCLE=0;
			  osDelay(5);
		  }

		  // Anti tie down not active.
		  else
		  {
			  if (FLAG_CYCLE_STARTED == 1)
			  {
				  // Buzz
				  Buzz(FAST);

				  // Slow Down Stepper
				  stepper_setSpeed(10);

				  // Set Hands Released Flag
				  FAULT_HANDS_RELEASED_FLAG=1;

				  FLAG_CYCLE_STARTED=0;

				  uint32_t current_time_hands_released= HAL_GetTick();

				  while(HAL_GetTick() - current_time_hands_released < 1000 )
				  {
					  stepper_Move_Down();
					  osDelay(1);
				  }



				  // Stop The stepper
				  stepper_stop();

				  // Abort Cycle
				  abort_cycle();

				  // Jump to Cooling
				  goto Cooling;
			  }
				  if (FLAG_HEATING_STARTED == 1 && FLAG_REPEAT_CYCLE==0 )
				  {
				  // Abort Cycle
				  abort_cycle();

				  // Jump to Cooling
				  goto Cooling;
			  }
				  if (FLAG_REPEAT_CYCLE==0)
				  {
					  FLAG_IN_CYCLE =0;
				  }

			  FLAG_WAITING_IN_CYCLE=1;
			  osDelay(10);
		  }


	  }
	  FLAG_IN_PRE_CYCLE=0;

	  if (FLAG_HALT == 1)
	  {
		  // Abort Cycle
		  abort_cycle();
		  if (FLAG_HEATING_STARTED)
		  {
			  goto Cooling;
		  }
	  }

	  if (FLAG_IN_CYCLE == 1)

	  {

		  // let the tips sink and wait for distance to stop changing.
		  uint32_t current_time = HAL_GetTick();
		  double current_tip1_collapse = Tip1_Collapse;
		  double current_tip2_collapse = Tip2_Collapse;
		  double current_tip3_collapse = Tip3_Collapse;
		  double current_tip4_collapse = Tip4_Collapse;

		  double previous_tip1_collapse = Tip1_Collapse;
		  double previous_tip2_collapse = Tip2_Collapse;
		  double previous_tip3_collapse = Tip3_Collapse;
		  double previous_tip4_collapse = Tip4_Collapse;

		  while((HAL_GetTick() - current_time < DELAY_AFTER_HEATING))
		  {
		      current_tip1_collapse = Tip1_Collapse;
		      current_tip2_collapse = Tip2_Collapse;
		      current_tip3_collapse = Tip3_Collapse;
		      current_tip4_collapse = Tip4_Collapse;

		      // Prevent timeout before this distance is reached by tip 1
		      if (!Brass_Mode_Enabled && Tip1_Collapse < 2.0)
		      {
		    		  current_time = HAL_GetTick();
		      }

		      else
		      {
			      if (
					  (fabs(current_tip1_collapse - previous_tip1_collapse) > DISTANCE_CHANGE_TOLERANCE) ||
					  (fabs(current_tip2_collapse - previous_tip2_collapse) > DISTANCE_CHANGE_TOLERANCE) ||
					  (fabs(current_tip3_collapse - previous_tip3_collapse) > DISTANCE_CHANGE_TOLERANCE) ||
					  (fabs(current_tip4_collapse - previous_tip4_collapse) > DISTANCE_CHANGE_TOLERANCE)
					  )
			      {
			          previous_tip1_collapse = current_tip1_collapse;
			          previous_tip2_collapse = current_tip2_collapse;
			          previous_tip3_collapse = current_tip3_collapse;
			          previous_tip4_collapse = current_tip4_collapse;
			          current_time = HAL_GetTick();
			      }
		      }


		      if ((Tip1_Collapse >= Tip_1_Distance_Val)  && FLAG_TIP1_HEATING_START==1)
				{
		    	  FLAG_TIP1_HEATING_START=0;
		    	  Reached_time[0] = HAL_GetTick() - Heat_Start_Time ;

//		    	  // Turn Rest of the Tips on
//		    	  TIP_2 =1;
//		    	  TIP_3 =1;
//		    	  TIP_4 =1;
//		    	  // Cool
//		    	  cooling_burst(COOLING_BURST);
//		    	  // Turn rest of the Tips off
//		    	  TIP_2 =0;
//		    	  TIP_3 =0;
//		    	  TIP_4 =0;
//		    	  // Add heating to Rest of the Tips
//		    	  tip2_energy_val_set += HEATING_BURST;
//		    	  tip3_energy_val_set += HEATING_BURST;
//		    	  tip4_energy_val_set += HEATING_BURST;

				}

		      if ((Tip2_Collapse >= Tip_2_Distance_Val)  && FLAG_TIP2_HEATING_START==1)
				{
		    	  FLAG_TIP2_HEATING_START=0;

		    	  Reached_time[1] = HAL_GetTick() - Heat_Start_Time ;

				}

		      if ((Tip3_Collapse >= Tip_3_Distance_Val)  && FLAG_TIP3_HEATING_START==1)
				{
		    	  FLAG_TIP3_HEATING_START=0;
		    	  Reached_time[2] = Heat_Start_Time - HAL_GetTick();

				}

		      if ((Tip4_Collapse >= Tip_4_Distance_Val)  && FLAG_TIP4_HEATING_START==1)
				{
		    	  FLAG_TIP4_HEATING_START=0;
		    	  Reached_time[3] = Heat_Start_Time - HAL_GetTick();

				}

//		      // Set Encoder Colors to Green when tips reach
//				if (Tip1_Collapse >= (Tip_1_Distance_Val))
//				{
//					set_encoder_led_color(GREEN, LINEAR_ENCODER_1_ADDRESS,!Tip_1_Enabled);
//				}
//
//			    if (Tip2_Collapse >= (Tip_2_Distance_Val))
//			    {
//			    	set_encoder_led_color(GREEN, LINEAR_ENCODER_2_ADDRESS,!Tip_2_Enabled);
//			    }
//
//			    if (Tip3_Collapse >= (Tip_3_Distance_Val))
//			    {
//			    	set_encoder_led_color(GREEN, LINEAR_ENCODER_3_ADDRESS,!Tip_3_Enabled);
//			    }

		      // Check if all tips have reached the end distance
		      if (
		    	 ( (Tip1_Collapse >= Tip_1_Distance_Val)  || Tip_1_Enabled) &&
		         ( (Tip2_Collapse >= Tip_2_Distance_Val)  || Tip_2_Enabled) &&
		         ( (Tip3_Collapse >= Tip_3_Distance_Val)  || Tip_3_Enabled) &&
		         ( (Tip4_Collapse >= Tip_4_Distance_Val)  || Tip_4_Enabled)
		         )
		      {
		          break;
		      }

		      osDelay(5);
		  }

		  // Calculate Heating Delays for next cycle
		 // heating_delays();


Cooling:

					abort_heating();
					FLAG_HOLD_DISTANCE_READING=1;

					if (FLAG_CYCLE_ABORTED)
					{
						if (FLAG_BOSS_NOT_PRESENT[0])
						{
							set_encoder_led_color(RED,encoders[0].ID);
							FAULT_BOSS_NOT_PRESENT_FLAG_TIP1=1; // Home banner
						}
						else if (FLAG_BOSS_NOT_PRESENT[1])
						{
							set_encoder_led_color(RED,encoders[1].ID);
							FAULT_BOSS_NOT_PRESENT_FLAG_TIP2=1;
						}
						else if (FLAG_BOSS_NOT_PRESENT[2])
						{
							set_encoder_led_color(RED,encoders[2].ID);
							FAULT_BOSS_NOT_PRESENT_FLAG_TIP3=1;
						}
						else if (FLAG_BOSS_NOT_PRESENT[3])
						{
							set_encoder_led_color(RED,encoders[3].ID);
							FAULT_BOSS_NOT_PRESENT_FLAG_TIP4=1;
						}
						else if (FLAG_INTERFERENCE[0])
						{
							set_encoder_led_color(RED,encoders[0].ID);
							FAULT_SE108_FLAG=1;
						}
						else if (FLAG_INTERFERENCE[1])
						{
							set_encoder_led_color(RED,encoders[1].ID);
							FAULT_SE208_FLAG=1;
						}
						else if (FLAG_INTERFERENCE[2])
						{
							set_encoder_led_color(RED,encoders[2].ID);
							FAULT_SE308_FLAG=1;
						}
						else if (FLAG_INTERFERENCE[3])
						{
							set_encoder_led_color(RED,encoders[3].ID);
							FAULT_SE408_FLAG=1;
						}
					}

					else
					{
							set_encoder_led_color(BLUE,encoders[0].ID);
							set_encoder_led_color(BLUE,encoders[1].ID);
							set_encoder_led_color(BLUE,encoders[2].ID);
							set_encoder_led_color(BLUE,encoders[3].ID);
					}

					if (FLAG_HEATING_STARTED)
					{
						if (!Brass_Mode_Enabled)
						{
			  	  	  	  	Cooling_Valve = 1;
			  	  	  	  	osDelay(1000);
			  	  		// Set stepper to low speed and start moving up
			  	  		stepper_setSpeed(AF_SLOW);
			  	  		slow_down_distance= HMI_MANUAL_PLATEN_DISTANCE - PLATEN_RETRACT_AFTER_COOLING;
			  	  		while((HMI_MANUAL_PLATEN_DISTANCE > slow_down_distance) && FLAG_GOING_HOME ==0)
			  	  		{
			  	  			stepper_Move_Up();
			  	  			osDelay(1);
			  	  		}
		  	  	  	  	osDelay(3000);

						}
						else
						{
							if (FLAG_HOMING_WITH_COOLING==0)
							{
				  	  	  	  	Cooling_Valve = 1;
					  		  	//osDelay(HMI_COOLING_TIME_SET * 100);
					  		  	//Cooling_Valve = 0;
							}
						}


			  		    FLAG_HEATING_STARTED=0;
					}

		  		  	// Check if tips have reached distance only if there is no cycle fault
		  		  	if (FLAG_CYCLE_SUCCESS ==1)
		  		  	{
						if (!Tip_1_Enabled)
							{
								  if (Tip1_Collapse >= (Tip_1_Distance_Val- DISTANCE_STOP_TOLERANCE))
								  {
									  set_encoder_led_color(GREEN, encoders[0].ID);
									  FAULT_TE112_FLAG=0;
								  }
								  else
								  {
									  set_encoder_led_color(RED, encoders[0].ID);
									  FAULT_TE112_FLAG=1;
								  }

							}

						if (!Tip_2_Enabled)
						{
							  if (Tip2_Collapse >= (Tip_2_Distance_Val- DISTANCE_STOP_TOLERANCE))
							  {
								  set_encoder_led_color(GREEN, encoders[1].ID);
								  FAULT_TE212_FLAG=0;
							  }
							  else
							  {
								  set_encoder_led_color(RED, encoders[1].ID);
								  FAULT_TE212_FLAG=1;
							  }
						}


						if (!Tip_3_Enabled)
						{
							  if (Tip3_Collapse >= (Tip_3_Distance_Val- DISTANCE_STOP_TOLERANCE))
							  {
								  set_encoder_led_color(GREEN, encoders[2].ID);
								  FAULT_TE312_FLAG=0;
							  }
							  else
							  {
								  set_encoder_led_color(RED, encoders[2].ID);
								  FAULT_TE312_FLAG=1;
							  }
						}

						if (!Tip_4_Enabled)
						{
							  if (Tip4_Collapse >= (Tip_4_Distance_Val- DISTANCE_STOP_TOLERANCE))
							  {
								  set_encoder_led_color(GREEN, encoders[3].ID);
								  FAULT_TE412_FLAG=0;
							  }
							  else
							  {
								  set_encoder_led_color(RED, encoders[3].ID);
								  FAULT_TE412_FLAG=1;
							  }
						}
		  		  	}



		FLAG_CYCLE_COMPLETED=1;
		//Go_Home();
		double current_distance_cycle = HMI_MANUAL_PLATEN_DISTANCE;
		stepper_setSpeed(CU_LOW);
		while(HMI_MANUAL_PLATEN_DISTANCE > (current_distance_cycle - 1000) && (HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin)))
		{
			stepper_Move_Up();
			osDelay(2);
		}
		stepper_setSpeed(CU_HIGH);
		if (FLAG_HOMING_WITH_COOLING)
		{
			osDelay(HOMING_WITH_COOLING_DELAY);
		}
		Go_Home_AutoCycle();
		osDelay(HMI_COOLING_TIME_SET * 100);
		Cooling_Valve = 0;
		FLAG_IN_CYCLE=0;

	  }
	  CYCLE_START_CLEARS=1;
	  FLAG_CYCLE_STARTED=0;


    osDelay(500);
  }
  /* USER CODE END autoCycleTask */
}

/* USER CODE BEGIN Header_GoHomeTask */
/**
* @brief Function implementing the GoHome thread.
* @param argument: Not used
* @retval None
*/



/* USER CODE END Header_GoHomeTask */
void GoHomeTask(void *argument)
{
  /* USER CODE BEGIN GoHomeTask */
  /* Infinite loop */
  for(;;)
  {
	  HMI_MONITOR_OVERTRAVEL = !HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin);
	  if (FLAG_HALT == 0) {
			uint32_t time_difference = HAL_GetTick() - Heat_Start_Time;
	      if (FLAG_TIP1_HEATING_START && !Tip_1_Enabled && (time_difference > Tip_Time_Offset[0])) {
	          if (FLAG_TIP1_HEATING_IN_PROGRESS == 0) {
	              // Set the heating start time
	              tip1_heating_start_time = millis();
	          }

	          // Check if Tip 1 has reached current set energy level
	          if (Tip_1_Energy_Consumed <= tip1_energy_val_set) {
	              FLAG_TIP1_HEATING_IN_PROGRESS = 1;


	              switch (tip1_heating_state) {
	                  case STATE_HEATING:
	              // Turn On Heating for Tip 1
	              TIP_1 = 1;

	              // Calculate Energy for Tip 1
	              tip1_heating_time_passed_ms = millis() - tip1_heating_start_time;

	              // Calculate the time passed during this iteration for Tip 1
	              tip1_iteration_time_passed_s = tip1_heating_time_passed_ms / 1000.0;

	              if (FLAG_IN_PRE_CYCLE) {
	                  // Calculate the energy consumed during this iteration for Tip 1 and add it to the total
	                  Tip_1_Energy_Consumed += (SSR_VOLTAGE * current1_amps * tip1_iteration_time_passed_s) / 2;
	              }
	              else {
	                  // Calculate the energy consumed during this iteration for Tip 1 and add it to the total
	                  Tip_1_Energy_Consumed += SSR_VOLTAGE * current1_amps * tip1_iteration_time_passed_s;
	              }

	              // Update the heating_start_time for Tip 1 to be used for the next iteration
	              tip1_heating_start_time = millis();

	              // Set HMI Energy Values for Tip 1
	              HOME_Tip_1_Energy = (uint16_t)Tip_1_Energy_Consumed;

	              // Check if the energy consumption has increased by 50 or more Joules since the last delay
	              if (Tip_1_Energy_Consumed - last_delay_energy_value >= JOULES_PER_STEP) {
	                  // Update the last energy value at which delay was introduced
	                  last_delay_energy_value = Tip_1_Energy_Consumed;

	                  // Set the state to delay
	                  tip1_heating_state = STATE_DELAY;
	                  tip1_delay_start_time = HAL_GetTick();
	              }
	              break;

	          case STATE_DELAY:
	              // Turn Off Heating for Tip 1 during the delay
	              TIP_1 = 0;

	              // Check if 200 ms have passed since the delay started
	              if (HAL_GetTick() - tip1_delay_start_time >= HEATING_DELAY_PER_STEP) {
	                  // Set the state back to heating
	                  tip1_heating_state = STATE_HEATING;
	              }
	              break;
	      }
	          }
	          else {
	              // Turn Off Heating for Tip 1
                  // Set the state back to heating
                  tip1_heating_state = STATE_HEATING;
                  last_delay_energy_value=0;
	              TIP_1 = 0;
	              FLAG_TIP1_HEATING_IN_PROGRESS = 0;
	          }
	      }
	      else {
	          //control_data[0] = 0;
              // Set the state back to heating
              tip1_heating_state = STATE_HEATING;
              last_delay_energy_value=0;
	          TIP_1 = 0;
	          FLAG_TIP1_HEATING_IN_PROGRESS = 0;
	      }

	      if (FLAG_TIP2_HEATING_START && !Tip_2_Enabled && (time_difference > Tip_Time_Offset[1])) {
	          if (FLAG_TIP2_HEATING_IN_PROGRESS == 0) {
	              // Set the heating start time for Tip 2
	              tip2_heating_start_time = millis();
	          }

	          // Check if Tip 2 has reached current set energy level
	          if (Tip_2_Energy_Consumed <= tip2_energy_val_set) {
	              FLAG_TIP2_HEATING_IN_PROGRESS = 1;
	              switch (tip2_heating_state) {
	                  case STATE_HEATING:
	              TIP_2 = 1;

	              // Calculate Energy for Tip 2
	              tip2_heating_time_passed_ms = millis() - tip2_heating_start_time;

	              // Calculate the time passed during this iteration for Tip 2
	              tip2_iteration_time_passed_s = tip2_heating_time_passed_ms / 1000.0;

	              if (FLAG_IN_PRE_CYCLE) {
	                  // Calculate the energy consumed during this iteration for Tip 2 and add it to the total
	                  Tip_2_Energy_Consumed += (SSR_VOLTAGE * current2_amps * tip2_iteration_time_passed_s) / 2;
	              }
	              else {
	                  // Calculate the energy consumed during this iteration for Tip 2 and add it to the total
	                  Tip_2_Energy_Consumed += SSR_VOLTAGE * current2_amps * tip2_iteration_time_passed_s;
	              }

	              // Update the heating_start_time for Tip 2 to be used for the next iteration
	              tip2_heating_start_time = millis();

	              // Set HMI Energy Values for Tip 2
	              HOME_Tip_2_Energy = (uint16_t)Tip_2_Energy_Consumed;

	              // Check if the energy consumption has increased by 50 or more Joules since the last delay
	              if (Tip_2_Energy_Consumed - last_delay_energy_value_2 >= JOULES_PER_STEP) {
	                  // Update the last energy value at which delay was introduced
	                  last_delay_energy_value_2 = Tip_2_Energy_Consumed;

	                  // Set the state to delay
	                  tip2_heating_state = STATE_DELAY;
	                  tip2_delay_start_time = HAL_GetTick();
	              }
	              break;

	          case STATE_DELAY:
	              // Turn Off Heating for Tip 2 during the delay
	              TIP_2 = 0;

	              // Check if ms have passed since the delay started
	              if (HAL_GetTick() - tip2_delay_start_time >= HEATING_DELAY_PER_STEP) {
	                  // Set the state back to heating
	                  tip2_heating_state = STATE_HEATING;
	              }
	              break;
	      }
	          }
	          else {
	              // Turn Off Heating for Tip 2
                  tip2_heating_state = STATE_HEATING;
                  last_delay_energy_value_2=0;
	              TIP_2 = 0;
	              FLAG_TIP2_HEATING_IN_PROGRESS = 0;
	          }
	      }
	      else {
              tip2_heating_state = STATE_HEATING;
              last_delay_energy_value_2=0;
	          TIP_2 = 0;
	          FLAG_TIP2_HEATING_IN_PROGRESS = 0;
	      }

	      if (FLAG_TIP3_HEATING_START && !Tip_3_Enabled && (time_difference > Tip_Time_Offset[2])) {
	          if (FLAG_TIP3_HEATING_IN_PROGRESS == 0) {
	              // Set the heating start time for Tip 3
	              tip3_heating_start_time = millis();
	          }

	          // Check if Tip 3 has reached current set energy level
	          if (Tip_3_Energy_Consumed <= tip3_energy_val_set) {
	              FLAG_TIP3_HEATING_IN_PROGRESS = 1;
	              switch (tip3_heating_state) {
	                  case STATE_HEATING:
	              TIP_3 = 1;

	              // Calculate Energy for Tip 3
	              tip3_heating_time_passed_ms = millis() - tip3_heating_start_time;

	              // Calculate the time passed during this iteration for Tip 3
	              tip3_iteration_time_passed_s = tip3_heating_time_passed_ms / 1000.0;

	              if (FLAG_IN_PRE_CYCLE) {
	                  // Calculate the energy consumed during this iteration for Tip 3 and add it to the total
	                  Tip_3_Energy_Consumed += (SSR_VOLTAGE * current3_amps * tip3_iteration_time_passed_s) / 2;
	              }
	              else {
	                  // Calculate the energy consumed during this iteration for Tip 3 and add it to the total
	                  Tip_3_Energy_Consumed += SSR_VOLTAGE * current3_amps * tip3_iteration_time_passed_s;
	              }

	              // Update the heating_start_time for Tip 3 to be used for the next iteration
	              tip3_heating_start_time = millis();

	              // Set HMI Energy Values for Tip 3
	              HOME_Tip_3_Energy = (uint16_t)Tip_3_Energy_Consumed;

	              // Check if the energy consumption has increased by 50 or more Joules since the last delay
	              if (Tip_3_Energy_Consumed - last_delay_energy_value_3 >= JOULES_PER_STEP) {
	                  // Update the last energy value at which delay was introduced
	                  last_delay_energy_value_3 = Tip_3_Energy_Consumed;

	                  // Set the state to delay
	                  tip3_heating_state = STATE_DELAY;
	                  tip3_delay_start_time = HAL_GetTick();
	              }
	              break;

	          case STATE_DELAY:
	              // Turn Off Heating for Tip 2 during the delay
	              TIP_3 = 0;

	              // Check if ms have passed since the delay started
	              if (HAL_GetTick() - tip3_delay_start_time >= HEATING_DELAY_PER_STEP) {
	                  // Set the state back to heating
	                  tip3_heating_state = STATE_HEATING;
	              }
	              break;
	      }
	          }
	          else {
	              // Turn Off Heating for Tip 3
                  tip3_heating_state = STATE_HEATING;
                  last_delay_energy_value_3=0;
	              TIP_3 = 0;
	              FLAG_TIP3_HEATING_IN_PROGRESS = 0;
	          }
	      }
	      else {
              tip3_heating_state = STATE_HEATING;
              last_delay_energy_value_3=0;
	          TIP_3 = 0;
	          FLAG_TIP3_HEATING_IN_PROGRESS = 0;
	      }

	      if (FLAG_TIP4_HEATING_START && !Tip_4_Enabled && (time_difference > Tip_Time_Offset[3])) {
	          if (FLAG_TIP4_HEATING_IN_PROGRESS == 0) {
	              // Set the heating start time for Tip 3
	              tip4_heating_start_time = millis();
	          }

	          // Check if Tip 4 has reached current set energy level
	          if (Tip_4_Energy_Consumed <= tip4_energy_val_set) {
	              FLAG_TIP4_HEATING_IN_PROGRESS = 1;
	              switch (tip4_heating_state) {
	                  case STATE_HEATING:
	              TIP_4 = 1;

	              // Calculate Energy for Tip 3
	              tip4_heating_time_passed_ms = millis() - tip4_heating_start_time;

	              // Calculate the time passed during this iteration for Tip 3
	              tip4_iteration_time_passed_s = tip4_heating_time_passed_ms / 1000.0;

	              if (FLAG_IN_PRE_CYCLE) {
	                  // Calculate the energy consumed during this iteration for Tip 3 and add it to the total
	                  Tip_4_Energy_Consumed += (SSR_VOLTAGE * current4_amps * tip4_iteration_time_passed_s) / 2;
	              }
	              else {
	                  // Calculate the energy consumed during this iteration for Tip 3 and add it to the total
	                  Tip_4_Energy_Consumed += SSR_VOLTAGE * current4_amps * tip4_iteration_time_passed_s;
	              }

	              // Update the heating_start_time for Tip 3 to be used for the next iteration
	              tip4_heating_start_time = millis();

	              // Set HMI Energy Values for Tip 3
	              HOME_Tip_4_Energy = (uint16_t)Tip_4_Energy_Consumed;

	              // Check if the energy consumption has increased by 50 or more Joules since the last delay
	              if (Tip_4_Energy_Consumed - last_delay_energy_value_4 >= JOULES_PER_STEP) {
	                  // Update the last energy value at which delay was introduced
	                  last_delay_energy_value_4 = Tip_4_Energy_Consumed;

	                  // Set the state to delay
	                  tip4_heating_state = STATE_DELAY;
	                  tip4_delay_start_time = HAL_GetTick();
	              }
	              break;

	          case STATE_DELAY:
	              // Turn Off Heating for Tip 2 during the delay
	              TIP_4 = 0;

	              // Check if ms have passed since the delay started
	              if (HAL_GetTick() - tip4_delay_start_time >= HEATING_DELAY_PER_STEP) {
	                  // Set the state back to heating
	                  tip4_heating_state = STATE_HEATING;
	              }
	              break;
	      }
	          }
	          else {
	              // Turn Off Heating for Tip 3
	              tip4_heating_state = STATE_HEATING;
	              last_delay_energy_value_4=0;
	              TIP_4 = 0;
	              FLAG_TIP4_HEATING_IN_PROGRESS = 0;
	          }
	      }
	      else {
              tip4_heating_state = STATE_HEATING;
              last_delay_energy_value_4=0;
	          TIP_4 = 0;
	          FLAG_TIP4_HEATING_IN_PROGRESS = 0;
	      }

	  }
	  else {
		  TIP_1=0;
		  TIP_2=0;
		  TIP_3=0;
		  TIP_4=0;

	  }


	  osDelay(5);
  }

  /* USER CODE END GoHomeTask */
}

/* USER CODE BEGIN Header_Timer10OverflowTask */
/**
* @brief Function implementing the Timer10Overflow thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Timer10OverflowTask */
void Timer10OverflowTask(void *argument)
{
  /* USER CODE BEGIN Timer10OverflowTask */
  /* Infinite loop */
  for(;;)
  {
	  UpdatePlatenDistance();
	  MonitorStepper();
	  stepper_setSpeed_accel(last_stepper_speed_set);
	  osDelay(3);
  }
  /* USER CODE END Timer10OverflowTask */
}

/* USER CODE BEGIN Header_T_Autofind */
/**
* @brief Function implementing the Autofindtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_T_Autofind */
void T_Autofind(void *argument)
{
  /* USER CODE BEGIN T_Autofind */
  /* Infinite loop */
  for(;;)
  {
	 if (HMI_AUTOFIND)
	{
		Auto_Find();
	 }
    osDelay(10);
  }
  /* USER CODE END T_Autofind */
}

/* USER CODE BEGIN Header_T_Buzzer */
/**
* @brief Function implementing the Buzzer_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_T_Buzzer */
void T_Buzzer(void *argument)
{
  /* USER CODE BEGIN T_Buzzer */
  /* Infinite loop */
  for(;;)
  {
		if (FLAG_RUN_BUZZER)
		{
			for (int i=0;i<Buzzer_beeps;i++)
			{
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,0);
				osDelay(Buzzer_delay);
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,1);
				osDelay(Buzzer_delay);
			}
			FLAG_RUN_BUZZER=0;
		}

    osDelay(10);
  }
  /* USER CODE END T_Buzzer */
}

/* USER CODE BEGIN Header_Cooling */
/**
* @brief Function implementing the CoolingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Cooling */
void Cooling(void *argument)
{
  /* USER CODE BEGIN Cooling */
  /* Infinite loop */
  for(;;)
  {
		if (FLAG_HOMING_WITH_COOLING)
		{
	  	  	Cooling_Valve = 1;
		  	osDelay(HMI_COOLING_TIME_SET * 100);
		  	Cooling_Valve = 0;
		  	FLAG_HOMING_WITH_COOLING=0;
		}
    osDelay(10);
  }
  /* USER CODE END Cooling */
}

/* USER CODE BEGIN Header_repeatAutoCycleTask */
/**
* @brief Function implementing the repeatAutoCycle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_repeatAutoCycleTask */
void repeatAutoCycleTask(void *argument)
{
  /* USER CODE BEGIN repeatAutoCycleTask */
  /* Infinite loop */
  for(;;)
  {

	  if (FLAG_REPEAT_CYCLE)
	  {
		  // Clear Encoders
		 		reset_linear_encoders();

		 		Tip_1_Distance = 0;
		 		HOME_Tip_1_Energy = 0;
		 		Tip_1_Energy_Consumed = 0;
		 		Tip1_Collapse = 0;

		 		Tip_2_Distance = 0;
		 		HOME_Tip_2_Energy = 0;
		 		Tip_2_Energy_Consumed = 0;
		 		Tip2_Collapse = 0;

		 		Tip_3_Distance = 0;
		 		HOME_Tip_3_Energy = 0;
		 		Tip_3_Energy_Consumed = 0;
		 		Tip3_Collapse = 0;

		 		Tip_4_Distance = 0;
		 		HOME_Tip_4_Energy = 0;
		 		Tip_4_Energy_Consumed = 0;
		 		Tip4_Collapse = 0;

		 	  // Reset all tip energies
		 	  reset_tips(1,1,1,1);

		 	  // System Ready State
		 	  while( FLAG_REQUIRE_HOME == 0 && FLAG_HALT == 0 )
		 	  {
		 		 FLAG_IN_CYCLE=1;
		 		  // Check if heating distance reached
		 		 	  if ( (HMI_MANUAL_PLATEN_DISTANCE >= (PREHEAT_START_DISTANCE*HMI_TEST_PRESS_DOWN_POSITION)) && FLAG_WORK_POSTION_SAVED==1)
		 		 	  {
		 		 		  // Set Heating Started
		 		 		  FLAG_HEATING_STARTED=1;

		 		 		  // Set Energy Val to reach
		 		 		  tip1_energy_val_set = Tip_1_Energy_Val;

		 		 		  // Start Heating
		 		 		  FLAG_TIP1_HEATING_START=1;

		 		 		  // Set Energy Val to reach for Tip 2
		 		 		  tip2_energy_val_set = Tip_2_Energy_Val;

		 		 		  // Start Heating for Tip 2
		 		 		  FLAG_TIP2_HEATING_START = 1;

		 		 		  // Set Energy Val to reach for Tip 3
		 		 		  tip3_energy_val_set = Tip_3_Energy_Val;

		 		 		  // Start Heating for Tip 3
		 		 		  FLAG_TIP3_HEATING_START = 1;

		 		 		  // Set Energy Val to reach for Tip 4
		 		 		  tip4_energy_val_set = Tip_4_Energy_Val;

		 		 		  // Start Heating for Tip 3
		 		 		  FLAG_TIP4_HEATING_START = 1;

		 		 	  }

		 		 	  // Check if Platen has reached the work position
		 		 	  if ((HMI_MANUAL_PLATEN_DISTANCE >= HMI_TEST_PRESS_DOWN_POSITION ) && FLAG_WORK_POSTION_SAVED==1)
		 		 	  {
		 		 		  // Cycle Reached Proper position to succeed to Distance Calc
		 		 		 // success_cycle();
		 		 		  // Stop stepper and move to heating
		 		 		  break;

		 		 	  }

		 		 	  // Max Compression Reached By Tips. Stop Stepper
		 		 		if ( (fabs(Tip_1_Distance) > AUTO_CYCLE_STOP_HEIGHT)
		 		 			|| (fabs(Tip_2_Distance) > AUTO_CYCLE_STOP_HEIGHT)
		 		 			|| (fabs(Tip_3_Distance) > AUTO_CYCLE_STOP_HEIGHT)
		 		 			|| (fabs(Tip_4_Distance) > AUTO_CYCLE_STOP_HEIGHT))
		 		 		{
		 		 			  // Cycle Reached Proper position to succeed to Distance Calc
		 		 			  //success_cycle();
		 		 			break;
		 		 		}

		 				  if ((HMI_MANUAL_PLATEN_DISTANCE > (HMI_TEST_PRESS_DOWN_POSITION-5000)) && FLAG_WORK_POSTION_SAVED==1 )
		 				  {
		 					  stepper_setSpeed(CD_LOW);
		 				  }
		 				  else
		 				  {
		 					  stepper_setSpeed(CD_HIGH);
		 				  }

		 		 		stepper_Move_Down();
		 		 		osDelay(2);
		 	  }


		 	  while(FLAG_TIP1_HEATING_IN_PROGRESS==1 || FLAG_TIP2_HEATING_IN_PROGRESS==1 || FLAG_TIP3_HEATING_IN_PROGRESS==1 || FLAG_TIP4_HEATING_IN_PROGRESS==1)
		 	  {
		 		  osDelay(5);
		 	  }

		 	    abort_heating();

		 	  	Cooling_Valve = 1;
		 	  	osDelay(HMI_COOLING_TIME_SET * 100);

				double current_distance_cycle = HMI_MANUAL_PLATEN_DISTANCE;
				stepper_setSpeed(CU_LOW);
				while(HMI_MANUAL_PLATEN_DISTANCE > (current_distance_cycle - 1000) && (HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin)))
				{
					stepper_Move_Up();
					osDelay(2);
				}
				stepper_setSpeed(CU_HIGH);


		 	  	if (FLAG_GOING_HOME==0)
		 	  	{
			 	  	Go_Home_AutoCycle();
		 	  	}

		 	  	Cooling_Valve = 0;
		 	  	osDelay(REPEAT_CYCLE_DELAY);
		 	  	HAL_GPIO_WritePin(ARD_OUT_GPIO_Port, ARD_OUT_Pin,1);
		 	  	osDelay(200);
		 	  	HAL_GPIO_WritePin(ARD_OUT_GPIO_Port, ARD_OUT_Pin,0);
		 	  	FLAG_IN_CYCLE=0;
	  }

    osDelay(10);
  }
  /* USER CODE END repeatAutoCycleTask */
}

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x20000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0xC0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
	MX_I2C1_Init();

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
