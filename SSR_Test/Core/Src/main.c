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
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t ADC_ConvertedValues[4]; // Array to store ADC converted values

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include <stdbool.h>
#include <math.h>

#define ADC_8BIT 8 // ADC bit depth
#define ADC_10BIT 10
#define ADC_12BIT 12
#define ADC_16BIT 16

#define BLR_ON 1 // baseline restore switch option
#define BLR_OFF 0

#define SGL_SCAN 1 // single scan mode
#define CNT_SCAN 0 // continuous scanning mode

#define AC_FREQ 60
const double TIP_1_PWM = 0.9;
const double TIP_2_PWM = 0.9;
const double TIP_3_PWM = 0.9;
const double TIP_4_PWM = 0.9;



const uint32_t period = 1000/AC_FREQ; // PWM period in milliseconds
uint32_t tip_1_onTime = TIP_1_PWM*period;
uint32_t tip_2_onTime = TIP_2_PWM*period;
uint32_t tip_3_onTime = TIP_3_PWM*period;
uint32_t tip_4_onTime = TIP_4_PWM*period;

typedef struct Rms {
    int instVal;
    float rmsVal;
    int dcBias;
    bool acquire;
    bool acqRdy;

    // private members
    bool blr; // baseline restoration switch
    bool mode;
    int rmsWindow;
    float alpha; // baseline restoration filter constant
    int error;
    short sampleIdx;
    float scaling;
    float scalingSq;
    float msVal;
    float sumInstVal;
    float temp_sumInstVal;
    float temp_sumSqInstVal;
    float sumSqInstVal;
} Rms;

void Rms_begin(Rms *self, float _range, unsigned char _rmsWindow, unsigned char _adcNob, bool _blr, bool _mode);
void Rms_start(Rms *self);
void Rms_stop(Rms *self);
void Rms_update(Rms *self, int _instVal);
void Rms_publish(Rms *self);

void Rms_begin(Rms *self, float _range, unsigned char _rmsWindow, unsigned char _adcNob, bool _blr, bool _mode) {
    self->mode = _mode;
    self->rmsWindow = _rmsWindow;
    self->blr = _blr;
    self->dcBias = pow(2, _adcNob) / 2; // dc-bias starting value
    self->scalingSq = (_range * _range) / (float)(self->rmsWindow * pow(2, 2 * _adcNob));
    self->temp_sumSqInstVal = 0;
    self->sampleIdx = 0;
    self->alpha = 0.7; // baseline restoration filter constant
}

void Rms_start(Rms *self) {
    self->acquire = true;
    self->acqRdy = false;
}

void Rms_stop(Rms *self) {
    self->acquire = false;
}

void Rms_update(Rms *self, int _instVal) {
    if (self->acquire) {
        if (self->blr) {
            self->instVal = _instVal - self->dcBias; // subtract DC-offset to restore the baseline.
            self->temp_sumInstVal += self->instVal;
        } else {
            self->instVal = _instVal;
        }
        self->temp_sumSqInstVal += (float)self->instVal * (float)self->instVal;
        if (self->sampleIdx == self->rmsWindow) {
            self->sumSqInstVal = self->temp_sumSqInstVal;
            if (self->blr) {
                self->sumInstVal = self->alpha * self->temp_sumInstVal + (1 - self->alpha) * self->sumInstVal; // calculate running average of sum instant values.
                self->error = (int)round(self->sumInstVal / self->rmsWindow); // calculate error of DC-bias
                self->dcBias = self->dcBias + self->error; // restore baseline
                self->temp_sumInstVal = 0;
                if (self->mode == SGL_SCAN) { //update status bits
                   self->acquire = false;
                self->acqRdy = true;
            }
        }
        self->temp_sumSqInstVal = 0;
        self->sampleIdx = 0;
    }
    self->sampleIdx++;
}
}

void Rms_publish(Rms *self) {
self->msVal = self->sumSqInstVal * self->scalingSq;
self->rmsVal = sqrt(self->msVal);
self->acqRdy = false;
}


#define LPERIOD 1000    // loop period time in us. In this case 1.0ms
#define ADC_INPUT 0     // define the used ADC input channel
//#define RMS_WINDOW 40   // rms window of 40 samples, means 2 periods @50Hz
#define RMS_WINDOW 50   // rms window of 50 samples, means 3 periods @60Hz

unsigned long nextLoop;
int adcVal;
int cnt=0;
float VoltRange = 5; // The full scale value is set to 5.00 Volts but can be changed when using an
                        // input scaling circuit in front of the ADC.

Rms readRms1;
Rms readRms2;
Rms readRms3;
Rms readRms4;

uint8_t received_data[9]={0,0,0,0,0,0,0};
uint8_t sent_data[11];
uint32_t overflow_count = 0; //variable to store overflow count
uint64_t microseconds = 0; //variable to store current microseconds
volatile uint8_t i2c_error_flag = 0;

uint64_t timeout = 500; // 500 milliseconds
uint64_t previous_time=0;

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
    htim2.Init.Period = 0xFFFF; //set timer period
    htim2.Init.Prescaler = 71; //set timer prescaler for microseconds
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
    microseconds = (uint64_t)overflow_count * (0xFFFF + 1) + __HAL_TIM_GET_COUNTER(&htim2); //update microseconds
    return microseconds;
}

void delayMicroseconds(uint64_t us)
{
	uint64_t _current_us = micros();
	while(micros() - _current_us < us );
}

uint64_t millis(void) //function to get current milliseconds
{
    return micros() / 1000;
}


#define I2C_TIMEOUT 100

void I2C_Error_Handler(I2C_HandleTypeDef *hi2c) {
    // Check for errors and clear flags
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF)) {
        // Clear the Acknowledge Failure Flag
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
    }

    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ARLO)) {
        // Clear the Arbitration Lost Flag
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ARLO);
    }

    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BERR)) {
        // Clear the Bus Error Flag
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BERR);
    }

    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_OVR)) {
        // Clear the Overrun/Underrun Flag
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_OVR);
    }

    // Reset the I2C peripheral
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);

    // Re-enable I2C listening after resetting the peripheral
    HAL_I2C_EnableListen_IT(hi2c);

    // Wait for the bus to become free
    while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) {
        HAL_Delay(1);
    }
}

#define MAX_CONSECUTIVE_ERRORS 3

uint8_t consecutive_errors = 0;


void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	HAL_StatusTypeDef status;

	if (TransferDirection == I2C_DIRECTION_TRANSMIT)
	{
		do {
			status = HAL_I2C_Slave_Sequential_Receive_IT(hi2c, received_data, 9, I2C_LAST_FRAME);
			if (status != HAL_OK) {
				// Receive error
				I2C_Error_Handler(&hi2c1);
			} else {
				// Update the states and timers after receiving new data
				previous_time = millis();
				consecutive_errors = 0;
			}
		} while (status != HAL_OK && consecutive_errors < MAX_CONSECUTIVE_ERRORS);
	}
	else
	{
		do {
			status = HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, sent_data, 11, I2C_LAST_FRAME);
			if (status != HAL_OK) {
				// Transmission error
				I2C_Error_Handler(&hi2c1);
			} else {
				consecutive_errors = 0;
			}
		} while (status != HAL_OK && consecutive_errors < MAX_CONSECUTIVE_ERRORS);
	}
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	// Re-enable I2C listening after a complete transaction
	HAL_I2C_EnableListen_IT(hi2c);
}

void no_color(void) {
    HAL_GPIO_WritePin(OUTPUT2_GPIO_Port, OUTPUT2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OUTPUT3_GPIO_Port, OUTPUT3_Pin, GPIO_PIN_RESET);
}

void red_color(void) {
    HAL_GPIO_WritePin(OUTPUT2_GPIO_Port, OUTPUT2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OUTPUT3_GPIO_Port, OUTPUT3_Pin, GPIO_PIN_SET);
}

void green_color(void) {
    HAL_GPIO_WritePin(OUTPUT2_GPIO_Port, OUTPUT2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OUTPUT3_GPIO_Port, OUTPUT3_Pin, GPIO_PIN_RESET);
}

void yellow_color(void) {
    HAL_GPIO_WritePin(OUTPUT2_GPIO_Port, OUTPUT2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OUTPUT3_GPIO_Port, OUTPUT3_Pin, GPIO_PIN_SET);
}

void set_led_color(uint8_t led_color) {
    switch (led_color) {
        case 0: // No color
            no_color();
            break;
        case 1: // Red color
        	green_color();
            break;
        case 2: // Green color
            red_color();
            break;
        case 3: // Yellow color
            yellow_color();
            break;
        default:
            break;
    }
}

uint8_t FLAG_TIP_1_ON=0;
uint32_t currentTime=0;


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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	  // configure for automatic base-line restoration and continuous scan mode:
	  Rms_begin(&readRms1, VoltRange, RMS_WINDOW, ADC_12BIT, BLR_ON, CNT_SCAN);
	  Rms_begin(&readRms2, VoltRange, RMS_WINDOW, ADC_12BIT, BLR_ON, CNT_SCAN);
	  Rms_begin(&readRms3, VoltRange, RMS_WINDOW, ADC_12BIT, BLR_ON, CNT_SCAN);
	  Rms_begin(&readRms4, VoltRange, RMS_WINDOW, ADC_12BIT, BLR_ON, CNT_SCAN);
	  Rms_start(&readRms1); //start measuring
	  Rms_start(&readRms2); //start measuring
	  Rms_start(&readRms3); //start measuring
	  Rms_start(&readRms4); //start measuring

	  nextLoop = micros() + LPERIOD; // Set the loop timer variable for the next loop interval.

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, ADC_ConvertedValues, 5); // start adc in DMA mode
  HAL_I2C_EnableListen_IT(&hi2c1);
  init_timer();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		// Inputs Reading
		uint8_t left_switch = HAL_GPIO_ReadPin(INPUT3_GPIO_Port, INPUT3_Pin);
		uint8_t right_switch = HAL_GPIO_ReadPin(INPUT3_GPIO_Port, INPUT3_Pin);
		uint8_t home_switch = HAL_GPIO_ReadPin(INPUT2_GPIO_Port, INPUT2_Pin);

		// Current Reading
		uint16_t current1 = ADC_ConvertedValues[0];
		uint16_t current2 = ADC_ConvertedValues[1];
		uint16_t current3 = ADC_ConvertedValues[2];
		uint16_t current4 = ADC_ConvertedValues[3];
		uint16_t pressure_sensor_val = ADC_ConvertedValues[4];

		print_string("Pressure: ");
		print_number(pressure_sensor_val);
		print_string("\r\n");

		uint16_t adcVal1 = current1;
		uint16_t adcVal2 = current2;
		uint16_t adcVal3 = current3;
		uint16_t adcVal4 = current4;

	    // Read instant value from ADC
		  Rms_update(&readRms1, adcVal1); // for BLR_ON or for DC(+AC) signals with BLR_OFF
		  Rms_update(&readRms2, adcVal2);
		  Rms_update(&readRms3, adcVal3);
		  Rms_update(&readRms4, adcVal4);
		  cnt++;

		  if(cnt >= 50) { // publish every 0.5s
		    Rms_publish(&readRms1);
		    Rms_publish(&readRms2);
		    Rms_publish(&readRms3);
		    Rms_publish(&readRms4);
		    adcVal1 = readRms1.rmsVal*1000;
		    adcVal2 = readRms2.rmsVal*1000;
		    adcVal3 = readRms3.rmsVal*1000;
		    adcVal4 = readRms4.rmsVal*1000;
		  }

		sent_data[0] = right_switch;
		sent_data[1] = left_switch;
		sent_data[2] = home_switch;

		// Store current1 values
		sent_data[3] = (uint8_t)(adcVal1 >> 8);
		sent_data[4] = (uint8_t)(adcVal1 & 0xFF);

		// Store current2 values
		sent_data[5] = (uint8_t)(adcVal2 >> 8);
		sent_data[6] = (uint8_t)(adcVal2 & 0xFF);

		// Store current3 values
		sent_data[7] = (uint8_t)(adcVal3 >> 8);
		sent_data[8] = (uint8_t)(adcVal3 & 0xFF);

		// Store current4 values
		sent_data[9] = (uint8_t)(adcVal4 >> 8);
		sent_data[10] = (uint8_t)(adcVal4 & 0xFF);


	  uint8_t _ssr1_state = received_data [0];
	  uint8_t _ssr2_state = received_data [1];
	  uint8_t _ssr3_state = received_data [2];
	  uint8_t _ssr4_state = received_data [3];

	  uint8_t _cooling_valve = received_data [4];
//	  uint8_t _output4_state = received_data [5];
//	  uint8_t _output5_state = received_data [6];
//	  uint8_t _output6_state = received_data [7];
	  uint8_t _led_color = received_data[8];

	  set_led_color(_led_color);

	  if (millis() - previous_time < timeout)
	  {

		 HAL_GPIO_WritePin(SSR1_GPIO_Port, SSR1_Pin,_ssr1_state);
		 HAL_GPIO_WritePin(SSR2_GPIO_Port, SSR2_Pin, _ssr2_state);
		 HAL_GPIO_WritePin(SSR3_GPIO_Port, SSR3_Pin, _ssr3_state);
		 HAL_GPIO_WritePin(SSR4_GPIO_Port, SSR4_Pin, _ssr4_state);
		 HAL_GPIO_WritePin(OUTPUT1_GPIO_Port, OUTPUT1_Pin, _cooling_valve);
	  }
	  else
	  {
		  // SSRS Control
		 HAL_GPIO_WritePin(SSR1_GPIO_Port, SSR1_Pin, 0);
		 HAL_GPIO_WritePin(SSR2_GPIO_Port, SSR2_Pin, 0);
		 HAL_GPIO_WritePin(SSR3_GPIO_Port, SSR3_Pin, 0);
		 HAL_GPIO_WritePin(SSR4_GPIO_Port, SSR4_Pin, 0);
		 HAL_GPIO_WritePin(OUTPUT1_GPIO_Port, OUTPUT1_Pin, 0);
	  }

	 // OUTPUTS Control
//	 HAL_GPIO_WritePin(OUTPUT1_GPIO_Port, OUTPUT1_Pin, right_switch);
//	 HAL_GPIO_WritePin(OUTPUT2_GPIO_Port, OUTPUT2_Pin, left_switch);

//	 HAL_GPIO_WritePin(OUTPUT4_GPIO_Port, OUTPUT4_Pin, _output4_state);
//	 HAL_GPIO_WritePin(OUTPUT5_GPIO_Port, OUTPUT5_Pin, _output5_state);
//	 HAL_GPIO_WritePin(OUTPUT6_GPIO_Port, OUTPUT6_Pin, _output6_state);

	  // wait until the end of the loop time interval
	  while(nextLoop > micros());

	  // set next loop time to current time + LOOP_PERIOD
	  nextLoop += LPERIOD;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.OwnAddress1 = 96;
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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SSR1_Pin|SSR2_Pin|SSR3_Pin|SSR4_Pin
                          |OUTPUT1_Pin|OUTPUT2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OUTPUT3_Pin|OUTPUT4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SSR1_Pin SSR2_Pin SSR3_Pin SSR4_Pin
                           OUTPUT1_Pin OUTPUT2_Pin */
  GPIO_InitStruct.Pin = SSR1_Pin|SSR2_Pin|SSR3_Pin|SSR4_Pin
                          |OUTPUT1_Pin|OUTPUT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT3_Pin OUTPUT4_Pin */
  GPIO_InitStruct.Pin = OUTPUT3_Pin|OUTPUT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT1_Pin INPUT2_Pin INPUT3_Pin INPUT4_Pin */
  GPIO_InitStruct.Pin = INPUT1_Pin|INPUT2_Pin|INPUT3_Pin|INPUT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	//MX_I2C1_Init();
//  while (1)
//  {
//  }
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
