/**
 * \file main.c
 * \brief main file for the CanNode project for Cedarville Supermileage.
 * Automatically generated by STCubeMX then hacked beyond recognition by
 * Samuel Ellicott
 *
 * \author Samuel Ellicott
 * \date 6-8-16
 */

#include <climits>
#include <cstdlib>
#include <cstring>
#include <stm32f0xx_hal.h>

// Uncomment next line to enable data via USB
//#define USBDEBUG

#include <CanNode.h>

//includes for USB code
#ifdef USBDEBUG
#include <usbd_cdc.h>
#include <usb_device.h>
#include <usbd_cdc_if.h>
#endif


#define IO1_ADC ADC_CHANNEL_8
#define IO2_ADC ADC_CHANNEL_7

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC_Init(void);

/* Private function prototypes -----------------------------------------------*/
void countRTR(CanMessage *data);
void timeRTR(CanMessage *data);
void pitotRTR(CanMessage *data);

/// Struct for initilizing ADC
ADC_HandleTypeDef hadc;
CAN_HandleTypeDef hcan;
/// Struct for transmitting the wheel RPS
CanNode *wheelCountNode;
/// Struct for transmitting the wheel revolution time
CanNode *wheelTimeNode;
/// Struct for transmitting the pitot tube voltage (in mili-volts)
CanNode *pitotNode;

/// varible for wheel RPS, modified by a ISR
volatile uint8_t wheelCount;
/// Varible for wheel revolution time, modified by a ISR
volatile uint32_t wheelTime;
/// Varible used to reset wheelTime if vehicle is stopped
volatile uint32_t wheelStartTime;
// Varible used to hold the pitot voltage in mv
uint16_t pitotVoltage;

/// Timeout for reseting wheelTime (~5s without pulse)
#define WHEEL_TIMEOUT 5000
/// Make the time as long as we can to indicate a stopped wheel
#define WHEEL_STOPPED 0xFFFFFFFF
/// global flag (set in \ref Src/usb_cdc_if.c) for whether USB is connected
volatile uint8_t USBConnected;

int main(void) {
  // setup globals
  wheelCount = 0;
  wheelTime = WHEEL_STOPPED;
  USBConnected = false;

  //variables for measuring the pitot voltage
  pitotVoltage = 0;
  const int NUM_SAMPLES = 50;
  uint16_t pitotVoltages[NUM_SAMPLES] = {0};
  int sampleNum = 0;
  uint32_t time = 0;
  uint32_t prev_time = 0;
  int8_t ms_cnt5 = 0;
  int16_t ms_cnt250 = 0;
  int16_t ms_cnt1000 = 0;


  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();
  // Configure the system clock
  SystemClock_Config();

#ifdef USBDEBUG
  MX_USB_DEVICE_Init();
#endif

  // Initialize all configured peripherals
  MX_GPIO_Init();

  MX_ADC_Init();

  // HAL_Delay(100);

  // setup CAN, ID's, and gives each an RTR callback
  
  // number of wheel revoltutions in one second
  CanNode wheel_tach(WHEEL_TACH, countRTR);
  wheelCountNode = &wheel_tach;


  // ammount of time the previous wheel revolution took
  CanNode wheel_time(WHEEL_TIME, timeRTR);
  wheelTimeNode = &wheel_time;

  // wind speed sensor
  CanNode pitot(PITOT, pitotRTR);
  pitotNode = &pitot;

  while (1) {
    // check if there is a message necessary for CanNode functionality
    CanNode::checkForMessages();

    prev_time = time;
    // get the current time
    time = HAL_GetTick();
    auto time_diff = time-prev_time;
    time_diff = (time_diff <=0) ? 1 : time_diff;
    ms_cnt5 += time_diff;
    ms_cnt250 += time_diff;
    ms_cnt1000 += time_diff;

    if (ms_cnt5 >= 5){
      //Sting has a pitot tube and Urbie does not
      // local varibles
      uint32_t temp;
      uint16_t adcVal;

      // read ADC value
      HAL_ADC_Start(&hadc);
      HAL_ADC_PollForConversion(&hadc, 5);
      adcVal = HAL_ADC_GetValue(&hadc);

      // do some math
      // voltage (in milivolts) = adc * 3300/4096
      // voltage (in milivolts) = adc * 825/1024
      temp = adcVal * 825; 
      // multiply by the value necessary to convert to 0-3.6V signal
      temp /= 1024; 
      pitotVoltages[sampleNum++] = (uint16_t) temp; // put into an integer number
      if (sampleNum >= NUM_SAMPLES) {
        sampleNum = 0;
      }
      ms_cnt5 = 0;
    }

    // stuff to do every quarter second
    if (ms_cnt250 >= 250) {

      //average data
      long temp = 0;
      for(auto n : pitotVoltages){
          temp += n;
      }
      pitotVoltage = temp/NUM_SAMPLES;
      // send the pitot voltage
      pitotNode->sendData_uint16(pitotVoltage);

      // send ammount of time per revolution
      wheelTimeNode->sendData_uint32((uint32_t) wheelTime);
      // We have sent the latest data, set to invalid data
      wheelTime = WHEEL_STOPPED;
      ms_cnt250 = 0;
    }

//USB Debugging
#ifdef USBDEBUG
    // USB handling code (send data out USB port)
    if (USBConnected && ms_cnt250 >= 250) {

      // NOTE: the maximum buffer length is set in the
      // USB code to be 64 bytes.
      char buff[48];
      char buff1[16];

      // setup the buffer with the required information
      // The data is sent in a CSV format like the following
      // RPS, Time per revolution in ms, ADC value
      itoa(wheelCount, buff, 10);
      strcat(buff, ", ");

      itoa(wheelTime, buff1, 10);
      strcat(buff, buff1);
      strcat(buff, ", ");

      itoa(pitotVoltage, buff1, 10);
      strcat(buff, buff1);
      // send a break between data sets
      strcat(buff, "\f");

      CDC_Transmit_FS((uint8_t *)buff, strlen(buff));
      ms_cnt250=0; 
    }
#endif

    // stuff to do every second
    if (ms_cnt1000 >= 1000) {
      // send RPS data
      wheelCountNode->sendData_uint16(wheelCount);

      // reset RPS varible
      wheelCount = 0;

      // blink heartbeat LED
      HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
      ms_cnt1000 = 0;
    }

    if (time % 30000 == 0){
        //reset can every 30 seconds
        //can_init();
        //can_set_bitrate(CAN_BITRATE_500K);
        //can_enable();

    }

    //make sure that the same code does not run twice
    HAL_Delay(1);
  }
}

/// RTR handler for the RPS id
void countRTR(CanMessage *data) {
  wheelCountNode->sendData_uint16(wheelCount);
}

/// RTR handler for the wheel revolution time id
void timeRTR(CanMessage *data) {
  wheelTimeNode->sendData_uint32(wheelTime);
}

void pitotRTR(CanMessage *data) {
  pitotNode->sendData_uint16(pitotVoltage);
}

/// callback for pin6 (IO1) interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  // when was our last pulse
  static uint32_t startTime = 0; //static so that this remains through function calls
  // what is our current time
  uint32_t newTime = HAL_GetTick();
  // how long did this revolution take
  uint32_t tempTime = newTime - startTime;

  /* If the new time is less than DELAY_TIME (ms) than it was a fluke
   */
  const int DELAY_TIME = 60;
  if (tempTime < DELAY_TIME) {
    return;
  }
  // set the new start time
  startTime = wheelStartTime = newTime;

  wheelTime = tempTime; // Valid pulse, save the value
  ++wheelCount;

  // toggle led
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

// ----------------------- Automatically generated code -----------------------------------------
/** System Clock Configuration
*/
void SystemClock_Config(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14 |
                                     RCC_OSCILLATORTYPE_HSI48 |
                                     RCC_OSCILLATORTYPE_LSI;

  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI48;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV6;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
void MX_ADC_Init(void) {
  ADC_ChannelConfTypeDef sConfig;

  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment
   * and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  HAL_ADC_Init(&hadc);

  /**Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = IO1_ADC;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
}

/** Configure pins as
 * Analog
 * Input
 * Output
.* EVENT_OUT
.* EXTI
.* Free pins are configured automatically as Analog (this feature is enabled
through
.* the Code Generation settings)
*/
void MX_GPIO_Init(void) {

  GPIO_InitTypeDef GPIO_InitStruct;

  // Make all unused pins analog to save power
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                     PA4 PA5 PA6 PA8
                     PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                        GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8 |
                        GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB11
                     PB12 PB13 PB14 PB15
                     PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 |
                        GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 |
                        GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOF_CLK_DISABLE();

  // Enable used io pins
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin | LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin | LED1_Pin, GPIO_PIN_SET);

  // Urbie's sensor is an open drain active-low sensor, so it needs a pullup
  // resistor and a
  // falling edge interrupt. It is on IO Pin 1
  // Sting's sensor is just a low active switch with a filter
  // configure IO1 as a rising edge interrupt pin
  // GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  // GPIO_InitStruct.Pull = GPIO_PULLUP;

  // for switch board outside cars
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;

  GPIO_InitStruct.Pin = IO2_Pin;
  HAL_GPIO_Init(IO2_GPIO_Port, &GPIO_InitStruct);

  // Sting's interrupt is on EXTI 7
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void _Error_Handler(char*, int){

    HAL_GPIO_WritePin(GPIOB, LED2_Pin | LED1_Pin, GPIO_PIN_RESET);

}
