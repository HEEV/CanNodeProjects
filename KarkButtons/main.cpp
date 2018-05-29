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
void buttonRTR(CanMessage *data);

/// Struct for initilizing ADC
ADC_HandleTypeDef hadc;
CAN_HandleTypeDef hcan;

CanNode *startNode;
CanNode *stopNode;
CanNode *hornNode;

#define STOP_MAX_V 3600
#define STOP_MIN_V 3200

#define HORN_MAX_V 2700
#define HORN_MIN_V 2500

#define START_MAX_V 3000
#define START_MIN_V 2750

/// global flag (set in \ref Src/usb_cdc_if.c) for whether USB is connected
volatile uint8_t USBConnected;

int main(void) {
  // setup globals
  USBConnected = false;

  //variables for measuring the pitot voltage
  uint16_t buttonVoltage = 0;
  int sampleNum = 0;
  uint32_t prevLoopTime = 0;
  uint32_t time = 0;

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
  CanNode start_node(START_SWITCH, buttonRTR);
  startNode = &start_node;

  // ammount of time the previous wheel revolution took
  CanNode stop_node(STOP_SWITCH, buttonRTR);
  stopNode = &stop_node;

  // wind speed sensor
  CanNode horn_node(HORN_SWITCH, buttonRTR);
  hornNode = &horn_node;

  while (1) {
    // check if there is a message necessary for CanNode functionality
    CanNode::checkForMessages();

    // get the current time
    prevLoopTime = time; 
    time = HAL_GetTick();

    if (time % 5 == 0){
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
      buttonVoltage = temp;
    }

    if (time % 100 == 0) {

        if (buttonVoltage > STOP_MIN_V && buttonVoltage < STOP_MAX_V) {
            stopNode->sendData_uint8(1);
        }
        else if (buttonVoltage > START_MIN_V && buttonVoltage < START_MAX_V) {
            startNode->sendData_uint8(1);
        }
        else if (buttonVoltage > HORN_MIN_V && buttonVoltage < HORN_MAX_V) {
           hornNode->sendData_uint8(1);
        }

    }


//USB Debugging
#ifdef USBDEBUG
    // USB handling code (send data out USB port)
    if (USBConnected && time % 499 == 0) {

      // NOTE: the maximum buffer length is set in the
      // USB code to be 64 bytes.
      char buff[48];
      char buff1[16];

      // setup the buffer with the required information
      // The data is sent in a CSV format like the following
      // RPS, Time per revolution in ms, ADC value
      itoa(buttonVoltage, buff, 10);
      strcat(buff, ", ");

      //itoa(wheelTime, buff1, 10);
      //strcat(buff, buff1);
      //strcat(buff, ", ");

      //itoa(pitotVoltage, buff1, 10);
      //strcat(buff, buff1);
      // send a break between data sets
      strcat(buff, "\n");

      CDC_Transmit_FS((uint8_t *)buff, strlen(buff));
      
    }
#endif

    // stuff to do every second
    if (time % 1000 == 0) {
      // blink heartbeat LED
      HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
    }

    //make sure that the same code does not run twice
    HAL_Delay(1);
  }
}

/// RTR handler for the RPS id

void buttonRTR(CanMessage *data) {
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
  //GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;

  //GPIO_InitStruct.Pin = IO2_Pin;
  //HAL_GPIO_Init(IO2_GPIO_Port, &GPIO_InitStruct);
}

void _Error_Handler(char*, int){

    HAL_GPIO_WritePin(GPIOB, LED2_Pin | LED1_Pin, GPIO_PIN_RESET);

}
