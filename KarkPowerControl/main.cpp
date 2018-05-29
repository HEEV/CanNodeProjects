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

// includes for USB code
#ifdef USBDEBUG
#include <usb_device.h>
#include <usbd_cdc.h>
#include <usbd_cdc_if.h>
#endif

#define IO1_ADC ADC_CHANNEL_8
#define IO2_ADC ADC_CHANNEL_7

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC_Init(void);

/* Private function prototypes -----------------------------------------------*/
void pwrCtlRTR(CanMessage *data);
void starterHandler(CanMessage *data);
void stopHandler(CanMessage *data);
void killHandler(CanMessage *data);

#define LOCKOUT_TIME 500
#define STARTER_LOCKOUT -LOCKOUT_TIME
#define STARTER_MAX_TIME 2000
#define AUTO_TIME 1000
#define MAX_MESSAGE_TIME 175 // maximum time between messages before turning off

volatile int starterTimeout_g;
volatile bool starterLockout_g;
volatile int starterAutoStop_g;
volatile uint32_t prevMessageTime_g;

/// Struct for initilizing ADC
ADC_HandleTypeDef hadc;
CAN_HandleTypeDef hcan;
/// Starter motor enable node
CanNode *powerCtlNode;
/// EFI enable node

/// global flag (set in \ref Src/usb_cdc_if.c) for whether USB is connected
volatile uint8_t USBConnected;

int main(void) {
  uint32_t time = 0;
  uint32_t prevLoopTime = 0;
  // setup globals
  USBConnected = false;
  starterTimeout_g = STARTER_LOCKOUT;
  starterLockout_g = false;
  starterAutoStop_g = 0;
  prevMessageTime_g = 0;

  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();
  // Configure the system clock
  SystemClock_Config();

#ifdef USBDEBUG
  MX_USB_DEVICE_Init();
#endif

  // Initialize all configured peripherals
  MX_GPIO_Init();

  // setup CAN, ID's, and gives each an RTR callback

  // setup node for handling power control functions
  CanNode power_control(POWER_CTL, pwrCtlRTR);
  powerCtlNode = &power_control;

  bool filterRet;
  // setup handler functions
  filterRet = powerCtlNode->addFilter(START_SWITCH, starterHandler);
  filterRet = powerCtlNode->addFilter(STOP_SWITCH, stopHandler);
  powerCtlNode->addFilter(KILL_SWITCH, stopHandler);

  // blink the lights to show it's on
  HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin, GPIO_PIN_RESET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin, GPIO_PIN_RESET);

  while (1) {
    // get the current time
    prevLoopTime = time;
    time = HAL_GetTick();

    // check if there is a message necessary for CanNode functionality
    CanMessage msg;
    // if(is_can_msg_pending()) {
    //    can_rx(&msg, 5);
    //}
    CanNode::checkForMessages();

    starterTimeout_g -= time - prevLoopTime;
    if (starterTimeout_g < STARTER_LOCKOUT ||
        (time - prevMessageTime_g >= LOCKOUT_TIME && starterLockout_g)) {

      starterTimeout_g = STARTER_LOCKOUT;
      starterLockout_g = false;
    }
    else if ((time - prevMessageTime_g >= MAX_MESSAGE_TIME && !starterLockout_g) ||
        (starterTimeout_g < 0 && starterTimeout_g > -50)) {

      if (starterTimeout_g - AUTO_TIME < 0) {

          // Turn off Starter
          HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, GPIO_PIN_RESET);

          // start lockout phase
          if (starterTimeout_g > 0) {
            starterTimeout_g = 0;
          }
          // Lockout turning it on again for a while
          starterLockout_g = true;
          // clear starter LED
          HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
        }
    }

// USB Debugging
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
      itoa(starterTimeout_g, buff, 10);
      strcat(buff, ", ");

      itoa(time - prevMessageTime_g, buff1, 10);
      strcat(buff, buff1);
      strcat(buff, ", ");

      itoa(msg.id, buff1, 10);
      strcat(buff, buff1);
      // send a break between data sets
      strcat(buff, "\n");

      CDC_Transmit_FS((uint8_t *)buff, strlen(buff));
    }
#endif

    if (time % 30000 == 0) {
      // reset can every 30 seconds
      // can_init();
      // can_set_bitrate(CAN_BITRATE_500K);
      // can_enable();
    }

    // make sure that the same code does not run twice
    HAL_Delay(1);
  }
}

void pwrCtlRTR(CanMessage *data) {}

void starterHandler(CanMessage *data) {
  if (starterTimeout_g < 0 && !starterLockout_g) {
    // Turn on the starter motor for a certain ammount of time
    HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, GPIO_PIN_SET);

    // Turn on the EFI controller
    HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, GPIO_PIN_SET);

    starterTimeout_g = STARTER_MAX_TIME;

    // turn on both leds
    HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin, GPIO_PIN_SET);
  }
  if (!starterLockout_g) {
    prevMessageTime_g = HAL_GetTick();
  }
}

void stopHandler(CanMessage *data) {
  // Turn off the EFI controller
  HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, GPIO_PIN_RESET);

  // Make sure that the starter motor is off
  HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, GPIO_PIN_RESET);

  starterLockout_g = false;

  // Reset Both Leds
  HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin, GPIO_PIN_RESET);
}

// ----------------------- Automatically generated code
// -----------------------------------------
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
  HAL_GPIO_WritePin(GPIOB, LED2_Pin | LED1_Pin, GPIO_PIN_RESET);

  // Urbie's sensor is an open drain active-low sensor, so it needs a pullup
  // resistor and a
  // falling edge interrupt. It is on IO Pin 1
  // Sting's sensor is just a low active switch with a filter
  // configure IO1 as a rising edge interrupt pin
  // GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  // GPIO_InitStruct.Pull = GPIO_PULLUP;

  // Starter motor control on IO2
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = IO2_Pin;
  HAL_GPIO_Init(IO2_GPIO_Port, &GPIO_InitStruct);

  // EFI control on IO1
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = IO1_Pin;
  HAL_GPIO_Init(IO1_GPIO_Port, &GPIO_InitStruct);
}

void _Error_Handler(char *, int) {

  HAL_GPIO_WritePin(GPIOB, LED2_Pin | LED1_Pin, GPIO_PIN_RESET);
}
