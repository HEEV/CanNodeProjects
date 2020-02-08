# Include file for all the other makefiles

CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar
LD=arm-none-eabi-ld
OBJCOPY=arm-none-eabi-objcopy

# output directory
OUTPUT := lib

GENERATED = Inc
STM_LIB_SRC = Drivers/STM32F0xx_HAL_Driver
CMSIS_CORE = Drivers/CMSIS/Include
CMSIS_STM32 = Drivers/CMSIS/Device/ST/STM32F0xx
STM_USB_CORE = Middlewares/ST/STM32_USB_Device_Library/Core
STM_USB_CDC = Middlewares/ST/STM32_USB_Device_Library/Class/CDC

STARTUP := $(CMSIS_STM32)/Source/Templates/gcc/startup_stm32f042x6.s

STM_INCLUDE := -I../$(GENERATED)
STM_INCLUDE += -I../$(STM_LIB_SRC)/Inc
STM_INCLUDE += -I../$(CMSIS_CORE)
STM_INCLUDE += -I../$(CMSIS_STM32)/Include

UTIL_INCLUDE := -I../util/include
CAN_NODE_INC := -I../CanNode/include

PRJCFLAGS := -Os -g -Wall --std=gnu11 --specs=nosys.specs -mthumb -mcpu=cortex-m0
