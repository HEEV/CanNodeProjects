#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_i2c.h>

#ifndef _NUNCHUK_DRV_H
#define _NUNCHUK_DRV_H

class WiiNunchuk
{
public:
        WiiNunchuk(I2C_HandleTypeDef *handle);
        nunchuckInit();
        updateNunchuckData();
        uint16_t GetAccelerometerX();
        uint16_t GetAccelerometerY();
        uint16_t GetAccelerometerZ();
        uint8_t GetAnalogStickX();
        uint8_t GetAnalogStickY();
        bool isCKeyDown();
        bool isZKeyDown();

private:
        I2C_HandleTypeDef *i2c;
        const uint16_t NUNCHUK_ADDRESS = 0x52;

        uint16_t accelerometerX;
        uint16_t accelerometerY;
        uint16_t accelerometerZ;
        uint8_t  analogStickX;
        uint8_t  analogStickY;
        bool cKeyDown;
        bool zKeyDown;
};

#endif //_NUNCHUCK_DRV_H
