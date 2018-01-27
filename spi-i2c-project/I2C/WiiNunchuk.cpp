#include "WiiNunchuk.h"


WiiNunchuk::WiiNunchuk(I2C_HandleTypeDef *handle)
{
     i2c = handle;

     //Defaults for everyone.
     accelerometerX = 0x00;
     accelerometerY = 0x00;
     accelerometerZ = 0x00;
     analogStickX = 0x00;
     analogStickY - 0x00;
     cKeyDown = false;
     zKeyDown = false;
}
    

//This function requires that the I2C bus be set up and read to transmit.
void WiiNunchuk::nunchuckInit()
{
    //Start by sending the proper I2C commands to the Nunchuck to communicate.

    //We start by disabling encryption on the data.
    HAL_I2C_Master_Transmit(i2c, NUNCHUK_ADDRESS, [0xF0, 0x55], 2, 100);

    //Now actually init the controller.
    HAL_I2C_Master_Transmit(i2c, NUNCHUK_ADDRESS, [0xFB, 0x00], 2, 100);
}

int16_t WiiNunchuk::GetAccelerometerX()
{

}
        
uint16_t WiiNunchuk::GetAccelerometerY()
{

}
        
uint16_t WiiNunchuk::GetAccelerometerZ()
{

}
        
uint8_t WiiNunchuk::GetAnalogStickX()
{

}
        
uint8_t WiiNunchukGetAnalogStickY()
{

}
        
bool WiiNunchuk::isCKeyDown()
{

}
        
bool WiiNunchuk::isZKeyDown()
{

}