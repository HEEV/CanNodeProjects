# README #
Code for Sting's wheel speed sensor and pitot tube for communication
over CAN bus.

This code will transmit the time required for the last full wheel revolution (in mili-seconds) every 250ms. This can be used to find
the speed of the vehicle (upadted every quarter-second). This data is
published on the WHEEL_TIME ID (CAN-ID 1154 base-10). The data is
in a uint32_t format and is updated every 250ms.

Also published, is the number of wheel revolutions that were completed
in the last second. This information can be used to make a fairly accurate
estimate of the distance traveled by the vehicle by summing the values
given by this id. This data is published on the WHEEL_TACH ID (CAN-ID
1150 base-10). The data is in a uint16_t format and is updated every
1000ms.

Additionaly, the voltage from the Pitot pressure sensor is published
from this device. This value is the voltage (in mili-volts) produced
by the pitot pressure sensor. It can be used (after doing some math)
to produce the wind speed of the vehicle. This data is published on the 
PITOT ID (CAN-ID 950). The data is in a uint16_t format and is updated
every 250ms.

# Useful Hardware Notes
The pitot tube should be plugged into the IO1 port and the wheel speed
sensor into IO2