# Controls Final Project
## Overview
Designed to control a fan using PWM that is set by a PID controller
## Functionality
The system maintians a desired temperature on a heating element by taking readings from a temerautre sensor and acting on them usign a PID controller. The desired temeprature is set using a potentiometer. The range for desired temepratures is set to 50-80 degrees C. The reference voltage for the potentiometer is 3.3 VDC such that 3.3 V corresponds to 50 degrees C and 0 V corresponds to 80 degrees C. The relationship is given as: desired temperature = 80 - (91 * potentiometer votlage).
## Use
The device is connected as shown below in the schematic. The desired and current temperature can be read through a serial connection using a program like RealTerm.
## Pins
 * P3.5 - PWM output from MSP430
 * P6.0 temperature sensor voltage reading
 * P6.1 potentiometer votlage reading
 ## Schematic
 <img src="schematic.png">
