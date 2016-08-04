BMP280 Arduino Library
======================

This library uses I2C communication with Arduino/ESP8266 and read the Temperature( degC) , Pressure (mBar) and Altitude (m).


Pin Connection : 
======

BMP280-----Arduino

VDD   ----> 3.3V

GND   ----> GND

SDA   ----> PIN20 (arduino mega, changeable in begin) 

SCL   ----> PIN21 (arduino mega, changeable in begin)

SDO   ----> GND   (slave address 0x76)

CS    ----> VDD   (HIGH for I2C)

VDDIO ----> VDD


Instructions :
=============

Copy the BMP280 folder to Arduino/libraries

Restart Arduino and Upload "measurments" sketch in Arduino.




