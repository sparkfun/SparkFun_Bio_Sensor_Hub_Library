#ifndef _SPARKFUN_POX_HRMON_H
#define _SPARKFUN_POX_HRMON_H

#include <Wire.h>
#include <Arduino.h>

enum SF_POX_HRMON_FAMILY_REGISTERS {
  
  HUB_STATUS          = 0x00,
  DEVICE_MODE,
  READ_DEVICE_MODE,
  DATA_OUTPUT,
  INT_THRESHOLD,
  NUM_IN_OUT_FIFO,
  READ_OUT_FIFO,
  DUMP_REGISTERS      = 0x43,
  ENABLE_MAX30101     = 0x44, 
  SET_GAIN_CONTROL = 0x50,
  AGE, HEIGHT, WEIGHT"?"
  
};

typdef enum {

  POX_HRMON_WRITE_ADRESS = 0xAA,
  POX_HRMON_READ_ADRESS = 0xAB

}i2cAddress;


class SparkFun_Bio_Sensor_HUB
{
  Public:  
  // Variables

  Private:   
  // Variables 
  byte _address;
  byte _resetPin;
  byte _mfioPin;
};
#endif

