#ifndef _SPARKFUN_POX_HRMON_H
#define _SPARKFUN_POX_HRMON_H

#include <Wire.h>
#include <Arduino.h>

enum SF_POX_HRMON_FAMILY_REGISTERS {
  
  HUB_STATUS               = 0x00,
  DEVICE_MODE,
  READ_DEVICE_MODE,
  OUTPUT_FORMAT            = 0x10,  
  READ_DATA_OUTPUT         = 0x12,
  READ_DATA_INPUT,
  WRITE_INPUT,
  WRITE_REGISTER           = 0x40, 
  READ_REGISTER,
  READ_ATTRIBUTES_AFE,
  DUMP_REGISTERS,
  ENABLE_SENSOR, 
  CHANGE_ALGORITHM_CONFIG = 0x50,
  READ_ALGORITHM_CONFIG,
  ENABLE_ALGORITHM,
  BOOTLOADER_FLASH        = 0x80,
  BOOTLOADER_INFO,
  IDENTITY                = 0xFF

};

enum READ_STATUS_BYTE_VALUE {

  SUCCESS                  = 0x00,
  ERR_UNAVAIL_CMD,
  ERR_UNAVAIL_FUNC,
  ERR_DATA_FORMAT,
  ERR_INPUT_VALUE,
  ERR_TRY_AGAIN,
  ERR_BTLDR_GENERAL        = 0x80,
  ERR_BTLDR_CHECKSUM       = 0x81,
  ERR_BTLDR_AUTH           = 0x82,
  ERR_BTLDR_INVALID_APP    = 0x83,
  ERR_UNKNOWN              = 0xFF

};

typdef enum {

  WRITE_ADDRESS           = 0xAA,
  READ_ADDRESS            = 0xAB

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

