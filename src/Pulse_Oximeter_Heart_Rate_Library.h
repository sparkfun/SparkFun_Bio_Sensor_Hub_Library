/* 
  This is an Arduino Library written for the MAXIM 32664 Biometric Sensor Hub 
  The MAX32664 Biometric Sensor Hub is in actuality a small Cortex M4 microcontroller
  with pre-loaded firmware and algorithms used to interact with the a number of MAXIM
  sensors; specifically the MAX30101 Pulse Oximter and Heart Rate Monitor and
  the KX122 Accelerometer. With that in mind, this library is built to
  communicate with a middle-person and so has a unique method of communication
  (family, index, and write bytes). 

  SparkFun Electronics
  March, 2019
  Author: Elias Santistevan
 */
#ifndef _SPARKFUN_BIO_SENSOR_HUB_H
#define _SPARKFUN_BIO_SENSOR_HUB_H

#include <Wire.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

enum _FAMILY_REGISTERS {
  
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
  CHANGE_ALGORITHM_CONFIG  = 0x50,
  READ_ALGORITHM_CONFIG,
  ENABLE_ALGORITHM,
  BOOTLOADER_FLASH         = 0x80,
  BOOTLOADER_INFO,
  IDENTITY                 = 0xFF

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

enum OUTPUT_MODE_INDEX_BYTE {

  SET_FORMAT,
  SET_THRESHOLD

};

enum FIFO_OUTPUT_INDEX_BYTE {

  NUM_SAMPLES,
  READ_DATA

};

enum FIFO_EXTERNAL_INDEX_BYTE {

  SAMPLE_SIZE,
  READ_INPUT_DATA,
  READ_SENSOR_DATA,
  READ_NUM_SAMPLES_INPUT,
  READ_NUM_SAMPLES_SENSOR

};

enum WRITE_REGISTER_INDEX_BYTE {

  WRITE_MAX86140,
  WRITE_MAX30205,
  WRITE_MAX30001,
  WRITE_MAX30101,
  WRITE_ACCELEROMETER

};

enum READ_REGISTER_INDEX_BYTE {

  READ_MAX86140,
  READ_MAX30205,
  READ_MAX30001,
  READ_MAX30101,
  READ_ACCELEROMETER

};

enum GET_AFE_INDEX_BYTE {
  
  RETRIEVE_AFE_MAX86140,
  RETRIEVE_AFE_MAX30205,
  RETRIEVE_AFE_MAX30001,
  RETRIEVE_AFE_MAX30101,
  RETRIEVE_AFE_ACCELEROMETER

};

enum DUMP_REGISTER_INDEX_BYTE {
  
  DUMP_REGISTER_MAX86140,
  DUMP_REGISTER_MAX30205,
  DUMP_REGISTER_MAX30001,
  DUMP_REGISTER_MAX30101,
  DUMP_REGISTER_ACCELEROMETER

};

enum SENSOR_ENABLE_INDEX_BYTE {
  
  ENABLE_MAX86140,
  ENABLE_MAX30205,
  ENABLE_MAX30001,
  ENABLE_MAX30101,
  ENABLE_ACCELEROMETER

};

enum ALGORITHM_CONFIG_INDEX_BYTE {

  SET_TARG_PERC,
  SET_STEP_SIZE,
  SET_SENSITIVITY,
  SET_AVG_SAMPLES,
  SET_SAMPLE_WHRM,
  SET_WHRM_MAX_HEIGHT      = 0x02,
  SET_WHRM_MAX_WEIGHT      = 0x02,
  SET_WHRM_MAX_AGE         = 0x02,
  SET_WHRM_MIN_HEIGHT      = 0x02,
  SET_WHRM_MIN_WEIGHT      = 0x02,
  SET_WHRM_MIN_AGE         = 0x02,
  SET_WHRM_DEFAULT_HEIGHT  = 0x02,
  SET_WHRM_DEFAULT_WEIGHT  = 0x02,
  SET_WHRM_DEFAULT_AGE     = 0x02,
  SET_WHRM_BPM             = 0x02,
  SET_PULSE_OX_COEF        = 0x02,
  SET_EXPOSURE_CNTRL       = 0x02,
  SET_SKIN_CONTACT_DET     = 0x02,
  SET_PHOTO_DETECT         = 0x02,
  SET_SCD_DEBOUNCE         = 0x02,
  SET_WHRM_THRESH          = 0x02,
  SET_WHRM_MIN_PD          = 0x02,
  SET_WHRM_PPG             = 0x02,
  SET_BPT                  = 0x04,
  SET_BPT_DIASTOLIC        = 0x04,
  SET_BPT_SYSTOLIC         = 0x04,
  CALIBRATE_BPT            = 0x04,
  SET_BPT_EST_DATE         = 0x04,
  SET_BPT_REST             = 0x04,
  SET_BPT_SPO2             = 0x04,
  SET_WSPO2_COEF           = 0x05,
  SET_WSP02_SRATE          = 0x05,
  SET_WSP02_RUN            = 0x05,
  SET_WSP02_AGC            = 0x05,
  SET_WSP02_MOT_DETECT     = 0x05,
  SET_WSP02_DTCT_PER       = 0x05,
  SET_WSP02_THRESH         = 0x05,
  SET_WSP02_AGC_TOUT       = 0x05,
  SET_WSP02_ALG_TOUT       = 0x05,
  SET_WSP02_PPG_SIG        = 0x05,

};

// 0x51
enum READ_ALGORITHM_INDEX_BYTE {

  READ_AGC_PERCENTAGE      = 0x00,
  READ_AGC_STEP_SIZE       = 0x00,
  READ_AGC_SENSITIVITY     = 0x00,
  READ_AGC_NUM_SAMPLES     = 0x00,
  READ_WHRM_SAMPLE_RATE    = 0x02,
  READ_WHRM_MAX_HEIGHT     = 0x02,
  READ_WHRM_MAX_WEIGHT     = 0x02,
  READ_WHRM_MAX_AGE        = 0x02,
  READ_WHRM_MIN_HEIGHT     = 0x02,
  READ_WHRM_MIN_WEIGHT     = 0x02,
  READ_WHRM_MIN_AGE        = 0x02,
  READ_WHRM_DEF_HEIGHT     = 0x02,
  READ_WHRM_DEF_WEIGHT     = 0x02,
  READ_WHRM_DEF_AGE        = 0x02,
  READ_WHRM_INIT_HR        = 0x02,
  READ_MAX_FAST_COEF       = 0x02,
  READ_WHRM_AEC_EN         = 0x02,
  READ_WHRM_SCD_EN         = 0x02,
  READ_WHRM_PD_PRD         = 0x02,
  READ_WHRM_SCD_DEB        = 0x02,
  READ_WHRM_MOT_MAG        = 0x02,
  READ_WHRM_PD_MIN         = 0x02,
  READ_WHRM_PD_PPG         = 0x02,
  READ_WHRM_BPT_RESULTS    = 0x04,
  READ_WSP02_COEF          = 0x05,
  READ_WSP02_SAMP_RATE     = 0x05,
  READ_WSP02_RUN_MODE      = 0x05,
  READ_WSP02_AGC_STAT      = 0x05,
  READ_WSP02_MD_STAT       = 0x05,
  READ_WSP02_MD_PRD        = 0x05,
  READ_WSP02_MOT_THRESH    = 0x05,
  READ_WSP02_AGC_TO        = 0x05,
  READ_WSP02_ALGTHM_TO     = 0x05,
  READ_WSP02_PD_PPG        = 0x05

};

// 0x52
enum ALGORITHM_MODE_ENABLE_INDEX_BYTE {

  ENABLE_AGC_ALM           = 0x00;
  ENABLE_AEC_ALM,
  ENABLE_WHRM_ALM,
  ENABLE_ECG_ALM,
  ENABLE_BPT_ALM,
  ENABLE_WSP02_ALM

};

// 0x80
enum BOOTLOADER_FLASH_INDEX_BYTE {

  SET_INIT_VECTOR_BYTES    = 0x00;
  SET_AUTH_BYTES,
  SET_NUM_PAGES,
  ERASE_FLASH,
  SEND_PAGE_VALUE

};

// 0x81
enum BOOTLOADER_INFO_INDEX_BYTE {

  BOOTLOADER_VERS          = 0x00,
  PAGE_SIZE

};

// 0xFF
enum IDENTITY_INDEX_BYTES {

  READ_MCU_TYPE            = 0x00,
  READ_SENSOR_HUB_VERS     = 0x03,
  READ_ALM_VERS            = 0x07
};

typdef enum {

  WRITE_ADDRESS           = 0xAA,
  READ_ADDRESS            = 0xAB

}i2cAddress;

#define WRITE_FIFO_INPUT_BYTE 0x04

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

