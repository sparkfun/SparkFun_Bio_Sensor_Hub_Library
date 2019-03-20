#ifndef _SPARKFUN_BIO_SENSOR_HUB_LIBRARY_H
#define _SPARKFUN_BIO_SENSOR_HUB_LIBRARY_H

#include <Wire.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define WRITE_FIFO_INPUT_BYTE 0x04
#define DISABLE 0x00
#define ENABLE 0x01
#define APP_MODE 0x00
#define BOOTLOADER_MODE 0x08
#define CMD_DELAY 60 //microseconds
#define NO_WRITE 0x00 

// Read and write I-squared-C addresses
typedef enum {

  WRITE_ADDRESS           = 0xAA,
  READ_ADDRESS            = 0xAB

}i2cAddress;

// The family registers are the largest 
enum FAMILY_REGISTER_BYTES {
  
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

// Status Bytes are communicated back after every I-squared-C transmission.
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


// All the defines below are 1. Index Bytes nestled in the larger category of the
// family registry bytes listed above and 2. The Write Bytes associated with
// their Index Bytes.

// Family Registry Byte 0x01, Index Bye 0x00
enum DEVICE_MODE_WRITE_BYTES {

  EXIT_BOOTLOADER          = 0x00,
  RESET                    = 0x02,
  ENTER_BOOTLOADER         = 0x08

};

// Index Byte associated with Family Registry 0x10
enum OUTPUT_MODE_INDEX_BYTE {

  SET_FORMAT,
  SET_THRESHOLD

};

// Write Bytes associated with OUTPUT_MODE_INDEX_BYTE: SET_FORMAT
// 0x00. 
enum OUTPUT_MODE_WRITE_BYTE {

  PAUSE                    = 0x00,
  SENSOR_DATA,
  ALM_DATA,
  SENSOR_AND_ALGORITHM,
  PAUSE_TWO,
  SENSOR_COUNTER_BYTE,
  ALM_COUNTER_BYTE,
  SENSOR_ALM_COUNTER

};

// Index Byte associated with Family Registry Byte 0x12
enum FIFO_OUTPUT_INDEX_BYTE {

  NUM_SAMPLES,
  READ_DATA

};


// Index Byte associated with Family Registry Byte 0x13
enum FIFO_EXTERNAL_INDEX_BYTE {

  SAMPLE_SIZE,
  READ_INPUT_DATA,
  READ_SENSOR_DATA,
  READ_NUM_SAMPLES_INPUT,
  READ_NUM_SAMPLES_SENSOR

};

// Write Byte associated with FIFO_EXTERNAL_INDEX_BYTE:
// SAMPLE_SIZE, READ_SENSOR_DATA, and READ_NUM_SAMPLES_INPUT. 
enum FIFO_OUTPUT_WRITE_BYTE {

  ACCELEROMETER = 0x04 

};
// Index Byte associated with Family Registry Byte 0x40
enum WRITE_REGISTER_INDEX_BYTE {

  WRITE_MAX86140,
  WRITE_MAX30205,
  WRITE_MAX30001,
  WRITE_MAX30101,
  WRITE_ACCELEROMETER

};

// Write Bytes associated with Index Byte WRITE_REGISTER_INDEX_BYTE
enum WRITE_REGISTER_WRITE_BYTE {

  WRITE_MAX86140_ID,
  WRITE_MAX30205_ID,
  WRITE_MAX30001_ID,
  WRITE_MAX30101_ID,
  WRITE_ACCELEROMETER_ID

};

// Index Byte associated with Family Registry Byte 0x41
enum READ_REGISTER_INDEX_BYTE {

  READ_MAX86140,
  READ_MAX30205,
  READ_MAX30001,
  READ_MAX30101,
  READ_ACCELEROMETER

};

// Index Byte associated with Family Registry Byte 0x42
enum GET_AFE_INDEX_BYTE {
  
  RETRIEVE_AFE_MAX86140,
  RETRIEVE_AFE_MAX30205,
  RETRIEVE_AFE_MAX30001,
  RETRIEVE_AFE_MAX30101,
  RETRIEVE_AFE_ACCELEROMETER

};

// Index Byte associated with Family Registry Byte 0x43
enum DUMP_REGISTER_INDEX_BYTE {
  
  DUMP_REGISTER_MAX86140,
  DUMP_REGISTER_MAX30205,
  DUMP_REGISTER_MAX30001,
  DUMP_REGISTER_MAX30101,
  DUMP_REGISTER_ACCELEROMETER

};

// Index Byte associated with Family Registry Byte 0x44
enum SENSOR_ENABLE_INDEX_BYTE {
  
  ENABLE_MAX86140,
  ENABLE_MAX30205,
  ENABLE_MAX30001,
  ENABLE_MAX30101,
  ENABLE_ACCELEROMETER

};

// Index Byte associated with Family Registry Byte 0x50
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
  SET_WSP02_PPG_SIG        = 0x05

};

// Write Bytes associated with the ALGORITHM_CONFIG_INDEX_BYTE: SET_TARG_PERC
// BYTE
enum ALM_AGC_WRITE_BYTE {
  
  AGC_GAIN_ID              = 0x00, 
  AGC_SENSITIVITY_ID,
  AGC_NUM_SAMP_ID

};

// Write Bytes associated with the ALGORITHM_CONFIG_INDEX_BYTE: WHRM Bytes
// specifically.
enum ALM_WHRM_WRITE_BYTE {

  WHRM_SAMP_RATE_ID        = 0x00,
  WHRM_MAX_HEIGHT_ID,
  WHRM_MAX_WEIGHT_ID,
  WHRM_MAX_AGE_ID,
  WHRM_MIN_HEIGHT_ID,
  WHRM_MIN_WEIGHT_ID,
  WHRM_MIN_AGE_ID,
  WHRM_DEF_HEIGHT_ID,
  WHRM_DEF_WEIGHT_ID,
  WHRM_DEF_AGE_ID,
  MAXIMFAST_COEF_ID        = 0x0A,
  WHRM_AEC_ID,                       // Automatic Exposure Control
  WHRM_SCD_ID,                       // Skin Contact Detect
  WHRM_PD_ID,                        // Photo Detector
  WHRM_SCD_DEBOUNCE_ID,
  WHRM_MOTION_ID,
  WHRM_MIN_PD_ID           = 0x10,
  WHRM_PPG_PD_ID           = 0x11

};

// Write Bytes associated with the ALGORITHM_CONFIG_INDEX_BYTE: WHRM Bytes
enum ALM_BPT_WRITE_BYTE {

  BPT_BLOOD_PRESSURE_ID    = 0x00,
  BPT_DIASTOLIC_ID,
  BPT_SYSTOLIC_ID,
  BPT_CALIBRATE_ID,
  BPT_DATE_ID,
  BPT_RESTING_ID,
  BPT_SP02_COEF_ID

};

// Write Bytes associated with the ALGORITHM_CONFIG_INDEX_BYTE: WSPO2 Bytes
enum ALM_WSP02_WRITE_BYTE {

  WSP02_COEF_ID         = 0x00,
  WSP02_SAMPLE_RATE_ID,
  WSP02_RUN_MODE_ID,
  WSP02_MOT_DTCT_ID,
  WSP02_MOT_DTCT_PER_ID,
  WSP02_MOT_THRESH_ID,
  WSP02_AGC_TO_ID,
  WSP02_PD_CONFIG

};

// Index Byte associated with Family Registry Byte 0x51
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

// Write Bytes associated with the Index Byte: READ_ALGORITHM_INDEX_BYTE, AGC
// bytes.
enum READ_AGC_ALM_WRITE_BYTE {
  
  READ_AGC_ID              = 0x00,
  READ_AGC_STEP_SIZE_ID,
  READ_AGC_SENSITIVITY_ID,
  READ_AGC_NUM_SAMPLES_ID

};

// Write Bytes associated with the Index Byte: READ_ALGORITHM_INDEX_BYTE, WHRM
// bytes
enum READ_WHRM_ALM_WRITE_BYTE {

  READ_WHRM_SAMPLE_RATE_ID = 0x00,
  READ_WHRM_MAX_HEIGHT_ID,
  READ_WHRM_MAX_WEIGHT_ID,
  READ_WHRM_MAX_AGE_ID,
  READ_WHRM_MIN_HEIGHT_ID,
  READ_WHRM_MIN_WEIGHT_ID,
  READ_WHRM_MIN_AGE_ID,
  READ_WHRM_DEF_HEIGHT_ID,
  READ_WHRM_DEF_WEIGHT_ID,
  READ_WHRM_DEF_AGE_ID,
  READ_WHRM_INIT_HR_ID     = 0x0A,
  READ_MAX_FAST_COEF_ID,
  READ_WHRM_AEC_EN_ID,
  READ_WHRM_SCD_EN_ID,
  READ_WHRM_PD_PRD_ID,
  READ_WHRM_SCD_DEB_ID,
  READ_WHRM_MOT_MAG_ID,
  READ_WHRM_PD_MIN_ID         = 0x10,
  READ_WHRM_PD_PPG_ID
  // READ_WHRM_BPT_RESULTS = 0x03,
 
};

// Write Bytes associated with the Index Byte: READ_ALGORITHM_INDEX_BYTE, WSP02
// bytes
enum READ_WSP02_ALM_WRITE_BYTE {

  READ_WSP02_COEF_ID       = 0x00,
  READ_WSP02_SAMP_RATE_ID,
  READ_WSP02_RUN_MODE_ID,
  READ_WSP02_AGC_STAT_ID,
  READ_WSP02_MD_STAT_ID,
  READ_WSP02_MD_PRD_ID,
  READ_WSP02_MOT_THRESH_ID,
  READ_WSP02_AGC_TO_ID,
  READ_WSP02_ALGTHM_TO_ID,
  READ_WSP02_PD_PPG_ID

};

// Index Byte associated with Family Registry Byte 0x52
enum ALGORITHM_MODE_ENABLE_INDEX_BYTE {

  ENABLE_AGC_ALM           = 0x00,
  ENABLE_AEC_ALM,
  ENABLE_WHRM_ALM,
  ENABLE_ECG_ALM,
  ENABLE_BPT_ALM,
  ENABLE_WSP02_ALM

};

// Index Byte associated with Family Registry Byte 0x80
enum BOOTLOADER_FLASH_INDEX_BYTE {

  SET_INIT_VECTOR_BYTES    = 0x00,
  SET_AUTH_BYTES,
  SET_NUM_PAGES,
  ERASE_FLASH,
  SEND_PAGE_VALUE

};

// Index Byte associated with Family Registry Byte 0x81
enum BOOTLOADER_INFO_INDEX_BYTE {

  BOOTLOADER_VERS          = 0x00,
  PAGE_SIZE

};

// Index Byte associated with Family Registry Byte 0xFF
enum IDENTITY_INDEX_BYTES {

  READ_MCU_TYPE            = 0x00,
  READ_SENSOR_HUB_VERS     = 0x03,
  READ_ALM_VERS            = 0x07
};

class SparkFun_Bio_Sensor_Hub
{
  public:  
  // Variables
 
  // Constructor 
  SparkFun_Bio_Sensor_Hub( uint8_t resetPin, uint8_t mfioPin ); 

  // Functions
  bool begin( TwoWire &wirePort = Wire);
  bool beginBootloader( TwoWire &wirePort = Wire); 
  bool setDeviceMode( uint8_t boot_mode ); 
  bool setOutputMode(uint8_t outputType);
  bool setFIFOThreshold(uint8_t intThresh);   
  uint8_t numSamplesOutFIFO();
  uint8_t getDataOutFIFO();
  uint8_t numSamplesExternalSensor();
  bool writeRegisterMAX861X(uint8_t regAddr, uint8_t regVal); 
  bool writeRegisterMAX30205(uint8_t regAddr, uint8_t regVal);
  bool writeRegisterMAX30001(uint8_t regAddr, uint8_t regVal);
  bool writeRegisterMAX30101(uint8_t regAddr, uint8_t regVal); 
  bool writeRegisterAccel(uint8_t regAddr, uint8_t regVal);
  uint8_t readRegisterMAX8614X(uint8_t regAddr);
  uint8_t readRegisterMAX30205(uint8_t regAddr);
  uint8_t readRegisterMAX30001(uint8_t regAddr);
  uint8_t readRegisterMAX30101(uint8_t regAddr);
  uint8_t readRegisterAccel(uint8_t regAddr);

  private:   
  // Variables 
  byte _resetPin;
  byte _mfioPin;
  
  // I-squared-C Class
  TwoWire *_i2cPort;

  // Functions
  uint8_t readByte( uint8_t _familyByte, uint8_t _indexByte,uint8_t _writeByte, uint8_t numOfReads ); 
  uint8_t writeRegister(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _regAddr, uint8_t _regVal);
  uint8_t writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte);
};
#endif
