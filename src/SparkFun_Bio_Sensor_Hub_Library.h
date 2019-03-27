#ifndef _SPARKFUN_BIO_SENSOR_HUB_LIBRARY_H
#define _SPARKFUN_BIO_SENSOR_HUB_LIBRARY_H

#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#define WRITE_FIFO_INPUT_BYTE 0x04
#define DISABLE 0x00
#define ENABLE 0x01
#define APP_MODE 0x00
#define BOOTLOADER_MODE 0x08
#define CMD_DELAY 60 //microseconds
#define NO_WRITE 0x00 
#define INCORR_PARAM 0xFF
const int BIO_ADDRESS = 0x55;

// The family registers are the largest 
enum FAMILY_REGISTER_BYTES {
  
  HUB_STATUS               = 0x00,
  SET_DEVICE_MODE,
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
  
  ENABLE_MAX86140 = 0x00,
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
  // Variables ------------
 
  // Constructor ----------
  SparkFun_Bio_Sensor_Hub(int address, uint8_t resetPin, uint8_t mfioPin ); 

  // Functions ------------
  
  // Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
  // The following function initializes the sensor. To place the MAX32664 into
  // application mode, the MFIO pin must be pulled HIGH while the board is held
  // in reset for 10ms. After 50 addtional ms have elapsed the board should be 
  // in application mode and will return two bytes, the first 0x00 is a 
  // successful communcation byte, followed by 0x00 which is the byte indicating 
  // which mode the IC is in. 
  uint8_t begin( TwoWire &wirePort = Wire);
  // Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
  // The following function puts the MAX32664 into bootloader mode. To place the MAX32664 into
  // bootloader mode, the MFIO pin must be pulled LOW while the board is held
  // in reset for 10ms. After 50 addtional ms have elapsed the board should be 
  // in bootloader mode and will return two bytes, the first 0x00 is a 
  // successful communcation byte, followed by 0x08 which is the byte indicating 
  // that the board is in bootloader mode. 
  bool beginBootloader( TwoWire &wirePort = Wire); 
  // Family Byte: SET_DEVICE_MODE (0x01), Index Byte: 0x01, Write Byte: 0x00
  // The following function is an alternate way to set the mode of the of
  // MAX32664. It can take three parameters: Enter and Exit Bootloader Mode, as
  // well as reset. 
  // INCOMPLETE
  uint8_t setOperatingMode(uint8_t selection); 
  // Family Byte: IDENTITY (0x01), Index Byte: READ_MCU_TYPE, Write Byte: NONE
  // The following function returns a byte that signifies the microcontoller that
  // is in communcation with your host microcontroller. Returns 0x00 for the
  // MAX32625 and 0x01 for the MAX32660/MAX32664. 
  // INCOMPLETE
  uint8_t getMCUtype(); 
  // Family Byte: BOOTLOADER_INFO (0x80), Index Byte: BOOTLOADER_VERS (0x00) 
  // This function checks the version number of the bootloader on the chip and
  // returns a four bytes: Major version Byte, Minor version Byte, Space Byte,
  // and the Revision Byte. 
  // INCOMPLETE
  long getBootloaderInf();

  // Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX86140 (0x00), Write
  // Byte: enable (parameter - 0x00 or 0x01). 
  // This function enables the MAX86140. 
  bool enableSensorMAX86140(uint8_t enable);
  // Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX30205 (0x01), Write
  // Byte: enable (parameter - 0x00 or 0x01). 
  // This function enables the MAX30205. 
  bool enableSensorMAX30205(uint8_t enable); 
  // Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX30001 (0x02), Write
  // Byte: enable (parameter - 0x00 or 0x01). 
  // This function enables the MAX30001. 
  bool enableSensorMAX30001(uint8_t enable);
  // Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX30101 (0x03), Write
  // Byte: enable (parameter - 0x00 or 0x01).
  // This function enables the MAX30101. 
  uint8_t enableSensorMAX30101(uint8_t enable);
  // Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_ACCELEROMETER (0x04), Write
  // Byte: enable (parameter - 0x00 or 0x01). 
  // This function enables the ACCELEROMETER. 
  bool enableSensorAccel(uint8_t enable);

  // Family Byte: OUTPUT_FORMAT (0x10), Index Byte: SET_FORMAT (0x00), 
  // Write Byte : outputType (Parameter values in OUTPUT_MODE_WRITE_BYTE)
  bool setOutputMode(uint8_t outputType);
  // Family Byte: OUTPUT_FORMAT, Index Byte: SET_THRESHOLD, Write byte: intThres
  // (parameter - value betwen 0 and 0xFF).
  // This function changes the threshold for the FIFO interrupt bit/pin. The
  // interrupt pin is the MFIO pin which is set to INPUT after IC initialization
  // (begin). 
  bool setFIFOThreshold(uint8_t intThresh);   
  // Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: NUM_SAMPLES (0x00), Write
  // Byte: NONE
  // This function returns the number of samples available in the FIFO. 
  // INCOMPLETE
  uint8_t numSamplesOutFIFO();
  // Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: READ_DATA (0x00), Write
  // Byte: NONE
  // This function returns the data in the FIFO. 
  // INCOMPLETE
  uint8_t getDataOutFIFO();
  // Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: READ_DATA (0x00), Write
  // Byte: NONE
  // This function adds support for the acceleromter that is NOT included on
  // SparkFun's product, The Family Registery of 0x13 and 0x14 is skipped for now. 
  uint8_t numSamplesExternalSensor();

  // Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_MAX86140 (0x00), Write Bytes:
  // Register Address and Register Value
  // This function writes the given register value at the given register address
  // for the MAX86140 and MAX86141 Sensor and returns a boolean indicating a successful 
  // or non-successful write.  
  bool writeRegisterMAX861X(uint8_t regAddr, uint8_t regVal); 
  // Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_MAX30205 (0x01), Write Bytes:
  // Register Address and Register Value
  // This function writes the given register value at the given register address
  // for the MAX30205 sensor and returns a boolean indicating a successful or
  // non-successful write. 
  bool writeRegisterMAX30205(uint8_t regAddr, uint8_t regVal);
  // Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_MAX30001 (0x02), Write Bytes:
  // Register Address and Register Value
  // This function writes the given register value at the given register address
  // for the MAX30001 sensor and returns a boolean indicating a successful or
  // non-successful write. 
  bool writeRegisterMAX30001(uint8_t regAddr, uint8_t regVal);
  // Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_MAX30101 (0x03), Write Bytes:
  // Register Address and Register Value
  // This function writes the given register value at the given register address
  // for the MAX30101 sensor and returns a boolean indicating a successful or
  // non-successful write. 
  bool writeRegisterMAX30101(uint8_t regAddr, uint8_t regVal); 
  // Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_ACCELEROMETER (0x04), Write Bytes:
  // Register Address and Register Value
  // This function writes the given register value at the given register address
  // for the Accelerometer and returns a boolean indicating a successful or
  // non-successful write. 
  bool writeRegisterAccel(uint8_t regAddr, uint8_t regVal);

  // Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX86140 (0x00), Write Byte: 
  // Register Address
  // This function reads the given register address for the MAX86140 and MAX8641
  // Sensors and returns the values at that register. 
  uint8_t readRegisterMAX8614X(uint8_t regAddr);
  // Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30205 (0x01), Write Byte: 
  // Register Address
  // This function reads the given register address for the MAX30205 Sensor and
  // returns the values at that register. 
  uint8_t readRegisterMAX30205(uint8_t regAddr);
  // Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30001 (0x02), Write Byte: 
  // Register Address
  // This function reads the given register address for the MAX30001 Sensor and
  // returns the values at that register. 
  uint8_t readRegisterMAX30001(uint8_t regAddr);
  // Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30101 (0x03), Write Byte: 
  // Register Address
  // This function reads the given register address for the MAX30101 Sensor and
  // returns the values at that register. 
  uint8_t readRegisterMAX30101(uint8_t regAddr);
  // Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30101 (0x03), Write Byte: 
  // Register Address
  // This function reads the given register address for the MAX30101 Sensor and
  // returns the values at that register. 
  uint8_t readRegisterAccel(uint8_t regAddr);

  private:   
  // Variables -----------
  uint8_t _resetPin;
  uint8_t _mfioPin;
  int _address; 
  
  // I-squared-C Class----
  TwoWire *_i2cPort;

  // Functions------------
  
  // This function uses the given family, index, and write byte to communicate
  // with the MAX32664 which in turn communicates with downward sensors. There
  // are two steps demonstrated in this function. First a write to the MCU
  // indicating what you want to do, a delay, and then a read to confirm positive
  // transmission. 
  uint8_t writeByte( uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte );
  // This function sends information to the MAX32664 to specifically write values
  // to the registers of downward sensors and so also requires a
  // register address and register value as parameters. Again there is the write
  // of the specific bytes followed by a read to confirm positive transmission. 
  uint8_t writeRegister( uint8_t _familyByte, uint8_t _indexByte, uint8_t _regAddr, uint8_t _regVal );
  // This function handles all read commands or stated another way, all information
  // requests. It starts a request by writing the family byte, index byte, and
  // delays 60 microseconds, during which the MAX32664 retrieves the requested 
  // information. An I-squared-C request is then issued, and the information is read.
  uint8_t * readByte( uint8_t _familyByte, uint8_t _indexByte, uint16_t _numOfReads ); 
  // This function is exactly as the one above except it accepts a Write Byte as
  // a paramter. It starts a request by writing the family byte, index byte, and
  // write byte to the MAX32664, delays 60 microseconds, during which
  // the MAX32664 retrieves the requested information. A I-squared-C request is
  // then issued, and the information is read.
  uint8_t * readByte( uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint16_t _numOfReads ); 
};

#endif
