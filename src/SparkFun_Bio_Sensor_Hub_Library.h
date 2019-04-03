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

// Status Bytes are communicated back after every I-squared-C transmission and
// are indicators of success or failure of the previous transmission.
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

// The family register bytes are the larger umbrella for all the Index and
// Write Bytes listed below. You can not reference a nestled byte without first
// referencing it's larger category: Family Register Byte.
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

// All the defines below are: 1. Index Bytes nestled in the larger category of the
// family registry bytes listed above and 2. The Write Bytes nestled even
// farther under their Index Bytes.

// Write Bytes under Family Byte: SET_DEVICE_MODE (0x01) and Index
// Byte: 0x00. 
enum DEVICE_MODE_WRITE_BYTES {

  EXIT_BOOTLOADER          = 0x00,
  RESET                    = 0x02,
  ENTER_BOOTLOADER         = 0x08

};

// Index Byte under Family Byte: OUTPUT_FORMAT (0x10)
enum OUTPUT_MODE_INDEX_BYTE {

  SET_FORMAT,
  SET_THRESHOLD

};

// Write Bytes under Family Byte: OUTPUT_FORMAT (0x10) and Index byte: SET_FORMAT
// (0x00)
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

// Index Byte under the Family Byte: READ_DATA_OUTPUT (0x12)
enum FIFO_OUTPUT_INDEX_BYTE {

  NUM_SAMPLES,
  READ_DATA

};

// Index Byte under the Family Byte: READ_DATA_INPUT (0x13)
enum FIFO_EXTERNAL_INDEX_BYTE {

  SAMPLE_SIZE,
  READ_INPUT_DATA,
  READ_SENSOR_DATA,
  READ_NUM_SAMPLES_INPUT,
  READ_NUM_SAMPLES_SENSOR

};

// Write Byte under the Family Byte: READ_DATA_INPUT (0x13) and the Index Bytes: 
// FIFO_EXTERNAL_INDEX_BYTE.
enum FIFO_OUTPUT_WRITE_BYTE {

  ACCELEROMETER = 0x04 

};
// Index Byte under the Family Registry Byte: WRITE_REGISTER (0x40)
enum WRITE_REGISTER_INDEX_BYTE {

  WRITE_MAX86140,
  WRITE_MAX30205,
  WRITE_MAX30001,
  WRITE_MAX30101,
  WRITE_ACCELEROMETER

};

// Index Byte under the Family Registry Byte: READ_REGISTER (0x41)
enum READ_REGISTER_INDEX_BYTE {

  READ_MAX86140,
  READ_MAX30205,
  READ_MAX30001,
  READ_MAX30101,
  READ_ACCELEROMETER

};

// Index Byte under the Family Registry Byte: READ_ATTRIBUTES_AFE (0x42)
enum GET_AFE_INDEX_BYTE {
  
  RETRIEVE_AFE_MAX86140,
  RETRIEVE_AFE_MAX30205,
  RETRIEVE_AFE_MAX30001,
  RETRIEVE_AFE_MAX30101,
  RETRIEVE_AFE_ACCELEROMETER

};

// Index Byte under the Family Byte: DUMP_REGISTERS (0x43)
enum DUMP_REGISTER_INDEX_BYTE {
  
  DUMP_REGISTER_MAX86140,
  DUMP_REGISTER_MAX30205,
  DUMP_REGISTER_MAX30001,
  DUMP_REGISTER_MAX30101,
  DUMP_REGISTER_ACCELEROMETER

};

// Index Byte under the Family Byte: ENABLE_SENSOR (0x44)
enum SENSOR_ENABLE_INDEX_BYTE {
  
  ENABLE_MAX86140 = 0x00,
  ENABLE_MAX30205,
  ENABLE_MAX30001,
  ENABLE_MAX30101,
  ENABLE_ACCELEROMETER

};

// Index Byte under the Family Byte: CHANGE_ALGORITHM_CONFIG (0x50)
enum ALGORITHM_CONFIG_INDEX_BYTE {

  SET_TARG_PERC,
  SET_STEP_SIZE            = 0x00,
  SET_SENSITIVITY          = 0x00,
  SET_AVG_SAMPLES          = 0x00,
  SET_SAMPLE_WHRM          = 0x02,
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
  SET_BPT_MED              = 0x04,
  SET_BPT_DIASTOLIC        = 0x04,
  SET_BPT_SYSTOLIC         = 0x04,
  CALIBRATE_BPT            = 0x04,
  SET_BPT_EST_DATE         = 0x04,
  SET_BPT_REST             = 0x04,
  SET_BPT_SPO2_COEF        = 0x04,
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

// Write Bytes under the Family Byte: READ_ALGORITHM_CONFIG (0x51) and the
// Index Byte: ALGORITHM_CONFIG_INDEX_BYTE - SET_TARG_PERC
enum ALM_AGC_WRITE_BYTE {
  
  AGC_GAIN_ID              = 0x00, 
  AGC_STEP_SIZE_ID,
  AGC_SENSITIVITY_ID,
  AGC_NUM_SAMP_ID

};

// Write Bytes under the Family Byte: READ_ALGORITHM_CONFIG (0x51) and the
// Index Byte: ALGORITHM_CONFIG_INDEX_BYTE - WHRM 
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
  WHRM_BPM_INIT            = 0x0A,
  MAXIMFAST_COEF_ID,
  WHRM_AEC_ID              = 0x0B,    // Automatic Exposure Control
  WHRM_SCD_ID,                       // Skin Contact Detect
  WHRM_PD_ID,                        // Photo Detector
  WHRM_SCD_DEBOUNCE_ID,
  WHRM_MOTION_ID,
  WHRM_MIN_PD_ID           = 0x10,
  WHRM_PPG_PD_ID           = 0x11

};

// Write Bytes under the Family Byte: READ_ALGORITHM_CONFIG (0x51) and the
// Index Byte: ALGORITHM_CONFIG_INDEX_BYTE - BPT 
enum ALM_BPT_WRITE_BYTE {

  BPT_BLOOD_PRESSURE_ID    = 0x00,
  BPT_DIASTOLIC_ID,
  BPT_SYSTOLIC_ID,
  BPT_CALIBRATE_ID,
  BPT_DATE_ID,
  BPT_RESTING_ID,
  BPT_SP02_COEF_ID

};

// Write Bytes under the Family Byte: READ_ALGORITHM_CONFIG (0x51) and the
// Index Byte: ALGORITHM_CONFIG_INDEX_BYTE - WSPO2 
enum ALM_WSP02_WRITE_BYTE {

  WSP02_COEF_ID         = 0x00,
  WSP02_SAMPLE_RATE_ID,
  WSP02_RUN_MODE_ID,
  WSP02_AGC_MODE_ID,
  WSP02_MOT_DTCT_ID,
  WSP02_MOT_DTCT_PER_ID,
  WSP02_MOT_THRESH_ID,
  WSP02_AGC_TO_ID,
  WSP02_ALM_TO_ID,
  WSP02_PD_CONFIG

};

// Index Bytes under the Family Byte: READ_ALGORITHM_CONFIG (0x51)
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

// Write Bytes under the Family Byte: READ_ALGORITHM_CONFIG (0x51) and Index Byte: 
// READ_ALGORITHM_INDEX_BYTE - AGC
enum READ_AGC_ALM_WRITE_BYTE {
  
  READ_AGC_PERC_ID              = 0x00,
  READ_AGC_STEP_SIZE_ID,
  READ_AGC_SENSITIVITY_ID,
  READ_AGC_NUM_SAMPLES_ID

};

// Write Bytes under Family Byte: READ_ALGORITHM_CONFIG (0x51) and the Index Byte: 
// READ_ALGORITHM_INDEX_BYTE - WHRM
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
  READ_WHRM_AEC_EN_ID      = 0x0B,
  READ_WHRM_SCD_EN_ID,
  READ_WHRM_PD_PRD_ID,
  READ_WHRM_SCD_DEB_ID,
  READ_WHRM_MOT_MAG_ID,
  READ_WHRM_PD_MIN_ID      = 0x10,
  READ_WHRM_PD_PPG_ID,
  READ_WHRM_BPT_RESULTS_ID = 0x03
 
};

// Write Bytes under the Family Byte: READ_ALGORITHM_CONFIG (0x51) and the Index Byte: 
// READ_ALGORITHM_INDEX_BYTE
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

// Index Byte under the Family Byte: ENABLE_ALGORITHM (0x52).
enum ALGORITHM_MODE_ENABLE_INDEX_BYTE {

  ENABLE_AGC_ALM           = 0x00,
  ENABLE_AEC_ALM,
  ENABLE_WHRM_ALM,
  ENABLE_ECG_ALM,
  ENABLE_BPT_ALM,
  ENABLE_WSP02_ALM

};

// Index Byte under the Family Byte: BOOTLOADER_FLASH (0x80).
enum BOOTLOADER_FLASH_INDEX_BYTE {

  SET_INIT_VECTOR_BYTES    = 0x00,
  SET_AUTH_BYTES,
  SET_NUM_PAGES,
  ERASE_FLASH,
  SEND_PAGE_VALUE

};

// Index Byte under the Family Byte: BOOTLOADER_INFO (0x81).
enum BOOTLOADER_INFO_INDEX_BYTE {

  BOOTLOADER_VERS          = 0x00,
  PAGE_SIZE

};

// Index Byte under the Family Byte: IDENTITY (0xFF).
enum IDENTITY_INDEX_BYTES {

  READ_MCU_TYPE            = 0x00,
  READ_SENSOR_HUB_VERS     = 0x03,
  READ_ALM_VERS            = 0x07

};

class SparkFun_Bio_Sensor_Hub
{
  public:  
  // Variables ------------
  long readCoefArr[3];
  long writeCoefArr[3];

  struct version {
    byte major; 
    byte minor; 
    byte revision; 
  }; 

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
  // Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte: RETRIEVE_AFE_MAX86140 (0x00)
  // This function retrieves the attributes of the AFE (Analog Front End) of the
  // MAX8640/1 sensors. It returns the number of bytes in a word for the sensor
  // and the number of registers available. 
  // INCOMPLETE - must check datasheet of individual sensor to know how many
  // registers are returned. 
  uint8_t getAFEAttributesMAX86140();
  // Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte: RETRIEVE_AFE_MAX30205 (0x01)
  // This function retrieves the attributes of the AFE (Analog Front End) of the
  // MAX30205 sensor. It returns the number of bytes in a word for the sensor
  // and the number of registers available. 
  // INCOMPLETE - must check datasheet of individual sensor to know how many
  // registers are returned. 
  uint8_t getAFEAttributesMAX30205();
  // Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte: RETRIEVE_AFE_MAX30001 (0x02)
  // This function retrieves the attributes of the AFE (Analog Front End) of the
  // MAX30001 sensor. It returns the number of bytes in a word for the sensor
  // and the number of registers available. 
  // INCOMPLETE - must check datasheet of individual sensor to know how many
  // registers are returned. 
  uint8_t getAFEAttributesMAX30001();
  // Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte: RETRIEVE_AFE_MAX30101/ (0x03)
  // This function retrieves the attributes of the AFE (Analog Front End) of the
  // MAX30101 sensor. It returns the number of bytes in a word for the sensor
  // and the number of registers available. 
  // INCOMPLETE - must check datasheet of individual sensor to know how many
  // registers are returned. 
  uint8_t getAFEAttributesMAX30101();
  // Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte:
  // RETRIEVE_AFE_ACCELEROMETER (0x04)
  // This function retrieves the attributes of the AFE (Analog Front End) of the
  // Accelerometer. It returns the number of bytes in a word for the sensor
  // and the number of registers available. 
  // INCOMPLETE - must check datasheet of individual sensor to know how many
  // registers are returned. 
  uint8_t getAFEAttributesAccelerometer();
  // Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_MAX86140 (0x00)
  // This function returns all registers and register values sequentially of the
  // MAX86140/1 Sensors: register zero and register value zero to register n and 
  // register value n.
  // INCOMPLETE: Need to read datasheets to get exact amount of registers.
  uint8_t dumpRegisterMAX86140();
  // Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_MAX30205 (0x01)
  // This function returns all registers and register values sequentially of the
  // MAX30205 sensor: register zero and register value zero to register n and 
  // register value n.
  // INCOMPLETE: Need to read datasheets to get exact amount of registers.
  uint8_t dumpRegisterMAX30205();
  // Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_MAX30001 (0x02)
  // This function returns all registers and register values sequentially of the
  // MAX30001 sensor: register zero and register value zero to register n and 
  // register value n.
  // INCOMPLETE: Need to read datasheets to get exact amount of registers.
  uint8_t dumpRegisterMAX30001();
  // Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_MAX30101 (0x03)
  // This function returns all registers and register values sequentially of the
  // MAX30101 sensor: register zero and register value zero to register n and 
  // register value n.
  // INCOMPLETE: Need to read datasheets to get exact amount of registers.
  uint8_t dumpRegisterMAX30101();
  // Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_ACCELEROMETER (0x04)
  // This function returns all registers and register values sequentially of the
  // Accelerometer: register zero and register value zero to register n and 
  // register value n.
  // INCOMPLETE: Need to read datasheets to get exact amount of registers.
  uint8_t dumpRegisterAccelerometer();
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_TARG_PERC (0x00), Write Byte: AGC_GAIN_ID (0x00) 
  // This function sets the target percentage of the full-scale ADC range that
  // the automatic gain control algorithm uses. It takes a paramater of zero to 
  // 100 percent. 
  bool configALMrange(uint8_t perc);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_STEP_SIZE (0x00), Write Byte: AGC_STEP_SIZE_ID (0x01) 
  // This function changes the step size toward the target for the AGC algorithm. 
  // It takes a paramater of zero to 100 percent. 
  bool configALMStepSize(uint8_t step);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_SENSITIVITY (0x00), Write Byte: AGC_SENSITIVITY_ID (0x02)
  // This function changes the sensitivity of the AGC algorithm.
  bool configALMsensitivity(uint8_t sense);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_AVG_SAMPLES (0x00), Write Byte: AGC_NUM_SAMP_ID (0x03)
  // This function changes the number of samples that are averaged. 
  // It takes a paramater of zero to 255. 
  bool configALMsamples(uint8_t avg);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_SAMPLE_WHRM (0x02), Write Byte: WHRM_SAMP_RATE_ID (0x00)
  // This function sets the sample rate for the wrist heart rate monitor
  // (WHRM) algorithm. 
  bool configWHRMsampRate(uint16_t samp);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_WHRM_MAX_HEIGHT (0x02), Write Byte: WHRM_MAX_HEIGHT_ID (0x01)
  // This function sets the maximum height for the wrist heart rate monitor
  // (WHRM) algorithm. 
  bool configWHRMMaxHeight(uint16_t maxHeight);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_WHRM_MAX_WEIGHT (0x02), Write Byte: WHRM_MAX_WEIGHT_ID (0x02)
  // This function sets the maximum weight for the wrist heart rate monitor
  // (WHRM) algorithm. 
  bool configWHRMMaxWeight(uint16_t maxWeight);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_WHRM_MAX_AGE (0x02), Write Byte: WHRM_MAX_AGE_ID (0x03)
  // This function sets the maximum age for the wrist heart rate monitor
  // (WHRM) algorithm. 
  bool configWHRMMaxAge(uint8_t maxAge);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_WHRM_MIN_HEIGHT (0x02), Write Byte: WHRM_MIN_HEIGHT_ID (0x04)
  // This function sets the minimum height for the wrist heart rate monitor
  // (WHRM) algorithm. 
  bool configWHRMMinHeight(uint16_t minHeight);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_WHRM_MIN_HEIGHT (0x02), Write Byte: WHRM_MIN_HEIGHT_ID (0x04)
  // This function sets the minimum height for the wrist heart rate monitor
  // (WHRM) algorithm. 
  bool configWHRMMinWeight(uint16_t minWeight);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_WHRM_MIN_AGE (0x02), Write Byte: WHRM_MIN_AGE_ID (0x06)
  // This function sets the minimum age for the wrist heart rate monitor
  // (WHRM) algorithm. 
  bool configWHRMMinAge(uint8_t minAge);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_WHRM_DEFAULT_HEIGHT (0x02), Write Byte: WHRM_DEF_HEIGHT_ID (0x07)
  // This function sets the default height for the wrist heart rate monitor
  // (WHRM) algorithm. 
  bool configWHRMDefHeight(uint16_t defHeight);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_WHRM_DEFAULT_WEIGHT (0x02), Write Byte: WHRM_DEF_WEIGHT_ID (0x08)
  // This function sets the default weight for the wrist heart rate monitor
  // (WHRM) algorithm. 
  bool configWHRMDefWeight(uint16_t defWeight);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_WHRM_DEFAULT_AGE (0x02), Write Byte: WHRM_DEF_AGE_ID (0x09)
  // This function sets the default age for the wrist heart rate monitor
  // (WHRM) algorithm. 
  bool configWHRMDefAge(uint8_t defAge);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_WHRM_BPM (0x02), Write Byte: WHRM_BPM_INIT (0x0A)
  // This function sets the maximum age for the wrist heart rate monitor
  // (WHRM) algorithm. 
  bool configWHRMBPM(uint8_t bpm);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_PULSE_OX_COEF (0x02), Write Byte: MAXIMFAST_COEF_ID (0x0B)
  // This function takes three values that are used as the Sp02 coefficients.
  bool configWHRMCoef(long coef1, long coef2, long coef3);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: SET_EXPOSURE_CNTRL
  // (0x02), Write Byte: WHRM_AEC_ID (0x0B)
  // This function enables or disables automatic exposure control (AEC). The
  // function takes the parameter zero for disable and one for enable. 
  bool enableAutoExpCont(uint8_t enable);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: 
  // SET_SKIN_CONTACT_DET (0x02), Write Byte: WHRM_SCD_ID (0x0C)
  // This function enables or disables skin contact detection. The
  // function takes the parameter zero for disable and one for enable. 
  bool enableSkinDetect(uint8_t enable);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: 
  // SET_PHOTO_DETECT (0x02), Write Byte: WHRM_PD_ID (0x0D)
  // This function sets target photo detector current period in seconds.
  bool adjustPhotoDet(uint16_t per);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: 
  // SET_SCD_DEBOUNCE (0x02), Write Byte: WHRM_SCD_DEBOUNCE_ID (0x0E)
  // This function sets the skin contract detect debounce window. It's not clear
  // if this is in seconds or not in the datasheet.
  bool setSCDWindow(uint16_t time);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: 
  // SET_WHRM_THRESH (0x02), Write Byte: WHRM_MOTION_ID (0x0F)
  // This function sets motion magnitude threshold in 0.1g
  bool setMotionMag(uint16_t mag);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: 
  // SET_MIN_PD (0x02), Write Byte: WHRM_MIN_PD_ID (0x10)
  // This function changes the minimum photodetector currrent in 0.1mA
  // increments. 
  bool changePDCurrent(uint16_t curr);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: 
  // SET_WHRM_PPG (0x02), Write Byte: WHRM_PPG_PD_ID (0x11)
  // This function changes the source of the photoplethysmography (PPG) signal for 
  // the photodetector (PD). The paramater "pd" accepts one of three values: zero - PD1, 
  // one - PD2, and three - PD1 and PD2.
  bool changePPGSource(uint8_t pd);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: 
  // SET_BPT_MED (0x04), Write Byte: BPT_BLOOD_PRESSURE_ID (0x00)
  // The function configure the blood pressure trending (BPT) algorithm for
  // the users that are on blood pressure medicine. The parameter accepts the
  // value of zero (not using) or one (using). 
  bool bptMedicine(uint8_t onbpm);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: 
  // SET_BPT_DIASTOLIC (0x04), Write Byte: BPT_DIASTOLIC_ID (0x01)
  // This funciton writes the three givin diastolic BP byte values needed by the
  // calibration procedure.  
  bool setDiastolicVal(uint8_t val1, uint8_t val2, uint8_t val3);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: 
  // SET_BPT_SYSTOLIC (0x04), Write Byte: BPT_SYSTOLIC_ID (0x02)
  // This funciton writes the three givin systolic BP byte values needed by the
  // calibration procedure.  
  bool setSystolicVal(uint8_t val1, uint8_t val2, uint8_t val3);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: SET_BPT_EST_DATE
  // (0x04), Write Byte: BPT_DATE_ID (0x04)
  // This function sets the estimation date with the given month/day integer. 
  bool setBPTEstimationDate(uint16_t monthDay);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: SET_BPT_REST
  // (0x04), Write Byte: BPT_RESTING_ID (0x05)
  // This function adjusts the blood pressure trending algorithm for a user that
  // is resting (zero) or not resting (one). 
  bool setUserResting(uint8_t resting);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: SET_BPT_SPO2_COEF
  // (0x04), Write Byte: BPT_SP02_COEF_ID (0x06)
  // This function sets the given Sp02 coefficients for the blood pressure trending
  // algorithm. 
  bool adjustBPTcoef(long spCoef1, long spCoef2, long spCoef3 );
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: SET_WSPO2_COEF
  // (0x05), Write Byte: WSP02_COEF_ID (0x00)
  // This function sets the given wrist Sp02 (WSp02) coefficients for WSp02
  // algorithm. Defaults are in order: 159584, -3465966, and 11268987. 
  bool adjustWSP02Coef(long wspCoef1, long wspCoef2, long wspCoef3 );
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: SET_WSP02_SRATE
  // (0x05), Write Byte: WSP02_SAMPLE_RATE_ID (0x01)
  // This function changes the wrist Sp02 sample rate to 100Hz (zero) or 25Hz
  // (one).
  bool changeWSP02SampRate(uint8_t rate);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: SET_WSP02_RUN
  // (0x05), Write Byte: WSP02_RUN_MODE_ID (0x02)
  // This function changes the writs Sp02 algorithm run mode from continuous
  // (zero), from/to one-shot (one).
  bool changeWSP02RunMode(uint8_t mode);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: SET_WSP02_AGC
  // (0x05), Write Byte: WSP02_AGC_MODE_ID (0x03)
  // This function changes the wrist Sp02 algorithm's AGC mode. You can disable
  // it (zero) or enable it (one). 
  bool changeWSP02AGCMode(uint8_t enable);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_WSP02_MOT_DETECT (0x05), Write Byte: WSP02_MOT_DTCT_ID (0x04)
  // This function enables (one) or disables (zero) motion detect.
  bool enableWSP02MotDet(uint8_t enable);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_WSP02_DTCT_PER (0x05), Write Byte: WSP02_MOT_DTCT_PER_ID (0x05)
  // This function changes the period of the motion detect and though the
  // datasheet does not specify, I assume is in seconds. 
  bool enableWSP02MotDetPer(uint16_t detPer);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
  // SET_WSP02_THRESH (0x05), Write Byte: WSP02_MOT_THRESH_ID (0x06)
  // This function changes the motion threshold for the WSp02 algorithm. The
  // given number is multiplied by 100,000. 
  bool setWSP02MotThresh(long threshVal);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: SET_WSP02_AGC_TOUT
  // (0x05), Write Byte: WSP02_AGC_TO_ID (0x07)
  // This function changes the timeout period of the wrist Sp02 AGC algorithm. The
  // paramter should be given in seconds. 
  bool setWSP02AGCTimeout(uint8_t toVal);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: SET_WSP02_ALG_TOUT
  // (0x05), Write Byte: WSP02_ALM_TO_ID (0x08)
  // This function changes the timeout period of the wrist Sp02 algorithm. The
  // paramter should be given in seconds. 
  bool setWSP02ALMTimeout(uint8_t toVal);
  // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: SET_WSP02_PPG_SIG
  // (0x05), Write Byte: WSP02_PD_CONFIG (0x09)
  // This function changes the source of the photoplethysmographic source for the wrist Sp02 algorithm.
  // The parameter choses the photodetector to use: PD1 (0x01) or PD2 (0x02). 
  bool setWSP02PPGSource(uint8_t pd);
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_AGC_PERCENTAGE (0x00), Write Byte: READ_AGC_PERC_ID (0x00) 
  // This function reads and returns the currently set target percentage 
  // of the full-scale ADC range that the Automatic Gain Control algorithm is using. 
  uint8_t readALMrange();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_AGC_STEP_SIZE (0x00), Write Byte: READ_AGC_STEP_SIZE_ID (0x01) 
  // This function returns the step size toward the target for the AGC algorithm. 
  // It returns a value between zero and 100 percent. 
  uint8_t readALMStepSize();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_AGC_SENSITIVITY_ID (0x00), Write Byte: READ_AGC_SENSITIVITY_ID (0x02)
  // This function returns the sensitivity (percentage) of the automatic gain control. 
  uint8_t readALMsensitivity();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_AGC_NUM_SAMPLES (0x00), Write Byte: READ_AGC_NUM_SAMPlES_ID (0x03)
  // This function changes the number of samples that are averaged. 
  // It takes a paramater of zero to 255. 
  uint8_t readALMsamples();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_WHRM_SAMPLE_RATE (0x02), Write Byte: READ_WHRM_SAMPLE_RATE_ID (0x00)
  // This function reads the sample rate for the wrist heart rate monitor
  // (WHRM) algorithm. 
  uint8_t readWHRMsampRate();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_WHRM_MAX_HEIGHT (0x02), Write Byte: READ_WHRM_MAX_HEIGHT_ID (0x01)
  // This function reads the maximum height for the wrist heart rate monitor
  // (WHRM) algorithm. 
  uint16_t readWHRMMaxHeight();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_WHRM_MAX_WEIGHT (0x02), Write Byte: READ_WHRM_MAX_WEIGHT_ID (0x02)
  // This function reads the maximum weight for the wrist heart rate monitor
  // (WHRM) algorithm. 
  uint16_t readWHRMMaxWeight();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_WHRM_MAX_AGE (0x02), Write Byte: READ_MAX_AGE_ID (0x03)
  // This function reads the maximum age for the wrist heart rate monitor
  // (WHRM) algorithm. 
  uint16_t readWHRMMaxAge();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_WHRM_MIN_HEIGHT (0x02), Write Byte: READ_WHRM_MIN_HEIGHT_ID (0x04)
  // This function reads the minimum height for the wrist heart rate monitor
  // (WHRM) algorithm. 
  uint16_t readWHRMMinHeight();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_WHRM_MIN_WEIGHT (0x02), Write Byte: READ_WHRM_MIN_WEIGHT_ID (0x05)
  // This function reads the minimum weight for the wrist heart rate monitor
  // (WHRM) algorithm. 
  uint16_t readWHRMMinWeight();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_WHRM_MIN_AGE (0x02), Write Byte: READ_WHRM_MIN_AGE_ID (0x06)
  // This function reads the minimum age for the wrist heart rate monitor
  // (WHRM) algorithm. 
  uint8_t readWHRMMinAge();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_WHRM_DEFAULT_HEIGHT (0x02), Write Byte: READ_WHRM_DEF_HEIGHT_ID (0x07)
  // This function reads the default height for the wrist heart rate monitor
  // (WHRM) algorithm. 
  uint16_t readWHRMDefHeight();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_WHRM_DEFAULT_WEIGHT (0x02), Write Byte: READ_WHRM_DEF_WEIGHT_ID (0x08)
  // This function reads the default weight for the wrist heart rate monitor
  // (WHRM) algorithm. 
  uint16_t readWHRMDefWeight();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_WHRM_DEFAULT_AGE (0x02), Write Byte: READ_WHRM_DEF_AGE_ID (0x09)
  // This function returns the default age for the wrist heart rate monitor
  // (WHRM) algorithm. 
  uint8_t readWHRMDefAge();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_WHRM_INIT_HR (0x02), Write Byte: READ_WHRM_INIT_HR_ID (0x0A)
  // This function reads the maximum age for the wrist heart rate monitor
  // (WHRM) algorithm. 
  uint8_t readWHRMBPM();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_MAX_FAST_COEF (0x02), Write Byte: READ_MAX_FAST_COEF_ID (0x0B)
  // This function reads the maximum age for the wrist heart rate monitor
  // (WHRM) algorithm. It returns three long integers that are 
  // multiplied by 100,000.
  // INCOMPLETE
  long * readWHRMCoef();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: 
  // READ_WHRM_AEC_EN (0x02), Write Byte: READ_WHRM_AEC_EN_ID (0x0B)
  // This function reads whether or not the automatic exposure control(AEC) is
  // disabled (zero) or enabled (one). 
  uint8_t readAutoExpCont();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: 
  // READ_WHRM_SCD_EN (0x02), Write Byte: READ_WHRM_SCD_EN_ID (0x0C)
  // This function reads wehther or not the skin contact detection is enabled or
  // disabled. 
  uint8_t readSkinDetect();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: 
  // READ_WHRM_PD_PRD (0x02), Write Byte: READ_WHRM_PD_PRD_ID (0x0D)
  // This function reads the current period of the photo detector in seconds.
  uint16_t readPhotoDetPer();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: 
  // READ_WHRM_SCD_DEB (0x02), Write Byte: READ_WHRM_SCD_DEB_ID (0x0E)
  // This function reads the skin contract detect debounce window. It's not clear
  // if this is in seconds according to the datasheet. 
  uint16_t readSCDWindow();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: 
  // READ_WHRM_MOT_MAG (0x02), Write Byte: READ_WHRM_MOT_MAG_ID (0x0F)
  // This function reads the motion magnitude threshold in 0.1g increments. 
  uint16_t readMotionMag();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: 
  // READ_WHRM_PD_MIN (0x02), Write Byte: READ_WHRM_PD_MIN_ID (0x10)
  // This function reads the set minimum photodetector currrent in 0.1mA.
  uint16_t readPDCurrent();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: 
  // READ_WHRM_PD_PPG (0x02), Write Byte: READ_WHRM_PD_PPG_ID (0x11)
  // This function reads the current source of the photoplethysmography (PPG) signal for 
  // the photodetector (PD). It will return one of three values: zero - PD1, 
  // one - PD2, and three - PD1 and PD2.
  uint8_t readPPGSource();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: 
  // READ_WSP02_COEF (0x05), Write Byte: READ_WSP02_COEF_ID (0x00)
  // This function reads the coefficiencts used for the WSP02 algorithm. It
  // returns the three long integers that are multiplied by 100,000 that are used
  // as teh coefficients. 
  // INCOMPLETE
  long * readWSP02SampRate();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: 
  // READ_WSP02_RUN_MODE (0x05), Write Byte: READ_WSP02_RUN_MODE_ID (0x02)
  // This function returns the run mode of the WSP02 algorithm: zer0- continuous
  // or one - One-shot. 
  uint8_t readWSP02RunMode();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: 
  // READ_WSP02_AGC_STAT (0x05), Write Byte: READ_WSP02_AGC_STAT_ID (0x03)
  // This function reads whether AGC mode is enabled or disabled. 
  uint8_t readAGCmode();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: 
  // READ_WSP02_MD_STAT (0x05), Write Byte: READ_WSP02_MD_STAT_ID (0x04)
  // This function checks whether motion detection is enable (one) or not (zero). 
  uint8_t readMotionDetect();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: 
  // READ_WSP02_MD_PRD (0x05), Write Byte: READ_WSP02_MD_PRD (0x05)
  // This function reads the motion detection period in seconds. 
  uint16_t readMotionDetecPer();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: READ_WSP02_MOT_THRESH
  // (0x05), Write Byte: READ_WSP02_MOT_THRESH (0x06)
  // This function reads the long integer that is the motion threshold times
  // 100,000. 
  long readMotThresh();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: READ_WSP02_AGC_TO
  // (0x05), Write Byte: READ_WSP02_AGC_TO_ID (0x07)
  // This function reads the time out period of the AGC for the WSp02 Algorithm. 
  uint8_t readWSP02AGCTimeOut();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: READ_WSP02_ALGTHM_TO 
  // (0x05), Write Byte: READ_WSP02_ALGTHM_TO_ID (0x08)
  // This function returns the timeout period of the WSp02 Algorithm.
  uint8_t readWSP02AlgTimeOut();
  // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte: READ_WSP02_PD_PPG
  // (0x05), Write Byte: READ_WSP02_PD_PPG_ID (0x03)
  // This function reads the source of the photoplethysmogorphy: 0x01 = PD1 or
  // 0x02 = PD2.  
  uint8_t readWSP02PPGSource();
  // Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
  // ENABLE_AGC_ALM (0x00)
  // This function enables (one) or disables (zero) the automatic gain control algorithm. 
  bool enableAGCalgorithm(uint8_t enable);
  // Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
  // ENABLE_AEC_ALM (0x01)
  // This function enables (one) or disables (zero) the automatic exposure
  // control (AEC) algorithm.
  bool enableAECAlgorithm(uint8_t enable);
  // Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
  // ENABLE_WHRM_ALM (0x02)
  // This function enables (one) or disables (zero) the wrist heart rate monitor
  // algorithm.
  bool enableWHRMFastAlgorithm(uint8_t enable);
  // Family Byte: ENABLE_ALGORITHM (0x52), Index Byte: ENABLE_ECG_ALM
  // (0x03)
  // This function enables (one) or disables (zero) the electrocardiogram 
  // (ECG) algorithm.
  bool enableECGAlgorithm(uint8_t enable);
  // Family Byte: ENABLE_ALGORITHM (0x52), Index Byte: ENABLE_BPT_ALM
  // (0x04)
  // This function enables (one) or disables (zero) the electrocardiogram 
  // (ECG) algorithm.
  bool enableBPTAlgorithm(uint8_t enable);
  // Family Byte: ENABLE_ALGORITHM (0x52), Index Byte: ENABLE_WSP02_ALM
  // (0x05)
  // This function enables (one) or disables (zero) the WSP02 algorithm..
  bool enableWSP02Algorithm(uint8_t enable);
  // Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: SET_NUM_PAGES (0x02),
  // Write Bytes: 0x00 - Number of pages at byte 0x44 from .msbl file. 
  bool setNumPages(uint8_t totalPages);
  // Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: ERASE_FLASH (0x03)
  bool eraseFlash();
  // Family Byte: BOOTLOADER_INFO (0x81), Index Byte: BOOTLOADER_VERS (0x00)
  version readBootloaderVers();
  // Family Byte: BOOTLOADER_INFO (0x81), Index Byte: PAGE_SIZE (0x01)
  // Family Byte: IDENTITY (0xFF), Index Byte: READ_SENSOR_HUB_VERS (0x03)
  version readSensorHubVersion();
  // Family Byte: IDENTITY (0xFF), Index Byte: READ_ALM_VERS (0x07)
  version readAlgorithmVersion();

  private:   
  // Variables -----------
  uint8_t _resetPin;
  uint8_t _mfioPin;
  int _address; 
  uint8_t _calibData[608];
  
  // I-squared-C Class----
  TwoWire *_i2cPort;

  // Functions------------
  
  // For Read: Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
  // READ_WHRM_BPT_RESULTS (0x04), Write Byte: BPT_CALIBRATE_ID (0x03) 
  // For Write: Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: CALIBRATE_BPT
  // (0x04), Write Byte: BPT_CALIBRATE_ID (0x03)
  // This function takes the 608 data points acquired by the calibration
  // procedure and feeds them into the blood pressure trending
  // algorithm. 
  bool calibrateBPTAlm();
  // This function uses the given family, index, and write byte to communicate
  // with the MAX32664 which in turn communicates with downward sensors. There
  // are two steps demonstrated in this function. First a write to the MCU
  // indicating what you want to do, a delay, and then a read to confirm positive
  // transmission. 
  uint8_t writeByte( uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte );
  // This function sends is simliar to the one above and sends info to the MAX32664 
  // but takes an additional uint8_t as a paramter. Again there is the write
  // of the specific bytes followed by a read to confirm positive transmission. 
  uint8_t writeByte( uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _writeVal );
  // This function is the same as the function above and uses the given family, 
  // index, and write byte, but also takes a 16 bit integer as a paramter to communicate
  // with the MAX32664 which in turn communicates with downward sensors. There
  // are two steps demonstrated in this function. First a write to the MCU
  // indicating what you want to do, a delay, and then a read to confirm positive
  // transmission. 
  uint8_t writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint16_t _val);
  // This function sends information to the MAX32664 to specifically write values
  // to the registers of downward sensors and so also requires a
  // register address and register value as parameters. Again there is the write
  // of the specific bytes followed by a read to confirm positive transmission. 
  uint8_t writeLongBytes(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, long _writeVal[3]);
  // This function handles all read commands or stated another way, all information
  // requests. It starts a request by writing the family byte, index byte, and
  // delays 60 microseconds, during which the MAX32664 retrieves the requested 
  // information. An I-squared-C request is then issued, and the information is read and returned.
  uint8_t * readByte( uint8_t _familyByte, uint8_t _indexByte, uint8_t _numOfReads ); 
  // This function is exactly as the one above except it accepts a Write Byte as
  // a paramter. It starts a request by writing the family byte, index byte, and
  // write byte to the MAX32664, delays 60 microseconds, during which
  // the MAX32664 retrieves the requested information. A I-squared-C request is
  // then issued, and the information is read and returned. 
  uint8_t * readByte( uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _numOfReads ); 

  uint16_t * readByte( uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _numOfReads ); 
}

#endif
