#ifndef _SPARKFUN_BIO_SENSOR_HUB_LIBRARY_H_
#define _SPARKFUN_BIO_SENSOR_HUB_LIBRARY_H_

#include <Arduino.h>
#include <Wire.h>

#define WRITE_FIFO_INPUT_BYTE 0x04
#define DISABLE 0x00
#define ENABLE 0x01
#define MODE_ONE 0x01
#define MODE_TWO 0x02
#define APP_MODE 0x00
#define BOOTLOADER_MODE 0x08
#define NO_WRITE 0x00
#define INCORR_PARAM 0xEE

#define CONFIGURATION_REGISTER 0x0A
#define PULSE_MASK 0xFC
#define READ_PULSE_MASK 0x03
#define SAMP_MASK 0xE3
#define READ_SAMP_MASK 0x1C
#define ADC_MASK 0x9F
#define READ_ADC_MASK 0x60

#define ENABLE_CMD_DELAY 45     // Milliseconds
#define ALGO_CMD_DELAY_SHORT 45 // Milliseconds
#define ALGO_CMD_DELAY_LONG 45  // Milliseconds
#define CMD_DELAY 2             // Milliseconds
#define MAXFAST_ARRAY_SIZE 6    // Number of bytes....
#define MAXFAST_EXTENDED_DATA 5
#define MAX30101_LED_ARRAY 12 // 4 values of 24 bit (3 byte) LED values

#define SET_FORMAT 0x00
#define READ_FORMAT 0x01         // Index Byte under Family Byte: READ_OUTPUT_MODE (0x11)
#define WRITE_SET_THRESHOLD 0x01 // Index Byte for WRITE_INPUT(0x14)
#define WRITE_EXTERNAL_TO_FIFO 0x00

const uint8_t BIO_ADDRESS = 0x55;

struct bioData
{

    uint32_t irLed;
    uint32_t redLed;
    uint16_t heartRate;  // LSB = 0.1bpm
    uint8_t confidence;  // 0-100% LSB = 1%
    uint16_t oxygen;     // 0-100% LSB = 1%
    uint8_t status;      // 0: Success, 1: Not Ready, 2: Object Detectected, 3: Finger Detected
    float rValue;        // -- Algorithm Mode 2 vv
    int8_t extStatus;    // --
    uint8_t reserveOne;  // --
    uint8_t resserveTwo; // -- Algorithm Mode 2 ^^
};

struct version
{
    // 3 bytes total
    uint8_t major;
    uint8_t minor;
    uint8_t revision;
};

struct sensorAttr
{

    uint8_t byteWord;
    uint8_t availRegisters;
};

// Status Bytes are communicated back after every I-squared-C transmission and
// are indicators of success or failure of the previous transmission.
enum READ_STATUS_BYTE_VALUE
{

    SFE_BIO_SUCCESS = 0x00,
    ERR_UNAVAIL_CMD,
    ERR_UNAVAIL_FUNC,
    ERR_DATA_FORMAT,
    ERR_INPUT_VALUE,
    ERR_TRY_AGAIN,
    ERR_BTLDR_GENERAL = 0x80,
    ERR_BTLDR_CHECKSUM,
    ERR_BTLDR_AUTH,
    ERR_BTLDR_INVALID_APP,
    ERR_UNKNOWN = 0xFF
};

// The family register bytes are the larger umbrella for all the Index and
// Write Bytes listed below. You can not reference a nestled byte without first
// referencing it's larger category: Family Register Byte.
enum FAMILY_REGISTER_BYTES
{

    HUB_STATUS = 0x00,
    SET_DEVICE_MODE,
    READ_DEVICE_MODE,
    OUTPUT_MODE = 0x10,
    READ_OUTPUT_MODE,
    READ_DATA_OUTPUT,
    READ_DATA_INPUT,
    WRITE_INPUT,
    WRITE_REGISTER = 0x40,
    READ_REGISTER,
    READ_ATTRIBUTES_AFE,
    DUMP_REGISTERS,
    ENABLE_SENSOR,
    READ_SENSOR_MODE,
    CHANGE_ALGORITHM_CONFIG = 0x50,
    READ_ALGORITHM_CONFIG,
    ENABLE_ALGORITHM,
    BOOTLOADER_FLASH = 0x80,
    BOOTLOADER_INFO,
    IDENTITY = 0xFF
};

// All the defines below are: 1. Index Bytes nestled in the larger category of the
// family registry bytes listed above and 2. The Write Bytes nestled even
// farther under their Index Bytes.

// Write Bytes under Family Byte: SET_DEVICE_MODE (0x01) and Index
// Byte: 0x00.
enum DEVICE_MODE_WRITE_BYTES
{

    EXIT_BOOTLOADER = 0x00,
    SFE_BIO_RESET = 0x02,
    ENTER_BOOTLOADER = 0x08
};

// Write Bytes under Family Byte: OUTPUT_MODE (0x10) and Index byte: SET_FORMAT
// (0x00)
enum OUTPUT_MODE_WRITE_BYTE
{

    PAUSE = 0x00,
    SENSOR_DATA,
    ALGO_DATA,
    SENSOR_AND_ALGORITHM,
    PAUSE_TWO,
    SENSOR_COUNTER_BYTE,
    ALGO_COUNTER_BYTE,
    SENSOR_ALGO_COUNTER
};

// Index Byte under the Family Byte: READ_DATA_OUTPUT (0x12)
enum FIFO_OUTPUT_INDEX_BYTE
{

    NUM_SAMPLES,
    READ_DATA
};

// Index Byte under the Family Byte: READ_DATA_INPUT (0x13)
enum FIFO_EXTERNAL_INDEX_BYTE
{

    SAMPLE_SIZE,
    READ_INPUT_DATA,
    READ_SENSOR_DATA,       // For external accelerometer
    READ_NUM_SAMPLES_INPUT, // For external accelerometer
    READ_NUM_SAMPLES_SENSOR
};

// Index Byte under the Family Registry Byte: WRITE_REGISTER (0x40)
enum WRITE_REGISTER_INDEX_BYTE
{

    WRITE_MAX30101 = 0x03,
    WRITE_ACCELEROMETER
};

// Index Byte under the Family Registry Byte: READ_REGISTER (0x41)
enum READ_REGISTER_INDEX_BYTE
{

    READ_MAX30101 = 0x03,
    READ_ACCELEROMETER
};

// Index Byte under the Family Registry Byte: READ_ATTRIBUTES_AFE (0x42)
enum GET_AFE_INDEX_BYTE
{

    RETRIEVE_AFE_MAX30101 = 0x03,
    RETRIEVE_AFE_ACCELEROMETER
};

// Index Byte under the Family Byte: DUMP_REGISTERS (0x43)
enum DUMP_REGISTER_INDEX_BYTE
{

    DUMP_REGISTER_MAX30101 = 0x03,
    DUMP_REGISTER_ACCELEROMETER
};

// Index Byte under the Family Byte: ENABLE_SENSOR (0x44)
enum SENSOR_ENABLE_INDEX_BYTE
{

    ENABLE_MAX30101 = 0x03,
    ENABLE_ACCELEROMETER
};

// Index Byte for the Family Byte: READ_SENSOR_MODE (0x45)
enum READ_SENSOR_ENABLE_INDEX_BYTE
{

    READ_ENABLE_MAX30101 = 0x03,
    READ_ENABLE_ACCELEROMETER
};

// Index Byte under the Family Byte: CHANGE_ALGORITHM_CONFIG (0x50)
enum ALGORITHM_CONFIG_INDEX_BYTE
{

    SET_TARG_PERC = 0x00,
    SET_STEP_SIZE = 0x00,
    SET_SENSITIVITY = 0x00,
    SET_AVG_SAMPLES = 0x00,
    SET_PULSE_OX_COEF = 0x02,
    BPT_CONFIG = 0x04
};

// Write Bytes under the Family Byte: CHANGE_ALGORITHM_CONFIG (0x50) and the
// Index Byte: ALGORITHM_CONFIG_INDEX_BYTE - SET_TARG_PERC
enum ALGO_AGC_WRITE_BYTE
{

    AGC_GAIN_ID = 0x00,
    AGC_STEP_SIZE_ID,
    AGC_SENSITIVITY_ID,
    AGC_NUM_SAMP_ID,
    MAXIMFAST_COEF_ID = 0x0B
};

enum ALGO_BPT_WRITE_BYTE
{

    BPT_MEDICATION = 0x00,
    SYSTOLIC_VALUE,
    DIASTOLIC_VALUE,
    BPT_CALIB_DATA, // Index + 824 bytes of calibration data
    PATIENT_RESTING = 0x05,
    AGC_SP02_COEFS = 0x0B
};

// Index Bytes under the Family Byte: READ_ALGORITHM_CONFIG (0x51)
enum READ_ALGORITHM_INDEX_BYTE
{

    READ_AGC_PERCENTAGE = 0x00,
    READ_AGC_STEP_SIZE = 0x00,
    READ_AGC_SENSITIVITY = 0x00,
    READ_AGC_NUM_SAMPLES = 0x00,
    READ_MAX_FAST_COEF = 0x02
};

// Write Bytes under the Family Byte: READ_ALGORITHM_CONFIG (0x51) and Index Byte:
// READ_ALGORITHM_INDEX_BYTE - AGC
enum READ_AGC_ALGO_WRITE_BYTE
{

    READ_AGC_PERC_ID = 0x00,
    READ_AGC_STEP_SIZE_ID,
    READ_AGC_SENSITIVITY_ID,
    READ_AGC_NUM_SAMPLES_ID,
    READ_MAX_FAST_COEF_ID = 0x0B
};

// Index Byte under the Family Byte: ENABLE_ALGORITHM (0x52).
enum ALGORITHM_MODE_ENABLE_INDEX_BYTE
{

    ENABLE_AGC_ALGO = 0x00,
    ENABLE_WHRM_ALGO = 0x02
};

// Index Byte under the Family Byte: BOOTLOADER_FLASH (0x80).
enum BOOTLOADER_FLASH_INDEX_BYTE
{

    SET_INIT_VECTOR_BYTES = 0x00,
    SET_AUTH_BYTES,
    SET_NUM_PAGES,
    ERASE_FLASH,
    SEND_PAGE_VALUE
};

// Index Byte under the Family Byte: BOOTLOADER_INFO (0x81).
enum BOOTLOADER_INFO_INDEX_BYTE
{

    BOOTLOADER_VERS = 0x00,
    PAGE_SIZE
};

// Index Byte under the Family Byte: IDENTITY (0xFF).
enum IDENTITY_INDEX_BYTES
{

    READ_MCU_TYPE = 0x00,
    READ_SENSOR_HUB_VERS = 0x03,
    READ_ALGO_VERS = 0x07
};

class SparkFun_Bio_Sensor_Hub
{
  public:
    // Variables ------------
    uint8_t bpmArr[MAXFAST_ARRAY_SIZE]{};
    uint8_t bpmArrTwo[MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA]{};
    uint8_t senArr[MAX30101_LED_ARRAY]{};
    uint8_t bpmSenArr[MAXFAST_ARRAY_SIZE + MAX30101_LED_ARRAY]{};
    uint8_t bpmSenArrTwo[MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA + MAX30101_LED_ARRAY]{};

    // Constructor ----------
    SparkFun_Bio_Sensor_Hub(int resetPin = -1, int mfioPin = -1, uint8_t address = 0x55);

    // Functions ------------

    // Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
    // The following function initializes the sensor. To place the MAX32664 into
    // application mode, the MFIO pin must be pulled HIGH while the board is held
    // in reset for 10ms. After 50 addtional ms have elapsed the board should be
    // in application mode and will return two bytes, the first 0x00 is a
    // successful communcation byte, followed by 0x00 which is the byte indicating
    // which mode the IC is in.
    uint8_t begin(TwoWire &wirePort = Wire, int resetPin = -1, int mfioPin = -1);

    // Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
    // The following function puts the MAX32664 into bootloader mode. To place the MAX32664 into
    // bootloader mode, the MFIO pin must be pulled LOW while the board is held
    // in reset for 10ms. After 50 addtional ms have elapsed the board should be
    // in bootloader mode and will return two bytes, the first 0x00 is a
    // successful communcation byte, followed by 0x08 which is the byte indicating
    // that the board is in bootloader mode.
    uint8_t beginBootloader(TwoWire &wirePort = Wire, int resetPin = -1, int mfioPin = -1);

    // Family Byte: HUB_STATUS (0x00), Index Byte: 0x00, No Write Byte.
    // The following function checks the status of the FIFO.
    uint8_t readSensorHubStatus();

    // Family Byte: SET_DEVICE_MODE (0x01), Index Byte: 0x01, Write Byte: 0x00
    // The following function is an alternate way to set the mode of the of
    // MAX32664. It can take three parameters: Enter and Exit Bootloader Mode, as
    // well as reset.
    // INCOMPLETE
    uint8_t setOperatingMode(uint8_t);

    // This function sets very basic settings to get sensor and biometric data.
    // The biometric data includes data about heartrate, the confidence
    // level, SpO2 levels, and whether the sensor has detected a finger or not.
    uint8_t configBpm(uint8_t);

    // This function sets very basic settings to get LED count values from the MAX30101.
    // Sensor data includes 24 bit LED values for the three LED channels: Red, IR,
    // and Green.
    uint8_t configSensor();

    // This function sets very basic settings to get sensor and biometric data.
    // Sensor data includes 24 bit LED values for the two LED channels: Red and IR.
    // The biometric data includes data about heartrate, the confidence
    // level, SpO2 levels, and whether the sensor has detected a finger or not.
    // Of note, the number of samples is set to one.
    uint8_t configSensorBpm(uint8_t);

    // This function takes the 8 bytes from the FIFO buffer related to the wrist
    // heart rate algortihm: heart rate (uint16_t), confidence (uint8_t) , SpO2 (uint16_t),
    // and the finger detected status (uint8_t). Note that the the algorithm is stated as
    // "wrist" though the sensor only works with the finger. The data is loaded
    // into the whrmFifo and returned.
    bioData readBpm();

    // This function takes 9 bytes of LED values from the MAX30101 associated with
    // the RED, IR, and GREEN LEDs. In addition it gets the 8 bytes from the FIFO buffer
    // related to the wrist heart rate algortihm: heart rate (uint16_t), confidence (uint8_t),
    // SpO2 (uint16_t), and the finger detected status (uint8_t). Note that the the algorithm
    // is stated as "wrist" though the sensor only works with the finger. The data is loaded
    // into the whrmFifo and returned.
    bioData readSensor();

    // This function takes the information of both the LED value and the biometric
    // data from the MAX32664's FIFO. In essence it combines the two functions
    // above into a single function call.
    bioData readSensorBpm();

    // This function modifies the pulse width of the MAX30101 LEDs. All of the LEDs
    // are modified to the same width. This will affect the number of samples that
    // can be collected and will also affect the ADC resolution.
    // Width(us) - Resolution -  Sample Rate
    // Default: 69us - 15 resolution - 50 samples per second.
    //  69us     -    15      -   <= 3200 (fastest - least resolution)
    //  118us    -    16      -   <= 1600
    //  215us    -    17      -   <= 1600
    //  411us    -    18      -   <= 1000 (slowest - highest resolution)
    uint8_t setPulseWidth(uint16_t);

    // This function reads the CONFIGURATION_REGISTER (0x0A), bits [1:0] from the
    // MAX30101 Sensor. It returns one of the four settings in microseconds.
    uint16_t readPulseWidth();

    // This function changes the sample rate of the MAX30101 sensor. The sample
    // rate is affected by the set pulse width of the MAX30101 LEDs.
    // Default: 69us - 15 resolution - 50 samples per second.
    // Width(us) - Resolution -  Sample Rate
    //  69us     -    15      -   <= 3200 (fastest - least resolution)
    //  118us    -    16      -   <= 1600
    //  215us    -    17      -   <= 1600
    //  411us    -    18      -   <= 1000 (slowest - highest resolution)
    //  Samples Options:
    //  50, 100, 200, 400, 800, 1000, 1600, 3200
    uint8_t setSampleRate(uint16_t);

    // This function reads the CONFIGURATION_REGISTER (0x0A), bits [4:2] from the
    // MAX30101 Sensor. It returns one of the 8 possible sample rates.
    uint16_t readSampleRate();

    // MAX30101 Register: CONFIGURATION_REGISTER (0x0A), bits [6:5]
    // This functions sets the dynamic range of the MAX30101's ADC. The function
    // accepts the higher range as a parameter.
    // Default Range: 7.81pA - 2048nA
    // Possible Ranges:
    // 7.81pA  - 2048nA
    // 15.63pA - 4096nA
    // 32.25pA - 8192nA
    // 62.5pA  - 16384nA
    uint8_t setAdcRange(uint16_t);

    // MAX30101 Register: CONFIGURATION_REGISTER (0x0A), bits [6:5]
    // This function returns the set ADC range of the MAX30101 sensor.
    uint16_t readAdcRange();

    // Family Byte: IDENTITY (0x01), Index Byte: READ_MCU_TYPE, Write Byte: NONE
    // The following function returns a byte that signifies the microcontoller that
    // is in communcation with your host microcontroller. Returns 0x00 for the
    // MAX32625 and 0x01 for the MAX32660/MAX32664.
    uint8_t getMcuType();

    // Family Byte: BOOTLOADER_INFO (0x80), Index Byte: BOOTLOADER_VERS (0x00)
    // This function checks the version number of the bootloader on the chip and
    // returns a four bytes: Major version Byte, Minor version Byte, Space Byte,
    // and the Revision Byte.
    int32_t getBootloaderInf();

    // Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX30101 (0x03), Write
    // Byte: senSwitch (parameter - 0x00 or 0x01).
    // This function enables the MAX30101.
    uint8_t max30101Control(uint8_t);

    // Family Byte: READ_SENSOR_MODE (0x45), Index Byte: READ_ENABLE_MAX30101 (0x03)
    // This function checks if the MAX30101 is enabled or not.
    uint8_t readMAX30101State();

    // Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_ACCELEROMETER (0x04), Write
    // Byte: accelSwitch (parameter - 0x00 or 0x01).
    // This function enables the ACCELEROMETER.
    uint8_t accelControl(uint8_t);

    // Family Byte: OUTPUT_MODE (0x10), Index Byte: SET_FORMAT (0x00),
    // Write Byte : outputType (Parameter values in OUTPUT_MODE_WRITE_BYTE)
    uint8_t setOutputMode(uint8_t);

    // Family Byte: OUTPUT_MODE, Index Byte: WRITE_SET_THRESHOLD, Write byte: intThres
    // (parameter - value betwen 0 and 0xFF).
    // This function changes the threshold for the FIFO interrupt bit/pin. The
    // interrupt pin is the MFIO pin which is set to INPUT after IC initialization
    // (begin).
    uint8_t setFifoThreshold(uint8_t);

    // Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: NUM_SAMPLES (0x00), Write
    // Byte: NONE
    // This function returns the number of samples available in the FIFO.
    uint8_t numSamplesOutFifo();

    // Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: READ_DATA (0x00), Write
    // Byte: NONE
    // This function returns the data in the FIFO.
    uint8_t *getDataOutFifo(uint8_t data[]);

    // Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: READ_DATA (0x00), Write
    // Byte: NONE
    // This function adds support for the acceleromter that is NOT included on
    // SparkFun's product, The Family Registery of 0x13 and 0x14 is skipped for now.
    uint8_t numSamplesExternalSensor();

    // Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_MAX30101 (0x03), Write Bytes:
    // Register Address and Register Value
    // This function writes the given register value at the given register address
    // for the MAX30101 sensor and returns a boolean indicating a successful or
    // non-successful write.
    void writeRegisterMAX30101(uint8_t, uint8_t);

    // Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_ACCELEROMETER (0x04), Write Bytes:
    // Register Address and Register Value
    // This function writes the given register value at the given register address
    // for the Accelerometer and returns a boolean indicating a successful or
    // non-successful write.
    void writeRegisterAccel(uint8_t, uint8_t);

    // Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30101 (0x03), Write Byte:
    // Register Address
    // This function reads the given register address for the MAX30101 Sensor and
    // returns the values at that register.
    uint8_t readRegisterMAX30101(uint8_t);

    // Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30101 (0x03), Write Byte:
    // Register Address
    // This function reads the given register address for the MAX30101 Sensor and
    // returns the values at that register.
    uint8_t readRegisterAccel(uint8_t);

    // Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte: RETRIEVE_AFE_MAX30101/ (0x03)
    // This function retrieves the attributes of the AFE (Analog Front End) of the
    // MAX30101 sensor. It returns the number of bytes in a word for the sensor
    // and the number of registers available.
    sensorAttr getAfeAttributesMAX30101();

    // Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte:
    // RETRIEVE_AFE_ACCELEROMETER (0x04)
    // This function retrieves the attributes of the AFE (Analog Front End) of the
    // Accelerometer. It returns the number of bytes in a word for the sensor
    // and the number of registers available.
    sensorAttr getAfeAttributesAccelerometer();

    // Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_MAX30101 (0x03)
    // This function returns all registers and register values sequentially of the
    // MAX30101 sensor: register zero and register value zero to register n and
    // register value n. There are 36 registers in this case.
    uint8_t dumpRegisterMAX30101(uint8_t regArray[]);

    // Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_ACCELEROMETER (0x04)
    // This function returns all registers and register values sequentially of the
    // Accelerometer: register zero and register value zero to register n and
    // register value n.
    uint8_t dumpRegisterAccelerometer(uint8_t, uint8_t regArray[]);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
    // SET_TARG_PERC (0x00), Write Byte: AGC_GAIN_ID (0x00)
    // This function sets the target percentage of the full-scale ADC range that
    // the automatic gain control algorithm uses. It takes a paramater of zero to
    // 100 percent.
    uint8_t setAlgoRange(uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
    // SET_STEP_SIZE (0x00), Write Byte: AGC_STEP_SIZE_ID (0x01)
    // This function changes the step size toward the target for the AGC algorithm.
    // It takes a paramater of zero to 100 percent.
    uint8_t setAlgoStepSize(uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
    // SET_SENSITIVITY (0x00), Write Byte: AGC_SENSITIVITY_ID (0x02)
    // This function changes the sensitivity of the AGC algorithm.
    uint8_t setAlgoSensitivity(uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
    // SET_AVG_SAMPLES (0x00), Write Byte: AGC_NUM_SAMP_ID (0x03)
    // This function changes the number of samples that are averaged.
    // It takes a paramater of zero to 255.
    uint8_t setAlgoSamples(uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
    // SET_PULSE_OX_COEF (0x02), Write Byte: MAXIMFAST_COEF_ID (0x0B)
    // This function takes three values that are used as the Sp02 coefficients.
    uint8_t setMaximFastCoef(int32_t, int32_t, int32_t);

    // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
    // READ_AGC_PERCENTAGE (0x00), Write Byte: READ_AGC_PERC_ID (0x00)
    // This function reads and returns the currently set target percentage
    // of the full-scale ADC range that the Automatic Gain Control algorithm is using.
    uint8_t readAlgoRange();

    // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
    // READ_AGC_STEP_SIZE (0x00), Write Byte: READ_AGC_STEP_SIZE_ID (0x01)
    // This function returns the step size toward the target for the AGC algorithm.
    // It returns a value between zero and 100 percent.
    uint8_t readAlgoStepSize();

    // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
    // READ_AGC_SENSITIVITY_ID (0x00), Write Byte: READ_AGC_SENSITIVITY_ID (0x02)
    // This function returns the sensitivity (percentage) of the automatic gain control.
    uint8_t readAlgoSensitivity();

    // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
    // READ_AGC_NUM_SAMPLES (0x00), Write Byte: READ_AGC_NUM_SAMPlES_ID (0x03)
    // This function changes the number of samples that are averaged.
    // It takes a paramater of zero to 255.
    uint8_t readAlgoSamples();

    // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
    // READ_MAX_FAST_COEF (0x02), Write Byte: READ_MAX_FAST_COEF_ID (0x0B)
    // This function reads the maximum age for the wrist heart rate monitor
    // (WHRM) algorithm. It returns three uint32_t integers that are
    // multiplied by 100,000.
    // INCOMPLETE
    uint8_t readMaximFastCoef(int32_t coefArr[3]);

    // Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
    // ENABLE_AGC_ALGO (0x00)
    // This function enables (one) or disables (zero) the automatic gain control algorithm.
    uint8_t agcAlgoControl(uint8_t);

    // Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
    // ENABLE_WHRM_ALGO (0x02)
    // This function enables (one) or disables (zero) the wrist heart rate monitor
    // algorithm.
    uint8_t maximFastAlgoControl(uint8_t);

    // Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: SET_NUM_PAGES (0x02),
    // Write Bytes: 0x00 - Number of pages at byte 0x44 from .msbl file.
    bool setNumPages(uint8_t);

    // Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: ERASE_FLASH (0x03)
    // Returns true on successful communication.
    bool eraseFlash();

    // Family Byte: BOOTLOADER_INFO (0x81), Index Byte: BOOTLOADER_VERS (0x00)
    version readBootloaderVers();

    // Family Byte: BOOTLOADER_INFO (0x81), Index Byte: PAGE_SIZE (0x01)
    // Family Byte: IDENTITY (0xFF), Index Byte: READ_SENSOR_HUB_VERS (0x03)
    version readSensorHubVersion();

    // Family Byte: IDENTITY (0xFF), Index Byte: READ_ALGO_VERS (0x07)
    version readAlgorithmVersion();

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: BPT_MEDICATION (0x00)
    uint8_t isPatientBPMedication(uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: BPT_MEDICATION (0x00)
    uint8_t isPatientBPMedication();

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: DIASTOLIC_VALUE (0x02)
    uint8_t writeDiastolicVals(uint8_t, uint8_t, uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: DIASTOLIC_VALUE (0x02)
    uint8_t readDiastolicVals(uint8_t userArray[]);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: SYSTOLIC_VALUE (0x01)
    uint8_t writeSystolicVals(uint8_t, uint8_t, uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: SYSTOLIC_VALUE (0x01)
    uint8_t readSystolicVals(uint8_t userArray[]);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: BPT_CALIB_DATA (0x03)
    uint8_t writeBPTAlgoData(uint8_t bptCalibData[]);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: BPT_CALIB_DATA (0x03)
    uint8_t readBPTAlgoData(uint8_t userArray[]);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: PATIENT_RESTING (0x05)
    uint8_t isPatientResting(uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: PATIENT_RESTING (0x05)
    uint8_t isPatientResting();

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: AGC_SP02_COEFS (0x0B)
    uint8_t writeSP02AlgoCoef(int32_t, int32_t, int32_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: AGC_SP02_COEFS (0x0B)
    uint8_t readSP02AlgoCoef(int32_t userArray[]);

  private:
    // Variables -----------
    int _resetPin;
    int _mfioPin;
    uint8_t _address;
    uint32_t _writeCoefArr[3]{};
    uint8_t _userSelectedMode;
    uint8_t _sampleRate = 100;

    // I-squared-C Class----
    TwoWire *_i2cPort;

    // Functions------------

    // This function uses the given family, index, and write byte to enable
    // the given sensor.
    uint8_t enableWrite(uint8_t, uint8_t, uint8_t);

    // This function uses the given family, index, and write byte to communicate
    // with the MAX32664 which in turn communicates with downward sensors. There
    // are two steps demonstrated in this function. First a write to the MCU
    // indicating what you want to do, a delay, and then a read to confirm positive
    // transmission.
    uint8_t writeByte(uint8_t, uint8_t, uint8_t);

    // This function sends is simliar to the one above and sends info to the MAX32664
    // but takes an additional uint8_t as a paramter. Again there is the write
    // of the specific bytes followed by a read to confirm positive transmission.
    uint8_t writeByte(uint8_t, uint8_t, uint8_t, uint8_t);

    // This function is the same as the function above and uses the given family,
    // index, and write byte, but also takes a 16 bit integer as a paramter to communicate
    // with the MAX32664 which in turn communicates with downward sensors. There
    // are two steps demonstrated in this function. First a write to the MCU
    // indicating what you want to do, a delay, and then a read to confirm positive
    // transmission.
    uint8_t writeByte(uint8_t, uint8_t, uint8_t, uint16_t);

    // This function sends information to the MAX32664 to specifically write values
    // to the registers of downward sensors and so also requires a
    // register address and register value as parameters. Again there is the write
    // of the specific bytes followed by a read to confirm positive transmission.
    uint8_t writeLongBytes(uint8_t, uint8_t, uint8_t, int32_t _writeVal[], const size_t);

    // This function sends information to the MAX32664 to specifically write values
    // to the registers of downward sensors and so also requires a
    // register address and register value as parameters. Again there is the write
    // of the specific bytes followed by a read to confirm positive transmission.
    uint8_t writeBytes(uint8_t, uint8_t, uint8_t, uint8_t _writeVal[], const size_t);

    // This function handles all read commands or stated another way, all information
    // requests. It starts a request by writing the family byte, index byte, and
    // delays 60 microseconds, during which the MAX32664 retrieves the requested
    // information. An I-squared-C request is then issued, and the information is read and returned.
    uint8_t readByte(uint8_t, uint8_t);

    // This function is exactly as the one above except it accepts a Write Byte as
    // a paramter. It starts a request by writing the family byte, index byte, and
    // write byte to the MAX32664, delays 60 microseconds, during which
    // the MAX32664 retrieves the requested information. A I-squared-C request is
    // then issued, and the information is read and returned.
    uint8_t readByte(uint8_t, uint8_t, uint8_t);

    // This function handles all read commands or stated another way, all information
    // requests. It starts a request by writing the family byte, an index byte, and
    // a write byte and then then delays 60 microseconds, during which the MAX32664
    // retrieves the requested information. An I-squared-C request is then issued,
    // and the information is read. This differs from the above read commands in
    // that it returns a 16 bit integer instead of 8.
    uint16_t readIntByte(uint8_t, uint8_t, uint8_t);

    // This function handles all read commands or stated another way, all information
    // requests. It starts a request by writing the family byte, an index byte, and
    // a write byte and then then delays 60 microseconds, during which the MAX32664
    // retrieves the requested information. An I-squared-C request is then issued,
    // and the information is read. This function is very similar to the one above
    // except it returns three uint32_t bytes instead of one.
    uint8_t readMultipleBytes(uint8_t, uint8_t, uint8_t, const size_t, int32_t userArray[]);

    // This function handles all read commands or stated another way, all information
    // requests. It starts a request by writing the family byte, an index byte, and
    // a write byte and then then delays 60 microseconds, during which the MAX32664
    // retrieves the requested information. An I-squared-C request is then issued,
    // and the information is read. This function is very similar to the one above
    // except it returns multiple requested bytes.
    uint8_t readMultipleBytes(uint8_t, uint8_t, uint8_t, const size_t, uint8_t userArray[]);

    // Needs comment - INCOMPLETE
    uint8_t readFillArray(uint8_t, uint8_t, uint8_t, uint8_t array[]);
};
#endif
