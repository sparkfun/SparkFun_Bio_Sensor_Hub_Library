/*
  This is an Arduino Library written for the MAXIM 32664 Biometric Sensor Hub 
  The MAX32664 Biometric Sensor Hub is in actuality a small Cortex M4 microcontroller
  with pre-loaded firmware and algorithms used to interact with the a number of MAXIM
  sensors; specifically the MAX30101 Pulse Oximter and Heart Rate Monitor and
  the KX122 Accelerometer. With that in mind, this library is built to
  communicate with a middle-person and so has a unique method of communication
  (family, index, and write bytes) that is more simplistic than writing and reading to 
  registers, but includes a larger set of definable values.  

  SparkFun Electronics
  Date: March, 2019
  Author: Elias Santistevan
kk
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
*/


#include "SparkFun_Bio_Sensor_Hub_Library.h"

SparkFun_Bio_Sensor_Hub::SparkFun_Bio_Sensor_Hub(int address, uint8_t resetPin, uint8_t mfioPin ) { 
  
  _resetPin = resetPin; 
  _mfioPin = mfioPin;
  _address = address; 
  pinMode(_mfioPin, OUTPUT); 
  pinMode(_resetPin, OUTPUT); // Set these pins as output
  
}

// Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
// The following function initializes the sensor. To place the MAX32664 into
// application mode, the MFIO pin must be pulled HIGH while the board is held
// in reset for 10ms. After 50 addtional ms have elapsed the board should be 
// in application mode and will return two bytes, the first 0x00 is a 
// successful communcation byte, followed by 0x00 which is the byte indicating 
// which mode the IC is in. 
uint8_t SparkFun_Bio_Sensor_Hub::begin( TwoWire &wirePort ) {

  uint8_t * responseByte;
  _i2cPort = &wirePort;
  //  _i2cPort->begin(); A call to Wire.begin should occur in sketch 
  //  to avoid multiple begins with other sketches.

  digitalWrite(_resetPin, LOW); 
  digitalWrite(_mfioPin, HIGH); 
  delay(10); 
  digitalWrite(_resetPin, HIGH); 
  delay(50); //Application mode is enabled when this ends 
  pinMode(_resetPin, OUTPUT); 
  pinMode(_mfioPin, INPUT); // Input so that it may be used

  responseByte = readByte(READ_DEVICE_MODE, 0x00, 2); // 0x00 only possible Index Byte.
  return responseByte[1];

}

// Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
// The following function puts the MAX32664 into bootloader mode. To place the MAX32664 into
// bootloader mode, the MFIO pin must be pulled LOW while the board is held
// in reset for 10ms. After 50 addtional ms have elapsed the board should be 
// in bootloader mode and will return two bytes, the first 0x00 is a 
// successful communcation byte, followed by 0x08 which is the byte indicating 
// that the board is in bootloader mode. 
bool SparkFun_Bio_Sensor_Hub::beginBootloader( TwoWire &wirePort ) {

  _i2cPort = &wirePort; 
  //  _i2cPort->begin(); A call to Wire.begin should occur in sketch 
  //  to avoid multiple begins with other sketches.
  
  digitalWrite(_mfioPin, LOW); 
  digitalWrite(_resetPin, LOW); 
  delay(10); 
  digitalWrite(_resetPin, HIGH); 
  delay(50);  //Bootloader mode is enabled when this ends.  
  pinMode(_resetPin, OUTPUT); 
  pinMode(_mfioPin, OUTPUT); 
  
  // Let's check to see if the device made it into bootloader mode.  
  uint8_t responseByte = readByte(READ_DEVICE_MODE, 0x00, 2); // 0x00 only possible Index Byte
  return responseByte;

}

// Family Byte: SET_DEVICE_MODE (0x01), Index Byte: 0x01, Write Byte: 0x00
// The following function is an alternate way to set the mode of the of
// MAX32664. It can take three parameters: Enter and Exit Bootloader Mode, as
// well as reset. 
// INCOMPLETE
uint8_t SparkFun_Bio_Sensor_Hub::setOperatingMode(uint8_t selection) {
   
    // Must be one of the three....
    if (selection != EXIT_BOOTLOADER || selection != RESET || selection != ENTER_BOOTLOADER)
      return INCORR_PARAM;

    uint8_t statusByte = writeByte(SET_DEVICE_MODE, 0x00, selection);
    if (statusByte != SUCCESS ) 
      return statusByte; 

    // Here we'll check if the board made it into Bootloader mode...
    uint8_t responseByte = readByte(READ_DEVICE_MODE, 0x00, 2); // 0x00 only possible Index Byte
    return responseByte; // This is in fact the status byte, need second returned byte - bootloader mode

}

// Family Byte: IDENTITY (0x01), Index Byte: READ_MCU_TYPE, Write Byte: NONE
// The following function returns a byte that signifies the microcontoller that
// is in communcation with your host microcontroller. Returns 0x00 for the
// MAX32625 and 0x01 for the MAX32660/MAX32664. 
// INCOMPLETE
uint8_t SparkFun_Bio_Sensor_Hub::getMCUtype() { 

  uint8_t mcu = readByte(IDENTITY, READ_MCU_TYPE, NO_WRITE, 2);  
  return mcu; // Needs the second byte returned, not the first. 

}

// Family Byte: BOOTLOADER_INFO (0x80), Index Byte: BOOTLOADER_VERS (0x00) 
// This function checks the version number of the bootloader on the chip and
// returns a four bytes: Major version Byte, Minor version Byte, Space Byte,
// and the Revision Byte. 
// INCOMPLETE
long SparkFun_Bio_Sensor_Hub::getBootloaderInf() {

  long bootVers = 0;
  uint8_t * revNum = readByte(BOOTLOADER_INFO, BOOTLOADER_VERS, 0x00, 4);   
  if( revNum[1] != SUCCESS )
    return ERR_UNKNOWN; 
  else {
    bootVers |= (revNum[1] << 16);
    bootVers |= (revNum[2] << 8); 
    bootVers |= revNum[3]; 
    return bootVers;
  }

}

// Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX86140 (0x00), Write
// Byte: enable (parameter - 0x00 or 0x01). 
// This function enables the MAX86140. 
bool SparkFun_Bio_Sensor_Hub::enableSensorMAX86140(uint8_t enable) {

  if(enable != 0 || enable != 1)
    return false; 

  // Check that communication was successful, not that the sensor is enabled.
  uint8_t statusByte = writeByte(ENABLE_SENSOR, ENABLE_MAX86140, enable);
  if( statusByte == SUCCESS )
    return true; 
  else
    return false; 

}

// Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX30205 (0x01), Write
// Byte: enable (parameter - 0x00 or 0x01). 
// This function enables the MAX30205. 
bool SparkFun_Bio_Sensor_Hub::enableSensorMAX30205(uint8_t enable) {

  if(enable != 0 || enable != 1)
    return false; 
  
  // Check that communication was successful, not that the sensor is enabled.
  uint8_t statusByte = writeByte(ENABLE_SENSOR, ENABLE_MAX30205, enable);
  if( statusByte == SUCCESS ) 
    return true; 
  else
    return false; 

}

// Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX30001 (0x02), Write
// Byte: enable (parameter - 0x00 or 0x01). 
// This function enables the MAX30001. 
bool SparkFun_Bio_Sensor_Hub::enableSensorMAX30001(uint8_t enable) {

  if(enable != 0 || enable != 1)
    return false; 

  // Check that communication was successful, not that the sensor is enabled.
  uint8_t statusByte = writeByte(ENABLE_SENSOR, ENABLE_MAX30001, enable);
  if( statusByte == SUCCESS ) 
    return true; 
  else
    return false;

}

// Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX30101 (0x03), Write
// Byte: enable (parameter - 0x00 or 0x01).
// This function enables the MAX30101. 
uint8_t SparkFun_Bio_Sensor_Hub::enableSensorMAX30101(uint8_t enable) {

  if(enable != 0 || enable != 1)
    return false; 

  // Check that communication was successful, not that the sensor is enabled.
  uint8_t responseByte = writeByte(ENABLE_SENSOR, ENABLE_MAX30101, enable);
  if( responseByte == SUCCESS ) 
    return true; 
  else
    return false; 

}

// Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_ACCELEROMETER (0x04), Write
// Byte: enable (parameter - 0x00 or 0x01). 
// This function enables the Accelerometer. 
bool SparkFun_Bio_Sensor_Hub::enableSensorAccel(uint8_t enable) {

  if(enable != 0 || enable != 1)
    return false; 
  
  // Check that communication was successful, not that the sensor is enabled.
  uint8_t statusByte = writeByte(ENABLE_SENSOR, ENABLE_ACCELEROMETER, enable);
  if( statusByte == SUCCESS ) 
    return true; 
  else
    return false; 

}

// Family Byte: OUTPUT_FORMAT (0x10), Index Byte: SET_FORMAT (0x00), 
// Write Byte : outputType (Parameter values in OUTPUT_MODE_WRITE_BYTE)
bool SparkFun_Bio_Sensor_Hub::setOutputMode(uint8_t outputType) {

  if (outputType < PAUSE || outputType > SENSOR_ALM_COUNTER) // Bytes between 0x00 and 0x07
    return false; 

  // Check that communication was successful, not that the IC is outputting
  // correct format. 
  uint8_t statusByte = writeByte(OUTPUT_FORMAT, SET_FORMAT, outputType);  
  if( statusByte == SUCCESS)
    return true; 
  else
    return false; 

}

// Family Byte: OUTPUT_FORMAT, Index Byte: SET_THRESHOLD, Write byte: intThres
// (parameter - value betwen 0 and 0xFF).
// This function changes the threshold for the FIFO interrupt bit/pin. The
// interrupt pin is the MFIO pin which is set to INPUT after IC initialization
// (begin). 
bool SparkFun_Bio_Sensor_Hub::setFIFOThreshold(uint8_t intThresh) {

  if( intThresh < 0 || intThresh > 255)
    return false; 
 
  // Checks that there was succesful communcation, not that the threshold was
  // set correctly. 
  uint8_t statusByte = writeByte(OUTPUT_FORMAT, SET_THRESHOLD, intThresh); 
  if( statusByte == SUCCESS)
    return true; 
  else
    return false; 

}

// Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: NUM_SAMPLES (0x00), Write
// Byte: NONE
// This function returns the number of samples available in the FIFO. 
// INCOMPLETE
uint8_t SparkFun_Bio_Sensor_Hub::numSamplesOutFIFO() {

  uint8_t sampAvail;

  // Checks the status byte but not the number of samples....
  uint8_t statusByte = readByte(READ_DATA_OUTPUT, NUM_SAMPLES, NO_WRITE, 1); 
  if( statusByte == SUCCESS )
    return sampAvail;

}

// Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: READ_DATA (0x00), Write
// Byte: NONE
// This function returns the data in the FIFO. 
// INCOMPLETE
uint8_t SparkFun_Bio_Sensor_Hub::getDataOutFIFO() {

  uint8_t sampAvail = readByte(READ_DATA_OUTPUT, NUM_SAMPLES, NO_WRITE, 1); 
  uint8_t dataAvail = readByte(READ_DATA_OUTPUT, READ_DATA, NO_WRITE, sampAvail); 
  return dataAvail; //pointer instead?

}

// Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: READ_DATA (0x00), Write
// Byte: NONE
// This function adds support for the acceleromter that is NOT included on
// SparkFun's product, The Family Registery of 0x13 and 0x14 is skipped for now. 
uint8_t SparkFun_Bio_Sensor_Hub::numSamplesExternalSensor() {

  uint8_t sampAvail = readByte(READ_DATA_INPUT, SAMPLE_SIZE, ACCELEROMETER, 1); 
  return sampAvail;

}

// Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_MAX86140 (0x00), Write Bytes:
// Register Address and Register Value
// This function writes the given register value at the given register address
// for the MAX86140 and MAX86141 Sensor and returns a boolean indicating a successful 
// or non-successful write.  
bool SparkFun_Bio_Sensor_Hub::writeRegisterMAX861X(uint8_t regAddr, uint8_t regVal) {

  uint8_t writeStat = writeRegister(WRITE_REGISTER, WRITE_MAX86140, regAddr, regVal);
  if( writeStat == SUCCESS) 
    return true; 
  else
    return false; 
}

// Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_MAX30205 (0x01), Write Bytes:
// Register Address and Register Value
// This function writes the given register value at the given register address
// for the MAX30205 sensor and returns a boolean indicating a successful or
// non-successful write. 
bool SparkFun_Bio_Sensor_Hub::writeRegisterMAX30205(uint8_t regAddr, uint8_t regVal) {
 
  uint8_t writeStat = writeRegister(WRITE_REGISTER, WRITE_MAX30205, regAddr, regVal);
  if( writeStat == SUCCESS) 
    return true; 
  else
    return false; 
}

// Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_MAX30001 (0x02), Write Bytes:
// Register Address and Register Value
// This function writes the given register value at the given register address
// for the MAX30001 sensor and returns a boolean indicating a successful or
// non-successful write. 
bool SparkFun_Bio_Sensor_Hub::writeRegisterMAX30001(uint8_t regAddr, uint8_t regVal) {
  
  uint8_t writeStat = writeRegister(WRITE_REGISTER, WRITE_MAX30001, regAddr, regVal);
  if( writeStat == SUCCESS) 
    return true; 
  else
    return false; 
}

// Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_MAX30101 (0x03), Write Bytes:
// Register Address and Register Value
// This function writes the given register value at the given register address
// for the MAX30101 sensor and returns a boolean indicating a successful or
// non-successful write. 
bool SparkFun_Bio_Sensor_Hub::writeRegisterMAX30101(uint8_t regAddr, uint8_t regVal) {

  uint8_t writeStat = writeRegister(WRITE_REGISTER, WRITE_MAX30101, regAddr, regVal);
  if( writeStat == SUCCESS) 
    return true; 
  else
    return false; 
}

// Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_ACCELEROMETER (0x04), Write Bytes:
// Register Address and Register Value
// This function writes the given register value at the given register address
// for the Accelerometer and returns a boolean indicating a successful or
// non-successful write. 
bool SparkFun_Bio_Sensor_Hub::writeRegisterAccel(uint8_t regAddr, uint8_t regVal) {

  uint8_t writeStat = writeRegister(WRITE_REGISTER, WRITE_ACCELEROMETER, regAddr, regVal);
  if( writeStat == SUCCESS) 
    return true; 
  else
    return false; 
}

// Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX86140 (0x00), Write Byte: 
// Register Address
// This function reads the given register address for the MAX86140 and MAX8641
// Sensors and returns the values at that register. 
uint8_t SparkFun_Bio_Sensor_Hub::readRegisterMAX8614X(uint8_t regAddr) {

  uint8_t regCont = readByte(READ_REGISTER, READ_MAX86140, regAddr, 1); 
  return regCont;

}

// Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30205 (0x01), Write Byte: 
// Register Address
// This function reads the given register address for the MAX30205 Sensor and
// returns the values at that register. 
uint8_t SparkFun_Bio_Sensor_Hub::readRegisterMAX30205(uint8_t regAddr) {

  uint8_t regCont = readByte(READ_REGISTER, READ_MAX30205, regAddr, 1); 
  return regCont;

}

// Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30001 (0x02), Write Byte: 
// Register Address
// This function reads the given register address for the MAX30001 Sensor and
// returns the values at that register. 
uint8_t SparkFun_Bio_Sensor_Hub::readRegisterMAX30001(uint8_t regAddr) {

  uint8_t regCont = readByte(READ_REGISTER, READ_MAX30001, regAddr, 1); 
  return regCont;

}

// Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30101 (0x03), Write Byte: 
// Register Address
// This function reads the given register address for the MAX30101 Sensor and
// returns the values at that register. 
uint8_t SparkFun_Bio_Sensor_Hub::readRegisterMAX30101(uint8_t regAddr) {

  uint8_t regCont = readByte(READ_REGISTER, READ_MAX30101, regAddr, 1); 
  return regCont;

}

// Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30101 (0x03), Write Byte: 
// Register Address
// This function reads the given register address for the MAX30101 Sensor and
// returns the values at that register. 
uint8_t SparkFun_Bio_Sensor_Hub::readRegisterAccel(uint8_t regAddr) {

  uint8_t regCont = readByte(READ_REGISTER, READ_ACCELEROMETER, regAddr, 1); 
  return regCont;

}

// Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte: RETRIEVE_AFE_MAX86140 (0x00)
// This function retrieves the attributes of the AFE (Analog Front End) of the
// MAX8640/1 sensors. It returns the number of bytes in a word for the sensor
// and the number of registers available. 
// INCOMPLETE - must check datasheet of individual sensor to know how many
// registers are returned. 
uint8_t SparkFun_Bio_Sensor_Hub::getAFEAttributesMAX86140() {

  uint8_t = statusByte readByte(READ_ATTRIBUTES_AFE, RETRIEVE_AFE_MAX86140, 20);// Fake read amount  
  if( statusByte == SUCCESS ){
    // Get attributes here
  }
    
}

// Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte: RETRIEVE_AFE_MAX30205 (0x01)
// This function retrieves the attributes of the AFE (Analog Front End) of the
// MAX30205 sensor. It returns the number of bytes in a word for the sensor
// and the number of registers available. 
// INCOMPLETE - must check datasheet of individual sensor to know how many
// registers are returned. 
uint8_t SparkFun_Bio_Sensor_Hub::getAFEAttributesMAX30205() {

  uint8_t = statusByte readByte(READ_ATTRIBUTES_AFE, RETRIEVE_AFE_MAX30205, 20);// Fake read amount  
  if( statusByte == SUCCESS ){
    // return attributes here
  }
    
}

// Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte: RETRIEVE_AFE_MAX30001 (0x02)
// This function retrieves the attributes of the AFE (Analog Front End) of the
// MAX30001 sensor. It returns the number of bytes in a word for the sensor
// and the number of registers available. 
// INCOMPLETE - must check datasheet of individual sensor to know how many
// registers are returned. 
uint8_t SparkFun_Bio_Sensor_Hub::getAFEAttributesMAX30001() {

  uint8_t = statusByte readByte(READ_ATTRIBUTES_AFE, RETRIEVE_AFE_MAX30001, 20);// Fake read amount  
  if( statusByte == SUCCESS ){
    // return attributes here
  }
    
}

// Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte: RETRIEVE_AFE_MAX30101/ (0x03)
// This function retrieves the attributes of the AFE (Analog Front End) of the
// MAX30101 sensor. It returns the number of bytes in a word for the sensor
// and the number of registers available. 
// INCOMPLETE - must check datasheet of individual sensor to know how many
// registers are returned. 
uint8_t SparkFun_Bio_Sensor_Hub::getAFEAttributesMAX30101() {

  uint8_t = statusByte readByte(READ_ATTRIBUTES_AFE, RETRIEVE_AFE_MAX30101, 20);// Fake read amount  
  if( statusByte == SUCCESS ){
    // return attributes here
  }
    
}


// Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte:
// RETRIEVE_AFE_ACCELEROMETER (0x04)
// This function retrieves the attributes of the AFE (Analog Front End) of the
// Accelerometer. It returns the number of bytes in a word for the sensor
// and the number of registers available. 
// INCOMPLETE - must check datasheet of individual sensor to know how many
// registers are returned. 
uint8_t SparkFun_Bio_Sensor_Hub::getAFEAttributesAccelerometer() {

  uint8_t = statusByte readByte(READ_ATTRIBUTES_AFE, RETRIEVE_AFE_ACCELEROMETER, 20);// Fake read amount  
  if( statusByte == SUCCESS ){
    // return attributes here
  }
    
}

// Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_MAX86140 (0x00)
// This function returns all registers and register values sequentially of the
// MAX86140/1 Sensors: register zero and register value zero to register n and 
// register value n.
// INCOMPLETE: Need to read datasheets to get exact amount of registers.
uint8_t SparkFun_Bio_Sensor_Hub::dumpRegisterMAX86140(){
  
  uint8_t statusByte = readByte(DUMP_REGISTERS, DUMP_REGISTER_MAX86140, 20); //Fake read amount
  if( statusByte == SUCCESS ){
    // return attributes here
  }

}

// Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_MAX30205 (0x01)
// This function returns all registers and register values sequentially of the
// MAX30205 sensor: register zero and register value zero to register n and 
// register value n.
// INCOMPLETE: Need to read datasheets to get exact amount of registers.
uint8_t SparkFun_Bio_Sensor_Hub::dumpRegisterMAX30205() {
  
  uint8_t statusByte = readByte(DUMP_REGISTERS, DUMP_REGISTER_MAX30205, 20); //Fake read amount
  if( statusByte == SUCCESS ){
    // return attributes here
  }

}

// Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_MAX30001 (0x02)
// This function returns all registers and register values sequentially of the
// MAX30001 sensor: register zero and register value zero to register n and 
// register value n.
// INCOMPLETE: Need to read datasheets to get exact amount of registers.
uint8_t SparkFun_Bio_Sensor_Hub::dumpRegisterMAX30001() {
  
  uint8_t statusByte = readByte(DUMP_REGISTERS, DUMP_REGISTER_MAX30001, 20); //Fake read amount
  if( statusByte == SUCCESS ){
    // return attributes here
  }

}

// Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_MAX30101 (0x03)
// This function returns all registers and register values sequentially of the
// MAX30101 sensor: register zero and register value zero to register n and 
// register value n.
// INCOMPLETE: Need to read datasheets to get exact amount of registers.
uint8_t SparkFun_Bio_Sensor_Hub::dumpRegisterMAX30101() {
  
  uint8_t statusByte = readByte(DUMP_REGISTERS, DUMP_REGISTER_MAX30101, 20); //Fake read amount
  if( statusByte == SUCCESS ){
    // return attributes here
  }

}

// Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_ACCELEROMETER (0x04)
// This function returns all registers and register values sequentially of the
// Accelerometer: register zero and register value zero to register n and 
// register value n.
// INCOMPLETE: Need to read datasheets to get exact amount of registers.
uint8_t SparkFun_Bio_Sensor_Hub::dumpRegisterAccelerometer() {
  
  uint8_t statusByte = readByte(DUMP_REGISTERS, DUMP_REGISTER_ACCELEROMETER, 20); //Fake read amount
  if( statusByte == SUCCESS ){
    // return attributes here
  }

}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_TARG_PERC (0x00), Write Byte: AGC_GAIN_ID (0x00) 
// This function sets the target percentage of the full-scale ADC range that
// the automatic gain control algorithm uses. It takes a paramater of zero to 
// 100 percent. 
bool SparkFun_Bio_Sensor_Hub::configALMrange(uint8_t perc){

  if( perc < 0 || perc > 100)
    return; 

  // Successful communication or no?
  uint8_t statusByte = writeRegister(CHANGE_ALGORITHM_CONFIG, SET_TARG_PERC, AGC_GAIN_ID, perc); 
  if( statusByte == SUCCESS )
    return true;
  else
    return false;

}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_STEP_SIZE (0x00), Write Byte: AGC_STEP_SIZE_ID (0x01) 
// This function chnages the step size toward the target for the AGC algorithm. 
// It takes a paramater of zero to 100 percent. 
bool SparkFun_Bio_Sensor_Hub::configALMStepSize(uint8_t step){

  if( perc < 0 || perc > 100)
    return false; 

  // Successful communication or no?
  uint8_t statusByte = writeRegister(CHANGE_ALGORITHM_CONFIG, SET_TARG_PERC, AGC_STEP_SIZE_ID, step); 
  if( statusByte == SUCCESS )
    return true; 
  else 
    return false; 

}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_SENSITIVITY (0x00), Write Byte: AGC_SENSITIVITY_ID (0x02)
// This function chnages the step size toward the target for the AGC algorithm. 
// It takes a paramater of zero to 100 percent. 
bool SparkFun_Bio_Sensor_Hub::configALMsensitivity(uint8_t sense){

  if( perc < 0 || perc > 100)
    return false; 

  // Successful communication or no?
  uint8_t statusByte = writeRegister(CHANGE_ALGORITHM_CONFIG, SET_TARG_PERC, AGC_SENSITIVITY_ID, sense); 
  if( statusByte == SUCCESS )
    return true; 
  else 
    return false; 

}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_AVG_SAMPLES (0x00), Write Byte: AGC_NUM_SAMP_ID (0x03)
// This function changes the number of samples that are averaged. 
// It takes a paramater of zero to 255. 
bool SparkFun_Bio_Sensor_Hub::configALMsensitivity(uint8_t avg){

  if( perc < 0 || perc > 100)
    return false; 

  // Successful communication or no?
  uint8_t statusByte = writeRegister(CHANGE_ALGORITHM_CONFIG, SET_TARG_PERC, AGC_SENSITIVITY_ID, avg); 
  if( statusByte == SUCCESS )
    return true; 
  else 
    return false; 

}

// This function uses the given family, index, and write byte to communicate
// with the MAX32664 which in turn communicates with downward sensors. There
// are two steps demonstrated in this function. First a write to the MCU
// indicating what you want to do, a delay, and then a read to confirm positive
// transmission. 
uint8_t SparkFun_Bio_Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte) {

  _i2cPort->beginTransmission(_address);     
  _i2cPort->write(_familyByte);    
  _i2cPort->write(_indexByte);    
  _i2cPort->write(_writeByte); 
  _i2cPort->endTransmission(); 
  delayMicroseconds(CMD_DELAY); 

  _i2cPort->requestFrom(_address, 1); // Status Byte, success or no? 0x00 is a successful transmit
  uint8_t statusByte = _i2cPort->read(); 
  return statusByte; 

}

// This function is the same as the function above and uses the given family, 
// index, and write byte, but also takes a 16 bit integer as a paramter to communicate
// with the MAX32664 which in turn communicates with downward sensors. There
// are two steps demonstrated in this function. First a write to the MCU
// indicating what you want to do, a delay, and then a read to confirm positive
// transmission. 
uint8_t SparkFun_Bio_Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint16_t _val) {

  _i2cPort->beginTransmission(_address);     
  _i2cPort->write(_familyByte);    
  _i2cPort->write(_indexByte);    
  _i2cPort->write(_writeByte); 
  _i2cPort->write((_val >> 8)); // MSB
  _i2cPort->write(_val);  // LSB
  _i2cPort->endTransmission(); 
  delayMicroseconds(CMD_DELAY); 

  _i2cPort->requestFrom(_address, 1); // Status Byte, success or no? 0x00 is a successful transmit
  uint8_t statusByte = _i2cPort->read(); 
  return statusByte; 

}

// This function sends information to the MAX32664 to specifically write values
// to the registers of downward sensors and so also requires a
// register address and register value as parameters. Again there is the write
// of the specific bytes followed by a read to confirm positive transmission. 
uint8_t SparkFun_Bio_Sensor_Hub::writeRegister(uint8_t _familyByte, uint8_t _indexByte, uint8_t _regAddr, uint8_t _regVal)
{

  _i2cPort->beginTransmission(_address);     
  _i2cPort->write(_familyByte);    
  _i2cPort->write(_indexByte);    
  _i2cPort->write(_regAddr);    
  _i2cPort->write(_regVal);    
  _i2cPort->endTransmission(); 
  delayMicroseconds(CMD_DELAY); 

  _i2cPort->requestFrom(_address, 1); // Status Byte, 0x00 is a successful transmit.
  uint8_t statusByte = _i2cPort->read(); 
  _i2cPort->endTransmission();
  return statusByte; 

}

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte, index byte, and
// delays 60 microseconds, during which the MAX32664 retrieves the requested 
// information. An I-squared-C request is then issued, and the information is read.
uint8_t * SparkFun_Bio_Sensor_Hub::readByte(uint8_t _familyByte, uint8_t _indexByte, uint16_t _numOfReads )
{

   uint8_t returnByte[_numOfReads]; 
  _i2cPort->beginTransmission(_address);
  _i2cPort->write(_familyByte);    
  _i2cPort->write(_indexByte);    
  _i2cPort->endTransmission();
  delayMicroseconds(CMD_DELAY); 

  _i2cPort->requestFrom(_address, _numOfReads); //Will always get a status byte
  for(int i = 0; i < _numOfReads; i++){
    returnByte[i] = _i2cPort->read(); //Status byte is always sent before read value
  }

  return returnByte; 

}

// This function is exactly as the one above except it accepts a Write Byte as
// a paramter. It starts a request by writing the family byte, index byte, and
// write byte to the MAX32664, delays 60 microseconds, during which
// the MAX32664 retrieves the requested information. A I-squared-C request is
// then issued, and the information is read.
uint8_t * SparkFun_Bio_Sensor_Hub::readByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint16_t _numOfReads )
{

   uint8_t returnByte[_numOfReads]; 
  _i2cPort->beginTransmission(_address);
  _i2cPort->write(_familyByte);    
  _i2cPort->write(_indexByte);    
  _i2cPort->write(_writeByte);    
  _i2cPort->endTransmission();
  delayMicroseconds(CMD_DELAY); 

  _i2cPort->requestFrom(_address, _numOfReads); //Will always get a status byte
  for(int i = 0; i < _numOfReads; i++){
    returnByte[i] = _i2cPort->read(); //Status byte is always sent before read value
  }

  return returnByte; 

}
