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

  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
*/


#include "SparkFun_Bio_Sensor_Hub_Library.h"

SparkFun_Bio_Sensor_Hub::SparkFun_Bio_Sensor_Hub( uint8_t resetPin, uint8_t mfioPin ) 
{ 
  _resetPin = resetPin; 
  _mfioPin = mfioPin;
  pinMode(_resetPin, OUTPUT); // Set these pins as output
  // May also receive information over this line in which case it should be set as input: 
  pinMode(_mfioPin, OUTPUT); 
}

// Begin does not need an  I-squared-C address
// because there is only two, one for reading and another for writing, and they
// are not configureable. The reset and MFIO pin are required to set the board
// into application and bootloader mode. 
bool SparkFun_Bio_Sensor_Hub::begin( TwoWire &wirePort )
{
  _i2cPort = &wirePort;
  //  _i2cPort->begin(); A call to Wire.begin should occur in sketch 
  //  to avoid multiple begins with other sketches.

  // To enter the bio-sensor hub into 'application' mode we need to hold the
  // reset pin low while we set the MFIO pin high. After 10ms the reset pin
  // then needs to be pushed high. After 50ms the MAX32664 will be in
  // Application mode. I then set the pins back to output so that they are not
  // held uneccesarily in these states but are instead pulled high by their
  // internal resistors. 
  digitalWrite(_mfioPin, LOW); 
  delay(1); // Just want to ensure the pin is LOW before we continue
  digitalWrite(_resetPin, LOW); 
  digitalWrite(_mfioPin, HIGH); 
  delay(10); 
  digitalWrite(_resetPin, HIGH); 
  delay(50); //Application mode is enabled when this ends 
  pinMode(_resetPin, OUTPUT); 
  pinMode(_mfioPin, OUTPUT); 

  // Let's check to see if the device made it into application mode.  
  uint8_t responseByte = readByte(READ_DEVICE_MODE, 0x00, 0x00, 1); 
  if(responseByte != APP_MODE)
    return true;
  else
    return false; 
}

// This funcation allows the Bio-sensor HUB to enter into bootloader mode which
// allows the user to program it. This is achieved by toggling the reset and
// MFIO pin in a different order than what is used for application mode. 
bool SparkFun_Bio_Sensor_Hub::beginBootloader( TwoWire &wirePort ) 
{
  _i2cPort = &wirePort; 

  // Bootloader mode is selected by writing the MFIO pin LOW while the reset pin
  // is low, and then after 10ms writing the resetPin high. After 50ms the bio
  // will be in 'bootloader' mode. 
  digitalWrite(_mfioPin, HIGH);
  delay(1); // Just want to ensure the pin is HIGH before we continue
  digitalWrite(_resetPin, LOW); 
  digitalWrite(_mfioPin, LOW); 
  delay(10); 
  digitalWrite(_resetPin, HIGH); 
  delay(50);  //Bootloader mode is enabled when this ends.  
  pinMode(_resetPin, OUTPUT); 
  pinMode(_mfioPin, OUTPUT); 
  
  // Let's check to see if the device made it into bootloader mode.  
  uint8_t responseByte = readByte(READ_DEVICE_MODE, 0x00, 0x00, 1); 
  if(responseByte != BOOTLOADER_MODE)
    return true; 
  else
    return false; 

}

// Skipping this for now. How does this differ from starting the device in
// bootloader or application mode?
bool SparkFun_Bio_Sensor_Hub::setDeviceMode(uint8_t boot_mode) 
{

}

bool SparkFun_Bio_Sensor_Hub::setOutputMode(uint8_t outputType)
{
  if (outputType < 0x00 || outputType > 0x07) // Bytes between PAUSE and SENSOR_ALM_COUNTER
    return false; 

  //
  uint8_t responseByte = writeByte(OUTPUT_FORMAT, SET_FORMAT, outputType);  

  if( responseByte != SUCCESS)
    return false; 
  else
    return true; 
}

// Takes value between 0-255
bool SparkFun_Bio_Sensor_Hub::setFIFOThreshold(uint8_t intThresh) {
  if( intThresh < 0 || intThresh > 255)
    return false; 

  uint8_t responseByte = writeByte(OUTPUT_FORMAT, SET_THRESHOLD, intThresh); 
  if( responseByte != SUCCESS)
    return false; 
  else
    return true; 

}

uint8_t SparkFun_Bio_Sensor_Hub::numSamplesOutFIFO(){

  uint8_t sampAvail = readByte(READ_DATA_OUTPUT, NUM_SAMPLES, NO_WRITE, 1); 
  return sampAvail;

}

uint8_t SparkFun_Bio_Sensor_Hub::getDataOutFIFO(){

  uint8_t sampAvail = readByte(READ_DATA_OUTPUT, NUM_SAMPLES, NO_WRITE, 1); 
  uint8_t dataAvail = readByte(READ_DATA_OUTPUT, READ_DATA, NO_WRITE, sampAvail); 
  return dataAvail; //pointer instead?

}

// This function adds support for the acceleromter that is NOT included on
// SparkFun's product, The Family Registery of 0x13 and 0x14 is skipped for now. 
uint8_t SparkFun_Bio_Sensor_Hub::numSamplesExternalSensor(){

  uint8_t sampAvail = readByte(READ_DATA_INPUT, SAMPLE_SIZE, ACCELEROMETER, 1); 
  return sampAvail;

}

bool SparkFun_Bio_Sensor_Hub::writeRegisterMAX861X(uint8_t regAddr, uint8_t regVal)
{
  // Multiple writes, adjust writeRegister function
  uint8_t writeStat = writeRegister(WRITE_REGISTER, WRITE_MAX86140, WRITE_MAX86140_ID, regAddr, regVal);
  if( writeStat == SUCCESS) 
    return true; 
  else
    return false; 
}

bool SparkFun_Bio_Sensor_Hub::writeRegisterMAX30205(uint8_t regAddr, uint8_t regVal)
{
  // Multiple writes, adjust writeRegister function
  uint8_t writeStat = writeRegister(WRITE_REGISTER, WRITE_MAX30205, WRITE_MAX30205_ID, regAddr, regVal);
  if( writeStat == SUCCESS) 
    return true; 
  else
    return false; 
}

bool SparkFun_Bio_Sensor_Hub::writeRegisterMAX30001(uint8_t regAddr, uint8_t regVal)
{
  // Multiple writes, adjust writeRegister function
  uint8_t writeStat = writeRegister(WRITE_REGISTER, WRITE_MAX30001, WRITE_MAX30001_ID, regAddr, regVal);
  if( writeStat == SUCCESS) 
    return true; 
  else
    return false; 
}

bool SparkFun_Bio_Sensor_Hub::writeRegisterMAX30101(uint8_t regAddr, uint8_t regVal)
{
  // Multiple writes, adjust writeRegister function
  uint8_t writeStat = writeRegister(WRITE_REGISTER, WRITE_MAX30101, WRITE_MAX30101_ID, regAddr, regVal);
  if( writeStat == SUCCESS) 
    return true; 
  else
    return false; 
}

bool SparkFun_Bio_Sensor_Hub::writeRegisterAccel(uint8_t regAddr, uint8_t regVal)
{
  // Multiple writes, adjust writeRegister function
  uint8_t writeStat = writeRegister(WRITE_REGISTER, WRITE_ACCELEROMETER, WRITE_ACCELEROMETER_ID, regAddr, regVal);
  if( writeStat == SUCCESS) 
    return true; 
  else
    return false; 
}

uint8_t SparkFun_Bio_Sensor_Hub::readRegisterMAX8614X(uint8_t regAddr){

  uint8_t regCont = readByte(READ_REGISTER, READ_MAX86140, regAddr, 1); 
  return regCont;
}

uint8_t SparkFun_Bio_Sensor_Hub::readRegisterMAX30205(uint8_t regAddr){

  uint8_t regCont = readByte(READ_REGISTER, READ_MAX30205, regAddr, 1); 
  return regCont;
}

uint8_t SparkFun_Bio_Sensor_Hub::readRegisterMAX30001(uint8_t regAddr){

  uint8_t regCont = readByte(READ_REGISTER, READ_MAX30001, regAddr, 1); 
  return regCont;
}

uint8_t SparkFun_Bio_Sensor_Hub::readRegisterMAX30101(uint8_t regAddr){

  uint8_t regCont = readByte(READ_REGISTER, READ_MAX30101, regAddr, 1); 
  return regCont;
}

uint8_t SparkFun_Bio_Sensor_Hub::readRegisterAccel(uint8_t regAddr){

  uint8_t regCont = readByte(READ_REGISTER, READ_ACCELEROMETER, regAddr, 1); 
  return regCont;
}


uint8_t SparkFun_Bio_Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte)
{
  _i2cPort->beginTransmission(WRITE_ADDRESS);     
  _i2cPort->write(_familyByte);    
  _i2cPort->write(_indexByte);    
  _i2cPort->write(0x00);    
  _i2cPort->write(_writeByte); //multiple writes?
  _i2cPort->endTransmission(); 
  delayMicroseconds(CMD_DELAY); 

  _i2cPort->requestFrom(READ_ADDRESS, 1); //Status Byte
  uint8_t statusByte = _i2cPort->read(); 
  _i2cPort->endTransmission();
  return statusByte; 
}

uint8_t SparkFun_Bio_Sensor_Hub::writeRegister(uint8_t _familyByte, uint8_t _indexByte, uint8_t _regAddr, uint8_t _regVal)
{
  _i2cPort->beginTransmission(WRITE_ADDRESS);     
  _i2cPort->write(_familyByte);    
  _i2cPort->write(_indexByte);    
  _i2cPort->write(0x00);    
  _i2cPort->write(_regAddr);    
  _i2cPort->write(_regVal);    
  _i2cPort->endTransmission(); 
  delayMicroseconds(CMD_DELAY); 

  _i2cPort->requestFrom(READ_ADDRESS, 1); //Status Byte
  uint8_t statusByte = _i2cPort->read(); 
  _i2cPort->endTransmission();
  return statusByte; 
}

// Some reads requre a writeByte
uint8_t SparkFun_Bio_Sensor_Hub::readByte(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _numOfReads )
{
  _i2cPort->beginTransmission(WRITE_ADDRESS);
  _i2cPort->write(_familyByte);    
  _i2cPort->write(_indexByte);    
  _i2cPort->endTransmission();
  delayMicroseconds(CMD_DELAY); 
  _i2cPort->requestFrom(READ_ADDRESS, numOfReads + 1); //Will always get a status byte
  uint8_t statusByte = _i2cPort->read(); //Status byte is alwasy sent before read value

  if (!statusByte) { // The status byte is zero upon success
    while(_i2cPort->available){
   // How many of these, how will these sepcified, let's see if any function
   // requires multiples read bytes.  
    uint8_t someVal = _i2cPort->read();
    _i2cPort->endTransmission();
    return someVal; 
  }
  else {
    _i2cPort->endTransmission();
    return statusByte; 
  }

}

