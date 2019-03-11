/*
  This is a library for the SparkFun Pulse Oximeter and Heart Rate Monitor
  By: Elias Santistevan
  SparkFun Electronics
  Date: March, 2019
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
*/


#include "SparkFun_Bio_Sensor_HUB.h"

// As far as I know the constructor does not need an I-squared-C address
// because there is only two, one for reading and another for writing, and they
// are not configureable. The reset and MFIO pin are required to set the board
// into application and bootloader mode. 
SparkFun_Bio_Sensor_HUB::SparkFun_Bio_Sensor_HUB(i2cAddress address, uint8_t resetPin, uint8_t mfioPin) 
{ 
  _address = address; 
  _resetPin = resetPin; 
  _mfioPin = mfioPin;
  pinMode(_resetPin, OUTPUT); // Set these pins as output
  // May also receive information over this line in which case it should be set as input: 
  pinMode(_mfioPin, OUTPUT); 

}

bool SparkFun_Bio_Sensor_HUB::begin( TwoWire &wirePort )
{
  _i2cPort = &wirePort;
  //  _i2cPort->begin(); A call to Wire.begin should occur in sketch 
  //  to avoid multiple begins with other sketches.

  // To enter the bio-sensor hub into 'application' mode we need to hold the
  // reset pin low while we set the MFIO pin high. After 10ms the reset pin
  // then needs to be pushed high. After 50ms the MAX32664 will be in
  // Application mode. I then set the pins back to output so that they are not
  // held uneccesarily in some strange state. 
  digitalWrite(_mfioPin, LOW); 
  delay(1); // Just want to ensure the pin is LOW before we continue
  digitalWrite(_resetPin, LOW); 
  digitalWrite(_mfioPin, HIGH); 
  delay(10); 
  digitalWrite(_resetPin, HIGH); 
  delay(50); //Application mode is enabled when this ends 
  pinMode(_resetPin, OUTPUT); 
  pinMode(_mfioPin, OUTPUT); 

  // Here let's test that communication with the sensor is ok. 
  _i2cPort->beginTransmission(_address);
  uint8_t _ret = _i2cPort->endTransmission();
  if(!_ret)
    return true;
  else
    return false; 
}

// This funcation allows the Bio-sensor HUB to enter into bootloader mode which
// allows the user to program it. This is achieved by toggling the reset and
// MFIO pin in a different order than what is used for application mode. 
bool SparkFun_Bio_Sensor_HUB:: beginBootloader( TwoWire &wirePort ) 
{
  _i2cport = &wirePort; 

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

  //Not sure if this is possible just yet. 
//  _i2cPort->beginTransmission(_address);
//  uint8_t _ret = _i2cPort->endTransmission();
//  if(!_ret)
//    return true;
//  else
//    return false; 

}

uint8_t SparkFun_Bio_Sensor_HUB::writeRegister(uint8_t _familyByte, uint8_t _mask, uint8_t _bits, uint8_t _indexByte)
{
  _i2cPort->beginTransmission(WRITE_ADDRESS);     
  _i2cPort->write(_familyByte);    
  _i2cPort->write(_indexByte);    
  _i2cPort->write(0x00);    
  _i2cPort->(_bits); 
  _i2cPort->endTransmission(); 
  delay(); //Not sure of this yet
  uint8_t statusByte = readRegister();  
  // Do I want this byte to be 
  return statusByte; 
  
}


uint8_t SparkFun_Bio_Sensor_HUB::readRegister(uint8_t _familyByte, uint8_t indexByte)
{
  _i2cPort->beginTransmission(WRITE_ADDRESS);
  _i2cPort->write(_familyByte);    
  _i2cPort->write(_indexByte);    
  _i2cPort->endTransmission();
  _i2cPort->beginTransmission(READ_ADDRESS);
  uint8_t statusByte = _i2cPort->read(); //Status byte is alwasy sent before read occurs
  if(!statusByte){ // The status byte is zero upon success
    uint8_t someVal = _i2cPort->read(); //How many of these, how will these sepcified. 
    _i2cPort->endTransmission();
    return someVal; 
  }
  if(statusByte){ //Upon error return the error. 
    _i2cPort->endTransmission();
    return statusByte; 
  }
}

