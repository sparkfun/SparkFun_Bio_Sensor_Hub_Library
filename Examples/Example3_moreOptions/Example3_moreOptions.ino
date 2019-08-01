/*
 This example sketch first demonstrates how to retrieve the values of the
 sensor's LEDs: RED, GREEN, and IR. It then shows you how to adjust the pulse
 width of these LEDs. The pulse width leads to longer exposure and so more
 accurate, though slower readings. Check SparkFun's hookup guide
 (https://sparkfun.com/) for a table of pulse width and respective available
 sample settings. This board requires I-squared-C connections but also 
 connections to the reset and MFIO pins. When using the device keep LIGHT and 
 CONSISTENT pressure on the sensor. Otherwise you may crush the capillaries 
 in your finger which results in bad or no results. A summary of the hardware 
 connections are as follows: 
 SDA -> SDA
 SCL -> SCL
 RESET -> PIN 4
 MFIO -> PIN 5

 Author: Elias Santistevan
 Date: 8/2019
 SparkFun Electronics

 If you run into an error code check the following table to help diagnose your
 problem: 
 1 = Unavailable Command
 2 = Unavailable Function
 3 = Data Format Error
 4 = Input Value Error
 5 = Try Again
 255 = Error Unknown
*/

#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>

// No other Address options.
#define DEF_ADDR 0x55

// Reset pin, MFIO pin
const int resPin = 4;
const int mfioPin = 5;

// Possible widths: 69, 118, 215, 411us
int width = 411; 
// Possible samples: 50, 100, 200, 400, 800, 1000, 1600, 3200 samples/second
// Not every sample amount is possible with every width; check out our hookup
// guide for more information.
int samples = 400; 
int pulseWidthVal;
int sampleVal;

// Takes address, reset pin, and MFIO pin.
SparkFun_Bio_Sensor_Hub bioHub(DEF_ADDR, resPin, mfioPin); 

bioData body; 
// ^^^^^^^^^
// What's this!? This is a type (like "int", "byte", "long") unique to the SparkFun
// Pulse Oximeter and Heart Rate Monitor. Unlike those other types it holds
// specific information on the LED count values of the sensor and ALSO the
// biometric data: heart rate, oxygen levels, and confidence. "bioData" is 
// actually a specific kind of type, known as a "struct". I chose the name
// "body" but you could use another variable name like "blood", "readings",
// "ledBody" or whatever. Using the variable in the following way gives the
// following data: 
// body.irLed  - Infrared LED counts. 
// body.redLed - Red LED counts. 
// body.heartrate - Heartrate
// body.confidence - Confidence in the heartrate value
// body.oxygen - Blood oxygen level
// body.status - Has a finger been sensed?

void setup(){

  Serial.begin(115200);

  Wire.begin();
  int result = bioHub.begin();
  if (!result)
    Serial.println("Sensor started!");


  // Set pulse width.
 // int error = bioHub.setPulseWidth(width);
 // if (!error){
 //   Serial.println("Pulse Width Set.");
 // }
 // else {
 //   Serial.println("Could not set Pulse Width.");
 //   Serial.print("Error: "); 
 //   Serial.println(error); 
 // }

 // // Check that the pulse width was set. 
 // pulseWidthVal = bioHub.readPulseWidth();
 // Serial.print("Pulse Width: ");
 // Serial.println(pulseWidthVal);

 // // Set sample rate per second. Remember that not every sample rate is
 // // available with every pulse width. Check hookup guide for more information.  
 // error = bioHub.setSampleRate(samples);
 // if (!error){
 //   Serial.println("Sample Rate Set.");
 // }
 // else {
 //   Serial.println("Could not set Sample Rate!");
 //   Serial.print("Error: "); 
 //   Serial.println(error); 
 // }

 // // Check sample rate.
 // sampleVal = bioHub.readSampleRate();
 // Serial.print("Sample rate is set to: ");
 // Serial.println(sampleVal); 
 
  if(!bioHub.bptMedicine(1))
    Serial.println("Blood pressure medicine settting set.");
  else
    Serial.println("Blood pressure medicine setting NOT set.");

  // Set the max height in cm of the persons you'll be monitoring. 
  if(!bioHub.setUserResting(1))
    Serial.println("User Resting Setting Set");

  // Set the max height in cm of the persons you'll be monitoring. 
  if(!bioHub.setWhrmMaxHeight(177))
    Serial.println("Maximum Height Set!");
  int heightVal = bioHub.readWhrmMaxHeight();
  Serial.print("Maximum height set to: ");    
  Serial.println(heightVal);

  // Set the default height in cm of the persons you'll be monitoring. 
  if(!bioHub.setWhrmDefHeight(165))
    Serial.println("Default Height Set!");
  heightVal = bioHub.readWhrmDefHeight();
  Serial.print("Default height set to: ");    
  Serial.println(heightVal);

  // Set the min height in cm of the persons you'll be monitoring. 
  if(!bioHub.setWhrmMinHeight(152))
    Serial.println("Minimum Height Set!");
  heightVal = bioHub.readWhrmMinHeight();
  Serial.print("Minimum height set to: ");    
  Serial.println(heightVal);

  // Set the max weight in kg of the persons you'll be monitoring. 
  if(!bioHub.setWhrmMaxWeight(91))
    Serial.println("Maximum Weight Set!");
  int weightVal = bioHub.readWhrmMaxWeight();
  Serial.print("Maximum weight set to: ");    
  Serial.println(weightVal);

  // Set the default weight in kg of the persons you'll be monitoring. 
  if(!bioHub.setWhrmDefWeight(175))
    Serial.println("Maximum Weight Set!");
  weightVal = bioHub.readWhrmDefWeight();
  Serial.print("Maximum weight set to: ");    
  Serial.println(weightVal);

  // Set the min weight in kg of the persons you'll be monitoring. 
  if(!bioHub.setWhrmMinWeight(68))
    Serial.println("Maximum Weight Set!");
  weightVal = bioHub.readWhrmMaxWeight();
  Serial.print("Maximum weight set to: ");    
  Serial.println(weightVal);

  // Set the max age in years of the persons you'll be monitoring. 
  if(!bioHub.setWhrmMaxAge(35))
    Serial.println("Maximum Age Set!");
  int ageVal = bioHub.readWhrmMaxAge();
  Serial.print("Maximum Age set to: ");    
  Serial.println(ageVal);
  
  // Set the min age in years of the persons you'll be monitoring. 
  if(!bioHub.setWhrmMinAge(25))
    Serial.println("Minimum Age Set!");
  ageVal = bioHub.readWhrmMinAge();
  Serial.print("Minimum Age set to: ");    
  Serial.println(ageVal);

  // Set the default age in years of the persons you'll be monitoring. 
  if(!bioHub.setWhrmDefAge(25))
    Serial.println("Default Age Set!");
  ageVal = bioHub.readWhrmDefAge();
  Serial.print("Default Age set to: ");    
  Serial.println(ageVal);
  Serial.println(bioHub.readAlgoRange());
  Serial.println(bioHub.readAlgoStepSize());
  Serial.println(bioHub.readAlgoSensitivity());
  Serial.println(bioHub.readAlgoSamples());
  Serial.println(bioHub.readSkinDetect());
  Serial.println(bioHub.readPhotoDetPer());
  Serial.println(bioHub.readScdWindow());
  Serial.println(bioHub.readPDCurrent());
  Serial.println(bioHub.readPpgSource());
  Serial.println(bioHub.readWSP02SampRate());
  Serial.println(bioHub.readMotionDetect());
  Serial.println(bioHub.readMotionDetecPer());
  Serial.println(bioHub.readMotThresh());
  // Now that everything is configured.....
  Serial.println("Configuring Sensor...."); 
  int error = bioHub.configBpm(); // Configure Sensor and BPM mode 
  if(!error){
    Serial.println("Sensor configured.");
  }
  else {
    Serial.println("Error configuring sensor.");
    Serial.print("Error: "); 
    Serial.println(error); 
  }
 
  Serial.println(bioHub.readAgcMode());
  while(1);

}

void loop(){

}
