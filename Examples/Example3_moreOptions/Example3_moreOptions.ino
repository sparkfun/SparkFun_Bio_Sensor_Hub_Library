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

sensorAttr attributes;
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

  Serial.println(bioHub.max30101Control(1));
  Serial.println();
  Serial.println(bioHub.readMAX30101State());
  Serial.println(bioHub.setAlgoRange(80));
  Serial.println(bioHub.setAlgoStepSize(20));
  Serial.println(bioHub.setAlgoSensitivity(20));
  Serial.println(bioHub.setAlgoSamples(10));
  Serial.println(bioHub.readAlgoRange());
  Serial.println(bioHub.readAlgoStepSize());
  Serial.println(bioHub.readAlgoSensitivity());
  Serial.println(bioHub.readAlgoSamples());

  while(1);

}

void loop(){

}
