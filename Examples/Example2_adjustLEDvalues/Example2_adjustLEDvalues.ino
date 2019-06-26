/*
 This example sketch first demonstrates how to retrieve the values of the
 sensor's LEDs: RED, GREEN, and IR. It then shows you how to adjust the pulse
 width of these LEDs. 
 This board requires I-squared-C connections but also connections to the reset
 and mfio pins. When using the device keep LIGHT and CONSISTENT pressure on the
 sensor. Otherwise you may crush the capillaries in your finger which results
 in bad or no results. A summary of the hardware connections are as follows: 
 SDA -> SDA
 SCL -> SCL
 RESET -> PIN 4
 MFIO -> PIN 5

 Author: Elias Santistevan
 Date: 7/2019
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

// Takes address, reset pin, and MFIO pin.
SparkFun_Bio_Sensor_Hub bioHub(DEF_ADDR, resPin, mfioPin); 

ledData led;  
// ^^^^^^^^^
// What's this!? This is a type (like int, byte, long) unique to the SparkFun
// Pulse Oximeter and Heart Rate Monitor. Unlike those other types it holds
// specific information on the LED count value of the sensor. "ledData" is 
// actually a specific kind of type, known as a "struct". I think "led" is a
// good variable name for the ledData type, but you can choose whatever. 
// When used in the following way it gives access to the corresponding data:
// led.irLed  - Infrared LED counts. 
// led.redLed - Red LED counts. 

void setup(){

  Serial.begin(115200);

  Wire.begin();
  int result = bioHub.begin();
  if (!result)
    Serial.println("Sensor started!");
 
  Serial.println("Configuring Sensor...."); 
  int error = bioHub.beginMaxSensor();
  if(!error){
    Serial.println("Sensor configured.");
  }
  else {
    Serial.println("Error configuring sensor.");
    Serial.print("Error: "); 
    Serial.println(error); 
  }
}

void loop(){

    // Information from the readSensor function will be saved to our "led"
    // variable.  
    led = bioHub.readSensor();
    Serial.print("Infrared LED counts: ");
    Serial.println(led.irLed); 
    Serial.print("Red LED counts: ");
    Serial.println(led.redLed); 
    delay(250); // Slowing it down, we don't need to break our necks here.
}
