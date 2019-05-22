#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>

#define ENABLE 1 
#define DEF_ADDR 0x55

uint8_t resPin = 7;
uint8_t mfioPin = 8;

//reset pin, mfio pin
SparkFun_Bio_Sensor_Hub bioHub(DEF_ADDR, resPin, mfioPin); 

void setup(){

  Serial.begin(115200);

  Wire.begin();
  uint8_t result = bioHub.begin();
  Serial.print("0x0");
  Serial.println(result); 
  if(bioHub.enableSensorMAX30101(ENABLE))
    Serial.println("Enabling sensor."); 
  readBPM(); 
    
}

void loop(){
}

