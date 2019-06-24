#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>

#define ENABLE 1 
#define DEF_ADDR 0x55

const int resPin = 4;
const int mfioPin = 5;
uint8_t* p = 0; //Null pointer, only possible value to assign to a pointer 
int result = 0; 
int total = 0; 


//reset pin, mfio pin
SparkFun_Bio_Sensor_Hub bioHub(DEF_ADDR, resPin, mfioPin); 

void setup(){

  Serial.begin(115200);

  Wire.begin();
  result = bioHub.begin();
  if (!result)
    Serial.println("Sensor Ready!");
  delay(1000);
  

  // If status is 24, then FIFO has overflowed and data was lost. 
  for ( int i = 0; i < 10; i ++ ){

    Serial.print("Status: ");
    result = bioHub.readSensorHubStatus();
    Serial.println(result); 

    Serial.print("Number of samples in the Bio Sensor FIFO: ");
    result = bioHub.numSamplesOutFIFO();
    Serial.println(result);

   
    bioHub.readBPM();
    delay(1000);
  }
  
}

void loop(){
}
