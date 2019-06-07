#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>

#define ENABLE 1 
#define DEF_ADDR 0x55

const int resPin = 4;
const int mfioPin = 5;
uint8_t* p = 0; //Null pointer, only possible value to assign to a pointer 

//reset pin, mfio pin
SparkFun_Bio_Sensor_Hub bioHub(DEF_ADDR, resPin, mfioPin); 

void setup(){

  Serial.begin(115200);

  Wire.begin();
  uint8_t result = bioHub.begin();
  if (!result)
    Serial.println("Sensor Ready!");
  delay(1000);

  // Should be 60, returning 48
  result = bioHub.readRegisterMAX30101(0x07); 
  Serial.print("Max30101 FIFO: ");
  Serial.println(result);

  bioHub.setFIFOThreshold(0x02); //2 samples - can hold 32
  bioHub.max30101Control(1); //enable
  bioHub.whrmFastAlgorithmControl(1);

  Serial.print("Status after enabling algorithms and max30101: "); 
  result = bioHub.readSensorHubStatus();
  Serial.println(result);

  Serial.print("Number of samples in the Bio Sensor FIFO: ");
  result = bioHub.numSamplesOutFIFO();
  Serial.println(result);

  
}

void loop(){
}
