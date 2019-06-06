#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>

#define ENABLE 1 
#define DEF_ADDR 0x55

const int resPin = 7;
const int mfioPin = 8;
uint8_t* p = 0; //Null pointer, only possible value to assign to a pointer 

//reset pin, mfio pin
SparkFun_Bio_Sensor_Hub bioHub(DEF_ADDR, resPin, mfioPin); 

void setup(){

  Serial.begin(115200);

  Wire.begin();
  uint8_t result = bioHub.begin();
  Serial.print("0x0");
  Serial.println(result); 
  delay(1000);  

  result = bioHub.readRegisterMAX30101(0x07); 
  Serial.print("0x0");
  Serial.print(result, HEX);
  Serial.print(" : ");
  Serial.println(result);

  bioHub.setFIFOThreshold(0x02); //2 samples - can hold 32
  bioHub.max30101Control(1);
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
