#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>

#define ENABLE 1 
#define DEF_ADDR 0x55

const int resPin = 4;
const int mfioPin = 5;
uint8_t* p = 0; //Null pointer, only possible value to assign to a pointer 
int result = 0; 

//reset pin, mfio pin
SparkFun_Bio_Sensor_Hub bioHub(DEF_ADDR, resPin, mfioPin); 

void setup(){

  Serial.begin(115200);

  Wire.begin();
  result = bioHub.begin();
  if (!result)
    Serial.println("Sensor Ready!");
  delay(1000);


  // Should be 60, returning 48 or 51
  result = bioHub.readRegisterMAX30101(0x07); 
  Serial.print("Max30101 FIFO: ");
  Serial.println(result);

  result = bioHub.setOutputMode(0x02); //ALGORITHM MODE
  Serial.print("Setting output mode: ");
  Serial.println(result);

  bioHub.setFIFOThreshold(0x0F); //2 samples - can hold 32
  bioHub.max30101Control(1); //enable
  Serial.print("Enable Fast ALgorithm: ");
  result = bioHub.whrmFastAlgorithmControl(1);
  delay(1000);
  Serial.println(result);
  
  bioHub.readSensorHubStatus();

  Serial.print("Number of samples in the Bio Sensor FIFO: ");
  result = bioHub.numSamplesOutFIFO();
  Serial.println(result);

 
  bioHub.readBPM(1);
  delay(2000);
  
}

void loop(){

  for (int i = 0; i < 5; i++){
    result = bioHub.readRegisterMAX30101(0x07); 
    Serial.print("Max30101 FIFO: ");
    Serial.println(result);

    result = bioHub.setOutputMode(0x02); //ALGORITHM MODE
    Serial.print("Setting output mode: ");
    Serial.println(result);

    bioHub.setFIFOThreshold(0x0F); //2 samples - can hold 32
    bioHub.max30101Control(1); //enable
    Serial.print("Enable Fast ALgorithm: ");
    result = bioHub.whrmFastAlgorithmControl(1);
    Serial.println(result);
    
    bioHub.readSensorHubStatus();

    Serial.print("Number of samples in the Bio Sensor FIFO: ");
    result = bioHub.numSamplesOutFIFO();//Figure this out.....
    Serial.println(result);

   
    bioHub.readBPM(1);
    delay(100);
  }
  while(1);
}
