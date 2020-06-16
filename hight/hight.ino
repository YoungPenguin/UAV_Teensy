#include <Wire.h>
#include "SparkFunMPL3115A2.h"

volatile int cycles;
//Create an instance of the object
MPL3115A2 myPressure;
float Height;
void setup()
{
  Wire.begin();        // Join i2c bus
  Serial.begin(9600);  // Start serial for output
  myPressure.begin(); // Get sensor online
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa

  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128 from datasheet (select 7)
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

void loop() {
   uint32_t startCycleCPU;
  startCycleCPU = ARM_DWT_CYCCNT;
  float pressure = myPressure.readPressure();
  float temperature = 25+273.15;// = myPressure.readTemp() + 273.15;

  Height = -(log(pressure / 100900) * 8.3143 * temperature) / (0.28401072);
  cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;

  while (cycles < 720000) {
    cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;
      Serial.println(millis());

 
  }
  


}
