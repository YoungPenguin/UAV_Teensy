#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "helpers.h"

helpers::helpers() {}

void helpers::dataVector(float* data) {
  for (uint8_t i = 0; i < sizeof(data); i++) {
    Serial.print(data[i]);
    Serial.print(", ");
  }
  Serial.println(data[sizeof(data)]);
}

unsigned int helpers::StickPos(uint8_t* a, int* input1, int* input2, int* input3, int* input4) {
  if ((*input1 < 1000) && (*input2 > 1700) && (*input3 > 1700) && (*input4 > 1700)) *a = 0;
  if ((*input1 < 1100) && (*input2 < 1100) && (*input3 > 1700) && (*input4 < 1100)) *a = 1;
  if ((*input1 < 1000) && (*input2 > 1700) && (*input3 < 1000) && (*input4 > 1700)) *a = 2;
  return *a;
}
