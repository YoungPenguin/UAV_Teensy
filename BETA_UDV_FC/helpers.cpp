#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "helpers.h"

helpers::helpers() {

}
void helpers::discretize(float* terms, float* a, float* T) {
  Kp = &terms[0];
  Ki = &terms[1];
  Kd = &terms[2];
  sampleTime = T;
  float K = 2.0 / *sampleTime;

  *f1 = ((*Kp / *Ki) * K + 1 ) / ((*Kp / *Ki) * K);
  *f2 = (1 - (*Kp / *Ki) * K) / ((*Kp / *Ki) * K);
  *K1 = ((*Kd / *Kp) * K + 1) / (1 + *a * (*Kd / *Kp) * K);
  *K2 = (1 - (*Kd / *Kp) * K) / (1 + *a * (*Kd / *Kp) * K);
  *K3 = (1 - *a * (*Kd / *Kp) * K) / (1 + *a * (*Kd / *Kp) * K);
}

void helpers::dataVector(float* data) {
  for (unsigned int i = 0; i < sizeof(data); i++) {
    Serial.print(data[i]);
    Serial.print(", ");
  }
  Serial.println(data[sizeof(data)]);
}

unsigned int helpers::StickPos(unsigned int* a, int* input1, int* input2, int* input3, int* input4) {
  if ((*input1 < 1000) && (*input2 > 1700) && (*input3 > 1700) && (*input4 > 1700)) *a = 0;
  if ((*input1 < 1100) && (*input2 < 1100) && (*input3 > 1700) && (*input4 < 1100)) *a = 1;
  if ((*input1 < 1000) && (*input2 > 1700) && (*input3 < 1000) && (*input4 > 1700)) *a = 2;
  return *a;
}
