#include<math.h>
#include <Arduino.h>
#include "common.h"
#include "stabilize.h"

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}

// void printMPUOffsets()
// {
//     Serial.print(F("Accelero X: "));
//     Serial.print(mpu.getAccXoffset());
//     Serial.print(F("\tY: "));
//     Serial.print(mpu.getAccYoffset());
//     Serial.print(F("\tZ: "));
//     Serial.println(mpu.getAccZoffset());

//     Serial.print(F("Gyro     X: "));
//     Serial.print(mpu.getGyroXoffset());
//     Serial.print(F("\tY: "));
//     Serial.print(mpu.getGyroYoffset());
//     Serial.print(F("\tZ: "));
//     Serial.println(mpu.getGyroZoffset());
// }
