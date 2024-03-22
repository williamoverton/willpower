#ifndef MPU_H // To make sure you don't declare the function more than once by including the header multiple times.
#define MPU_H

#include <MPU6050.h>

extern MPU6050 mpu;

void setupMPU();
void getIMUData();
void calculateIMUError();

extern float GyroX;
extern float GyroY;
extern float GyroZ;

extern float AccX;
extern float AccY;
extern float AccZ;

#endif