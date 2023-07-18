#ifndef COMMON_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define COMMON_H

// Dont require MPU6050 or Controller. Also never take off lol.
#define GROUND_MODE true

#include <MPU6050_light.h>

extern MPU6050 mpu;

#endif