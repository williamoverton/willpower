#ifndef UTILS_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define UTILS_H

#define LOOP_RATE 500

void debug();
float fmap(float, float, float, float, float);
void printMPUOffsets();
float invSqrt(float x);
void printGPSInfo();
#endif