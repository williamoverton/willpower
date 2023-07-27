#ifndef CONTROLLER_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define CONTROLLER_H

void readRawControllerValues();
void setupPPM();

extern float commandedPitch;
extern float commandedRoll;
extern float commandedYaw;
extern float commandedThrottle;

extern float commandedAux1;
extern float commandedAux2;

#endif