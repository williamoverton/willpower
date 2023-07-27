#ifndef STABILIZE_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define STABILIZE_H

void stabilize();
void setupPIDs();
void resetPIDs();
void calibrateAttitude();

// -----------------------
// Current State
extern float pitch;
extern float roll;
extern float yaw;

extern float pitchRate;
extern float rollRate;
extern float yawRate;

// -----------------------

// Output States
extern float outputPitch; // -1.0 to 1.0
extern float outputRoll; // -1.0 to 1.0
extern float outputYaw; // -1.0 to 1.0
extern float outputThrottle; // 0.0 to 1.0
extern float outputAux1; // 0.0 to 1.0
extern float outputAux2; // 0.0 to 1.0
// -----------------------

#endif