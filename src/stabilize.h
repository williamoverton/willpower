#ifndef STABILIZE_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define STABILIZE_H

void stabilize();
void setupServos();
void setupPIDs();

// -----------------------
// Current State
extern float pitch;
extern float roll;
extern float yaw;

// -----------------------

// Output States
extern float ouputPitch; // -1.0 to 1.0
extern float ouputRoll; // -1.0 to 1.0
extern float ouputYaw; // -1.0 to 1.0
// -----------------------

#endif