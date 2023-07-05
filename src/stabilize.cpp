#include "common.h"
#include "utils.h"
#include "state.h"
#include <PID_v1.h>
#include "controller.h"
#include "outputs.h"

// -----------------------
// Current State
float pitch = 0.0; // -1.0 to 1.0
float roll = 0.0;  // -1.0 to 1.0
float yaw = 0.0;   // -1.0 to 1.0

float pitchRate = 0.0; // Degress per second!?!?!?!?!?!?!?!?!?!?!?! CHECK THIS
float rollRate = 0.0;  // Degress per second!?!?!?!?!?!?!?!?!?!?!?! CHECK THIS
float yawRate = 0.0;   // Degress per second!?!?!?!?!?!?!?!?!?!?!?! CHECK THIS

// Output States
float outputPitch = 0.0;    // -1.0 to 1.0
float outputRoll = 0.0;     // -1.0 to 1.0
float outputYaw = 0.0;      // -1.0 to 1.0
float outputThrottle = 0.0; // 0.0 to 1.0

// -----------------------
// PIDs

// Define Variables we'll be connecting to
double pitchSetpoint, pitchInput, stabilizedPitchOutput;
double rollSetpoint, rollInput, stabilizedRollOutput;
double yawSetpoint, yawInput, stabilizedYawOutput;

// Specify the links and initial tuning parameters
double pitchKp = 0.2, pitchKi = 0.3, pitchKd = 0.05; 
double rollKp = 0.2, rollKi = 1.2, rollKd = 0.05;
double yawKp = 0.3, yawKi = 0.05, yawKd = 0.00011;

PID pitchPID(&pitchInput, &stabilizedPitchOutput, &pitchSetpoint, pitchKp, pitchKi, pitchKd, DIRECT);
PID rollPID(&rollInput, &stabilizedRollOutput, &rollSetpoint, rollKp, rollKi, rollKd, DIRECT);
PID yawPID(&yawInput, &stabilizedYawOutput, &yawSetpoint, yawKp, yawKi, yawKd, DIRECT);
// -----------------------

void setupPIDs()
{
    // Turn the PIDs on
    pitchPID.SetMode(AUTOMATIC);
    rollPID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);

    pitchPID.SetOutputLimits(-1.0, 1.0);
    rollPID.SetOutputLimits(-1.0, 1.0);
    yawPID.SetOutputLimits(-1.0, 1.0);

    pitchPID.SetSampleTime(2); // 2ms
    rollPID.SetSampleTime(2);
    yawPID.SetSampleTime(2);

}

void resetPIDs()
{
    pitchPID.SetMode(MANUAL);
    rollPID.SetMode(MANUAL);
    yawPID.SetMode(MANUAL);

    stabilizedPitchOutput = 0.0;
    stabilizedRollOutput = 0.0;
    stabilizedYawOutput = 0.0;

    pitchPID.SetMode(AUTOMATIC);
    rollPID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);
}

void updatePIDsAngle()
{
    pitchInput = (double)pitch;
    rollInput = (double)roll;
    yawInput = (double)yawRate / 360.0;

    pitchSetpoint = (double)commandedPitch * 0.5; // Limit to 90 degrees
    rollSetpoint = (double)commandedRoll * 0.5;   // Limit to 90 degrees
    yawSetpoint = (double)commandedYaw * 0.5;     // Not sure what to do here

    if (currentState == PASSIVE)
    {
        resetPIDs();
    }
    else
    {
        pitchPID.SetMode(AUTOMATIC);
        rollPID.SetMode(AUTOMATIC);
        yawPID.SetMode(AUTOMATIC);
    }

    pitchPID.Compute();
    rollPID.Compute();
    yawPID.Compute();
}

void updatePIDsRates()
{
    pitchInput = (double)pitchRate / 360.0;
    rollInput = (double)rollRate / 360.0;
    yawInput = (double)yawRate / 360.0;

    if (currentState == PASSIVE)
    {
        resetPIDs();
    }
    else
    {
        pitchSetpoint = (double)commandedPitch;
        rollSetpoint = (double)commandedRoll;
        yawSetpoint = (double)commandedYaw;

        pitchPID.SetMode(AUTOMATIC);
        rollPID.SetMode(AUTOMATIC);
        yawPID.SetMode(AUTOMATIC);
    }

    pitchPID.Compute();
    rollPID.Compute();
    yawPID.Compute();
}

void mixOutputs()
{
    if (currentState == PASSIVE)
    {
        outputPitch = commandedPitch * 0.8;
        outputRoll = commandedRoll * 0.7;
        outputYaw = commandedYaw * 0.9;
        outputThrottle = commandedThrottle;
        return;
    }

    // Update output pitch, roll, and yaw
    outputPitch = (float)stabilizedPitchOutput;
    outputRoll = (float)stabilizedRollOutput;
    
    // TODO: Figure out how to mix yaw
    outputYaw = (float)stabilizedYawOutput;
    // outputYaw = commandedYaw;

    // Update output throttle. Just passthrough for fixedwing.
    outputThrottle = commandedThrottle;
}

void stabilize()
{
    // Get latest MPU6050 data
    mpu.update();

    // // Update pitch, roll, and yaw
    pitch = fmap(mpu.getAngleY(), -180.0, 180.0, -1.0, 1.0);
    roll = fmap(mpu.getAngleX(), -180.0, 180.0, -1.0, 1.0);
    yaw = fmap(mpu.getAngleZ(), -180.0, 180.0, -1.0, 1.0);

    // Update pitch, roll, and yaw rates
    pitchRate = mpu.getGyroY();
    rollRate = mpu.getGyroX();
    yawRate = mpu.getGyroZ();

    // Update commanded pitch, roll, and yaw
    readRawControllerValues();

    // Update PIDs
    if (currentMode == ANGLE)
    {
        updatePIDsAngle();
    }
    else if (currentMode == RATES)
    {
        updatePIDsRates();
    }

    // Serial.println("Pitch: " + String(pitch) + " Roll: " + String(roll) + " Yaw: " + String(yaw));
    // Serial.println("Commanded Pitch: " + String(commandedPitch) + " Commanded Roll: " + String(commandedRoll) + " Commanded Yaw: " + String(commandedYaw));
    // Serial.println("Output Pitch: " + String(outputPitch) + " Output Roll: " + String(outputRoll) + " Output Yaw: " + String(outputYaw));
    // Serial.println("Rates:" + String(pitchRate) + " " + String(rollRate) + " " + String(yawRate));

    mixOutputs();

    positionServos();
}
