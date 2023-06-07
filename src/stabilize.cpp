#include "common.h"
#include "utils.h"
#include "state.h"
#include <Servo.h>
#include <PID_v1.h>
#include "controller.h"
#include "pins.h"

// -----------------------
// Current State
float pitch = 0.0; // -1.0 to 1.0
float roll = 0.0;  // -1.0 to 1.0
float yaw = 0.0;   // -1.0 to 1.0

float pitchRate = 0.0; // Degress per second!?!?!?!?!?!?!?!?!?!?!?! CHECK THIS
float rollRate = 0.0;  // Degress per second!?!?!?!?!?!?!?!?!?!?!?! CHECK THIS
float yawRate = 0.0;   // Degress per second!?!?!?!?!?!?!?!?!?!?!?! CHECK THIS

// Output States
float ouputPitch = 0.0; // -1.0 to 1.0
float ouputRoll = 0.0;  // -1.0 to 1.0
float ouputYaw = 0.0;   // -1.0 to 1.0

// Servos
Servo pitchServo;
Servo rollServo;
Servo yawServo;

// -----------------------
// PIDs

// Define Variables we'll be connecting to
double pitchSetpoint, pitchInput, stabilizedPitchOutput;
double rollSetpoint, rollInput, stabilizedRollOutput;
double yawSetpoint, yawInput, stabilizedYawOutput;

// Specify the links and initial tuning parameters
double pitchKp = 2, pitchKi = 0, pitchKd = 1;
double rollKp = 2, rollKi = 0, rollKd = 1;
double yawKp = 2, yawKi = 0, yawKd = 1;

PID pitchPID(&pitchInput, &stabilizedPitchOutput, &pitchSetpoint, pitchKp, pitchKi, pitchKd, DIRECT);
PID rollPID(&rollInput, &stabilizedRollOutput, &rollSetpoint, rollKp, rollKi, rollKd, DIRECT);
PID yawPID(&yawInput, &stabilizedYawOutput, &yawSetpoint, yawKp, yawKi, yawKd, DIRECT);
// -----------------------

void positionServos()
{
    // Position servos based on output pitch, roll, and yaw
    // Remap output pitch, roll and yaw to 0-180 degrees for servo
    pitchServo.write(constrain(int(fmap(ouputPitch, -1.0, 1.0, 0.0, 180.0)), 0, 180));
    rollServo.write(constrain(int(fmap(ouputRoll, -1.0, 1.0, 0.0, 180.0)), 0, 180));
    yawServo.write(constrain(int(fmap(ouputYaw, -1.0, 1.0, 0.0, 180.0)), 0, 180));
}

void setupPIDs()
{
    // Turn the PIDs on
    pitchPID.SetMode(AUTOMATIC);
    rollPID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);

    pitchPID.SetOutputLimits(-1.0, 1.0);
    rollPID.SetOutputLimits(-1.0, 1.0);
    yawPID.SetOutputLimits(-1.0, 1.0);
}

void resetPIDs()
{
    pitchSetpoint = 0.0;
    rollSetpoint = 0.0;
    yawSetpoint = 0.0;

    pitchPID.SetMode(MANUAL);
    rollPID.SetMode(MANUAL);
    yawPID.SetMode(MANUAL);
}

void updatePIDsAngle()
{
    pitchInput = (double)pitch;
    rollInput = (double)roll;
    yawInput = (double)yaw;

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

void updatePIDsRates()
{
    pitchInput = (double)pitchRate;
    rollInput = (double)rollRate;
    yawInput = (double)yawRate;

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

    // Update output pitch, roll, and yaw
    ouputPitch = (float)stabilizedPitchOutput;
    ouputRoll = (float)stabilizedRollOutput;
    ouputYaw = (float)stabilizedYawOutput;

    // Serial.println("Pitch: " + String(pitch) + " Roll: " + String(roll) + " Yaw: " + String(yaw));
    // Serial.println("Commanded Pitch: " + String(commandedPitch) + " Commanded Roll: " + String(commandedRoll) + " Commanded Yaw: " + String(commandedYaw));
    // Serial.println("Output Pitch: " + String(ouputPitch) + " Output Roll: " + String(ouputRoll) + " Output Yaw: " + String(ouputYaw));
    // Serial.println("Rates:" + String(pitchRate) + " " + String(rollRate) + " " + String(yawRate));

    positionServos();
}

void setupServos()
{
    pitchServo.attach(PITCH_SERVO_PIN);
}