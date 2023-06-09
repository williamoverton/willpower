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
double pitchKp = 0.96, pitchKi = 1.44, pitchKd = 0.24;
double rollKp = 0.96, rollKi = 1.44, rollKd = 0.24;
double yawKp = 1.4, yawKi = 0.24, yawKd = 0.00072;

PID pitchPID(&pitchInput, &stabilizedPitchOutput, &pitchSetpoint, pitchKp, pitchKi, pitchKd, DIRECT);
PID rollPID(&rollInput, &stabilizedRollOutput, &rollSetpoint, rollKp, rollKi, rollKd, DIRECT);
PID yawPID(&yawInput, &stabilizedYawOutput, &yawSetpoint, yawKp, yawKi, yawKd, DIRECT);
// -----------------------

void positionServos()
{
    const float maxServoAngle = 180.0;
    const float minServoAngle = 0.0;

    // Serial.print("Output Pitch: ");
    // Serial.print(ouputPitch);
    // Serial.print("\t Commanded:");
    // Serial.print(commandedPitch);
    // Serial.print("\t Stab:");
    // Serial.print(stabilizedPitchOutput);
    // Serial.print("\t Servo:");
    // Serial.println(fmap(ouputPitch, -1.0, 1.0, minServoAngle, maxServoAngle));

    // Position servos based on output pitch, roll, and yaw
    // Remap output pitch, roll and yaw to 0-180 degrees for servo
    pitchServo.write(constrain(int(fmap(ouputPitch, -1.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));
    rollServo.write(constrain(int(fmap(ouputRoll, -1.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));
    yawServo.write(constrain(int(fmap(ouputYaw, -1.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));
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
    yawInput = (double)yawRate;

    pitchSetpoint = (double)commandedPitch;
    rollSetpoint = (double)commandedRoll;
    yawSetpoint = (double)commandedYaw;

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

void mixOutputs()
{
    // Update output pitch, roll, and yaw
    ouputPitch = (float)stabilizedPitchOutput;
    ouputRoll = (float)stabilizedRollOutput;
    ouputYaw = (float)stabilizedYawOutput;
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
    // Serial.println("Output Pitch: " + String(ouputPitch) + " Output Roll: " + String(ouputRoll) + " Output Yaw: " + String(ouputYaw));
    // Serial.println("Rates:" + String(pitchRate) + " " + String(rollRate) + " " + String(yawRate));

    mixOutputs();

    positionServos();
}

void setupServos()
{
    pitchServo.attach(PITCH_SERVO_PIN);
}