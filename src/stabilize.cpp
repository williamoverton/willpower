#include <Arduino.h>
#include <QuickPID.h>
#include "common.h"
#include "utils.h"
#include "state.h"
#include "controller.h"
#include "outputs.h"
#include "mpu.h"

// -----------------------
// Current State
float pitch = 0.0; // -1.0 to 1.0
float roll = 0.0;  // -1.0 to 1.0
float yaw = 0.0;   // -1.0 to 1.0

// Output States
float outputLeftMotor = 0;  // -1.0 to 1.0
float outputRightMotor = 0; // -1.0 to 1.0
float outputLeftavon = 0;   // -1.0 to 1.0
float outputRightavon = 0;  // 0.0 to 1.0
float outputAux1 = 0;       // 0.0 to 1.0
float outputAux2 = 0;       // 0.0 to 1.0

// -----------------------
// PIDs

// Define Variables we'll be connecting to
float pitchSetpoint = 0, pitchInput = 0, stabilizedPitchOutput = 0;
float rollSetpoint = 0, rollInput = 0, stabilizedRollOutput = 0;
float yawSetpoint = 0, yawInput = 0, stabilizedYawOutput = 0;

// Specify the links and initial tuning parameters
float pitchKp = 3.3, pitchKi = 1.0, pitchKd = 1.0;
float rollKp = 1.3, rollKi = 0.5, rollKd = 0.4;
float yawKp = 0.45, yawKi = 0.05, yawKd = 0.0003;

QuickPID pitchPID(&pitchInput, &stabilizedPitchOutput, &pitchSetpoint);
QuickPID rollPID(&rollInput, &stabilizedRollOutput, &rollSetpoint);
QuickPID yawPID(&yawInput, &stabilizedYawOutput, &yawSetpoint);
// -----------------------

// -----------------------
// Madgwick Filter
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
float dt;
long currentTime, prevTime;

float B_madgwick = 0.04; // Madgwick filter parameter
float q0 = 1.0f;         // Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
// -----------------------

void setupPIDs()
{
    pitchPID.SetOutputLimits(-1.0, 1.0);
    rollPID.SetOutputLimits(-1.0, 1.0);
    yawPID.SetOutputLimits(-1.0, 1.0);

    pitchPID.SetSampleTimeUs(500);
    rollPID.SetSampleTimeUs(500);
    yawPID.SetSampleTimeUs(500);

    // Turn the PIDs on
    pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
    rollPID.SetTunings(rollKp, rollKi, rollKd);
    yawPID.SetTunings(yawKp, yawKi, yawKd);

    pitchPID.SetMode(QuickPID::Control::automatic);
    rollPID.SetMode(QuickPID::Control::automatic);
    yawPID.SetMode(QuickPID::Control::automatic);
}

void resetPIDs()
{
    Serial.println("Resetting PIDs");
    pitchPID.SetMode(QuickPID::Control::automatic);
    rollPID.SetMode(QuickPID::Control::automatic);
    yawPID.SetMode(QuickPID::Control::automatic);

    stabilizedPitchOutput = 0.0;
    stabilizedRollOutput = 0.0;
    stabilizedYawOutput = 0.0;
}

void updatePIDsAngle()
{
    pitchInput = (double)pitch;
    rollInput = (double)roll;
    yawInput = (double)-GyroZ / 131;

    pitchSetpoint = (double)commandedPitch * 0.2;
    rollSetpoint = (double)commandedRoll * 0.4;
    yawSetpoint = (double)commandedYaw;

    if (commandedThrottle < 0.1)
    {
        pitchPID.pTerm = 0.0;
        pitchPID.iTerm = 0.0;
        pitchPID.dTerm = 0.0;
        pitchPID.SetOutputSum(0.0);
        
        rollPID.pTerm = 0.0;
        rollPID.iTerm = 0.0;
        rollPID.dTerm = 0.0;
        rollPID.SetOutputSum(0.0);

        yawPID.pTerm = 0.0;
        yawPID.iTerm = 0.0;
        yawPID.dTerm = 0.0;
        yawPID.SetOutputSum(0.0);
    }

    pitchPID.Compute();
    rollPID.Compute();
    yawPID.Compute();

    Serial.println(">PitchITerm: " + String(pitchPID.GetIterm()));
}

void handleMotorKillSwitchCheck()
{
    if (commandedAux2 > 0.8)
    {
        outputLeftMotor = 0.0;
        outputRightMotor = 0.0;
    }
}

void mixOutputs()
{
    outputLeftMotor = commandedThrottle + stabilizedRollOutput;
    outputRightMotor = commandedThrottle - stabilizedRollOutput;

    if (commandedThrottle < 0.01)
    {
        outputLeftMotor = 0.0;
        outputRightMotor = 0.0;
    }

    bool isHovering = commandedAux1 < 0.1;

    outputLeftavon = -stabilizedPitchOutput;
    outputRightavon = stabilizedPitchOutput;

    if (isHovering)
    {
        outputLeftavon += stabilizedYawOutput;
        outputRightavon += stabilizedYawOutput;
    }
    else
    {
        outputLeftavon += -stabilizedRollOutput;
        outputRightavon += -stabilizedRollOutput;
    }

    // outputLeftavon = 0;
    // outputRightavon = 0;

    Serial.println(">stabilizedPitchOutput: " + String(stabilizedPitchOutput));
    Serial.println(">pitchSetpoint: " + String(pitchSetpoint));
    Serial.println(">pitch: " + String(pitch));
    // Serial.println(">stabilizedRollOutput: " + String(stabilizedRollOutput));
}

void stabilize()
{
    prevTime = currentTime;
    currentTime = micros();
    dt = (currentTime - prevTime) / 1000000.0;

    // Get latest MPU6050 data
    getIMUData();

    // // Update pitch, roll, and yaw
    Madgwick6DOF(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, dt);

    // Update commanded pitch, roll, and yaw
    readRawControllerValues();

    updatePIDsAngle();

    mixOutputs();

    handleMotorKillSwitchCheck();

    positionServos();
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq)
{

    // DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
    /*
     * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
     * available (for example when using the recommended MPU6050 IMU for the default setup).
     */
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = 1.0 / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = 1.0 / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= B_madgwick * s0;
        qDot2 -= B_madgwick * s1;
        qDot3 -= B_madgwick * s2;
        qDot4 -= B_madgwick * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    // Normalise quaternion
    recipNorm = 1.0 / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    bool isHovering = commandedAux1 < 0.1;

    // Compute angles
    if (!isHovering)
    { // cruise mode
        // compute angles (std aircraft: psi, theta, phi)In the IMU defined axes NOT AC stability axes!!
        roll = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29577951; // degrees
        pitch = -asin(-2.0f * (q1 * q3 - q0 * q2)) * 57.29577951;                // degrees
        yaw = -atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29577951; // degrees
    }
    else
    { // hover mode
        // compute angles (reverse phi,theta rotation order for tailsitter in hover: psi, phi, theta)In the IMU defined axes NOT AC stability axes!!
        pitch = atan2(q1 * q3 - q0 * q2, 0.5f - (q1 * q1 + q2 * q2)) * 57.29577951 - 90.0f; // degrees, zero is nose vertical!
        roll = asin(2.0f * (q0 * q1 + q2 * q3)) * 57.29577951;                              // degrees
        yaw = -atan2(-q1 * q2 + q0 * q3, 0.5f - (q1 * q1 + q3 * q3)) * 57.29577951;         // degrees;
        // also need to swap roll and yaw gyro data!!
        float temp = GyroZ;
        GyroZ = GyroX;
        GyroX = -temp;
    }

    // Map to -1 to 1
    roll = fmap(roll, -180.0, 180.0, -1.0, 1.0);
    pitch = fmap(pitch, -180.0, 180.0, 1.0, -1.0);
    yaw = fmap(yaw, -180.0, 180.0, -1.0, 1.0);
}