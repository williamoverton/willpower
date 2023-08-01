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

float pitchRate = 0.0; // Whatever comes out the imu / 360. (CHECK THIS ONE DAY)
float rollRate = 0.0;  // Whatever comes out the imu / 360. (CHECK THIS ONE DAY)
float yawRate = 0.0;   // Whatever comes out the imu / 360. (CHECK THIS ONE DAY)

// Output States
float outputPitch = 0.0;    // -1.0 to 1.0
float outputRoll = 0.0;     // -1.0 to 1.0
float outputYaw = 0.0;      // -1.0 to 1.0
float outputThrottle = 0.0; // 0.0 to 1.0
float outputAux1 = 0.0;     // 0.0 to 1.0
float outputAux2 = 0.0;     // 0.0 to 1.0

// -----------------------
// PIDs

// Define Variables we'll be connecting to
double pitchSetpoint, pitchInput, stabilizedPitchOutput;
double rollSetpoint, rollInput, stabilizedRollOutput;
double yawSetpoint, yawInput, stabilizedYawOutput;

// Specify the links and initial tuning parameters
double pitchKp = 3.0, pitchKi = 1.0, pitchKd = 0.1;
double rollKp = 3.0, rollKi = 0.5, rollKd = 0.1;
double yawKp = 0.2, yawKi = 0.05, yawKd = 0.001;

PID pitchPID(&pitchInput, &stabilizedPitchOutput, &pitchSetpoint, pitchKp, pitchKi, pitchKd, DIRECT);
PID rollPID(&rollInput, &stabilizedRollOutput, &rollSetpoint, rollKp, rollKi, rollKd, DIRECT);
PID yawPID(&yawInput, &stabilizedYawOutput, &yawSetpoint, yawKp, yawKi, yawKd, DIRECT);
// -----------------------

// -----------------------
// Madgwick Filter
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
float dt;
long currentTime, prevTime;

float B_madgwick = 0.04 * 2; // Madgwick filter parameter
float B_accel = 0.14;        // Accelerometer LP filter paramter
float B_gyro = 0.14;         // Gyro LP filter paramter
float q0 = 1.0f;             // Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
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
    yawInput = (double)-yawRate;

    pitchSetpoint = (double)commandedPitch * 0.6; // Limit to *SOME* degrees
    rollSetpoint = (double)commandedRoll * 0.6;   // Limit to *SOME* degrees
    yawSetpoint = (double)((commandedYaw * 2.0)); //- (commandedRoll * 1.0)); // Mix yaw with roll to help with turns

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
    yawInput = (double)-yawRate;

    if (currentState == PASSIVE)
    {
        resetPIDs();
    }
    else
    {
        pitchSetpoint = (double)commandedPitch;
        rollSetpoint = (double)commandedRoll;
        yawSetpoint = (double)commandedYaw;
    }

    pitchPID.Compute();
    rollPID.Compute();
    yawPID.Compute();

    // Serial.print("Input: ");
    // Serial.print(pitchInput);
    // Serial.print(" Setpoint: ");
    // Serial.print(pitchSetpoint);
    // Serial.print(" Output: ");
    // Serial.println(stabilizedPitchOutput);
}

void handleMotorKillSwitchCheck()
{
    if (commandedAux2 > 0.8)
    {
        outputThrottle = 0.0;
    }
}

void mixOutputs()
{
    // outputAux1 is connected to Flaps.
    // We'll control it with commandedAux2 but we want it to move slowly and not in instant jumps.
    float flapSpeed = 0.001;
    if (commandedAux2 > outputAux1)
    {
        outputAux1 += flapSpeed;
    }
    else if (commandedAux2 < outputAux1)
    {
        outputAux1 -= flapSpeed * 1.5;
    }

    outputAux1 = constrain(outputAux1, 0.0, 1.0);

    /**
     * If we're in passive mode, we'll just passthrough the commanded values and return.
     */
    if (currentState == PASSIVE)
    {
        outputPitch = commandedPitch * 0.7;
        outputRoll = commandedRoll * 0.6;
        outputYaw = commandedYaw * 0.9;
        outputThrottle = commandedThrottle;
        return;
    }

    // Update output pitch, roll, and yaw
    outputPitch = (float)stabilizedPitchOutput;
    outputRoll = (float)stabilizedRollOutput;
    // outputRoll = fmap(rollRate, -180.0, 180.0, -1.0, 1.0);

    // TODO: Figure out how to mix yaw
    outputYaw = (float)stabilizedYawOutput;
    // outputYaw = commandedYaw;

    // Update output throttle. Just passthrough for fixedwing.
    outputThrottle = commandedThrottle;
}

float GyroX = 0.0;
float GyroY = 0.0;
float GyroZ = 0.0;
float prevGyroX = 0.0;
float prevGyroY = 0.0;
float prevGyroZ = 0.0;

float AccX = 0.0;
float AccY = 0.0;
float AccZ = 0.0;
float prevAccX = 0.0;
float prevAccY = 0.0;
float prevAccZ = 0.0;

void getIMUData()
{
    mpu.update();

    GyroX = mpu.getGyroX();
    GyroY = mpu.getGyroY();
    GyroZ = mpu.getGyroZ();

    AccX = mpu.getAccX();
    AccY = mpu.getAccY();
    AccZ = mpu.getAccZ();

    GyroX = (1.0 - B_gyro) * prevGyroX + B_gyro * GyroX;
    GyroY = (1.0 - B_gyro) * prevGyroY + B_gyro * GyroY;
    GyroZ = (1.0 - B_gyro) * prevGyroZ + B_gyro * GyroZ;

    AccX = (1.0 - B_accel) * prevAccX + B_accel * AccX;
    AccY = (1.0 - B_accel) * prevAccY + B_accel * AccY;
    AccZ = (1.0 - B_accel) * prevAccZ + B_accel * AccZ;

    prevGyroX = GyroX;
    prevGyroY = GyroY;
    prevGyroZ = GyroZ;

    prevAccX = AccX;
    prevAccY = AccY;
    prevAccZ = AccZ;
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

    // Divide by 131 to get degrees per second
    // https://github.com/TKJElectronics/KalmanFilter/blob/master/examples/MPU6050/MPU6050.ino#L116C28-L116C35
    float rateScaleAmount = 131.0;
    // Update pitch, roll, and yaw rates
    pitchRate = mpu.getGyroY() / rateScaleAmount;
    rollRate = mpu.getGyroX() / rateScaleAmount;
    yawRate = mpu.getGyroZ() / rateScaleAmount;

    // Update commanded pitch, roll, and yaw
    readRawControllerValues();

    // Update PIDs
    if (currentMode == ANGLE)
    {
        // Serial.println("Updating PIDs in angle mode");
        updatePIDsAngle();
    }
    else if (currentMode == RATES)
    {
        // Serial.println("Updating PIDs in rates mode");
        updatePIDsRates();
    }

    // Serial.println("Pitch: " + String(pitch) + " Roll: " + String(roll) + " Yaw: " + String(yaw));
    // Serial.println("Commanded Pitch: " + String(commandedPitch) + " Commanded Roll: " + String(commandedRoll) + " Commanded Yaw: " + String(commandedYaw));
    // Serial.println("Output Pitch: " + String(outputPitch) + " Output Roll: " + String(outputRoll) + " Output Yaw: " + String(outputYaw));
    // Serial.println("Rates:" + String(pitchRate) + " " + String(rollRate) + " " + String(yawRate));

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
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
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
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
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
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Compute angles
    roll = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29577951; // degrees
    pitch = -asin(-2.0f * (q1 * q3 - q0 * q2)) * 57.29577951;                // degrees
    yaw = -atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29577951; // degrees

    // Map to -1 to 1
    roll = fmap(roll, -180.0, 180.0, -1.0, 1.0);
    pitch = fmap(pitch, -180.0, 180.0, 1.0, -1.0);
    yaw = fmap(yaw, -180.0, 180.0, -1.0, 1.0);
}
