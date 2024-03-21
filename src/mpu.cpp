#include <MPU6050.h>
#include <Arduino.h>
#include "common.h"

MPU6050 mpu;

void setupMPU()
{
    mpu.initialize();

    // Check if MPU6050 is connected
    // #if !GROUND_MODE
    while (mpu.testConnection() == false)
    {
        Serial.println("MPU6050 error!");
        delay(1000);
    }

    mpu.setFullScaleGyroRange(GYRO_SCALE);
    mpu.setFullScaleAccelRange(ACCEL_SCALE);
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

float B_accel = 0.14;    // Accelerometer LP filter paramter
float B_gyro = 0.1;      // Gyro LP filter paramter

void getIMUData()
{
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

    long time = micros();
    mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    long time2 = micros() - time;
    Serial.println(">MPU_TIME:" + String(time2));

    // Accelerometer
    AccX = AcX / ACCEL_SCALE_FACTOR; // G's
    AccY = AcY / ACCEL_SCALE_FACTOR;
    AccZ = AcZ / ACCEL_SCALE_FACTOR;

    // Correct the outputs with the calculated error values
    // AccX = AccX - AccErrorX;
    // AccY = AccY - AccErrorY;
    // AccZ = AccZ - AccErrorZ;

    // LP filter accelerometer data
    AccX = (1.0 - B_accel) * prevAccX + B_accel * AccX;
    AccY = (1.0 - B_accel) * prevAccY + B_accel * AccY;
    AccZ = (1.0 - B_accel) * prevAccZ + B_accel * AccZ;

    // Gyro
    GyroX = GyX / GYRO_SCALE_FACTOR; // deg/sec
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    // Correct the outputs with the calculated error values
    // GyroX = GyroX - GyroErrorX;
    // GyroY = GyroY - GyroErrorY;
    // GyroZ = GyroZ - GyroErrorZ;

    // LP filter gyro data
    GyroX = (1.0 - B_gyro) * prevGyroX + B_gyro * GyroX;
    GyroY = (1.0 - B_gyro) * prevGyroY + B_gyro * GyroY;
    GyroZ = (1.0 - B_gyro) * prevGyroZ + B_gyro * GyroZ;

    prevGyroX = GyroX;
    prevGyroY = GyroY;
    prevGyroZ = GyroZ;

    prevAccX = AccX;
    prevAccY = AccY;
    prevAccZ = AccZ;
}