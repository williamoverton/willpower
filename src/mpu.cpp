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

float B_accel = 0.14; // Accelerometer LP filter paramter
float B_gyro = 0.1;   // Gyro LP filter paramter

float AccErrorX = 0.08;
float AccErrorY = -0.00;
float AccErrorZ = -0.04;
float GyroErrorX = -0.64;
float GyroErrorY = 0.46;
float GyroErrorZ = 0.51;

// float AccErrorX = 0;
// float AccErrorY = 0;
// float AccErrorZ = 0;
// float GyroErrorX = 0;
// float GyroErrorY = 0;
// float GyroErrorZ = 0;


void getIMUData()
{
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

  // long time = micros();
  mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
  // long time2 = micros() - time;
  // Serial.println(">MPU_TIME:" + String(time2));

  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel)*prevAccX + B_accel*AccX;
  AccY = (1.0 - B_accel)*prevAccY + B_accel*AccY;
  AccZ = (1.0 - B_accel)*prevAccZ + B_accel*AccZ;
  prevAccX = AccX;
  prevAccY = AccY;
  prevAccZ = AccZ;

  //Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;

  //LP filter gyro data
  GyroX = (1.0 - B_gyro)*prevGyroX + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*prevGyroY + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*prevGyroZ + B_gyro*GyroZ;
  prevGyroX = GyroX;
  prevGyroY = GyroY;
  prevGyroZ = GyroZ;
}

void calculateIMUError()
{
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY = 0.0;
  GyroErrorZ = 0.0;

  // Read IMU values 12000 times
  int c = 0;
  while (c < 12000)
  {
    mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

    AccX = AcX / ACCEL_SCALE_FACTOR;
    AccY = AcY / ACCEL_SCALE_FACTOR;
    AccZ = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;

    // Sum all readings
    AccErrorX = AccErrorX + AccX;
    AccErrorY = AccErrorY + AccY;
    AccErrorZ = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  // Divide the sum by 12000 to get the error value
  AccErrorX = AccErrorX / c;
  AccErrorY = AccErrorY / c;
  AccErrorZ = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");

  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");

  while (true)
  {
    delay(1000);
  }
}