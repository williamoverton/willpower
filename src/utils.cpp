#include <Arduino.h>
#include "state.h"
#include "common.h"
#include "barometer.h"
#include "stabilize.h"
#include "gps.h"

// -----------------------
// Tinker Variables
long unsigned int printInterval = 1000; // ms
// -----------------------

// -----------------------
// Internal Variables

// Logging
long unsigned int _lastPrintTime = 0;
unsigned _clockTicks = 0;

// -----------------------

void printWelcomeMessage()
{
    Serial.begin(9600);
    Serial.println("-----------------------------------------------");
    Serial.println("WILKOMEN TO THE WILL POWER FLIGHT CONTROLLER!");
    Serial.println("HOLD ON TO YOUR BUTTS!");
    Serial.println("-----------------------------------------------");
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long unsigned int _lastBlinkTime = 0;
void blink()
{
    unsigned int printInterval = 0;

    if (currentState == INIT)
    {
        printInterval = 1000;
    }
    else if (currentState == CALIBRATE)
    {
        printInterval = 300;
    }
    else if (currentState == PASSIVE)
    {
        printInterval = 2000;
    }
    else if (currentState == ACTIVE)
    {
        printInterval = 50;
    }

    if (millis() - _lastBlinkTime > printInterval)
    {
        _lastBlinkTime = millis();

        // Toggle built in LED
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}

void PrintMPUData()
{
    Serial.print(F("TEMPERATURE: "));
    Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO  X: "));
    Serial.print(mpu.getAccX());
    Serial.print("\tY: ");
    Serial.print(mpu.getAccY());
    Serial.print("\tZ: ");
    Serial.println(mpu.getAccZ());

    Serial.print(F("GYRO      X: "));
    Serial.print(mpu.getGyroX());
    Serial.print("\tY: ");
    Serial.print(mpu.getGyroY());
    Serial.print("\tZ: ");
    Serial.println(mpu.getGyroZ());

    Serial.print(F("ACC ANGLE X: "));
    Serial.print(mpu.getAccAngleX());
    Serial.print("\tY: ");
    Serial.println(mpu.getAccAngleY());

    Serial.print(F("ANGLE     X: "));
    Serial.print(mpu.getAngleX());
    Serial.print("\tY: ");
    Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");
    Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));
}

void PrintStabilizeData() 
{
    Serial.print(F("Pitch: "));
    Serial.print(pitch);
    Serial.print(F("\tRoll: "));
    Serial.print(roll);
    Serial.print(F("\tYaw: "));
    Serial.println(yaw);

    Serial.print(F("Pitch Rate: "));
    Serial.print(pitchRate);
    Serial.print(F("\tRoll Rate: "));
    Serial.print(rollRate);
    Serial.print(F("\tYaw Rate: "));
    Serial.println(yawRate);

}

void printMPUOffsets()
{
    Serial.print(F("Accelero X: "));
    Serial.print(mpu.getAccXoffset());
    Serial.print(F("\tY: "));
    Serial.print(mpu.getAccYoffset());
    Serial.print(F("\tZ: "));
    Serial.println(mpu.getAccZoffset());

    Serial.print(F("Gyro     X: "));
    Serial.print(mpu.getGyroXoffset());
    Serial.print(F("\tY: "));
    Serial.print(mpu.getGyroYoffset());
    Serial.print(F("\tZ: "));
    Serial.println(mpu.getGyroZoffset());
}

void printGPSInfo()
{
    Serial.print("Satellites: ");
    gps.satellites.isValid() ? Serial.print(gps.satellites.value()) : Serial.print(F("INVALID"));

    Serial.print(F(" Location: "));

    if (gps.location.isValid())
    {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(" Location Age: ");
    Serial.print(gps.location.age());

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
        Serial.print(gps.date.year());
        Serial.print(F("/"));
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.println();
}

void debug()
{
    _clockTicks++;

    if (millis() - _lastPrintTime > printInterval)
    {
        // PrintMPUData();
        PrintStabilizeData();

        // printBarometerData();

        Serial.print("Ticks Per Second: ");
        Serial.println(_clockTicks / (printInterval / 1000.0));

        _lastPrintTime = millis();
        _clockTicks = 0;
    }
}

void limitLoopRate(int rateHz)
{
  static unsigned long lastLoopTime = 0;
  unsigned long loopTime = micros();
  unsigned long loopRate = 1000000 / (loopTime - lastLoopTime);

  if (loopRate > rateHz)
  {
    delayMicroseconds(1000000 / rateHz - (loopTime - lastLoopTime));
  }

  lastLoopTime = micros();
}

float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}