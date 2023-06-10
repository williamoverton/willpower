#include <Arduino.h>
#include "state.h"
#include "common.h"
#include "barometer.h"
#include "stabilize.h"

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
