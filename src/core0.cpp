#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include "common.h"
#include "utils.h"
#include "state.h"
#include "stabilize.h"
#include "controller.h"
#include "pins.h"
#include "outputs.h"
#include "crossCore.h"
// #include "barometer.h"

// -----------------------
// Internal Variables
FlightState currentState;
FlightMode currentMode;
MPU6050 mpu(Wire);

// -----------------------

static void limitLoopRate(int rateHz);

void core_setup0()
{
  Serial.begin(9600);

  currentState = INIT;
  currentMode = ANGLE;

  // Setup PPM
  setupPPM();

  // Initialize I2C for MPU6050
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  // Wire.setTimeout(100);
  Wire.begin();

  // Initialize MPU6050
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  // Check if MPU6050 is connected
  // #if !GROUND_MODE
  while (status != 0)
  {
    Serial.println("MPU6050 error!");
    delay(1000);
  }
  // #endif

  // Calibrate MPU6050
  currentState = CALIBRATE;
  Serial.println("Starting calibration...");
  mpu.calcOffsets(true, true);
  Serial.println("Calibration complete!");

  // while(true){
  // printMPUOffsets();
  //   delay(1000);
  //   Serial.println("");
  // }

  // mpu.setAccOffsets(0.03, 0.0, -0.58);
  // mpu.setGyroOffsets(-5.95, -0.91, -0.58);

  // Setup servos
  setupServos();

  // Setup PIDs
  setupPIDs();

  // Setup barometer
  // setupBarometer(); // Nah lmao

  // Set current state to active as its GO TIME!
  currentState = ACTIVE;
}

void handleArmCheck()
{
  FlightState previousState = currentState;

  if (commanedAux1 < 0.1)
  {
    currentState = PASSIVE;
  }
  else
  {
    currentState = ACTIVE;
  }

  if (previousState != currentState)
  {
    Serial.print("State changed to: ");
    Serial.println(currentState);
  }
}

// void handleModeCheck()
// {
//   FlightMode previousMode = currentMode;

//   if (commanedAux2 < 0.1)
//   {
//     currentMode = ANGLE;
//   }
//   else
//   {
//     currentMode = RATES;
//   }

//   if (previousMode != currentMode)
//   {
//     resetPIDs();

//     Serial.print("Mode changed to: ");
//     Serial.println(currentMode);
//   }
// }

void fly()
{
  stabilize();
  handleArmCheck();
  // handleModeCheck();
}

static long lastUpdateCoreTime = 0;
static long coreUpdateInterval = 1000000 / 100; // Hz
static int messageType = 0;
static void sendCrossCoreData()
{
  if (micros() - lastUpdateCoreTime < coreUpdateInterval)
  {
    return;
  }

  lastUpdateCoreTime = micros();

  if (messageType == 0)
  {
    writeToOtherCore("ANGLE," + String(pitch) + "," + String(roll) + "," + String(yaw));
  }
  else if (messageType == 1)
  {
    writeToOtherCore("RATES," + String(pitchRate) + "," + String(rollRate) + "," + String(yawRate));
  }
  else if (messageType == 2)
  {
    writeToOtherCore("STATE," + String(currentState));
  }
  else if (messageType == 3)
  {
    writeToOtherCore("MODE," + String(currentMode));
  }
  else if (messageType == 4)
  {
    writeToOtherCore("AUX," + String(commanedAux1) + "," + String(commanedAux2));
  }
  else if (messageType == 5)
  {
    writeToOtherCore("OUT," + String(outputPitch) + "," + String(outputRoll) + "," + String(outputYaw) + "," + String(outputThrottle));
  }
  else
  {
    writeToOtherCore("CTL," + String(commandedPitch) + "," + String(commandedRoll) + "," + String(commandedYaw) + "," + String(commandedThrottle));
    messageType = -1;
  }

  messageType++;
}

void core_loop0()
{
  debug();
  fly();
  readFromOtherCore();

  sendCrossCoreData();

  limitLoopRate(LOOP_RATE);
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