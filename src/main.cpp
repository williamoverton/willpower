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
// #include "barometer.h"

// -----------------------
// Internal Variables
FlightState currentState;
FlightMode currentMode;
MPU6050 mpu(Wire);
// -----------------------

void setup()
{
  currentState = INIT;
  currentMode = ANGLE;
  
  // Initialize LED pin
  pinMode(LED_BUILTIN, OUTPUT);

  printWelcomeMessage();

  // Setup PPM
  setupPPM();

  // Initialize I2C for MPU6050
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

  // Initialize MPU6050
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  // Check if MPU6050 is connected
  while (status != 0)
  {
    Serial.println("MPU6050 error!");
    delay(1000);
  }

  // Calibrate MPU6050
  currentState = CALIBRATE;
  Serial.println("Starting calibration...");
  mpu.calcOffsets(true, true);
  Serial.println("Calibration complete!");

  // Setup servos
  setupServos();

  // Setup PIDs
  setupPIDs();

  // Setup barometer
  // setupBarometer(); // Nah lmao

  // Setup builtin button
  pinMode(RESET_MPU_BUTTON, INPUT_PULLUP);

  // Set current state to active as its GO TIME!
  currentState = ACTIVE;
}

void handleButtonPress() {
  if (digitalRead(RESET_MPU_BUTTON) == LOW) {
    Serial.println("Resetting MPU6050...");
    mpu.calcOffsets(true, true);
    Serial.println("Reset complete!");

    // Reset PIDs
    resetPIDs();
  }
}

void handleArmCheck() {
  FlightState previousState = currentState;

  if(commanedAux1 < 0.1) {
    currentState = PASSIVE;
  } else {
    currentState = ACTIVE;
  }

  if(previousState != currentState) {
    Serial.print("State changed to: ");
    Serial.println(currentState);
  }
}

void logic()
{
  handleButtonPress();
  stabilize();
  handleArmCheck();
}

void loop()
{
  debug();
  blink();

  logic();
}
