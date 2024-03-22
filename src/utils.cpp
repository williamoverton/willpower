#include<math.h>
#include <Arduino.h>
#include "common.h"
#include "stabilize.h"
#include "controller.h"

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void printAttitudes() {
  Serial.println(">Pitch:" + String(pitch));
  Serial.println(">Roll:" + String(roll));
  Serial.println(">Yaw:" + String(yaw));
}

void printCommandedValues() {
  Serial.println(">CommandedPitch:" + String(commandedPitch));
  Serial.println(">CommandedRoll:" + String(commandedRoll));
  Serial.println(">CommandedYaw:" + String(commandedYaw));
}

void printOutputs() {
  Serial.println(">OutputLeftMotor:" + String(outputLeftMotor));
  Serial.println(">OutputRightMotor:" + String(outputRightMotor));
  Serial.println(">OutputLeftavon:" + String(outputLeftavon));
  Serial.println(">OutputRightavon:" + String(outputRightavon));
}

long unsigned lastPrintDebugTime = 0;
long printDebugInterval = 100;

void printDebugInfo() {

  if (millis() - lastPrintDebugTime < printDebugInterval) {
    return;
  }

  printAttitudes();
  printCommandedValues();
  printOutputs();

  lastPrintDebugTime = millis();
}