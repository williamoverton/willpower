#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "common.h"
#include "utils.h"
#include "mpu.h"
#include "state.h"
#include "controller.h"
#include "outputs.h"
#include "pins.h"
#include "stabilize.h"

static void limitLoopRate(unsigned long rateHz);

int ticks = 0;
long lastPrintTicks = 0;

FlightState currentState;
FlightMode currentMode;

void setup() {
  // Enable the Serial communication
  Serial.begin(115200);
  // Print the message to the Serial Monitor
  Serial.println("Hello World!");

  // Initialize the state
  currentState = INIT;
  currentMode = ANGLE;

  // Setup I2C
  // Wire.setSDA(SDA_PIN);
  // Wire.setSCL(SCL_PIN);
  // Wire.setClock(1000000);
  Wire.setClock(10);
  Wire.begin();
  Wire.setClock(1000000);
  
  setupMPU();

  // Setup servos
  setupServos();
  setupPIDs();

  // Setup internal LED
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  stabilize();
  
  // Serial.println(mpu.getAccX());

  // Print ticks every second
  if (millis() - lastPrintTicks > 1000)
  {
    Serial.println(">TPS:" + String(ticks));
    lastPrintTicks = millis();
    ticks = 0;
  }

  ticks++;

  limitLoopRate(2000);
}

void limitLoopRate(unsigned long rateHz)
{
  // Work in microseconds
  unsigned long period = 1000000 / rateHz;
  static unsigned long lastMicros = 0;
  unsigned long currentMicros = micros();
  unsigned long elapsedMicros = currentMicros - lastMicros;

  if (elapsedMicros < period)
  {
    delayMicroseconds(period - elapsedMicros);
  }

  lastMicros = micros();
}