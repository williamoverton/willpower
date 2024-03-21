#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include "common.h"
#include "utils.h"
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
MPU6050 mpu(Wire);

void setup() {
  // Enable the Serial communication
  Serial.begin(115200);
  // Print the message to the Serial Monitor
  Serial.println("Hello World!");

  // Initialize the state
  currentState = INIT;
  currentMode = ANGLE;


  // Setup I2C
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.setClock(1000000);
  Wire.begin();

  byte status = mpu.begin(0, 0); // 500 deg/s / +-4g
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  // Check if MPU6050 is connected
  // #if !GROUND_MODE
  while (status != 0)
  {
    Serial.println("MPU6050 error!");
    delay(1000);
  }

   // Initialize MPU6050
  // Forward
  // mpu.setAccOffsets(0.02, 0.00, -0.57);
  // mpu.setGyroOffsets(-6.10, -0.79, -0.76);

  // Upwards
  // mpu.setAccOffsets(0.53, 0.02, -1.07);
  // mpu.setGyroOffsets(-6.22, -0.66, -0.78);

  mpu.calcOffsets(true, true);
  // Serial.println("Calibration complete!");

  // while(true) {
  //   printMPUOffsets();
  //   delay(1000);
  // }

  // Setup servos
  setupServos();
  setupPIDs();

  // Setup internal LED
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // // Blink
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(100);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(100);

  // Serial.println("Hello World!!!");

  // limitLoopRate(2000);

  stabilize();
  
  // Serial.println(mpu.getAccX());

  // Print ticks every second
  if (millis() - lastPrintTicks > 1000)
  {
    Serial.println(">TPS:" + String(ticks));
    Serial.println(">Pitch:" + String(pitch));

    lastPrintTicks = millis();
    ticks = 0;
  }

  ticks++;
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