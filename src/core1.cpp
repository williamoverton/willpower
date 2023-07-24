#include <Arduino.h>
#include "gps.h"
#include "storage.h"
#include "crossCore.h"
#include "utils.h"

#define FASTLED_FORCE_SOFTWARE_SPI // Dont use hardware SPI as this is used by the SD card.
#include <FastLED.h>

CRGB leds[1];

void helloBlink();
static void receiveCrossCoreData(char buffer[]);
static void limitLoopRate(int rateHz);

void core_setup1()
{
  // Setup Core2
  Serial.begin(9600);

  // Setup LED
  FastLED.addLeds<WS2812, 16>(leds, 1);
  FastLED.showColor(CRGB(255, 0, 255));

  // Setup GPS
  setupGPS();
  FastLED.showColor(CRGB::Orange);

  // Setup Storage
  setupStorage();

  // Setup Cross Core
  coreOneCallbackFunction = &receiveCrossCoreData;

  helloBlink();
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
    writeToOtherCore("LOC," + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6));
  }
  else if (messageType == 1)
  {
    writeToOtherCore("ALT," + String(gps.altitude.meters()));
  }
  else if (messageType == 2)
  {
    writeToOtherCore("GPS_STATUS," + String(gps.satellites.value()) + "," + String(gps.hdop.value()));
  }
  else
  {
    writeToOtherCore("COURSE," + String(gps.course.value()) + "," + String(gps.speed.kmph()));
    messageType = -1;
  }

  messageType++;
}

void updateLogInfo()
{
  logData.timeMillis = millis();
  logData.date = gps.date.isValid() ? String(gps.date.value()) : "INVALID";
  logData.time = gps.time.isValid() ? String(gps.time.value()) : "INVALID";

  logData.lat = gps.location.isValid() ? gps.location.lat() : -1;
  logData.lng = gps.location.isValid() ? gps.location.lng() : -1;
  logData.altitude = gps.altitude.isValid() ? gps.altitude.meters() : -1;

  logData.speedKmph = gps.speed.isValid() ? gps.speed.kmph() : -1;
  logData.courseDegrees = gps.course.isValid() ? gps.course.deg() : -1;

  logData.satellites = gps.satellites.isValid() ? gps.satellites.value() : -1;
  logData.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : -1;
  logData.locationAge = gps.location.isValid() ? gps.location.age() : -1;
}

void core_loop1()
{
  updateGPS();

  readFromOtherCore();
  sendCrossCoreData();

  updateLogInfo();
  writeLog();

  if (gps.satellites.value() > 0)
  {
    FastLED.showColor(CRGB(0, 255, 0));
  }
  else
  {
    FastLED.showColor(CRGB(255, 0, 0));
  }

  limitLoopRate(LOOP_RATE);
}

void helloBlink()
{
  int fadeSpeed = 5;

  for (int j = 0; j < 3; j++)
  {
    for (int i = 0; i < 255; i += fadeSpeed)
    {
      leds[0] = CRGB(i, 0, 255 - i);
      FastLED.show();
      delay(1);
    }

    for (int i = 0; i < 255; i += fadeSpeed)
    {
      leds[0] = CRGB(255 - i, 0, i);
      FastLED.show();
      delay(1);
    }
  }
}

void receiveCrossCoreData(char buffer[])
{
  // Example output: ANGLE,-0.13,0.16,0.56
  // Amount of entires changes.

  // split the data into an array
  char *entry = strtok(buffer, ",");

  // first entry is the message type
  String messageType = String(entry);

  if (messageType == "ANGLE")
  {
    double pitch = atof(strtok(NULL, ","));
    double roll = atof(strtok(NULL, ","));
    double yaw = atof(strtok(NULL, ","));

    logData.pitch = pitch;
    logData.roll = roll;
    logData.yaw = yaw;
  }
  if (messageType == "RATES")
  {
    double pitchRate = atof(strtok(NULL, ","));
    double rollRate = atof(strtok(NULL, ","));
    double yawRate = atof(strtok(NULL, ","));

    logData.pitchRate = pitchRate;
    logData.rollRate = rollRate;
    logData.yawRate = yawRate;
  }
  else
  {
    // Serial.print("Unknown message type: ");
    // Serial.println(messageType);
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