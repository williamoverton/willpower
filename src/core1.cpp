#include <Arduino.h>
#include "gps.h"
#include "storage.h"
#include "crossCore.h"
#include "utils.h"

#define FASTLED_FORCE_SOFTWARE_SPI // Dont use hardware SPI as this is used by the SD card.
#include <FastLED.h>

CRGB leds[1];

void core_setup1()
{
  // Setup Core2
  Serial.begin(9600);

  // Setup Storage
  setupStorage();

  // Setup LED
  FastLED.addLeds<WS2812, 16>(leds, 1);
  FastLED.showColor(CRGB(255, 0, 255));

  // Setup GPS
  setupGPS();

  delay(1000); // TODO: Is this needed?

  FastLED.showColor(CRGB(0, 255, 0));
  delay(100);
  FastLED.showColor(CRGB(0, 0, 255));
  delay(100);
  FastLED.showColor(CRGB(0, 255, 0));
  delay(100);
  FastLED.showColor(CRGB(0, 0, 255));
  delay(100);
  FastLED.showColor(CRGB(0, 255, 0));
  delay(100);
  FastLED.showColor(CRGB(0, 0, 255));
  delay(100);
  FastLED.showColor(CRGB(0, 255, 0));
  delay(100);
  FastLED.showColor(CRGB(0, 0, 255));
  delay(100);
  FastLED.showColor(CRGB(0, 255, 0));
  delay(100);
  FastLED.showColor(CRGB(0, 0, 255));
  delay(100);
}

static long lastUpdateCoreTime = 0;
static long coreUpdateInterval = 1000000 / 50; // Hz
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

void core_loop1()
{
  updateGPS();

  readFromOtherCore();
  sendCrossCoreData();

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