#include <Arduino.h>
#include <SPI.h>
#include <SDFS.h>
#include "pins.h"
#include "gps.h"

#define SHOULD_LOG_STORAGE true

static File f;
static bool sdCardWorking = false;
static long logInterval = 100; // ms
static String filename;

String buildLogHeader()
{
    String line = "";

    line += "TimeMillis,";
    line += "Date,Time,";
    line += "Lat,Lng,";
    line += "Altitude,";
    line += "SpeedKmph,";
    line += "CourseDegrees,";
    line += "Satellites,";
    line += "HDOP,";
    line += "LocationAge";

    return line;
}

void setupSDCard()
{
    SPI1.setRX(SD_SPI1_MISO);
    SPI1.setCS(SD_SPI1_CSn);
    SPI1.setSCK(SD_SPI1_SCK);
    SPI1.setTX(SD_SPI1_MOSI);

#if SHOULD_LOG_STORAGE
    Serial.println("SPI1 initialized.");
    Serial.println("Initializing SD card...");
#endif

    SDFSConfig c2;
    c2.setSPI(SPI1);
    c2.setAutoFormat(true);
    c2.setCSPin(SD_SPI1_CSn);
    SDFS.setConfig(c2);

    bool beginRes = SDFS.begin();

    delay(100);

#if SHOULD_LOG_STORAGE
    Serial.println("SD card config set.");

    if (beginRes)
    {
        Serial.println("SD card initialized.");
    }
    else
    {
        Serial.println("SD card initialization failed.");
    }

#endif

    f = SDFS.open(filename, "w");

    if (!f)
    {
#if SHOULD_LOG_STORAGE
        Serial.println("File open failed.");
#endif
        return;
    }
    else
    {
        f.println(buildLogHeader());
        sdCardWorking = true;
    }

    f.close();

#if SHOULD_LOG_STORAGE
    Serial.println("SD Card setup complete! :)");
#endif
}

void setupStorage()
{
    filename = "/log.csv";
    setupSDCard();
}

String buildLogMessage()
{
    String line = "";

    line += String(millis());
    line += ",";

    line += gps.date.isValid() ? String(gps.date.value()) : "INVALID";
    line += ",";
    line += gps.time.isValid() ? String(gps.time.value()) : "INVALID";
    line += ",";

    line += gps.location.isValid() ? String(gps.location.lat(), 6) : "INVALID";
    line += ",";
    line += gps.location.isValid() ? String(gps.location.lng(), 6) : "INVALID";
    line += ",";

    line += gps.altitude.isValid() ? String(gps.altitude.meters(), 2) : "INVALID";
    line += ",";

    line += gps.speed.isValid() ? String(gps.speed.kmph(), 2) : "INVALID";
    line += ",";

    line += gps.course.isValid() ? String(gps.course.deg(), 2) : "INVALID";
    line += ",";

    line += gps.satellites.isValid() ? String(gps.satellites.value()) : "INVALID";
    line += ",";

    line += gps.hdop.isValid() ? String(gps.hdop.hdop(), 2) : "INVALID";
    line += ",";

    line += gps.location.isValid() ? String(gps.location.age()) : "INVALID";

    return line;
}

long lastLog = 0;
void writeLog()
{
    if (millis() - lastLog < logInterval)
    {
        return;
    }

    lastLog = millis();

    if (!sdCardWorking)
    {
#if SHOULD_LOG_STORAGE
        Serial.println("SD card not working.");
#endif
        return;
    }

    long start = millis();

    f = SDFS.open(filename, "a+");

    String message = buildLogMessage();

#if SHOULD_LOG_STORAGE
    Serial.println(message);
#endif

    if (f)
    {
        f.println(message);
    }
    else
    {
#if SHOULD_LOG_STORAGE
        Serial.println("File open failed.");
#endif
    }

    f.close();

#if SHOULD_LOG_STORAGE
    Serial.print("Writing log took ");
    Serial.print(millis() - start);
    Serial.println(" ms.");
    Serial.flush();
#endif
}
