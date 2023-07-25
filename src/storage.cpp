#include <Arduino.h>
#include <SPI.h>
#include <SDFS.h>
#include "pins.h"
#include "gps.h"
#include "storage.h"

#define SHOULD_LOG_STORAGE false

static File f;
static bool sdCardWorking = false;
static long logInterval = 100; // ms
static String filename;

LogData logData;

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

    line += "Pitch,Roll,Yaw";
    line += "PitchRate,RollRate,YawRate";

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

    logData = {
        // Time
        0,
        "",
        "",

        // position
        0,
        0,
        0,

        // speed & direction
        0,
        0,

        // GPS debug info
        -1,
        -1,
        -1,

        // IMU debug info
        -1,
        -1,
        -1,

        -1,
        -1,
        -1};

    setupSDCard();
}

String buildLogMessage()
{
    String line = "";

    line += String(logData.timeMillis) + ",";
    line += logData.date + "," + logData.time + ",";

    line += String(logData.lat, 6) + "," + String(logData.lng, 6) + ",";
    line += String(logData.altitude) + ",";

    line += String(logData.speedKmph) + ",";
    line += String(logData.courseDegrees) + ",";

    line += String(logData.satellites) + ",";
    line += String(logData.hdop) + ",";
    line += String(logData.locationAge) + ",";

    line += String(logData.pitch) + "," + String(logData.roll) + "," + String(logData.yaw) + ",";
    line += String(logData.pitchRate) + "," + String(logData.rollRate) + "," + String(logData.yawRate);

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
