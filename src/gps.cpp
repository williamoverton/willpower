#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include "pins.h"
#include "utils.h"

#define SHOULD_LOG_GPS false

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // RX, TX

void setupGPS()
{
    Serial.println("Setting up GPS");

    gpsSerial.begin(38400);
}

void updateGPS()
{
    while (gpsSerial.available() > 0)
    {
        char c = gpsSerial.read();

        if (gps.encode(c))
        {
#if SHOULD_LOG_GPS
            printGPSInfo();
#endif
        }
    }
}