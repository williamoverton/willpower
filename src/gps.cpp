#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include "pins.h"
#include <FastLED.h>

// LED STUFF
CRGB leds[1];

float latitude;
float longitude;
float altitude;

float speedMetresPerSecond;
float courseDegrees;

int satellites;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // RX, TX

// Talking to GPS is wacky. These posts helped:
// https://portal.u-blox.com/s/question/0D52p00008HKD3fCAH/byte-command-to-set-baudrate-115200
// https://stackoverflow.com/questions/73242403/how-do-i-configure-the-baud-rate-of-the-neo-6m-gps-on-ttgo-t-beam

void sendPacket(byte *packet, byte len) {
    for (byte i = 0; i < len; i++)
    {
        gpsSerial.write(packet[i]);
    }
}

void changeFrequency() {
    byte packet[] = {
      0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12,
    };
    sendPacket(packet, sizeof(packet));
}

void changeBaudrate() {
    // byte packet38400[] = {
    //   0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 
    //   0x00, 0xD0, 0x08, 0x00, 0x00, 0xF0, 0x87, 0x00, 0x00, 
    //   0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x74, 0x24,
    // };

    byte packet115200[] = {
      0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 
      0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 
      0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E,
    };
    sendPacket(packet115200, sizeof(packet115200));
}

void setupGPS()
{
    Serial.println("Setting up GPS");

    Serial.println("Setting up GPS status LED");
    FastLED.addLeds<WS2812, 16, RGB>(leds, 1);
    leds[0] = CRGB::DarkMagenta;

    delay(10000);

    // put your setup code here, to run once:
    Serial.begin(9600);

    gpsSerial.begin(9600);

    delay(2000);
    
    changeFrequency();

    delay(100);

    changeBaudrate();

    gpsSerial.flush();

    delay(100);
    gpsSerial.end();

    gpsSerial.begin(115200);
}

void printGPSInfo()
{

    Serial.print(F("Satellites: "));
    gps.satellites.isValid() ? Serial.print(gps.satellites.value()) : Serial.print(F("INVALID"));

    Serial.print(F(" Location: "));

    if (gps.location.isValid())
    {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
        Serial.print(gps.date.year());
        Serial.print(F("/"));
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.println();
}

void setStatusLED()
{
    leds[0] = CRGB::Green;

    if (gps.satellites.isValid())
    {
        leds[0] = CRGB::Yellow;

        if (gps.satellites.value() > 3)
        {
            leds[0] = CRGB::Green;

            if (gps.location.age() > 5000)
            {
                leds[0] = CRGB::BlanchedAlmond;
            }
        }
    }
    else
    {
        leds[0] = CRGB::Red;
    }
}

void updateGPS()
{
    while (gpsSerial.available() > 0)
    {
        char c = gpsSerial.read();

        Serial.write(c);

        if (gps.encode(c))
        {
            latitude = gps.location.lat();
            longitude = gps.location.lng();
            altitude = gps.altitude.meters();

            speedMetresPerSecond = gps.speed.mps();
            courseDegrees = gps.course.deg();

            satellites = gps.satellites.value();

            printGPSInfo();
            setStatusLED();
        }
    }

    FastLED.show();
}