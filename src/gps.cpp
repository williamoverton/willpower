#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <FastLED.h>
#include "pins.h"
#include "utils.h"

#define SHOULD_LOG_GPS false

// LED STUFF
CRGB leds[1];

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // RX, TX

// Talking to GPS is wacky. These posts helped:
// https://portal.u-blox.com/s/question/0D52p00008HKD3fCAH/byte-command-to-set-baudrate-115200
// https://stackoverflow.com/questions/73242403/how-do-i-configure-the-baud-rate-of-the-neo-6m-gps-on-ttgo-t-beam

void sendPacket(byte *packet, byte len)
{
    for (byte i = 0; i < len; i++)
    {
        gpsSerial.write(packet[i]);
    }
}

void changeFrequency()
{
    byte packet[] = {
        0xB5,
        0x62,
        0x06,
        0x08,
        0x06,
        0x00,
        0x64,
        0x00,
        0x01,
        0x00,
        0x01,
        0x00,
        0x7A,
        0x12,
    };
    sendPacket(packet, sizeof(packet));
}

void changeBaudrate()
{
    // byte packet38400[] = {
    //   0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00,
    //   0x00, 0xD0, 0x08, 0x00, 0x00, 0xF0, 0x87, 0x00, 0x00,
    //   0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x74, 0x24,
    // };

    byte packet115200[] = {
        0xB5,
        0x62,
        0x06,
        0x00,
        0x14,
        0x00,
        0x01,
        0x00,
        0x00,
        0x00,
        0xD0,
        0x08,
        0x00,
        0x00,
        0x00,
        0xC2,
        0x01,
        0x00,
        0x07,
        0x00,
        0x03,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0xC0,
        0x7E,
    };
    sendPacket(packet115200, sizeof(packet115200));
}

void setupGPS()
{
    Serial.println("Setting up GPS");

    Serial.println("Setting up GPS status LED");
    FastLED.addLeds<WS2812, 16, RGB>(leds, 1);
    leds[0] = CRGB::DarkMagenta;

    delay(1000);

    gpsSerial.begin(9600);

    // delay(2000);

    // Serial.println("Changing GPS update frequency");
    // changeFrequency();

    // delay(100);

    // Serial.println("Changing GPS baudrate");
    // changeBaudrate();

    // gpsSerial.flush();

    // delay(100);
    // gpsSerial.end();

    // Serial.println("Connecting to GPS serial at 115200");
    // gpsSerial.begin(115200);
}

void setStatusLED()
{
    leds[0] = CRGB::Green;

    if (gps.satellites.isValid())
    {
        leds[0] = CRGB::Amethyst;

        if (gps.satellites.value() > 3)
        {
            leds[0] = CRGB::Green;

            if (gps.location.age() > 5000)
            {
                leds[0] = CRGB::Blue;
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

        // Serial.write(c);

        // leds[0] = CRGB::Red;

        if (gps.encode(c))
        {
#if SHOULD_LOG_GPS
            printGPSInfo();
#endif
            setStatusLED();
        }
    }

    FastLED.show();
}