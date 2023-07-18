#include "common.h"
#include "utils.h"
#include "pins.h"
#include "state.h"
#include <PPMReader.h>

// -----------------------
// RC Controller inputs
float commandedPitch = 0.0; // -1.0 to 1.0
float commandedRoll = 0.0;  // -1.0 to 1.0
float commandedYaw = 0.0;   // -1.0 to 1.0
float commandedThrottle = 0.0;  // 0.0 to 1.0
float commanedAux1 = 0.0; // 0.0 to 1.0
float commanedAux2 = 0.0; // 0.0 to 1.0
// -----------------------

// Initialize a PPMReader on digital pin 3 with 6 expected channels.
byte channelAmount = 6;
PPMReader ppm(15, channelAmount);

void setupPPM() {
    ppm.begin();
}

void printRawValues() {
    // Print latest valid values from all channels
    for (byte channel = 1; channel <= channelAmount; ++channel) {
        unsigned value = ppm.rawChannelValue(channel);
        Serial.print(value);
        if(channel < channelAmount) Serial.print('\t');
    }
    Serial.println();
}

void failSafeControllerInput() {
    /**
     * OH LORD, I PRAY TO THEE
     * PLEASE DO NOT LET ME CRASH
    */
    commandedPitch = 0.0;
    commandedRoll = 0.0;
    commandedYaw = 0.0;
    commandedThrottle = 0.0;
    commanedAux1 = 0.0;
    commanedAux2 = 0.0;
#if !GROUND_MODE
    Serial.println("Controller disconnected");
#endif
}

void readRawControllerValues() {
    
    // Check if the controller is connected
    if (ppm.rawChannelValue(3) < 500) {
        failSafeControllerInput();
        return;
    }

    commandedRoll = fmap(ppm.rawChannelValue(1), 1000, 2000, -1.0, 1.0);
    commandedPitch = fmap(ppm.rawChannelValue(2), 1000, 2000, -1.0, 1.0);
    commandedYaw = fmap(ppm.rawChannelValue(4), 1000, 2000, -1.0, 1.0);
    commandedThrottle = fmap(ppm.rawChannelValue(3), 1000, 2000, 0.0, 1.0);

    commanedAux1 = fmap(ppm.rawChannelValue(5), 1000, 2000, 0.0, 1.0);
    commanedAux2 = fmap(ppm.rawChannelValue(6), 1000, 2000, 0.0, 1.0);
}