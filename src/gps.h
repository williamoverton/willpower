#ifndef GPS_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define GPS_H
#include <TinyGPSPlus.h>

void updateGPS();
void setupGPS();

extern TinyGPSPlus gps;

#endif