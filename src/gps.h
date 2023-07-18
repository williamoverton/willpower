#ifndef GPS_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define GPS_H

void updateGPS();
void setupGPS();

extern float latitude;
extern float longitude;
extern float altitude;

extern float speedMetresPerSecond;
extern float courseDegrees;

extern int satellites;

#endif