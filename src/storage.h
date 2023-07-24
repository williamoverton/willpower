#ifndef STORAGE_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define STORAGE_H

void setupStorage();
void writeLog();

// Struct containing all the data we want to log
struct LogData
{
    // Time
    long timeMillis;
    String date;
    String time;

    // position
    double lat;
    double lng;
    double altitude;

    // speed & direction
    double speedKmph;
    double courseDegrees;

    // GPS debug info
    int satellites;
    double hdop;
    double locationAge;

    // IMU debug info
    double pitch;
    double roll;
    double yaw;

    double pitchRate;
    double rollRate;
    double yawRate;
};

extern LogData logData;
#endif