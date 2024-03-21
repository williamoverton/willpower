#ifndef STATE_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define STATE_H

enum FlightState
{
    INIT,
    CALIBRATE,
    PASSIVE,
    ACTIVE
};

enum FlightMode
{
    ANGLE,
    RATES
};

#endif

extern FlightState currentState;
extern FlightMode currentMode;