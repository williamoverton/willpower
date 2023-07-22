#ifndef CROSSCORE_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define CROSSCORE_H

extern void readFromOtherCore();
void writeToOtherCore(String message);

extern void (*coreZeroCallbackFunction)(char buffer[]);
extern void (*coreOneCallbackFunction)(char buffer[]);

#endif