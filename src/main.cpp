#include <Arduino.h>

#include "core0.h"
#include "core1.h"

void setup()
{
  core_setup0();
}

void loop()
{
  core_loop0();
}

void setup1()
{
  core_setup1();
}

void loop1()
{
  core_loop1();
}