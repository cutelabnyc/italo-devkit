#include <Arduino.h>
#include <interfaces.hpp>

#ifdef MESSD_UP
#include "messd-up.hpp"
#elif MISSED_OPPORTUNITIES
#include "missed-opportunities.hpp"
#endif

Module module;

unsigned long time;
float lastdelta = 0;

void setup() {
  module.initHardware();
  time = micros();
}

/**
 * The three step process consists of reading GPIO values,
 * processing the data, and writing the output values.
 **/
void loop() {
  unsigned long nexttime = micros();
  float delta;
  if (nexttime < time) {
    delta = (4294967295 - time) + nexttime;
  } else {
    delta = (float)(nexttime - time);
  }
  module.process(delta / 1000.0);
  lastdelta = delta;
  time = nexttime;
}
