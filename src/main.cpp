#include <Arduino.h>
#include <interfaces.hpp>

#ifdef MESSD_UP
#include "messd-up.hpp"
#elif MISSED_OPPORTUNITIES
#include "missed-opportunities.hpp"
#elif PASS_THRU
#include "pass-thru.hpp"
#endif

Module module;

unsigned long currenttime;
float lastdelta = 0;

void setup() {
  module.initHardware();
  currenttime = micros();
}

/**
 * The three step process consists of reading GPIO values,
 * processing the data, and writing the output values.
 **/
void loop() {
  unsigned long nextcurrenttime = micros();
  float delta;
  if (nextcurrenttime < currenttime) {
    delta = (4294967295 - currenttime) + nextcurrenttime;
  } else {
    delta = (float)(nextcurrenttime - currenttime);
  }
  module.process(delta / 1000.0);
  lastdelta = delta;
  currenttime = nextcurrenttime;
}
