#include <Arduino.h>
#include <interfaces.hpp>

#ifdef MESSD_UP
#include "messd-up.hpp"
#elif MISSED_OPPORTUNITIES
#include "missed-opportunities.hpp"
#endif

Module module;

void setup() {
  Serial.begin(9600);
  module.init();
}

/**
 * The three step process consists of reading GPIO values,
 * processing the data, and writing the output values.
 **/
void loop() { module.process(0); }
