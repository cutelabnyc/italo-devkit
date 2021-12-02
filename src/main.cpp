/**
 * main.cpp —— (Max Ardito, 07/09/20)
 *
 * Main functional loop for the Mess'd Up module. It's the
 * place where the hardware [GPIO_t] and the code API [/lib/] go on
 * expensive and indulgent, yet rewarding dinner dates, noshing on
 * the signals served by [buffer_t CV_in/CV_out].
 */

#include <interfaces.hpp>

ModuleInterface<double> *module = buildModule();

/**
 * Initializes the ATMEGA328's pins, initializes
 * the other structs' variables, starts off the Serial
 * monitor for possible debugging
 **/
void setup() {
  Serial.begin(9600);

  module->init();
}

/**
 * The three step process consists of reading GPIO values,
 * processing the data, and writing the output values.
 **/
void loop() {
  module->readParameters();
  module->process();
  module->writeParameters();
}
