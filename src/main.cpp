/**
 * main.cpp —— (Max Ardito, 07/09/20)
 *
 * Main functional loop for the Mess'd Up module. It's the
 * place where the hardware [GPIO_t] and the code API [/lib/] go on
 * expensive and indulgent, yet rewarding dinner dates, noshing on
 * the signals served by [buffer_t CV_in/CV_out].
 */

// #include <interfaces.hpp>
#include <../modules/messd-up/messd-up.cpp>
/* #include "interfaces.hpp" */

ModuleInterface *module = buildModule();
unsigned long previousTime;

/**
 * Initializes the ATMEGA328's pins, initializes
 * the other structs' variables, starts off the Serial
 * monitor for possible debugging
 **/
void setup() {
  GPIO_init(module->getInputPinSchematic(), module->getNumInputs());
  GPIO_init(module->getOutputPinSchematic(), module->getNumOutputs());

  Serial.begin(9600);
  previousTime = micros();
  module->init();
}

/**
 * The three step process consists of reading GPIO values,
 * processing the data, and writing the output values.
 **/
void loop() {
  int startMillis = micros();
  int delta = startMillis - previousTime;
  previousTime = startMillis;

  GPIO_read(module->getInputPinSchematic(), module->getInputBuffer(),
            module->getNumInputs());

  module->process(delta);

  GPIO_write(module->getOutputPinSchematic(), module->getOutputBuffer(),
             module->getNumOutputs());
}
