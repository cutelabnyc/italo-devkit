/**
 * interfaces.hpp —— (Max Ardito, 07/22/22)
 *
 * Represents the generic interface of a single Eurorack module.
 * This includes the lowest level GPIO schematic, agnostic to
 * anything beyond pin addresses, values, and input/output. It
 * also includes a process function, in which the GPIO can be read,
 * written to, and processed by the module functions defined in
 * cutesynth.
 */
#ifndef __INTERFACES
#define __INTERFACES

#include <Arduino.h>
#include <cutemodules.h>

typedef char pin_t;

/* template <typename T> struct pin_t { */
/*   char address; */
/*   T value; */
/*   bool type; */
/* }; */

/**
 * Struct representing the entire IO for the module
 */
typedef struct GPIO {
  pin_t IN[4];
  pin_t OUT[4];
  pin_t RESEED;
  pin_t RESET;
  pin_t DENSITY;
  pin_t PULSE_OUT;
  pin_t LEDS[2];
  pin_t MISMATCH;
  pin_t MISSEDOPPORTUNITIES[3];

  char densityReadSequence;
} GPIO_t;

template <typename I, typename O> class ModuleInterface {
private:
  virtual GPIO_t GPIO_init();
  virtual void GPIO_read(GPIO_t *self, I *ins, O *outs);
  virtual void GPIO_write(GPIO_t *self, I *ins, O *outs);

public:
  virtual void process(float msDelta);
};
#endif
