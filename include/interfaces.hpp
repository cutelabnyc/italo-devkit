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
template <typename I, typename O> class ModuleInterface {

private:
  virtual void GPIO_read(I *ins, O *outs);
  virtual void GPIO_write(I *ins, O *outs);

  class GPIO;

public:
  virtual void process(float msDelta);
};
#endif
