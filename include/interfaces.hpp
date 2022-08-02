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

/**
 * Struct representing the entire IO for the module
 */
template <typename I, typename O> class ModuleInterface {
private:
  virtual void HardwareRead(I *ins, O *outs);
  virtual void HardwareWrite(I *ins, O *outs);

public:
  template <class T> class Hardware {
  protected:
    typedef unsigned char pin_t;
  };

  virtual void initHardware();
  virtual void process(float msDelta);
};
#endif
