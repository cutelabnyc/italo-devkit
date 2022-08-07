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

template <typename T> struct Pin {
  T val;
  uint8_t address;
  uint8_t type; // should be ternary - INPUT OUTPUT PULLUP

  void PinInit() { pinMode(this->address, this->type); };
  void PinRead() { this->val = digitalRead(this->address); };
  void PinWrite(T val) { digitalWrite(this->address, this->val); };
};

/**
 * Struct representing the entire IO for the module
 */
template <typename I, typename O> class ModuleInterface {
private:
  virtual void HardwareRead(I *ins, O *outs);
  virtual void HardwareWrite(I *ins, O *outs);

public:
  template <class T> class Hardware {};

  virtual void initHardware();
  virtual void process(float msDelta);
};
#endif
