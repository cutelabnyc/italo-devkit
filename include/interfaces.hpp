#ifndef __INTERFACES
#define __INTERFACES

#include <Arduino.h>

class ModuleInterface {
public:
  virtual void initHardware();
  virtual void process(float msDelta);
};
#endif
