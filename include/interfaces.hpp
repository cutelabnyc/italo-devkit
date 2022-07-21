#ifndef __INTERFACES
#define __INTERFACES

class ModuleInterface {
public:
  virtual void init();
  virtual void process(float msDelta);
};
#endif
