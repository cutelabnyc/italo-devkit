#ifndef __INTERFACES
#define __INTERFACES
using namespace std;

class ModuleInterface {
public:
  virtual void init();
  virtual void process(float msDelta);
};
#endif
