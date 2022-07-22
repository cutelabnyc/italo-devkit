#include "interfaces.hpp"

#include "limits.h"
#include <cutemodules.h>

class Module : public ModuleInterface<opportunity_ins_t, opportunity_outs_t> {
private:
  opportunity_t opportunities;
  opportunity_ins_t opportunities_ins;
  opportunity_outs_t opportunities_outs;

  unsigned int _makeRandomSeed();

  GPIO_t GPIO;

  GPIO_t GPIO_init();
  void GPIO_read(GPIO_t *self, opportunity_ins_t *ins,
                 opportunity_outs_t *outs);
  void GPIO_write(GPIO_t *self, opportunity_ins_t *ins,
                  opportunity_outs_t *outs);

public:
  Module();

  void process(float msDelta);
};
