#include <interfaces.hpp>
#include <stdint.h>

typedef struct picoled_ins {
	uint16_t test;
} picoled_ins_t;

typedef struct picoled_outs {
	uint16_t test;
} picoled_outs_t;

class Module : public ModuleInterface<picoled_ins_t, picoled_outs_t> {
private:

  void HardwareRead(picoled_ins_t *ins, picoled_outs_t *outs);
  void HardwareWrite(picoled_ins_t *ins, picoled_outs_t *outs);

public:
  Module();

  void initHardware();
  void process(float msDelta);
};
