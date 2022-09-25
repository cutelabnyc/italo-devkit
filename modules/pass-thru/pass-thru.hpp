#include <interfaces.hpp>
#include <stdint.h>

typedef struct passthru_ins {
	uint16_t test;
} passthru_ins_t;

typedef struct passthru_outs {
	uint16_t test;
} passthru_outs_t;

class Module : public ModuleInterface<passthru_ins_t, passthru_outs_t> {
private:

  void HardwareRead(passthru_ins_t *ins, passthru_outs_t *outs);
  void HardwareWrite(passthru_ins_t *ins, passthru_outs_t *outs);

public:
  Module();

  void initHardware();
  void process(float msDelta);
};
