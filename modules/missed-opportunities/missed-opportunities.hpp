#include "gpio.hpp"
#include "interfaces.hpp"

#include "limits.h"
#include <cutemodules.h>

class Module : public ModuleInterface {
private:
  opportunity_t opportunities;

  channel_t channels[4];

  uint16_t GATE_in[4];
  bool GATE_out[4];
  bool MISSED_opportunities[3];

  char RESEED_in;
  char RESET_in;
  uint16_t DENSITY_in;
  char MISMATCH_in;
  uint16_t AUTOPULSE_out;
  char last_reseed = 0;
  uint16_t random_counter = 0;
  uint16_t original_seed;

  uint16_t lastMsec = 0;

  float time_dilation = SPEEDUP;

  void _scaleValues();
  unsigned int _makeRandomSeed();

  GPIO_t GPIO;

public:
  Module();

  void initHardware();
  void process(float msDelta);
};
