#include "missed-opportunities.hpp"

void Module::_scaleValues() {}

unsigned int Module::_makeRandomSeed() {
  unsigned int out = 0;
  uint16_t in[4];
  char reseed;
  char reset;
  uint16_t density;
  char mismatch;
  unsigned int readBits = 0;
  while (readBits < CHAR_BIT * sizeof(unsigned int)) {
    GPIO_read(&GPIO, in, &reseed, &reset, &density, &mismatch);
    for (char i = 0; i < 4; i++) {
      out = out << 1;
      out = out | (in[i] & 1);
      readBits++;
    }
  }

  return out;
}

Module::Module() {
  original_seed = _makeRandomSeed();

  OP_init(&opportunities, channels, 4, 1023, 700, 3, original_seed);
};

void Module::initHardware() {
  // Analog read pin for the digital mux
  GPIO = GPIO_init();
}

void Module::process(float msDelta) {

  GPIO_read(&GPIO, GATE_in, &RESEED_in, &RESET_in, &DENSITY_in, &MISMATCH_in);

  uint16_t time = millis();
  uint16_t msec = (time - lastMsec) * time_dilation;
  lastMsec = time;
  bool reseed = RESEED_in == HIGH;
  if (!last_reseed && reseed) {
    random_counter++;
    OP_set_seed(&opportunities, original_seed + random_counter * 69 + time);
  }
  last_reseed = reseed;

  OP_process(&opportunities, GATE_in, GATE_out, RESET_in > 0, &DENSITY_in,
             &AUTOPULSE_out, MISSED_opportunities, (char)msec);

  // In "match" mode, copy from outputs to the missed opportunities
  // This is basically the same as the straight normalization from before
  if (MISMATCH_in == 1) {
    for (char i = 0; i < 3; i++) {
      MISSED_opportunities[i] = GATE_out[i];
    }
  }

  GPIO_write(&GPIO, GATE_out, &AUTOPULSE_out, MISSED_opportunities);
};
