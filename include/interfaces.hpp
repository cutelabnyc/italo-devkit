#ifndef __INTERFACES
#define __INTERFACES

#include <Arduino.h>

typedef char pin_t;

/**
 * Struct representing the entire IO for the module
 */
typedef struct GPIO {
  pin_t IN[4];
  pin_t OUT[4];
  pin_t RESEED;
  pin_t RESET;
  pin_t DENSITY;
  pin_t PULSE_OUT;
  pin_t LEDS[2];
  pin_t MISMATCH;
  pin_t MISSEDOPPORTUNITIES[3];

  char densityReadSequence;

} GPIO_t;

class ModuleInterface {
private:
  virtual GPIO_t GPIO_init();
  virtual void GPIO_read(GPIO_t *self, uint16_t *in, char *reseed, char *reset,
                         uint16_t *density, char *mismatch);
  virtual void GPIO_write(GPIO_t *self, bool *out, uint16_t *pulse_out,
                          bool *missed_opportunities);

public:
  virtual void initHardware();
  virtual void process(float msDelta);
};
#endif
