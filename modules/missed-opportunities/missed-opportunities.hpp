#include "interfaces.hpp"

#include "limits.h"
#include <cutemodules.h>

class Module : public ModuleInterface<opportunity_ins_t, opportunity_outs_t> {
private:
  opportunity_t opportunities;
  opportunity_ins_t opportunities_ins;
  opportunity_outs_t opportunities_outs;

  unsigned int _makeRandomSeed();

  class MissedOpportunitiesPins : public GPIO<MissedOpportunitiesPins> {
  public:
    pin_t IN[4] = {A5, A0, A1, A2}; // CV Ins -- AD5, AD0, AD1, AD2
    pin_t OUT[4] = {4, 12, 10, 8};  // CV Ins -- PD4, PB4, PB2, PB0
    pin_t RESEED = 2;               // Reseed In -- PD2
    pin_t RESET = A3;               // Reset In -- AD3
    pin_t DENSITY = A4;             // Density In -- AD4
    pin_t PULSE_OUT = 3;            // Pulse Out -- PD3
    pin_t LEDS[2] = {6, 5}; // Reset and Density LEDs (respectively) -- PD6, PD5
    pin_t MISMATCH = 7;     // Miss / Match -- PD7
    pin_t MISSEDOPPORTUNITIES[3] = {
        13, 11, 9};                // "Missed" Opportunities -- PB5, PB3, PB1
    pin_t DENSITYREADSEQUENCE = 0; // Only read density every other clock
  };

  void GPIO_read(opportunity_ins_t *ins, opportunity_outs_t *outs);
  void GPIO_write(opportunity_ins_t *ins, opportunity_outs_t *outs);

public:
  Module();
  MissedOpportunitiesPins pins;
  void process(float msDelta);
};
