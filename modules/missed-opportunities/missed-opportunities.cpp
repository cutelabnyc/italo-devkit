#include "missed-opportunities.hpp"

static uint8_t b_register;
#ifndef ANALOG_READ
static uint8_t c_register;
#endif
static uint8_t d_register;

#define __fastwrite(pin, val)                                                  \
  (pin < 8) ? (val ? PORTD |= digitalPinToBitMask(pin)                         \
                   : PORTD &= ~digitalPinToBitMask(pin))                       \
            : (val ? PORTB |= digitalPinToBitMask(pin)                         \
                   : PORTB &= ~digitalPinToBitMask(pin))

unsigned int Module::_makeRandomSeed() {
  unsigned int out = 0;
  uint16_t in[4];
  unsigned int readBits = 0;
  while (readBits < CHAR_BIT * sizeof(unsigned int)) {
    GPIO_read(&opportunities_ins, &opportunities_outs);
    for (char i = 0; i < 4; i++) {
      out = out << 1;
      out = out | (in[i] & 1);
      readBits++;
    }
  }

  return out;
}

void Module::GPIO_read(opportunity_ins_t *ins, opportunity_outs_t *outs) {
  uint16_t analogVal;
  b_register = PINB;
#ifndef ANALOG_READ
  c_register = PINC;
#endif
  d_register = PIND;

#ifdef ANALOG_READ
  for (char i = 0; i < 4; i++) {
    ins->gates[i] = analogRead(pins.schematic.IN[i]);
  }
#else
  c_register = PINC;
  for (char i = 0; i < 4; i++) {
    ins->gates[i] =
        (c_register & digitalPinToBitMask(pins.schematic.IN[i])) ? 1024 : 0;
  }
#endif

  ins->reseed =
      (d_register & digitalPinToBitMask(pins.schematic.RESEED)) ? HIGH : LOW;

#ifdef ANALOG_READ
  analogVal = analogRead(pins.schematic.RESET);
  ins->reset = analogVal > 700;
#else
  ins->reset = (c_register & digitalPinToBitMask(pins.schematic.RESET));
#endif
  if (pins.schematic.DENSITYREADSEQUENCE++ == 2) {
    ins->density = analogRead(pins.schematic.DENSITY);
    pins.schematic.DENSITYREADSEQUENCE = 0;
  }
  ins->miss_match =
      (d_register & digitalPinToBitMask(pins.schematic.MISMATCH)) ? HIGH : LOW;

  // *reset = 0;
  // *density = 768;
  // *mismatch = 0; // assume this is match mode

  // Reset should appear to be binary, even though it's reading an analog value
  analogWrite(pins.schematic.LEDS[0], ins->reset ? 200 : 0);
  // analogWrite(pins.schematic.LEDS[0], *reset / 4);
  // analogWrite(pins.schematic.LEDS[0], 255);

  // analogWrite(pins.schematic.LEDS[0], *mismatch ? 255 : 0);
  // analogWrite(pins.schematic.LEDS[1], *density / 32);
  float dreg = (ins->density / 4) / 255.0;
  analogWrite(pins.schematic.LEDS[1], (uint8_t)(dreg * dreg * dreg * 255));
  // analogWrite(5, (uint16_t) (dreg * dreg * dreg * 255));
  // analogWrite(5, *mismatch ? 255 : 0 );

  // analogWrite(pins.schematic.LEDS[0], 255);
  // analogWrite(pins.schematic.LEDS[1], 255);
}

void Module::GPIO_write(opportunity_ins_t *ins, opportunity_outs_t *outs) {
  for (char i = 0; i < 4; i++) {
    __fastwrite(pins.schematic.OUT[i], outs->gates[i]);
    // digitalWrite(pins.schematic.OUT[i], 255);

    // Write the Missed Opportunities
    if (i < 4 - 1) {
      __fastwrite(pins.schematic.MISSEDOPPORTUNITIES[i],
                  ins->missed_opportunities[i]);
      // digitalWrite(pins.schematic.MISSED_OPPORTUNITIES[i], 255);
      // digitalWrite(13, HIGH);
      // digitalWrite(pins.schematic.MISSED_OPPORTUNITIES[0], HIGH);
    }
  }

  // static bool po = false;
  // po != po;
  // digitalWrite(pins.schematic.PULSE_OUT, po ? 255 : 0);
  __fastwrite(pins.schematic.PULSE_OUT, outs->autopulse);
  // digitalWrite(pins.schematic.PULSE_OUT, HIGH);
}

Module::Module() {
  opportunities.original_seed = _makeRandomSeed();

  for (char i = 0; i < 4; i++) {
    pinMode(pins.schematic.IN[i], INPUT);
    pinMode(pins.schematic.OUT[i], OUTPUT);
  }

  pinMode(pins.schematic.RESEED, INPUT_PULLUP);
  pinMode(pins.schematic.RESET, INPUT);
  pinMode(pins.schematic.DENSITY, INPUT);
  pinMode(pins.schematic.PULSE_OUT, OUTPUT);

  for (char i = 0; i < 2; i++) {
    pinMode(pins.schematic.LEDS[i], OUTPUT);
  }

  pinMode(pins.schematic.MISMATCH, INPUT);

  for (char i = 0; i < 4 - 1; i++) {
    pinMode(pins.schematic.MISSEDOPPORTUNITIES[i], OUTPUT);
  }

  OP_init(&opportunities, 4, 1023, 700, 3);
};

void Module::process(float msDelta) {

  GPIO_read(&opportunities_ins, &opportunities_outs);

  uint16_t time = millis();
  uint16_t msec = (time - opportunities.lastMsec) * opportunities.time_dilation;
  opportunities.lastMsec = time;
  bool reseed = opportunities_ins.reseed == HIGH;
  if (!opportunities.last_reseed && reseed) {
    opportunities.random_counter++;
    OP_set_seed(&opportunities, opportunities.original_seed +
                                    opportunities.random_counter * 69 + time);
  }
  opportunities.last_reseed = reseed;

  OP_process(&opportunities, &opportunities_ins, &opportunities_outs,
             (char)msec);

  // In "match" mode, copy from outputs to the missed opportunities
  // This is basically the same as the straight normalization from before
  if (opportunities_ins.miss_match == 1) {
    for (char i = 0; i < 3; i++) {
      opportunities_ins.missed_opportunities[i] = opportunities_outs.gates[i];
    }
  }

  GPIO_write(&opportunities_ins, &opportunities_outs);
};
