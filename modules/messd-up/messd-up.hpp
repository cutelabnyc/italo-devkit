#include "encoder.hpp"
#include "interfaces.hpp"
#include "mux.hpp"
#include "seven-segment.hpp"

#include <Arduino.h>
#include <cutemodules.h>

class Module : public ModuleInterface {
private:
  messd_t messd;
  messd_ins_t ins;
  messd_outs_t outs;

  // Shift register pins (seven segment)
  int dataPinA = 11;  // PB3
  int latchPinA = 9;  // PB1
  int clockPinA = 10; // PB2

  // Mux controller pins
  int m0 = 4; // PD4
  int m1 = 3; // PD3
  int m2 = 2; // PD2

  // consts
  int beatsDivMin = 2;
  int beatsDivMax = 32;

  int analogMuxIn = A0;  // PC0
  int digitalMuxIn = A2; // PC2

  int digitCounter = 0;

  // Storage for the encoders
  int divEncA = 0;
  int divEncB = 0;
  int encStateDiv = 0;
  int beatsEncA = 0;
  int beatsEncB = 0;
  int encStateBeats = 0;

  int analogMuxOuts[8];
  int digitalMuxOuts[8];

  // Storage for the pots
  int pot0 = 0;
  int pot1 = 0;

  // Storage for the latch switches
  int beat_latch = 0;
  int div_latch = 0;
  int beat_switch_state_prev = 0;
  int div_switch_state_prev = 0;

  mux_t analog_mux = {{m0, m1, m2}, analogMuxIn, true, analogMuxOuts, 8};
  mux_t digital_mux = {{m0, m1, m2}, digitalMuxIn, false, digitalMuxOuts, 8};

  void _scaleValues();
  void _processEncoders();

public:
  Module();

  struct state {
    int beats = 4;
    int div = 7;
  } state;

  struct AnalogMux {
    int DIVIDE_ATV = 0;
    int TRUNCATE_ATV = 1;
    int LATCH_SWITCH = 2;
    int BEAT_INPUT = 3;
    int ROUND_SWITCH = 4;
    int DIVIDE_INPUT = 5;
    int MOD_INPUT = 6;
    int TRUNCATE_INPUT = 7;
  } AnalogMux;

  struct DigitalMux {
    int BEAT_ENC_A = 0;
    int BEAT_ENC_B = 1;
    int DIV_SWITCH = 2;
    int BEAT_SWITCH = 3;
    int DIVIDE_ENC_B = 4;
    int CLOCK_IN = 5;
    int DIVIDE_ENC_A = 6;
    int CLOCK_SWITCH = 7;
  } DigitalMux;

  shift_register_t shift_register = {dataPinA, latchPinA, clockPinA};

  encoder_t div_encoder = {HIGH, HIGH, 0};
  encoder_t beat_encoder = {HIGH, HIGH, 0};

  void init();
  void process(float msDelta);
};
