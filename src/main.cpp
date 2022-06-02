#include "encoder.hpp"
#include "mux.hpp"
#include "seven-segment.hpp"

#include <Arduino.h>
// consts
int beatsDivMin = 2;
int beatsDivMax = 32;

// Shift register pins (seven segment)
int dataPinA = 11;  // PB3
int latchPinA = 9;  // PB1
int clockPinA = 10; // PB2

// Mux controller pins
int m0 = 4;            // PD4
int m1 = 3;            // PD3
int m2 = 2;            // PD2
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

encoder_t div_encoder = {HIGH, HIGH, 0};
encoder_t beat_encoder = {HIGH, HIGH, 0};

shift_register_t shift_register = {dataPinA, latchPinA, clockPinA};

int analogMuxOuts[8];
int digitalMuxOuts[8];

mux_t analog_mux = {{m0, m1, m2}, analogMuxIn, true, analogMuxOuts, 8};
mux_t digital_mux = {{m0, m1, m2}, digitalMuxIn, false, digitalMuxOuts, 8};

// Storage for the pots
int pot0 = 0;
int pot1 = 0;

// Storage for the latch switches
int beat_latch = 0;
int div_latch = 0;
int beat_switch_state_prev = 0;
int div_switch_state_prev = 0;

// State
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

// Which digit is currently being written
int currentDigit = 0;

void setup() {
  Serial.begin(9600);

  Serial.println("setup");

  // Analog read pin for the digital mux
  pinMode(digitalMuxIn, INPUT);

  // Setting all of the shift register pins to be outputs
  pinMode(clockPinA, OUTPUT);
  pinMode(latchPinA, OUTPUT);
  pinMode(dataPinA, OUTPUT);

  // Mux selector pins are outputs
  pinMode(m0, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
}

void processEncoders() {
  int inc =
      encoder_process(&beat_encoder, digital_mux.outputs[DigitalMux.BEAT_ENC_A],
                      digital_mux.outputs[DigitalMux.BEAT_ENC_B]);
  if (inc != 0) {
    state.beats += inc;
    if (state.beats < beatsDivMin)
      state.beats = beatsDivMax;
    if (state.beats > beatsDivMax)
      state.beats = beatsDivMin;
  }
  inc = encoder_process(&div_encoder,
                        digital_mux.outputs[DigitalMux.DIVIDE_ENC_A],
                        digital_mux.outputs[DigitalMux.DIVIDE_ENC_B]);
  if (inc != 0) {
    state.div += inc;
    if (state.div < beatsDivMin)
      state.div = beatsDivMax;
    if (state.div > beatsDivMax)
      state.div = beatsDivMin;
  }
}

/**
 * The three step process consists of reading GPIO values,
 * processing the data, and writing the output values.
 **/
void loop() {
  processEncoders();
  mux_process(&digital_mux);
  mux_process(&analog_mux);

  if (digitCounter == 4) {
    digitCounter = 0;
  }

  int value;
  switch (digitCounter) {
  case 0:
    value = state.div / 10;
    break;
  case 1:
    value = state.div % 10;
    break;
  case 2:
    value = state.beats / 10;
    break;
  case 3:
    value = state.beats % 10;
    break;
  }
  seven_segment_process(&shift_register, digitCounter, value);
  digitCounter++;
}
