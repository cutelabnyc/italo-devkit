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

typedef struct EncoderState {
  int lastPin1;
  int lastPin2;
  int direction;
} EncoderState;

EncoderState divEncoderState = {HIGH, HIGH, 0};
EncoderState beatsEncoderState = {HIGH, HIGH, 0};

shift_register_t shift_register = {dataPinA, latchPinA, clockPinA};

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

// Look for lagging edges
int updateEncoderState(EncoderState *oldState, int pin1, int pin2) {
  int retval = 0;
  if (pin1 == LOW && oldState->lastPin1 == HIGH) {
    if (oldState->direction == 0) {
      oldState->direction = -1;
    } else if (oldState->direction == 1) {
      oldState->direction = 0;
      retval = 1;
    }
  } else if (pin2 == LOW && oldState->lastPin2 == HIGH) {
    if (oldState->direction == 0) {
      oldState->direction = 1;
    } else if (oldState->direction == -1) {
      oldState->direction = 0;
      retval = -1;
    }
  }

  oldState->lastPin1 = pin1;
  oldState->lastPin2 = pin2;
  return retval;
}

int makeEncoderState(int pinA, int pinB) { return (pinB << 1) | pinA; }

int feedForwardState(int &oldState, int newState) {
  int out = 0;

  if (oldState != newState) {
    int isFwd = ((newState == 0) && (oldState == 1)) ||
                ((newState == 1) && (oldState == 3)) ||
                ((newState == 3) && (oldState == 2)) ||
                ((newState == 2) && (oldState == 0));
    out = isFwd ? 1 : -1;
  }

  oldState = newState;
  return out;
}

void readMultiplexers() {
  int analogVal = 0;
  int digitalVal = 0;

  // Read the mux inputs
  for (int i = 0; i < 8; i++) {
    digitalWrite(m0, bitRead(i, 0));
    digitalWrite(m1, bitRead(i, 1));
    digitalWrite(m2, bitRead(i, 2));

    analogVal = analogRead(analogMuxIn);
    digitalVal = digitalRead(digitalMuxIn);

    // Serial.print(i);
    // Serial.print(": ");
    // Serial.print(analogVal);
    // Serial.print(i == 7 ? "\n" : "\t");

    // continue;

    if (i == AnalogMux.DIVIDE_ATV) {
      pot0 = analogVal;
      // Serial.print("pot 1: ");
      // Serial.print(pot0);
      // Serial.print("\t");
    } else if (i == AnalogMux.TRUNCATE_ATV) {
      pot1 = analogVal;
      // Serial.print("pot 2: ");
      // Serial.print(pot1);
      // Serial.print("\t");
    }

    if (i == AnalogMux.LATCH_SWITCH) {
      // Serial.print("Latch switch: ");
      // Serial.print(analogVal);
      // Serial.print("\t");
    } else if (i == AnalogMux.ROUND_SWITCH) {
      // Serial.print("Round switch: ");
      // Serial.println(analogVal);
      // Serial.print("\n");
    }

    if (i == DigitalMux.DIVIDE_ENC_A) {
      divEncA = digitalVal;
      // Serial.print("divide-a: ");
      // Serial.print(digitalVal);
      // Serial.print("\t");
    } else if (i == DigitalMux.DIVIDE_ENC_B) {
      divEncB = digitalVal;
      // Serial.print("divide-b: ");
      // Serial.print(digitalVal);
      // Serial.print("\t");
    } else if (i == DigitalMux.BEAT_ENC_A) {
      beatsEncA = digitalVal;
      // Serial.print("beats-a: ");
      // Serial.print(digitalVal);
      // Serial.print("\t");
    } else if (i == DigitalMux.BEAT_ENC_B) {
      beatsEncB = digitalVal;
      // Serial.print("beats-b: ");
      // Serial.print(digitalVal);
      // Serial.print("\n");
    } else if (i == DigitalMux.BEAT_SWITCH) {
      if (digitalVal && (beat_switch_state_prev == 0)) {
        beat_latch = !beat_latch;
      }
      beat_switch_state_prev = digitalVal;
    } else if (i == DigitalMux.DIV_SWITCH) {
      if (digitalVal && (div_switch_state_prev == 0)) {
        div_latch = !div_latch;
      }
      div_switch_state_prev = digitalVal;
    }
  }
}

int digitToDisplay(int digitIndex) {
  if (digitIndex < 2) {
    if (digitIndex == 0)
      return (state.div / 10);
    return (state.div % 10);
  } else {
    if (digitIndex == 2)
      return (state.beats / 10);
    return (state.beats % 10);
  }
}

void processEncoders() {
  int newState = makeEncoderState(beatsEncA, beatsEncB);
  int inc = updateEncoderState(&beatsEncoderState, beatsEncA, beatsEncB);
  if (inc != 0) {
    state.beats += inc;
    if (state.beats < beatsDivMin)
      state.beats = beatsDivMax;
    if (state.beats > beatsDivMax)
      state.beats = beatsDivMin;
  }
  inc = updateEncoderState(&divEncoderState, divEncA, divEncB);
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
  readMultiplexers();
  processEncoders();

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
  seven_segment_write(&shift_register, digitCounter, value);
  digitCounter++;
}
