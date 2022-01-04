/**
 * gpio.h —— (Max Ardito, 07/10/20)
 *
 * Wrapper classs used in order to create a layer of abstraction
 * in between the hardware and the module's API. Consists of the general
 * structure for the pin layout used on the ATMEGA328, as well as
 * read/write wrapper functions based on the <Arduino.h> lib.
 */
#pragma once

#ifndef GPIO_H
#define GPIO_H

#define NUM_MUX_INS 8
#define NUM_SHIFT_REGISTER_OUTS 16

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DIGITAL 0x0
#define ANALOG 0x1

// Struct representing a single pin
typedef struct pin {
  uint8_t pin;
  uint8_t mode;
  bool isAnalog;
} pin_t;

typedef enum muxUtils { A_PIN, B_PIN, C_PIN, READ_PIN } muxUtils_t;
typedef enum shiftRegisterUtils {
  CLOCK_PIN,
  DATA_PIN,
  LATCH_PIN,
} shiftRegisterUtils_t;

/**
 * Returns the global pin IO struct
 */
static void GPIO_init(pin_t *self, uint8_t numArgs) {
  for (int i = 0; i < numArgs; i++) {
    pinMode(self[i].pin, self[i].mode);
  }
}

/**
 * Reads incoming data from all inputs
 */
static void GPIO_read(pin_t *self, double *inputValues, uint8_t numArgs) {
  for (int i = 0; i < NUM_MUX_INS; i++) {
    digitalWrite(self[A_PIN].pin, i & 1);
    digitalWrite(self[B_PIN].pin, (i >> 1) & 1);
    digitalWrite(self[C_PIN].pin, (i >> 2) & 1);

    inputValues[i] = analogRead(self[READ_PIN].pin);
  }
}
/**
 * Writes data to all outputs
 */
static void GPIO_write(pin_t *self, double *outputValues, uint8_t numArgs) {
  digitalWrite(self[LATCH_PIN].pin, 0);

  for (int i = 0; i < NUM_SHIFT_REGISTER_OUTS; i++) {
    digitalWrite(self[DATA_PIN].pin, 0);
    digitalWrite(self[CLOCK_PIN].pin, 0);

    digitalWrite(self[DATA_PIN].pin, ((outputValues[i] > 0.5 ? 0 : 1) & 1));

    digitalWrite(self[CLOCK_PIN].pin, 1);
    digitalWrite(self[DATA_PIN].pin, 0);
  }

  digitalWrite(self[LATCH_PIN].pin, 1);
}

#ifdef __cplusplus
}
#endif

#endif // GPIO_H
