#include <Arduino.h>

typedef struct encoder {
  int lastPin1;
  int lastPin2;
  int direction;
} encoder_t;

// Look for lagging edges
static int encoder_process(encoder_t *oldState, int pin1, int pin2) {
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
