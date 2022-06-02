#include <Arduino.h>

typedef struct shift_register {
  int data;
  int latch;
  int clock;
} shift_register_t;

static void shift_register_process(shift_register_t *self, uint8_t *bitsToWrite,
                                   uint8_t numBitsToWrite) {
  digitalWrite(self->latch, LOW);

  for (int i = 0; i < numBitsToWrite; i++) {
    digitalWrite(self->clock, LOW);
    digitalWrite(self->data, bitsToWrite[i]);
    digitalWrite(self->clock, HIGH);
  }

  digitalWrite(self->latch, HIGH);
}
