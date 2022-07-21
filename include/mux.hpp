#include <Arduino.h>

typedef struct mux {
  uint8_t selectorPins[3];
  uint8_t inputPin;
  bool isAnalog;

  uint16_t *outputs;
  uint8_t muxSize;
} mux_t;

static void mux_process(mux_t *self, int offset = 0) {
    for (int i = 0; i < self->muxSize; i++) {
        int oi = (i + offset) % self->muxSize;
        for (int j = 0; j < 3; j++) {
            digitalWrite(self->selectorPins[j], bitRead(oi, j));
        }

        self->outputs[oi] = self->isAnalog ? analogRead(self->inputPin)
                                           : digitalRead(self->inputPin);
    }
}
