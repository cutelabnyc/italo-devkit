#include <Arduino.h>
#include <interfaces.hpp>

#define MUXSIZE 8

class Mux {
private:
  Pin<uint8_t> selectors[3];
  Pin<uint8_t> input;
  bool isAnalog;

  uint16_t outputs[MUXSIZE];
  uint8_t size;

public:
  Mux(uint8_t selectors[3], uint8_t input, bool isAnalog) {
    for (int i = 0; i < 3; i++) {
      this->selectors[i] = {0, selectors[i], OUTPUT};
      pinMode(this->selectors[i].address, this->selectors[i].type);
    }
    this->input = {0, input, INPUT};
    pinMode(this->input.address, this->input.type);

    this->isAnalog = isAnalog;
    this->size = 8;
  }

  void process(int offset = 0) {
    for (int i = 0; i < this->size; i++) {
      int oi = (i + offset) % this->size;
      for (int j = 0; j < 3; j++) {
        digitalWrite(this->selectors[j].address, bitRead(oi, j));
      }

      // Very small delay, apparently absolutely necessary in order to get
      // an accurate reading
      delayMicroseconds(1);

      this->outputs[oi] = this->isAnalog ? analogRead(this->input.address)
                                         : digitalRead(this->input.address);
    }
  }

  uint16_t getOutput(uint8_t index) {
    // TODO: Check if array is out of bounds
    return this->outputs[index];
  }
};
