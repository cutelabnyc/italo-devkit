#include <Arduino.h>
#include <interfaces.hpp>

class ShiftRegister {
private:
  Pin<uint8_t> data;
  Pin<uint8_t> latch;
  Pin<uint8_t> clock;

public:
  ShiftRegister(uint8_t dataPinAddress, uint8_t latchPinAddress,
                uint8_t clockPinAddress) {
    data = {0, dataPinAddress, INPUT};
    latch = {0, latchPinAddress, INPUT};
    clock = {0, clockPinAddress, INPUT};
  };

  void process(uint8_t *bitsToWrite, uint8_t numBitsToWrite, bool reverse) {
    digitalWrite(latch.address, LOW);

    for (int i = 0; i < numBitsToWrite; i++) {
      digitalWrite(clock.address, LOW);
      digitalWrite(data.address, bitsToWrite[reverse ? 7 - i : i]);
      digitalWrite(clock.address, HIGH);
    }

    digitalWrite(latch.address, HIGH);
  }
};
