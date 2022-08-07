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
    data = {0, dataPinAddress, OUTPUT};
    latch = {0, latchPinAddress, OUTPUT};
    clock = {0, clockPinAddress, OUTPUT};

    /* data.PinInit(); */
    /* data.PinInit(); */
    /* data.PinInit(); */
    pinMode(data.address, data.type);
    pinMode(latch.address, data.type);
    pinMode(clock.address, data.type);
  };

  void process(uint8_t *bitsToWrite, uint8_t numBitsToWrite, bool reverse) {
    digitalWrite(latch.address, LOW);

    for (int i = 0; i < numBitsToWrite; i++) {
      digitalWrite(clock.address, LOW);
      digitalWrite(data.address, bitsToWrite[reverse ? 7 - i : i]);
      digitalWrite(clock.address, HIGH);
    }

    digitalWrite(latch.address, HIGH);
    /* latch.PinWrite(LOW); */

    /* for (int i = 0; i < numBitsToWrite; i++) { */
    /* clock.PinWrite(LOW); */
    /* data.PinWrite(bitsToWrite[reverse ? 7 - i : i]); */
    /* clock.PinWrite(HIGH); */
    /* } */

    /* latch.PinWrite(HIGH); */
  }
};
