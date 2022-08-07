#include <Arduino.h>

class Encoder {
private:
  uint16_t lastPin1;
  uint16_t lastPin2;
  int8_t direction;

public:
  Encoder(uint16_t lastPin1, uint16_t lastPin2, uint8_t direction) {
    this->lastPin1 = lastPin1;
    this->lastPin2 = lastPin2;
    this->direction = direction;
  }

  uint16_t process(int pin1, int pin2) {
    int retval = 0;
    if (pin1 == LOW && this->lastPin1 == HIGH) {
      if (this->direction == 0) {
        this->direction = -1;
      } else if (this->direction == 1) {
        this->direction = 0;
        retval = 1;
      }
    } else if (pin2 == LOW && this->lastPin2 == HIGH) {
      if (this->direction == 0) {
        this->direction = 1;
      } else if (this->direction == -1) {
        this->direction = 0;
        retval = -1;
      }
    } else if (pin1 == HIGH && pin2 == HIGH) {
      this->direction = 0;
    }

    this->lastPin1 = pin1;
    this->lastPin2 = pin2;
    return retval;
  }
};
