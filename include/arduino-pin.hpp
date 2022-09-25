#include <Arduino.h>

template <typename T> struct Pin {
  T val;
  uint8_t address;
  uint8_t type; // should be ternary - INPUT OUTPUT PULLUP

  void PinInit() { pinMode(this->address, this->type); };
  void PinRead() { this->val = digitalRead(this->address); };
  void PinWrite(T val) { digitalWrite(this->address, this->val); };
};
