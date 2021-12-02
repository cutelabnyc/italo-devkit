/**
 * gpio.h —— (Max Ardito, 07/10/20)
 *
 * Wrapper classs used in order to create a layer of abstraction
 * in between the hardware and the module's API. Consists of the general
 * structure for the pin layout used on the ATMEGA328, as well as
 * read/write wrapper functions based on the <Arduino.h> lib.
 */
#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>

#define DIGITAL 0x0
#define ANALOG 0x1

template <typename T> class Parameter {
public:
  Parameter<T>(uint8_t pin, uint8_t mode, bool isAnalog) {
    pinMode(pin, mode);
    this->val = 0;
    this->isAnalog = isAnalog;
  }

  ~Parameter(){};

  void read() {
    if (this->isAnalog)
      val = (T)analogRead(this->pin);
    else
      val = (T)digitalRead(this->pin);
  };

  void write() {
    if (this->isAnalog)
      analogWrite(this->pin, this->val);
    else
      digitalWrite(this->pin, this->val);
  };

  T getValue() { return this->val; };
  void setValue(T val) { this->val = val; };

private:
  uint8_t pin;
  uint8_t mode;
  T val;
  bool isAnalog;
};
