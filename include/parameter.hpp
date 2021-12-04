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

template <typename T> struct PIN {
  uint8_t name;    // Enum name
  uint8_t address; // GPIO Address (A6, 4, etc.)
  uint8_t mode;    // INPUT or OUTPUT
  T val;           // Stored value
  bool isAnalog;   // Analog or digital
};

template <typename T> class Parameter {
public:
  Parameter<T>(uint8_t pin, uint8_t mode, bool isAnalog) {
    pinMode(pin, mode);
    this->pin.val = 0;
    this->pin.isAnalog = isAnalog;
  }

  ~Parameter(){};

  void read() {
    if (this->pin.isAnalog)
      this->pin.val = (T)analogRead(this->pin.address);
    else
      this->pin.val = (T)digitalRead(this->pin.address);
  };

  void write() {
    if (this->pin.isAnalog)
      analogWrite(this->pin.address, this->pin.val);
    else
      digitalWrite(this->pin.address, this->pin.val);
  };

  T getValue() { return this->pin.val; };
  void setValue(T val) { this->pin.val = val; };

private:
  struct PIN<T> pin;
};
