#include <parameter.hpp>

using namespace std;

template <typename T> class ModuleInterface {
public:
  virtual void init();
  virtual void process();

  virtual pin_t *getInputPinSchematic();
  virtual pin_t *getOutputPinSchematic();

  T *getBuffer(uint8_t IO_type) {
    T *buffer;

    if (IO_type == INPUT) {
      buffer = this->IO_buffer.inputBuffer;
    } else if (IO_type == OUTPUT) {
      buffer = this->IO_buffer.outputBuffer;
    }

    return buffer;
  };

  uint8_t getNumInputs() { return this->numInputs; };

  uint8_t getNumOutputs() { return this->numOutputs; }

protected:
  uint8_t numInputs;
  uint8_t numOutputs;

  // TUPLES?
  Parameter *parameter;
  struct IO_BUFFER {
    T *inputBuffer;
    T *outputBuffer;
  } IO_buffer;

  template <typename Module, typename Ins, typename Outs> struct moduleIO {
    Module module;
    Ins ins;
    Outs outs;
  };
};

ModuleInterface<double> *buildModule();
