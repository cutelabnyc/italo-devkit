#include <parameter.hpp>

using namespace std;

template <typename T> class ModuleInterface {
public:
  virtual void init();
  virtual void process();

  Parameter<T> *getBuffer(uint8_t IO_type) {
    Parameter<T> *buffer;
    if (IO_type == INPUT) {
      buffer = this->IO_buffer->inputs;
    } else if (IO_type == OUTPUT) {
      buffer = this->IO_buffer->outputs;
    }
    return buffer;
  };

  void readParameters() {
    for (int i = 0; i < this->numInputs; i++) {
      this->IO_buffer.inputs[i].read();
    }
  }

  void writeParameters() {
    for (int i = 0; i < this->numOutputs; i++) {
      this->IO_buffer.outputs[i].write();
    }
  }

  uint8_t getNumInputs() { return this->numInputs; };
  uint8_t getNumOutputs() { return this->numOutputs; }

protected:
  uint8_t numInputs;
  uint8_t numOutputs;

  // TUPLES?
  // Parameter linked list?
  struct IO_BUFFER {
    Parameter<T> *inputs;
    Parameter<T> *outputs;
  } IO_buffer;

  template <typename Module, typename Ins, typename Outs> struct moduleIO {
    Module module;
    Ins ins;
    Outs outs;
  };
};

ModuleInterface<double> *buildModule();
