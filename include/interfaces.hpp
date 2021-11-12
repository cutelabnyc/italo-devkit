#include "parameter.hpp"

#define NUM_INPUTS 5
#define NUM_OUTPUTS 4

template <typename T> class ModuleInterface {
public:
  virtual void init();
  virtual void process();

  ModuleInterface(uint8_t numInputs, uint8_t numOutputs) {
    for (int i = 0; i < numInputs; i++) {
      inBuffer[i] = new Parameter<float>(A5, INPUT, true); /* YAML */
    }
    for (int i = 0; i < numOutputs; i++) {
      outBuffer[i] = new Parameter<double>(3, OUTPUT, false); /* YAML */
    }
  };

  ~ModuleInterface() {
    delete inBuffer[NUM_INPUTS];
    delete outBuffer[NUM_OUTPUTS];
  };

  ParamTree *getIObuffer(uint8_t IO_type) {
    ParamTree *buffer;

    if (IO_type == INPUT) {
      buffer = this->inputs;
    } else if (IO_type == OUTPUT) {
      buffer = this->outputs;
    }

    return buffer;
  };

  void readAll() {
    for (int i = 0; i < NUM_INPUTS; i++) {
      inBuffer[i]->read();
    }
  };

  void writeAll() {
    for (int i = 0; i < NUM_OUTPUTS; i++) {
      outBuffer[i]->write();
    }
  };

protected:
  // TODO: Tuple?
  ParamTree *inBuffer[NUM_INPUTS];
  ParamTree *outBuffer[NUM_OUTPUTS];

  template <typename Module, typename Ins, typename Outs> struct moduleIO {
    Module module;
    Ins ins;
    Outs outs;
  };
};

ModuleInterface<double> *buildModule() {
  return new ModuleInterface<double>(NUM_INPUTS, NUM_OUTPUTS);
}
