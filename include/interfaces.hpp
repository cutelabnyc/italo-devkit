#include <gpio.hpp>

using namespace std;

class ModuleInterface {
public:
	virtual void init();
	virtual void process();

	virtual pin_t *getInputPinSchematic();
	virtual pin_t *getOutputPinSchematic();

	virtual double *getInputBuffer();
	virtual double *getOutputBuffer();

	virtual uint8_t getNumInputs();
	virtual uint8_t getNumOutputs();
};

ModuleInterface *buildModule();


