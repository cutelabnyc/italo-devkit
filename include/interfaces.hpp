#include <gpio.hpp>

using namespace std;

template<typename T>
class ModuleInterface {
public:
	virtual void init();
	virtual void process();

	virtual pin_t *getInputPinSchematic();
	virtual pin_t *getOutputPinSchematic();

	double *getBuffer(uint8_t IO_type)
	{
		if (IO_type == INPUT)
		{
			return this->IO_buffer.inputBuffer;
		}
		else if (IO_type == OUTPUT)
		{
			return this->IO_buffer.outputBuffer;
		}
	};

	uint8_t getNumInputs()
	{
		return this->numInputs;
	};

	uint8_t getNumOutputs()
	{
		return this->numOutputs;
	}

protected:
	uint8_t numInputs;
	uint8_t numOutputs;

	// TUPLES?

	struct IO_BUFFER {
		T *inputBuffer;
		T *outputBuffer;
	} IO_buffer;

	template<typename Module, typename Ins, typename Outs>
	struct moduleIO{
		Module module;
		Ins ins;
		Outs outs;
	};
};

ModuleInterface<double> *buildModule();


