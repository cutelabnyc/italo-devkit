#include "messd-up.hpp"

void Module::_scaleValues()
{
	this->ins.tempo = (this->ins.tempo > 0 ? this->ins.tempo : 1);
	this->ins.beatsPerMeasure = (this->ins.beatsPerMeasure > 0 ? this->ins.beatsPerMeasure : 1);
	this->ins.subdivisionsPerMeasure = (this->ins.subdivisionsPerMeasure > 0 ? this->ins.subdivisionsPerMeasure : 1);
	this->ins.phase = (this->ins.phase <= 1 && ins.phase >= 0 ? this->ins.phase : 0);
	this->ins.pulseWidth = (this->ins.pulseWidth < 1 && this->ins.pulseWidth > 0 ? this->ins.pulseWidth : 0.5);
}

Module::Module(uint8_t numInputs, uint8_t numOutputs)
{
	this->numInputs = numInputs;
	this->numOutputs = numOutputs;

	this->IO_buffer.inputBuffer = (double *)malloc(sizeof(double) * numInputs);
	this->IO_buffer.outputBuffer = (double *)malloc(sizeof(double) * numOutputs);
};

void Module::init()
{
	MS_init(&this->messd);
};

void Module::process()
{
	this->ins.delta = 1000.0 / 1000.0;
	this->ins.tempo = this->IO_buffer.inputBuffer[TEMPO];
	// this->ins.beatsPerMeasure = this->inputBuffer[BEATS];
	// this->ins.subdivisionsPerMeasure = this->inputBuffer[SUBDIVISIONS];
	// this->ins.phase = this->inputBuffer[PHASE];
	this->ins.beatsPerMeasure = 2;
	this->ins.subdivisionsPerMeasure = 3;
	this->ins.phase = 0;

	this->ins.ext_clock = 0;

	this->ins.metricModulation = 0;
	this->ins.latchToDownbeat = 0;
	this->ins.invert = 0;
	this->ins.isRoundTrip = 0;
	this->ins.reset = 0;

	this->ins.wrap = 0;
	this->ins.pulseWidth = 0.5;

	_scaleValues();

	MS_process(&this->messd, &this->ins, &this->outs);

	this->IO_buffer.outputBuffer[BEATS_OUT] = this->outs.beat;
	this->IO_buffer.outputBuffer[SUBDIVISIONS_OUT] = this->outs.subdivision;
	this->IO_buffer.outputBuffer[DOWNBEAT_OUT] = this->outs.downbeat;
	this->IO_buffer.outputBuffer[PHASE_OUT] = this->outs.phase;
};

pin_t *Module::getInputPinSchematic()
{
	return inputPinSchematic;
};

pin_t *Module::getOutputPinSchematic()
{
	return outputPinSchematic;
};

ModuleInterface<double> *buildModule()
{
	return new Module(NUM_INPUTS, NUM_OUTPUTS);
}

