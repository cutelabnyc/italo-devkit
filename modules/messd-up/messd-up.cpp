#include "messd-up.hpp"

void Module::_scaleValues()
{
	this->messd.ins.tempo = (this->messd.ins.tempo > 0 ? this->messd.ins.tempo : 1);
	this->messd.ins.beatsPerMeasure = (this->messd.ins.beatsPerMeasure > 0 ? this->messd.ins.beatsPerMeasure : 1);
	this->messd.ins.subdivisionsPerMeasure = (this->messd.ins.subdivisionsPerMeasure > 0 ? this->messd.ins.subdivisionsPerMeasure : 1);
	this->messd.ins.phase = (this->messd.ins.phase <= 1 && messd.ins.phase >= 0 ? this->messd.ins.phase : 0);
	this->messd.ins.pulseWidth = (this->messd.ins.pulseWidth < 1 && this->messd.ins.pulseWidth > 0 ? this->messd.ins.pulseWidth : 0.5);
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
	MS_init(&this->messd.module);
};

void Module::process()
{
	this->messd.ins.delta = 1000.0 / 1000.0;
	this->messd.ins.tempo = this->IO_buffer.inputBuffer[TEMPO];
	// this->messd.ins.beatsPerMeasure = this->inputBuffer[BEATS];
	// this->messd.ins.subdivisionsPerMeasure = this->inputBuffer[SUBDIVISIONS];
	// this->messd.ins.phase = this->inputBuffer[PHASE];
	this->messd.ins.beatsPerMeasure = 2;
	this->messd.ins.subdivisionsPerMeasure = 3;
	this->messd.ins.phase = 0;

	this->messd.ins.ext_clock = 0;

	this->messd.ins.metricModulation = 0;
	this->messd.ins.latchToDownbeat = 0;
	this->messd.ins.invert = 0;
	this->messd.ins.isRoundTrip = 0;
	this->messd.ins.reset = 0;

	this->messd.ins.wrap = 0;
	this->messd.ins.pulseWidth = 0.5;

	_scaleValues();

	MS_process(&this->messd.module, &this->messd.ins, &this->messd.outs);

	this->IO_buffer.outputBuffer[BEATS_OUT] = this->messd.outs.beat;
	this->IO_buffer.outputBuffer[SUBDIVISIONS_OUT] = this->messd.outs.subdivision;
	this->IO_buffer.outputBuffer[DOWNBEAT_OUT] = this->messd.outs.downbeat;
	this->IO_buffer.outputBuffer[PHASE_OUT] = this->messd.outs.phase;
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

