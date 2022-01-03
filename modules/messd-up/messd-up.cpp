#include "interfaces.hpp"
#include <Arduino.h>
#include <cutemodules.h>

#define NUM_INPUTS 5
#define NUM_OUTPUTS 4

static pin_t inputPinSchematic[] = {
    {A6, INPUT, ANALOG}, // Clock In
    {A3, INPUT, ANALOG}, // Downbeat in
    {A4, INPUT, ANALOG}, // Subdivision in
    {A7, INPUT, ANALOG}, // Phase in
    {7, INPUT, DIGITAL}  // Metric Modulation
};

static pin_t outputPinSchematic[] = {
    {4, OUTPUT, DIGITAL},  // Clock out,
    {12, OUTPUT, DIGITAL}, // Downbeat out
    {10, OUTPUT, DIGITAL}, // Subdivision out,
    {8, OUTPUT, DIGITAL}   // Phase out
};

typedef enum inputNames {
  TEMPO,
  BEATS,
  SUBDIVISIONS,
  PHASE,
  METRIC_MODULATION
} inputNames_t;

typedef enum outputNames {
  BEATS_OUT,
  DOWNBEAT_OUT,
  SUBDIVISIONS_OUT,
  PHASE_OUT,
} outputNames_t;

class Module : public ModuleInterface {
private:
  uint8_t numInputs;
  uint8_t numOutputs;

  double *inputBuffer;
  double *outputBuffer;

  messd_t messd;
  messd_ins_t ins;
  messd_outs_t outs;

  void _scaleValues() {
    this->ins.tempo = (this->ins.tempo > 0 ? this->ins.tempo : 1);
    this->ins.beatsPerMeasure =
        (this->ins.beatsPerMeasure > 0 ? this->ins.beatsPerMeasure : 1);
    this->ins.subdivisionsPerMeasure =
        (this->ins.subdivisionsPerMeasure > 0 ? this->ins.subdivisionsPerMeasure
                                              : 1);
    this->ins.phase =
        (this->ins.phase <= 1 && ins.phase >= 0 ? this->ins.phase : 0);
    this->ins.pulseWidth = (this->ins.pulseWidth < 1 && this->ins.pulseWidth > 0
                                ? this->ins.pulseWidth
                                : 0.5);
  }

public:
  Module(uint8_t numInputs, uint8_t numOutputs) {
    this->numInputs = numInputs;
    this->numOutputs = numOutputs;

    this->inputBuffer = (double *)malloc(sizeof(double) * numInputs);
    this->outputBuffer = (double *)malloc(sizeof(double) * numOutputs);
  };

  void init() { MS_init(&this->messd); };

  void process() {
    this->ins.delta = 1000.0 / 1000.0;
    /* this->ins.tempo = this->inputBuffer[TEMPO]; */
    this->ins.tempo = 120;
    // this->ins.beatsPerMeasure = this->inputBuffer[BEATS];
    // this->ins.subdivisionsPerMeasure = this->inputBuffer[SUBDIVISIONS];
    // this->ins.phase = this->inputBuffer[PHASE];
    this->ins.beatsPerMeasure = 2;
    this->ins.subdivisionsPerMeasure = 3;
    this->ins.phase = 0;

    this->ins.ext_clock = 0;

    this->ins.metricModulation = 0;
    this->ins.latchChangesToDownbeat = 0;
    this->ins.invert = 0;
    this->ins.isRoundTrip = 0;
    this->ins.reset = 0;

    this->ins.wrap = 0;
    this->ins.pulseWidth = 0.5;

    _scaleValues();

    MS_process(&this->messd, &this->ins, &this->outs);

    this->outputBuffer[BEATS_OUT] = this->outs.beat;
    this->outputBuffer[SUBDIVISIONS_OUT] = this->outs.subdivision;
    this->outputBuffer[DOWNBEAT_OUT] = this->outs.downbeat;
    this->outputBuffer[PHASE_OUT] = this->outs.phase;
  };

  pin_t *getInputPinSchematic() { return inputPinSchematic; };

  pin_t *getOutputPinSchematic() { return outputPinSchematic; };

  double *getInputBuffer() { return this->inputBuffer; };

  double *getOutputBuffer() { return this->outputBuffer; };

  uint8_t getNumInputs() { return this->numInputs; };

  uint8_t getNumOutputs() { return this->numOutputs; };
};

ModuleInterface *buildModule() { return new Module(NUM_INPUTS, NUM_OUTPUTS); }
