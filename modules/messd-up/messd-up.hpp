#include <Arduino.h>
#include <cutemodules.h>
#include <interfaces.hpp>

#define NUM_INPUTS 5
#define NUM_OUTPUTS 4

static uint8_t inputPinSchematic[4][5] = {
    {TEMPO, A6, INPUT, ANALOG, double},          // Clock In
    {BEATS, A3, INPUT, ANALOG, uint8_t},         // Downbeat in
    {SUBDIVISIONS, A4, INPUT, ANALOG, uint8_t},  // Subdivision in
    {PHASE, A7, INPUT, ANALOG, double},          // Phase in
    {METRIC_MODULATION, 7, INPUT, DIGITAL, bool} // Metric Modulation
};

static pin_t outputPinSchematic[] = {
    {BEATS_OUT, 4, OUTPUT, DIGITAL, bool},         // Clock out,
    {DOWNBEAT_OUT, 12, OUTPUT, DIGITAL, bool},     // Downbeat out
    {SUBDIVISIONS_OUT, 10, OUTPUT, DIGITAL, bool}, // Subdivision out,
    {PHASE_OUT, 8, OUTPUT, DIGITAL, bool}          // Phase out
};

// Add enums for parameters

// const & inputSchematic argument
//  See:
//  https://stackoverflow.com/questions/31211319/no-matching-function-for-call-to-class-constructor

class Module : public ModuleInterface {
private:
  moduleIO<messd_t, messd_ins_t, messd_outs_t> messd;

  void _scaleValues();

public:
  Module(uint8_t numInputs, uint8_t numOutputs);

  void init();
  void process();

  pin_t *getInputPinSchematic();
  pin_t *getOutputPinSchematic();
};
