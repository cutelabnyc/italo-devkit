#include <Arduino.h>
#include <cutemodules.h>
#include <interfaces.hpp>

#define NUM_INPUTS 5
#define NUM_OUTPUTS 4
#define NUM_PIN_ITEMS 3

typedef enum inputNames {
  TEMPO,
  BEATS,
  SUBDIVISION,
  PHASE,
  METRIC_MODULATION
} inputNames_t;

typedef enum outputNames {
  BEATS_OUT,
  DOWNBEAT_OUT,
  SUBDIVISIONS_OUT,
  PHASE_OUT
} outputNames_t;

static PIN<double> inputPinSchematic[] = {
    {TEMPO, A6, INPUT, 0, ANALOG},            // Clock In
    {BEATS, A3, INPUT, 0, ANALOG},            // Downbeat in
    {SUBDIVISION, A4, INPUT, 0, ANALOG},      // Subdivision in
    {PHASE, A7, INPUT, 0, ANALOG},            // Phase in
    {METRIC_MODULATION, 7, INPUT, 0, DIGITAL} // Metric Modulation
};

static PIN<double> outputPinSchematic[] = {
    {BEATS_OUT, 4, OUTPUT, 0, DIGITAL},         // Clock out,
    {DOWNBEAT_OUT, 12, OUTPUT, 0, DIGITAL},     // Downbeat out
    {SUBDIVISIONS_OUT, 10, OUTPUT, 0, DIGITAL}, // Subdivision out,
    {PHASE_OUT, 8, OUTPUT, 0, DIGITAL}          // Phase out
};

class Module : public ModuleInterface<double> {
private:
  moduleIO<messd_t, messd_ins_t, messd_outs_t> messd;

  void _scaleValues();

public:
  Module(uint8_t numInputs, uint8_t numOutputs, PIN<double> *inputPinSchematic,
         PIN<double> *outputPinSchematic);

  void init();
  void process();
};
