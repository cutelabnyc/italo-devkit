#include <Arduino.h>
#include <cutemodules.h>
#include <interfaces.hpp>

#define NUM_INPUTS 5
#define NUM_OUTPUTS 4
#define NUM_PIN_ITEMS 3

static pin inputPinSchematic[] = {
    {A6, INPUT, ANALOG}, // Clock In
    {A3, INPUT, ANALOG}, // Downbeat in
    {A4, INPUT, ANALOG}, // Subdivision in
    {A7, INPUT, ANALOG}, // Phase in
    {7, INPUT, DIGITAL}  // Metric Modulation
};

static pin outputPinSchematic[] = {
    {4, OUTPUT, DIGITAL},  // Clock out,
    {12, OUTPUT, DIGITAL}, // Downbeat out
    {10, OUTPUT, DIGITAL}, // Subdivision out,
    {8, OUTPUT, DIGITAL}   // Phase out
};

class Module : public ModuleInterface<double> {
private:
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

  moduleIO<messd_t, messd_ins_t, messd_outs_t> messd;

  void _scaleValues();

public:
  Module(uint8_t numInputs, uint8_t numOutputs, pin *inputPinSchematic,
         pin *outputPinSchematic);

  void init();
  void process();
};
