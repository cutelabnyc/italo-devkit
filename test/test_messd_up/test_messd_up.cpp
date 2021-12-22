#include "../test_util.hpp"
#include <cutemodules.h>
#include <unity.h>

#define NUM_INPUTS 5
#define NUM_OUTPUTS 4

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
    {TEMPO, 0, INPUT, 0, ANALOG},             // Clock In
    {BEATS, 0, INPUT, 0, ANALOG},             // Downbeat in
    {SUBDIVISION, 0, INPUT, 0, ANALOG},       // Subdivision in
    {PHASE, 0, INPUT, 0, ANALOG},             // Phase in
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

ModuleInterface<double> *module = buildModule();

void setUp(void) {
  // set stuff up here
  module->init();
}

/* void test_pt(void) { */
/*   uint16_t in_data[4][2] = {{0, 1}, {0, 2}, {0, 3}, {0, 4}}; */
/*   uint16_t out_data[4][2]; */

/*   for (uint8_t s = 0; s < 4; s++) { */
/*     PT_process(&pt, in_data[s], out_data[s], 2); */
/*   } */

/*   for (uint8_t c = 0; c < 2; c++) { */
/*     for (uint8_t s = 0; s < 4; s++) { */
/*       TEST_ASSERT_EQUAL_UINT16(in_data[s][c], out_data[s][c]); */
/*     } */
/*   } */
/* } */

int main(int argc, char **argv) {
  UNITY_BEGIN();
  /* RUN_TEST(test_pt); */
  UNITY_END();

  return 0;
}
